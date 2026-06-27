"""Mock 调度器 —— 4号同学替换为真实实现"""

import time
from typing import List, Callable, Optional
from interfaces.scheduler_interface import IScheduler
from interfaces.types import Order, Task, TaskResult, TaskStatus, RobotState, ProcessType


# 产品 → 工艺序列 + 参数
_PRODUCT_PROCESSES = {
    "A": {
        "processes": [
            (ProcessType.FEED, "feed_area", "P_FEED_01", ["R1"], 6),
            (ProcessType.ASSEMBLE, "assembly_area", "P_ASSEMBLY_01", ["R2"], 12),
            (ProcessType.SCREW, "screw_area", "P_SCREW_01", ["R3"], 10),
            (ProcessType.INSPECT, "inspect_area", "P_INSPECT_01", ["R3"], 5),
        ],
    },
    "B": {
        "processes": [
            (ProcessType.FEED, "feed_area", "P_FEED_01", ["R1"], 7),
            (ProcessType.ASSEMBLE, "assembly_area", "P_ASSEMBLY_01", ["R2"], 16),
            (ProcessType.SCREW, "screw_area", "P_SCREW_01", ["R3"], 14),
            (ProcessType.INSPECT, "inspect_area", "P_INSPECT_01", ["R3"], 6),
        ],
    },
    "C": {
        "processes": [
            (ProcessType.FEED, "feed_area", "P_FEED_01", ["R1"], 8),
            (ProcessType.ASSEMBLE, "assembly_area", "P_ASSEMBLY_01", ["R2"], 20),
            (ProcessType.SCREW, "screw_area", "P_SCREW_01", ["R3"], 16),
            (ProcessType.INSPECT, "inspect_area", "P_INSPECT_01", ["R3"], 7),
        ],
    },
}

# 检测后 R4 分拣（根据检测结果）
_SORT_PROCESSES = {
    ProcessType.SORT_GOOD: ("sort_area", "P_GOOD_01", ["R4"], 4),
    ProcessType.SORT_DEFECT: ("sort_area", "P_DEFECT_01", ["R4"], 4),
}


class MockScheduler(IScheduler):
    """模拟调度器：按固定工艺模板生成任务，FIFO + 优先级分配"""

    def __init__(self):
        self._task_counter = 0
        self._callbacks: List[Callable] = []

    def set_state_change_callback(self, callback: Callable) -> None:
        self._callbacks.append(callback)

    def _notify(self) -> None:
        for cb in self._callbacks:
            try:
                cb()
            except Exception:
                pass

    def _next_task_id(self) -> str:
        self._task_counter += 1
        return f"T{self._task_counter:04d}"

    def generate_tasks(self, orders: List[Order]) -> List[Task]:
        tasks: List[Task] = []
        for order in orders:
            product_cfg = _PRODUCT_PROCESSES.get(
                order.product_type,
                _PRODUCT_PROCESSES["A"],
            )
            prev_id: Optional[str] = None
            for process, area, point, robots, duration in product_cfg["processes"]:
                task = Task(
                    task_id=self._next_task_id(),
                    order_id=order.order_id,
                    product_type=order.product_type,
                    process=process.value,
                    target_area=area,
                    target_point=point,
                    available_robots=list(robots),
                    duration=duration,
                    predecessors=[prev_id] if prev_id else [],
                    priority=order.priority,
                    status=TaskStatus.PENDING.value,
                )
                tasks.append(task)
                prev_id = task.task_id

            # 为检测任务附加 R4 分拣任务
            inspect_task_id = prev_id
            for sort_process in [ProcessType.SORT_GOOD, ProcessType.SORT_DEFECT]:
                area, point, robots, duration = _SORT_PROCESSES[sort_process]
                sort_task = Task(
                    task_id=self._next_task_id(),
                    order_id=order.order_id,
                    product_type=order.product_type,
                    process=sort_process.value,
                    target_area=area,
                    target_point=point,
                    available_robots=list(robots),
                    duration=duration,
                    predecessors=[inspect_task_id],
                    priority=order.priority,
                    status=TaskStatus.PENDING.value,
                )
                tasks.append(sort_task)

        return tasks

    def schedule(
        self, tasks: List[Task], robots: List[RobotState],
    ) -> List[Task]:
        idle_robots = {r.robot_id for r in robots if r.status == "idle"}
        robot_task_map = {
            "R1": ProcessType.FEED.value,
            "R1": ProcessType.UNLOAD.value,
            "R2": ProcessType.ASSEMBLE.value,
            "R3": ProcessType.SCREW.value,
            "R3": ProcessType.INSPECT.value,
            "R4": ProcessType.SORT_GOOD.value,
            "R4": ProcessType.SORT_DEFECT.value,
        }

        for task in tasks:
            if task.status != TaskStatus.PENDING.value:
                continue
            # 检查前置任务
            if task.predecessors:
                pred_done = all(
                    any(
                        t.task_id == pid and t.status == TaskStatus.FINISHED.value
                        for t in tasks
                    )
                    for pid in task.predecessors
                )
                if not pred_done:
                    task.status = TaskStatus.WAITING.value
                    continue
            # 分配机械臂
            available = [r for r in task.available_robots if r in idle_robots]
            if available:
                task.status = TaskStatus.RUNNING.value
                # 标记对应机械臂为忙碌（由上层处理）
        return tasks

    def insert_urgent_order(self, order: Order) -> List[Task]:
        order.priority = 10
        return self.generate_tasks([order])

    def handle_robot_fault(self, robot_id: str, tasks: List[Task]) -> List[Task]:
        for task in tasks:
            if (
                task.status in (TaskStatus.RUNNING.value, TaskStatus.PENDING.value)
                and robot_id in task.available_robots
            ):
                task.status = TaskStatus.PENDING.value
                task.available_robots = [
                    r for r in task.available_robots if r != robot_id
                ]
        self._notify()
        return tasks

    def on_task_complete(
        self, result: TaskResult, tasks: List[Task], robots: List[RobotState],
    ) -> List[Task]:
        for task in tasks:
            if task.task_id == result.task_id:
                task.status = result.status
                break
        # 解锁后续任务
        for task in tasks:
            if task.status == TaskStatus.WAITING.value and task.predecessors:
                pred_done = all(
                    any(
                        t.task_id == pid and t.status == TaskStatus.FINISHED.value
                        for t in tasks
                    )
                    for pid in task.predecessors
                )
                if pred_done:
                    task.status = TaskStatus.PENDING.value
        # 如果检测结果为 NG，触发 sort_defect
        if result.quality_result == "NG":
            for task in tasks:
                if (
                    task.process == ProcessType.SORT_GOOD.value
                    and task.order_id == result.task_id
                ):
                    task.status = TaskStatus.PENDING.value  # 跳过良品分拣，执行不良品分拣
        self._notify()
        return tasks
