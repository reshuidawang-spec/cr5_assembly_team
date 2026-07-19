"""Mock 调度器 —— 4号同学替换为真实实现"""

import time
from typing import List, Callable, Optional
from interfaces.scheduler_interface import IScheduler
from interfaces.types import Order, Task, TaskResult, TaskStatus, RobotState, ProcessType


# 产品 → 工艺序列 + 参数
_PRODUCT_PROCESSES = {
    "A": {
        "processes": [
            (ProcessType.BOX_FEED, "box_supply_area", "R1_BOX_PLACE_TCP", ["R1"], 6, ["box_supply_area", "assembly_fixture"]),
            (ProcessType.PCB_INSTALL, "pcb_supply_area", "R2_PCB_PLACE_TCP", ["R2"], 8, ["pcb_supply_area", "assembly_fixture"]),
            (ProcessType.MODULE_INSTALL, "module_supply_area", "R3_MODULE_PLACE_TCP", ["R3"], 7, ["module_supply_area", "assembly_fixture"]),
            (ProcessType.TERMINAL_INSTALL, "terminal_supply_area", "R1_TERMINAL_PLACE_TCP", ["R1"], 7, ["terminal_supply_area", "assembly_fixture"]),
            (ProcessType.TRANSFER_TO_INSPECTION, "transfer_area", "R3_PRODUCT_PLACE_INSPECTION_TCP", ["R3"], 8, ["assembly_fixture", "inspection_platform_area"]),
            (ProcessType.INSPECT, "camera_area", "CAMERA_INSPECTION_CENTER", ["CAMERA"], 5, ["inspection_platform_area", "camera_area"]),
            (ProcessType.SCREW, "inspection_screw_area", "R4_SCREW_PRESS", ["R4"], 9, ["inspection_screw_area", "inspection_platform_area"]),
        ],
    },
    "B": {
        "processes": [
            (ProcessType.BOX_FEED, "box_supply_area", "R1_BOX_PLACE_TCP", ["R1"], 7, ["box_supply_area", "assembly_fixture"]),
            (ProcessType.PCB_INSTALL, "pcb_supply_area", "R2_PCB_PLACE_TCP", ["R2"], 10, ["pcb_supply_area", "assembly_fixture"]),
            (ProcessType.MODULE_INSTALL, "module_supply_area", "R3_MODULE_PLACE_TCP", ["R3"], 9, ["module_supply_area", "assembly_fixture"]),
            (ProcessType.TERMINAL_INSTALL, "terminal_supply_area", "R1_TERMINAL_PLACE_TCP", ["R1"], 8, ["terminal_supply_area", "assembly_fixture"]),
            (ProcessType.TRANSFER_TO_INSPECTION, "transfer_area", "R3_PRODUCT_PLACE_INSPECTION_TCP", ["R3"], 9, ["assembly_fixture", "inspection_platform_area"]),
            (ProcessType.INSPECT, "camera_area", "CAMERA_INSPECTION_CENTER", ["CAMERA"], 6, ["inspection_platform_area", "camera_area"]),
            (ProcessType.SCREW, "inspection_screw_area", "R4_SCREW_PRESS", ["R4"], 11, ["inspection_screw_area", "inspection_platform_area"]),
        ],
    },
    "C": {
        "processes": [
            (ProcessType.BOX_FEED, "box_supply_area", "R1_BOX_PLACE_TCP", ["R1"], 8, ["box_supply_area", "assembly_fixture"]),
            (ProcessType.PCB_INSTALL, "pcb_supply_area", "R2_PCB_PLACE_TCP", ["R2"], 12, ["pcb_supply_area", "assembly_fixture"]),
            (ProcessType.MODULE_INSTALL, "module_supply_area", "R3_MODULE_PLACE_TCP", ["R3"], 10, ["module_supply_area", "assembly_fixture"]),
            (ProcessType.TERMINAL_INSTALL, "terminal_supply_area", "R1_TERMINAL_PLACE_TCP", ["R1"], 10, ["terminal_supply_area", "assembly_fixture"]),
            (ProcessType.TRANSFER_TO_INSPECTION, "transfer_area", "R3_PRODUCT_PLACE_INSPECTION_TCP", ["R3"], 10, ["assembly_fixture", "inspection_platform_area"]),
            (ProcessType.INSPECT, "camera_area", "CAMERA_INSPECTION_CENTER", ["CAMERA"], 7, ["inspection_platform_area", "camera_area"]),
            (ProcessType.SCREW, "inspection_screw_area", "R4_SCREW_PRESS", ["R4"], 13, ["inspection_screw_area", "inspection_platform_area"]),
        ],
    },
}

# 检测结果先记录，R4 锁付完成后由 R5 分拣（根据检测结果）
_SORT_PROCESSES = {
    ProcessType.SORT_GOOD: ("good_conveyor_area", "R5_GOOD_PLACE_TCP", ["R5"], 5, ["inspection_platform_area", "good_conveyor_area"]),
    ProcessType.SORT_DEFECT: ("defect_conveyor_area", "R5_DEFECT_PLACE_TCP", ["R5"], 5, ["inspection_platform_area", "defect_conveyor_area"]),
}


class MockScheduler(IScheduler):
    """模拟调度器：按固定工艺模板生成任务，FIFO + 优先级分配"""

    def __init__(self):
        self._task_counter = 0
        self._callbacks: List[Callable] = []
        self._quality_by_order = {}

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
            for unit_index in range(order.quantity):
                suffix = "" if order.quantity == 1 else f"-{unit_index + 1:02d}"
                unit_order_id = f"{order.order_id}{suffix}"
                prev_id: Optional[str] = None
                for process, area, point, robots, duration, required_areas in product_cfg["processes"]:
                    task = Task(
                        task_id=self._next_task_id(),
                        order_id=unit_order_id,
                        product_type=order.product_type,
                        process=process.value,
                        target_area=area,
                        target_point=point,
                        available_robots=list(robots),
                        duration=duration,
                        predecessors=[prev_id] if prev_id else [],
                        priority=order.priority,
                        status=(
                            TaskStatus.PENDING.value
                            if not prev_id
                            else TaskStatus.WAITING.value
                        ),
                        required_areas=list(required_areas),
                    )
                    tasks.append(task)
                    prev_id = task.task_id

        return tasks

    def schedule(
        self, tasks: List[Task], robots: List[RobotState],
    ) -> List[Task]:
        idle_robots = {r.robot_id for r in robots if r.status == "idle"}
        for task in sorted(tasks, key=lambda item: (-item.priority, item.task_id)):
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
                robot_id = available[0]
                task.status = TaskStatus.RUNNING.value
                task.available_robots = [robot_id] + [
                    candidate for candidate in task.available_robots if candidate != robot_id
                ]
                idle_robots.remove(robot_id)
        return tasks

    def insert_urgent_order(self, order: Order) -> List[Task]:
        order.priority = 10
        return self.generate_tasks([order])

    def handle_robot_fault(self, robot_id: str, tasks: List[Task]) -> List[Task]:
        for task in tasks:
            assigned_robot = task.available_robots[0] if task.available_robots else None
            if task.status == TaskStatus.RUNNING.value and assigned_robot == robot_id:
                task.status = TaskStatus.PENDING.value
        self._notify()
        return tasks

    def on_task_complete(
        self, result: TaskResult, tasks: List[Task], robots: List[RobotState],
    ) -> List[Task]:
        completed_task: Optional[Task] = None
        for task in tasks:
            if task.task_id == result.task_id:
                task.status = result.status
                completed_task = task
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
        if (
            completed_task
            and completed_task.process == ProcessType.INSPECT.value
            and result.status == TaskStatus.FINISHED.value
            and result.quality_result in ("OK", "NG")
        ):
            self._quality_by_order[completed_task.order_id] = result.quality_result

        if (
            completed_task
            and completed_task.process == ProcessType.SCREW.value
            and result.status == TaskStatus.FINISHED.value
        ):
            quality_result = self._quality_by_order.get(completed_task.order_id, "")
            if quality_result not in ("OK", "NG"):
                self._notify()
                return tasks
            process = (
                ProcessType.SORT_GOOD
                if quality_result == "OK"
                else ProcessType.SORT_DEFECT
            )
            area, point, available_robots, duration, required_areas = _SORT_PROCESSES[process]
            tasks.append(Task(
                task_id=self._next_task_id(),
                order_id=completed_task.order_id,
                product_type=completed_task.product_type,
                process=process.value,
                target_area=area,
                target_point=point,
                available_robots=list(available_robots),
                duration=duration,
                predecessors=[completed_task.task_id],
                priority=completed_task.priority,
                status=TaskStatus.PENDING.value,
                required_areas=list(required_areas),
            ))
        self._notify()
        return tasks
