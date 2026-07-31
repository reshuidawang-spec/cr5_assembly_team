"""动态任务调度器。

采用综合评分驱动的动态列表调度：
1. 前置任务完成后才可执行；
2. 机械臂空闲且区域锁可用时派发；
3. 多个候选任务按优先级、交期松弛、等待老化和剩余关键路径综合排序。
"""

from pathlib import Path
from typing import Callable, Dict, List, Optional, Set

from interfaces.scheduler_interface import IScheduler
from interfaces.types import Order, RobotState, Task, TaskResult, TaskStatus
from scheduler.config_loader import load_yaml
from scheduler.task_generator import TaskGenerator


class Scheduler(IScheduler):
    """真实调度器。"""

    def __init__(
        self,
        product_config_path: Optional[str] = None,
        scheduler_config_path: Optional[str] = None,
    ):
        root = Path(__file__).resolve().parents[1]
        self.task_generator = TaskGenerator(product_config_path)
        self.scheduler_config_path = (
            Path(scheduler_config_path)
            if scheduler_config_path
            else root / "configs" / "scheduler.yaml"
        )
        self.config = self._load_scheduler_config(self.scheduler_config_path)
        self._callbacks: List[Callable] = []
        self._area_locks: Dict[str, Set[str]] = {}
        self._station_owners: Dict[str, str] = {}
        self._dispatched_tasks: Set[str] = set()
        self._conflict_count = 0
        self._logical_time = 0.0
        self._ready_since: Dict[str, float] = {}
        self._robot_reservations: Dict[str, float] = {}
        self._quality_by_order: Dict[str, str] = {}

    @property
    def conflict_count(self) -> int:
        return self._conflict_count

    def set_state_change_callback(self, callback: Callable) -> None:
        self._callbacks.append(callback)

    def generate_tasks(self, orders: List[Order]) -> List[Task]:
        tasks = self.task_generator.generate(orders)
        for task in tasks:
            if not task.predecessors:
                self._ready_since[task.task_id] = self._logical_time
        return tasks

    def schedule(self, tasks: List[Task], robots: List[RobotState]) -> List[Task]:
        self._unlock_ready_waiting_tasks(tasks)

        idle_robots = {
            r.robot_id
            for r in robots
            if r.status == "idle" and r.current_task is None
        }
        candidates = [
            task
            for task in tasks
            if task.status == TaskStatus.PENDING.value
            and task.task_id not in self._dispatched_tasks
            and self._predecessors_finished(task, tasks)
            and self._station_flow_allows(task)
        ]
        scoring = self.config.get("scoring", {})
        candidates.sort(
            key=lambda task: self.task_generator.task_sort_key(
                task,
                current_time=self._logical_time,
                ready_time=self._ready_since.get(task.task_id, self._logical_time),
                remaining_work=self._remaining_order_work(task, tasks),
                weights=scoring,
            )
        )

        for task in candidates:
            robot_id = self._select_idle_robot(task, idle_robots)
            if not robot_id:
                continue
            if not self._try_lock_area(task):
                self._conflict_count += 1
                continue

            task.status = TaskStatus.RUNNING.value
            task.available_robots = [robot_id] + [
                candidate for candidate in task.available_robots if candidate != robot_id
            ]
            self._dispatched_tasks.add(task.task_id)
            self._claim_station_for_task(task)
            idle_robots.remove(robot_id)
            self._reserve_successor_robots(
                task, tasks, self._logical_time + task.duration
            )

        self._notify()
        return tasks

    def insert_urgent_order(self, order: Order) -> List[Task]:
        urgent_threshold = int(self.config.get("urgent_threshold", 5))
        order.priority = max(order.priority, urgent_threshold)
        order.arrival_time = max(order.arrival_time, self._logical_time)
        return self.generate_tasks([order])

    def handle_robot_fault(self, robot_id: str, tasks: List[Task]) -> List[Task]:
        for task in tasks:
            assigned_robot = task.available_robots[0] if task.available_robots else None
            if task.status == TaskStatus.RUNNING.value and assigned_robot == robot_id:
                task.status = TaskStatus.PENDING.value
                self._dispatched_tasks.discard(task.task_id)
                self._release_task_locks(task.task_id)
                self._release_station_on_interruption(task)
                self._ready_since[task.task_id] = self._logical_time

        self._notify()
        return tasks

    def on_task_complete(
        self,
        result: TaskResult,
        tasks: List[Task],
        robots: List[RobotState],
    ) -> List[Task]:
        self._logical_time = max(self._logical_time, result.end_time)
        completed_task: Optional[Task] = None
        for task in tasks:
            if task.task_id == result.task_id:
                task.status = result.status
                completed_task = task
                break

        self._dispatched_tasks.discard(result.task_id)
        self._release_task_locks(result.task_id)
        if completed_task:
            self._release_station_after_task(completed_task)

        if (
            completed_task
            and completed_task.process == "inspect"
            and result.status == TaskStatus.FINISHED.value
            and result.quality_result in ("OK", "NG")
        ):
            self._quality_by_order[completed_task.order_id] = result.quality_result

        if (
            completed_task
            and completed_task.process in ("inspect", "screw")
            and result.status == TaskStatus.FINISHED.value
        ):
            quality_result = (
                result.quality_result
                if result.quality_result in ("OK", "NG")
                else self._quality_by_order.get(completed_task.order_id, "")
            )
            post_task = self.task_generator.build_post_inspection_task(
                completed_task,
                quality_result,
            )
            if post_task:
                tasks.append(post_task)
                self._ready_since[post_task.task_id] = self._logical_time

        self._unlock_ready_waiting_tasks(tasks)
        self._notify()
        return tasks

    def _load_scheduler_config(self, path: Path) -> dict:
        raw = load_yaml(path)
        return raw.get("scheduler", raw)

    def _notify(self) -> None:
        for callback in self._callbacks:
            try:
                callback()
            except Exception:
                pass

    def _unlock_ready_waiting_tasks(self, tasks: List[Task]) -> None:
        for task in tasks:
            if task.status == TaskStatus.WAITING.value and self._predecessors_finished(task, tasks):
                task.status = TaskStatus.PENDING.value
                self._ready_since.setdefault(task.task_id, self._logical_time)

    def _predecessors_finished(self, task: Task, tasks: List[Task]) -> bool:
        if not task.predecessors:
            return True
        finished_ids = {
            item.task_id
            for item in tasks
            if item.status == TaskStatus.FINISHED.value
        }
        return all(pred in finished_ids for pred in task.predecessors)

    def _select_idle_robot(self, task: Task, idle_robots: Set[str]) -> Optional[str]:
        urgent_threshold = int(self.config.get("urgent_threshold", 5))
        for robot_id in task.available_robots:
            if robot_id not in idle_robots:
                continue
            reserved_until = self._robot_reservations.get(robot_id, 0.0)
            if (
                task.priority < urgent_threshold
                and reserved_until > self._logical_time
                and self._logical_time + task.duration > reserved_until
            ):
                continue
            return robot_id
        return None

    def _remaining_order_work(self, task: Task, tasks: List[Task]) -> float:
        return sum(
            item.duration
            for item in tasks
            if item.order_id == task.order_id
            and item.status not in (TaskStatus.FINISHED.value, TaskStatus.FAILED.value)
        )

    def _reserve_successor_robots(
        self,
        task: Task,
        tasks: List[Task],
        ready_at: float,
    ) -> Set[str]:
        reserved: Set[str] = set()
        urgent_threshold = int(self.config.get("urgent_threshold", 5))
        if task.priority < urgent_threshold:
            return reserved
        for successor in tasks:
            if task.task_id not in successor.predecessors:
                continue
            for robot_id in successor.available_robots:
                reserved.add(robot_id)
                self._robot_reservations[robot_id] = max(
                    self._robot_reservations.get(robot_id, 0.0),
                    ready_at,
                )
        return reserved

    def _try_lock_area(self, task: Task) -> bool:
        areas = [area for area in self._task_lock_areas(task) if self._area_needs_lock(area)]
        for area in areas:
            owners = self._area_locks.get(area, set())
            if task.task_id not in owners and len(owners) >= self._area_capacity(area):
                return False
        for area in areas:
            self._area_locks.setdefault(area, set()).add(task.task_id)
        return True

    def _task_lock_areas(self, task: Task) -> List[str]:
        return task.required_areas or [task.target_area]

    def _area_needs_lock(self, area: str) -> bool:
        areas = self.config.get("areas", {})
        return bool(areas.get(area, {}).get("lock_required", False))

    def _area_capacity(self, area: str) -> int:
        areas = self.config.get("areas", {})
        return max(int(areas.get(area, {}).get("max_robots", 1)), 1)

    def _release_task_locks(self, task_id: str) -> None:
        for area, owners in list(self._area_locks.items()):
            owners.discard(task_id)
            if not owners:
                del self._area_locks[area]

    def _station_flow_allows(self, task: Task) -> bool:
        areas = self._task_lock_areas(task)
        assembly_owner = self._station_owners.get("assembly_fixture")
        if "assembly_fixture" in areas:
            if assembly_owner and assembly_owner != task.order_id:
                return False
            if not assembly_owner and task.process != "box_feed" and task.predecessors:
                return False

        inspection_owner = self._station_owners.get("inspection_platform_area")
        if "inspection_platform_area" in areas:
            if inspection_owner and inspection_owner != task.order_id:
                return False
            if not inspection_owner and task.process != "transfer_to_inspection" and task.predecessors:
                return False
        return True

    def _claim_station_for_task(self, task: Task) -> None:
        if task.process == "box_feed":
            self._station_owners.setdefault("assembly_fixture", task.order_id)
        elif task.process == "transfer_to_inspection":
            self._station_owners.setdefault("inspection_platform_area", task.order_id)

    def _release_station_after_task(self, task: Task) -> None:
        if task.process == "transfer_to_inspection":
            if self._station_owners.get("assembly_fixture") == task.order_id:
                del self._station_owners["assembly_fixture"]
        elif task.process in ("sort_good", "sort_defect"):
            if self._station_owners.get("inspection_platform_area") == task.order_id:
                del self._station_owners["inspection_platform_area"]

    def _release_station_on_interruption(self, task: Task) -> None:
        if task.process == "box_feed" and self._station_owners.get("assembly_fixture") == task.order_id:
            del self._station_owners["assembly_fixture"]
        elif task.process == "transfer_to_inspection":
            if self._station_owners.get("inspection_platform_area") == task.order_id:
                del self._station_owners["inspection_platform_area"]
