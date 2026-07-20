"""Current-scene scheduler adapter for the validated five-arm process.

This adapter deliberately schedules one A-type unit in the exact calibrated
seven-task order.  It does not claim multi-order replenishment, cross-robot
fault reassignment, or a dynamic optimization algorithm that the current scene
cannot execute safely yet.
"""

from __future__ import annotations

from typing import Callable, List

from interfaces.scheduler_interface import IScheduler
from interfaces.types import Order, RobotState, Task, TaskResult, TaskStatus
from robot_control.r1_motion import R1_BOX_PLACED, R1_TERMINAL_PLACED
from robot_control.r2_motion import R2_PCB_PLACED
from robot_control.r3_motion import R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION
from robot_control.r4_motion import R4_SCREW_DONE
from robot_control.r5_motion import R5_SORT_DEFECT_DONE, R5_SORT_GOOD_DONE


PROCESS_SEQUENCE = (
    (R1_BOX_PLACED, "R1", "assembly_area", "assemble"),
    (R2_PCB_PLACED, "R2", "assembly_area", "assemble"),
    (R3_MODULE_PLACED, "R3", "assembly_area", "assemble"),
    (R1_TERMINAL_PLACED, "R1", "assembly_area", "assemble"),
    (R3_PRODUCT_TO_INSPECTION, "R3", "inspection_screw_area", "transfer"),
    (R4_SCREW_DONE, "R4", "inspection_screw_area", "screw"),
)


class Scheduler(IScheduler):
    """Expose the validated fixed process through the team scheduler contract."""

    def __init__(self):
        self._callbacks: List[Callable] = []
        self._quality_by_order: dict[str, str] = {}

    def set_state_change_callback(self, callback: Callable) -> None:
        self._callbacks.append(callback)

    def _notify(self) -> None:
        for callback in self._callbacks:
            try:
                callback()
            except Exception:
                pass

    @staticmethod
    def _validate_order(order: Order) -> str:
        if order.product_type.upper() != "A":
            raise ValueError(
                "the current CoppeliaSim cell is calibrated only for product type A"
            )
        if order.quantity != 1:
            raise ValueError(
                "real mode supports one unit per clean scene; quantity must be 1"
            )
        quality = order.expected_quality.upper()
        if quality not in {"OK", "NG"}:
            raise ValueError("expected_quality must be OK or NG")
        return quality

    def quality_for_order(self, order_id: str) -> str:
        try:
            return self._quality_by_order[order_id]
        except KeyError as exc:
            raise ValueError(f"quality is unknown for order {order_id}") from exc

    def generate_tasks(self, orders: List[Order]) -> List[Task]:
        if len(orders) != 1:
            raise ValueError(
                "real mode requires exactly one order in a clean scene"
            )
        order = orders[0]
        quality = self._validate_order(order)
        self._quality_by_order[order.order_id] = quality
        entries = list(PROCESS_SEQUENCE)
        entries.append(
            (
                R5_SORT_GOOD_DONE if quality == "OK" else R5_SORT_DEFECT_DONE,
                "R5",
                "sort_area",
                "sort_good" if quality == "OK" else "sort_defect",
            )
        )

        tasks: List[Task] = []
        previous = None
        for index, (action, robot_id, area, process) in enumerate(entries, start=1):
            task = Task(
                task_id=f"{order.order_id}-{index:02d}-{action}",
                order_id=order.order_id,
                product_type=order.product_type,
                process=process,
                target_area=area,
                target_point=action,
                available_robots=[robot_id],
                predecessors=[previous] if previous else [],
                priority=order.priority,
                status=TaskStatus.PENDING.value,
            )
            tasks.append(task)
            previous = task.task_id
        self._notify()
        return tasks

    @staticmethod
    def _predecessors_finished(task: Task, tasks: List[Task]) -> bool:
        by_id = {candidate.task_id: candidate for candidate in tasks}
        return all(
            predecessor in by_id
            and by_id[predecessor].status == TaskStatus.FINISHED.value
            for predecessor in task.predecessors
        )

    @staticmethod
    def _propagate_failed_predecessors(tasks: List[Task]) -> None:
        """Fail every downstream task, even if the list is not topological."""
        by_id = {task.task_id: task for task in tasks}
        changed = True
        while changed:
            changed = False
            for task in tasks:
                if task.status in {
                    TaskStatus.FINISHED.value,
                    TaskStatus.FAILED.value,
                }:
                    continue
                if any(
                    predecessor not in by_id
                    or by_id[predecessor].status == TaskStatus.FAILED.value
                    for predecessor in task.predecessors
                ):
                    task.status = TaskStatus.FAILED.value
                    changed = True

    def schedule(
        self, tasks: List[Task], robots: List[RobotState]
    ) -> List[Task]:
        self._propagate_failed_predecessors(tasks)
        if any(task.status == TaskStatus.RUNNING.value for task in tasks):
            return tasks
        idle = {robot.robot_id for robot in robots if robot.status == "idle"}
        for task in tasks:
            if task.status not in {
                TaskStatus.PENDING.value,
                TaskStatus.WAITING.value,
            }:
                continue
            if not self._predecessors_finished(task, tasks):
                task.status = TaskStatus.WAITING.value
                continue
            if task.available_robots[0] in idle:
                task.status = TaskStatus.RUNNING.value
                break
        self._notify()
        return tasks

    def insert_urgent_order(self, order: Order) -> List[Task]:
        del order
        raise ValueError(
            "real mode cannot insert another order into the single-product scene"
        )

    def handle_robot_fault(self, robot_id: str, tasks: List[Task]) -> List[Task]:
        for task in tasks:
            if (
                robot_id in task.available_robots
                and task.status
                not in {TaskStatus.FINISHED.value, TaskStatus.FAILED.value}
            ):
                task.status = TaskStatus.FAILED.value
        self._propagate_failed_predecessors(tasks)
        self._notify()
        return tasks

    def on_task_complete(
        self,
        result: TaskResult,
        tasks: List[Task],
        robots: List[RobotState],
    ) -> List[Task]:
        del robots
        for task in tasks:
            if task.task_id == result.task_id:
                task.status = result.status
                break
        if result.status == TaskStatus.FAILED.value:
            self._propagate_failed_predecessors(tasks)
        else:
            for task in tasks:
                if (
                    task.status == TaskStatus.WAITING.value
                    and self._predecessors_finished(task, tasks)
                ):
                    task.status = TaskStatus.PENDING.value
        self._notify()
        return tasks


__all__ = ["PROCESS_SEQUENCE", "Scheduler"]
