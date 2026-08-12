"""GUI-independent order, scheduling and execution closed loop."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

from interfaces.robot_interface import IRobotExecutor
from interfaces.scheduler_interface import IScheduler
from interfaces.types import Order, Task, TaskResult, TaskStatus


@dataclass(frozen=True)
class OrchestratorEvent:
    kind: str
    message: str = ""
    task_id: str = ""
    result: Optional[TaskResult] = None


EventCallback = Callable[[OrchestratorEvent], None]


class CellOrchestrator:
    """Own one scheduler/executor pair and dispatch every task exactly once."""

    TERMINAL_STATES = {
        TaskStatus.FINISHED.value,
        TaskStatus.FAILED.value,
    }

    def __init__(
        self,
        scheduler: IScheduler,
        executor: IRobotExecutor,
        poll_interval: float = 0.05,
    ):
        self.scheduler = scheduler
        self.executor = executor
        self.poll_interval = max(float(poll_interval), 0.005)
        self.orders: list[Order] = []
        self.tasks: list[Task] = []
        self._results_by_task: dict[str, TaskResult] = {}
        self._callbacks: list[EventCallback] = []
        self._dispatched_task_ids: set[str] = set()
        self._active_task_ids: set[str] = set()
        self._lock = threading.RLock()
        self._wake_event = threading.Event()
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._done_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._status = "idle"

    @property
    def status(self) -> str:
        with self._lock:
            return self._status

    @property
    def dispatched_task_ids(self) -> set[str]:
        with self._lock:
            return set(self._dispatched_task_ids)

    @property
    def results_by_task(self) -> dict[str, TaskResult]:
        """Return a stable copy of all executor results in this cycle."""
        with self._lock:
            return dict(self._results_by_task)

    def add_event_callback(self, callback: EventCallback) -> None:
        self._callbacks.append(callback)

    def start(self, orders: Iterable[Order]) -> list[Task]:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("orchestrator is already running")
            self.orders = list(orders)
            self.tasks = self.scheduler.generate_tasks(self.orders)
            self._dispatched_task_ids.clear()
            self._active_task_ids.clear()
            self._results_by_task.clear()
            self._stop_event.clear()
            self._pause_event.clear()
            self._done_event.clear()
            self._wake_event.clear()
            self._status = "running"
            self._thread = threading.Thread(
                target=self._run_loop,
                name="cell-orchestrator",
                daemon=True,
            )
            self._thread.start()
            tasks = list(self.tasks)
        self._emit(
            OrchestratorEvent(
                "started",
                f"generated {len(tasks)} initial tasks",
            )
        )
        return tasks

    def pause(self) -> None:
        with self._lock:
            if self._status != "running":
                return
            self._status = "paused"
            self._pause_event.set()
        self._emit(OrchestratorEvent("paused"))

    def resume(self) -> None:
        with self._lock:
            if self._status != "paused":
                return
            self._status = "running"
            self._pause_event.clear()
            self._wake_event.set()
        self._emit(OrchestratorEvent("resumed"))

    def stop(self) -> None:
        self._stop_event.set()
        self._pause_event.clear()
        self._wake_event.set()

    def wait(self, timeout: Optional[float] = None) -> str:
        self._done_event.wait(timeout)
        return self.status

    def add_order(self, order: Order, urgent: bool = False) -> list[Task]:
        """Add an order to a running batch without restarting dispatched work."""
        with self._lock:
            if self._status not in {"running", "paused"}:
                raise RuntimeError("orchestrator is not accepting live orders")
            new_tasks = (
                self.scheduler.insert_urgent_order(order)
                if urgent
                else self.scheduler.generate_tasks([order])
            )
            self.orders.append(order)
            self.tasks.extend(new_tasks)
        self._emit(
            OrchestratorEvent(
                "order_added",
                f"generated {len(new_tasks)} tasks for {order.order_id}",
            )
        )
        self._wake_event.set()
        return list(new_tasks)

    def _run_loop(self) -> None:
        try:
            while True:
                if self._stop_event.is_set():
                    self._finish("stopped", "execution stopped")
                    return
                if self._pause_event.is_set():
                    self._wake_event.wait(self.poll_interval)
                    self._wake_event.clear()
                    continue

                with self._lock:
                    failed_tasks = [
                        task
                        for task in self.tasks
                        if task.status == TaskStatus.FAILED.value
                    ]
                    if failed_tasks:
                        should_fail = True
                        failure_details = []
                        for task in failed_tasks:
                            result = self._results_by_task.get(task.task_id)
                            detail = (
                                result.message.strip()
                                if result is not None and result.message.strip()
                                else "executor reported a failed task"
                            )
                            robot_id = (
                                result.robot_id
                                if result is not None and result.robot_id
                                else (
                                    task.available_robots[0]
                                    if task.available_robots
                                    else "unassigned"
                                )
                            )
                            failure_details.append(
                                f"{task.task_id} order={task.order_id} "
                                f"process={task.process} robot={robot_id}: {detail}"
                            )
                        failure_message = " | ".join(failure_details)
                        to_dispatch: list[Task] = []
                    else:
                        should_fail = False
                        failure_message = ""
                        robots = self.executor.get_robot_states()
                        self.tasks = self.scheduler.schedule(
                            self.tasks, robots
                        )
                        to_dispatch = [
                            task
                            for task in self.tasks
                            if task.status == TaskStatus.RUNNING.value
                            and task.task_id
                            not in self._dispatched_task_ids
                        ]
                        for task in to_dispatch:
                            self._dispatched_task_ids.add(task.task_id)
                            self._active_task_ids.add(task.task_id)
                        complete = (
                            not self._active_task_ids
                            and all(
                                task.status == TaskStatus.FINISHED.value
                                for task in self.tasks
                            )
                        )

                if should_fail:
                    self._finish("failed", failure_message)
                    return
                if complete:
                    self._finish("finished", "all tasks finished")
                    return

                for task in to_dispatch:
                    self._emit(
                        OrchestratorEvent(
                            "task_dispatched",
                            task_id=task.task_id,
                        )
                    )
                    try:
                        self.executor.execute_task_async(
                            task,
                            lambda result, current=task: self._on_task_done(
                                current, result
                            ),
                        )
                    except Exception as exc:
                        now = time.time()
                        robot_id = (
                            task.available_robots[0]
                            if task.available_robots
                            else ""
                        )
                        self._on_task_done(
                            task,
                            TaskResult(
                                task_id=task.task_id,
                                robot_id=robot_id,
                                status=TaskStatus.FAILED.value,
                                start_time=now,
                                end_time=now,
                                message=str(exc),
                            ),
                        )

                self._wake_event.wait(self.poll_interval)
                self._wake_event.clear()
        except Exception as exc:
            self._finish("failed", str(exc))

    def _on_task_done(self, task: Task, result: TaskResult) -> None:
        with self._lock:
            if result.task_id != task.task_id:
                result = TaskResult(
                    task_id=task.task_id,
                    robot_id=result.robot_id,
                    status=TaskStatus.FAILED.value,
                    start_time=result.start_time,
                    end_time=result.end_time,
                    message=(
                        "executor returned mismatched task id "
                        f"{result.task_id}"
                    ),
                )
            self.tasks = self.scheduler.on_task_complete(
                result,
                self.tasks,
                self.executor.get_robot_states(),
            )
            self._results_by_task[task.task_id] = result
            self._active_task_ids.discard(task.task_id)
        self._emit(
            OrchestratorEvent(
                "task_completed",
                task_id=task.task_id,
                result=result,
            )
        )
        self._wake_event.set()

    def _finish(self, status: str, message: str) -> None:
        with self._lock:
            self._status = status
            self._done_event.set()
        self._emit(OrchestratorEvent(status, message))

    def _emit(self, event: OrchestratorEvent) -> None:
        for callback in list(self._callbacks):
            try:
                callback(event)
            except Exception:
                pass
