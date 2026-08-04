"""Scheduler adapter for the validated five-arm CoppeliaSim motion cycle."""

from __future__ import annotations

import threading
import time
from typing import Callable, Mapping, Optional

from interfaces.robot_interface import IRobotExecutor
from interfaces.types import RobotState, RobotStatus, Task, TaskResult, TaskStatus
from robot_control.robot_executor import RobotExecutor
from sim_bridge.coppelia_client import SimBridge


class SimulationCellExecutor(IRobotExecutor):
    """Expose R1-R5 motion plus the virtual camera through one interface.

    This executor controls only objects in the open CoppeliaSim scene.  It has
    no ROS driver, controller IP address, or physical-robot transport.
    """

    def __init__(
        self,
        bridge: SimBridge,
        quality_by_order: Optional[Mapping[str, str]] = None,
        default_quality: str = "OK",
        speed_deg_s: float = 50.0,
        hold_seconds: float = 0.25,
    ):
        self.bridge = bridge
        self.motion = RobotExecutor(
            sim_bridge=bridge,
            speed_deg_s=speed_deg_s,
            hold_seconds=hold_seconds,
        )
        self.quality_by_order = {
            str(key): str(value).strip().upper()
            for key, value in (quality_by_order or {}).items()
        }
        self.default_quality = str(default_quality).strip().upper()
        if self.default_quality not in {"OK", "NG"}:
            raise ValueError("default_quality must be OK or NG")
        self._camera = RobotState(robot_id="CAMERA")
        self._camera_lock = threading.RLock()
        self._prepared = False

    @property
    def prepared(self) -> bool:
        return self._prepared

    def prepare_cycle(self) -> dict:
        """Validate all plans, start deterministic stepping and enter READY."""
        self.bridge.set_visual_owner("executor")
        # The signal is consumed by the embedded product-stage script on the
        # first deterministic simulation step performed during READY.
        self.bridge.set_string_signal("cell_product_state", "reset")
        evidence = self.motion.prepare_cycle(
            quality="good",
            preload_both_r5=True,
            preposition_front_half=False,
        )
        # CoppeliaSim clears transient string signals when a simulation starts.
        # Re-assert ownership after READY so later scheduler completion signals
        # cannot create duplicate template products on top of the real parts.
        self.bridge.set_visual_owner("executor")
        self._prepared = True
        return evidence

    def _quality_for(self, order_id: str) -> str:
        quality = self.quality_by_order.get(order_id, self.default_quality)
        return quality if quality in {"OK", "NG"} else self.default_quality

    def execute_task(self, task: Task) -> TaskResult:
        if task.process != "inspect" and (
            not task.available_robots or task.available_robots[0] != "CAMERA"
        ):
            if not self._prepared:
                now = time.time()
                return TaskResult(
                    task_id=task.task_id,
                    robot_id=(task.available_robots[0] if task.available_robots else ""),
                    status=TaskStatus.FAILED.value,
                    start_time=now,
                    end_time=now,
                    message="CoppeliaSim motion cycle was not prepared",
                )
            return self.motion.execute_task(task)

        started = time.time()
        with self._camera_lock:
            if self._camera.status == RobotStatus.FAULT.value:
                return TaskResult(
                    task_id=task.task_id,
                    robot_id="CAMERA",
                    status=TaskStatus.FAILED.value,
                    start_time=started,
                    end_time=time.time(),
                    message="CAMERA is in fault state",
                )
            self._camera.status = RobotStatus.BUSY.value
            self._camera.current_task = task.task_id
        try:
            quality = self._quality_for(task.order_id)
            self.bridge.send_quality_result(quality)
            return TaskResult(
                task_id=task.task_id,
                robot_id="CAMERA",
                status=TaskStatus.FINISHED.value,
                start_time=started,
                end_time=time.time(),
                message=f"virtual camera inspection completed: {quality}",
                quality_result=quality,
            )
        finally:
            with self._camera_lock:
                if self._camera.status != RobotStatus.FAULT.value:
                    self._camera.status = RobotStatus.IDLE.value
                self._camera.current_task = None
                self._camera.completed_tasks += 1

    def execute_task_async(
        self, task: Task, callback: Callable[[TaskResult], None]
    ) -> None:
        threading.Thread(
            target=lambda: callback(self.execute_task(task)),
            name=f"simulation-task-{task.task_id}",
            daemon=True,
        ).start()

    def move_to_point(self, robot_id: str, point_name: str) -> bool:
        return self.motion.move_to_point(robot_id, point_name)

    def gripper_open(self, robot_id: str) -> bool:
        return self.motion.gripper_open(robot_id)

    def gripper_close(self, robot_id: str) -> bool:
        return self.motion.gripper_close(robot_id)

    def screw_execute(self, robot_id: str, point_name: str) -> bool:
        return self.motion.screw_execute(robot_id, point_name)

    def robot_home(self, robot_id: str) -> bool:
        return self.motion.robot_home(robot_id)

    def get_robot_states(self) -> list[RobotState]:
        states = self.motion.get_robot_states()
        with self._camera_lock:
            camera = RobotState(
                robot_id=self._camera.robot_id,
                status=self._camera.status,
                current_task=self._camera.current_task,
                position=self._camera.position,
                utilization=self._camera.utilization,
                completed_tasks=self._camera.completed_tasks,
            )
        return states + [camera]

    def set_robot_fault(self, robot_id: str) -> None:
        if robot_id == "CAMERA":
            with self._camera_lock:
                self._camera.status = RobotStatus.FAULT.value
            return
        self.motion.set_robot_fault(robot_id)

    def clear_robot_fault(self, robot_id: str) -> None:
        if robot_id == "CAMERA":
            with self._camera_lock:
                self._camera.status = RobotStatus.IDLE.value
            return
        self.motion.clear_robot_fault(robot_id)


__all__ = ["SimulationCellExecutor"]
