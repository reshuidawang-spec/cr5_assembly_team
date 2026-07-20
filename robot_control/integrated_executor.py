"""Scheduler/GUI adapter around the validated real RobotExecutor."""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from interfaces.robot_interface import IRobotExecutor
from interfaces.types import RobotState, Task, TaskResult, TaskStatus
from robot_control.motion_timing import PersistentJointMotionMonitorFactory
from robot_control.r4_motion import R4_SCREW_DONE
from robot_control.robot_executor import RobotExecutor
from sim_bridge.coppelia_client import SimBridge


class IntegratedRobotExecutor(IRobotExecutor):
    """Add camera quality and first-motion evidence without changing motion code."""

    def __init__(
        self,
        bridge: Optional[SimBridge] = None,
        executor: Optional[RobotExecutor] = None,
        quality_resolver: Optional[Callable[[str], str]] = None,
        speed_deg_s: float = 50.0,
        hold_seconds: float = 0.8,
        motion_monitor_factory: Optional[Callable[[str], object]] = None,
    ):
        self.bridge = bridge or SimBridge()
        self.executor = executor or RobotExecutor(
            sim_bridge=self.bridge,
            speed_deg_s=speed_deg_s,
            hold_seconds=hold_seconds,
        )
        self.quality_resolver = quality_resolver or (lambda order_id: "OK")
        self.motion_monitor_factory = motion_monitor_factory or (
            PersistentJointMotionMonitorFactory(
                self.bridge.host, self.bridge.port
            )
        )
        self._timing_lock = threading.Lock()
        self._previous_task_end_simulation_s: Optional[float] = None

    def _ensure_connected(self) -> None:
        if not self.bridge.is_connected() and not self.bridge.connect():
            raise RuntimeError(
                self.bridge.last_error or "cannot connect to CoppeliaSim"
            )

    def _simulation_time(self) -> Optional[float]:
        try:
            self._ensure_connected()
            return float(self.bridge.sim.getSimulationTime())
        except Exception:
            return None

    def prepare_cycle(
        self, quality: str = "good", preload_both_r5: bool = False
    ) -> dict:
        prepare_monitor = getattr(self.motion_monitor_factory, "prepare", None)
        if prepare_monitor is not None:
            prepare_monitor()
        evidence = self.executor.prepare_cycle(
            quality=quality, preload_both_r5=preload_both_r5
        )
        with self._timing_lock:
            self._previous_task_end_simulation_s = None
        return evidence

    def close(self) -> None:
        close_monitor = getattr(self.motion_monitor_factory, "close", None)
        if close_monitor is not None:
            close_monitor()

    def _prepare_camera(self, task: Task) -> Optional[dict]:
        if task.target_point != R4_SCREW_DONE:
            return None
        quality = self.quality_resolver(task.order_id).upper()
        if quality not in {"OK", "NG"}:
            raise ValueError("quality resolver must return OK or NG")
        signal = "camera_good" if quality == "OK" else "camera_defect"
        self._ensure_connected()
        stepping_was_enabled = bool(
            getattr(self.bridge, "stepping_enabled", False)
        )
        self.bridge.set_stepping(True)
        before = float(self.bridge.sim.getSimulationTime())
        try:
            self.bridge.set_string_signal("cell_product_state", signal)
            if not self.bridge.step():
                raise RuntimeError(
                    self.bridge.last_error or "camera result simulation step failed"
                )
        finally:
            if not stepping_was_enabled:
                self.bridge.set_stepping(False)
        return {
            "quality": quality,
            "signal": signal,
            "simulation_time_before": before,
            "simulation_time_after": float(self.bridge.sim.getSimulationTime()),
        }

    def execute_task(self, task: Task) -> TaskResult:
        task_started = time.time()
        robot_id = task.available_robots[0] if task.available_robots else ""
        monitor = None
        monitor_start_error = ""
        try:
            monitor = self.motion_monitor_factory(robot_id)
            monitor.start()
        except Exception as exc:
            monitor_start_error = str(exc)
        camera = None
        try:
            camera = self._prepare_camera(task)
            result = self.executor.execute_task(task)
        except Exception as exc:
            now = time.time()
            result = TaskResult(
                task_id=task.task_id,
                robot_id=robot_id,
                status=TaskStatus.FAILED.value,
                start_time=task_started,
                end_time=now,
                message=str(exc),
            )
        try:
            timing = (
                monitor.stop()
                if monitor is not None
                else {
                    "robot_id": robot_id,
                    "motion_detected": False,
                    "monitor_error": "",
                }
            )
        except Exception as exc:
            timing = {
                "robot_id": robot_id,
                "motion_detected": False,
                "monitor_error": str(exc),
            }
        if monitor_start_error:
            existing = str(timing.get("monitor_error", ""))
            timing["monitor_error"] = "; ".join(
                message for message in (monitor_start_error, existing) if message
            )
        timing["task_call_wall_epoch_s"] = task_started
        first_motion_wall = timing.get("first_motion_wall_epoch_s")
        timing["task_call_to_first_motion_wall_s"] = (
            None
            if first_motion_wall is None
            else max(0.0, float(first_motion_wall) - task_started)
        )
        end_simulation = self._simulation_time()
        with self._timing_lock:
            previous_end = self._previous_task_end_simulation_s
            first_motion = timing.get("first_motion_simulation_time_s")
            handoff = None
            if previous_end is not None and first_motion is not None:
                delta = float(first_motion) - previous_end
                if delta >= -0.05:
                    handoff = max(0.0, delta)
            self._previous_task_end_simulation_s = end_simulation
        result.metrics.update(
            {
                "motion_timing": timing,
                "handoff_to_first_motion_simulation_s": handoff,
                "task_end_simulation_time_s": end_simulation,
            }
        )
        if camera is not None:
            result.metrics["camera_transition"] = camera
            result.quality_result = camera["quality"]
        return result

    def execute_task_async(
        self, task: Task, callback: Callable[[TaskResult], None]
    ) -> None:
        thread = threading.Thread(
            target=lambda: callback(self.execute_task(task)), daemon=True
        )
        thread.start()

    def move_to_point(self, robot_id: str, point_name: str) -> bool:
        return self.executor.move_to_point(robot_id, point_name)

    def gripper_open(self, robot_id: str) -> bool:
        return self.executor.gripper_open(robot_id)

    def gripper_close(self, robot_id: str) -> bool:
        return self.executor.gripper_close(robot_id)

    def screw_execute(self, robot_id: str, point_name: str) -> bool:
        return self.executor.screw_execute(robot_id, point_name)

    def robot_home(self, robot_id: str) -> bool:
        return self.executor.robot_home(robot_id)

    def get_robot_states(self) -> list[RobotState]:
        return self.executor.get_robot_states()

    def set_robot_fault(self, robot_id: str) -> None:
        self.executor.set_robot_fault(robot_id)

    def clear_robot_fault(self, robot_id: str) -> None:
        self.executor.clear_robot_fault(robot_id)


__all__ = ["IntegratedRobotExecutor"]
