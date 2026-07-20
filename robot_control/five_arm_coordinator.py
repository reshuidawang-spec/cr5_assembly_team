"""Long-lived, single-cycle coordinator for the five-CR5A visual cell."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Optional

from interfaces.types import Task, TaskStatus
from robot_control.motion_timing import JointMotionMonitor
from robot_control.r1_motion import R1_BOX_PLACED, R1_TERMINAL_PLACED
from robot_control.r2_motion import R2_PCB_PLACED
from robot_control.r3_motion import R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION
from robot_control.r4_motion import R4_SCREW_DONE
from robot_control.r5_motion import R5_SORT_DEFECT_DONE, R5_SORT_GOOD_DONE
from robot_control.robot_executor import RobotExecutor
from sim_bridge.coppelia_client import SimBridge


QUALITY_ACTIONS = {
    "good": R5_SORT_GOOD_DONE,
    "defect": R5_SORT_DEFECT_DONE,
}


class FiveArmCoordinator:
    """Execute one visual product through all five robots without reconnects."""

    def __init__(
        self,
        bridge: Optional[SimBridge] = None,
        executor: Optional[RobotExecutor] = None,
        speed_deg_s: float = 50.0,
        hold_seconds: float = 0.8,
        motion_monitor_factory=None,
    ):
        self.bridge = bridge or SimBridge()
        self.executor = executor or RobotExecutor(
            sim_bridge=self.bridge,
            speed_deg_s=speed_deg_s,
            hold_seconds=hold_seconds,
        )
        monitor_host = getattr(self.bridge, "host", "127.0.0.1")
        monitor_port = getattr(self.bridge, "port", 23000)
        self.motion_monitor_factory = motion_monitor_factory or (
            lambda robot_id: JointMotionMonitor(
                monitor_host, monitor_port, robot_id
            )
        )

    @staticmethod
    def _task(
        action: str,
        robot_id: str,
        index: int,
        order_id: str,
        area: str,
        process: str,
    ) -> Task:
        return Task(
            task_id=f"CELL-{index:02d}-{action}",
            order_id=order_id,
            product_type="A",
            process=process,
            target_area=area,
            target_point=action,
            available_robots=[robot_id],
        )

    def _sequence(self, quality: str, order_id: str) -> list[Task]:
        quality = quality.strip().lower()
        if quality not in QUALITY_ACTIONS:
            raise ValueError("quality must be 'good' or 'defect'")
        entries = [
            (R1_BOX_PLACED, "R1", "assembly_area", "assemble"),
            (R2_PCB_PLACED, "R2", "assembly_area", "assemble"),
            (R3_MODULE_PLACED, "R3", "assembly_area", "assemble"),
            (R1_TERMINAL_PLACED, "R1", "assembly_area", "assemble"),
            (
                R3_PRODUCT_TO_INSPECTION,
                "R3",
                "inspection_screw_area",
                "transfer",
            ),
            (R4_SCREW_DONE, "R4", "inspection_screw_area", "screw"),
            (QUALITY_ACTIONS[quality], "R5", "sort_area", f"sort_{quality}"),
        ]
        return [
            self._task(action, robot, index, order_id, area, process)
            for index, (action, robot, area, process) in enumerate(entries, start=1)
        ]

    def _simulation_time(self) -> float:
        return float(self.bridge.sim.getSimulationTime())

    def _set_camera_result(self, quality: str) -> dict[str, Any]:
        signal = "camera_good" if quality == "good" else "camera_defect"
        self.bridge.set_stepping(True)
        before = self._simulation_time()
        try:
            self.bridge.set_string_signal("cell_product_state", signal)
            if not self.bridge.step():
                raise RuntimeError(
                    self.bridge.last_error or "camera result simulation step failed"
                )
            after = self._simulation_time()
        finally:
            self.bridge.set_stepping(False)
        return {
            "quality": quality.upper(),
            "signal": signal,
            "simulation_time_before": before,
            "simulation_time_after": after,
        }

    def execute_cycle(
        self,
        quality: str = "good",
        order_id: str = "FIVE-ARM-DEMO",
    ) -> dict[str, Any]:
        quality = quality.strip().lower()
        tasks = self._sequence(quality, order_id)
        if not self.bridge.is_connected() and not self.bridge.connect():
            raise RuntimeError(
                self.bridge.last_error or "cannot connect to CoppeliaSim"
            )

        scene = Path(self.bridge.scene_path())
        started_wall = time.time()
        evidence: dict[str, Any] = {
            "status": "running",
            "order_id": order_id,
            "quality": quality.upper(),
            "scene": str(scene),
            "started_at_epoch_s": started_wall,
            "tasks": [],
            "camera": None,
        }
        previous_end_sim: Optional[float] = None
        try:
            for task in tasks:
                if task.target_point == R4_SCREW_DONE:
                    evidence["camera"] = self._set_camera_result(quality)
                start_wall = time.time()
                start_sim = self._simulation_time()
                record: dict[str, Any] = {
                    "task": task.to_dict(),
                    "start_wall_epoch_s": start_wall,
                    "start_simulation_time_s": start_sim,
                    "handoff_delay_simulation_s": (
                        None
                        if previous_end_sim is None
                        else max(0.0, start_sim - previous_end_sim)
                    ),
                }
                monitor = None
                monitor_start_error = ""
                try:
                    monitor = self.motion_monitor_factory(
                        task.available_robots[0]
                    )
                    monitor.start()
                except Exception as exc:
                    monitor_start_error = str(exc)
                try:
                    result = self.executor.execute_task(task)
                finally:
                    try:
                        motion_timing = (
                            monitor.stop()
                            if monitor is not None
                            else {
                                "robot_id": task.available_robots[0],
                                "motion_detected": False,
                                "monitor_error": "",
                            }
                        )
                    except Exception as exc:
                        motion_timing = {
                            "robot_id": task.available_robots[0],
                            "motion_detected": False,
                            "monitor_error": str(exc),
                        }
                if monitor_start_error:
                    existing = str(motion_timing.get("monitor_error", ""))
                    motion_timing["monitor_error"] = "; ".join(
                        message
                        for message in (monitor_start_error, existing)
                        if message
                    )
                end_wall = time.time()
                end_sim = self._simulation_time()
                first_motion_sim = motion_timing.get(
                    "first_motion_simulation_time_s"
                )
                handoff_to_first_motion = None
                if previous_end_sim is not None and first_motion_sim is not None:
                    delta = float(first_motion_sim) - previous_end_sim
                    if delta >= -0.05:
                        handoff_to_first_motion = max(0.0, delta)
                record.update(
                    {
                        "end_wall_epoch_s": end_wall,
                        "end_simulation_time_s": end_sim,
                        "wall_duration_s": end_wall - start_wall,
                        "simulation_duration_s": max(0.0, end_sim - start_sim),
                        "motion_timing": motion_timing,
                        "handoff_to_first_motion_simulation_s": (
                            handoff_to_first_motion
                        ),
                        "result": result.to_dict(),
                    }
                )
                evidence["tasks"].append(record)
                previous_end_sim = end_sim
                if result.status != TaskStatus.FINISHED.value:
                    evidence["status"] = "failed"
                    evidence["failed_action"] = task.target_point
                    break
            else:
                evidence["status"] = "finished"
        except Exception as exc:
            evidence["status"] = "failed"
            evidence["error"] = str(exc)
        finally:
            evidence["finished_at_epoch_s"] = time.time()
            evidence["wall_duration_s"] = (
                evidence["finished_at_epoch_s"] - started_wall
            )
            evidence["robot_states"] = [
                state.to_dict() for state in self.executor.get_robot_states()
            ]
        return evidence


__all__ = ["FiveArmCoordinator", "QUALITY_ACTIONS"]
