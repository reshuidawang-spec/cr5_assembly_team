"""Execution adapter that commits completed task state to CoppeliaSim."""

from __future__ import annotations

import threading
from typing import Callable

from interfaces.robot_interface import IRobotExecutor
from interfaces.types import RobotState, Task, TaskResult, TaskStatus
from sim_bridge.coppelia_client import SimBridge


class SceneAwareExecutor(IRobotExecutor):
    """Wrap an executor and update scene signals only after task success."""

    def __init__(self, executor: IRobotExecutor, bridge: SimBridge):
        self.executor = executor
        self.bridge = bridge
        # The ZeroMQ Remote API client owns one REQ socket. Robot tasks may
        # complete concurrently, but scene state commits must use that socket
        # one at a time.
        self._scene_lock = threading.Lock()

    def execute_task(self, task: Task) -> TaskResult:
        result = self.executor.execute_task(task)
        if result.status != TaskStatus.FINISHED.value:
            return result
        try:
            with self._scene_lock:
                if result.quality_result:
                    self.bridge.send_quality_result(result.quality_result)
                if task.scene_command:
                    self.bridge.send_process_command(task.scene_command)
            return result
        except Exception as exc:
            return TaskResult(
                task_id=result.task_id,
                robot_id=result.robot_id,
                status=TaskStatus.FAILED.value,
                start_time=result.start_time,
                end_time=result.end_time,
                message=f"scene state update failed: {exc}",
                quality_result=result.quality_result,
            )

    def execute_task_async(
        self,
        task: Task,
        callback: Callable[[TaskResult], None],
    ) -> None:
        threading.Thread(
            target=lambda: callback(self.execute_task(task)),
            daemon=True,
        ).start()

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

    def stop_simulation(self) -> bool:
        """Serialize simulation stop with any final scene-state commit."""
        with self._scene_lock:
            return self.bridge.stop_simulation()
