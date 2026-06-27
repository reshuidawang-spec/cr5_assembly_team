"""Mock 机械臂执行器 —— 3号同学替换为真实实现"""

import time
import threading
from typing import List, Callable, Dict
from interfaces.robot_interface import IRobotExecutor
from interfaces.types import Task, TaskResult, RobotState, TaskStatus


class MockRobotExecutor(IRobotExecutor):
    """模拟机械臂执行：用 sleep 模拟动作耗时，返回假结果"""

    def __init__(self):
        self._robots: Dict[str, RobotState] = {
            "R1": RobotState(robot_id="R1", status="idle", position="home"),
            "R2": RobotState(robot_id="R2", status="idle", position="home"),
            "R3": RobotState(robot_id="R3", status="idle", position="home"),
            "R4": RobotState(robot_id="R4", status="idle", position="home"),
        }
        self._sim_time = 0.0
        self._lock = threading.Lock()

    def execute_task(self, task: Task) -> TaskResult:
        robot_id = task.available_robots[0] if task.available_robots else "R1"
        with self._lock:
            self._robots[robot_id].status = "busy"
            self._robots[robot_id].current_task = task.task_id
            self._robots[robot_id].position = task.target_point

        start = self._sim_time
        time.sleep(task.duration * 0.1)  # Mock 加速 10 倍
        self._sim_time += task.duration
        end = self._sim_time

        # R3 检测任务随机给出 OK / NG
        quality = ""
        if task.process == "inspect":
            import random
            quality = random.choice(["OK", "NG"])

        with self._lock:
            self._robots[robot_id].status = "idle"
            self._robots[robot_id].current_task = None
            self._robots[robot_id].completed_tasks += 1

        return TaskResult(
            task_id=task.task_id,
            robot_id=robot_id,
            status=TaskStatus.FINISHED.value,
            start_time=start,
            end_time=end,
            message=f"{task.process} completed at {task.target_point}",
            quality_result=quality,
        )

    def execute_task_async(self, task: Task, callback: Callable[[TaskResult], None]) -> None:
        def _run():
            result = self.execute_task(task)
            callback(result)
        t = threading.Thread(target=_run, daemon=True)
        t.start()

    def move_to_point(self, robot_id: str, point_name: str) -> bool:
        with self._lock:
            if robot_id in self._robots:
                self._robots[robot_id].position = point_name
                return True
        return False

    def gripper_open(self, robot_id: str) -> bool:
        return robot_id in self._robots

    def gripper_close(self, robot_id: str) -> bool:
        return robot_id in self._robots

    def screw_execute(self, robot_id: str, point_name: str) -> bool:
        return robot_id in self._robots

    def robot_home(self, robot_id: str) -> bool:
        with self._lock:
            if robot_id in self._robots:
                self._robots[robot_id].position = "home"
                return True
        return False

    def get_robot_states(self) -> List[RobotState]:
        with self._lock:
            return [RobotState(
                robot_id=r.robot_id,
                status=r.status,
                current_task=r.current_task,
                position=r.position,
                utilization=r.utilization,
                completed_tasks=r.completed_tasks,
            ) for r in self._robots.values()]

    def set_robot_fault(self, robot_id: str) -> None:
        with self._lock:
            if robot_id in self._robots:
                self._robots[robot_id].status = "fault"

    def clear_robot_fault(self, robot_id: str) -> None:
        with self._lock:
            if robot_id in self._robots:
                self._robots[robot_id].status = "idle"
