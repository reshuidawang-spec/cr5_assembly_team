"""机械臂控制模块接口 —— 3号同学实现"""

from abc import ABC, abstractmethod
from typing import List, Callable, Optional
from .types import Task, TaskResult, RobotState


class IRobotExecutor(ABC):
    """机械臂执行接口

    职责：
      1. 接收 Task，控制机械臂执行对应动作
      2. 管理夹爪、螺丝锁付、运动路径
      3. 返回 TaskResult

    3号同学需要实现：
      - execute_task(task)     → 执行单个任务
      - move_to_point(robot, point) → 机械臂移动到目标点
      - gripper_open / gripper_close  → 夹爪控制
      - screw_execute(robot, point)   → 螺丝锁付
      - get_robot_states()     → 返回所有机械臂状态
    """

    @abstractmethod
    def execute_task(self, task: Task) -> TaskResult:
        """执行单个任务，阻塞或异步均可，返回执行结果"""
        ...

    @abstractmethod
    def execute_task_async(self, task: Task, callback: Callable[[TaskResult], None]) -> None:
        """异步执行任务，完成后调用 callback"""
        ...

    @abstractmethod
    def move_to_point(self, robot_id: str, point_name: str) -> bool:
        """控制指定机械臂移动到目标点位"""
        ...

    @abstractmethod
    def gripper_open(self, robot_id: str) -> bool:
        """夹爪打开"""
        ...

    @abstractmethod
    def gripper_close(self, robot_id: str) -> bool:
        """夹爪闭合"""
        ...

    @abstractmethod
    def screw_execute(self, robot_id: str, point_name: str) -> bool:
        """螺丝锁付动作"""
        ...

    @abstractmethod
    def robot_home(self, robot_id: str) -> bool:
        """机械臂回零"""
        ...

    @abstractmethod
    def get_robot_states(self) -> List[RobotState]:
        """返回所有机械臂当前状态"""
        ...

    @abstractmethod
    def set_robot_fault(self, robot_id: str) -> None:
        """模拟故障（调试用）"""
        ...

    @abstractmethod
    def clear_robot_fault(self, robot_id: str) -> None:
        """清除故障"""
        ...
