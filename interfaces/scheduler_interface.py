"""调度模块接口 —— 4号同学实现"""

from abc import ABC, abstractmethod
from typing import List, Callable, Optional
from .types import Order, Task, TaskResult, RobotState


class IScheduler(ABC):
    """任务调度接口

    职责：
      1. 将订单分解为工序任务队列
      2. 根据机械臂状态、优先级、区域占用进行动态任务分配
      3. 处理急单插入和故障重调度

    4号同学需要实现：
      - generate_tasks(orders)  → 订单 → 任务队列
      - schedule(tasks, robots) → 任务 + 机械臂状态 → 分配方案
      - insert_urgent(order)    → 急单插入
      - handle_fault(robot_id)  → 故障重分配
    """

    @abstractmethod
    def generate_tasks(self, orders: List[Order]) -> List[Task]:
        """将订单分解为任务队列，设置前置依赖"""
        ...

    @abstractmethod
    def schedule(
        self,
        tasks: List[Task],
        robots: List[RobotState],
    ) -> List[Task]:
        """根据机械臂状态调度任务，返回更新后的任务列表"""
        ...

    @abstractmethod
    def insert_urgent_order(self, order: Order) -> List[Task]:
        """急单插入：生成高优先级任务并重新调度"""
        ...

    @abstractmethod
    def handle_robot_fault(self, robot_id: str, tasks: List[Task]) -> List[Task]:
        """机械臂故障：将该臂未完成任务重新分配给其他可用机械臂"""
        ...

    @abstractmethod
    def on_task_complete(
        self,
        result: TaskResult,
        tasks: List[Task],
        robots: List[RobotState],
    ) -> List[Task]:
        """任务完成回调：解锁后续任务，返回更新后的任务列表"""
        ...

    @abstractmethod
    def set_state_change_callback(self, callback: Callable) -> None:
        """注册状态变更回调，供 GUI 刷新"""
        ...
