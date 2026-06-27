"""仿真通信模块接口 —— 2号/3号同学实现"""

from abc import ABC, abstractmethod
from typing import List, Dict, Optional


class ISimBridge(ABC):
    """CoppeliaSim 通信接口

    职责：
      1. 与 CoppeliaSim 建立/断开连接
      2. 获取场景对象句柄
      3. 驱动机器人关节运动
      4. 读取仿真状态

    2号/3号同学需要实现：
      - connect / disconnect
      - get_object_handle(name)
      - move_robot_joints(robot_id, joint_angles)
      - get_robot_pose(robot_id)
      - read_proximity_sensor / read_vision_sensor
    """

    @abstractmethod
    def connect(self, host: str = "127.0.0.1", port: int = 19997) -> bool:
        """连接 CoppeliaSim"""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """断开连接"""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """检查连接状态"""
        ...

    @abstractmethod
    def get_object_handle(self, name: str) -> int:
        """根据物体名称获取句柄"""
        ...

    @abstractmethod
    def get_object_handles(self, names: List[str]) -> Dict[str, int]:
        """批量获取物体句柄"""
        ...

    @abstractmethod
    def move_robot_joints(
        self, robot_id: str, joint_angles: List[float],
    ) -> bool:
        """设置机械臂关节角度"""
        ...

    @abstractmethod
    def move_robot_pose(
        self, robot_id: str, x: float, y: float, z: float,
        roll: float = 0, pitch: float = 0, yaw: float = 0,
    ) -> bool:
        """设置机械臂末端位姿"""
        ...

    @abstractmethod
    def get_robot_pose(self, robot_id: str) -> Optional[Dict]:
        """获取机械臂末端位姿"""
        ...

    @abstractmethod
    def set_gripper(self, robot_id: str, open: bool) -> bool:
        """控制夹爪开合"""
        ...

    @abstractmethod
    def start_simulation(self) -> bool:
        """启动仿真"""
        ...

    @abstractmethod
    def stop_simulation(self) -> bool:
        """停止仿真"""
        ...

    @abstractmethod
    def step(self) -> bool:
        """单步仿真"""
        ...
