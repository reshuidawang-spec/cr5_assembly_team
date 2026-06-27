"""Mock 仿真桥接 —— 2号/3号同学替换为真实 CoppeliaSim 连接"""

from typing import List, Dict, Optional
from interfaces.sim_interface import ISimBridge


class MockSimBridge(ISimBridge):
    """模拟仿真桥接：所有操作返回成功，用于离线开发调试"""

    def __init__(self):
        self._connected = False
        self._simulating = False
        self._handles: Dict[str, int] = {}
        self._counter = 1000

    def connect(self, host: str = "127.0.0.1", port: int = 19997) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False
        self._simulating = False

    def is_connected(self) -> bool:
        return self._connected

    def get_object_handle(self, name: str) -> int:
        if name not in self._handles:
            self._counter += 1
            self._handles[name] = self._counter
        return self._handles[name]

    def get_object_handles(self, names: List[str]) -> Dict[str, int]:
        return {name: self.get_object_handle(name) for name in names}

    def move_robot_joints(self, robot_id: str, joint_angles: List[float]) -> bool:
        return self._connected

    def move_robot_pose(
        self, robot_id: str, x: float, y: float, z: float,
        roll: float = 0, pitch: float = 0, yaw: float = 0,
    ) -> bool:
        return self._connected

    def get_robot_pose(self, robot_id: str) -> Optional[Dict]:
        return {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}

    def set_gripper(self, robot_id: str, open: bool) -> bool:
        return self._connected

    def start_simulation(self) -> bool:
        self._simulating = True
        return True

    def stop_simulation(self) -> bool:
        self._simulating = False
        return True

    def step(self) -> bool:
        return self._simulating
