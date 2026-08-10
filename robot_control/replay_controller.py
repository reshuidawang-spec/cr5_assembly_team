"""单臂预录路径回放控制器 (新方案)

用法:
    from robot_control.replay_controller import ReplayController

    ctl = ReplayController(bridge, "R3")          # bridge 已连好场景
    pts = ctl.load("r3_module_pick_descend")      # 加载预录段
    ctl.replay(pts)                                # 回放 (步进模式)
    ctl.replay(pts, reverse=True)                  # 反走 (抬升/退出)
    ctl.gripper(0.080)                             # 夹爪开度
    ctl.attach("CONTROL_MODULE_SUPPLY")            # attach 工件
    ctl.detach(handle)                             # detach 工件

路径数据: data/captured_paths/*.json (key_poses 定义姿态, 段文件为轨迹)
"""
import json
import math
import time
from pathlib import Path
from typing import Any, Iterable, Optional

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "data" / "captured_paths"


class ReplayController:
    def __init__(self, bridge, robot_id: str, max_vel_deg_s: float = 500.0):
        self.bridge = bridge
        self.sim = bridge.sim
        self.robot_id = robot_id
        self.joints = bridge.get_robot_joint_handles(robot_id)
        self.robot = bridge.get_object_handle(robot_id)
        self._orig_maxvel = []
        for j in self.joints:
            self._orig_maxvel.append(
                self.sim.getObjectFloatParam(j, self.sim.jointfloatparam_maxvel)
            )
            self.sim.setObjectFloatParam(
                j, self.sim.jointfloatparam_maxvel, math.radians(max_vel_deg_s)
            )
        self.bridge.set_stepping(True)

    # ------------------------------------------------------------------
    # 路径加载
    # ------------------------------------------------------------------
    def load(self, name: str) -> list[list[float]]:
        """加载 data/captured_paths/<name>.json 的关节角序列 (度)."""
        d = json.load(open(OUT / f"{name}.json"))
        return d["trajectories"][0]["points_deg"]

    def load_pose(self, key: str) -> list[float]:
        """从 r<臂>_key_poses.json 读关键姿态."""
        f = OUT / f"{self.robot_id.lower()}_key_poses.json"
        return json.load(open(f))["key_poses"][key]

    # ------------------------------------------------------------------
    # 回放
    # ------------------------------------------------------------------
    def replay(self, pts: Iterable[list[float]], reverse: bool = False,
               max_step_deg: float = 5.0) -> None:
        """回放一段轨迹 (步进模式, 每步最大关节变化 max_step_deg 度)."""
        seq = list(reversed(list(pts))) if reverse else list(pts)
        for i in range(len(seq) - 1):
            a, c = seq[i], seq[i + 1]
            gap = max(abs(x - y) for x, y in zip(a, c))
            n = max(1, int(gap / max_step_deg) + 1)
            for k in range(1, n + 1):
                f = k / n
                tgt = [a[m] + (c[m] - a[m]) * f for m in range(6)]
                for j, v in zip(self.joints, [math.radians(x) for x in tgt]):
                    self.sim.setJointPosition(j, v)
                self.bridge.step()
                time.sleep(0.0002)
        self.to_pose(seq[-1])

    def to_pose(self, deg: Iterable[float]) -> None:
        """直接设置到指定关节角 (度)."""
        for j, v in zip(self.joints, [math.radians(x) for x in deg]):
            self.sim.setJointPosition(j, v)
        for _ in range(5):
            self.bridge.step()
        time.sleep(0.1)

    def run_flow(self, flow, on_segment_end=None) -> None:
        """按流程回放: flow = [(段名, 方向1/-1), ...]

        on_segment_end(name, direction) 可选, 每段结束后回调
        (用于 attach/detach/事件等).
        """
        for name, direction in flow:
            pts = self.load(name)
            self.replay(pts, reverse=(direction == -1))
            if on_segment_end is not None:
                on_segment_end(name, direction)

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def gripper(self, gap_m: float) -> bool:
        return self.bridge.set_gripper_gap(self.robot_id, gap_m)

    def attach(self, object_name: str) -> None:
        self.bridge.attach_object(object_name, self.robot_id)

    def detach(self, handle: int) -> None:
        self.bridge.detach_object(handle)

    def place_material(self, name: str, position, yaw_deg: float = 0.0,
                       visible: bool = True) -> int:
        """复位物料: 位置+方向+可见性, 返回 handle."""
        h = self.bridge.get_object_handle(name)
        self.sim.setObjectPosition(h, -1, list(position))
        if abs(yaw_deg) > 1e-6:
            self.sim.setObjectQuaternion(h, -1, [0, 0, 0, 1])
        layer = 1 if visible else 0
        for s in self.sim.getObjectsInTree(h, self.sim.object_shape_type, 0):
            self.sim.setObjectInt32Param(s, self.sim.objintparam_visibility_layer, layer)
        return h

    def reset_to_home(self) -> None:
        for j in self.joints:
            self.sim.setJointPosition(j, 0.0)
        for _ in range(5):
            self.bridge.step()

    def restore_maxvel(self) -> None:
        for j, mv in zip(self.joints, self._orig_maxvel):
            try:
                self.sim.setObjectFloatParam(j, self.sim.jointfloatparam_maxvel, mv)
            except Exception:
                pass
