"""场景对象管理 —— 2号实现

Compact Multi-CR5 Collaborative Cell 场景对象映射。
场景由 build_scene.lua 生成，路径在 /CompactCell 分组下。
"""

# 标准点位路径（3号/4号按此查找）
# 场景中所有 P_* 点位的完整路径
POINTS = {
    # R1 上料
    "P_FEED_01": "/CompactCell/Targets/P_FEED_01",

    # R2 装配
    "P_ASSEMBLY_01": "/CompactCell/Targets/P_ASSEMBLY_01",
    "P_ASSEMBLY_02": "/CompactCell/Targets/P_ASSEMBLY_02",

    # R3 锁付/检测
    "P_SCREW_01": "/CompactCell/Targets/P_SCREW_01",
    "P_SCREW_02": "/CompactCell/Targets/P_SCREW_02",
    "P_INSPECT_01": "/CompactCell/Targets/P_INSPECT_01",

    # R4 分拣
    "P_UNLOAD_01": "/CompactCell/Targets/P_UNLOAD_01",
    "P_GOOD_01": "/CompactCell/Targets/P_GOOD_01",
    "P_DEFECT_01": "/CompactCell/Targets/P_DEFECT_01",
    "P_REWORK_01": "/CompactCell/Targets/P_REWORK_01",

    # 传送带
    "P_CONVEYOR_START": "/CompactCell/Targets/P_CONVEYOR_START",
    "P_CONVEYOR_END": "/CompactCell/Targets/P_CONVEYOR_END",

    # 共享区
    "P_SHARED_TRANSFER": "/CompactCell/Targets/P_SHARED_TRANSFER",
}

# 机械臂基座路径
ROBOT_BASES = {
    "R1": "/CompactCell/RobotBases/R1_Base",
    "R2": "/CompactCell/RobotBases/R2_Base",
    "R3": "/CompactCell/RobotBases/R3_Base",
    "R4": "/CompactCell/RobotBases/R4_Base",
}

# 机械臂 Home 位
HOME_POINTS = {
    "R1": "/CompactCell/Targets/R1_Home",
    "R2": "/CompactCell/Targets/R2_Home",
    "R3": "/CompactCell/Targets/R3_Home",
    "R4": "/CompactCell/Targets/R4_Home",
}

# 关节命名：保持 URDF 导入后的默认名 joint1..6
# 每个机械臂是独立模型，路径自然隔离：/R1/joint1, /R2/joint1
JOINT_PATTERN = "/{robot}/joint{joint}"


def get_point_path(name: str) -> str:
    """根据标准点位名返回 CoppeliaSim 路径"""
    return POINTS.get(name, f"/CompactCell/Targets/{name}")


def get_robot_base_path(robot_id: str) -> str:
    """返回机械臂基座路径"""
    return ROBOT_BASES.get(robot_id, f"/CompactCell/RobotBases/{robot_id}_Base")


def get_home_path(robot_id: str) -> str:
    """返回机械臂 Home 位路径"""
    return HOME_POINTS.get(robot_id, f"/CompactCell/Targets/{robot_id}_Home")


def get_joint_path(robot_id: str, joint_num: int) -> str:
    """返回关节路径，如 R1_joint1"""
    return JOINT_PATTERN.format(robot=robot_id, joint=joint_num)
