"""Object and target names defined by the five-CR5A scene contract.

The authoritative names come from ``Five_CR5A_Cell_Control_Interface.md`` and
``scenes/main_cell_generator.lua``. Robot joint paths are intentionally not
hard-coded because imported CR5A models contain nested trees; callers must
discover aliases ``joint1`` through ``joint6`` below ``/R1`` ... ``/R5``.
"""

from __future__ import annotations


SCENE_ROOT = "/FiveCR5A_Cell"
ROBOT_IDS = ("R1", "R2", "R3", "R4", "R5")
ARM_JOINT_ALIASES = tuple(f"joint{index}" for index in range(1, 7))

ROBOT_ROOTS = {robot_id: f"/{robot_id}" for robot_id in ROBOT_IDS}
ROBOT_BASES = {
    robot_id: f"{SCENE_ROOT}/RobotBases/{robot_id}_Base"
    for robot_id in ROBOT_IDS
}
ROBOT_TIPS = {
    "R1": "R1_gripper_tip",
    "R2": "R2_gripper_tip",
    "R3": "R3_gripper_tip",
    "R4": "R4_tool_tip",
    "R5": "R5_gripper_tip",
}
ROBOT_TOOL_ROOTS = {
    "R1": "/R1/R1_ROBOTIQ85",
}

TARGET_GROUPS = {
    "R1": "R1_Targets",
    "R2": "R2_Targets",
    "R3": "R3_Targets",
    "R4": "R4_Targets",
    "R5": "R5_Targets",
    "SENSOR": "Sensor_Targets",
}

ROBOT_TARGET_NAMES = {
    "R1": (
        "R1_HOME_REF",
        "R1_BOX_PICK_APP",
        "R1_BOX_PICK_TCP",
        "R1_BOX_PLACE_APP",
        "R1_BOX_PLACE_TCP",
        "R1_TERMINAL_PICK_APP",
        "R1_TERMINAL_PICK_TCP",
        "R1_TERMINAL_PLACE_APP",
        "R1_TERMINAL_PLACE_TCP",
    ),
    "R2": (
        "R2_HOME_REF",
        "R2_PCB_PICK_APP",
        "R2_PCB_PICK_TCP",
        "R2_PCB_PLACE_APP",
        "R2_PCB_PLACE_TCP",
    ),
    "R3": (
        "R3_HOME_REF",
        "R3_MODULE_PICK_APP",
        "R3_MODULE_PICK_TCP",
        "R3_MODULE_PLACE_APP",
        "R3_MODULE_PLACE_TCP",
        "R3_PRODUCT_PICK_APP",
        "R3_PRODUCT_PICK_TCP",
        "R3_PRODUCT_PLACE_INSPECTION_APP",
        "R3_PRODUCT_PLACE_INSPECTION_TCP",
    ),
    "R4": (
        "R4_HOME_REF",
        "R4_SCREW_APP",
        "R4_SCREW_TCP",
        "R4_SCREW_PRESS",
    ),
    "R5": (
        "R5_HOME_REF",
        "R5_PRODUCT_PICK_APP",
        "R5_PRODUCT_PICK_TCP",
        "R5_GOOD_PLACE_APP",
        "R5_GOOD_PLACE_TCP",
        "R5_DEFECT_PLACE_APP",
        "R5_DEFECT_PLACE_TCP",
    ),
}


def _target_paths() -> dict[str, str]:
    result: dict[str, str] = {}
    for robot_id, target_names in ROBOT_TARGET_NAMES.items():
        group = TARGET_GROUPS[robot_id]
        for target_name in target_names:
            result[target_name] = f"{SCENE_ROOT}/Targets/{group}/{target_name}"
    result["CAMERA_INSPECTION_CENTER"] = (
        f"{SCENE_ROOT}/Targets/{TARGET_GROUPS['SENSOR']}/"
        "CAMERA_INSPECTION_CENTER"
    )
    return result


POINTS = _target_paths()
HOME_POINTS = {
    robot_id: POINTS[f"{robot_id}_HOME_REF"] for robot_id in ROBOT_IDS
}

PARTS = {
    "BOX_BLANK": f"{SCENE_ROOT}/Parts/Box_Blank",
    "PCB_SUPPLY": f"{SCENE_ROOT}/Parts/PCB_Supply",
    "CONTROL_MODULE_SUPPLY": f"{SCENE_ROOT}/Parts/Control_Module_Supply",
    "TERMINAL_BLOCK_SUPPLY": f"{SCENE_ROOT}/Parts/Terminal_Block_Supply",
    "ASSEMBLY_PRODUCT": f"{SCENE_ROOT}/Parts/Assembly_ControlBox_Product",
    "INSPECTION_PRODUCT": f"{SCENE_ROOT}/Parts/Inspection_ControlBox_Product",
}

AREAS = {
    "BOX_SUPPLY": f"{SCENE_ROOT}/Areas/Box_Supply_Area",
    "TERMINAL_SUPPLY": f"{SCENE_ROOT}/Areas/Terminal_Supply_Area",
    "PCB_SUPPLY": f"{SCENE_ROOT}/Areas/PCB_Supply_Area",
    "MODULE_SUPPLY": f"{SCENE_ROOT}/Areas/Module_Supply_Area",
    "ASSEMBLY": f"{SCENE_ROOT}/Areas/Assembly_Area",
    "ASSEMBLY_FIXTURE": f"{SCENE_ROOT}/Areas/Assembly_Fixture",
    "INSPECTION_SCREW": f"{SCENE_ROOT}/Areas/Inspection_Screw_Area",
    "INSPECTION_PLATFORM": f"{SCENE_ROOT}/Areas/Inspection_Platform",
}

CONVEYORS = {
    "GOOD": f"{SCENE_ROOT}/Conveyors/Good_Conveyor",
    "DEFECT": f"{SCENE_ROOT}/Conveyors/Defect_Conveyor",
}

ROS_TOPICS = {
    "MAIN_COMMAND": "/compact_cell/main_cmd",
    "GLOBAL_COMMAND": "/compact_cell/cmd",
    "STATUS": "/compact_cell/status",
    **{
        f"{robot_id}_COMMAND": f"/compact_cell/{robot_id.lower()}_cmd"
        for robot_id in ROBOT_IDS
    },
}

PROCESS_COMMANDS = {
    "RESET_CELL",
    "R1_READY",
    "R1_BOX_PLACED",
    "R1_TERMINAL_PLACED",
    "R2_READY",
    "R2_PCB_PLACED",
    "R3_READY",
    "R3_MODULE_PLACED",
    "R3_PRODUCT_TO_INSPECTION",
    "R4_READY",
    "R4_SCREW_DONE",
    "R5_READY",
    "R5_SORT_GOOD_DONE",
    "R5_SORT_DEFECT_DONE",
}

# Validated R1 runtime-only safety regions. Other robot regions are added only
# after their scene-specific calibration; they must not be guessed here.
WORKSPACES = {
    "R1": {
        "lower": (-2.25, -0.12, 0.04),
        "upper": (-0.82, 1.18, 1.55),
    },
    "R1_PRIVATE_SUPPLY": {
        "lower": (-2.08, 0.00, 0.15),
        "upper": (-1.55, 0.55, 0.65),
        "lock_required": False,
    },
    "R2": {
        "lower": (-1.90, -0.55, 0.04),
        "upper": (-0.95, 0.38, 1.55),
    },
    "R2_PRIVATE_SUPPLY": {
        "lower": (-1.46, -0.42, 0.14),
        "upper": (-1.10, -0.14, 0.55),
        "lock_required": False,
    },
    "R3": {
        "lower": (-1.40, -0.38, 0.04),
        "upper": (0.45, 0.72, 1.55),
    },
    "R3_PRIVATE_SUPPLY": {
        "lower": (-1.02, -0.20, 0.14),
        "upper": (-0.68, 0.10, 0.55),
        "lock_required": False,
    },
    "R4": {
        "lower": (-0.05, -0.20, 0.04),
        "upper": (0.80, 0.65, 1.55),
    },
    "R5": {
        "lower": (-0.68, -1.48, 0.04),
        "upper": (0.98, 0.48, 1.55),
    },
    "ASSEMBLY_SHARED": {
        "lower": (-1.43, -0.04, 0.20),
        "upper": (-0.84, 0.48, 0.68),
        "lock_required": True,
        "max_robots": 1,
    },
    "INSPECTION_SHARED": {
        "lower": (-0.55, -0.35, 0.20),
        "upper": (0.80, 0.65, 0.75),
        "lock_required": True,
        "max_robots": 1,
    },
}


def normalize_robot_id(robot_id: str) -> str:
    normalized = robot_id.strip().upper()
    if normalized not in ROBOT_ROOTS:
        raise KeyError(f"unknown robot id: {robot_id}")
    return normalized


def get_point_path(name: str) -> str:
    """Return the exact five-arm target path for a contract target name."""
    try:
        return POINTS[name]
    except KeyError as exc:
        raise KeyError(f"unknown target point: {name}") from exc


def get_robot_root_path(robot_id: str) -> str:
    return ROBOT_ROOTS[normalize_robot_id(robot_id)]


def get_robot_base_path(robot_id: str) -> str:
    return ROBOT_BASES[normalize_robot_id(robot_id)]


def get_home_path(robot_id: str) -> str:
    return HOME_POINTS[normalize_robot_id(robot_id)]


def get_tip_alias(robot_id: str) -> str:
    return ROBOT_TIPS[normalize_robot_id(robot_id)]


def get_joint_alias(robot_id: str, joint_num: int) -> str:
    normalize_robot_id(robot_id)
    if joint_num not in range(1, 7):
        raise ValueError(f"joint number must be 1..6, got {joint_num}")
    return ARM_JOINT_ALIASES[joint_num - 1]


def get_joint_path(robot_id: str, joint_num: int) -> str:
    """Return a path hint; real clients must recursively resolve the alias."""
    robot_id = normalize_robot_id(robot_id)
    return f"/{robot_id}/{get_joint_alias(robot_id, joint_num)}"


def resolve_object_path(name: str) -> str:
    """Resolve a contract name or pass through an absolute scene path."""
    if name.startswith("/"):
        return name
    for mapping in (POINTS, ROBOT_ROOTS, ROBOT_BASES, PARTS, AREAS, CONVEYORS):
        if name in mapping:
            return mapping[name]
    raise KeyError(f"unknown scene object name: {name}")
