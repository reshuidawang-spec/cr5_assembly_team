"""R3 native-gripper replay for module installation and product transfer.

The R3 paths are replayed from the RViz/MoveIt captures saved during manual
teaching. Runtime execution does not solve IK and does not create a suction
TCP: the scene's native R3 gripper and ``R3_gripper_tip`` are used directly.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import math
import threading
from pathlib import Path
from typing import Any, Iterable, Optional

from robot_control.r1_motion import (
    PLAN_PATH as R1_PLAN_PATH,
    cumulative_max_joint_distance,
    interpolate_path,
    load_r1_plan,
    minimum_jerk,
)
from robot_control.runtime_cartesian import (
    build_tip_pose_path,
    build_tip_translation_path,
    find_unique_alias,
    interpolate_joint_line,
    join_paths,
    solve_target,
    tip_pose_for_config,
    unwrap_path,
    wrap_near,
)
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import (
    AREAS,
    PARTS,
    ROBOT_BASES,
    ROBOT_TIPS,
    WORKSPACES,
)


R3_MODULE_PLACED = "R3_MODULE_PLACED"
R3_PRODUCT_TO_INSPECTION = "R3_PRODUCT_TO_INSPECTION"
R3_PRODUCT_TRANSFER_CLEARANCE = "R3_PRODUCT_TRANSFER_CLEARANCE"
R3_ACTIONS = frozenset({R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION})

PLAN_VERSION = 1
PLAN_PATH = Path(__file__).with_name("plans") / "r3_gripper_cycle_plan.json"
ROBOT_ID = "R3"
SCENE_NAME = "compact_cell1ttt.ttt"

TARGET_NAMES = (
    "R3_HOME_REF",
    "R3_MODULE_PICK_APP",
    "R3_MODULE_PICK_TCP",
    "R3_MODULE_PLACE_APP",
    "R3_MODULE_PLACE_TCP",
    "R3_PRODUCT_PICK_APP",
    "R3_PRODUCT_PICK_TCP",
    "R3_PRODUCT_PLACE_INSPECTION_APP",
    "R3_PRODUCT_PLACE_INSPECTION_TCP",
)
PROTECTED_TARGETS = {
    "R3_HOME_REF": {
        "position": [-0.60, 0.35, 0.70],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R3_MODULE_PICK_APP": {
        "position": [-0.78, -0.20, 0.451],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R3_MODULE_PICK_TCP": {
        "position": [-0.78, -0.20, 0.271],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R3_MODULE_PLACE_APP": {
        "position": [-1.105, 0.145, 0.4833],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R3_MODULE_PLACE_TCP": {
        "position": [-1.105, 0.145, 0.3033],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R3_PRODUCT_PICK_APP": {
        "position": [-1.08, 0.12, 0.432],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R3_PRODUCT_PICK_TCP": {
        "position": [-1.08, 0.12, 0.252],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R3_PRODUCT_PLACE_INSPECTION_APP": {
        "position": [-0.04, 0.00, 0.432],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R3_PRODUCT_PLACE_INSPECTION_TCP": {
        "position": [-0.04, 0.00, 0.252],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
}

MODULE_REQUIRED_PATHS = (
    "initial_to_pick_app",
    "pick_descend",
    "lift_and_transfer",
    "place_descend",
    "retreat_to_clear",
)
PRODUCT_REQUIRED_PATHS = (
    "clear_to_pick_app",
    "pick_descend",
    "lift_and_transfer",
    "place_descend",
    "retreat_and_return_home",
)
REQUIRED_PATHS_BY_ACTION = {
    R3_MODULE_PLACED: MODULE_REQUIRED_PATHS,
    R3_PRODUCT_TO_INSPECTION: PRODUCT_REQUIRED_PATHS,
}

RUNTIME_BRIDGE_ALIAS = "R3_Runtime_Command_Bridge"
RUNTIME_BRIDGE_CODE = """function sysCall_init()
end

function setJointTargets(handles, targets)
    for i=1,#handles do
        sim.setJointTargetPosition(handles[i], targets[i])
    end
end

function getJointPositions(handles)
    local positions = {}
    for i=1,#handles do
        positions[i] = sim.getJointPosition(handles[i])
    end
    return positions
end
"""

BOX_ASSEMBLY_POSITION = (-1.078563, 0.120898, 0.21946)
PCB_ASSEMBLY_POSITION = (-1.080058, 0.120636, 0.264725)
MODULE_SUPPLY_POSITION = (-0.78, -0.20, 0.1665)
MODULE_GRASP_POSITION = (-0.78, -0.20, 0.202)
MODULE_ASSEMBLY_POSITION = (-1.105, 0.145, 0.3033)
TERMINAL_SUPPLY_POSITION = (-1.82, -0.02, 0.1665)
TERMINAL_ASSEMBLY_POSITION = (-1.05397, 0.086735, 0.271573)
INSPECTION_PRODUCT_POSITION = (-0.04, 0.00, 0.216)
ASSEMBLY_PRODUCT_GRASP_POSITION = (-1.08, 0.12, 0.262)
INSPECTION_PRODUCT_ORIENTATION = (0.0, 0.0, 0.0)
INSPECTION_PRODUCT_APPROACH_POSITION = (
    INSPECTION_PRODUCT_POSITION[0],
    INSPECTION_PRODUCT_POSITION[1],
    INSPECTION_PRODUCT_POSITION[2]
    + (
        PROTECTED_TARGETS["R3_PRODUCT_PLACE_INSPECTION_APP"]["position"][2]
        - PROTECTED_TARGETS["R3_PRODUCT_PLACE_INSPECTION_TCP"]["position"][2]
    ),
)
PRODUCT_RELEASE_SETTLE_STEPS = 0
PRODUCT_STANDARD_ALIGNMENT_STEPS = 10
VERTICAL_APPROACH_CLEARANCE_M = 0.180
PRODUCT_BOX_WIDTH_M = 0.150
PRODUCT_GRASP_OPEN_GAP_M = 0.170
PRODUCT_GRASP_CLOSED_GAP_M = PRODUCT_BOX_WIDTH_M
# The finger-link command assumes 20 mm pads, while the rendered rubber-pad
# bounding boxes project 26.4 mm across the closing axis.  Compensate only the
# command; contact validation below still measures the real 150 mm opening.
PRODUCT_GRASP_COMMAND_GAP_M = 0.1564
PRODUCT_GRASP_SURFACE_ALIGNMENT_TOLERANCE_DEG = 1.0
PRODUCT_GRASP_CENTER_TOLERANCE_M = 0.006
PRODUCT_BOX_HEIGHT_M = 0.072
INSPECTION_HANDOFF_TOLERANCE_M = 0.002
PRODUCT_TRANSFER_CLEARANCE_Z_M = 0.48
PRODUCT_FLANGE_ORIENTATION_TOLERANCE_DEG = 0.75
# R3 is fixed to one base.  The inspection handoff was moved into the shared
# R3/R4/R5 workspace so no robot or workpiece has to translate independently.
R3_FIXED_BASE_XY_M = (-0.60, 0.40)
R3_FIXED_BASE_POSITION_TOLERANCE_M = 0.001
PRODUCT_SECURE_TRANSFER_MAX_TILT_DEG = 0.75
PRODUCT_FIXED_ROUTE_POSES = (
    (-0.90, -0.05, PRODUCT_TRANSFER_CLEARANCE_Z_M),
    (-0.68, -0.20, PRODUCT_TRANSFER_CLEARANCE_Z_M),
    (-0.42, -0.22, PRODUCT_TRANSFER_CLEARANCE_Z_M),
)

POSITION_TOLERANCE_M = 0.008
TARGET_TOLERANCE = 1e-6
JOINT_TOLERANCE_DEG = 0.35
WORKSPACE_TOLERANCE_M = 0.003
TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_CAP_DEG_S = 36.0
HOLD_SECONDS = 0.8
MAX_UNWRAPPED_JOINT_RAD = 2.0 * math.pi + 1e-6
MAX_PATH_BOUNDARY_JUMP_RAD = math.radians(0.5)

CAMERA_VIEW_PATH = (
    "/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area"
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _near(first: list[float], second: list[float], tolerance: float) -> bool:
    return len(first) == len(second) and max(
        abs(a - b) for a, b in zip(first, second)
    ) <= tolerance


def _finite_joint_path(value: Any) -> bool:
    return (
        isinstance(value, list)
        and len(value) >= 2
        and all(
            isinstance(config, list)
            and len(config) == 6
            and all(math.isfinite(float(joint)) for joint in config)
            for config in value
        )
    )


def _max_joint_gap(first: list[float], second: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(first, second))


def _path_has_invalid_joint_branch(configs: list[list[float]]) -> bool:
    if any(
        abs(float(joint)) > MAX_UNWRAPPED_JOINT_RAD
        for config in configs
        for joint in config
    ):
        return True
    return any(
        _max_joint_gap(first, second) > math.pi
        for first, second in zip(configs, configs[1:])
    )


def load_r3_plan(path: Path = PLAN_PATH) -> dict[str, Any]:
    """Load and structurally validate the R3 native-gripper replay plan."""
    try:
        plan = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load R3 plan {path}: {exc}") from exc

    if plan.get("plan_version") != PLAN_VERSION:
        raise RuntimeError(f"unsupported R3 plan version in {path}")
    if plan.get("tool") != "native_gripper":
        raise RuntimeError("R3 plan is not for the native gripper")
    if plan.get("tip_link") != "R3_gripper_tip":
        raise RuntimeError("R3 plan does not use R3_gripper_tip")
    if plan.get("protected_targets_modified") is not False:
        raise RuntimeError("R3 plan does not preserve the protected Git targets")

    protected = plan.get("protected_targets")
    if not isinstance(protected, dict) or set(protected) != set(TARGET_NAMES):
        raise RuntimeError("R3 plan target snapshot is incomplete")
    for name, expected in PROTECTED_TARGETS.items():
        actual = protected.get(name, {})
        if not _near(actual.get("position", []), expected["position"], TARGET_TOLERANCE):
            raise RuntimeError(f"R3 plan target position differs: {name}")
        if not _near(
            actual.get("orientation_euler", []),
            expected["orientation_euler"],
            TARGET_TOLERANCE,
        ):
            raise RuntimeError(f"R3 plan target orientation differs: {name}")

    paths = plan.get("paths")
    if not isinstance(paths, dict):
        raise RuntimeError("R3 plan has no paths")
    for action, names in REQUIRED_PATHS_BY_ACTION.items():
        action_paths = paths.get(action)
        if not isinstance(action_paths, dict):
            raise RuntimeError(f"R3 plan has no paths for {action}")
        for name in names:
            if not _finite_joint_path(action_paths.get(name)):
                raise RuntimeError(f"R3 plan path is invalid: {action}.{name}")
            if _path_has_invalid_joint_branch(action_paths[name]):
                raise RuntimeError(
                    f"R3 plan uses an invalid joint branch: {action}.{name}"
                )
        for first_name, second_name in zip(names, names[1:]):
            if (
                _max_joint_gap(action_paths[first_name][-1], action_paths[second_name][0])
                > MAX_PATH_BOUNDARY_JUMP_RAD
            ):
                raise RuntimeError(
                    "R3 plan path boundary jumps: "
                    f"{action}.{first_name} -> {action}.{second_name}"
                )

    workspace = plan.get("workspace", {})
    expected_workspace = WORKSPACES["R3"]
    if tuple(workspace.get("lower", ())) != tuple(expected_workspace["lower"]):
        raise RuntimeError("R3 plan lower workspace wall differs from the contract")
    if tuple(workspace.get("upper", ())) != tuple(expected_workspace["upper"]):
        raise RuntimeError("R3 plan upper workspace wall differs from the contract")
    for key in ("assembly_shared", "inspection_shared"):
        expected_shared = WORKSPACES[key.upper()]
        shared = workspace.get(key, {})
        if tuple(shared.get("lower", ())) != tuple(expected_shared["lower"]):
            raise RuntimeError(f"R3 plan {key} lower bound differs")
        if tuple(shared.get("upper", ())) != tuple(expected_shared["upper"]):
            raise RuntimeError(f"R3 plan {key} upper bound differs")

    validation = plan.get("validation", {})
    fingerprint = validation.get("scene_fingerprint", {})
    if not isinstance(fingerprint.get("sha256"), str) or not isinstance(
        fingerprint.get("size"), int
    ):
        raise RuntimeError("R3 plan has no validated scene fingerprint")
    return plan


def _quaternion_multiply(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    rotated = _quaternion_multiply(
        _quaternion_multiply((x, y, z, w), (*vector, 0.0)),
        (-x, -y, -z, w),
    )
    return rotated[:3]


def _compose_poses(first: list[float], second: list[float]) -> list[float]:
    translated = _rotate_vector(tuple(first[3:]), tuple(second[:3]))
    return [
        first[index] + translated[index] for index in range(3)
    ] + list(_quaternion_multiply(tuple(first[3:]), tuple(second[3:])))


def _inverse_pose(pose: Iterable[float]) -> list[float]:
    values = [float(value) for value in pose]
    inverse_quaternion = (
        -values[3],
        -values[4],
        -values[5],
        values[6],
    )
    inverse_position = _rotate_vector(
        inverse_quaternion,
        (-values[0], -values[1], -values[2]),
    )
    return [*inverse_position, *inverse_quaternion]


def _surface_aligned_product_grasp_quaternion(
    source_quaternion: Iterable[float],
) -> tuple[float, float, float, float]:
    """Keep the tool approach axis vertical and align jaws to box walls.

    The scene gripper extends along local X and closes along local Y.  The
    taught product APP already points local X vertically.  Rotate only around
    local X so the pads become parallel to the box's front/rear walls.
    """
    quaternion = tuple(float(value) for value in source_quaternion)
    closing_axis = _rotate_vector(quaternion, (0.0, 1.0, 0.0))
    current_yaw = math.atan2(closing_axis[1], closing_axis[0])
    delta = (
        0.5 * math.pi - current_yaw + 0.5 * math.pi
    ) % math.pi - 0.5 * math.pi
    local_x_rotation = (
        math.sin(0.5 * delta),
        0.0,
        0.0,
        math.cos(0.5 * delta),
    )
    return _quaternion_multiply(quaternion, local_x_rotation)


def _path_suffix_from_config(
    path: list[list[float]],
    config: list[float],
    label: str,
) -> list[list[float]]:
    index = min(
        range(len(path)),
        key=lambda candidate: _max_joint_gap(path[candidate], config),
    )
    gap = _max_joint_gap(path[index], config)
    if gap > math.radians(0.5):
        raise RuntimeError(
            f"{label} has no APP boundary; nearest gap is "
            f"{math.degrees(gap):.3f} deg"
        )
    suffix = [list(config)] + [list(item) for item in path[index + 1 :]]
    if len(suffix) < 2:
        raise RuntimeError(f"{label} APP suffix has no motion")
    return suffix


def _shape_tree_bounds(
    sim: Any,
    shapes: set[int],
    shape_bbs: dict[int, tuple[list[float], list[float]]],
) -> tuple[list[float], list[float]]:
    lower = [math.inf, math.inf, math.inf]
    upper = [-math.inf, -math.inf, -math.inf]
    for shape in shapes:
        size, bb_pose = shape_bbs[shape]
        world_bb_pose = _compose_poses(sim.getObjectPose(shape, -1), bb_pose)
        for signs in itertools.product((-0.5, 0.5), repeat=3):
            local = tuple(size[index] * signs[index] for index in range(3))
            rotated = _rotate_vector(tuple(world_bb_pose[3:]), local)
            point = [
                world_bb_pose[index] + rotated[index] for index in range(3)
            ]
            lower = [min(a, b) for a, b in zip(lower, point)]
            upper = [max(a, b) for a, b in zip(upper, point)]
    return lower, upper


class R3SafetyGuard:
    """R3 environment, self, payload, and workspace checks."""

    def __init__(
        self,
        sim: Any,
        robot: int,
        payload: Optional[int] = None,
        ignored_environment: Optional[set[int]] = None,
        allowed_payload_contacts: Optional[set[int]] = None,
    ):
        self.sim = sim
        ignored_environment = ignored_environment or set()
        self.allowed_payload_contacts = set(allowed_payload_contacts or ())
        robot_shapes = set(
            sim.getObjectsInTree(robot, sim.object_shape_type, 0)
        )
        self.payload_shapes = (
            set(sim.getObjectsInTree(payload, sim.object_shape_type, 0))
            if payload is not None
            else set()
        )
        self.mover_shapes = robot_shapes | self.payload_shapes
        self.mover = sim.createCollection(1)
        self.environment = sim.createCollection(1)
        for handle in self.mover_shapes:
            sim.addItemToCollection(
                self.mover, sim.handle_single, handle, 0
            )

        self.robot_mover: Optional[int] = None
        self.allowed_contact_environment: Optional[int] = None
        if self.allowed_payload_contacts:
            self.robot_mover = sim.createCollection(1)
            self.allowed_contact_environment = sim.createCollection(1)
            for handle in robot_shapes:
                sim.addItemToCollection(
                    self.robot_mover, sim.handle_single, handle, 0
                )
            for handle in self.allowed_payload_contacts:
                sim.addItemToCollection(
                    self.allowed_contact_environment,
                    sim.handle_single,
                    handle,
                    0,
                )

        robot_base = sim.getObject(ROBOT_BASES[ROBOT_ID])
        for handle in sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        ):
            if handle in self.mover_shapes or handle == robot_base:
                continue
            if handle in ignored_environment:
                continue
            if handle in self.allowed_payload_contacts:
                continue
            if sim.getObjectInt32Param(
                handle, sim.objintparam_visibility_layer
            ) == 0:
                continue
            sim.addItemToCollection(
                self.environment, sim.handle_single, handle, 0
            )

        link_aliases = ["base_link_respondable"] + [
            f"Link{index}_respondable" for index in range(1, 7)
        ]
        by_alias = {
            sim.getObjectAlias(handle): handle for handle in robot_shapes
        }
        missing = [alias for alias in link_aliases if alias not in by_alias]
        if missing:
            raise RuntimeError(f"R3 collision links missing: {missing}")
        self.links = [by_alias[alias] for alias in link_aliases]

        self.payload_collection: Optional[int] = None
        self.arm_collection: Optional[int] = None
        if payload is not None:
            self.payload_collection = sim.createCollection(1)
            self.arm_collection = sim.createCollection(1)
            for handle in self.payload_shapes:
                sim.addItemToCollection(
                    self.payload_collection, sim.handle_single, handle, 0
                )
            for handle in self.links:
                sim.addItemToCollection(
                    self.arm_collection, sim.handle_single, handle, 0
                )

        self.shape_bbs = {
            handle: sim.getShapeBB(handle) for handle in self.mover_shapes
        }

    def close(self) -> None:
        self.sim.destroyCollection(self.mover)
        self.sim.destroyCollection(self.environment)
        if self.robot_mover is not None:
            self.sim.destroyCollection(self.robot_mover)
        if self.allowed_contact_environment is not None:
            self.sim.destroyCollection(self.allowed_contact_environment)
        if self.payload_collection is not None:
            self.sim.destroyCollection(self.payload_collection)
        if self.arm_collection is not None:
            self.sim.destroyCollection(self.arm_collection)

    def check(
        self,
        label: str,
        check_workspace: bool = True,
        check_internal: bool = True,
    ) -> None:
        state, pair = self.sim.checkCollision(self.mover, self.environment)
        if state:
            paths = [self.sim.getObjectAlias(handle, 1) for handle in pair]
            raise RuntimeError(f"collision during {label}: {paths}")
        if (
            self.robot_mover is not None
            and self.allowed_contact_environment is not None
        ):
            state, pair = self.sim.checkCollision(
                self.robot_mover, self.allowed_contact_environment
            )
            if state:
                paths = [self.sim.getObjectAlias(handle, 1) for handle in pair]
                raise RuntimeError(f"collision during {label}: {paths}")
        if check_internal:
            for index, first in enumerate(self.links):
                for second in self.links[index + 2 :]:
                    state, pair = self.sim.checkCollision(first, second)
                    if state:
                        paths = [
                            self.sim.getObjectAlias(handle, 1) for handle in pair
                        ]
                        raise RuntimeError(
                            f"R3 self collision during {label}: {paths}"
                        )
            if self.payload_collection is not None:
                state, pair = self.sim.checkCollision(
                    self.payload_collection, self.arm_collection
                )
                if state:
                    paths = [
                        self.sim.getObjectAlias(handle, 1) for handle in pair
                    ]
                    raise RuntimeError(
                        f"payload-to-R3 collision during {label}: {paths}"
                    )
        if not check_workspace:
            return

        lower, upper = _shape_tree_bounds(
            self.sim, self.mover_shapes, self.shape_bbs
        )
        allowed = WORKSPACES[ROBOT_ID]
        for axis, actual_low, actual_high, allowed_low, allowed_high in zip(
            "xyz", lower, upper, allowed["lower"], allowed["upper"]
        ):
            if (
                actual_low < allowed_low - WORKSPACE_TOLERANCE_M
                or actual_high > allowed_high + WORKSPACE_TOLERANCE_M
            ):
                raise RuntimeError(
                    f"R3 workspace violation during {label}: axis={axis}, "
                    f"actual=[{actual_low:.4f},{actual_high:.4f}], "
                    f"allowed=[{allowed_low:.4f},{allowed_high:.4f}]"
                )


class _SmoothRunner:
    def __init__(
        self,
        bridge: SimBridge,
        robot: int,
        joints: list[int],
        command_script: int,
        payloads: list[int],
        ignored_environment: Optional[set[int]],
        allowed_payload_contacts: Optional[dict[int, set[int]]],
        collision_check_interval: int,
        workspace_check_interval: int,
    ):
        self.bridge = bridge
        self.sim = bridge.sim
        self.joints = joints
        self.command_script = command_script
        self.collision_check_interval = max(1, collision_check_interval)
        self.workspace_check_interval = max(1, workspace_check_interval)
        self.dt = float(self.sim.getSimulationTimeStep())
        allowed_payload_contacts = allowed_payload_contacts or {}
        self.guards = {
            None: R3SafetyGuard(
                self.sim, robot, ignored_environment=ignored_environment
            )
        }
        for payload in payloads:
            self.guards[payload] = R3SafetyGuard(
                self.sim,
                robot,
                payload,
                ignored_environment=ignored_environment,
                allowed_payload_contacts=allowed_payload_contacts.get(payload),
            )
        self.guard = self.guards[None]
        self.active_payload: Optional[int] = None
        self.payload_world_position_lock: Optional[list[float]] = None
        self.payload_world_orientation_lock: Optional[list[float]] = None
        self.step_index = 0

    def close(self) -> None:
        for guard in self.guards.values():
            guard.close()

    def set_payload(self, payload: Optional[int]) -> None:
        self.guard = self.guards[payload]
        self.active_payload = payload

    def lock_payload_world_orientation(
        self, orientation_euler: Optional[Iterable[float]]
    ) -> None:
        self.payload_world_orientation_lock = (
            None if orientation_euler is None else list(orientation_euler)
        )

    def lock_payload_world_position(
        self, position: Optional[Iterable[float]]
    ) -> None:
        self.payload_world_position_lock = (
            None if position is None else list(position)
        )

    def joint_positions(self) -> list[float]:
        return [
            float(value)
            for value in self.sim.callScriptFunction(
                "getJointPositions", self.command_script, self.joints
            )
        ]

    def step(
        self,
        label: str,
        force_collision: bool = False,
        force_full: bool = False,
    ) -> None:
        if not self.bridge.step():
            raise RuntimeError(
                self.bridge.last_error or "R3 simulation step failed"
            )
        if (
            self.payload_world_position_lock is not None
            and self.active_payload is not None
        ):
            self.sim.setObjectPosition(
                self.active_payload,
                -1,
                self.payload_world_position_lock,
            )
        if (
            self.payload_world_orientation_lock is not None
            and self.active_payload is not None
        ):
            self.sim.setObjectOrientation(
                self.active_payload,
                -1,
                self.payload_world_orientation_lock,
            )
        self.step_index += 1
        collision_due = (
            force_collision
            or force_full
            or self.step_index % self.collision_check_interval == 0
        )
        if not collision_due:
            return
        full_due = (
            force_full
            or self.step_index % self.workspace_check_interval == 0
        )
        self.guard.check(
            label,
            check_workspace=full_due,
            check_internal=full_due,
        )

    def hold(self, seconds: float, label: str) -> None:
        for _ in range(max(1, math.ceil(seconds / self.dt))):
            self.step(label)

    def execute_path(
        self,
        label: str,
        configs: list[list[float]],
        peak_speed_rad_s: float,
    ) -> None:
        cumulative = cumulative_max_joint_distance(configs)
        total = cumulative[-1]
        if total <= 1e-12:
            raise RuntimeError(f"{label} has no joint motion")
        duration = max(0.55, 1.875 * total / peak_speed_rad_s)
        step_count = max(2, math.ceil(duration / self.dt))
        for index in range(1, step_count + 1):
            progress = minimum_jerk(index / step_count)
            target = interpolate_path(configs, cumulative, total * progress)
            self.sim.callScriptFunction(
                "setJointTargets", self.command_script, self.joints, target
            )
            self.step(label)

        final = configs[-1]
        for _ in range(100):
            errors = [
                abs(actual - expected)
                for actual, expected in zip(self.joint_positions(), final)
            ]
            if max(errors) <= math.radians(0.12):
                break
            self.step(f"{label} settle")
        else:
            raise RuntimeError(f"{label} did not settle at its endpoint")
        self.guard.check(f"{label} endpoint")

    def execute_path_with_payload_pose(
        self,
        label: str,
        configs: list[list[float]],
        peak_speed_rad_s: float,
        payload_start_position: Iterable[float],
        payload_target_position: Iterable[float],
        payload_orientation: Iterable[float],
    ) -> None:
        cumulative = cumulative_max_joint_distance(configs)
        total = cumulative[-1]
        if total <= 1e-12:
            raise RuntimeError(f"{label} has no joint motion")
        start_position = list(payload_start_position)
        target_position = list(payload_target_position)
        orientation = list(payload_orientation)
        duration = max(0.55, 1.875 * total / peak_speed_rad_s)
        step_count = max(2, math.ceil(duration / self.dt))
        for index in range(1, step_count + 1):
            progress = minimum_jerk(index / step_count)
            target = interpolate_path(configs, cumulative, total * progress)
            self.lock_payload_world_position(
                [
                    start + (finish - start) * progress
                    for start, finish in zip(start_position, target_position)
                ]
            )
            self.lock_payload_world_orientation(orientation)
            self.sim.callScriptFunction(
                "setJointTargets", self.command_script, self.joints, target
            )
            self.step(f"{label} [{index}/{step_count}]")

        final = configs[-1]
        self.lock_payload_world_position(target_position)
        self.lock_payload_world_orientation(orientation)
        for _ in range(100):
            errors = [
                abs(actual - expected)
                for actual, expected in zip(self.joint_positions(), final)
            ]
            if max(errors) <= math.radians(0.12):
                break
            self.step(f"{label} settle")
        else:
            raise RuntimeError(f"{label} did not settle at its endpoint")
        self.guard.check(f"{label} endpoint")


def _tree_visibility(sim: Any, root: int) -> dict[int, int]:
    return {
        handle: int(sim.getObjectInt32Param(handle, sim.objintparam_visibility_layer))
        for handle in sim.getObjectsInTree(root, sim.handle_all, 0)
    }


def _set_visibility(sim: Any, layers: dict[int, int], visible: bool) -> None:
    layer = 1 if visible else 0
    for handle in layers:
        sim.setObjectInt32Param(
            handle,
            sim.objintparam_visibility_layer,
            layer,
        )


def _restore_visibility(sim: Any, layers: dict[int, int]) -> None:
    for handle, layer in layers.items():
        sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, layer)


def _set_world_pose(
    sim: Any,
    handle: int,
    position: Iterable[float],
    orientation: Iterable[float],
) -> None:
    sim.setObjectPosition(handle, -1, list(position))
    sim.setObjectOrientation(handle, -1, list(orientation))


def _angle_delta(start: float, finish: float) -> float:
    delta = finish - start
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def _settle_visible_object_pose(
    runner: _SmoothRunner,
    handle: int,
    target_position: tuple[float, float, float],
    target_orientation: tuple[float, float, float],
    steps: int,
    label: str,
) -> None:
    sim = runner.sim
    start_position = [float(value) for value in sim.getObjectPosition(handle, -1)]
    start_orientation = [
        float(value) for value in sim.getObjectOrientation(handle, -1)
    ]
    orientation_delta = [
        _angle_delta(start, finish)
        for start, finish in zip(start_orientation, target_orientation)
    ]
    for index in range(1, max(1, steps) + 1):
        fraction = index / max(1, steps)
        progress = minimum_jerk(fraction)
        sim.setObjectPosition(
            handle,
            -1,
            [
                start + (finish - start) * progress
                for start, finish in zip(start_position, target_position)
            ],
        )
        sim.setObjectOrientation(
            handle,
            -1,
            [
                start + delta * progress
                for start, delta in zip(start_orientation, orientation_delta)
            ],
        )
        runner.step(label, force_collision=index == max(1, steps))
    _set_world_pose(sim, handle, target_position, target_orientation)


class R3MotionController:
    """Execute the taught R3 gripper actions."""

    def __init__(
        self,
        bridge: SimBridge,
        r1_plan_path: Path = R1_PLAN_PATH,
        r3_plan_path: Path = PLAN_PATH,
        assembly_lock: Optional[threading.Lock] = None,
        inspection_lock: Optional[threading.Lock] = None,
        speed_deg_s: float = TRANSFER_SPEED_DEG_S,
        hold_seconds: float = HOLD_SECONDS,
        collision_check_interval: int = 5,
        workspace_check_interval: int = 20,
    ):
        if speed_deg_s <= 0.0:
            raise ValueError("speed_deg_s must be positive")
        self.bridge = bridge
        self.r1_plan_path = Path(r1_plan_path)
        self.r3_plan_path = Path(r3_plan_path)
        self.assembly_lock = assembly_lock or threading.Lock()
        self.inspection_lock = inspection_lock or threading.Lock()
        self.speed_deg_s = float(speed_deg_s)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.collision_check_interval = int(collision_check_interval)
        self.workspace_check_interval = int(workspace_check_interval)
        self._prepared_paths: dict[str, dict[str, list[list[float]]]] = {}
        self._prepared_pick_alignment_paths: dict[
            str, list[list[float]]
        ] = {}
        self._prepared_grasp_paths: dict[str, list[list[float]]] = {}
        self._prepared_place_app_configs: dict[str, list[float]] = {}
        self._prepared_product_transfer_path: list[list[float]] = []
        self._prepared_product_place_path: list[list[float]] = []
        self._prepared_product_retreat_path: list[list[float]] = []
        self._prepared_product_grasp_quaternion = (0.0, 0.0, 0.0, 1.0)
        self._prepared_product_flange_tilt_deg = 0.0
        self._prepared_product_surface_alignment_error_deg = 0.0
        self._prepared_product_max_orientation_error_deg = 0.0
        self._product_grasp_surface_evidence: dict[str, Any] = {}
        self._pre_positioned_config: dict[str, list[float]] = {}
        self._continuous_stepping = False
        self._coordinated_mode = False
        self._assembly_entry_waits: dict[str, Any] = {}

    def _target_snapshot(self) -> dict[str, dict[str, list[float]]]:
        result = {}
        for name in TARGET_NAMES:
            pose = self.bridge.get_target_pose(name)
            result[name] = {
                "position": [round(float(value), 9) for value in pose["position"]],
                "orientation_euler": [
                    round(float(value), 9) for value in pose["orientation"]
                ],
            }
        return result

    def _validate_static(self) -> dict[str, Any]:
        r1_plan = load_r1_plan(self.r1_plan_path)
        r3_plan = load_r3_plan(self.r3_plan_path)
        scene = Path(self.bridge.scene_path())
        if scene.name != SCENE_NAME:
            raise RuntimeError(f"unexpected CoppeliaSim scene: {scene}")
        scene_sha256 = _sha256(scene)
        scene_size = scene.stat().st_size
        for label, plan in (("R1", r1_plan), ("R3", r3_plan)):
            fingerprint = plan["validation"]["scene_fingerprint"]
            if scene_size != int(fingerprint.get("size", -1)):
                raise RuntimeError(
                    f"{label} scene size differs; repeat full preflight"
                )
            if scene_sha256 != fingerprint["sha256"]:
                raise RuntimeError(
                    f"{label} scene hash differs; repeat full preflight"
                )

        current_targets = self._target_snapshot()
        for name, expected in PROTECTED_TARGETS.items():
            actual = current_targets[name]
            if not _near(
                actual["position"], expected["position"], TARGET_TOLERANCE
            ) or not _near(
                actual["orientation_euler"],
                expected["orientation_euler"],
                TARGET_TOLERANCE,
            ):
                raise RuntimeError(f"protected Git target changed: {name}")
        return r3_plan

    def _validate_part(
        self,
        object_name: str,
        expected_position: tuple[float, float, float],
        tolerance: float = POSITION_TOLERANCE_M,
    ) -> int:
        sim = self.bridge.sim
        handle = self.bridge.get_object_handle(object_name)
        parts = sim.getObject(PARTS[object_name].rsplit("/", 1)[0])
        if sim.getObjectParent(handle) != parts:
            raise RuntimeError(f"{object_name} is not owned by /Parts")
        actual = [float(value) for value in sim.getObjectPosition(handle, -1)]
        if not _near(actual, list(expected_position), tolerance):
            raise RuntimeError(
                f"{object_name} is not at its validated position: {actual}"
            )
        return handle

    def _validate_fixed_base(self) -> None:
        sim = self.bridge.sim
        robot = self.bridge.get_object_handle(ROBOT_ID)
        position = [float(value) for value in sim.getObjectPosition(robot, -1)]
        error = math.hypot(
            position[0] - R3_FIXED_BASE_XY_M[0],
            position[1] - R3_FIXED_BASE_XY_M[1],
        )
        if error > R3_FIXED_BASE_POSITION_TOLERANCE_M:
            raise RuntimeError(
                "R3 base is not at its validated fixed position: "
                f"error={error * 1000.0:.3f} mm"
            )

    def _validate_module_assembly_ready(self) -> None:
        self._validate_part("BOX_BLANK", BOX_ASSEMBLY_POSITION)
        self._validate_part("PCB_SUPPLY", PCB_ASSEMBLY_POSITION, 0.010)

    def _validate_module_preflight(
        self,
        paths: dict[str, list[list[float]]],
        verify_static: bool,
    ) -> None:
        sim = self.bridge.sim
        if sim.getSimulationState() == sim.simulation_stopped:
            raise RuntimeError("R3 module action requires the coordinated scene")
        if verify_static:
            self._validate_static()
        self._validate_fixed_base()

        expected_start = self._pre_positioned_config.get(
            R3_MODULE_PLACED,
            paths["initial_to_pick_app"][0],
        )
        if not _near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            expected_start,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError(
                "R3 is not at the validated module start "
                f"(expected pre-positioned={R3_MODULE_PLACED in self._pre_positioned_config})"
            )
        self._validate_part("CONTROL_MODULE_SUPPLY", MODULE_SUPPLY_POSITION)
        if not self._coordinated_mode:
            self._validate_module_assembly_ready()

    def _validate_product_preflight(
        self,
        paths: dict[str, list[list[float]]],
        verify_static: bool,
    ) -> None:
        sim = self.bridge.sim
        if sim.getSimulationState() == sim.simulation_stopped:
            raise RuntimeError("R3 product transfer requires the coordinated scene")
        if verify_static:
            self._validate_static()
        self._validate_fixed_base()
        expected_start = self._pre_positioned_config.get(
            R3_PRODUCT_TO_INSPECTION,
            paths["clear_to_pick_app"][0],
        )
        if not _near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            expected_start,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError(
                "R3 is not at the validated product-transfer start "
                f"(expected pre-positioned={R3_PRODUCT_TO_INSPECTION in self._pre_positioned_config})"
            )
        self._validate_part("BOX_BLANK", BOX_ASSEMBLY_POSITION)
        self._validate_part("PCB_SUPPLY", PCB_ASSEMBLY_POSITION, 0.010)
        self._validate_part("CONTROL_MODULE_SUPPLY", MODULE_ASSEMBLY_POSITION)
        self._validate_part("TERMINAL_BLOCK_SUPPLY", TERMINAL_ASSEMBLY_POSITION)

    def _paths(self, action: str) -> dict[str, list[list[float]]]:
        prepared = self._prepared_paths.get(action)
        if prepared is not None:
            return prepared
        plan = load_r3_plan(self.r3_plan_path)
        return {
            name: [
                [float(joint) for joint in config]
                for config in plan["paths"][action][name]
            ]
            for name in REQUIRED_PATHS_BY_ACTION[action]
        }

    def prepare(self, action: str) -> dict[str, Any]:
        if action not in R3_ACTIONS:
            raise ValueError(f"unsupported R3 action: {action}")
        sim = self.bridge.sim
        if sim.getSimulationState() != sim.simulation_stopped:
            raise RuntimeError("R3 preparation requires a stopped scene")
        plan = self._validate_static()
        if not _near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            [0.0] * 6,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError("R3 is not zero during preparation")
        prepared_paths = {
            name: [
                [float(joint) for joint in config]
                for config in plan["paths"][action][name]
            ]
            for name in REQUIRED_PATHS_BY_ACTION[action]
        }
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        robot = self.bridge.get_object_handle(ROBOT_ID)
        self._validate_fixed_base()
        joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
        tip = find_unique_alias(sim, robot, ROBOT_TIPS[ROBOT_ID])
        target_position = (
            MODULE_GRASP_POSITION
            if action == R3_MODULE_PLACED
            else ASSEMBLY_PRODUCT_GRASP_POSITION
        )
        pick_app_config = list(prepared_paths["pick_descend"][0])
        pick_app_pose = tip_pose_for_config(
            sim, tip, joints, pick_app_config
        )
        if action == R3_PRODUCT_TO_INSPECTION:
            product_grasp_quaternion = (
                _surface_aligned_product_grasp_quaternion(pick_app_pose[3:])
            )
            approach_axis = _rotate_vector(
                product_grasp_quaternion, (1.0, 0.0, 0.0)
            )
            closing_axis = _rotate_vector(
                product_grasp_quaternion, (0.0, 1.0, 0.0)
            )
            flange_tilt_deg = math.degrees(
                math.acos(min(1.0, abs(approach_axis[2])))
            )
            surface_alignment_error_deg = math.degrees(
                math.acos(min(1.0, abs(closing_axis[1])))
            )
            if flange_tilt_deg > PRODUCT_FLANGE_ORIENTATION_TOLERANCE_DEG:
                raise RuntimeError(
                    "R3 taught product grasp does not point the real gripper "
                    f"approach axis down: tilt={flange_tilt_deg:.3f} deg"
                )
            if (
                surface_alignment_error_deg
                > PRODUCT_GRASP_SURFACE_ALIGNMENT_TOLERANCE_DEG
            ):
                raise RuntimeError(
                    "R3 product jaws are not parallel to the box surfaces: "
                    f"error={surface_alignment_error_deg:.3f} deg"
                )
            self._prepared_product_grasp_quaternion = (
                product_grasp_quaternion
            )
            self._prepared_product_flange_tilt_deg = flange_tilt_deg
            self._prepared_product_surface_alignment_error_deg = (
                surface_alignment_error_deg
            )
            top_pose = [
                ASSEMBLY_PRODUCT_GRASP_POSITION[0],
                ASSEMBLY_PRODUCT_GRASP_POSITION[1],
                pick_app_pose[2],
                *product_grasp_quaternion,
            ]
            horizontal = build_tip_translation_path(
                sim,
                client.require("simIK"),
                robot,
                tip,
                joints,
                pick_app_config,
                top_pose[:3],
                61,
                "R3_product_safe_height_horizontal_alignment",
            )
            orientation = build_tip_pose_path(
                sim,
                client.require("simIK"),
                robot,
                tip,
                joints,
                horizontal[-1],
                top_pose,
                81,
                "R3_product_safe_height_top_orientation",
            )
            pick_alignment = join_paths(horizontal, orientation)
            self._prepared_grasp_paths[action] = build_tip_translation_path(
                sim,
                client.require("simIK"),
                robot,
                tip,
                joints,
                pick_alignment[-1],
                ASSEMBLY_PRODUCT_GRASP_POSITION,
                61,
                "R3_product_vertical_grasp_contact",
            )

            pickup_payload_pose = [
                *BOX_ASSEMBLY_POSITION,
                0.0,
                0.0,
                0.0,
                1.0,
            ]
            pickup_tip_pose = [
                *ASSEMBLY_PRODUCT_GRASP_POSITION,
                *product_grasp_quaternion,
            ]
            payload_in_tip = _compose_poses(
                _inverse_pose(pickup_tip_pose), pickup_payload_pose
            )
            target_tip_pose = _compose_poses(
                [
                    *INSPECTION_PRODUCT_APPROACH_POSITION,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ],
                _inverse_pose(payload_in_tip),
            )
            target_tip_tcp_pose = _compose_poses(
                [
                    *INSPECTION_PRODUCT_POSITION,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ],
                _inverse_pose(payload_in_tip),
            )
            desired_quaternion = target_tip_pose[3:]

            # Build one continuous flange-down route around the fixed base.
            # The inspection station now lies in the shared R3/R4/R5 workspace.
            transfer_segments: list[list[list[float]]] = []
            current = list(pick_alignment[-1])
            route_poses = [
                [*position, *product_grasp_quaternion]
                for position in PRODUCT_FIXED_ROUTE_POSES
            ]
            route_poses.append(target_tip_pose)
            for index, route_pose in enumerate(route_poses, start=1):
                segment = build_tip_pose_path(
                    sim,
                    client.require("simIK"),
                    robot,
                    tip,
                    joints,
                    current,
                    route_pose,
                    61,
                    f"R3_product_fixed_transfer_{index}",
                )
                transfer_segments.append(segment)
                current = segment[-1]
            self._prepared_product_place_path = build_tip_pose_path(
                sim,
                client.require("simIK"),
                robot,
                tip,
                joints,
                current,
                target_tip_tcp_pose,
                61,
                "R3_product_fixed_vertical_place",
            )
            self._prepared_product_transfer_path = join_paths(
                *transfer_segments
            )
            self._prepared_product_retreat_path = list(
                reversed(self._prepared_product_transfer_path)
            )
            max_orientation_error = 0.0
            for config in join_paths(
                self._prepared_product_transfer_path,
                self._prepared_product_place_path,
            ):
                pose = tip_pose_for_config(sim, tip, joints, config)
                quaternion_dot = min(
                    1.0,
                    abs(
                        sum(
                            actual * expected
                            for actual, expected in zip(
                                pose[3:], desired_quaternion
                            )
                        )
                    ),
                )
                max_orientation_error = max(
                    max_orientation_error,
                    math.degrees(2.0 * math.acos(quaternion_dot)),
                )
            if (
                max_orientation_error
                > PRODUCT_SECURE_TRANSFER_MAX_TILT_DEG
            ):
                raise RuntimeError(
                    "R3 clamped product transfer exceeds its taught tilt "
                    f"limit: {max_orientation_error:.3f} deg"
                )
            self._prepared_product_max_orientation_error_deg = (
                max_orientation_error
            )
        else:
            pick_alignment = build_tip_translation_path(
                sim,
                client.require("simIK"),
                robot,
                tip,
                joints,
                pick_app_config,
                (
                    target_position[0],
                    target_position[1],
                    pick_app_pose[2],
                ),
                21,
                f"R3_{action}_safe_height_XY_alignment",
            )
            self._prepared_grasp_paths[action] = build_tip_translation_path(
                sim,
                client.require("simIK"),
                robot,
                tip,
                joints,
                pick_alignment[-1],
                target_position,
                61,
                f"R3_{action}_vertical_grasp_contact",
            )
            prepared_paths["lift_and_transfer"] = _path_suffix_from_config(
                prepared_paths["lift_and_transfer"],
                pick_app_config,
                f"R3 {action} lift_and_transfer",
            )
        retreat_name = "retreat_to_clear"
        if action == R3_PRODUCT_TO_INSPECTION:
            retreat_name = "retreat_and_return_home"
        place_app_config = list(prepared_paths["place_descend"][0])
        if action == R3_MODULE_PLACED:
            prepared_paths[retreat_name] = _path_suffix_from_config(
                prepared_paths[retreat_name],
                place_app_config,
                f"R3 {action} {retreat_name}",
            )
        self._prepared_paths[action] = prepared_paths
        self._prepared_pick_alignment_paths[action] = pick_alignment
        self._prepared_place_app_configs[action] = place_app_config
        evidence = {
            "robot_id": ROBOT_ID,
            "prepared_actions": [action],
            "path_source": str(self.r3_plan_path),
            "path_points": {
                name: len(path)
                for name, path in self._prepared_paths[action].items()
            },
            "pick_alignment_points": len(pick_alignment),
            "vertical_grasp_points": len(
                self._prepared_grasp_paths[action]
            ),
        }
        if action == R3_PRODUCT_TO_INSPECTION:
            evidence.update(
                {
                    "product_transfer_points": len(
                        self._prepared_product_transfer_path
                    ),
                    "product_place_points": len(
                        self._prepared_product_place_path
                    ),
                    "product_retreat_points": len(
                        self._prepared_product_retreat_path
                    ),
                    "flange_down_at_pick_and_place": True,
                    "payload_level_through_transfer": True,
                    "inspection_place_source": "R3_fixed_base",
                    "r3_fixed_base_xy_m": list(R3_FIXED_BASE_XY_M),
                    "rigid_flange_down_fixed_transfer": True,
                    "tool_approach_axis": "local X vertical",
                    "jaw_closing_axis": "box local Y / width surfaces",
                    "product_box_width_m": PRODUCT_BOX_WIDTH_M,
                    "product_grasp_gap_m": PRODUCT_GRASP_CLOSED_GAP_M,
                    "product_grasp_command_gap_m": (
                        PRODUCT_GRASP_COMMAND_GAP_M
                    ),
                    "flange_tilt_deg": round(
                        self._prepared_product_flange_tilt_deg, 6
                    ),
                    "surface_alignment_error_deg": round(
                        self._prepared_product_surface_alignment_error_deg,
                        6,
                    ),
                    "max_clamped_transfer_tilt_deg": round(
                        self._prepared_product_max_orientation_error_deg,
                        6,
                    ),
                }
            )
        return evidence

    def _release_alignment_path(
        self,
        robot: int,
        tip: int,
        joints: list[int],
        payload: int,
        target_position: tuple[float, float, float],
        label: str,
    ) -> list[list[float]]:
        sim = self.bridge.sim
        payload_position = [
            float(value) for value in sim.getObjectPosition(payload, -1)
        ]
        tip_position = [float(value) for value in sim.getObjectPosition(tip, -1)]
        target_tip = [
            tip_position[index] + target_position[index] - payload_position[index]
            for index in range(3)
        ]
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        return build_tip_translation_path(
            sim,
            client.require("simIK"),
            robot,
            tip,
            joints,
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            target_tip,
            25,
            label,
        )

    def _rigid_payload_pose_path(
        self,
        robot: int,
        tip: int,
        joints: list[int],
        payload: int,
        payload_target_pose: Iterable[float],
        point_count: int,
        label: str,
    ) -> list[list[float]]:
        """Move the TCP so an attached payload reaches an exact world pose."""
        sim = self.bridge.sim
        payload_in_tip = [
            float(value) for value in sim.getObjectPose(payload, tip)
        ]
        target_tip_pose = _compose_poses(
            [float(value) for value in payload_target_pose],
            _inverse_pose(payload_in_tip),
        )
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        return build_tip_pose_path(
            sim,
            client.require("simIK"),
            robot,
            tip,
            joints,
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            target_tip_pose,
            point_count,
            label,
        )

    def _tip_joint_alignment_path(
        self,
        robot: int,
        tip: int,
        joints: list[int],
        target_tip_pose: Iterable[float],
        point_count: int,
        label: str,
    ) -> list[list[float]]:
        """Solve a safe-height TCP pose using current and neutral branches."""
        sim = self.bridge.sim
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        sim_ik = client.require("simIK")
        base = find_unique_alias(sim, robot, "base_link_respondable")
        current = self.bridge.get_robot_joint_positions(ROBOT_ID)
        target_handle = sim.createDummy(0.004)
        sim.setObjectAlias(target_handle, f"Runtime_{label}")
        sim.setObjectPose(
            target_handle,
            -1,
            [float(value) for value in target_tip_pose],
        )
        sim.setObjectInt32Param(
            target_handle, sim.objintparam_visibility_layer, 0
        )
        solutions: list[list[float]] = []
        errors: list[str] = []
        try:
            for index, seed in enumerate((current, [0.0] * 6)):
                try:
                    solutions.append(
                        solve_target(
                            sim_ik,
                            base,
                            tip,
                            joints,
                            target_handle,
                            seed,
                            f"{label} seed {index}",
                        )
                    )
                except Exception as exc:
                    errors.append(str(exc))
        finally:
            sim.removeObjects([target_handle])
        if not solutions:
            raise RuntimeError(
                f"{label} has no reachable IK branch: {'; '.join(errors)}"
            )
        target = min(
            solutions,
            key=lambda candidate: _max_joint_gap(current, candidate),
        )
        return interpolate_joint_line(
            current, target, max(3, int(point_count))
        )

    def _rigid_payload_joint_alignment_path(
        self,
        robot: int,
        tip: int,
        joints: list[int],
        payload: int,
        payload_target_pose: Iterable[float],
        point_count: int,
        label: str,
        collision_pairs: Optional[Iterable[int]] = None,
    ) -> list[list[float]]:
        """Reach a rigid-payload pose through the nearest valid IK branch."""
        sim = self.bridge.sim
        payload_in_tip = [
            float(value) for value in sim.getObjectPose(payload, tip)
        ]
        target_tip_pose = _compose_poses(
            [float(value) for value in payload_target_pose],
            _inverse_pose(payload_in_tip),
        )
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        sim_ik = client.require("simIK")
        base = find_unique_alias(sim, robot, "base_link_respondable")
        current = self.bridge.get_robot_joint_positions(ROBOT_ID)
        solutions: list[list[float]] = []
        errors: list[str] = []
        target_handle = sim.createDummy(0.004)
        sim.setObjectAlias(target_handle, f"Runtime_{label}")
        sim.setObjectPose(target_handle, -1, target_tip_pose)
        sim.setObjectInt32Param(
            target_handle, sim.objintparam_visibility_layer, 0
        )
        try:
            for index, seed in enumerate((current, [0.0] * 6)):
                try:
                    solutions.append(
                        solve_target(
                            sim_ik,
                            base,
                            tip,
                            joints,
                            target_handle,
                            seed,
                            f"{label} seed {index}",
                        )
                    )
                except Exception as exc:
                    errors.append(str(exc))
        finally:
            sim.removeObjects([target_handle])
        if not solutions:
            raise RuntimeError(
                f"{label} has no reachable IK branch; "
                f"payload_in_tip={[round(value, 6) for value in payload_in_tip]}; "
                f"target_tip_pose={[round(value, 6) for value in target_tip_pose]}; "
                f"current_joints_deg="
                f"{[round(math.degrees(value), 3) for value in current]}; "
                f"errors={'; '.join(errors)}"
            )
        if collision_pairs is not None:
            planning_errors = []
            for target in sorted(
                solutions,
                key=lambda candidate: _max_joint_gap(current, candidate),
            ):
                try:
                    return self._plan_joint_path_ompl(
                        joints,
                        current,
                        target,
                        collision_pairs,
                        point_count,
                        label,
                    )
                except RuntimeError as exc:
                    planning_errors.append(str(exc))
            raise RuntimeError(
                f"{label} has no collision-free IK/OMPL branch: "
                + "; ".join(planning_errors)
            )
        target = min(
            solutions,
            key=lambda candidate: _max_joint_gap(current, candidate),
        )
        return interpolate_joint_line(
            current, target, max(3, int(point_count))
        )

    def _plan_joint_path_ompl(
        self,
        joints: list[int],
        start: list[float],
        goal: list[float],
        collision_pairs: Iterable[int],
        point_count: int,
        label: str,
    ) -> list[list[float]]:
        """Plan a collision-free joint path for the rigid product transfer."""
        sim = self.bridge.sim
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        previous_client_timeout = getattr(client, "timeout", None)
        socket = getattr(client, "socket", None)
        previous_receive_timeout = None
        zmq_module = None
        try:
            import zmq as zmq_module

            if socket is not None:
                previous_receive_timeout = socket.getsockopt(
                    zmq_module.RCVTIMEO
                )
                socket.setsockopt(zmq_module.RCVTIMEO, 60_000)
        except (ImportError, AttributeError):
            socket = None
        client.timeout = 60.0
        resume_simulation = False
        if sim.getSimulationState() not in (
            sim.simulation_stopped,
            sim.simulation_paused,
        ):
            sim.pauseSimulation()
            resume_simulation = True
        sim_ompl = client.require("simOMPL")
        task = sim_ompl.createTask(f"Runtime_{label}")
        try:
            state_spaces = []
            for index, joint in enumerate(joints):
                cyclic, interval = sim.getJointInterval(joint)
                lower = float(interval[0])
                upper = lower + float(interval[1])
                if cyclic or upper - lower > 4.0 * math.pi:
                    lower, upper = -2.0 * math.pi, 2.0 * math.pi
                state_spaces.append(
                    sim_ompl.createStateSpace(
                        f"{label}_joint_{index + 1}",
                        sim_ompl.StateSpaceType.joint_position,
                        joint,
                        [lower],
                        [upper],
                        1 if index < 3 else 0,
                        1.0,
                    )
                )
            sim_ompl.setStateSpace(task, state_spaces)
            sim_ompl.setAlgorithm(task, sim_ompl.Algorithm.RRTConnect)
            sim_ompl.setCollisionPairs(
                task, [int(value) for value in collision_pairs]
            )
            sim_ompl.setStateValidityCheckingResolution(task, 0.005)
            sim_ompl.setStartState(task, start)
            sim_ompl.setGoalState(task, goal)
            sim_ompl.setup(task)
            if not sim_ompl.isStateValid(task, start):
                raise RuntimeError(f"{label} OMPL start state is in collision")
            if not sim_ompl.isStateValid(task, goal):
                raise RuntimeError(f"{label} OMPL goal state is in collision")
            solved, flat_path = sim_ompl.compute(
                task,
                7.0,
                1.0,
                max(80, int(point_count)),
            )
            if not solved or not flat_path:
                raise RuntimeError(
                    f"{label} has no collision-free OMPL transfer path"
                )
            path = [
                [float(value) for value in flat_path[offset : offset + len(joints)]]
                for offset in range(0, len(flat_path), len(joints))
            ]
            path = unwrap_path(path)
            path[0] = list(start)
            path[-1] = [
                wrap_near(reference, value)
                for reference, value in zip(path[-2], goal)
            ]
            return path
        finally:
            try:
                sim_ompl.destroyTask(task)
            finally:
                # OMPL collision evaluation writes temporary joint states.
                # Restore the live attached-payload start before replay.
                for joint, value in zip(joints, start):
                    sim.setJointPosition(joint, float(value))
                    sim.setJointTargetPosition(joint, float(value))
                if previous_client_timeout is not None:
                    client.timeout = previous_client_timeout
                if (
                    socket is not None
                    and zmq_module is not None
                    and previous_receive_timeout is not None
                ):
                    socket.setsockopt(
                        zmq_module.RCVTIMEO, previous_receive_timeout
                    )
                if resume_simulation:
                    sim.startSimulation()

    def set_continuous_stepping(self, enabled: bool) -> None:
        self._continuous_stepping = bool(enabled)

    def set_pre_positioned(self, action: str, config: list[float]) -> None:
        if action is not None:
            self._pre_positioned_config[action] = list(config)

    def set_coordinated_mode(self, enabled: bool) -> None:
        self._coordinated_mode = bool(enabled)

    def set_assembly_entry_wait(self, action: str, waiter: Any | None) -> None:
        if waiter is None:
            self._assembly_entry_waits.pop(action, None)
        else:
            self._assembly_entry_waits[action] = waiter

    def _wait_before_assembly(self, action: str) -> None:
        waiter = self._assembly_entry_waits.get(action)
        if waiter is None:
            return
        if hasattr(waiter, "wait"):
            waiter.wait()
            return
        if callable(waiter):
            waiter()

    def _create_command_script(self, robot: int) -> int:
        sim = self.bridge.sim
        for handle in sim.getObjectsInTree(
            robot, sim.object_script_type, 0
        ):
            if sim.getObjectAlias(handle) == RUNTIME_BRIDGE_ALIAS:
                sim.removeObjects([handle])
        script = sim.createScript(
            sim.scripttype_simulation,
            RUNTIME_BRIDGE_CODE,
            0,
            "lua",
        )
        sim.setObjectAlias(script, RUNTIME_BRIDGE_ALIAS)
        sim.setObjectParent(script, robot, True)
        return script

    def _ignored_camera_view(self) -> set[int]:
        sim = self.bridge.sim
        ignored: set[int] = set()
        for path in (CAMERA_VIEW_PATH,):
            try:
                root = sim.getObject(path)
            except Exception:
                continue
            ignored.update(
                sim.getObjectsInTree(root, sim.object_shape_type, 0)
            )
            if sim.getObjectType(root) == sim.object_shape_type:
                ignored.add(root)
        return ignored

    def _set_joint_limits_and_start(
        self,
        joints: list[int],
        start_targets: list[float],
    ) -> list[float]:
        sim = self.bridge.sim
        original = [
            sim.getObjectFloatParam(joint, sim.jointfloatparam_maxvel)
            for joint in joints
        ]
        max_velocity = math.radians(max(60.0, self.speed_deg_s * 1.35))
        for joint, target in zip(joints, start_targets):
            sim.setObjectFloatParam(joint, sim.jointfloatparam_maxvel, max_velocity)
            sim.setJointTargetPosition(joint, float(target))
        if not self.bridge.start_simulation():
            raise RuntimeError(self.bridge.last_error or "cannot take R3 stepping")
        return original

    def _restore_joint_limits(
        self, joints: list[int], original_velocities: list[float]
    ) -> None:
        sim = self.bridge.sim
        for joint, original in zip(joints, original_velocities):
            sim.setObjectFloatParam(joint, sim.jointfloatparam_maxvel, original)

    def _open_gripper(self, runner: _SmoothRunner) -> None:
        if not self.bridge.set_gripper(ROBOT_ID, True):
            raise RuntimeError(self.bridge.last_error or "cannot open R3 gripper")
        runner.hold(0.45, "R3 open gripper")

    def _close_gripper(
        self, runner: _SmoothRunner, gap_m: float = 0.150
    ) -> None:
        if not self.bridge.set_gripper_gap(ROBOT_ID, gap_m):
            raise RuntimeError(self.bridge.last_error or "cannot close R3 gripper")
        runner.hold(0.65, "R3 close gripper")

    def _validate_product_surface_grasp(
        self, robot: int, product: int
    ) -> dict[str, Any]:
        """Prove that the live pads meet opposite box-width surfaces."""
        sim = self.bridge.sim
        left = find_unique_alias(sim, robot, "R3T_left_inner_rubber_pad")
        right = find_unique_alias(sim, robot, "R3T_right_inner_rubber_pad")
        left_position = [
            float(value) for value in sim.getObjectPosition(left, -1)
        ]
        right_position = [
            float(value) for value in sim.getObjectPosition(right, -1)
        ]
        pad_line = [
            right_position[index] - left_position[index]
            for index in range(3)
        ]
        pad_center_distance = math.sqrt(sum(value * value for value in pad_line))
        if pad_center_distance <= 1e-9:
            raise RuntimeError("R3 product grasp pads occupy the same position")
        pad_axis = [value / pad_center_distance for value in pad_line]

        product_pose = [
            float(value) for value in sim.getObjectPose(product, -1)
        ]
        box_surface_normal = _rotate_vector(
            tuple(product_pose[3:]), (0.0, 1.0, 0.0)
        )
        axis_dot = min(
            1.0,
            abs(sum(a * b for a, b in zip(pad_axis, box_surface_normal))),
        )
        surface_error_deg = math.degrees(math.acos(axis_dot))

        midpoint = [
            0.5 * (left_position[index] + right_position[index])
            for index in range(3)
        ]
        horizontal_center_error_m = math.hypot(
            midpoint[0] - product_pose[0], midpoint[1] - product_pose[1]
        )
        contact_height_m = midpoint[2] - product_pose[2]

        def projected_half_extent(shape: int) -> float:
            size, bb_pose = sim.getShapeBB(shape)
            world_bb_pose = _compose_poses(
                [float(value) for value in sim.getObjectPose(shape, -1)],
                [float(value) for value in bb_pose],
            )
            extent = 0.0
            for index in range(3):
                local_axis = [0.0, 0.0, 0.0]
                local_axis[index] = 1.0
                world_axis = _rotate_vector(
                    tuple(world_bb_pose[3:]), tuple(local_axis)
                )
                extent += 0.5 * float(size[index]) * abs(
                    sum(a * b for a, b in zip(world_axis, pad_axis))
                )
            return extent

        inner_surface_gap_m = (
            pad_center_distance
            - projected_half_extent(left)
            - projected_half_extent(right)
        )
        evidence = {
            "surface_alignment_error_deg": round(surface_error_deg, 6),
            "horizontal_center_error_m": round(
                horizontal_center_error_m, 6
            ),
            "contact_height_m": round(contact_height_m, 6),
            "inner_surface_gap_m": round(inner_surface_gap_m, 6),
            "box_width_m": PRODUCT_BOX_WIDTH_M,
            "pad_midpoint_world_m": [round(value, 6) for value in midpoint],
        }
        if surface_error_deg > PRODUCT_GRASP_SURFACE_ALIGNMENT_TOLERANCE_DEG:
            raise RuntimeError(
                "R3 live grasp pads do not face the box surfaces: "
                f"error={surface_error_deg:.3f} deg"
            )
        if horizontal_center_error_m > PRODUCT_GRASP_CENTER_TOLERANCE_M:
            raise RuntimeError(
                "R3 live grasp is not centered on the box: "
                f"offset={horizontal_center_error_m * 1000.0:.2f} mm"
            )
        if not 0.008 <= contact_height_m <= PRODUCT_BOX_HEIGHT_M - 0.008:
            raise RuntimeError(
                "R3 pads do not contact the vertical box wall: "
                f"height={contact_height_m * 1000.0:.2f} mm"
            )
        if abs(inner_surface_gap_m - PRODUCT_GRASP_CLOSED_GAP_M) > 0.004:
            raise RuntimeError(
                "R3 live pad opening differs from the box-surface grasp gap: "
                f"gap={inner_surface_gap_m * 1000.0:.2f} mm"
            )
        self._product_grasp_surface_evidence = evidence
        return evidence

    def execute(self, action: str) -> dict[str, Any]:
        if action == R3_MODULE_PLACED:
            return self._execute_module()
        if action == R3_PRODUCT_TO_INSPECTION:
            return self._execute_product_transfer()
        raise ValueError(f"unsupported R3 action: {action}")

    def _execute_module(self) -> dict[str, Any]:
        paths = self._paths(R3_MODULE_PLACED)
        prepared_mode = R3_MODULE_PLACED in self._prepared_paths
        self._validate_module_preflight(paths, verify_static=not prepared_mode)

        sim = self.bridge.sim
        robot = -1
        command_script = -1
        module = -1
        runner: Optional[_SmoothRunner] = None
        joints: list[int] = []
        original_velocities: list[float] = []
        attached = False
        succeeded = False
        try:
            self.bridge.set_stepping(True)
            robot = self.bridge.get_object_handle(ROBOT_ID)
            joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
            tip = find_unique_alias(sim, robot, ROBOT_TIPS[ROBOT_ID])
            module = self.bridge.get_object_handle("CONTROL_MODULE_SUPPLY")
            command_script = self._create_command_script(robot)
            start_targets = self._pre_positioned_config.get(
                R3_MODULE_PLACED,
                paths["initial_to_pick_app"][0],
            )
            original_velocities = self._set_joint_limits_and_start(
                joints, start_targets
            )
            runner = _SmoothRunner(
                self.bridge,
                robot,
                joints,
                command_script,
                [module],
                self._ignored_camera_view(),
                {
                    module: set(
                        sim.getObjectsInTree(
                            self.bridge.get_object_handle("PCB_SUPPLY"),
                            sim.object_shape_type,
                            0,
                        )
                    )
                    | set(
                        sim.getObjectsInTree(
                            self.bridge.get_object_handle("BOX_BLANK"),
                            sim.object_shape_type,
                            0,
                        )
                    )
                    | set(
                        sim.getObjectsInTree(
                            self.bridge.get_object_handle("ASSEMBLY_FIXTURE"),
                            sim.object_shape_type,
                            0,
                        )
                    )
                },
                self.collision_check_interval,
                self.workspace_check_interval,
            )
            if prepared_mode:
                runner.step("R3 runtime bridge initialized", force_full=True)
            transfer_speed = math.radians(self.speed_deg_s)
            descent_speed = math.radians(
                min(self.speed_deg_s * 0.75, DESCENT_SPEED_CAP_DEG_S)
            )

            self._open_gripper(runner)
            if R3_MODULE_PLACED not in self._pre_positioned_config:
                if not prepared_mode:
                    runner.hold(0.5, "R3 startup")
                runner.execute_path(
                    "R3 initial_to_module_pick_app",
                    paths["initial_to_pick_app"],
                    transfer_speed,
                )
            runner.hold(self.hold_seconds, "R3 hold above module")
            module_pick_alignment = self._prepared_pick_alignment_paths.get(
                R3_MODULE_PLACED
            )
            if module_pick_alignment is not None:
                runner.execute_path(
                    "R3 align above module at safe height",
                    module_pick_alignment,
                    descent_speed,
                )
            else:
                runner.execute_path(
                    "R3 descend_to_module_pick_tcp",
                    paths["pick_descend"],
                    descent_speed,
                )
            module_grasp = self._prepared_grasp_paths[R3_MODULE_PLACED]
            runner.set_payload(module)
            runner.execute_path(
                "R3 module vertical contact approach",
                module_grasp,
                descent_speed,
            )
            self._close_gripper(runner, 0.045)
            module_pickup_pose = [
                float(value) for value in sim.getObjectPose(module, -1)
            ]
            self.bridge.attach_object("CONTROL_MODULE_SUPPLY", ROBOT_ID)
            attached = True
            runner.set_payload(module)
            runner.step("R3 module attached", force_full=True)
            runner.execute_path(
                "R3 lift module from supply",
                list(reversed(module_grasp)),
                descent_speed,
            )
            if module_pick_alignment is not None:
                runner.execute_path(
                    "R3 leave module pick XY alignment",
                    list(reversed(module_pick_alignment)),
                    descent_speed,
                )

            self._wait_before_assembly(R3_MODULE_PLACED)
            with self.assembly_lock:
                if self._coordinated_mode:
                    self._validate_module_assembly_ready()
                runner.execute_path(
                    "R3 module_lift_and_transfer",
                    paths["lift_and_transfer"],
                    transfer_speed,
                )
                runner.hold(self.hold_seconds, "R3 hold above module place")
                if prepared_mode:
                    module_approach_pose = [
                        MODULE_ASSEMBLY_POSITION[0],
                        MODULE_ASSEMBLY_POSITION[1],
                        MODULE_ASSEMBLY_POSITION[2]
                        + VERTICAL_APPROACH_CLEARANCE_M,
                        *module_pickup_pose[3:],
                    ]
                    module_place_alignment = self._rigid_payload_pose_path(
                        robot,
                        tip,
                        joints,
                        module,
                        module_approach_pose,
                        41,
                        "R3_module_safe_height_place_alignment",
                    )
                    runner.execute_path(
                        "R3 align module above box at safe height",
                        module_place_alignment,
                        transfer_speed,
                    )
                    module_release = self._rigid_payload_pose_path(
                        robot,
                        tip,
                        joints,
                        module,
                        [*MODULE_ASSEMBLY_POSITION, *module_pickup_pose[3:]],
                        61,
                        "R3_module_vertical_place",
                    )
                else:
                    runner.execute_path(
                        "R3 descend_to_module_place_tcp",
                        paths["place_descend"],
                        descent_speed,
                    )
                    module_place_alignment = []
                    module_release = self._release_alignment_path(
                        robot,
                        tip,
                        joints,
                        module,
                        MODULE_ASSEMBLY_POSITION,
                        "R3_module_release_alignment",
                    )
                runner.execute_path(
                    "R3 lower module vertically into box",
                    module_release,
                    descent_speed,
                )
                # Keep the jaws narrow until they have left the assembled
                # box.  A full 170 mm opening at insertion depth would make
                # the pads expand into the box walls (and the terminal).
                if not self.bridge.set_gripper_gap(ROBOT_ID, 0.053):
                    raise RuntimeError(
                        self.bridge.last_error
                        or "cannot release module from R3 gripper"
                    )
                runner.hold(0.4, "R3 release module inside box")
                self.bridge.detach_object(module)
                attached = False
                runner.set_payload(module)
                runner.step("R3 module detached", force_full=True)
                runner.execute_path(
                    "R3 clear released module",
                    list(reversed(module_release)),
                    descent_speed,
                )
                self._open_gripper(runner)
                if module_place_alignment:
                    runner.execute_path(
                        "R3 leave module place alignment",
                        list(reversed(module_place_alignment)),
                        transfer_speed,
                    )
                runner.set_payload(None)
                runner.execute_path(
                    "R3 retreat_to_terminal_clearance",
                    paths["retreat_to_clear"],
                    transfer_speed,
                )
            runner.hold(0.35, "R3 final clearance hold")

            final_joints = runner.joint_positions()
            expected_final = paths["retreat_to_clear"][-1]
            if not _near(
                final_joints,
                expected_final,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError("R3 did not reach the terminal-clearance waypoint")
            result = {
                "action": R3_MODULE_PLACED,
                "visual_grasp_only": True,
                "contact_aligned_grasp": True,
                "attachment_snap_m": 0.0,
                "tool": "native_gripper",
                "tip_link": "R3_gripper_tip",
                "final_standby": "R3_TEMP_CLEAR_FOR_R1_TERMINAL_BEFORE_PRODUCT_PICK_APP",
                "final_joint_positions_deg": [
                    round(math.degrees(value), 6) for value in final_joints
                ],
                "module_position": [
                    round(float(value), 6)
                    for value in sim.getObjectPosition(module, -1)
                ],
            }
            succeeded = True
            return result
        except Exception:
            if attached and module != -1:
                try:
                    self.bridge.detach_object(module)
                except Exception:
                    pass
            raise
        finally:
            if runner is not None:
                runner.close()
            self._cleanup_runtime(
                succeeded, command_script, joints, original_velocities
            )

    def _execute_product_transfer(self) -> dict[str, Any]:
        paths = self._paths(R3_PRODUCT_TO_INSPECTION)
        prepared_mode = R3_PRODUCT_TO_INSPECTION in self._prepared_paths
        self._validate_product_preflight(paths, verify_static=not prepared_mode)
        self._product_grasp_surface_evidence = {}

        sim = self.bridge.sim
        robot = -1
        command_script = -1
        product = -1
        runner: Optional[_SmoothRunner] = None
        joints: list[int] = []
        original_velocities: list[float] = []
        attached = False
        succeeded = False
        visibility: dict[str, dict[int, int]] = {}
        release_position_before_correction: list[float] = []
        release_orientation_before_correction: list[float] = []
        release_position_delta_m: list[float] = []
        release_yaw_delta_deg = 0.0
        try:
            self.bridge.set_stepping(True)
            robot = self.bridge.get_object_handle(ROBOT_ID)
            joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
            tip = find_unique_alias(sim, robot, ROBOT_TIPS[ROBOT_ID])
            roots = {
                "box": self.bridge.get_object_handle("BOX_BLANK"),
                "pcb": self.bridge.get_object_handle("PCB_SUPPLY"),
                "module": self.bridge.get_object_handle("CONTROL_MODULE_SUPPLY"),
                "terminal": self.bridge.get_object_handle("TERMINAL_BLOCK_SUPPLY"),
                "assembly": self.bridge.get_object_handle("ASSEMBLY_PRODUCT"),
                "inspection": self.bridge.get_object_handle("INSPECTION_PRODUCT"),
            }
            visibility = {
                name: _tree_visibility(sim, handle)
                for name, handle in roots.items()
            }
            assembly = roots["assembly"]
            inspection = roots["inspection"]
            _set_world_pose(
                sim,
                assembly,
                BOX_ASSEMBLY_POSITION,
                INSPECTION_PRODUCT_ORIENTATION,
            )
            _set_world_pose(
                sim,
                inspection,
                BOX_ASSEMBLY_POSITION,
                INSPECTION_PRODUCT_ORIENTATION,
            )
            _set_visibility(sim, visibility["inspection"], True)
            for name in ("box", "pcb", "module", "terminal", "assembly"):
                _set_visibility(sim, visibility[name], False)
            product = inspection

            command_script = self._create_command_script(robot)
            start_targets = self._pre_positioned_config.get(
                R3_PRODUCT_TO_INSPECTION,
                paths["clear_to_pick_app"][0],
            )
            original_velocities = self._set_joint_limits_and_start(
                joints, start_targets
            )
            ignored_environment = self._ignored_camera_view()
            ignored_environment.update(
                sim.getObjectsInTree(product, sim.object_shape_type, 0)
            )
            inspection_platform = sim.getObject(AREAS["INSPECTION_PLATFORM"])
            inspection_screw_area = sim.getObject(AREAS["INSPECTION_SCREW"])
            runner = _SmoothRunner(
                self.bridge,
                robot,
                joints,
                command_script,
                [product],
                ignored_environment,
                {
                    product: {
                        inspection_platform,
                        inspection_screw_area,
                    }
                },
                self.collision_check_interval,
                self.workspace_check_interval,
            )
            if prepared_mode:
                runner.step("R3 runtime bridge initialized", force_full=True)
            transfer_speed = math.radians(self.speed_deg_s)
            descent_speed = math.radians(
                min(self.speed_deg_s * 0.75, DESCENT_SPEED_CAP_DEG_S)
            )

            if not self.bridge.set_gripper_gap(
                ROBOT_ID, PRODUCT_GRASP_OPEN_GAP_M
            ):
                raise RuntimeError(
                    self.bridge.last_error
                    or "cannot open R3 gripper around the box"
                )
            if prepared_mode:
                product_pick_alignment = self._prepared_pick_alignment_paths[
                    R3_PRODUCT_TO_INSPECTION
                ]
                if R3_PRODUCT_TO_INSPECTION in self._pre_positioned_config:
                    product_approach = product_pick_alignment
                else:
                    product_approach = join_paths(
                        paths["clear_to_pick_app"], product_pick_alignment
                    )
                runner.execute_path(
                    "R3 move once to product pick approach",
                    product_approach,
                    transfer_speed,
                )
                product_grasp = self._prepared_grasp_paths[
                    R3_PRODUCT_TO_INSPECTION
                ]
            else:
                product_pick_alignment = []
                if R3_PRODUCT_TO_INSPECTION not in self._pre_positioned_config:
                    runner.execute_path(
                        "R3 clear_to_product_pick_app",
                        paths["clear_to_pick_app"],
                        transfer_speed,
                    )
                runner.execute_path(
                    "R3 descend_to_product_pick_tcp",
                    paths["pick_descend"],
                    descent_speed,
                )
                product_grasp = self._prepared_grasp_paths[
                    R3_PRODUCT_TO_INSPECTION
                ]
            runner.set_payload(product)
            runner.execute_path(
                "R3 product vertical contact approach",
                product_grasp,
                descent_speed,
            )
            self._close_gripper(runner, PRODUCT_GRASP_COMMAND_GAP_M)
            self._validate_product_surface_grasp(robot, product)
            product_pickup_pose = [
                float(value) for value in sim.getObjectPose(product, -1)
            ]
            self.bridge.attach_object("INSPECTION_PRODUCT", ROBOT_ID)
            attached = True
            runner.set_payload(product)
            runner.step("R3 product attached", force_full=True)
            runner.execute_path(
                "R3 lift product from assembly fixture",
                list(reversed(product_grasp)),
                descent_speed,
            )

            with self.assembly_lock:
                with self.inspection_lock:
                    if not prepared_mode:
                        runner.execute_path(
                            "R3 product_lift_and_transfer",
                            paths["lift_and_transfer"],
                            transfer_speed,
                        )
                    if prepared_mode:
                        product_transfer_path = [
                            list(config)
                            for config in self._prepared_product_transfer_path
                        ]
                        if not product_transfer_path:
                            raise RuntimeError(
                                "R3 product transfer path was not prepared"
                            )
                        current = self.bridge.get_robot_joint_positions(ROBOT_ID)
                        if _max_joint_gap(
                            current, product_transfer_path[0]
                        ) > math.radians(0.5):
                            raise RuntimeError(
                                "R3 live top-grip config differs from the "
                                "prepared clamped transfer path"
                            )
                        product_transfer_path[0] = current
                        runner.execute_path(
                            "R3 rigidly align product above inspection",
                            product_transfer_path,
                            transfer_speed,
                        )
                        product_place_vertical = [
                            list(config)
                            for config in self._prepared_product_place_path
                        ]
                        if not product_place_vertical:
                            raise RuntimeError(
                                "R3 product vertical place path was not prepared"
                            )
                    else:
                        product_place_vertical = paths["place_descend"]
                    runner.execute_path(
                        "R3 lower product vertically to inspection",
                        product_place_vertical,
                        descent_speed,
                    )
                    release_position_before_correction = [
                        float(value)
                        for value in sim.getObjectPosition(product, -1)
                    ]
                    release_orientation_before_correction = [
                        float(value)
                        for value in sim.getObjectOrientation(product, -1)
                    ]
                    release_position_delta_m = [
                        actual - expected
                        for actual, expected in zip(
                            release_position_before_correction,
                            INSPECTION_PRODUCT_POSITION,
                        )
                    ]
                    if max(
                        abs(value) for value in release_position_delta_m
                    ) > INSPECTION_HANDOFF_TOLERANCE_M:
                        raise RuntimeError(
                            "R3 product is not centered for the R4/R5 "
                            "inspection handoff: delta_mm="
                            f"{[round(value * 1000.0, 3) for value in release_position_delta_m]}"
                        )
                    release_yaw_delta_deg = math.degrees(
                        release_orientation_before_correction[2]
                    )
                    if not self.bridge.set_gripper_gap(ROBOT_ID, 0.158):
                        raise RuntimeError(
                            self.bridge.last_error
                            or "cannot release product from R3 gripper"
                        )
                    runner.hold(0.25, "R3 release product at inspection")
                    self.bridge.detach_object(product)
                    attached = False
                    _set_visibility(sim, visibility["inspection"], True)
                    _set_visibility(sim, visibility["assembly"], False)
                    runner.step("R3 product released at inspection", force_full=True)
                    runner.execute_path(
                        "R3 clear released product vertically",
                        list(reversed(product_place_vertical)),
                        descent_speed,
                    )
                    if not self.bridge.set_gripper(ROBOT_ID, True):
                        raise RuntimeError(
                            self.bridge.last_error
                            or "cannot fully open R3 gripper"
                        )
                    runner.set_payload(None)
                    if prepared_mode:
                        if not self._prepared_product_retreat_path:
                            raise RuntimeError(
                                "R3 inspection-clear retreat was not prepared"
                            )
                        runner.execute_path(
                            "R3 park once outside the inspection workspace",
                            self._prepared_product_retreat_path,
                            transfer_speed,
                        )
                    else:
                        runner.execute_path(
                            "R3 retreat_and_return_home",
                            paths["retreat_and_return_home"],
                            transfer_speed,
                        )
            final_joints = runner.joint_positions()
            expected_final = (
                self._prepared_product_retreat_path[-1]
                if prepared_mode
                else paths["retreat_and_return_home"][-1]
            )
            if not _near(
                final_joints,
                expected_final,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError(
                    "R3 did not reach the validated product-transfer clearance"
                )
            result = {
                "action": R3_PRODUCT_TO_INSPECTION,
                "visual_grasp_only": True,
                "contact_aligned_grasp": True,
                "attachment_snap_m": 0.0,
                "tool": "native_gripper",
                "tip_link": "R3_gripper_tip",
                "payload_level_through_transfer": True,
                "flange_down_at_pick_and_place": True,
                "inspection_place_source": "R3_fixed_base",
                "rigid_flange_down_fixed_transfer": True,
                "r3_fixed_base_xy_m": list(R3_FIXED_BASE_XY_M),
                "tool_approach_axis": "local X vertical",
                "jaw_closing_axis": "box local Y / width surfaces",
                "product_grasp_surface": dict(
                    self._product_grasp_surface_evidence
                ),
                "max_clamped_transfer_tilt_deg": round(
                    self._prepared_product_max_orientation_error_deg,
                    6,
                ),
                "rigid_payload_transform": True,
                "final_standby": R3_PRODUCT_TRANSFER_CLEARANCE,
                "return_home_source": "R3_fixed_base_clearance",
                "final_joint_positions_deg": [
                    round(math.degrees(value), 6) for value in final_joints
                ],
                "inspection_product_position": [
                    round(float(value), 6)
                    for value in sim.getObjectPosition(roots["inspection"], -1)
                ],
                "inspection_product_approach_position": [
                    round(float(value), 6)
                    for value in INSPECTION_PRODUCT_APPROACH_POSITION
                ],
                "release_position_before_template_swap": [
                    round(value, 6) for value in release_position_before_correction
                ],
                "release_position_before_final_settle": [
                    round(value, 6) for value in release_position_before_correction
                ],
                "release_position_delta_m": [
                    round(value, 6) for value in release_position_delta_m
                ],
                "release_yaw_delta_deg": round(release_yaw_delta_deg, 6),
                "template_stage_swap": False,
                "template_stage_swap_same_step": False,
                "carried_visible_product": "INSPECTION_PRODUCT",
                "release_final_settle_steps": PRODUCT_RELEASE_SETTLE_STEPS,
            }
            succeeded = True
            return result
        except Exception:
            if attached and product != -1:
                try:
                    self.bridge.detach_object(product)
                except Exception:
                    pass
            raise
        finally:
            if not succeeded:
                for layers in visibility.values():
                    try:
                        _restore_visibility(sim, layers)
                    except Exception:
                        pass
            if runner is not None:
                runner.close()
            self._cleanup_runtime(
                succeeded, command_script, joints, original_velocities
            )

    def _cleanup_runtime(
        self,
        succeeded: bool,
        command_script: int,
        joints: list[int],
        original_velocities: list[float],
    ) -> None:
        sim = self.bridge.sim
        if succeeded:
            if command_script != -1:
                sim.removeObjects([command_script])
            self._restore_joint_limits(joints, original_velocities)
            if not self._continuous_stepping:
                self.bridge.set_stepping(False)
            return

        if sim.getSimulationState() != sim.simulation_stopped:
            self.bridge.stop_simulation()
        else:
            self.bridge.set_stepping(False)
        if command_script != -1:
            try:
                sim.removeObjects([command_script])
            except Exception:
                pass
        for joint, original in zip(joints, original_velocities):
            try:
                sim.setObjectFloatParam(joint, sim.jointfloatparam_maxvel, original)
            except Exception:
                pass


__all__ = [
    "PLAN_PATH",
    "R3_ACTIONS",
    "R3_MODULE_PLACED",
    "R3_PRODUCT_TRANSFER_CLEARANCE",
    "R3_PRODUCT_TO_INSPECTION",
    "R3MotionController",
    "load_r3_plan",
]
