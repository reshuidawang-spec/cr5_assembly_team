"""Validated R2 PCB motion for the five-CR5A CoppeliaSim cell.

R2 installs the real ``PCB_Supply`` into the real box left by
``R1_BOX_PLACED``.  The checked-in plan contains R2 joint-space samples
generated from RViz/MoveIt targets in the robot-relative frame, then
regenerated with a level suction orientation that keeps the suction plate axes
parallel to the PCB's initial long/short axes.  Grasping is a visual suction
attach operation; this module does not claim physical suction validation.
"""

from __future__ import annotations

import bisect
import hashlib
import itertools
import json
import math
import threading
from pathlib import Path
from typing import Any, Optional

from robot_control.r1_motion import PLAN_PATH as R1_PLAN_PATH
from robot_control.r1_motion import load_r1_plan
from robot_control.runtime_cartesian import (
    build_tip_translation_path,
    find_unique_alias,
)
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import PARTS, ROBOT_BASES, ROBOT_TIPS, WORKSPACES


R2_PCB_PLACED = "R2_PCB_PLACED"
R2_ACTIONS = frozenset({R2_PCB_PLACED})

PLAN_VERSION = 1
PLAN_PATH = Path(__file__).with_name("plans") / "r2_pcb_cycle_plan.json"
SCENE_NAME = "compact_cell1ttt.ttt"
ROBOT_ID = "R2"
TARGET_NAMES = (
    "R2_PCB_PICK_APP",
    "R2_PCB_PICK_TCP",
    "R2_PCB_PLACE_APP",
    "R2_PCB_PLACE_TCP",
)
PROTECTED_TARGETS = {
    "R2_PCB_PICK_APP": {
        "position": [-1.22, -0.42, 0.416],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R2_PCB_PICK_TCP": {
        "position": [-1.22, -0.42, 0.236],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R2_PCB_PLACE_APP": {
        "position": [-1.08, 0.12, 0.4704],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R2_PCB_PLACE_TCP": {
        "position": [-1.08, 0.12, 0.2904],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
}

RUNTIME_ORIENTATION_DEG = (0.0, 0.0, 180.0)
RUNTIME_ORIENTATION = tuple(
    math.radians(value) for value in RUNTIME_ORIENTATION_DEG
)
NATIVE_TCP_OFFSET_M = 0.124
RUNTIME_ATTACH_TCP_OFFSET_M = 0.0
PCB_VISUAL_OFFSET_Z = 0.052

BOX_ASSEMBLY_POSITION = (-1.078563, 0.120898, 0.21946)
PCB_SUPPLY_POSITION = (-1.22, -0.42, 0.1584)
PCB_GRASP_POSITION = (-1.22, -0.42, 0.1608)
# Canonical release pose consumed by R3's module-placement preflight.  The
# archived executor referenced this name during detach but omitted the
# constant, so R2 failed only after completing its visible trajectory.
PCB_ASSEMBLY_POSITION = (-1.080058, 0.120636, 0.264725)
PCB_RELEASE_ORIENTATION = (0.0, 0.0, 0.0)
POSITION_TOLERANCE_M = 0.002
JOINT_TOLERANCE_DEG = 0.30
TARGET_TOLERANCE = 1e-6
WORKSPACE_TOLERANCE_M = 0.003

TRANSFER_SPEED_DEG_S = 50.0
INITIAL_APPROACH_SPEED_MULTIPLIER = 2.4
DESCENT_SPEED_CAP_DEG_S = 36.0
HOLD_SECONDS = 0.8

REQUIRED_PATHS = (
    "initial_to_pick_app",
    "pick_descend",
    "lift_and_transfer",
    "place_descend",
    "return_home",
)
COORDINATED_SAFE_WAIT_PATHS = (
    "pick_tcp_to_safe_wait",
    "safe_wait_to_place_app",
)
MAX_PATH_BOUNDARY_JUMP_RAD = 1e-4
MAX_UNWRAPPED_JOINT_RAD = 2.0 * math.pi + 1e-6

RUNTIME_BRIDGE_ALIAS = "R2_Runtime_Command_Bridge"
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


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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


def load_r2_plan(path: Path = PLAN_PATH) -> dict[str, Any]:
    """Load and structurally validate the R2 replay plan."""
    try:
        plan = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load R2 plan {path}: {exc}") from exc

    if plan.get("plan_version") != PLAN_VERSION:
        raise RuntimeError(f"unsupported R2 plan version in {path}")
    if plan.get("protected_targets_modified") is not False:
        raise RuntimeError("R2 plan does not preserve the protected Git targets")
    if plan.get("suction_orientation_world_euler_deg") != list(
        RUNTIME_ORIENTATION_DEG
    ):
        raise RuntimeError("R2 suction runtime orientation is not validated")

    alignment = plan.get("suction_alignment", {})
    if alignment.get("plate_horizontal") is not True:
        raise RuntimeError("R2 suction plate is not marked horizontal")
    if alignment.get("long_short_edges_swapped") is not False:
        raise RuntimeError("R2 suction plate swaps PCB long/short axes")

    protected = plan.get("protected_targets")
    if not isinstance(protected, dict) or set(protected) != set(TARGET_NAMES):
        raise RuntimeError("R2 plan target snapshot is incomplete")
    paths = plan.get("paths")
    if not isinstance(paths, dict):
        raise RuntimeError("R2 plan has no paths")
    for name in REQUIRED_PATHS:
        if not _finite_joint_path(paths.get(name)):
            raise RuntimeError(f"R2 plan path is invalid: {name}")
        if _path_has_invalid_joint_branch(paths[name]):
            raise RuntimeError(f"R2 plan uses an invalid joint branch: {name}")
    for name in COORDINATED_SAFE_WAIT_PATHS:
        if not _finite_joint_path(paths.get(name)):
            raise RuntimeError(f"R2 coordinated safe-wait path is invalid: {name}")
        if _path_has_invalid_joint_branch(paths[name]):
            raise RuntimeError(
                f"R2 coordinated safe-wait path uses an invalid branch: {name}"
            )
    for first_name, second_name in zip(REQUIRED_PATHS, REQUIRED_PATHS[1:]):
        if (
            _max_joint_gap(paths[first_name][-1], paths[second_name][0])
            > MAX_PATH_BOUNDARY_JUMP_RAD
        ):
            raise RuntimeError(
                "R2 plan path boundary jumps: "
                f"{first_name} -> {second_name}"
            )
    for first_name, second_name in (
        ("pick_descend", "pick_tcp_to_safe_wait"),
        ("pick_tcp_to_safe_wait", "safe_wait_to_place_app"),
    ):
        if (
            _max_joint_gap(paths[first_name][-1], paths[second_name][0])
            > MAX_PATH_BOUNDARY_JUMP_RAD
        ):
            raise RuntimeError(
                "R2 coordinated safe-wait path boundary jumps: "
                f"{first_name} -> {second_name}"
            )

    endpoints = plan.get("endpoints_rad", {})
    for endpoint in ("pick_app", "pick_tcp", "place_app", "place_tcp"):
        value = endpoints.get(endpoint)
        if (
            not isinstance(value, list)
            or len(value) != 6
            or not all(math.isfinite(float(joint)) for joint in value)
        ):
            raise RuntimeError(f"R2 plan endpoint is invalid: {endpoint}")

    if (
        _max_joint_gap(paths["initial_to_pick_app"][-1], endpoints["pick_app"])
        > MAX_PATH_BOUNDARY_JUMP_RAD
    ):
        raise RuntimeError("R2 pick APP endpoint differs from initial path")
    if (
        _max_joint_gap(paths["pick_descend"][-1], endpoints["pick_tcp"])
        > MAX_PATH_BOUNDARY_JUMP_RAD
    ):
        raise RuntimeError("R2 pick TCP endpoint differs from descend path")
    if (
        _max_joint_gap(paths["lift_and_transfer"][-1], endpoints["place_app"])
        > MAX_PATH_BOUNDARY_JUMP_RAD
    ):
        raise RuntimeError("R2 place APP endpoint differs from transfer path")
    if (
        _max_joint_gap(paths["safe_wait_to_place_app"][-1], endpoints["place_app"])
        > MAX_PATH_BOUNDARY_JUMP_RAD
    ):
        raise RuntimeError("R2 coordinated safe-wait path does not end at place APP")
    if (
        _max_joint_gap(paths["place_descend"][-1], endpoints["place_tcp"])
        > MAX_PATH_BOUNDARY_JUMP_RAD
    ):
        raise RuntimeError("R2 place TCP endpoint differs from descend path")
    if (
        _max_joint_gap(paths["return_home"][-1], endpoints["pick_app"])
        > MAX_PATH_BOUNDARY_JUMP_RAD
    ):
        raise RuntimeError("R2 final standby is not pick APP")

    workspace = plan.get("workspace", {})
    expected_workspace = WORKSPACES["R2"]
    if tuple(workspace.get("lower", ())) != tuple(expected_workspace["lower"]):
        raise RuntimeError("R2 plan lower workspace wall differs from the contract")
    if tuple(workspace.get("upper", ())) != tuple(expected_workspace["upper"]):
        raise RuntimeError("R2 plan upper workspace wall differs from the contract")
    shared = workspace.get("assembly_shared", {})
    expected_shared = WORKSPACES["ASSEMBLY_SHARED"]
    if tuple(shared.get("lower", ())) != tuple(expected_shared["lower"]):
        raise RuntimeError("R2 plan shared-zone lower bound differs from the contract")
    if tuple(shared.get("upper", ())) != tuple(expected_shared["upper"]):
        raise RuntimeError("R2 plan shared-zone upper bound differs from the contract")

    validation = plan.get("validation", {})
    fingerprint = validation.get("scene_fingerprint", {})
    if not isinstance(fingerprint.get("sha256"), str) or not isinstance(
        fingerprint.get("size"), int
    ):
        raise RuntimeError("R2 plan has no validated scene fingerprint")
    if validation.get("native_tcp_offset_m") != NATIVE_TCP_OFFSET_M:
        raise RuntimeError("R2 plan does not use the native suction TCP")
    if (
        validation.get("return_home_semantics")
        != "place_tcp_lift_to_place_app_then_transfer_to_pick_app_standby"
    ):
        raise RuntimeError("R2 return-home semantics are not the standby plan")
    return plan


def _near(first: list[float], second: list[float], tolerance: float) -> bool:
    return len(first) == len(second) and max(
        abs(a - b) for a, b in zip(first, second)
    ) <= tolerance


def _set_world_pose(
    sim: Any,
    handle: int,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float],
) -> None:
    sim.setObjectPosition(handle, -1, list(position))
    sim.setObjectOrientation(handle, -1, list(orientation))


def _find_alias(sim: Any, root: int, alias: str) -> int:
    matches = [
        handle
        for handle in sim.getObjectsInTree(root, sim.handle_all, 0)
        if sim.getObjectAlias(handle) == alias
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one {alias} below {sim.getObjectAlias(root, 1)}, "
            f"found {len(matches)}"
        )
    return matches[0]


def _wrap_near(reference: float, value: float) -> float:
    while value - reference > math.pi:
        value -= 2.0 * math.pi
    while value - reference < -math.pi:
        value += 2.0 * math.pi
    return value


def _unwrap_path(configs: list[list[float]]) -> list[list[float]]:
    if not configs:
        return []
    result = [list(configs[0])]
    for config in configs[1:]:
        result.append(
            [
                _wrap_near(previous, value)
                for previous, value in zip(result[-1], config)
            ]
        )
    return result


def _interpolate_joint_line(
    first: list[float], second: list[float], count: int
) -> list[list[float]]:
    return [
        [
            start + (finish - start) * index / (count - 1)
            for start, finish in zip(first, second)
        ]
        for index in range(count)
    ]


def _join_paths(*paths: list[list[float]]) -> list[list[float]]:
    result: list[list[float]] = []
    for path in paths:
        if not path:
            raise RuntimeError("cannot join an empty R2 path")
        current = _unwrap_path(path)
        if result:
            current = [
                [
                    _wrap_near(previous, value)
                    for previous, value in zip(result[-1], config)
                ]
                for config in current
            ]
            discontinuity = max(
                abs(before - after)
                for before, after in zip(result[-1], current[0])
            )
            if discontinuity > math.radians(0.5):
                raise RuntimeError(
                    "R2 joined path discontinuity "
                    f"{math.degrees(discontinuity):.3f} deg"
                )
            current = current[1:]
        result.extend(current)
    return result


def _cumulative_max_joint_distance(
    configs: list[list[float]],
) -> list[float]:
    cumulative = [0.0]
    for first, second in zip(configs, configs[1:]):
        cumulative.append(
            cumulative[-1]
            + max(abs(b - a) for a, b in zip(first, second))
        )
    return cumulative


def _interpolate_path(
    configs: list[list[float]], cumulative: list[float], distance: float
) -> list[float]:
    if distance <= 0.0:
        return list(configs[0])
    if distance >= cumulative[-1]:
        return list(configs[-1])
    upper = bisect.bisect_right(cumulative, distance)
    lower = upper - 1
    span = cumulative[upper] - cumulative[lower]
    fraction = (distance - cumulative[lower]) / span if span > 0.0 else 0.0
    return [
        first + (second - first) * fraction
        for first, second in zip(configs[lower], configs[upper])
    ]


def _minimum_jerk(fraction: float) -> float:
    fraction = max(0.0, min(1.0, fraction))
    return fraction**3 * (10.0 - 15.0 * fraction + 6.0 * fraction**2)


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


def _solve_target(
    sim: Any,
    sim_ik: Any,
    base: int,
    tip: int,
    joints: list[int],
    target: int,
    seed_joint_values: list[float],
) -> dict[str, Any]:
    environment = sim_ik.createEnvironment()
    group = sim_ik.createGroup(environment)
    try:
        element, scene_to_ik, _ = sim_ik.addElementFromScene(
            environment,
            group,
            base,
            tip,
            target,
            sim_ik.constraint_pose,
        )
        for joint, value in zip(joints, seed_joint_values):
            sim_ik.setJointPosition(environment, scene_to_ik[joint], value)
        sim_ik.setGroupCalculation(
            environment,
            group,
            sim_ik.method_damped_least_squares,
            0.1,
            200,
        )
        sim_ik.setElementPrecision(
            environment, group, element, [0.001, math.radians(1.0)]
        )
        result, flags, precision = sim_ik.handleGroup(environment, group)
        joint_values = [
            float(
                sim_ik.getJointPosition(environment, scene_to_ik[joint])
            )
            for joint in joints
        ]
        return {
            "success": result == sim_ik.result_success,
            "result": result,
            "flags": flags,
            "linear_precision_m": float(precision[0]),
            "angular_precision_rad": float(precision[1]),
            "joint_positions_rad": joint_values,
        }
    finally:
        sim_ik.eraseEnvironment(environment)


def _generate_cartesian_path(
    sim_ik: Any,
    base: int,
    tip: int,
    joints: list[int],
    target: int,
    start: list[float],
    point_count: int,
) -> list[list[float]]:
    environment = sim_ik.createEnvironment()
    group = sim_ik.createGroup(environment)
    try:
        _, scene_to_ik, _ = sim_ik.addElementFromScene(
            environment,
            group,
            base,
            tip,
            target,
            sim_ik.constraint_pose,
        )
        ik_joints = [scene_to_ik[joint] for joint in joints]
        for joint, value in zip(ik_joints, start):
            sim_ik.setJointPosition(environment, joint, value)
        sim_ik.setGroupCalculation(
            environment,
            group,
            sim_ik.method_damped_least_squares,
            0.1,
            200,
        )
        flat = sim_ik.generatePath(
            environment,
            group,
            ik_joints,
            scene_to_ik[tip],
            point_count,
        )
        if len(flat) != point_count * len(joints):
            return []
        return [
            [float(value) for value in flat[index : index + len(joints)]]
            for index in range(0, len(flat), len(joints))
        ]
    finally:
        sim_ik.eraseEnvironment(environment)


class R2SafetyGuard:
    """R2 environment, self, payload, and invisible-wall checks."""

    def __init__(
        self,
        sim: Any,
        robot: int,
        payload: Optional[int] = None,
    ):
        self.sim = sim
        robot_shapes = {
            handle
            for handle in sim.getObjectsInTree(
                robot, sim.object_shape_type, 0
            )
            # The TCP marker is a decorative sphere centred on the suction
            # contact plane.  Treating its radius as solid geometry makes it
            # protrude through a thin PCB and falsely collide with the tray.
            if not sim.getObjectAlias(handle).endswith("_tcp_marker")
        }
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

        robot_base = sim.getObject(ROBOT_BASES[ROBOT_ID])
        for handle in sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        ):
            if handle in self.mover_shapes or handle == robot_base:
                continue
            if sim.getObjectAlias(handle).endswith("_tcp_marker"):
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
            raise RuntimeError(f"R2 collision links missing: {missing}")
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
            positions = [
                [
                    round(float(value), 6)
                    for value in self.sim.getObjectPosition(handle, -1)
                ]
                for handle in pair
            ]
            orientations = [
                [
                    round(math.degrees(float(value)), 3)
                    for value in self.sim.getObjectOrientation(handle, -1)
                ]
                for handle in pair
            ]
            raise RuntimeError(
                f"collision during {label}: {paths}, positions={positions}, "
                f"orientations_deg={orientations}"
            )
        if check_internal:
            for index, first in enumerate(self.links):
                for second in self.links[index + 2 :]:
                    state, pair = self.sim.checkCollision(first, second)
                    if state:
                        paths = [
                            self.sim.getObjectAlias(handle, 1) for handle in pair
                        ]
                        raise RuntimeError(
                            f"R2 self collision during {label}: {paths}"
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
                        f"PCB-to-R2 collision during {label}: {paths}"
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
                    f"R2 workspace violation during {label}: axis={axis}, "
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
        pcb: int,
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
        self.guards = {
            None: R2SafetyGuard(self.sim, robot),
            pcb: R2SafetyGuard(self.sim, robot, pcb),
        }
        self.guard = self.guards[None]
        self.step_index = 0

    def close(self) -> None:
        for guard in self.guards.values():
            guard.close()

    def set_payload(self, payload: Optional[int]) -> None:
        self.guard = self.guards[payload]

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
                self.bridge.last_error or "R2 simulation step failed"
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
        cumulative = _cumulative_max_joint_distance(configs)
        total = cumulative[-1]
        if total <= 1e-12:
            raise RuntimeError(f"{label} has no joint motion")
        duration = max(0.55, 1.875 * total / peak_speed_rad_s)
        step_count = max(2, math.ceil(duration / self.dt))
        for index in range(1, step_count + 1):
            progress = _minimum_jerk(index / step_count)
            target = _interpolate_path(
                configs, cumulative, total * progress
            )
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


class R2MotionController:
    """Execute the validated visual R2 PCB installation."""

    def __init__(
        self,
        bridge: SimBridge,
        r1_plan_path: Path = R1_PLAN_PATH,
        r2_plan_path: Path = PLAN_PATH,
        assembly_lock: Optional[threading.Lock] = None,
        speed_deg_s: float = TRANSFER_SPEED_DEG_S,
        hold_seconds: float = HOLD_SECONDS,
        collision_check_interval: int = 5,
        workspace_check_interval: int = 20,
    ):
        if speed_deg_s <= 0.0:
            raise ValueError("speed_deg_s must be positive")
        self.bridge = bridge
        self.r1_plan_path = Path(r1_plan_path)
        self.r2_plan_path = Path(r2_plan_path)
        self.assembly_lock = assembly_lock or threading.Lock()
        self.speed_deg_s = float(speed_deg_s)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.collision_check_interval = int(collision_check_interval)
        self.workspace_check_interval = int(workspace_check_interval)
        self._prepared_paths: Optional[dict[str, list[list[float]]]] = None
        self._prepared_grasp_path: Optional[list[list[float]]] = None
        self._pre_positioned_config: Optional[list[float]] = None
        self._continuous_stepping = False
        self._coordinated_mode = False

    def _target_snapshot(self) -> dict[str, dict[str, list[float]]]:
        result = {}
        for name in TARGET_NAMES:
            pose = self.bridge.get_target_pose(name)
            result[name] = {
                "position": [
                    round(float(value), 9) for value in pose["position"]
                ],
                "orientation_euler": [
                    round(float(value), 9)
                    for value in pose["orientation"]
                ],
            }
        return result

    def _validate_static(self) -> dict[str, Any]:
        r1_plan = load_r1_plan(self.r1_plan_path)
        r2_plan = load_r2_plan(self.r2_plan_path)
        scene = Path(self.bridge.scene_path())
        if scene.name != SCENE_NAME:
            raise RuntimeError(f"unexpected CoppeliaSim scene: {scene}")
        scene_sha256 = _sha256(scene)
        scene_size = scene.stat().st_size
        for label, plan in (("R1", r1_plan), ("R2", r2_plan)):
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
        return r1_plan

    def _validate_preflight(
        self, verify_static: bool = True
    ) -> dict[str, Any]:
        sim = self.bridge.sim
        if sim.getSimulationState() == sim.simulation_stopped:
            raise RuntimeError(
                "R2 requires the running scene preserved by R1_BOX_PLACED"
            )

        r1_plan = (
            self._validate_static()
            if verify_static
            else load_r1_plan(self.r1_plan_path)
        )

        if not self._coordinated_mode:
            r1_expected = r1_plan["paths"][
                "box_retreat_and_terminal_approach"
            ][-1]
            if not _near(
                self.bridge.get_robot_joint_positions("R1"),
                r1_expected,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError(
                    "R1 has not exited the assembly zone to "
                    "R1_TERMINAL_PICK_APP"
                )
        expected_r2 = (
            self._pre_positioned_config
            if self._pre_positioned_config is not None
            else [0.0] * 6
        )
        if not _near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            expected_r2,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError(
                "R2 is not at the validated start "
                f"(expected pre-positioned={self._pre_positioned_config is not None})"
            )

        parts = sim.getObject(f"{PARTS['BOX_BLANK'].rsplit('/', 1)[0]}")
        checks = [("PCB_SUPPLY", PCB_SUPPLY_POSITION)]
        if not self._coordinated_mode:
            checks.insert(0, ("BOX_BLANK", BOX_ASSEMBLY_POSITION))
        for object_name, expected_position in checks:
            handle = self.bridge.get_object_handle(object_name)
            if sim.getObjectParent(handle) != parts:
                raise RuntimeError(f"{object_name} is not owned by /Parts")
            actual_position = [
                float(value) for value in sim.getObjectPosition(handle, -1)
            ]
            if not _near(
                actual_position,
                list(expected_position),
                POSITION_TOLERANCE_M,
            ):
                if object_name == "BOX_BLANK":
                    raise RuntimeError(
                        "actual Box_Blank is not at the validated R1 "
                        "assembly position"
                    )
                raise RuntimeError(
                    "PCB_Supply is not at the R2 private supply position"
                )
        return r1_plan

    def _validate_box_ready_for_place(self) -> None:
        sim = self.bridge.sim
        parts = sim.getObject(f"{PARTS['BOX_BLANK'].rsplit('/', 1)[0]}")
        box = self.bridge.get_object_handle("BOX_BLANK")
        if sim.getObjectParent(box) != parts:
            raise RuntimeError("BOX_BLANK is not owned by /Parts")
        actual_position = [
            float(value) for value in sim.getObjectPosition(box, -1)
        ]
        if not _near(
            actual_position,
            list(BOX_ASSEMBLY_POSITION),
            POSITION_TOLERANCE_M,
        ):
            raise RuntimeError(
                "R2 coordinated place cannot start before R1 has placed the box"
            )

    def _load_planned_paths(self) -> dict[str, list[list[float]]]:
        plan = load_r2_plan(self.r2_plan_path)
        return {
            name: [
                [float(joint) for joint in config]
                for config in plan["paths"][name]
            ]
            for name in (*REQUIRED_PATHS, *COORDINATED_SAFE_WAIT_PATHS)
        }

    def prepare(self, action: str = R2_PCB_PLACED) -> dict[str, Any]:
        if action not in R2_ACTIONS:
            raise ValueError(f"unsupported R2 action: {action}")
        sim = self.bridge.sim
        if sim.getSimulationState() != sim.simulation_stopped:
            raise RuntimeError("R2 preparation requires a stopped scene")
        self._validate_static()
        if not _near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            [0.0] * 6,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError("R2 is not zero during preparation")
        self._prepared_paths = self._load_planned_paths()
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        robot = self.bridge.get_object_handle(ROBOT_ID)
        joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
        tip = find_unique_alias(sim, robot, ROBOT_TIPS[ROBOT_ID])
        self._prepared_grasp_path = build_tip_translation_path(
            sim,
            client.require("simIK"),
            robot,
            tip,
            joints,
            self._prepared_paths["pick_descend"][-1],
            PCB_GRASP_POSITION,
            31,
            "R2_PCB_suction_contact",
        )
        return {
            "robot_id": ROBOT_ID,
            "prepared_actions": [R2_PCB_PLACED],
            "path_points": {
                name: len(path) for name, path in self._prepared_paths.items()
            },
            "grasp_contact_points": len(self._prepared_grasp_path),
        }

    def _release_alignment_path(
        self,
        robot: int,
        tip: int,
        joints: list[int],
        pcb: int,
    ) -> list[list[float]]:
        sim = self.bridge.sim
        current_payload = [float(value) for value in sim.getObjectPosition(pcb, -1)]
        current_tip = [float(value) for value in sim.getObjectPosition(tip, -1)]
        target_tip = [
            current_tip[index]
            + PCB_ASSEMBLY_POSITION[index]
            - current_payload[index]
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
            "R2_PCB_release_alignment",
        )

    def set_continuous_stepping(self, enabled: bool) -> None:
        self._continuous_stepping = bool(enabled)

    def set_coordinated_mode(self, enabled: bool) -> None:
        self._coordinated_mode = bool(enabled)

    def set_pre_positioned(self, action: str, config: list[float]) -> None:
        """Record the joint config set by ``_preposition_robots()``.

        When not ``None`` the controller skips the initial-approach segment
        because the robot is already waiting at its pick APP.
        """
        _ = action
        self._pre_positioned_config = list(config)

    def _create_virtual_tcp(self, robot: int) -> int:
        sim = self.bridge.sim
        original_tip = _find_alias(sim, robot, ROBOT_TIPS["R2"])
        virtual_tip = sim.createDummy(0.004)
        sim.setObjectAlias(virtual_tip, "R2_Runtime_Attach_TCP")
        sim.setObjectParent(virtual_tip, original_tip, False)
        sim.setObjectPose(
            virtual_tip,
            original_tip,
            [
                0.0,
                0.0,
                RUNTIME_ATTACH_TCP_OFFSET_M,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        )
        sim.setObjectInt32Param(
            virtual_tip, sim.objintparam_visibility_layer, 0
        )
        return virtual_tip

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

    def _build_paths(
        self,
        sim_ik: Any,
        robot: int,
        virtual_tip: int,
        joints: list[int],
    ) -> dict[str, list[list[float]]]:
        sim = self.bridge.sim
        protected_before = self._target_snapshot()
        base = _find_alias(sim, robot, "base_link_respondable")
        temporary: dict[str, int] = {}
        try:
            for name in TARGET_NAMES:
                source = self.bridge.get_object_handle(name)
                target = sim.createDummy(0.004)
                sim.setObjectPosition(
                    target, -1, sim.getObjectPosition(source, -1)
                )
                sim.setObjectOrientation(
                    target, -1, list(RUNTIME_ORIENTATION)
                )
                sim.setObjectInt32Param(
                    target, sim.objintparam_visibility_layer, 0
                )
                temporary[name] = target

            solved: dict[str, list[float]] = {}
            for key, name, seed_key in (
                ("pick_app", TARGET_NAMES[0], None),
                ("pick_tcp", TARGET_NAMES[1], "pick_app"),
                ("place_app", TARGET_NAMES[2], "pick_app"),
                ("place_tcp", TARGET_NAMES[3], "place_app"),
            ):
                seed = [0.0] * 6 if seed_key is None else solved[seed_key]
                record = _solve_target(
                    sim,
                    sim_ik,
                    base,
                    virtual_tip,
                    joints,
                    temporary[name],
                    seed,
                )
                if not record["success"]:
                    raise RuntimeError(f"R2 IK failed for {name}: {record}")
                solved[key] = record["joint_positions_rad"]

            pick_descend = _unwrap_path(
                _generate_cartesian_path(
                    sim_ik,
                    base,
                    virtual_tip,
                    joints,
                    temporary[TARGET_NAMES[1]],
                    solved["pick_app"],
                    51,
                )
            )
            transfer = _unwrap_path(
                _generate_cartesian_path(
                    sim_ik,
                    base,
                    virtual_tip,
                    joints,
                    temporary[TARGET_NAMES[2]],
                    solved["pick_app"],
                    101,
                )
            )
            place_descend = _unwrap_path(
                _generate_cartesian_path(
                    sim_ik,
                    base,
                    virtual_tip,
                    joints,
                    temporary[TARGET_NAMES[3]],
                    solved["place_app"],
                    51,
                )
            )
            if not pick_descend or not transfer or not place_descend:
                raise RuntimeError("R2 Cartesian path generation failed")

            initial = _interpolate_joint_line(
                [0.0] * 6, solved["pick_app"], 101
            )
            paths = {
                "initial_to_pick_app": initial,
                "pick_descend": pick_descend,
                "lift_and_transfer": _join_paths(
                    list(reversed(pick_descend)), transfer
                ),
                "place_descend": place_descend,
                "return_home": _join_paths(
                    list(reversed(place_descend)),
                    list(reversed(transfer)),
                    list(reversed(initial)),
                ),
            }
            if self._target_snapshot() != protected_before:
                raise RuntimeError(
                    "R2 protected Git targets changed during planning"
                )
            return paths
        finally:
            if temporary:
                sim.removeObjects(list(temporary.values()))

    def execute(self, action: str) -> dict[str, Any]:
        if action not in R2_ACTIONS:
            raise ValueError(f"unsupported R2 action: {action}")
        prepared_mode = self._prepared_paths is not None
        self._validate_preflight(verify_static=not prepared_mode)

        sim = self.bridge.sim
        robot = -1
        virtual_tip = -1
        command_script = -1
        runner: Optional[_SmoothRunner] = None
        attached = False
        succeeded = False
        original_max_velocities: list[float] = []
        joints: list[int] = []
        pcb = -1
        try:
            # Take deterministic control over the running state left by R1.
            self.bridge.set_stepping(True)
            robot = self.bridge.get_object_handle(ROBOT_ID)
            joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
            pcb = self.bridge.get_object_handle("PCB_SUPPLY")
            parts = sim.getObject(f"{PARTS['PCB_SUPPLY'].rsplit('/', 1)[0]}")
            virtual_tip = self._create_virtual_tcp(robot)
            if self._prepared_paths is not None:
                paths = self._prepared_paths
            else:
                paths = self._load_planned_paths()

            original_max_velocities = [
                sim.getObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel
                )
                for joint in joints
            ]
            max_velocity = math.radians(
                max(
                    60.0,
                    self.speed_deg_s
                    * INITIAL_APPROACH_SPEED_MULTIPLIER
                    * 1.35,
                )
            )
            for joint in joints:
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, max_velocity
                )
            start_targets = (
                self._pre_positioned_config
                if self._pre_positioned_config is not None
                else [0.0] * 6
            )
            for joint, target in zip(joints, start_targets):
                sim.setJointTargetPosition(joint, float(target))

            command_script = self._create_command_script(robot)
            if not self.bridge.start_simulation():
                raise RuntimeError(
                    self.bridge.last_error or "cannot take R2 stepping"
                )
            runner = _SmoothRunner(
                self.bridge,
                robot,
                joints,
                command_script,
                pcb,
                self.collision_check_interval,
                self.workspace_check_interval,
            )
            if prepared_mode:
                runner.step("R2 runtime bridge initialized", force_full=True)
            transfer_speed = math.radians(self.speed_deg_s)
            initial_approach_speed = math.radians(
                self.speed_deg_s * INITIAL_APPROACH_SPEED_MULTIPLIER
            )
            descent_speed = math.radians(
                min(self.speed_deg_s * 0.75, DESCENT_SPEED_CAP_DEG_S)
            )

            if self._pre_positioned_config is None:
                if not prepared_mode:
                    runner.hold(0.5, "R2 startup")
                runner.execute_path(
                    "R2 initial_to_pick_app",
                    paths["initial_to_pick_app"],
                    initial_approach_speed,
                )
                runner.hold(self.hold_seconds, "R2 hold above PCB")
            # else: pre-positioned at pick APP — skip the initial approach
            # to reduce simulation-time handoff delay.
            runner.execute_path(
                "R2 descend_to_pick_tcp",
                paths["pick_descend"],
                descent_speed,
            )
            if self._prepared_grasp_path is None:
                raise RuntimeError("R2 grasp contact path was not prepared")
            runner.set_payload(pcb)
            runner.execute_path(
                "R2 final suction contact approach",
                self._prepared_grasp_path,
                descent_speed,
            )
            attached_pose = [float(value) for value in sim.getObjectPose(pcb, -1)]
            sim.setObjectParent(pcb, virtual_tip, True)
            # Explicitly preserve the world pose: attachment must not pull the
            # PCB toward the suction tool.
            sim.setObjectPose(pcb, -1, attached_pose)
            attached = True
            runner.set_payload(pcb)
            runner.step("R2 PCB rigid attachment", force_collision=True)
            runner.execute_path(
                "R2 lift PCB from supply",
                list(reversed(self._prepared_grasp_path)),
                descent_speed,
            )

            if self._coordinated_mode:
                runner.execute_path(
                    "R2 pick_tcp_to_safe_wait",
                    paths["pick_tcp_to_safe_wait"],
                    transfer_speed,
                )
                runner.hold(self.hold_seconds, "R2 hold at PCB safe wait")

            with self.assembly_lock:
                if self._coordinated_mode:
                    self._validate_box_ready_for_place()
                    runner.execute_path(
                        "R2 safe_wait_to_place_app",
                        paths["safe_wait_to_place_app"],
                        transfer_speed,
                    )
                else:
                    runner.execute_path(
                        "R2 lift_and_transfer",
                        paths["lift_and_transfer"],
                        transfer_speed,
                    )
                runner.hold(self.hold_seconds, "R2 hold above box")
                runner.execute_path(
                    "R2 descend_to_place_tcp",
                    paths["place_descend"],
                    descent_speed,
                )
                release_alignment = self._release_alignment_path(
                    robot, virtual_tip, joints, pcb
                )
                runner.execute_path(
                    "R2 lower PCB into box",
                    release_alignment,
                    descent_speed,
                )
                sim.setObjectParent(pcb, parts, True)
                attached = False
                # Keep the grasp-contact guard until the cups have cleared the
                # stationary PCB; detaching does not create instant clearance.
                runner.set_payload(pcb)
                runner.step("R2 PCB detached at canonical pose", force_full=True)
                runner.execute_path(
                    "R2 clear released PCB",
                    list(reversed(release_alignment)),
                    descent_speed,
                )
                runner.set_payload(None)
                runner.execute_path(
                    "R2 retreat_to_pick_app_standby",
                    paths["return_home"],
                    transfer_speed,
                )
                # R2 has no later task in this order.  Leaving it at the PCB
                # pick APP blocks R3's required top-grip reorientation, so
                # continue along the already validated initial approach in
                # reverse and vacate the shared left-side volume completely.
                runner.execute_path(
                    "R2 pick_app_to_initial_clearance",
                    list(reversed(paths["initial_to_pick_app"])),
                    initial_approach_speed,
                )
            runner.hold(0.4, "R2 final initial-clearance hold")

            final_joints = runner.joint_positions()
            expected_final = paths["initial_to_pick_app"][0]
            if not _near(
                final_joints,
                expected_final,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError("R2 did not return to its initial clearance")
            result = {
                "action": action,
                "visual_suction_only": True,
                "contact_aligned_grasp": True,
                "attachment_snap_m": 0.0,
                "runtime_orientation_deg": list(RUNTIME_ORIENTATION_DEG),
                "native_tcp_offset_m": NATIVE_TCP_OFFSET_M,
                "runtime_attach_tcp_offset_m": RUNTIME_ATTACH_TCP_OFFSET_M,
                "pcb_visual_offset_m": 0.0,
                "grasp_contact_position": list(PCB_GRASP_POSITION),
                "final_joint_positions_deg": [
                    round(math.degrees(value), 6)
                    for value in final_joints
                ],
                "final_standby": "R2_INITIAL_CLEARANCE",
                "coordinated_safe_wait_used": self._coordinated_mode,
                "pcb_position": [
                    round(float(value), 6)
                    for value in sim.getObjectPosition(pcb, -1)
                ],
                "box_position": [
                    round(float(value), 6)
                    for value in sim.getObjectPosition(
                        self.bridge.get_object_handle("BOX_BLANK"), -1
                    )
                ],
            }
            succeeded = True
            return result
        except Exception:
            if attached and pcb != -1:
                try:
                    self.bridge.detach_object(pcb)
                except Exception:
                    pass
            raise
        finally:
            if runner is not None:
                runner.close()
            if succeeded:
                if command_script != -1:
                    sim.removeObjects([command_script])
                if virtual_tip != -1:
                    sim.removeObjects([virtual_tip])
                for joint, original in zip(joints, original_max_velocities):
                    sim.setObjectFloatParam(
                        joint, sim.jointfloatparam_maxvel, original
                    )
                if not self._continuous_stepping:
                    self.bridge.set_stepping(False)
            else:
                if sim.getSimulationState() != sim.simulation_stopped:
                    self.bridge.stop_simulation()
                for handle in (command_script, virtual_tip):
                    if handle == -1:
                        continue
                    try:
                        sim.removeObjects([handle])
                    except Exception:
                        pass
                for joint, original in zip(joints, original_max_velocities):
                    try:
                        sim.setObjectFloatParam(
                            joint, sim.jointfloatparam_maxvel, original
                        )
                    except Exception:
                        pass


__all__ = [
    "PLAN_PATH",
    "R2_ACTIONS",
    "R2_PCB_PLACED",
    "R2MotionController",
    "load_r2_plan",
]
