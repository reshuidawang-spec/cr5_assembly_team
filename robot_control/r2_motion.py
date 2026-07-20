"""Validated R2 PCB motion for the five-CR5A CoppeliaSim cell.

R2 installs the real ``PCB_Supply`` into the real box left by
``R1_BOX_PLACED``.  The saved scene has no R2 vacuum-tool geometry, so this
visual milestone creates a runtime-only TCP 100 mm ahead of Link6 and attaches
the PCB to it.  The four Git APP/TCP targets are checked but never modified.
"""

from __future__ import annotations

import bisect
import hashlib
import itertools
import math
import threading
from pathlib import Path
from typing import Any, Optional

from robot_control.r1_motion import PLAN_PATH as R1_PLAN_PATH
from robot_control.r1_motion import load_r1_plan
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import PARTS, ROBOT_BASES, WORKSPACES


R2_PCB_PLACED = "R2_PCB_PLACED"
R2_ACTIONS = frozenset({R2_PCB_PLACED})

SCENE_NAME = "five_cr5a_cell.ttt"
ROBOT_ID = "R2"
TARGET_NAMES = (
    "R2_PCB_PICK_APP",
    "R2_PCB_PICK_TCP",
    "R2_PCB_PLACE_APP",
    "R2_PCB_PLACE_TCP",
)
PROTECTED_TARGETS = {
    "R2_PCB_PICK_APP": {
        "position": [-1.28, -0.28, 0.45],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R2_PCB_PICK_TCP": {
        "position": [-1.28, -0.28, 0.22],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R2_PCB_PLACE_APP": {
        "position": [-1.15, 0.20, 0.50],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R2_PCB_PLACE_TCP": {
        "position": [-1.15, 0.20, 0.29],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
}

RUNTIME_ORIENTATION_DEG = (195.0, 0.0, 90.0)
RUNTIME_ORIENTATION = tuple(
    math.radians(value) for value in RUNTIME_ORIENTATION_DEG
)
VIRTUAL_TCP_OFFSET_M = 0.100
PCB_VISUAL_OFFSET_Z = 0.052

BOX_ASSEMBLY_POSITION = (-1.150002, 0.199900, 0.215915)
PCB_SUPPLY_POSITION = (-1.280000, -0.280000, 0.160000)
POSITION_TOLERANCE_M = 0.002
JOINT_TOLERANCE_DEG = 0.30
TARGET_TOLERANCE = 1e-6
WORKSPACE_TOLERANCE_M = 0.003

TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_CAP_DEG_S = 24.0
HOLD_SECONDS = 0.8

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


def _near(first: list[float], second: list[float], tolerance: float) -> bool:
    return len(first) == len(second) and max(
        abs(a - b) for a, b in zip(first, second)
    ) <= tolerance


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

        robot_base = sim.getObject(ROBOT_BASES[ROBOT_ID])
        for handle in sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        ):
            if handle in self.mover_shapes or handle == robot_base:
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
        self.assembly_lock = assembly_lock or threading.Lock()
        self.speed_deg_s = float(speed_deg_s)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.collision_check_interval = int(collision_check_interval)
        self.workspace_check_interval = int(workspace_check_interval)
        self._prepared_paths: Optional[dict[str, list[list[float]]]] = None
        self._pre_positioned_config: Optional[list[float]] = None
        self._continuous_stepping = False

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
        scene = Path(self.bridge.scene_path())
        if scene.name != SCENE_NAME:
            raise RuntimeError(f"unexpected CoppeliaSim scene: {scene}")
        fingerprint = r1_plan["validation"]["scene_fingerprint"]
        if scene.stat().st_size != int(fingerprint.get("size", -1)):
            raise RuntimeError("R2 scene size differs; repeat full preflight")
        if _sha256(scene) != fingerprint["sha256"]:
            raise RuntimeError("R2 scene hash differs; repeat full preflight")

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
        for object_name, expected_position in (
            ("BOX_BLANK", BOX_ASSEMBLY_POSITION),
            ("PCB_SUPPLY", PCB_SUPPLY_POSITION),
        ):
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
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        sim_ik = client.require("simIK")
        robot = self.bridge.get_object_handle(ROBOT_ID)
        joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
        virtual_tip = self._create_virtual_tcp(robot)
        try:
            prepared_paths = self._build_paths(
                sim_ik, robot, virtual_tip, joints
            )
        finally:
            sim.removeObjects([virtual_tip])
        self._prepared_paths = prepared_paths
        return {
            "robot_id": ROBOT_ID,
            "prepared_actions": [R2_PCB_PLACED],
            "path_points": {
                name: len(path) for name, path in self._prepared_paths.items()
            },
        }

    def set_continuous_stepping(self, enabled: bool) -> None:
        self._continuous_stepping = bool(enabled)

    def set_pre_positioned(self, action: str, config: list[float]) -> None:
        """Record the joint config set by ``_preposition_robots()``.

        When not ``None`` the controller skips the initial-approach segment
        because the robot is already waiting at its pick APP.
        """
        _ = action
        self._pre_positioned_config = list(config)

    def _create_virtual_tcp(self, robot: int) -> int:
        sim = self.bridge.sim
        original_tip = _find_alias(sim, robot, "R2_gripper_tip")
        parent = sim.getObjectParent(original_tip)
        virtual_tip = sim.createDummy(0.004)
        sim.setObjectAlias(virtual_tip, "R2_Runtime_Vacuum_TCP")
        sim.setObjectParent(virtual_tip, parent, False)
        sim.setObjectPose(
            virtual_tip,
            parent,
            [0.0, 0.0, VIRTUAL_TCP_OFFSET_M, 0.0, 0.0, 0.0, 1.0],
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
                client = getattr(self.bridge, "_client", None)
                if client is None:
                    raise RuntimeError(
                        "CoppeliaSim remote client is unavailable"
                    )
                paths = self._build_paths(
                    client.require("simIK"), robot, virtual_tip, joints
                )

            original_max_velocities = [
                sim.getObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel
                )
                for joint in joints
            ]
            max_velocity = math.radians(
                max(60.0, self.speed_deg_s * 1.35)
            )
            for joint in joints:
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, max_velocity
                )
                sim.setJointTargetPosition(joint, 0.0)

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
            descent_speed = math.radians(
                min(self.speed_deg_s * 0.75, DESCENT_SPEED_CAP_DEG_S)
            )

            if self._pre_positioned_config is None:
                if not prepared_mode:
                    runner.hold(0.5, "R2 startup")
                runner.execute_path(
                    "R2 initial_to_pick_app",
                    paths["initial_to_pick_app"],
                    transfer_speed,
                )
                runner.hold(self.hold_seconds, "R2 hold above PCB")
            # else: pre-positioned at pick APP — skip the initial approach
            # to reduce simulation-time handoff delay.
            runner.execute_path(
                "R2 descend_to_pick_tcp",
                paths["pick_descend"],
                descent_speed,
            )

            sim.setObjectParent(pcb, virtual_tip, True)
            attached = True
            runner.set_payload(pcb)
            start = sim.getObjectPosition(pcb, -1)
            for index in range(1, 14):
                position = list(start)
                position[2] += PCB_VISUAL_OFFSET_Z * index / 13.0
                sim.setObjectPosition(pcb, -1, position)
                runner.step(
                    "R2 PCB visual offset", force_collision=True
                )

            with self.assembly_lock:
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

                sim.setObjectParent(pcb, parts, True)
                attached = False
                runner.set_payload(None)
                runner.step("R2 PCB detached", force_full=True)
                runner.execute_path(
                    "R2 retreat_and_return_home",
                    paths["return_home"],
                    transfer_speed,
                )
            runner.hold(0.4, "R2 final home hold")

            final_joints = runner.joint_positions()
            if not _near(
                final_joints,
                [0.0] * 6,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError("R2 did not return to the validated zero state")
            result = {
                "action": action,
                "visual_suction_only": True,
                "runtime_orientation_deg": list(RUNTIME_ORIENTATION_DEG),
                "virtual_tcp_offset_m": VIRTUAL_TCP_OFFSET_M,
                "pcb_visual_offset_m": PCB_VISUAL_OFFSET_Z,
                "final_joint_positions_deg": [
                    round(math.degrees(value), 6)
                    for value in final_joints
                ],
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


__all__ = ["R2_ACTIONS", "R2_PCB_PLACED", "R2MotionController"]
