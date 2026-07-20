"""Validated visual R4 screw motion for the five-CR5A simulation cell.

The saved scene does not contain a screwdriver model.  This controller creates
a runtime-only 100 mm tool, keeps every Git APP/TCP/PRESS target unchanged,
and uses the approved vertical ``(180, 0, -135)`` degree tool orientation.
The action is a visual screw-driving milestone; physical torque is not claimed.
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


R4_SCREW_DONE = "R4_SCREW_DONE"
R4_ACTIONS = frozenset({R4_SCREW_DONE})

ROBOT_ID = "R4"
SCENE_NAME = "five_cr5a_cell.ttt"
TARGET_NAMES = (
    "R4_SCREW_APP",
    "R4_SCREW_TCP",
    "R4_SCREW_PRESS",
)
PROTECTED_TARGETS = {
    "R4_SCREW_APP": {
        "position": [0.21, -0.02, 0.55],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R4_SCREW_TCP": {
        "position": [0.21, -0.02, 0.36],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
    "R4_SCREW_PRESS": {
        "position": [0.21, -0.02, 0.33],
        "orientation_euler": [0.0, 0.0, 0.0],
    },
}

RUNTIME_ORIENTATION_DEG = (180.0, 0.0, -135.0)
RUNTIME_ORIENTATION = tuple(
    math.radians(value) for value in RUNTIME_ORIENTATION_DEG
)
TOOL_TCP_OFFSET_M = 0.100
TOOL_HANDLE_LENGTH_M = 0.035
TOOL_HANDLE_DIAMETER_M = 0.032
TOOL_SHAFT_DIAMETER_M = 0.008
TOOL_ROTATION_TURNS = 2.0
TOOL_ROTATION_SECONDS = 1.6

INSPECTION_PRODUCT_POSITION = (0.15, 0.05, 0.216)
POSITION_TOLERANCE_M = 0.002
JOINT_TOLERANCE_DEG = 0.30
TARGET_TOLERANCE = 1e-6
WORKSPACE_TOLERANCE_M = 0.003

TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_CAP_DEG_S = 24.0
HOLD_SECONDS = 0.8

RUNTIME_PREFIX = "R4_Runtime_"
RUNTIME_BRIDGE_ALIAS = f"{RUNTIME_PREFIX}Command_Bridge"
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
            raise RuntimeError("cannot join an empty R4 path")
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
                    "R4 joined path discontinuity "
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
                world_bb_pose[index] + rotated[index]
                for index in range(3)
            ]
            lower = [min(a, b) for a, b in zip(lower, point)]
            upper = [max(a, b) for a, b in zip(upper, point)]
    return lower, upper


def _solve_target(
    sim_ik: Any,
    base: int,
    tip: int,
    joints: list[int],
    target: int,
    seed: list[float],
) -> list[float]:
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
        for joint, value in zip(joints, seed):
            sim_ik.setJointPosition(environment, scene_to_ik[joint], value)
        sim_ik.setGroupCalculation(
            environment,
            group,
            sim_ik.method_damped_least_squares,
            0.1,
            300,
        )
        sim_ik.setElementPrecision(
            environment, group, element, [0.001, math.radians(1.0)]
        )
        result, _, precision = sim_ik.handleGroup(environment, group)
        if result != sim_ik.result_success:
            raise RuntimeError(
                "R4 IK failed: "
                f"linear={float(precision[0]) * 1000.0:.3f} mm, "
                f"angular={math.degrees(float(precision[1])):.3f} deg"
            )
        return [
            float(sim_ik.getJointPosition(environment, scene_to_ik[joint]))
            for joint in joints
        ]
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
            300,
        )
        flat = sim_ik.generatePath(
            environment,
            group,
            ik_joints,
            scene_to_ik[tip],
            point_count,
        )
        if len(flat) != point_count * len(joints):
            raise RuntimeError("R4 Cartesian path generation failed")
        return _unwrap_path(
            [
                [float(value) for value in flat[index : index + len(joints)]]
                for index in range(0, len(flat), len(joints))
            ]
        )
    finally:
        sim_ik.eraseEnvironment(environment)


class R4SafetyGuard:
    """R4 environment, self, runtime-tool, and workspace checks."""

    def __init__(self, sim: Any, robot: int, tool_shapes: set[int]):
        self.sim = sim
        self.robot_shapes = set(
            sim.getObjectsInTree(robot, sim.object_shape_type, 0)
        )
        self.tool_shapes = set(tool_shapes)
        self.moving_shapes = self.robot_shapes | self.tool_shapes
        self.mover = sim.createCollection(1)
        self.environment = sim.createCollection(1)
        for handle in self.moving_shapes:
            sim.addItemToCollection(
                self.mover, sim.handle_single, handle, 0
            )

        inspection_product = sim.getObject(PARTS["INSPECTION_PRODUCT"])
        inspection_shapes = set(
            sim.getObjectsInTree(
                inspection_product, sim.object_shape_type, 0
            )
        )
        robot_base = sim.getObject(ROBOT_BASES[ROBOT_ID])
        for handle in sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        ):
            if handle in self.moving_shapes or handle == robot_base:
                continue
            if (
                sim.getObjectInt32Param(
                    handle, sim.objintparam_visibility_layer
                )
                == 0
                and handle not in inspection_shapes
            ):
                continue
            sim.addItemToCollection(
                self.environment, sim.handle_single, handle, 0
            )

        aliases = ["base_link_respondable"] + [
            f"Link{index}_respondable" for index in range(1, 7)
        ]
        by_alias = {
            sim.getObjectAlias(handle): handle
            for handle in self.robot_shapes
        }
        missing = [alias for alias in aliases if alias not in by_alias]
        if missing:
            raise RuntimeError(f"R4 collision links missing: {missing}")
        self.links = [by_alias[alias] for alias in aliases]

        self.tool_collection = sim.createCollection(1)
        self.lower_arm_collection = sim.createCollection(1)
        for handle in self.tool_shapes:
            sim.addItemToCollection(
                self.tool_collection, sim.handle_single, handle, 0
            )
        # Link6 is the intentional mounting contact for the runtime tool.
        for handle in self.links[:-1]:
            sim.addItemToCollection(
                self.lower_arm_collection, sim.handle_single, handle, 0
            )

        self.shape_bbs = {
            handle: sim.getShapeBB(handle) for handle in self.moving_shapes
        }

    def close(self) -> None:
        self.sim.destroyCollection(self.mover)
        self.sim.destroyCollection(self.environment)
        self.sim.destroyCollection(self.tool_collection)
        self.sim.destroyCollection(self.lower_arm_collection)

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
                            self.sim.getObjectAlias(handle, 1)
                            for handle in pair
                        ]
                        raise RuntimeError(
                            f"R4 self collision during {label}: {paths}"
                        )
            state, pair = self.sim.checkCollision(
                self.tool_collection, self.lower_arm_collection
            )
            if state:
                paths = [
                    self.sim.getObjectAlias(handle, 1) for handle in pair
                ]
                raise RuntimeError(
                    f"R4 tool-to-arm collision during {label}: {paths}"
                )

        if not check_workspace:
            return
        lower, upper = _shape_tree_bounds(
            self.sim, self.moving_shapes, self.shape_bbs
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
                    f"R4 workspace violation during {label}: axis={axis}, "
                    f"actual=[{actual_low:.4f},{actual_high:.4f}], "
                    f"allowed=[{allowed_low:.4f},{allowed_high:.4f}]"
                )


class _SmoothRunner:
    def __init__(
        self,
        bridge: SimBridge,
        joints: list[int],
        command_script: int,
        guard: R4SafetyGuard,
        collision_check_interval: int,
        workspace_check_interval: int,
    ):
        self.bridge = bridge
        self.sim = bridge.sim
        self.joints = joints
        self.command_script = command_script
        self.guard = guard
        self.collision_check_interval = max(1, collision_check_interval)
        self.workspace_check_interval = max(1, workspace_check_interval)
        self.dt = float(self.sim.getSimulationTimeStep())
        self.step_index = 0

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
                self.bridge.last_error or "R4 simulation step failed"
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


class R4MotionController:
    """Execute the approved visual R4 screw-driving action."""

    def __init__(
        self,
        bridge: SimBridge,
        r1_plan_path: Path = R1_PLAN_PATH,
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
        self.inspection_lock = inspection_lock or threading.Lock()
        self.speed_deg_s = float(speed_deg_s)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.collision_check_interval = int(collision_check_interval)
        self.workspace_check_interval = int(workspace_check_interval)
        self._prepared_paths: Optional[
            dict[str, list[list[float]]]
        ] = None
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

    def _validate_static(self) -> None:
        plan = load_r1_plan(self.r1_plan_path)
        scene = Path(self.bridge.scene_path())
        if scene.name != SCENE_NAME:
            raise RuntimeError(f"unexpected CoppeliaSim scene: {scene}")
        fingerprint = plan["validation"]["scene_fingerprint"]
        if scene.stat().st_size != int(fingerprint.get("size", -1)):
            raise RuntimeError("R4 scene size differs; repeat full preflight")
        if _sha256(scene) != fingerprint["sha256"]:
            raise RuntimeError("R4 scene hash differs; repeat full preflight")

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

    def _validate_preflight(self, verify_static: bool = True) -> None:
        sim = self.bridge.sim
        if sim.getSimulationState() == sim.simulation_stopped:
            raise RuntimeError("R4 screw action requires the coordinated scene")
        if verify_static:
            self._validate_static()

        expected_r4 = (
            self._pre_positioned_config
            if self._pre_positioned_config is not None
            else [0.0] * 6
        )
        if not _near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            expected_r4,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError(
                "R4 is not at the validated start "
                f"(expected pre-positioned={self._pre_positioned_config is not None})"
            )

        product = self.bridge.get_object_handle("INSPECTION_PRODUCT")
        parts = sim.getObject(f"{PARTS['INSPECTION_PRODUCT'].rsplit('/', 1)[0]}")
        if sim.getObjectParent(product) != parts:
            raise RuntimeError("INSPECTION_PRODUCT is not owned by /Parts")
        actual_position = [
            float(value) for value in sim.getObjectPosition(product, -1)
        ]
        if not _near(
            actual_position,
            list(INSPECTION_PRODUCT_POSITION),
            POSITION_TOLERANCE_M,
        ):
            raise RuntimeError(
                "inspection product is not at the validated screw position"
            )

    @staticmethod
    def _remove_stale_runtime_objects(sim: Any, robot: int) -> None:
        stale = {
            handle
            for handle in sim.getObjectsInTree(robot, sim.handle_all, 0)
            if sim.getObjectAlias(handle).startswith(RUNTIME_PREFIX)
        }
        depths: dict[int, int] = {}
        for handle in stale:
            depth = 0
            parent = sim.getObjectParent(handle)
            while parent in stale:
                depth += 1
                parent = sim.getObjectParent(parent)
            depths[handle] = depth
        for handle in sorted(stale, key=depths.get, reverse=True):
            sim.removeObjects([handle])

    @staticmethod
    def _remove_runtime_tool(sim: Any, tool: dict[str, Any]) -> None:
        # CoppeliaSim reparents children when only their parent is removed.
        # Delete the deepest objects first so no runtime shape survives.
        for handle in reversed(tool["objects"]):
            try:
                sim.removeObjects([handle])
            except Exception:
                pass

    def _create_runtime_tool(
        self, robot: int
    ) -> dict[str, Any]:
        sim = self.bridge.sim
        self._remove_stale_runtime_objects(sim, robot)
        link6 = _find_alias(sim, robot, "Link6_visual")

        tip = sim.createDummy(0.004)
        sim.setObjectAlias(tip, f"{RUNTIME_PREFIX}Screwdriver_TCP")
        sim.setObjectParent(tip, link6, False)
        sim.setObjectPose(
            tip,
            link6,
            [0.0, 0.0, TOOL_TCP_OFFSET_M, 0.0, 0.0, 0.0, 1.0],
        )
        sim.setObjectInt32Param(
            tip, sim.objintparam_visibility_layer, 0
        )

        spinner = sim.createDummy(0.003)
        sim.setObjectAlias(spinner, f"{RUNTIME_PREFIX}Screwdriver_Spinner")
        sim.setObjectParent(spinner, link6, False)
        sim.setObjectPose(
            spinner, link6, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        )
        sim.setObjectInt32Param(
            spinner, sim.objintparam_visibility_layer, 0
        )

        shaft_length = TOOL_TCP_OFFSET_M - TOOL_HANDLE_LENGTH_M
        shaft = sim.createPrimitiveShape(
            sim.primitiveshape_cylinder,
            [TOOL_SHAFT_DIAMETER_M, TOOL_SHAFT_DIAMETER_M, shaft_length],
            0,
        )
        sim.setObjectAlias(shaft, f"{RUNTIME_PREFIX}Screwdriver_Shaft")
        sim.setObjectParent(shaft, spinner, False)
        sim.setObjectPose(
            shaft,
            spinner,
            [
                0.0,
                0.0,
                TOOL_HANDLE_LENGTH_M + shaft_length / 2.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        )

        handle = sim.createPrimitiveShape(
            sim.primitiveshape_cylinder,
            [
                TOOL_HANDLE_DIAMETER_M,
                TOOL_HANDLE_DIAMETER_M,
                TOOL_HANDLE_LENGTH_M,
            ],
            0,
        )
        sim.setObjectAlias(handle, f"{RUNTIME_PREFIX}Screwdriver_Handle")
        sim.setObjectParent(handle, link6, False)
        sim.setObjectPose(
            handle,
            link6,
            [
                0.0,
                0.0,
                TOOL_HANDLE_LENGTH_M / 2.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ],
        )

        marker = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid,
            [0.004, 0.012, 0.006],
            0,
        )
        sim.setObjectAlias(marker, f"{RUNTIME_PREFIX}Rotation_Marker")
        sim.setObjectParent(marker, spinner, False)
        sim.setObjectPose(
            marker,
            spinner,
            [0.006, 0.0, 0.073, 0.0, 0.0, 0.0, 1.0],
        )

        for shape in (shaft, handle, marker):
            sim.setObjectInt32Param(shape, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(shape, sim.shapeintparam_respondable, 0)
        sim.setShapeColor(
            shaft,
            None,
            sim.colorcomponent_ambient_diffuse,
            [0.65, 0.68, 0.72],
        )
        sim.setShapeColor(
            handle,
            None,
            sim.colorcomponent_ambient_diffuse,
            [0.08, 0.10, 0.12],
        )
        sim.setShapeColor(
            marker,
            None,
            sim.colorcomponent_ambient_diffuse,
            [0.90, 0.15, 0.08],
        )
        return {
            "tip": tip,
            "spinner": spinner,
            "objects": [tip, spinner, shaft, handle, marker],
            "shapes": {shaft, handle, marker},
            "link6": link6,
        }

    def _build_paths(
        self,
        sim_ik: Any,
        robot: int,
        tip: int,
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
                sim.setObjectAlias(target, f"{RUNTIME_PREFIX}Target_{name}")
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

            app = _solve_target(
                sim_ik,
                base,
                tip,
                joints,
                temporary["R4_SCREW_APP"],
                [0.0] * 6,
            )
            tcp = _solve_target(
                sim_ik,
                base,
                tip,
                joints,
                temporary["R4_SCREW_TCP"],
                app,
            )
            _solve_target(
                sim_ik,
                base,
                tip,
                joints,
                temporary["R4_SCREW_PRESS"],
                tcp,
            )

            descend = _generate_cartesian_path(
                sim_ik,
                base,
                tip,
                joints,
                temporary["R4_SCREW_TCP"],
                app,
                61,
            )
            press = _generate_cartesian_path(
                sim_ik,
                base,
                tip,
                joints,
                temporary["R4_SCREW_PRESS"],
                tcp,
                21,
            )
            initial = _interpolate_joint_line([0.0] * 6, app, 101)
            paths = {
                "initial_to_app": initial,
                "descend_to_tcp": descend,
                "press": press,
                "retract_and_home": _join_paths(
                    list(reversed(press)),
                    list(reversed(descend)),
                    list(reversed(initial)),
                ),
            }
            if self._target_snapshot() != protected_before:
                raise RuntimeError(
                    "R4 protected Git targets changed during planning"
                )
            return paths
        finally:
            if temporary:
                sim.removeObjects(list(temporary.values()))

    def prepare(self, action: str = R4_SCREW_DONE) -> dict[str, Any]:
        if action not in R4_ACTIONS:
            raise ValueError(f"unsupported R4 action: {action}")
        sim = self.bridge.sim
        if sim.getSimulationState() != sim.simulation_stopped:
            raise RuntimeError("R4 preparation requires a stopped scene")
        self._validate_static()
        if not _near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            [0.0] * 6,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError("R4 is not zero during preparation")
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        robot = self.bridge.get_object_handle(ROBOT_ID)
        joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
        tool = self._create_runtime_tool(robot)
        try:
            prepared_paths = self._build_paths(
                client.require("simIK"), robot, tool["tip"], joints
            )
        finally:
            self._remove_runtime_tool(sim, tool)
        self._prepared_paths = prepared_paths
        return {
            "robot_id": ROBOT_ID,
            "prepared_actions": [R4_SCREW_DONE],
            "path_points": {
                name: len(path) for name, path in prepared_paths.items()
            },
        }

    def set_continuous_stepping(self, enabled: bool) -> None:
        self._continuous_stepping = bool(enabled)

    def set_pre_positioned(self, action: str, config: list[float]) -> None:
        """Record the joint config set by ``_preposition_robots()``."""
        _ = action
        self._pre_positioned_config = list(config)

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

    @staticmethod
    def _product_visibility(sim: Any, product: int) -> dict[int, int]:
        return {
            handle: int(
                sim.getObjectInt32Param(
                    handle, sim.objintparam_visibility_layer
                )
            )
            for handle in sim.getObjectsInTree(
                product, sim.object_shape_type, 0
            )
        }

    @staticmethod
    def _set_product_visible(sim: Any, layers: dict[int, int]) -> None:
        for handle in layers:
            sim.setObjectInt32Param(
                handle, sim.objintparam_visibility_layer, 1
            )

    @staticmethod
    def _restore_product_visibility(
        sim: Any, layers: dict[int, int]
    ) -> None:
        for handle, layer in layers.items():
            sim.setObjectInt32Param(
                handle, sim.objintparam_visibility_layer, layer
            )

    def execute(self, action: str) -> dict[str, Any]:
        if action not in R4_ACTIONS:
            raise ValueError(f"unsupported R4 action: {action}")
        prepared_mode = self._prepared_paths is not None
        self._validate_preflight(verify_static=not prepared_mode)

        sim = self.bridge.sim
        robot = -1
        command_script = -1
        runner: Optional[_SmoothRunner] = None
        guard: Optional[R4SafetyGuard] = None
        tool: Optional[dict[str, Any]] = None
        joints: list[int] = []
        original_max_velocities: list[float] = []
        product_layers: dict[int, int] = {}
        succeeded = False
        try:
            self.bridge.set_stepping(True)
            robot = self.bridge.get_object_handle(ROBOT_ID)
            joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
            product = self.bridge.get_object_handle("INSPECTION_PRODUCT")
            product_layers = self._product_visibility(sim, product)
            self._set_product_visible(sim, product_layers)

            tool = self._create_runtime_tool(robot)
            if self._prepared_paths is not None:
                paths = self._prepared_paths
            else:
                client = getattr(self.bridge, "_client", None)
                if client is None:
                    raise RuntimeError(
                        "CoppeliaSim remote client is unavailable"
                    )
                paths = self._build_paths(
                    client.require("simIK"), robot, tool["tip"], joints
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
                    self.bridge.last_error or "cannot start R4 stepping"
                )
            guard = R4SafetyGuard(sim, robot, tool["shapes"])
            runner = _SmoothRunner(
                self.bridge,
                joints,
                command_script,
                guard,
                self.collision_check_interval,
                self.workspace_check_interval,
            )
            if prepared_mode:
                runner.step("R4 runtime bridge initialized", force_full=True)
            transfer_speed = math.radians(self.speed_deg_s)
            descent_speed = math.radians(
                min(self.speed_deg_s * 0.75, DESCENT_SPEED_CAP_DEG_S)
            )

            with self.inspection_lock:
                if self._pre_positioned_config is None:
                    if not prepared_mode:
                        runner.hold(0.5, "R4 startup")
                    # Generator initialization hides template product shapes.
                    # R4 treats this inspection product as the executor-owned
                    # visual workpiece until the real R3 transfer is available.
                    self._set_product_visible(sim, product_layers)
                    runner.step("R4 inspection product visible", force_full=True)
                    runner.execute_path(
                        "R4 initial_to_screw_app",
                        paths["initial_to_app"],
                        transfer_speed,
                    )
                # else: pre-positioned at screw APP — skip the initial
                # approach to reduce simulation-time handoff delay.
                runner.hold(self.hold_seconds, "R4 hold above screw")
                runner.execute_path(
                    "R4 descend_to_screw_tcp",
                    paths["descend_to_tcp"],
                    descent_speed,
                )
                runner.hold(0.4, "R4 TCP hold")
                runner.execute_path(
                    "R4 press_screwdriver",
                    paths["press"],
                    descent_speed,
                )
                runner.hold(0.3, "R4 press hold")

                rotation_steps = max(
                    2, math.ceil(TOOL_ROTATION_SECONDS / runner.dt)
                )
                for index in range(1, rotation_steps + 1):
                    angle = (
                        2.0
                        * math.pi
                        * TOOL_ROTATION_TURNS
                        * index
                        / rotation_steps
                    )
                    sim.setObjectOrientation(
                        tool["spinner"], tool["link6"], [0.0, 0.0, angle]
                    )
                    runner.step("R4 visible screw rotation")
                sim.setObjectOrientation(
                    tool["spinner"], tool["link6"], [0.0, 0.0, 0.0]
                )
                runner.step("R4 rotation complete", force_full=True)
                self.bridge.set_string_signal("cell_screw_state", "done")

                runner.execute_path(
                    "R4 retract_and_return_home",
                    paths["retract_and_home"],
                    transfer_speed,
                )
                runner.hold(0.4, "R4 final home hold")

            final_joints = runner.joint_positions()
            if not _near(
                final_joints,
                [0.0] * 6,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError("R4 did not return to the validated zero state")
            result = {
                "action": action,
                "visual_screwdriver_only": True,
                "physical_torque_validated": False,
                "runtime_orientation_deg": list(RUNTIME_ORIENTATION_DEG),
                "runtime_tool_tcp_offset_m": TOOL_TCP_OFFSET_M,
                "rotation_turns": TOOL_ROTATION_TURNS,
                "final_joint_positions_deg": [
                    round(math.degrees(value), 6)
                    for value in final_joints
                ],
            }
            succeeded = True
            return result
        finally:
            if guard is not None:
                guard.close()
            if succeeded:
                if command_script != -1:
                    sim.removeObjects([command_script])
                if tool is not None:
                    self._remove_runtime_tool(sim, tool)
                for joint, original in zip(joints, original_max_velocities):
                    sim.setObjectFloatParam(
                        joint, sim.jointfloatparam_maxvel, original
                    )
                if not self._continuous_stepping:
                    self.bridge.set_stepping(False)
            else:
                if sim.getSimulationState() != sim.simulation_stopped:
                    self.bridge.stop_simulation()
                else:
                    self.bridge.set_stepping(False)
                if command_script != -1:
                    try:
                        sim.removeObjects([command_script])
                    except Exception:
                        pass
                if tool is not None:
                    self._remove_runtime_tool(sim, tool)
                for joint, original in zip(joints, original_max_velocities):
                    try:
                        sim.setObjectFloatParam(
                            joint, sim.jointfloatparam_maxvel, original
                        )
                    except Exception:
                        pass
                if product_layers:
                    try:
                        self._restore_product_visibility(sim, product_layers)
                    except Exception:
                        pass


__all__ = ["R4_ACTIONS", "R4_SCREW_DONE", "R4MotionController"]
