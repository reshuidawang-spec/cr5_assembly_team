"""Validated R1 box and terminal motion for the five-CR5A CoppeliaSim cell.

The checked-in plan contains joint-space samples generated and statically
validated against ``five_cr5a_cell.ttt``. Runtime execution still checks the
robot, payload, environment, self-collision, and R1 workspace. Grasping is a
visual attach operation; this module does not claim physical grasp validation.
"""

from __future__ import annotations

import bisect
import hashlib
import itertools
import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Optional

from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import (
    PARTS,
    ROBOT_BASES,
    ROBOT_ROOTS,
    WORKSPACES,
)


R1_BOX_PLACED = "R1_BOX_PLACED"
R1_TERMINAL_PLACED = "R1_TERMINAL_PLACED"
R1_COMPLETE_CYCLE = "R1_COMPLETE_CYCLE"
R1_ACTIONS = frozenset(
    {R1_BOX_PLACED, R1_TERMINAL_PLACED, R1_COMPLETE_CYCLE}
)

PLAN_VERSION = 2
PLAN_PATH = Path(__file__).with_name("plans") / "r1_complete_cycle_plan.json"
SCENE_NAME = "five_cr5a_cell.ttt"

BOX_SUPPLY_POSITION = (-1.8, 0.35, 0.156)
TERMINAL_SUPPLY_POSITION = (-1.9, 0.1, 0.1735)
BOX_VISUAL_OFFSET_Z = 0.060
TERMINAL_VISUAL_OFFSET_Z = 0.028
TERMINAL_COORDINATED_VISUAL_OFFSET_Z = 0.056
PCB_ASSEMBLY_POSITION = (-1.150058, 0.200161, 0.281739)

TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_CAP_DEG_S = 24.0
HOLD_SECONDS = 0.8
JOINT_START_TOLERANCE_DEG = 0.25
OBJECT_POSITION_TOLERANCE_M = 0.002
TARGET_TOLERANCE = 1e-6
WORKSPACE_TOLERANCE_M = 0.003

TARGET_NAMES = (
    "R1_BOX_PICK_APP",
    "R1_BOX_PICK_TCP",
    "R1_BOX_PLACE_APP",
    "R1_BOX_PLACE_TCP",
    "R1_TERMINAL_PICK_APP",
    "R1_TERMINAL_PICK_TCP",
    "R1_TERMINAL_PLACE_APP",
    "R1_TERMINAL_PLACE_TCP",
)

REQUIRED_PATHS = (
    "initial_to_box_pick_app",
    "box_descend",
    "box_lift_and_transfer",
    "box_place_descend",
    "box_retreat_and_terminal_approach",
    "terminal_descend",
    "terminal_lift_and_transfer",
    "terminal_place_descend",
    "return_home",
)

RUNTIME_BRIDGE_ALIAS = "R1_Runtime_Command_Bridge"
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


def load_r1_plan(path: Path = PLAN_PATH) -> dict[str, Any]:
    """Load and structurally validate the immutable R1 replay plan."""
    try:
        plan = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load R1 plan {path}: {exc}") from exc

    if plan.get("plan_version") != PLAN_VERSION:
        raise RuntimeError(f"unsupported R1 plan version in {path}")
    if plan.get("protected_targets_modified") is not False:
        raise RuntimeError("R1 plan does not preserve the protected Git targets")
    if plan.get("box_orientation_euler_deg") != [180.0, 0.0, -90.0]:
        raise RuntimeError("R1 box runtime orientation is not validated")
    if plan.get("terminal_orientation_euler_deg") != [180.0, 0.0, -180.0]:
        raise RuntimeError("R1 terminal runtime orientation is not validated")

    protected = plan.get("protected_targets")
    if not isinstance(protected, dict) or set(protected) != set(TARGET_NAMES):
        raise RuntimeError("R1 plan target snapshot is incomplete")
    paths = plan.get("paths")
    if not isinstance(paths, dict):
        raise RuntimeError("R1 plan has no paths")
    for name in REQUIRED_PATHS:
        if not _finite_joint_path(paths.get(name)):
            raise RuntimeError(f"R1 plan path is invalid: {name}")

    workspace = plan.get("workspace", {})
    expected_workspace = WORKSPACES["R1"]
    if tuple(workspace.get("lower", ())) != tuple(expected_workspace["lower"]):
        raise RuntimeError("R1 plan lower workspace wall differs from the contract")
    if tuple(workspace.get("upper", ())) != tuple(expected_workspace["upper"]):
        raise RuntimeError("R1 plan upper workspace wall differs from the contract")
    shared = workspace.get("assembly_shared", {})
    expected_shared = WORKSPACES["ASSEMBLY_SHARED"]
    if tuple(shared.get("lower", ())) != tuple(expected_shared["lower"]):
        raise RuntimeError("R1 plan shared-zone lower bound differs from the contract")
    if tuple(shared.get("upper", ())) != tuple(expected_shared["upper"]):
        raise RuntimeError("R1 plan shared-zone upper bound differs from the contract")

    validation = plan.get("validation", {})
    if validation.get("collision_free") is not True:
        raise RuntimeError("R1 plan has no successful collision validation")
    fingerprint = validation.get("scene_fingerprint", {})
    if not isinstance(fingerprint.get("sha256"), str):
        raise RuntimeError("R1 plan has no validated scene fingerprint")
    return plan


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def cumulative_max_joint_distance(configs: list[list[float]]) -> list[float]:
    cumulative = [0.0]
    for first, second in zip(configs, configs[1:]):
        cumulative.append(
            cumulative[-1]
            + max(abs(b - a) for a, b in zip(first, second))
        )
    return cumulative


def interpolate_path(
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


def minimum_jerk(fraction: float) -> float:
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
            point = [world_bb_pose[index] + rotated[index] for index in range(3)]
            lower = [min(a, b) for a, b in zip(lower, point)]
            upper = [max(a, b) for a, b in zip(upper, point)]
    return lower, upper


class _EnvironmentCollisionProbe:
    def __init__(self, sim: Any, robot: int, payload: Optional[int]):
        self.sim = sim
        self.mover = sim.createCollection(1)
        self.environment = sim.createCollection(1)
        self.payload_collection: Optional[int] = None
        self.arm_collection: Optional[int] = None

        tool = sim.getObject("/R1/R1_ROBOTIQ85")
        self.tool_shapes = set(
            sim.getObjectsInTree(tool, sim.object_shape_type, 0)
        )
        robot_shapes = set(
            sim.getObjectsInTree(robot, sim.object_shape_type, 0)
        )
        self.payload_shapes = (
            set(sim.getObjectsInTree(payload, sim.object_shape_type, 0))
            if payload is not None
            else set()
        )
        mover_shapes = robot_shapes | self.payload_shapes
        for handle in mover_shapes:
            sim.addItemToCollection(self.mover, sim.handle_single, handle, 0)

        if payload is not None:
            self.payload_collection = sim.createCollection(1)
            self.arm_collection = sim.createCollection(1)
            for handle in self.payload_shapes:
                sim.addItemToCollection(
                    self.payload_collection, sim.handle_single, handle, 0
                )
            for handle in robot_shapes - self.tool_shapes:
                alias = sim.getObjectAlias(handle)
                if alias == "base_link_respondable" or (
                    alias.startswith("Link") and alias.endswith("_respondable")
                ):
                    sim.addItemToCollection(
                        self.arm_collection, sim.handle_single, handle, 0
                    )

        robot_base = sim.getObject(ROBOT_BASES["R1"])
        for handle in sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        ):
            if handle in mover_shapes or handle == robot_base:
                continue
            if sim.getObjectInt32Param(
                handle, sim.objintparam_visibility_layer
            ) == 0:
                continue
            sim.addItemToCollection(
                self.environment, sim.handle_single, handle, 0
            )

    def close(self) -> None:
        self.sim.destroyCollection(self.mover)
        self.sim.destroyCollection(self.environment)
        if self.payload_collection is not None:
            self.sim.destroyCollection(self.payload_collection)
        if self.arm_collection is not None:
            self.sim.destroyCollection(self.arm_collection)

    def collision(self) -> tuple[bool, list[int]]:
        state, pair = self.sim.checkCollision(self.mover, self.environment)
        return bool(state), pair

    def pair_paths(self, pair: list[int]) -> list[str]:
        return [self.sim.getObjectAlias(handle, 1) for handle in pair]

    def payload_to_arm_collision(self) -> Optional[list[str]]:
        if self.payload_collection is None or self.arm_collection is None:
            return None
        state, pair = self.sim.checkCollision(
            self.payload_collection, self.arm_collection
        )
        return self.pair_paths(pair) if state else None


class _SelfCollisionProbe:
    def __init__(self, sim: Any, robot: int):
        self.sim = sim
        aliases = ["base_link_respondable"] + [
            f"Link{index}_respondable" for index in range(1, 7)
        ]
        by_alias = {
            sim.getObjectAlias(handle): handle
            for handle in sim.getObjectsInTree(
                robot, sim.object_shape_type, 0
            )
        }
        missing = [alias for alias in aliases if alias not in by_alias]
        if missing:
            raise RuntimeError(f"R1 self-collision links are missing: {missing}")
        links = [by_alias[alias] for alias in aliases]
        self.tail_collections: list[tuple[int, int]] = []
        for index in range(len(links) - 2):
            collection = sim.createCollection(1)
            for handle in links[index + 2 :]:
                sim.addItemToCollection(
                    collection, sim.handle_single, handle, 0
                )
            self.tail_collections.append((links[index], collection))

        tool = sim.getObject("/R1/R1_ROBOTIQ85")
        self.tool_collection = sim.createCollection(1)
        for handle in sim.getObjectsInTree(
            tool, sim.object_shape_type, 0
        ):
            sim.addItemToCollection(
                self.tool_collection, sim.handle_single, handle, 0
            )
        self.arm_collection = sim.createCollection(1)
        for handle in links:
            sim.addItemToCollection(
                self.arm_collection, sim.handle_single, handle, 0
            )

    def close(self) -> None:
        for _, collection in self.tail_collections:
            self.sim.destroyCollection(collection)
        self.sim.destroyCollection(self.tool_collection)
        self.sim.destroyCollection(self.arm_collection)

    def collision(self) -> Optional[list[str]]:
        for first, tail in self.tail_collections:
            state, pair = self.sim.checkCollision(first, tail)
            if state:
                return [self.sim.getObjectAlias(handle, 1) for handle in pair]
        state, pair = self.sim.checkCollision(
            self.tool_collection, self.arm_collection
        )
        if state:
            return [self.sim.getObjectAlias(handle, 1) for handle in pair]
        return None


class R1SafetyGuard:
    """Runtime collision and invisible-wall checks for one payload state."""

    def __init__(self, sim: Any, robot: int, payload: Optional[int] = None):
        self.sim = sim
        self.payload = payload
        self.environment = _EnvironmentCollisionProbe(sim, robot, payload)
        self.self_collision = _SelfCollisionProbe(sim, robot)
        self.workspace_shapes = set(
            sim.getObjectsInTree(robot, sim.object_shape_type, 0)
        )
        if payload is not None:
            self.workspace_shapes.update(
                sim.getObjectsInTree(payload, sim.object_shape_type, 0)
            )
        self.shape_bbs = {
            shape: sim.getShapeBB(shape) for shape in self.workspace_shapes
        }

    def close(self) -> None:
        self.environment.close()
        self.self_collision.close()

    def check(
        self,
        label: str,
        check_workspace: bool = True,
        check_internal: bool = True,
    ) -> None:
        collision, pair = self.environment.collision()
        if collision:
            raise RuntimeError(
                f"collision during {label}: {self.environment.pair_paths(pair)}"
            )
        if check_internal:
            self_pair = self.self_collision.collision()
            if self_pair is not None:
                raise RuntimeError(f"self collision during {label}: {self_pair}")
            payload_pair = self.environment.payload_to_arm_collision()
            if payload_pair is not None:
                raise RuntimeError(
                    f"payload-to-arm collision during {label}: {payload_pair}"
                )
        if not check_workspace:
            return

        lower, upper = _shape_tree_bounds(
            self.sim, self.workspace_shapes, self.shape_bbs
        )
        allowed = WORKSPACES["R1"]
        for axis, actual_low, actual_high, allowed_low, allowed_high in zip(
            "xyz", lower, upper, allowed["lower"], allowed["upper"]
        ):
            if (
                actual_low < allowed_low - WORKSPACE_TOLERANCE_M
                or actual_high > allowed_high + WORKSPACE_TOLERANCE_M
            ):
                raise RuntimeError(
                    f"R1 workspace violation during {label}: axis={axis}, "
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
        box: int,
        terminal: int,
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
            None: R1SafetyGuard(self.sim, robot),
            box: R1SafetyGuard(self.sim, robot, box),
            terminal: R1SafetyGuard(self.sim, robot, terminal),
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
            raise RuntimeError(self.bridge.last_error or "CoppeliaSim step failed")
        self.step_index += 1
        collision_due = (
            force_collision
            or force_full
            or self.step_index % self.collision_check_interval == 0
        )
        if not collision_due:
            return
        full_due = force_full or self.step_index % self.workspace_check_interval == 0
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

    def attach_visual_payload(
        self, object_name: str, payload: int, offset_z: float, label: str
    ) -> None:
        self.bridge.attach_object(object_name, "R1")
        self.set_payload(payload)
        start = self.sim.getObjectPosition(payload, -1)
        for index in range(1, 13):
            position = list(start)
            position[2] += offset_z * index / 12.0
            self.sim.setObjectPosition(payload, -1, position)
            self.step(f"{label} visual offset", force_collision=True)

    def detach_payload(
        self, object_name: str, payload: int, allow_release_contact: bool
    ) -> None:
        self.bridge.detach_object(payload)
        self.set_payload(payload if allow_release_contact else None)
        self.step("payload detached", force_collision=True)


class R1MotionController:
    """Execute only the three explicitly validated R1 task variants."""

    def __init__(
        self,
        bridge: SimBridge,
        plan_path: Path = PLAN_PATH,
        assembly_lock: Optional[threading.Lock] = None,
        speed_deg_s: float = TRANSFER_SPEED_DEG_S,
        hold_seconds: float = HOLD_SECONDS,
        collision_check_interval: int = 5,
        workspace_check_interval: int = 20,
    ):
        if speed_deg_s <= 0.0:
            raise ValueError("speed_deg_s must be positive")
        self.bridge = bridge
        self.plan_path = Path(plan_path)
        self.assembly_lock = assembly_lock or threading.Lock()
        self.speed_deg_s = float(speed_deg_s)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.collision_check_interval = int(collision_check_interval)
        self.workspace_check_interval = int(workspace_check_interval)
        self._prepared_plan: Optional[dict[str, Any]] = None
        self._continuous_stepping = False
        self._ready_gripper_open = False

    def _freeze_gripper(self) -> None:
        freeze = getattr(self.bridge, "freeze_gripper", None)
        if freeze is not None and not freeze("R1"):
            raise RuntimeError(
                self.bridge.last_error or "cannot freeze R1 gripper"
            )

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

    @staticmethod
    def _near(first: list[float], second: list[float], tolerance: float) -> bool:
        return len(first) == len(second) and max(
            abs(a - b) for a, b in zip(first, second)
        ) <= tolerance

    def _validate_preflight(
        self,
        action: str,
        plan: dict[str, Any],
        verify_static: bool = True,
    ) -> None:
        sim = self.bridge.sim
        simulation_state = sim.getSimulationState()
        if action == R1_TERMINAL_PLACED:
            if simulation_state == sim.simulation_stopped:
                raise RuntimeError(
                    "R1 terminal task requires the running state left by the "
                    "successful box task"
                )
        elif (
            simulation_state != sim.simulation_stopped
            and not self._ready_gripper_open
        ):
            raise RuntimeError(
                "R1 box/complete task requires a freshly loaded stopped scene"
            )
        if verify_static:
            scene = Path(self.bridge.scene_path())
            if scene.name != SCENE_NAME:
                raise RuntimeError(f"unexpected CoppeliaSim scene: {scene}")
            fingerprint = plan["validation"]["scene_fingerprint"]
            if scene.stat().st_size != int(fingerprint.get("size", -1)):
                raise RuntimeError(
                    "R1 plan scene size differs; repeat full preflight"
                )
            if _sha256(scene) != fingerprint["sha256"]:
                raise RuntimeError(
                    "R1 plan scene hash differs; repeat full preflight"
                )

            current_targets = self._target_snapshot()
            for name, expected in plan["protected_targets"].items():
                actual = current_targets[name]
                if not self._near(
                    actual["position"], expected["position"], TARGET_TOLERANCE
                ) or not self._near(
                    actual["orientation_euler"],
                    expected["orientation_euler"],
                    TARGET_TOLERANCE,
                ):
                    raise RuntimeError(f"protected Git target changed: {name}")

        paths = plan["paths"]
        expected_start = (
            paths["box_retreat_and_terminal_approach"][-1]
            if action == R1_TERMINAL_PLACED
            else paths["initial_to_box_pick_app"][0]
        )
        actual_joints = self.bridge.get_robot_joint_positions("R1")
        if not self._near(
            actual_joints,
            expected_start,
            math.radians(JOINT_START_TOLERANCE_DEG),
        ):
            raise RuntimeError(
                f"R1 is not at the validated start for {action}; reload the "
                "scene or complete the preceding R1 task"
            )

        parts = sim.getObject(f"{PARTS['BOX_BLANK'].rsplit('/', 1)[0]}")
        checks = [("TERMINAL_BLOCK_SUPPLY", TERMINAL_SUPPLY_POSITION)]
        if action != R1_TERMINAL_PLACED:
            checks.append(("BOX_BLANK", BOX_SUPPLY_POSITION))
        for object_name, expected_position in checks:
            handle = self.bridge.get_object_handle(object_name)
            if sim.getObjectParent(handle) != parts:
                raise RuntimeError(f"{object_name} is not owned by /Parts")
            actual_position = [
                float(value) for value in sim.getObjectPosition(handle, -1)
            ]
            if not self._near(
                actual_position,
                list(expected_position),
                OBJECT_POSITION_TOLERANCE_M,
            ):
                raise RuntimeError(
                    f"{object_name} is not at its validated supply position"
                )

    def prepare(self, action: str = R1_BOX_PLACED) -> dict[str, Any]:
        if action not in R1_ACTIONS:
            raise ValueError(f"unsupported R1 action: {action}")
        plan = load_r1_plan(self.plan_path)
        self._validate_preflight(R1_BOX_PLACED, plan, verify_static=True)
        self._prepared_plan = plan
        return {
            "robot_id": "R1",
            "prepared_actions": sorted(R1_ACTIONS),
            "path_source": str(self.plan_path),
        }

    def set_continuous_stepping(self, enabled: bool) -> None:
        self._continuous_stepping = bool(enabled)

    def enter_ready(self) -> dict[str, Any]:
        if self._prepared_plan is None:
            raise RuntimeError("R1 paths must be prepared before READY")
        sim = self.bridge.sim
        if sim.getSimulationState() != sim.simulation_stopped:
            raise RuntimeError("R1 READY requires a freshly stopped scene")
        if not self.bridge.start_simulation():
            raise RuntimeError(
                self.bridge.last_error or "cannot start R1 READY simulation"
            )
        robot = self.bridge.get_object_handle("R1")
        guard = R1SafetyGuard(sim, robot)
        dt = float(sim.getSimulationTimeStep())
        try:
            for _ in range(max(1, math.ceil(0.5 / dt))):
                if not self.bridge.step():
                    raise RuntimeError(
                        self.bridge.last_error or "R1 READY startup step failed"
                    )
            guard.check("R1 READY startup", check_workspace=False)
            if not self.bridge.set_gripper("R1", True):
                raise RuntimeError(
                    self.bridge.last_error or "cannot open R1 gripper in READY"
                )
            for _ in range(max(1, math.ceil(0.8 / dt))):
                if not self.bridge.step():
                    raise RuntimeError(
                        self.bridge.last_error or "R1 READY gripper step failed"
                    )
            self._freeze_gripper()
            guard.check("R1 READY gripper open", check_workspace=False)
        except Exception:
            self.bridge.stop_simulation()
            raise
        finally:
            guard.close()
        self._ready_gripper_open = True
        return {
            "simulation_time_s": float(sim.getSimulationTime()),
            "gripper_open": True,
            "stepping_held": True,
        }

    def _create_command_script(self, robot: int) -> int:
        sim = self.bridge.sim
        for handle in sim.getObjectsInTree(robot, sim.object_script_type, 0):
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

    def _terminal_visual_offset(self) -> float:
        """Raise the terminal above the real PCB only in coordinated mode."""
        sim = self.bridge.sim
        try:
            pcb = self.bridge.get_object_handle("PCB_SUPPLY")
            position = [
                float(value) for value in sim.getObjectPosition(pcb, -1)
            ]
            if self._near(
                position,
                list(PCB_ASSEMBLY_POSITION),
                OBJECT_POSITION_TOLERANCE_M,
            ):
                return TERMINAL_COORDINATED_VISUAL_OFFSET_Z
        except Exception:
            pass
        return TERMINAL_VISUAL_OFFSET_Z

    def execute(self, action: str) -> dict[str, Any]:
        if action not in R1_ACTIONS:
            raise ValueError(f"unsupported R1 action: {action}")
        plan = self._prepared_plan or load_r1_plan(self.plan_path)
        prepared_mode = self._prepared_plan is not None
        self._validate_preflight(
            action, plan, verify_static=not prepared_mode
        )

        sim = self.bridge.sim
        paths = plan["paths"]
        robot = self.bridge.get_object_handle("R1")
        joints = self.bridge.get_robot_joint_handles("R1")
        box = self.bridge.get_object_handle("BOX_BLANK")
        terminal = self.bridge.get_object_handle("TERMINAL_BLOCK_SUPPLY")
        original_max_velocities = [
            sim.getObjectFloatParam(joint, sim.jointfloatparam_maxvel)
            for joint in joints
        ]
        max_velocity = math.radians(max(60.0, self.speed_deg_s * 1.35))
        for joint in joints:
            sim.setObjectFloatParam(joint, sim.jointfloatparam_maxvel, max_velocity)

        command_script = self._create_command_script(robot)
        runner: Optional[_SmoothRunner] = None
        attached_payload: Optional[int] = None
        started = False
        succeeded = False
        transfer_speed = math.radians(self.speed_deg_s)
        descent_speed = math.radians(
            min(self.speed_deg_s * 0.75, DESCENT_SPEED_CAP_DEG_S)
        )
        terminal_visual_offset = self._terminal_visual_offset()
        try:
            if not self.bridge.start_simulation():
                raise RuntimeError(self.bridge.last_error or "cannot start simulation")
            started = True
            runner = _SmoothRunner(
                self.bridge,
                robot,
                joints,
                command_script,
                box,
                terminal,
                self.collision_check_interval,
                self.workspace_check_interval,
            )
            if prepared_mode:
                runner.step("R1 runtime bridge initialized", force_full=True)
            if not prepared_mode:
                runner.hold(0.5, "startup")
            if action != R1_TERMINAL_PLACED and not self._ready_gripper_open:
                if not self.bridge.set_gripper("R1", True):
                    raise RuntimeError(
                        self.bridge.last_error or "cannot open R1 gripper"
                    )
                runner.hold(0.8, "open gripper")
                self._freeze_gripper()

            if action in {R1_BOX_PLACED, R1_COMPLETE_CYCLE}:
                runner.execute_path(
                    "initial_to_box_pick_app",
                    paths["initial_to_box_pick_app"],
                    transfer_speed,
                )
                runner.hold(self.hold_seconds, "hold above box")
                runner.execute_path(
                    "descend_to_box_pick_tcp", paths["box_descend"], descent_speed
                )
                if not self.bridge.set_gripper("R1", False):
                    raise RuntimeError(
                        self.bridge.last_error or "cannot close R1 gripper"
                    )
                runner.hold(0.8, "close gripper")
                self._freeze_gripper()
                runner.attach_visual_payload(
                    "BOX_BLANK", box, BOX_VISUAL_OFFSET_Z, "box"
                )
                attached_payload = box
                with self.assembly_lock:
                    runner.execute_path(
                        "box_lift_and_transfer",
                        paths["box_lift_and_transfer"],
                        transfer_speed,
                    )
                    runner.hold(self.hold_seconds, "hold above box place")
                    runner.execute_path(
                        "descend_to_box_place_tcp",
                        paths["box_place_descend"],
                        descent_speed,
                    )
                    if not self.bridge.set_gripper("R1", True):
                        raise RuntimeError(
                            self.bridge.last_error or "cannot open R1 gripper"
                        )
                    runner.hold(0.8, "release box")
                    self._freeze_gripper()
                    runner.detach_payload("BOX_BLANK", box, True)
                    attached_payload = None
                    runner.execute_path(
                        "box_retreat_and_terminal_approach",
                        paths["box_retreat_and_terminal_approach"],
                        transfer_speed,
                    )
                runner.set_payload(None)
                runner.step("box release clearance complete", force_full=True)

            if action in {R1_TERMINAL_PLACED, R1_COMPLETE_CYCLE}:
                runner.hold(self.hold_seconds, "hold above terminal")
                runner.execute_path(
                    "descend_to_terminal_pick_tcp",
                    paths["terminal_descend"],
                    descent_speed,
                )
                if not self.bridge.set_gripper("R1", False):
                    raise RuntimeError(
                        self.bridge.last_error or "cannot close R1 gripper"
                    )
                runner.hold(0.8, "close gripper")
                self._freeze_gripper()
                runner.attach_visual_payload(
                    "TERMINAL_BLOCK_SUPPLY",
                    terminal,
                    terminal_visual_offset,
                    "terminal",
                )
                attached_payload = terminal
                with self.assembly_lock:
                    runner.execute_path(
                        "terminal_lift_and_transfer",
                        paths["terminal_lift_and_transfer"],
                        transfer_speed,
                    )
                    runner.hold(self.hold_seconds, "hold above terminal place")
                    runner.execute_path(
                        "descend_to_terminal_place_tcp",
                        paths["terminal_place_descend"],
                        descent_speed,
                    )
                    if not self.bridge.set_gripper("R1", True):
                        raise RuntimeError(
                            self.bridge.last_error or "cannot open R1 gripper"
                        )
                    runner.hold(0.8, "release terminal")
                    self._freeze_gripper()
                    runner.detach_payload(
                        "TERMINAL_BLOCK_SUPPLY", terminal, True
                    )
                    attached_payload = None
                    runner.execute_path(
                        "retreat_and_return_home",
                        paths["return_home"],
                        transfer_speed,
                    )
                runner.set_payload(None)
                runner.step("terminal release clearance complete", force_full=True)
                runner.hold(0.4, "final home hold")

            final_joints = runner.joint_positions()
            result = {
                "action": action,
                "visual_grasp_only": True,
                "final_joint_positions_deg": [
                    round(math.degrees(value), 6) for value in final_joints
                ],
                "box_position": [
                    round(float(value), 6)
                    for value in sim.getObjectPosition(box, -1)
                ],
                "terminal_position": [
                    round(float(value), 6)
                    for value in sim.getObjectPosition(terminal, -1)
                ],
                "terminal_visual_offset_m": terminal_visual_offset,
            }
            succeeded = True
            return result
        except Exception:
            if attached_payload is not None:
                try:
                    self.bridge.detach_object(attached_payload)
                except Exception:
                    pass
            raise
        finally:
            if runner is not None:
                runner.close()
            if succeeded:
                sim.removeObjects([command_script])
                for joint, original in zip(joints, original_max_velocities):
                    sim.setObjectFloatParam(
                        joint, sim.jointfloatparam_maxvel, original
                    )
                # Preserve the workpiece and joint state for the next robot
                # task while allowing the simulator to run normally.
                if not self._continuous_stepping:
                    self.bridge.set_stepping(False)
            else:
                if started and sim.getSimulationState() != sim.simulation_stopped:
                    self.bridge.stop_simulation()
                try:
                    sim.removeObjects([command_script])
                except Exception:
                    pass
                for joint, original in zip(joints, original_max_velocities):
                    try:
                        sim.setObjectFloatParam(
                            joint, sim.jointfloatparam_maxvel, original
                        )
                    except Exception:
                        pass


def expected_endpoint(plan: dict[str, Any], action: str) -> list[float]:
    """Return the planned final joint state for state reporting and tests."""
    if action == R1_BOX_PLACED:
        return list(plan["paths"]["box_retreat_and_terminal_approach"][-1])
    if action in {R1_TERMINAL_PLACED, R1_COMPLETE_CYCLE}:
        return list(plan["paths"]["return_home"][-1])
    raise ValueError(f"unsupported R1 action: {action}")
