"""R5 visual good/defect sorting for the current five-CR5A scene.

Both branches replay RViz/MoveIt taught native-gripper trajectories while
keeping the seven Git targets unchanged.  A runtime 100 mm virtual TCP carries
the inspection template through release; the saved scene and target Dummies
are never modified.
"""

from __future__ import annotations

import json
import math
import threading
from pathlib import Path
from typing import Any, Optional

from robot_control.runtime_cartesian import (
    RobotSafetyGuard,
    SmoothRunner,
    build_pick_place_paths,
    build_tip_translation_path,
    create_command_script,
    create_pose_dummy,
    create_virtual_tcp,
    find_unique_alias,
    generate_cartesian_path,
    interpolate_joint_line,
    join_paths,
    near,
    remove_runtime_objects,
    sha256_file,
    solve_target,
    tip_pose_for_config,
)
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import PARTS, WORKSPACES


R5_SORT_GOOD_DONE = "R5_SORT_GOOD_DONE"
R5_SORT_DEFECT_DONE = "R5_SORT_DEFECT_DONE"
R5_ACTIONS = frozenset({R5_SORT_GOOD_DONE, R5_SORT_DEFECT_DONE})
R5_WAIT_POINT = "R5_WAIT_POINT"

PLAN_VERSION = 1
PLAN_PATH = Path(__file__).with_name("plans") / "r5_sort_cycle_plan.json"
ROBOT_ID = "R5"
SCENE_NAME = "compact_cell.ttt"
TARGET_NAMES = (
    "R5_HOME_REF",
    "R5_PRODUCT_PICK_APP",
    "R5_PRODUCT_PICK_TCP",
    "R5_GOOD_PLACE_APP",
    "R5_GOOD_PLACE_TCP",
    "R5_DEFECT_PLACE_APP",
    "R5_DEFECT_PLACE_TCP",
)
PROTECTED_TARGETS = {
    "R5_HOME_REF": [0.35, -0.45, 0.70],
    "R5_PRODUCT_PICK_APP": [-0.04, 0.00, 0.432],
    "R5_PRODUCT_PICK_TCP": [-0.04, 0.00, 0.252],
    "R5_GOOD_PLACE_APP": [0.85, -1.10, 0.432],
    "R5_GOOD_PLACE_TCP": [0.85, -1.10, 0.252],
    "R5_DEFECT_PLACE_APP": [-0.15, -1.12, 0.432],
    "R5_DEFECT_PLACE_TCP": [-0.15, -1.12, 0.252],
}

PICK_ORIENTATION_DEG = (195.0, -45.0, 0.0)
VIRTUAL_TCP_OFFSET_M = 0.100
BELT_HEIGHT_CORRECTION_M = 0.0
DEFECT_TRANSFER_WAYPOINT = (-0.15, -0.15, 0.65)
GOOD_RUNTIME_XY_OFFSET_M = (-0.010, 0.020)
GOOD_BASE_TURN_DELTA_DEG = -121.0
GOOD_PREALIGN_ORIENTATION_DEG = (-143.152079, 31.403342, 104.057370)
GOOD_PLACE_ORIENTATION_DEG = (-134.007027, 10.545291, 79.271417)
GOOD_RELEASE_REACHABLE_ORIENTATION_DEG = (
    -173.156653,
    31.407957,
    119.060060,
)
GOOD_TARGET_PRODUCT_YAW_DEG = -90.0
DEFECT_TARGET_PRODUCT_YAW_DEG = 0.0
GOOD_TRANSFER_JOINT6_PRETURN_DEG = 0.0
# The release payload must remain rigidly attached to the taught TCP path.
# Conveyor clearance is handled by the taught APP->TCP route and target
# alignment, never by translating the product independently in world space.
GOOD_RELEASE_CENTERING_OFFSET_M = (0.0, 0.0, 0.0)
GOOD_RELEASE_CENTER_TOLERANCE_M = 0.003
GOOD_BELT_AXIS_ALIGNMENT_TOLERANCE_DEG = 1.0
LEVEL_PAYLOAD_TOLERANCE_DEG = 1.0
GOOD_TRANSFER_HEIGHT_M = 0.760
PICK_ENTRY_HEIGHT_M = 0.620
ACTUAL_GRIPPER_CLEARANCE_Z_M = 0.470
GOOD_BASE_TURN_POINTS = 121
GOOD_TRANSFER_POINTS = 61
GOOD_APP_POINTS = 81
GOOD_ALIGN_POINTS = 101
BELT_LOWER_POINTS = 15
BELT_ALIASES = {
    R5_SORT_GOOD_DONE: "Good_Conveyor_Belt_Black",
    R5_SORT_DEFECT_DONE: "Defect_Conveyor_Belt_Black",
}

INSPECTION_PRODUCT_POSITION = (-0.04, 0.00, 0.216)
PRODUCT_GRASP_POSITION = (-0.04, 0.00, 0.280)
GOOD_RELEASE_PRODUCT_POSITION = (0.98, -1.06, 0.270)
DEFECT_RELEASE_PRODUCT_POSITION = (-0.15, -1.12, 0.270)
POSITION_TOLERANCE_M = 0.003
TARGET_TOLERANCE = 1e-6
JOINT_TOLERANCE_DEG = 0.30
TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_CAP_DEG_S = 36.0
HOLD_SECONDS = 0.8
WAIT_TO_PICK_APP_START_DELAY_S = 0.4
CONVEYOR_ENTRY_CLEARANCE_SECONDS = 1.25
MAX_UNWRAPPED_JOINT_RAD = 2.0 * math.pi + 1e-6
MAX_PATH_BOUNDARY_JUMP_RAD = math.radians(0.5)

MANUAL_REQUIRED_PATHS = (
    "initial_to_pick_app",
    "pick_descend",
    "lift_and_transfer",
    "place_descend",
    "return_home",
)
WAIT_PREPOSITION_PATHS = (
    "home_to_wait",
    "wait_to_pick_app",
)
PREPARED_PATHS = MANUAL_REQUIRED_PATHS + WAIT_PREPOSITION_PATHS

CAMERA_VIEW_PATH = (
    "/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area"
)
RUNTIME_PREFIX = "R5_Runtime_"
RUNTIME_TCP_ALIAS = f"{RUNTIME_PREFIX}Vacuum_TCP"
RUNTIME_BRIDGE_ALIAS = f"{RUNTIME_PREFIX}Command_Bridge"


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


def load_r5_plan(path: Path = PLAN_PATH) -> dict[str, Any]:
    """Load and structurally validate the R5 native-gripper replay plan."""
    try:
        plan = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot load R5 plan {path}: {exc}") from exc

    if plan.get("plan_version") != PLAN_VERSION:
        raise RuntimeError(f"unsupported R5 plan version in {path}")
    if plan.get("robot_id") != ROBOT_ID:
        raise RuntimeError("R5 plan robot_id does not match")
    if plan.get("tool") != "native_wide_gripper_with_runtime_virtual_tcp":
        raise RuntimeError("R5 plan is not for the native wide gripper")
    if plan.get("tip_link") != "R5_gripper_tip":
        raise RuntimeError("R5 plan does not use R5_gripper_tip")
    if plan.get("protected_targets_modified") is not False:
        raise RuntimeError("R5 plan does not preserve the protected Git targets")

    protected = plan.get("protected_targets")
    if not isinstance(protected, dict) or set(protected) != set(TARGET_NAMES):
        raise RuntimeError("R5 plan target snapshot is incomplete")
    for name, expected_position in PROTECTED_TARGETS.items():
        actual = protected.get(name, {})
        if not near(
            actual.get("position", []),
            expected_position,
            TARGET_TOLERANCE,
        ):
            raise RuntimeError(f"R5 plan target position differs: {name}")
        if not near(
            actual.get("orientation_euler", []),
            [0.0, 0.0, 0.0],
            TARGET_TOLERANCE,
        ):
            raise RuntimeError(f"R5 plan target orientation differs: {name}")

    paths = plan.get("paths")
    if not isinstance(paths, dict):
        raise RuntimeError("R5 plan has no paths")
    for action in R5_ACTIONS:
        action_paths = paths.get(action)
        if not isinstance(action_paths, dict):
            raise RuntimeError(f"R5 plan has no paths for {action}")
        for name in PREPARED_PATHS:
            if not _finite_joint_path(action_paths.get(name)):
                raise RuntimeError(f"R5 plan path is invalid: {action}.{name}")
            if _path_has_invalid_joint_branch(action_paths[name]):
                raise RuntimeError(
                    f"R5 plan uses an invalid joint branch: {action}.{name}"
                )
        for first_name, second_name in (
            ("home_to_wait", "wait_to_pick_app"),
            ("wait_to_pick_app", "pick_descend"),
            ("initial_to_pick_app", "pick_descend"),
            ("pick_descend", "lift_and_transfer"),
            ("lift_and_transfer", "place_descend"),
            ("place_descend", "return_home"),
        ):
            if (
                _max_joint_gap(
                    action_paths[first_name][-1],
                    action_paths[second_name][0],
                )
                > MAX_PATH_BOUNDARY_JUMP_RAD
            ):
                raise RuntimeError(
                    "R5 plan path boundary jumps: "
                    f"{action}.{first_name} -> {second_name}"
                )
        if (
            _max_joint_gap(action_paths["home_to_wait"][0], [0.0] * 6)
            > MAX_PATH_BOUNDARY_JUMP_RAD
        ):
            raise RuntimeError(f"R5 plan {action} home_to_wait does not start home")
        if (
            _max_joint_gap(
                action_paths["initial_to_pick_app"][0],
                action_paths["home_to_wait"][0],
            )
            > MAX_PATH_BOUNDARY_JUMP_RAD
        ):
            raise RuntimeError(
                f"R5 plan {action} integrated entry does not start at home"
            )
        if (
            _max_joint_gap(action_paths["return_home"][-1], [0.0] * 6)
            > MAX_PATH_BOUNDARY_JUMP_RAD
        ):
            raise RuntimeError(f"R5 plan {action} does not return home")

    workspace = plan.get("workspace", {})
    expected_workspace = WORKSPACES["R5"]
    if tuple(workspace.get("lower", ())) != tuple(expected_workspace["lower"]):
        raise RuntimeError("R5 plan lower workspace wall differs from the contract")
    if tuple(workspace.get("upper", ())) != tuple(expected_workspace["upper"]):
        raise RuntimeError("R5 plan upper workspace wall differs from the contract")

    validation = plan.get("validation", {})
    fingerprint = validation.get("scene_fingerprint", {})
    if not isinstance(fingerprint.get("sha256"), str) or not isinstance(
        fingerprint.get("size"), int
    ):
        raise RuntimeError("R5 plan has no validated scene fingerprint")
    return plan


class R5MotionController:
    """Execute the two explicit visual quality-sorting branches."""

    def __init__(
        self,
        bridge: SimBridge,
        r1_plan_path: Path | None = None,
        inspection_lock: Optional[threading.Lock] = None,
        speed_deg_s: float = TRANSFER_SPEED_DEG_S,
        hold_seconds: float = HOLD_SECONDS,
        collision_check_interval: int = 5,
        workspace_check_interval: int = 20,
    ):
        if speed_deg_s <= 0.0:
            raise ValueError("speed_deg_s must be positive")
        _ = r1_plan_path
        self.bridge = bridge
        self.inspection_lock = inspection_lock or threading.Lock()
        self.speed_deg_s = float(speed_deg_s)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.collision_check_interval = int(collision_check_interval)
        self.workspace_check_interval = int(workspace_check_interval)
        self._prepared_paths: dict[
            str, dict[str, list[list[float]]]
        ] = {}
        self._prepared_transfer_waypoints: dict[str, list[float]] = {}
        self._prepared_grasp_paths: dict[str, list[list[float]]] = {}
        self._pre_positioned_config: dict[str, list[float]] = {}
        self._continuous_stepping = False

    def _target_snapshot(self) -> dict[str, dict[str, list[float]]]:
        result = {}
        for name in TARGET_NAMES:
            pose = self.bridge.get_target_pose(name)
            result[name] = {
                "position": [round(float(value), 9) for value in pose["position"]],
                "orientation": [
                    round(float(value), 9) for value in pose["orientation"]
                ],
            }
        return result

    def _validate_static(self) -> None:
        plan = load_r5_plan()
        scene = Path(self.bridge.scene_path())
        if scene.name != SCENE_NAME:
            raise RuntimeError(f"unexpected CoppeliaSim scene: {scene}")
        fingerprint = plan["validation"]["scene_fingerprint"]
        if scene.stat().st_size != int(fingerprint.get("size", -1)):
            raise RuntimeError("R5 scene size differs; repeat full preflight")
        if sha256_file(scene) != fingerprint["sha256"]:
            raise RuntimeError("R5 scene hash differs; repeat full preflight")

        snapshot = self._target_snapshot()
        for name, expected_position in PROTECTED_TARGETS.items():
            actual = snapshot[name]
            if not near(actual["position"], expected_position, TARGET_TOLERANCE):
                raise RuntimeError(f"protected Git target changed: {name}")
            if not near(actual["orientation"], [0.0, 0.0, 0.0], TARGET_TOLERANCE):
                raise RuntimeError(f"protected Git target orientation changed: {name}")

    def _validate_preflight(self, action: str, verify_static: bool = True) -> None:
        sim = self.bridge.sim
        if sim.getSimulationState() == sim.simulation_stopped:
            raise RuntimeError("R5 sorting requires the running coordinated scene")
        if verify_static:
            self._validate_static()
        actual_r5 = self.bridge.get_robot_joint_positions(ROBOT_ID)
        expected_r5 = self._pre_positioned_config.get(action, [0.0] * 6)
        accepted_starts = [expected_r5]
        if action in self._pre_positioned_config:
            # The front-half runner reports the wait point to the executor,
            # but a failed or interrupted handoff can leave the physical arm
            # at another known endpoint. Accept only the plan's HOME, WAIT,
            # or PICK_APP configurations; unknown states still fail closed.
            try:
                planned = load_r5_plan()["paths"][action]
                accepted_starts.extend(
                    [
                        [0.0] * 6,
                        planned["home_to_wait"][-1],
                        planned["initial_to_pick_app"][-1],
                    ]
                )
            except Exception:
                pass
        if not any(
            near(actual_r5, candidate, math.radians(JOINT_TOLERANCE_DEG))
            for candidate in accepted_starts
        ):
            raise RuntimeError(
                "R5 is not at the validated start "
                f"(pre-positioned={action in self._pre_positioned_config})"
            )

        product = self.bridge.get_object_handle("INSPECTION_PRODUCT")
        parts = sim.getObject(PARTS["INSPECTION_PRODUCT"].rsplit("/", 1)[0])
        if sim.getObjectParent(product) != parts:
            raise RuntimeError("INSPECTION_PRODUCT is not owned by /Parts")
        position = [float(value) for value in sim.getObjectPosition(product, -1)]
        if not near(
            position, INSPECTION_PRODUCT_POSITION, POSITION_TOLERANCE_M
        ):
            raise RuntimeError(
                f"inspection product is not at the validated position: {position}"
            )
        product_shapes = sim.getObjectsInTree(
            product, sim.object_shape_type, 0
        )
        visible = sum(
            sim.getObjectInt32Param(
                handle, sim.objintparam_visibility_layer
            )
            != 0
            for handle in product_shapes
        )
        if visible != len(product_shapes):
            raise RuntimeError("inspection product is not fully visible")

    def _positions(self, action: str) -> tuple[list[float], ...]:
        names = (
            "R5_PRODUCT_PICK_APP",
            "R5_PRODUCT_PICK_TCP",
            "R5_GOOD_PLACE_APP"
            if action == R5_SORT_GOOD_DONE
            else "R5_DEFECT_PLACE_APP",
            "R5_GOOD_PLACE_TCP"
            if action == R5_SORT_GOOD_DONE
            else "R5_DEFECT_PLACE_TCP",
        )
        positions = tuple(
            list(self.bridge.get_target_pose(name)["position"]) for name in names
        )
        if action == R5_SORT_GOOD_DONE:
            for position in positions[2:]:
                position[0] += GOOD_RUNTIME_XY_OFFSET_M[0]
                position[1] += GOOD_RUNTIME_XY_OFFSET_M[1]
        return positions

    @staticmethod
    def _belt_axis_alignment_error_deg(actual: float, target: float) -> float:
        """Accept either longitudinal or transverse placement on the belt."""
        parallel = abs((actual - target + 90.0) % 180.0 - 90.0)
        transverse = abs((actual - target) % 180.0 - 90.0)
        return min(parallel, transverse)

    @staticmethod
    def _target_product_yaw_deg(action: str) -> float:
        return (
            GOOD_TARGET_PRODUCT_YAW_DEG
            if action == R5_SORT_GOOD_DONE
            else DEFECT_TARGET_PRODUCT_YAW_DEG
        )

    @staticmethod
    def _level_error_deg(orientation_deg: list[float]) -> float:
        return max(abs(orientation_deg[0]), abs(orientation_deg[1]))

    @staticmethod
    def _pose_max_error(first: list[float], second: list[float]) -> float:
        if len(first) != 7 or len(second) != 7:
            raise ValueError("rigid payload poses must contain seven values")
        return max(abs(before - after) for before, after in zip(first, second))

    @staticmethod
    def _check_released_product_environment(sim: Any, product: int) -> None:
        product_shapes = set(
            sim.getObjectsInTree(product, sim.object_shape_type, 0)
        )
        allowed_aliases = {"Good_Conveyor_Belt_Black", "Camera_View_Area"}
        payload_collection = sim.createCollection(1)
        environment = sim.createCollection(1)
        try:
            for handle in product_shapes:
                sim.addItemToCollection(
                    payload_collection, sim.handle_single, handle, 0
                )
            for handle in sim.getObjectsInTree(
                sim.handle_scene, sim.object_shape_type, 0
            ):
                if handle in product_shapes:
                    continue
                if sim.getObjectAlias(handle) in allowed_aliases:
                    continue
                if sim.getObjectInt32Param(
                    handle, sim.objintparam_visibility_layer
                ) == 0:
                    continue
                sim.addItemToCollection(
                    environment, sim.handle_single, handle, 0
                )
            state, pair = sim.checkCollision(payload_collection, environment)
            if state:
                paths = [sim.getObjectAlias(handle, 1) for handle in pair]
                raise RuntimeError(
                    f"released good product collision at belt entry: {paths}"
                )
        finally:
            sim.destroyCollection(payload_collection)
            sim.destroyCollection(environment)

    @staticmethod
    def _copy_paths(
        paths: dict[str, list[list[float]]],
    ) -> dict[str, list[list[float]]]:
        return {
            name: [list(config) for config in path]
            for name, path in paths.items()
        }

    @staticmethod
    def _apply_good_transfer_joint6_preturn(
        paths: dict[str, list[list[float]]],
    ) -> dict[str, list[list[float]]]:
        adjusted = R5MotionController._copy_paths(paths)
        preturn = math.radians(GOOD_TRANSFER_JOINT6_PRETURN_DEG)

        transfer = adjusted["lift_and_transfer"]
        for index, config in enumerate(transfer):
            fraction = index / (len(transfer) - 1)
            config[5] -= preturn * fraction

        place = adjusted["place_descend"]
        for index, config in enumerate(place):
            fraction = index / (len(place) - 1)
            config[5] -= preturn * (1.0 - fraction)

        return adjusted

    @staticmethod
    def _good_transfer_preturn_orientation(
        target_level_orientation: list[float],
        adjusted_place_descend: list[list[float]],
    ) -> list[float]:
        remaining_joint6_delta = abs(
            adjusted_place_descend[-1][5] - adjusted_place_descend[0][5]
        )
        preturn = math.radians(GOOD_TRANSFER_JOINT6_PRETURN_DEG)
        if preturn <= 0.0:
            return [0.0, 0.0, 0.0]
        fraction = preturn / (preturn + remaining_joint6_delta)
        return [
            0.0,
            0.0,
            target_level_orientation[2] * fraction,
        ]

    @staticmethod
    def _defect_waypoints() -> list[dict[str, Any]]:
        return [
            {
                "name": "defect private transfer",
                "position": list(DEFECT_TRANSFER_WAYPOINT),
                "orientation_deg": PICK_ORIENTATION_DEG,
                "points": 81,
            },
        ]

    @staticmethod
    def _build_segment(
        sim_ik: Any,
        base: int,
        tip: int,
        joints: list[int],
        target: int,
        start: list[float],
        points: int,
        label: str,
    ) -> list[list[float]]:
        solve_target(sim_ik, base, tip, joints, target, start, label)
        return generate_cartesian_path(
            sim_ik,
            base,
            tip,
            joints,
            target,
            start,
            points,
            label,
        )

    @staticmethod
    def _good_transfer_position(
        sim: Any,
        robot: int,
        pick_app_position: list[float],
        place_app_position: list[float],
    ) -> list[float]:
        base_position = sim.getObjectPosition(robot, -1)
        delta = math.radians(GOOD_BASE_TURN_DELTA_DEG)
        offset_x = pick_app_position[0] - base_position[0]
        offset_y = pick_app_position[1] - base_position[1]
        turned_x = (
            base_position[0]
            + math.cos(delta) * offset_x
            - math.sin(delta) * offset_y
        )
        turned_y = (
            base_position[1]
            + math.sin(delta) * offset_x
            + math.cos(delta) * offset_y
        )
        return [
            0.5 * (turned_x + place_app_position[0]),
            0.5 * (turned_y + place_app_position[1]),
            GOOD_TRANSFER_HEIGHT_M,
        ]

    def _build_good_paths(
        self,
        sim: Any,
        sim_ik: Any,
        robot: int,
        virtual_tip: int,
        joints: list[int],
        positions: tuple[list[float], ...],
    ) -> tuple[dict[str, list[list[float]]], list[float]]:
        base = find_unique_alias(sim, robot, "base_link_respondable")
        transfer_position = self._good_transfer_position(
            sim, robot, positions[0], positions[2]
        )
        release_position = list(positions[3])
        release_position[2] += BELT_HEIGHT_CORRECTION_M
        specifications = (
            (
                "Pick_APP",
                positions[0],
                PICK_ORIENTATION_DEG,
            ),
            (
                "Pick_TCP",
                positions[1],
                PICK_ORIENTATION_DEG,
            ),
            (
                "Good_High",
                transfer_position,
                GOOD_PREALIGN_ORIENTATION_DEG,
            ),
            (
                "Good_APP",
                positions[2],
                GOOD_PREALIGN_ORIENTATION_DEG,
            ),
            (
                "Good_TCP",
                positions[3],
                GOOD_PLACE_ORIENTATION_DEG,
            ),
            (
                "Good_Release",
                release_position,
                GOOD_PLACE_ORIENTATION_DEG,
            ),
        )
        targets: list[int] = []
        try:
            for name, position, orientation in specifications:
                targets.append(
                    create_pose_dummy(
                        sim,
                        f"{RUNTIME_PREFIX}Target_{name}",
                        position,
                        orientation,
                    )
                )
            (
                pick_app_target,
                pick_tcp_target,
                high_target,
                place_app_target,
                place_tcp_target,
                release_target,
            ) = targets
            pick_app_config = solve_target(
                sim_ik, base, virtual_tip, joints, pick_app_target,
                [0.0] * 6, "R5 good pick APP",
            )
            pick_descend = generate_cartesian_path(
                sim_ik,
                base,
                virtual_tip,
                joints,
                pick_tcp_target,
                pick_app_config,
                51,
                "R5 good pick descent",
            )
            turned_config = list(pick_app_config)
            turned_config[0] += math.radians(GOOD_BASE_TURN_DELTA_DEG)
            base_turn = interpolate_joint_line(
                pick_app_config, turned_config, GOOD_BASE_TURN_POINTS
            )
            high_segment = self._build_segment(
                sim_ik,
                base,
                virtual_tip,
                joints,
                high_target,
                turned_config,
                GOOD_TRANSFER_POINTS,
                "R5 good high transfer",
            )
            app_segment = self._build_segment(
                sim_ik,
                base,
                virtual_tip,
                joints,
                place_app_target,
                high_segment[-1],
                GOOD_APP_POINTS,
                "R5 good place APP",
            )
            aligned_config = solve_target(
                sim_ik,
                base,
                virtual_tip,
                joints,
                place_tcp_target,
                app_segment[-1],
                "R5 good aligned TCP",
            )
            place_descend = interpolate_joint_line(
                app_segment[-1], aligned_config, GOOD_ALIGN_POINTS
            )
            belt_lower = self._build_segment(
                sim_ik,
                base,
                virtual_tip,
                joints,
                release_target,
                place_descend[-1],
                BELT_LOWER_POINTS,
                "R5 good rigid belt lower",
            )
            initial = interpolate_joint_line([0.0] * 6, pick_app_config, 101)
            transfer = join_paths(base_turn, high_segment, app_segment)
            return_home = join_paths(
                list(reversed(belt_lower)),
                list(reversed(place_descend)),
                list(reversed(app_segment)),
                list(reversed(high_segment)),
                list(reversed(base_turn)),
                list(reversed(initial)),
            )
            return (
                {
                    "initial_to_pick_app": initial,
                    "pick_descend": pick_descend,
                    "lift_and_transfer": join_paths(
                        list(reversed(pick_descend)), transfer
                    ),
                    "place_descend": place_descend,
                    "belt_lower": belt_lower,
                    "return_home": return_home,
                },
                transfer_position,
            )
        finally:
            for handle in reversed(targets):
                try:
                    sim.removeObjects([handle])
                except Exception:
                    pass

    def _build_defect_paths(
        self,
        sim: Any,
        sim_ik: Any,
        robot: int,
        virtual_tip: int,
        joints: list[int],
        positions: tuple[list[float], ...],
    ) -> dict[str, list[list[float]]]:
        paths = build_pick_place_paths(
            sim,
            sim_ik,
            robot,
            virtual_tip,
            joints,
            RUNTIME_PREFIX,
            positions[0],
            positions[1],
            positions[2],
            positions[3],
            PICK_ORIENTATION_DEG,
            PICK_ORIENTATION_DEG,
            self._defect_waypoints(),
            pick_entry_position=(
                positions[0][0],
                positions[0][1],
                PICK_ENTRY_HEIGHT_M,
            ),
        )
        release_position = list(positions[3])
        release_position[2] += BELT_HEIGHT_CORRECTION_M
        release_target = create_pose_dummy(
            sim,
            f"{RUNTIME_PREFIX}Target_Defect_Release",
            release_position,
            PICK_ORIENTATION_DEG,
        )
        try:
            base = find_unique_alias(sim, robot, "base_link_respondable")
            belt_lower = self._build_segment(
                sim_ik,
                base,
                virtual_tip,
                joints,
                release_target,
                paths["place_descend"][-1],
                BELT_LOWER_POINTS,
                "R5 defect rigid belt lower",
            )
        finally:
            sim.removeObjects([release_target])
        paths["belt_lower"] = belt_lower
        paths["return_home"] = join_paths(
            list(reversed(belt_lower)), paths["return_home"]
        )
        return paths

    def prepare(self, action: str) -> dict[str, Any]:
        if action not in R5_ACTIONS:
            raise ValueError(f"unsupported R5 action: {action}")
        sim = self.bridge.sim
        if sim.getSimulationState() != sim.simulation_stopped:
            raise RuntimeError("R5 preparation requires a stopped scene")
        self._validate_static()
        if not near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            [0.0] * 6,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError("R5 is not zero during preparation")
        plan = load_r5_plan()
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        robot = self.bridge.get_object_handle(ROBOT_ID)
        joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
        sim_ik = client.require("simIK")
        virtual_tip = create_virtual_tcp(
            sim, robot, f"{RUNTIME_PREFIX}Prepare_TCP", VIRTUAL_TCP_OFFSET_M
        )
        planned = plan["paths"][action]
        try:
            positions = self._positions(action)
            if action == R5_SORT_GOOD_DONE:
                prepared_paths, transfer_waypoint = self._build_good_paths(
                    sim, sim_ik, robot, virtual_tip, joints, positions
                )
            else:
                prepared_paths = self._build_defect_paths(
                    sim, sim_ik, robot, virtual_tip, joints, positions
                )
                transfer_waypoint = list(DEFECT_TRANSFER_WAYPOINT)
        finally:
            sim.removeObjects([virtual_tip])
        home_to_wait = [
            [float(joint) for joint in config]
            for config in planned["home_to_wait"]
        ]
        prepared_paths["home_to_wait"] = home_to_wait
        prepared_paths["wait_to_pick_app"] = join_paths(
            list(reversed(home_to_wait)),
            prepared_paths["initial_to_pick_app"],
        )
        visible_tip = find_unique_alias(sim, robot, "R5_gripper_tip")
        grasp_path = build_tip_translation_path(
            sim,
            client.require("simIK"),
            robot,
            visible_tip,
            joints,
            prepared_paths["pick_descend"][-1],
            PRODUCT_GRASP_POSITION,
            25,
            f"R5_{action}_grasp_contact",
        )
        if action == R5_SORT_GOOD_DONE:
            contact_config = list(grasp_path[-1])
            clearance_pose = tip_pose_for_config(
                sim, visible_tip, joints, contact_config
            )
            clearance_pose[2] = ACTUAL_GRIPPER_CLEARANCE_Z_M
            clearance_target = sim.createDummy(0.004)
            sim.setObjectAlias(
                clearance_target, f"{RUNTIME_PREFIX}Actual_Grip_Clearance"
            )
            sim.setObjectPose(clearance_target, -1, clearance_pose)
            sim.setObjectInt32Param(
                clearance_target, sim.objintparam_visibility_layer, 0
            )
            try:
                base = find_unique_alias(
                    sim, robot, "base_link_respondable"
                )
                contact_to_clearance = generate_cartesian_path(
                    sim_ik,
                    base,
                    visible_tip,
                    joints,
                    clearance_target,
                    contact_config,
                    81,
                    "R5 actual gripper vertical pickup clearance",
                )
            finally:
                sim.removeObjects([clearance_target])

            clearance_config = contact_to_clearance[-1]
            initial = interpolate_joint_line(
                [0.0] * 6, clearance_config, 121
            )
            old_lift = prepared_paths["lift_and_transfer"]
            app_segment = [
                list(config) for config in old_lift[-GOOD_APP_POINTS:]
            ]
            high_transition = interpolate_joint_line(
                clearance_config, app_segment[0], 121
            )
            prepared_paths["initial_to_pick_app"] = initial
            prepared_paths["pick_descend"] = list(
                reversed(contact_to_clearance)
            )
            prepared_paths["lift_and_transfer"] = join_paths(
                contact_to_clearance,
                high_transition,
                app_segment,
            )
            app_config = list(app_segment[-1])
            prepared_paths["place_descend"] = [
                app_config,
                list(app_config),
            ]
            prepared_paths.pop("belt_lower", None)
            prepared_paths["return_home"] = join_paths(
                list(reversed(app_segment)),
                list(reversed(high_transition)),
                list(reversed(initial)),
            )
            prepared_paths["wait_to_pick_app"] = join_paths(
                list(reversed(home_to_wait)), initial
            )
            grasp_path = [contact_config, list(contact_config)]

        self._prepared_paths[action] = prepared_paths
        self._prepared_transfer_waypoints[action] = list(transfer_waypoint)
        self._prepared_grasp_paths[action] = grasp_path
        return {
            "robot_id": ROBOT_ID,
            "prepared_actions": [action],
            "path_source": str(PLAN_PATH),
            "path_points": {
                name: len(path) for name, path in prepared_paths.items()
            },
            "transfer_waypoint": list(transfer_waypoint),
            "grasp_contact_points": len(grasp_path),
        }

    def _release_alignment_path(
        self,
        action: str,
        robot: int,
        tip: int,
        joints: list[int],
        product: int,
    ) -> list[list[float]]:
        sim = self.bridge.sim
        target_product = (
            GOOD_RELEASE_PRODUCT_POSITION
            if action == R5_SORT_GOOD_DONE
            else DEFECT_RELEASE_PRODUCT_POSITION
        )
        product_position = [
            float(value) for value in sim.getObjectPosition(product, -1)
        ]
        tip_position = [float(value) for value in sim.getObjectPosition(tip, -1)]
        target_tip = [
            tip_position[index] + target_product[index] - product_position[index]
            for index in range(3)
        ]
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        try:
            return build_tip_translation_path(
                sim,
                client.require("simIK"),
                robot,
                tip,
                joints,
                self.bridge.get_robot_joint_positions(ROBOT_ID),
                target_tip,
                25,
                f"R5_{action}_release_alignment",
            )
        except RuntimeError as exc:
            if action == R5_SORT_GOOD_DONE:
                target = create_pose_dummy(
                    sim,
                    f"{RUNTIME_PREFIX}Good_Release_Reachable",
                    (0.0, 0.0, 0.0),
                    GOOD_RELEASE_REACHABLE_ORIENTATION_DEG,
                )
                offset_probe = sim.createDummy(0.002)
                try:
                    local_product_position = [
                        float(value)
                        for value in sim.getObjectPosition(product, tip)
                    ]
                    sim.setObjectParent(offset_probe, target, False)
                    sim.setObjectPosition(
                        offset_probe, target, local_product_position
                    )
                    rotated_offset = [
                        float(value)
                        for value in sim.getObjectPosition(offset_probe, -1)
                    ]
                    rigid_target_tip = [
                        float(target_product[index]) - rotated_offset[index]
                        for index in range(3)
                    ]
                    sim.setObjectPosition(target, -1, rigid_target_tip)
                    start = self.bridge.get_robot_joint_positions(ROBOT_ID)
                    base = find_unique_alias(
                        sim, robot, "base_link_respondable"
                    )
                    solve_target(
                        client.require("simIK"),
                        base,
                        tip,
                        joints,
                        target,
                        start,
                        "R5 good reachable release alignment",
                    )
                    return generate_cartesian_path(
                        client.require("simIK"),
                        base,
                        tip,
                        joints,
                        target,
                        start,
                        41,
                        "R5 good reachable release alignment",
                    )
                except RuntimeError as fallback_exc:
                    exc = fallback_exc
                finally:
                    sim.removeObjects([offset_probe, target])
            raise RuntimeError(
                f"{exc}; product_position={product_position}; "
                f"target_product={list(target_product)}; "
                f"tip_position={tip_position}; target_tip={target_tip}"
            ) from exc

    def set_continuous_stepping(self, enabled: bool) -> None:
        self._continuous_stepping = bool(enabled)

    def set_pre_positioned(self, action: str, config: list[float]) -> None:
        if action not in R5_ACTIONS:
            raise ValueError(f"unsupported R5 action: {action}")
        self._pre_positioned_config[action] = list(config)

    def _startup_path(
        self,
        action: str,
        paths: dict[str, list[list[float]]],
        actual_config: Optional[list[float]] = None,
    ) -> tuple[str | None, list[list[float]] | None]:
        if actual_config is not None:
            tolerance = math.radians(JOINT_TOLERANCE_DEG)
            if near(actual_config, paths["initial_to_pick_app"][0], tolerance):
                return "R5 initial_to_pick_app", paths["initial_to_pick_app"]
            if near(actual_config, paths["home_to_wait"][-1], tolerance):
                return "R5 wait_to_pick_app", paths["wait_to_pick_app"]
            if near(actual_config, paths["initial_to_pick_app"][-1], tolerance):
                return None, None
            raise RuntimeError(
                "R5 is not at a known path entry: "
                f"joints_deg={[round(math.degrees(value), 3) for value in actual_config]}"
            )

        pre_positioned = self._pre_positioned_config.get(action)
        if pre_positioned is None:
            return "R5 initial_to_pick_app", paths["initial_to_pick_app"]

        tolerance = math.radians(JOINT_TOLERANCE_DEG)
        wait_config = paths["home_to_wait"][-1]
        pick_app_config = paths["initial_to_pick_app"][-1]
        if near(pre_positioned, wait_config, tolerance):
            return "R5 wait_to_pick_app", paths["wait_to_pick_app"]
        if near(pre_positioned, pick_app_config, tolerance):
            return None, None
        raise RuntimeError(
            "R5 pre-positioned config is neither the taught wait point nor "
            "R5_PRODUCT_PICK_APP"
        )

    @staticmethod
    def _merge_wait_paths(
        action: str,
        paths: dict[str, list[list[float]]],
    ) -> dict[str, list[list[float]]]:
        """Restore taught wait segments when paths were built dynamically.

        Dynamic IK planning builds the task-specific motion from HOME.  The
        coordinated front half may nevertheless leave R5 at its taught wait
        point, so the continuation must use the same cached wait-to-pick route
        as prepared execution.
        """
        if all(name in paths for name in WAIT_PREPOSITION_PATHS):
            return paths
        plan = load_r5_plan()
        planned = plan["paths"][action]
        merged = R5MotionController._copy_paths(paths)
        for name in WAIT_PREPOSITION_PATHS:
            merged[name] = [list(config) for config in planned[name]]
        return merged

    @staticmethod
    def _startup_delay_seconds(startup_label: str | None) -> float:
        if startup_label == "R5 wait_to_pick_app":
            return WAIT_TO_PICK_APP_START_DELAY_S
        return 0.0

    def execute(self, action: str) -> dict[str, Any]:
        if action not in R5_ACTIONS:
            raise ValueError(f"unsupported R5 action: {action}")
        prepared_paths = self._prepared_paths.get(action)
        prepared_mode = prepared_paths is not None
        self._validate_preflight(action, verify_static=not prepared_mode)

        sim = self.bridge.sim
        robot = -1
        virtual_tip = -1
        command_script = -1
        product = -1
        runner: Optional[SmoothRunner] = None
        joints: list[int] = []
        original_max_velocities: list[float] = []
        attached = False
        succeeded = False
        grasp_transform_max_error = 0.0
        release_product_orientation_deg: list[float] = []
        good_belt_axis_alignment_error_deg: Optional[float] = None
        release_detach_orientation_jump_deg: list[float] = []
        release_product_level_error_deg: Optional[float] = None
        manual_release_at_tcp = False
        place_descend_joint6_delta_deg = 0.0
        lift_transfer_joint6_delta_deg = 0.0
        good_transfer_preturn_orientation_deg: Optional[list[float]] = None
        good_release_centering_offset_m = [0.0, 0.0, 0.0]
        release_product_position_on_belt: list[float] = []
        try:
            self.bridge.set_stepping(True)
            robot = self.bridge.get_object_handle(ROBOT_ID)
            joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
            product = self.bridge.get_object_handle("INSPECTION_PRODUCT")
            remove_runtime_objects(sim, robot, RUNTIME_PREFIX)
            virtual_tip = create_virtual_tcp(
                sim, robot, RUNTIME_TCP_ALIAS, VIRTUAL_TCP_OFFSET_M
            )
            visible_tip = find_unique_alias(sim, robot, "R5_gripper_tip")

            positions = self._positions(action)
            if prepared_paths is not None:
                paths = self._copy_paths(prepared_paths)
                transfer_waypoint = self._prepared_transfer_waypoints[action]
                place_orientation = (
                    GOOD_PLACE_ORIENTATION_DEG
                    if action == R5_SORT_GOOD_DONE
                    else PICK_ORIENTATION_DEG
                )
            elif action == R5_SORT_GOOD_DONE:
                client = getattr(self.bridge, "_client", None)
                if client is None:
                    raise RuntimeError(
                        "CoppeliaSim remote client is unavailable"
                    )
                protected_before = self._target_snapshot()
                paths, transfer_waypoint = self._build_good_paths(
                    sim,
                    client.require("simIK"),
                    robot,
                    virtual_tip,
                    joints,
                    positions,
                )
                place_orientation = GOOD_PLACE_ORIENTATION_DEG
            else:
                client = getattr(self.bridge, "_client", None)
                if client is None:
                    raise RuntimeError(
                        "CoppeliaSim remote client is unavailable"
                    )
                protected_before = self._target_snapshot()
                paths = self._build_defect_paths(
                    sim,
                    client.require("simIK"),
                    robot,
                    virtual_tip,
                    joints,
                    positions,
                )
                transfer_waypoint = list(DEFECT_TRANSFER_WAYPOINT)
                place_orientation = PICK_ORIENTATION_DEG
            if not prepared_mode and self._target_snapshot() != protected_before:
                raise RuntimeError("R5 protected Git targets changed during planning")
            paths = self._merge_wait_paths(action, paths)
            if action == R5_SORT_GOOD_DONE:
                paths = self._apply_good_transfer_joint6_preturn(paths)

            original_max_velocities = [
                sim.getObjectFloatParam(joint, sim.jointfloatparam_maxvel)
                for joint in joints
            ]
            max_velocity = math.radians(max(60.0, self.speed_deg_s * 1.35))
            for joint in joints:
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, max_velocity
                )
            actual_before_start = self.bridge.get_robot_joint_positions(ROBOT_ID)
            recorded_start = self._pre_positioned_config.get(action)
            # Do not command a stale handoff pose merely because the
            # coordinator recorded it.  The physical joints are authoritative;
            # keep their current target when they are already at another
            # validated entry point.
            start_targets = (
                recorded_start
                if recorded_start is not None
                and near(
                    actual_before_start,
                    recorded_start,
                    math.radians(JOINT_TOLERANCE_DEG),
                )
                else actual_before_start
            )
            for joint, target in zip(joints, start_targets):
                sim.setJointTargetPosition(joint, float(target))
            command_script = create_command_script(
                sim, robot, RUNTIME_BRIDGE_ALIAS
            )
            if not self.bridge.start_simulation():
                raise RuntimeError(
                    self.bridge.last_error or "cannot take R5 stepping"
                )

            camera_view = sim.getObject(CAMERA_VIEW_PATH)
            ignored_environment = set(
                sim.getObjectsInTree(
                    camera_view, sim.object_shape_type, 0
                )
            )
            if sim.getObjectType(camera_view) == sim.object_shape_type:
                ignored_environment.add(camera_view)
            runner = SmoothRunner(
                self.bridge,
                robot,
                ROBOT_ID,
                joints,
                command_script,
                product,
                ignored_environment=ignored_environment,
                collision_check_interval=self.collision_check_interval,
                workspace_check_interval=self.workspace_check_interval,
            )
            if prepared_mode:
                runner.step("R5 runtime bridge initialized", force_full=True)
            transfer_speed = math.radians(self.speed_deg_s)
            descent_speed = math.radians(
                min(self.speed_deg_s * 0.75, DESCENT_SPEED_CAP_DEG_S)
            )

            with self.inspection_lock:
                if not prepared_mode:
                    runner.hold(0.5, "R5 startup")
                actual_start = self.bridge.get_robot_joint_positions(ROBOT_ID)
                startup_label, startup_path = self._startup_path(
                    action,
                    paths,
                    actual_config=actual_start,
                )
                if startup_path is not None:
                    startup_delay_s = self._startup_delay_seconds(startup_label)
                    if startup_delay_s > 0.0:
                        runner.hold(
                            startup_delay_s,
                            "R5 wait before pick APP",
                        )
                    runner.execute_path(
                        startup_label or "R5 startup",
                        startup_path,
                        transfer_speed,
                    )
                runner.hold(self.hold_seconds, "R5 hold above product")
                runner.set_payload(product)
                runner.execute_path(
                    "R5 descend_to_pick_tcp",
                    paths["pick_descend"],
                    descent_speed,
                )
                grasp_path = self._prepared_grasp_paths[action]
                if not near(grasp_path[0], grasp_path[-1], 1e-8):
                    runner.execute_path(
                        "R5 product final contact approach",
                        grasp_path,
                        descent_speed,
                    )
                else:
                    runner.step(
                        "R5 product contact reached", force_full=True
                    )
                if not self.bridge.set_gripper_gap(ROBOT_ID, 0.150):
                    raise RuntimeError(
                        self.bridge.last_error or "cannot close R5 gripper"
                    )
                runner.hold(0.6, "R5 close product gripper")
                world_pose = [
                    float(value) for value in sim.getObjectPose(product, -1)
                ]
                sim.setObjectParent(product, virtual_tip, True)
                sim.setObjectPose(product, -1, world_pose)
                attached = True
                runner.set_payload(product)
                pickup_orientation = [
                    float(value) for value in sim.getObjectOrientation(product, -1)
                ]
                carry_level_orientation = [0.0, 0.0, pickup_orientation[2]]
                target_product_yaw_deg = self._target_product_yaw_deg(action)
                target_level_orientation = [
                    0.0,
                    0.0,
                    math.radians(target_product_yaw_deg),
                ]
                transfer_release_orientation = carry_level_orientation
                if action == R5_SORT_GOOD_DONE:
                    transfer_release_orientation = (
                        self._good_transfer_preturn_orientation(
                            target_level_orientation,
                            paths["place_descend"],
                        )
                    )
                    good_transfer_preturn_orientation_deg = [
                        round(math.degrees(value), 6)
                        for value in transfer_release_orientation
                    ]
                runner.lock_payload_world_orientation(carry_level_orientation)
                runner.step("R5 product attached", force_full=True)
                runner.lock_payload_world_orientation(None)
                if not near(grasp_path[0], grasp_path[-1], 1e-8):
                    runner.execute_path(
                        "R5 lift product from inspection platform",
                        list(reversed(grasp_path)),
                        descent_speed,
                    )
                runner.lock_payload_world_orientation(carry_level_orientation)
                grasp_transform = [
                    float(value) for value in sim.getObjectPose(product, virtual_tip)
                ]
                lift_transfer_joint6_delta_deg = math.degrees(
                    paths["lift_and_transfer"][-1][5]
                    - paths["lift_and_transfer"][0][5]
                )
                if action == R5_SORT_GOOD_DONE:
                    runner.execute_path_with_payload_orientation(
                        "R5 lift_and_transfer_with_joint6_preturn",
                        paths["lift_and_transfer"],
                        transfer_speed,
                        carry_level_orientation,
                        transfer_release_orientation,
                    )
                else:
                    runner.execute_path(
                        "R5 lift_and_transfer",
                        paths["lift_and_transfer"],
                        transfer_speed,
                    )
                parts = sim.getObject(
                    PARTS["INSPECTION_PRODUCT"].rsplit("/", 1)[0]
                )
                runner.hold(self.hold_seconds, "R5 hold above conveyor")
                target_belt = find_unique_alias(
                    sim,
                    sim.handle_scene,
                    BELT_ALIASES[action],
                )
                allowed_release_contacts = {target_belt}
                if action == R5_SORT_GOOD_DONE:
                    allowed_release_contacts.add(
                        find_unique_alias(
                            sim,
                            sim.handle_scene,
                            "Good_Conveyor_Frame",
                        )
                    )
                release_guard = RobotSafetyGuard(
                    sim,
                    robot,
                    ROBOT_ID,
                    product,
                    ignored_environment=ignored_environment,
                    allowed_payload_contacts=allowed_release_contacts,
                )
                normal_payload_guard = runner.guard
                manual_release_at_tcp = "belt_lower" not in paths
                place_descend_joint6_delta_deg = math.degrees(
                    paths["place_descend"][-1][5]
                    - paths["place_descend"][0][5]
                )
                release_alignment: list[list[float]] = []
                release_transform: list[float] = []
                post_release_return: list[list[float]] | None = None
                try:
                    if manual_release_at_tcp:
                        runner.guard = release_guard
                    if not near(
                        paths["place_descend"][0],
                        paths["place_descend"][-1],
                        1e-8,
                    ):
                        runner.execute_path_with_payload_orientation(
                            "R5 descend_to_place_tcp_with_joint6_yaw",
                            paths["place_descend"],
                            descent_speed,
                            transfer_release_orientation,
                            target_level_orientation,
                        )
                    if not manual_release_at_tcp:
                        runner.guard = release_guard
                        runner.execute_path(
                            "R5 rigid payload lower to belt",
                            paths["belt_lower"],
                            descent_speed,
                        )
                    release_alignment = self._release_alignment_path(
                        action, robot, virtual_tip, joints, product
                    )
                    runner.execute_path(
                        "R5 lower product onto conveyor",
                        release_alignment,
                        descent_speed,
                    )
                    release_transform = [
                        float(value)
                        for value in sim.getObjectPose(product, virtual_tip)
                    ]
                    grasp_transform_max_error = self._pose_max_error(
                        grasp_transform, release_transform
                    )
                    # Break jaw contact before changing the product back to
                    # a scene-owned object.  Keep the release guard active:
                    # it permits payload-to-belt contact but still rejects
                    # any R5-link-to-belt collision.
                    if not self.bridge.set_gripper_gap(ROBOT_ID, 0.158):
                        raise RuntimeError(
                            self.bridge.last_error
                            or "cannot release product from R5 gripper"
                        )
                    runner.hold(0.4, "R5 release product on conveyor")
                    orientation_before_detach = [
                        float(value)
                        for value in sim.getObjectOrientation(product, -1)
                    ]
                    sim.setObjectParent(product, parts, True)
                    attached = False
                    runner.lock_payload_world_orientation(None)
                    runner.step("R5 product detached", force_full=True)
                    orientation_after_detach = [
                        float(value)
                        for value in sim.getObjectOrientation(product, -1)
                    ]
                    release_detach_orientation_jump_deg = [
                        abs(
                            math.degrees(
                                (after - before + math.pi)
                                % (2.0 * math.pi)
                                - math.pi
                            )
                        )
                        for before, after in zip(
                            orientation_before_detach,
                            orientation_after_detach,
                        )
                    ]
                    if max(release_detach_orientation_jump_deg) > 0.1:
                        raise RuntimeError(
                            "R5 product orientation jumped while detaching: "
                            f"delta={release_detach_orientation_jump_deg} deg"
                        )
                    release_product_position_on_belt = [
                        float(value)
                        for value in sim.getObjectPosition(product, -1)
                    ]
                    if action == R5_SORT_GOOD_DONE:
                        good_release_centering_offset_m = [
                            actual - expected
                            for actual, expected in zip(
                                release_product_position_on_belt,
                                GOOD_RELEASE_PRODUCT_POSITION,
                            )
                        ]
                        center_error = math.hypot(
                            good_release_centering_offset_m[0],
                            good_release_centering_offset_m[1],
                        )
                        if center_error > GOOD_RELEASE_CENTER_TOLERANCE_M:
                            raise RuntimeError(
                                "R5 good product release is not centered on "
                                f"the conveyor: error={center_error:.6f} m"
                            )
                    if action == R5_SORT_GOOD_DONE:
                        tip_position = [
                            float(value)
                            for value in sim.getObjectPosition(virtual_tip, -1)
                        ]
                        clearance_position = list(tip_position)
                        clearance_position[2] += 0.180
                        release_clearance = build_tip_translation_path(
                            sim,
                            self.bridge._client.require("simIK"),
                            robot,
                            virtual_tip,
                            joints,
                            self.bridge.get_robot_joint_positions(ROBOT_ID),
                            clearance_position,
                            61,
                            "R5_good_release_vertical_clearance",
                        )
                        runner.execute_path(
                            "R5 clear good product vertically",
                            release_clearance,
                            descent_speed,
                        )
                        post_release_return = interpolate_joint_line(
                            release_clearance[-1], [0.0] * 6, 121
                        )
                    else:
                        runner.execute_path(
                            "R5 clear released product",
                            list(reversed(release_alignment)),
                            descent_speed,
                        )
                finally:
                    runner.guard = normal_payload_guard
                    release_guard.close()
                runner.set_payload(None)
                runner.execute_path(
                    "R5 retreat_and_return_home",
                    post_release_return or paths["return_home"],
                    transfer_speed,
                )
                if not self.bridge.set_gripper(ROBOT_ID, True):
                    raise RuntimeError(
                        self.bridge.last_error
                        or "cannot fully open R5 gripper above conveyor"
                    )
                runner.hold(0.45, "R5 fully open above conveyor")
                branch = (
                    "good" if action == R5_SORT_GOOD_DONE else "defect"
                )
                # Start the conveyor only after R5 has cleared the released
                # product and returned home.  Opening the long native fingers
                # at the belt entrance would sweep the top rail into the box.
                self.bridge.set_string_signal("cell_conveyor_state", branch)
                runner.hold(
                    CONVEYOR_ENTRY_CLEARANCE_SECONDS,
                    f"R5 {branch} conveyor entry clearance",
                )
                release_product_orientation_deg = [
                    math.degrees(float(value))
                    for value in sim.getObjectOrientation(product, -1)
                ]
                release_product_level_error_deg = self._level_error_deg(
                    release_product_orientation_deg
                )
                if release_product_level_error_deg > LEVEL_PAYLOAD_TOLERANCE_DEG:
                    raise RuntimeError(
                        "R5 released product is not level with the ground: "
                        f"roll={release_product_orientation_deg[0]:.6f} deg, "
                        f"pitch={release_product_orientation_deg[1]:.6f} deg"
                    )
                if action == R5_SORT_GOOD_DONE:
                    good_belt_axis_alignment_error_deg = (
                        self._belt_axis_alignment_error_deg(
                            release_product_orientation_deg[2],
                            target_product_yaw_deg,
                        )
                    )
                    if (
                        good_belt_axis_alignment_error_deg
                        > GOOD_BELT_AXIS_ALIGNMENT_TOLERANCE_DEG
                    ):
                        raise RuntimeError(
                            "R5 good product is not aligned with a conveyor axis: "
                            f"yaw={release_product_orientation_deg[2]:.6f} deg, "
                            "axis_error="
                            f"{good_belt_axis_alignment_error_deg:.6f} deg"
                        )
                    self._check_released_product_environment(sim, product)
                runner.hold(0.4, "R5 final home hold")

            final_joints = runner.joint_positions()
            if not near(
                final_joints,
                [0.0] * 6,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError("R5 did not return to the validated zero state")

            result = {
                "action": action,
                "visual_suction_only": True,
                "contact_aligned_grasp": True,
                "attachment_snap_m": 0.0,
                "physical_grasp_validated": False,
                "runtime_pick_orientation_deg": list(PICK_ORIENTATION_DEG),
                "runtime_place_orientation_deg": list(place_orientation),
                "runtime_place_prealign_orientation_deg": (
                    list(GOOD_PREALIGN_ORIENTATION_DEG)
                    if action == R5_SORT_GOOD_DONE
                    else list(PICK_ORIENTATION_DEG)
                ),
                "runtime_place_app_position": [
                    round(float(value), 6) for value in positions[2]
                ],
                "runtime_place_tcp_position": [
                    round(float(value), 6) for value in positions[3]
                ],
                "runtime_good_target_xy_offset_m": (
                    list(GOOD_RUNTIME_XY_OFFSET_M)
                    if action == R5_SORT_GOOD_DONE
                    else [0.0, 0.0]
                ),
                "runtime_tool_tcp_offset_m": VIRTUAL_TCP_OFFSET_M,
                "belt_height_correction_m": BELT_HEIGHT_CORRECTION_M,
                "wait_to_pick_app_start_delay_s": (
                    WAIT_TO_PICK_APP_START_DELAY_S
                    if startup_label == "R5 wait_to_pick_app"
                    else 0.0
                ),
                "belt_height_correction_mode": (
                    "manual_rviz_tcp_release"
                    if manual_release_at_tcp
                    else "rigid_tcp_motion"
                ),
                "manual_rviz_release_at_tcp": manual_release_at_tcp,
                "rigid_visual_payload_through_release": False,
                "place_descent_payload_yaw_synced_to_joint6": (
                    action != R5_SORT_GOOD_DONE
                ),
                "place_descend_joint6_delta_deg": round(
                    place_descend_joint6_delta_deg,
                    6,
                ),
                "lift_transfer_joint6_delta_deg": round(
                    lift_transfer_joint6_delta_deg,
                    6,
                ),
                "good_transfer_joint6_preturn_deg": (
                    GOOD_TRANSFER_JOINT6_PRETURN_DEG
                    if action == R5_SORT_GOOD_DONE
                    else 0.0
                ),
                "good_transfer_preturn_orientation_deg": (
                    good_transfer_preturn_orientation_deg
                    if good_transfer_preturn_orientation_deg is not None
                    else None
                ),
                "good_release_centering_offset_m": (
                    good_release_centering_offset_m
                    if action == R5_SORT_GOOD_DONE
                    else [0.0, 0.0, 0.0]
                ),
                "release_product_position_on_belt": [
                    round(value, 6)
                    for value in release_product_position_on_belt
                ],
                "level_payload_bottom_parallel_to_ground": True,
                "grasp_transform_max_error": grasp_transform_max_error,
                "good_base_turn_delta_deg": (
                    GOOD_BASE_TURN_DELTA_DEG
                    if action == R5_SORT_GOOD_DONE
                    else 0.0
                ),
                "transfer_waypoint": list(transfer_waypoint),
                "release_product_orientation_deg": [
                    round(value, 6) for value in release_product_orientation_deg
                ],
                "release_orientation_strategy": "preserve_carried_pose",
                "forced_release_yaw_deg": None,
                "release_detach_orientation_jump_deg": [
                    round(value, 6)
                    for value in release_detach_orientation_jump_deg
                ],
                "good_target_product_yaw_deg": (
                    target_product_yaw_deg
                    if action == R5_SORT_GOOD_DONE
                    else None
                ),
                "target_product_yaw_deg": target_product_yaw_deg,
                "release_product_level_error_deg": (
                    round(release_product_level_error_deg, 6)
                    if release_product_level_error_deg is not None
                    else None
                ),
                "good_belt_axis_alignment_error_deg": (
                    round(good_belt_axis_alignment_error_deg, 6)
                    if good_belt_axis_alignment_error_deg is not None
                    else None
                ),
                "final_joint_positions_deg": [
                    round(math.degrees(value), 6) for value in final_joints
                ],
                "conveyor_branch": branch,
                "product_position": [
                    round(float(value), 6)
                    for value in sim.getObjectPosition(product, -1)
                ],
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
            if runner is not None:
                runner.close()
            if succeeded:
                for handle in (command_script, virtual_tip):
                    if handle != -1:
                        sim.removeObjects([handle])
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
    "R5_ACTIONS",
    "R5_SORT_DEFECT_DONE",
    "R5_SORT_GOOD_DONE",
    "R5_WAIT_POINT",
    "R5MotionController",
    "WAIT_TO_PICK_APP_START_DELAY_S",
    "load_r5_plan",
]
