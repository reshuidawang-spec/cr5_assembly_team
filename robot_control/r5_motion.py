"""R5 visual good/defect sorting for the current five-CR5A scene.

Both branches keep the seven Git targets unchanged.  A runtime 100 mm vacuum
TCP carries the inspection template through release.  The routes share the
same pickup and lift, then use opposite private transfer waypoints for the two
adjacent conveyors.  The 26 mm target-to-belt height mismatch is handled as a
visible runtime lowering step immediately before release; the saved scene and
target Dummies are never modified.
"""

from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Any, Optional

from robot_control.r1_motion import PLAN_PATH as R1_PLAN_PATH
from robot_control.r1_motion import load_r1_plan
from robot_control.runtime_cartesian import (
    RobotSafetyGuard,
    SmoothRunner,
    build_pick_place_paths,
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
)
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import PARTS


R5_SORT_GOOD_DONE = "R5_SORT_GOOD_DONE"
R5_SORT_DEFECT_DONE = "R5_SORT_DEFECT_DONE"
R5_ACTIONS = frozenset({R5_SORT_GOOD_DONE, R5_SORT_DEFECT_DONE})

ROBOT_ID = "R5"
SCENE_NAME = "five_cr5a_cell.ttt"
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
    "R5_HOME_REF": [0.15, -0.50, 0.80],
    "R5_PRODUCT_PICK_APP": [0.15, 0.05, 0.60],
    "R5_PRODUCT_PICK_TCP": [0.15, 0.05, 0.34],
    "R5_GOOD_PLACE_APP": [0.65, -1.10, 0.62],
    "R5_GOOD_PLACE_TCP": [0.65, -1.10, 0.42],
    "R5_DEFECT_PLACE_APP": [-0.35, -1.12, 0.62],
    "R5_DEFECT_PLACE_TCP": [-0.35, -1.12, 0.42],
}

PICK_ORIENTATION_DEG = (195.0, -45.0, 0.0)
VIRTUAL_TCP_OFFSET_M = 0.100
BELT_HEIGHT_CORRECTION_M = -0.026
DEFECT_TRANSFER_WAYPOINT = (-0.15, -0.15, 0.65)
GOOD_RUNTIME_XY_OFFSET_M = (-0.010, 0.020)
GOOD_BASE_TURN_DELTA_DEG = -121.0
GOOD_PREALIGN_ORIENTATION_DEG = (-143.152079, 31.403342, 104.057370)
GOOD_PLACE_ORIENTATION_DEG = (-134.007027, 10.545291, 79.271417)
GOOD_TARGET_PRODUCT_YAW_DEG = -90.0
GOOD_TRACK_PARALLEL_TOLERANCE_DEG = 1.0
GOOD_TRANSFER_HEIGHT_M = 0.760
GOOD_BASE_TURN_POINTS = 121
GOOD_TRANSFER_POINTS = 61
GOOD_APP_POINTS = 81
GOOD_ALIGN_POINTS = 101
BELT_LOWER_POINTS = 15
GRASP_TRANSFORM_TOLERANCE = 1e-9
BELT_ALIASES = {
    R5_SORT_GOOD_DONE: "Good_Conveyor_Belt_Black",
    R5_SORT_DEFECT_DONE: "Defect_Conveyor_Belt_Black",
}

INSPECTION_PRODUCT_POSITION = (0.15, 0.05, 0.216)
POSITION_TOLERANCE_M = 0.003
TARGET_TOLERANCE = 1e-6
JOINT_TOLERANCE_DEG = 0.30
TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_CAP_DEG_S = 24.0
HOLD_SECONDS = 0.8

CAMERA_VIEW_PATH = (
    "/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area"
)
RUNTIME_PREFIX = "R5_Runtime_"
RUNTIME_TCP_ALIAS = f"{RUNTIME_PREFIX}Vacuum_TCP"
RUNTIME_BRIDGE_ALIAS = f"{RUNTIME_PREFIX}Command_Bridge"


class R5MotionController:
    """Execute the two explicit visual quality-sorting branches."""

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
        self._prepared_paths: dict[
            str, dict[str, list[list[float]]]
        ] = {}
        self._prepared_transfer_waypoints: dict[str, list[float]] = {}
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
        plan = load_r1_plan(self.r1_plan_path)
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

    def _validate_preflight(self, verify_static: bool = True) -> None:
        sim = self.bridge.sim
        if sim.getSimulationState() == sim.simulation_stopped:
            raise RuntimeError("R5 sorting requires the running coordinated scene")
        if verify_static:
            self._validate_static()
        if not near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            [0.0] * 6,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError("R5 is not at the validated zero start")

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
    def _parallel_yaw_error_deg(actual: float, target: float) -> float:
        return abs((actual - target + 90.0) % 180.0 - 90.0)

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
                sim_ik,
                base,
                virtual_tip,
                joints,
                pick_app_target,
                [0.0] * 6,
                "R5 good pick APP",
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
        client = getattr(self.bridge, "_client", None)
        if client is None:
            raise RuntimeError("CoppeliaSim remote client is unavailable")
        robot = self.bridge.get_object_handle(ROBOT_ID)
        joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
        remove_runtime_objects(sim, robot, RUNTIME_PREFIX)
        virtual_tip = create_virtual_tcp(
            sim, robot, RUNTIME_TCP_ALIAS, VIRTUAL_TCP_OFFSET_M
        )
        try:
            protected_before = self._target_snapshot()
            positions = self._positions(action)
            sim_ik = client.require("simIK")
            if action == R5_SORT_GOOD_DONE:
                prepared_paths, transfer_waypoint = self._build_good_paths(
                    sim, sim_ik, robot, virtual_tip, joints, positions
                )
            else:
                prepared_paths = self._build_defect_paths(
                    sim, sim_ik, robot, virtual_tip, joints, positions
                )
                transfer_waypoint = list(DEFECT_TRANSFER_WAYPOINT)
            if self._target_snapshot() != protected_before:
                raise RuntimeError(
                    "R5 protected Git targets changed during preparation"
                )
        finally:
            sim.removeObjects([virtual_tip])
        self._prepared_paths[action] = prepared_paths
        self._prepared_transfer_waypoints[action] = list(transfer_waypoint)
        return {
            "robot_id": ROBOT_ID,
            "prepared_actions": [action],
            "path_points": {
                name: len(path) for name, path in prepared_paths.items()
            },
            "transfer_waypoint": list(transfer_waypoint),
        }

    def set_continuous_stepping(self, enabled: bool) -> None:
        self._continuous_stepping = bool(enabled)

    def execute(self, action: str) -> dict[str, Any]:
        if action not in R5_ACTIONS:
            raise ValueError(f"unsupported R5 action: {action}")
        prepared_paths = self._prepared_paths.get(action)
        prepared_mode = prepared_paths is not None
        self._validate_preflight(verify_static=not prepared_mode)

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
        good_track_parallel_error_deg: Optional[float] = None
        try:
            self.bridge.set_stepping(True)
            robot = self.bridge.get_object_handle(ROBOT_ID)
            joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
            product = self.bridge.get_object_handle("INSPECTION_PRODUCT")
            remove_runtime_objects(sim, robot, RUNTIME_PREFIX)
            virtual_tip = create_virtual_tcp(
                sim, robot, RUNTIME_TCP_ALIAS, VIRTUAL_TCP_OFFSET_M
            )

            positions = self._positions(action)
            if prepared_paths is not None:
                paths = prepared_paths
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

            original_max_velocities = [
                sim.getObjectFloatParam(joint, sim.jointfloatparam_maxvel)
                for joint in joints
            ]
            max_velocity = math.radians(max(60.0, self.speed_deg_s * 1.35))
            for joint in joints:
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, max_velocity
                )
                sim.setJointTargetPosition(joint, 0.0)
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
                runner.execute_path(
                    "R5 initial_to_pick_app",
                    paths["initial_to_pick_app"],
                    transfer_speed,
                )
                runner.hold(self.hold_seconds, "R5 hold above product")
                runner.execute_path(
                    "R5 descend_to_pick_tcp",
                    paths["pick_descend"],
                    descent_speed,
                )
                sim.setObjectParent(product, virtual_tip, True)
                attached = True
                runner.set_payload(product)
                runner.step("R5 product attached", force_full=True)
                grasp_transform = [
                    float(value) for value in sim.getObjectPose(product, virtual_tip)
                ]
                runner.execute_path(
                    "R5 lift_and_transfer",
                    paths["lift_and_transfer"],
                    transfer_speed,
                )
                parts = sim.getObject(
                    PARTS["INSPECTION_PRODUCT"].rsplit("/", 1)[0]
                )
                runner.hold(self.hold_seconds, "R5 hold above conveyor")
                runner.execute_path(
                    "R5 descend_to_place_tcp",
                    paths["place_descend"],
                    descent_speed,
                )
                target_belt = find_unique_alias(
                    sim,
                    sim.handle_scene,
                    BELT_ALIASES[action],
                )
                release_guard = RobotSafetyGuard(
                    sim,
                    robot,
                    ROBOT_ID,
                    product,
                    ignored_environment=ignored_environment,
                    allowed_payload_contacts={target_belt},
                )
                normal_payload_guard = runner.guard
                try:
                    runner.guard = release_guard
                    runner.execute_path(
                        "R5 rigid payload lower to belt",
                        paths["belt_lower"],
                        descent_speed,
                    )
                finally:
                    runner.guard = normal_payload_guard
                    release_guard.close()
                release_transform = [
                    float(value) for value in sim.getObjectPose(product, virtual_tip)
                ]
                grasp_transform_max_error = self._pose_max_error(
                    grasp_transform, release_transform
                )
                if grasp_transform_max_error > GRASP_TRANSFORM_TOLERANCE:
                    raise RuntimeError(
                        "R5 grasp transform changed before release: "
                        f"{grasp_transform_max_error:.12f}"
                    )
                sim.setObjectParent(product, parts, True)
                attached = False
                runner.set_payload(None)
                runner.step("R5 product detached", force_full=True)
                release_product_orientation_deg = [
                    math.degrees(float(value))
                    for value in sim.getObjectOrientation(product, -1)
                ]
                if action == R5_SORT_GOOD_DONE:
                    good_track_parallel_error_deg = self._parallel_yaw_error_deg(
                        release_product_orientation_deg[2],
                        GOOD_TARGET_PRODUCT_YAW_DEG,
                    )
                    if (
                        good_track_parallel_error_deg
                        > GOOD_TRACK_PARALLEL_TOLERANCE_DEG
                    ):
                        raise RuntimeError(
                            "R5 good product is not parallel to conveyor: "
                            f"yaw={release_product_orientation_deg[2]:.6f} deg, "
                            "parallel_error="
                            f"{good_track_parallel_error_deg:.6f} deg"
                        )
                    self._check_released_product_environment(sim, product)
                runner.execute_path(
                    "R5 retreat_and_return_home",
                    paths["return_home"],
                    transfer_speed,
                )
                runner.hold(0.4, "R5 final home hold")

            final_joints = runner.joint_positions()
            if not near(
                final_joints,
                [0.0] * 6,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError("R5 did not return to the validated zero state")

            branch = "good" if action == R5_SORT_GOOD_DONE else "defect"
            self.bridge.set_string_signal("cell_conveyor_state", branch)
            runner.hold(0.5, f"R5 {branch} conveyor start")
            result = {
                "action": action,
                "visual_suction_only": True,
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
                "belt_height_correction_mode": "rigid_tcp_motion",
                "rigid_visual_payload_through_release": True,
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
                "good_target_product_yaw_deg": (
                    GOOD_TARGET_PRODUCT_YAW_DEG
                    if action == R5_SORT_GOOD_DONE
                    else None
                ),
                "good_track_parallel_error_deg": (
                    round(good_track_parallel_error_deg, 6)
                    if good_track_parallel_error_deg is not None
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
    "R5_ACTIONS",
    "R5_SORT_DEFECT_DONE",
    "R5_SORT_GOOD_DONE",
    "R5MotionController",
]
