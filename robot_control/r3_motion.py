"""R3 visual module installation and assembled-product transfer.

The current scene has no R3 tool and its module layout overlaps the PCB/main
chip.  The controller therefore keeps every Git target unchanged, creates a
runtime-only 100 mm vacuum TCP, and uses the previously validated 46 mm visual
module lift.  Product transfer carries the executor-owned assembly template to
the inspection station and swaps to the matching inspection template only
after R3 has returned home.
"""

from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Any, Optional

from robot_control.r1_motion import PLAN_PATH as R1_PLAN_PATH
from robot_control.r1_motion import load_r1_plan
from robot_control.runtime_cartesian import (
    SmoothRunner,
    build_pick_place_paths,
    create_command_script,
    create_virtual_tcp,
    near,
    remove_runtime_objects,
    restore_visibility,
    set_visibility,
    sha256_file,
    tree_visibility,
)
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import PARTS


R3_MODULE_PLACED = "R3_MODULE_PLACED"
R3_PRODUCT_TO_INSPECTION = "R3_PRODUCT_TO_INSPECTION"
R3_ACTIONS = frozenset({R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION})

ROBOT_ID = "R3"
SCENE_NAME = "five_cr5a_cell.ttt"
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
    "R3_HOME_REF": [-0.55, 0.28, 0.80],
    "R3_MODULE_PICK_APP": [-0.85, -0.05, 0.45],
    "R3_MODULE_PICK_TCP": [-0.85, -0.05, 0.24],
    "R3_MODULE_PLACE_APP": [-1.12, 0.22, 0.50],
    "R3_MODULE_PLACE_TCP": [-1.12, 0.22, 0.34],
    "R3_PRODUCT_PICK_APP": [-1.15, 0.20, 0.60],
    "R3_PRODUCT_PICK_TCP": [-1.15, 0.20, 0.34],
    "R3_PRODUCT_PLACE_INSPECTION_APP": [0.15, 0.05, 0.60],
    "R3_PRODUCT_PLACE_INSPECTION_TCP": [0.15, 0.05, 0.34],
}

MODULE_ORIENTATION_DEG = (195.0, 0.0, -135.0)
PRODUCT_ORIENTATION_DEG = (195.0, 0.0, -135.0)
VIRTUAL_TCP_OFFSET_M = 0.100
MODULE_VISUAL_OFFSET_Z = 0.046
PRODUCT_VISUAL_LIFT_Z = 0.100

BOX_ASSEMBLY_POSITION = (-1.150002, 0.199900, 0.215915)
PCB_ASSEMBLY_POSITION = (-1.150058, 0.200161, 0.281739)
MODULE_SUPPLY_POSITION = (-0.85, -0.05, 0.1735)
MODULE_ASSEMBLY_POSITION = (-1.120464, 0.219603, 0.319289)
TERMINAL_SUPPLY_POSITION = (-1.9, 0.1, 0.1735)
TERMINAL_ASSEMBLY_POSITION = (-1.089629, 0.129919, 0.329536)
INSPECTION_PRODUCT_POSITION = (0.15, 0.05, 0.216)

POSITION_TOLERANCE_M = 0.003
TARGET_TOLERANCE = 1e-6
JOINT_TOLERANCE_DEG = 0.30
TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_CAP_DEG_S = 24.0
HOLD_SECONDS = 0.8

RUNTIME_PREFIX = "R3_Runtime_"
RUNTIME_TCP_ALIAS = f"{RUNTIME_PREFIX}Vacuum_TCP"
RUNTIME_BRIDGE_ALIAS = f"{RUNTIME_PREFIX}Command_Bridge"
CAMERA_VIEW_PATH = (
    "/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area"
)


class R3MotionController:
    """Execute only the two explicit R3 visual actions."""

    def __init__(
        self,
        bridge: SimBridge,
        r1_plan_path: Path = R1_PLAN_PATH,
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
        self.assembly_lock = assembly_lock or threading.Lock()
        self.inspection_lock = inspection_lock or threading.Lock()
        self.speed_deg_s = float(speed_deg_s)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.collision_check_interval = int(collision_check_interval)
        self.workspace_check_interval = int(workspace_check_interval)
        self._prepared_paths: dict[
            str, dict[str, list[list[float]]]
        ] = {}
        self._pre_positioned_config: dict[str, list[float]] = {}
        self._continuous_stepping = False

    def _target_snapshot(self) -> dict[str, dict[str, list[float]]]:
        return {
            name: {
                "position": [
                    round(float(value), 9)
                    for value in self.bridge.get_target_pose(name)["position"]
                ],
                "orientation": [
                    round(float(value), 9)
                    for value in self.bridge.get_target_pose(name)["orientation"]
                ],
            }
            for name in TARGET_NAMES
        }

    def _validate_scene_and_targets(self) -> dict[str, Any]:
        plan = load_r1_plan(self.r1_plan_path)
        scene = Path(self.bridge.scene_path())
        if scene.name != SCENE_NAME:
            raise RuntimeError(f"unexpected CoppeliaSim scene: {scene}")
        fingerprint = plan["validation"]["scene_fingerprint"]
        if scene.stat().st_size != int(fingerprint.get("size", -1)):
            raise RuntimeError("R3 scene size differs; repeat full preflight")
        if sha256_file(scene) != fingerprint["sha256"]:
            raise RuntimeError("R3 scene hash differs; repeat full preflight")

        snapshot = self._target_snapshot()
        for name, expected_position in PROTECTED_TARGETS.items():
            actual = snapshot[name]
            if not near(actual["position"], expected_position, TARGET_TOLERANCE):
                raise RuntimeError(f"protected Git target changed: {name}")
            if not near(actual["orientation"], [0.0, 0.0, 0.0], TARGET_TOLERANCE):
                raise RuntimeError(f"protected Git target orientation changed: {name}")
        return plan

    def _validate_part(
        self, name: str, expected_position: tuple[float, float, float]
    ) -> int:
        sim = self.bridge.sim
        handle = self.bridge.get_object_handle(name)
        parts = sim.getObject(PARTS["BOX_BLANK"].rsplit("/", 1)[0])
        if sim.getObjectParent(handle) != parts:
            raise RuntimeError(f"{name} is not owned by /Parts")
        actual = [float(value) for value in sim.getObjectPosition(handle, -1)]
        if not near(actual, expected_position, POSITION_TOLERANCE_M):
            raise RuntimeError(
                f"{name} is not at its validated position: {actual}"
            )
        return handle

    def _validate_preflight(
        self, action: str, verify_static: bool = True
    ) -> dict[str, Any]:
        sim = self.bridge.sim
        if sim.getSimulationState() == sim.simulation_stopped:
            raise RuntimeError(f"{action} requires the running coordinated scene")
        plan = (
            self._validate_scene_and_targets()
            if verify_static
            else load_r1_plan(self.r1_plan_path)
        )
        expected_r3 = self._pre_positioned_config.get(action, [0.0] * 6)
        if not near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            expected_r3,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError(
                "R3 is not at the validated start "
                f"(expected pre-positioned={action in self._pre_positioned_config})"
            )

        self._validate_part("BOX_BLANK", BOX_ASSEMBLY_POSITION)
        self._validate_part("PCB_SUPPLY", PCB_ASSEMBLY_POSITION)
        if action == R3_MODULE_PLACED:
            self._validate_part("CONTROL_MODULE_SUPPLY", MODULE_SUPPLY_POSITION)
            self._validate_part("TERMINAL_BLOCK_SUPPLY", TERMINAL_SUPPLY_POSITION)
            r1_expected = plan["paths"]["box_retreat_and_terminal_approach"][-1]
            if not near(
                self.bridge.get_robot_joint_positions("R1"),
                r1_expected,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError(
                    "R1 is not waiting at R1_TERMINAL_PICK_APP before R3 module"
                )
        else:
            self._validate_part("CONTROL_MODULE_SUPPLY", MODULE_ASSEMBLY_POSITION)
            self._validate_part("TERMINAL_BLOCK_SUPPLY", TERMINAL_ASSEMBLY_POSITION)
            if not near(
                self.bridge.get_robot_joint_positions("R1"),
                [0.0] * 6,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError("R1 is not home before R3 product transfer")
        return plan

    def _path_positions(self, action: str) -> tuple[list[float], ...]:
        if action == R3_MODULE_PLACED:
            names = TARGET_NAMES[1:5]
        else:
            names = TARGET_NAMES[5:9]
        return tuple(
            list(self.bridge.get_target_pose(name)["position"]) for name in names
        )

    @staticmethod
    def _product_transfer_waypoints() -> list[dict[str, Any]]:
        return [
            {
                "name": "assembly clear",
                "position": [-0.90, -0.08, 0.70],
                "orientation_deg": PRODUCT_ORIENTATION_DEG,
                "points": 61,
            },
            {
                "name": "center high",
                "position": [-0.35, -0.22, 0.72],
                "orientation_deg": PRODUCT_ORIENTATION_DEG,
                "points": 81,
            },
        ]

    def prepare(self, action: str) -> dict[str, Any]:
        if action not in R3_ACTIONS:
            raise ValueError(f"unsupported R3 action: {action}")
        sim = self.bridge.sim
        if sim.getSimulationState() != sim.simulation_stopped:
            raise RuntimeError("R3 preparation requires a stopped scene")
        self._validate_scene_and_targets()
        if not near(
            self.bridge.get_robot_joint_positions(ROBOT_ID),
            [0.0] * 6,
            math.radians(JOINT_TOLERANCE_DEG),
        ):
            raise RuntimeError("R3 is not zero during preparation")

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
            orientation = (
                MODULE_ORIENTATION_DEG
                if action == R3_MODULE_PLACED
                else PRODUCT_ORIENTATION_DEG
            )
            waypoints = (
                []
                if action == R3_MODULE_PLACED
                else self._product_transfer_waypoints()
            )
            protected_before = self._target_snapshot()
            positions = self._path_positions(action)
            prepared_paths = build_pick_place_paths(
                sim,
                client.require("simIK"),
                robot,
                virtual_tip,
                joints,
                RUNTIME_PREFIX,
                positions[0],
                positions[1],
                positions[2],
                positions[3],
                orientation,
                orientation,
                waypoints,
            )
            if self._target_snapshot() != protected_before:
                raise RuntimeError(
                    "R3 protected Git targets changed during preparation"
                )
        finally:
            sim.removeObjects([virtual_tip])
        self._prepared_paths[action] = prepared_paths
        return {
            "robot_id": ROBOT_ID,
            "prepared_actions": [action],
            "path_points": {
                name: len(path) for name, path in prepared_paths.items()
            },
        }

    def set_continuous_stepping(self, enabled: bool) -> None:
        self._continuous_stepping = bool(enabled)

    def set_pre_positioned(self, action: str, config: list[float]) -> None:
        """Record the joint config set by ``_preposition_robots()``."""
        if action is not None:
            self._pre_positioned_config[action] = list(config)

    def execute(self, action: str) -> dict[str, Any]:
        if action not in R3_ACTIONS:
            raise ValueError(f"unsupported R3 action: {action}")
        prepared_paths = self._prepared_paths.get(action)
        prepared_mode = prepared_paths is not None
        self._validate_preflight(action, verify_static=not prepared_mode)

        sim = self.bridge.sim
        robot = -1
        virtual_tip = -1
        command_script = -1
        payload = -1
        runner: Optional[SmoothRunner] = None
        joints: list[int] = []
        original_max_velocities: list[float] = []
        attached = False
        succeeded = False
        visibility: dict[str, dict[int, int]] = {}
        try:
            self.bridge.set_stepping(True)
            robot = self.bridge.get_object_handle(ROBOT_ID)
            joints = self.bridge.get_robot_joint_handles(ROBOT_ID)
            remove_runtime_objects(sim, robot, RUNTIME_PREFIX)
            virtual_tip = create_virtual_tcp(
                sim, robot, RUNTIME_TCP_ALIAS, VIRTUAL_TCP_OFFSET_M
            )

            if action == R3_MODULE_PLACED:
                payload = self.bridge.get_object_handle("CONTROL_MODULE_SUPPLY")
                orientation = MODULE_ORIENTATION_DEG
                waypoints: list[dict[str, Any]] = []
            else:
                payload = self.bridge.get_object_handle("ASSEMBLY_PRODUCT")
                orientation = PRODUCT_ORIENTATION_DEG
                waypoints = self._product_transfer_waypoints()
                roots = {
                    "box": self.bridge.get_object_handle("BOX_BLANK"),
                    "pcb": self.bridge.get_object_handle("PCB_SUPPLY"),
                    "module": self.bridge.get_object_handle("CONTROL_MODULE_SUPPLY"),
                    "terminal": self.bridge.get_object_handle("TERMINAL_BLOCK_SUPPLY"),
                    "assembly": payload,
                    "inspection": self.bridge.get_object_handle("INSPECTION_PRODUCT"),
                }
                visibility = {
                    name: tree_visibility(sim, handle)
                    for name, handle in roots.items()
                }
                for name in ("box", "pcb", "module", "terminal", "inspection"):
                    set_visibility(sim, visibility[name], False)
                set_visibility(sim, visibility["assembly"], True)

            positions = self._path_positions(action)
            if prepared_paths is not None:
                paths = prepared_paths
            else:
                client = getattr(self.bridge, "_client", None)
                if client is None:
                    raise RuntimeError(
                        "CoppeliaSim remote client is unavailable"
                    )
                protected_before = self._target_snapshot()
                paths = build_pick_place_paths(
                    sim,
                    client.require("simIK"),
                    robot,
                    virtual_tip,
                    joints,
                    RUNTIME_PREFIX,
                    positions[0],
                    positions[1],
                    positions[2],
                    positions[3],
                    orientation,
                    orientation,
                    waypoints,
                )
                if self._target_snapshot() != protected_before:
                    raise RuntimeError(
                        "R3 protected Git targets changed during planning"
                    )

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
                    self.bridge.last_error or "cannot take R3 stepping"
                )
            ignored_environment: set[int] = set()
            if action == R3_PRODUCT_TO_INSPECTION:
                camera_view = sim.getObject(CAMERA_VIEW_PATH)
                ignored_environment.update(
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
                payload,
                ignored_environment=ignored_environment,
                collision_check_interval=self.collision_check_interval,
                workspace_check_interval=self.workspace_check_interval,
            )
            if prepared_mode:
                runner.step("R3 runtime bridge initialized", force_full=True)
            transfer_speed = math.radians(self.speed_deg_s)
            descent_speed = math.radians(
                min(self.speed_deg_s * 0.75, DESCENT_SPEED_CAP_DEG_S)
            )

            if action not in self._pre_positioned_config:
                if not prepared_mode:
                    runner.hold(0.5, "R3 startup")
                runner.execute_path(
                    "R3 initial_to_pick_app",
                    paths["initial_to_pick_app"],
                    transfer_speed,
                )
                runner.hold(self.hold_seconds, "R3 hold above payload")
            # else: pre-positioned at pick APP — skip the initial approach
            # to reduce simulation-time handoff delay.
            runner.execute_path(
                "R3 descend_to_pick_tcp", paths["pick_descend"], descent_speed
            )
            sim.setObjectParent(payload, virtual_tip, True)
            attached = True
            runner.set_payload(payload)
            if action == R3_MODULE_PLACED:
                runner.animate_world_offset(
                    payload,
                    [0.0, 0.0, MODULE_VISUAL_OFFSET_Z],
                    12,
                    "R3 module visual offset",
                )
            else:
                runner.animate_world_offset(
                    payload,
                    [0.0, 0.0, PRODUCT_VISUAL_LIFT_Z],
                    20,
                    "R3 product visual lift",
                )

            if action == R3_MODULE_PLACED:
                lock_contexts = (self.assembly_lock,)
            else:
                lock_contexts = (self.assembly_lock, self.inspection_lock)
            for lock in lock_contexts:
                lock.acquire()
            try:
                runner.execute_path(
                    "R3 lift_and_transfer",
                    paths["lift_and_transfer"],
                    transfer_speed,
                )
                runner.hold(self.hold_seconds, "R3 hold above place")
                runner.execute_path(
                    "R3 descend_to_place_tcp",
                    paths["place_descend"],
                    descent_speed,
                )
                if action == R3_PRODUCT_TO_INSPECTION:
                    runner.animate_world_offset(
                        payload,
                        [0.0, 0.0, -PRODUCT_VISUAL_LIFT_Z],
                        20,
                        "R3 product visual lower",
                    )
                parts = sim.getObject(PARTS["BOX_BLANK"].rsplit("/", 1)[0])
                sim.setObjectParent(payload, parts, True)
                attached = False
                runner.set_payload(None)
                runner.step("R3 payload detached", force_full=True)
                runner.execute_path(
                    "R3 retreat_and_return_home",
                    paths["return_home"],
                    transfer_speed,
                )
            finally:
                for lock in reversed(lock_contexts):
                    lock.release()
            runner.hold(0.4, "R3 final home hold")

            final_joints = runner.joint_positions()
            if not near(
                final_joints,
                [0.0] * 6,
                math.radians(JOINT_TOLERANCE_DEG),
            ):
                raise RuntimeError("R3 did not return to the validated zero state")

            if action == R3_PRODUCT_TO_INSPECTION:
                set_visibility(sim, visibility["assembly"], False)
                set_visibility(sim, visibility["inspection"], True)
                inspection = self.bridge.get_object_handle("INSPECTION_PRODUCT")
                sim.setObjectPosition(
                    inspection, -1, list(INSPECTION_PRODUCT_POSITION)
                )
                sim.setObjectOrientation(inspection, -1, [0.0, 0.0, 0.0])

            result = {
                "action": action,
                "visual_suction_only": True,
                "runtime_orientation_deg": list(orientation),
                "virtual_tcp_offset_m": VIRTUAL_TCP_OFFSET_M,
                "final_joint_positions_deg": [
                    round(math.degrees(value), 6) for value in final_joints
                ],
                "payload_position": [
                    round(float(value), 6)
                    for value in sim.getObjectPosition(payload, -1)
                ],
            }
            if action == R3_MODULE_PLACED:
                result["module_visual_offset_m"] = MODULE_VISUAL_OFFSET_Z
            else:
                result["template_stage_swap"] = True
                result["product_visual_lift_m"] = PRODUCT_VISUAL_LIFT_Z
            succeeded = True
            return result
        except Exception:
            if attached and payload != -1:
                try:
                    self.bridge.detach_object(payload)
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
                for layers in visibility.values():
                    try:
                        restore_visibility(sim, layers)
                    except Exception:
                        pass


__all__ = [
    "R3_ACTIONS",
    "R3_MODULE_PLACED",
    "R3_PRODUCT_TO_INSPECTION",
    "R3MotionController",
]
