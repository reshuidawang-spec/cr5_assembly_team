"""Single-stepping coordinated front-half execution with R4/R5 pre-approach."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

from robot_control.r1_motion import (
    PLAN_PATH as R1_PLAN_PATH,
    cumulative_max_joint_distance,
    interpolate_path,
    load_r1_plan,
    minimum_jerk,
)
from robot_control.r2_motion import (
    INITIAL_APPROACH_SPEED_MULTIPLIER as R2_INITIAL_APPROACH_SPEED_MULTIPLIER,
    PLAN_PATH as R2_PLAN_PATH,
    R2_PCB_PLACED,
    load_r2_plan,
)
from robot_control.r3_motion import (
    BOX_ASSEMBLY_POSITION,
    INSPECTION_PRODUCT_POSITION,
    MODULE_ASSEMBLY_POSITION,
    PCB_ASSEMBLY_POSITION,
    R3_MODULE_PLACED,
    R3_PRODUCT_TO_INSPECTION,
    R3MotionController,
    TERMINAL_ASSEMBLY_POSITION,
    load_r3_plan,
)
from robot_control.r4_motion import (
    PLAN_PATH as R4_PLAN_PATH,
    R4_SCREW_DONE,
    R4_WAIT_POINT,
    load_r4_plan,
)
from robot_control.r5_motion import (
    PLAN_PATH as R5_PLAN_PATH,
    R5_SORT_DEFECT_DONE,
    R5_SORT_GOOD_DONE,
    R5_WAIT_POINT,
    WAIT_TO_PICK_APP_START_DELAY_S,
    load_r5_plan,
)
from robot_control.runtime_cartesian import create_command_script
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import PARTS, SCENE_ROOT


R1_BOX_PLACED = "R1_BOX_PLACED"
R1_TERMINAL_PLACED = "R1_TERMINAL_PLACED"
JOINT_SETTLE_TOLERANCE_DEG = 0.16
TRANSFER_SPEED_DEG_S = 45.0
DESCENT_SPEED_DEG_S = 36.0
COLLISION_CHECK_INTERVAL = 6
R1_MIDPOINT_MATCH_TOLERANCE_DEG = 0.02
COORDINATED_ROBOTS = ("R1", "R2", "R3", "R4", "R5")
R5_PREAPPROACH_START_DELAY_S = 0.0
R2_COORDINATED_TRANSFER_SPEED_MULTIPLIER = 1.6
R1_BOX_RETREAT_SPEED_MULTIPLIER = 1.6
R1_TERMINAL_RETURN_SPEED_MULTIPLIER = 2.2
R3_PRODUCT_PICK_APP_SPEED_MULTIPLIER = 0.45
STANDARD_PLACE_APPROACH_LIFT_M = 0.18
STANDARD_PLACE_ALIGNMENT_STEPS = 10


@dataclass
class _Motion:
    robot_id: str
    label: str
    path: list[list[float]]
    peak_speed_rad_s: float
    duration_s: float
    cumulative: list[float]
    start_delay_s: float = 0.0
    payload_handle: int | None = None
    payload_start_position: list[float] | None = None
    payload_target_position: list[float] | None = None
    payload_orientation: list[float] | None = None


def _duration(configs: list[list[float]], peak_speed_rad_s: float) -> float:
    cumulative = cumulative_max_joint_distance(configs)
    total = cumulative[-1]
    if total <= 1e-12:
        raise RuntimeError("motion path has no joint motion")
    return max(0.55, 1.875 * total / peak_speed_rad_s)


def _motion(
    robot_id: str,
    label: str,
    path: list[list[float]],
    peak_speed_rad_s: float,
    start_delay_s: float = 0.0,
    payload_handle: int | None = None,
    payload_start_position: list[float] | None = None,
    payload_target_position: list[float] | None = None,
    payload_orientation: list[float] | None = None,
) -> _Motion:
    return _Motion(
        robot_id=robot_id,
        label=label,
        path=path,
        peak_speed_rad_s=peak_speed_rad_s,
        duration_s=_duration(path, peak_speed_rad_s),
        cumulative=cumulative_max_joint_distance(path),
        start_delay_s=max(0.0, float(start_delay_s)),
        payload_handle=payload_handle,
        payload_start_position=payload_start_position,
        payload_target_position=payload_target_position,
        payload_orientation=payload_orientation,
    )


def _rad_path(path: list[list[float]]) -> list[list[float]]:
    return [[float(value) for value in config] for config in path]


def _joint_gap(first: list[float], second: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(first, second))


class CoordinatedFrontHalfRunner:
    """Drive R1-R5 startup motion in one deterministic CoppeliaSim loop."""

    def __init__(
        self,
        bridge: SimBridge,
        r1_plan_path=R1_PLAN_PATH,
        r2_plan_path=R2_PLAN_PATH,
        r4_plan_path=R4_PLAN_PATH,
        r5_plan_path=R5_PLAN_PATH,
        speed_deg_s: float = TRANSFER_SPEED_DEG_S,
        hold_seconds: float = 0.3,
    ) -> None:
        self.bridge = bridge
        self.r1_plan = load_r1_plan(r1_plan_path)
        self.r2_plan = load_r2_plan(r2_plan_path)
        self.r3_plan = load_r3_plan()
        self.r4_plan = load_r4_plan(r4_plan_path)
        self.r5_plan = load_r5_plan(r5_plan_path)
        self.speed = math.radians(float(speed_deg_s))
        self.r2_initial_speed = math.radians(
            float(speed_deg_s) * R2_INITIAL_APPROACH_SPEED_MULTIPLIER
        )
        self.r2_transfer_speed = math.radians(
            float(speed_deg_s) * R2_COORDINATED_TRANSFER_SPEED_MULTIPLIER
        )
        self.r1_box_retreat_speed = math.radians(
            float(speed_deg_s) * R1_BOX_RETREAT_SPEED_MULTIPLIER
        )
        self.r1_terminal_return_speed = math.radians(
            float(speed_deg_s) * R1_TERMINAL_RETURN_SPEED_MULTIPLIER
        )
        self.r3_product_pick_app_speed = math.radians(
            float(speed_deg_s) * R3_PRODUCT_PICK_APP_SPEED_MULTIPLIER
        )
        self.descent_speed = math.radians(min(float(speed_deg_s) * 0.75, DESCENT_SPEED_DEG_S))
        self.hold_seconds = min(1.0, max(0.0, float(hold_seconds)))
        self.step_index = 0
        self.joints: dict[str, list[int]] = {}
        self.original_max_velocities: dict[str, list[float]] = {}
        self.payloads: dict[str, int | None] = {
            robot_id: None for robot_id in COORDINATED_ROBOTS
        }
        self.payload_pose_locks: dict[str, tuple[int, list[float], list[float]]] = {}
        self.command_script = -1
        self.collision_collections: dict[str, int] = {}

    @property
    def sim(self) -> Any:
        return self.bridge.sim

    def _connect_and_start(self) -> None:
        if not self.bridge.is_connected():
            if not self.bridge.connect(self.bridge.host, self.bridge.port):
                raise RuntimeError(self.bridge.last_error or "cannot connect to CoppeliaSim")
        simulation_was_stopped = (
            self.sim.getSimulationState() == self.sim.simulation_stopped
        )
        for robot_id in COORDINATED_ROBOTS:
            self.joints[robot_id] = self.bridge.get_robot_joint_handles(robot_id)
            self.original_max_velocities[robot_id] = [
                self.sim.getObjectFloatParam(joint, self.sim.jointfloatparam_maxvel)
                for joint in self.joints[robot_id]
            ]
            peak_speed_deg = math.degrees(self.speed)
            if robot_id == "R2":
                peak_speed_deg = max(
                    peak_speed_deg,
                    math.degrees(self.r2_initial_speed),
                    math.degrees(self.r2_transfer_speed),
                )
            elif robot_id == "R1":
                peak_speed_deg = max(
                    peak_speed_deg,
                    math.degrees(self.r1_box_retreat_speed),
                )
            max_velocity = math.radians(max(60.0, peak_speed_deg * 1.35))
            for joint in self.joints[robot_id]:
                self.sim.setObjectFloatParam(joint, self.sim.jointfloatparam_maxvel, max_velocity)
                if simulation_was_stopped:
                    self.sim.setJointPosition(joint, 0.0)
                    self.sim.setJointTargetPosition(joint, 0.0)
        self.command_script = create_command_script(
            self.sim,
            self.sim.getObject(SCENE_ROOT),
            "R123_Runtime_CommandBridge",
        )
        self._create_inter_robot_collections()
        if not self.bridge.start_simulation():
            raise RuntimeError(self.bridge.last_error or "cannot start simulation")
        self.hold(0.5, "coordinated startup")
        self._set_gripper("R1", True)
        self._set_gripper("R3", True)
        self.hold(0.5, "open R1/R3 grippers")

    def _restore(self) -> None:
        for robot_id, joints in self.joints.items():
            for joint, original in zip(joints, self.original_max_velocities.get(robot_id, [])):
                try:
                    self.sim.setObjectFloatParam(joint, self.sim.jointfloatparam_maxvel, original)
                except Exception:
                    pass
        if self.command_script != -1:
            try:
                self.sim.removeObjects([self.command_script])
            except Exception:
                pass
            self.command_script = -1
        for collection in self.collision_collections.values():
            try:
                self.sim.destroyCollection(collection)
            except Exception:
                pass
        self.collision_collections = {}
        self.payload_pose_locks = {}

    def _set_gripper(self, robot_id: str, opened: bool) -> None:
        if not self.bridge.set_gripper(robot_id, opened):
            raise RuntimeError(self.bridge.last_error or f"cannot set {robot_id} gripper")

    def _joint_positions(self, robot_id: str) -> list[float]:
        if self.command_script != -1:
            return [
                float(value)
                for value in self.sim.callScriptFunction(
                    "getJointPositions",
                    self.command_script,
                    self.joints[robot_id],
                )
            ]
        return [float(self.sim.getJointPosition(joint)) for joint in self.joints[robot_id]]

    def _set_targets(self, robot_id: str, config: list[float]) -> None:
        self._set_parallel_targets({robot_id: config})

    def _set_parallel_targets(self, targets_by_robot: dict[str, list[float]]) -> None:
        handles: list[int] = []
        targets: list[float] = []
        for robot_id, config in targets_by_robot.items():
            handles.extend(self.joints[robot_id])
            targets.extend(float(value) for value in config)
        if self.command_script != -1:
            self.sim.callScriptFunction(
                "setJointTargets",
                self.command_script,
                handles,
                targets,
            )
            return
        for joint, value in zip(handles, targets):
            self.sim.setJointTargetPosition(joint, value)

    def _standard_approach_position(
        self,
        standard_position: tuple[float, float, float],
    ) -> list[float]:
        return [
            float(standard_position[0]),
            float(standard_position[1]),
            float(standard_position[2] + STANDARD_PLACE_APPROACH_LIFT_M),
        ]

    def _lock_payload_pose(
        self,
        robot_id: str,
        handle: int,
        position: list[float],
        orientation: list[float] | None = None,
    ) -> None:
        self.payload_pose_locks[robot_id] = (
            int(handle),
            [float(value) for value in position],
            [0.0, 0.0, 0.0] if orientation is None else [float(value) for value in orientation],
        )

    def _clear_payload_pose_lock(self, robot_id: str) -> None:
        self.payload_pose_locks.pop(robot_id, None)

    def _apply_payload_pose_locks(self) -> None:
        for handle, position, orientation in self.payload_pose_locks.values():
            self.sim.setObjectPosition(handle, -1, position)
            self.sim.setObjectOrientation(handle, -1, orientation)

    def _create_inter_robot_collections(self) -> None:
        for collection in self.collision_collections.values():
            try:
                self.sim.destroyCollection(collection)
            except Exception:
                pass
        self.collision_collections = {}
        for robot_id in COORDINATED_ROBOTS:
            robot = self.bridge.get_object_handle(robot_id)
            collection = self.sim.createCollection(1)
            self.sim.addItemToCollection(
                collection,
                self.sim.handle_tree,
                robot,
                0,
            )
            self.collision_collections[robot_id] = collection

    def _shape_collection(self, robot_id: str) -> int:
        cached = self.collision_collections.get(robot_id)
        if cached is not None:
            return cached
        robot = self.bridge.get_object_handle(robot_id)
        collection = self.sim.createCollection(1)
        for shape in self.sim.getObjectsInTree(robot, self.sim.object_shape_type, 0):
            self.sim.addItemToCollection(collection, self.sim.handle_single, shape, 0)
        payload = self.payloads.get(robot_id)
        if payload is not None:
            for shape in self.sim.getObjectsInTree(payload, self.sim.object_shape_type, 0):
                self.sim.addItemToCollection(collection, self.sim.handle_single, shape, 0)
        return collection

    def _check_inter_robot_collisions(self, label: str) -> None:
        collections = {
            robot_id: self._shape_collection(robot_id)
            for robot_id in COORDINATED_ROBOTS
        }
        for first_index, first in enumerate(COORDINATED_ROBOTS):
            for second in COORDINATED_ROBOTS[first_index + 1 :]:
                state, pair = self.sim.checkCollision(
                    collections[first], collections[second]
                )
                if not state:
                    continue
                paths = [self.sim.getObjectAlias(handle, 1) for handle in pair]
                joints_deg = {
                    robot_id: [
                        round(math.degrees(value), 3)
                        for value in self._joint_positions(robot_id)
                    ]
                    for robot_id in (first, second)
                }
                raise RuntimeError(
                    f"inter-robot collision during {label}: "
                    f"{first}-{second}: {paths}; joints_deg={joints_deg}"
                )

    def step(self, label: str, force_collision: bool = False) -> None:
        if not self.bridge.step():
            raise RuntimeError(self.bridge.last_error or "CoppeliaSim step failed")
        self._apply_payload_pose_locks()
        self.step_index += 1
        if force_collision or self.step_index % COLLISION_CHECK_INTERVAL == 0:
            self._check_inter_robot_collisions(label)

    def hold(self, seconds: float, label: str) -> None:
        dt = float(self.sim.getSimulationTimeStep())
        for _ in range(max(1, math.ceil(seconds / dt))):
            self.step(label)

    def run_parallel(self, motions: list[_Motion]) -> None:
        dt = float(self.sim.getSimulationTimeStep())
        max_duration = max(
            motion.start_delay_s + motion.duration_s for motion in motions
        )
        steps = max(2, math.ceil(max_duration / dt))
        for index in range(1, steps + 1):
            elapsed = min(max_duration, index * dt)
            targets_by_robot = {}
            for motion in motions:
                active_elapsed = elapsed - motion.start_delay_s
                if active_elapsed <= 0.0:
                    targets_by_robot[motion.robot_id] = motion.path[0]
                    continue
                fraction = min(1.0, active_elapsed / motion.duration_s)
                progress = minimum_jerk(fraction)
                total = motion.cumulative[-1]
                target = interpolate_path(motion.path, motion.cumulative, total * progress)
                targets_by_robot[motion.robot_id] = target
                if (
                    motion.payload_handle is not None
                    and motion.payload_start_position is not None
                    and motion.payload_target_position is not None
                ):
                    payload_position = [
                        start + (finish - start) * progress
                        for start, finish in zip(
                            motion.payload_start_position,
                            motion.payload_target_position,
                        )
                    ]
                    self._lock_payload_pose(
                        motion.robot_id,
                        motion.payload_handle,
                        payload_position,
                        motion.payload_orientation,
                    )
            self._set_parallel_targets(targets_by_robot)
            labels = ",".join(motion.label for motion in motions)
            self.step(labels)
        for motion in motions:
            self._settle(motion.robot_id, motion.path[-1], motion.label)

    def _align_payload_above_standard(
        self,
        robot_id: str,
        handle: int,
        standard_position: tuple[float, float, float],
        label: str,
    ) -> list[float]:
        target = self._standard_approach_position(standard_position)
        start = [float(value) for value in self.sim.getObjectPosition(handle, -1)]
        for index in range(1, STANDARD_PLACE_ALIGNMENT_STEPS + 1):
            progress = minimum_jerk(index / STANDARD_PLACE_ALIGNMENT_STEPS)
            position = [
                before + (after - before) * progress
                for before, after in zip(start, target)
            ]
            self._lock_payload_pose(robot_id, handle, position)
            self.step(label, force_collision=index == STANDARD_PLACE_ALIGNMENT_STEPS)
        self._lock_payload_pose(robot_id, handle, target)
        return target

    def _place_descend_motion(
        self,
        robot_id: str,
        label: str,
        path: list[list[float]],
        handle: int,
        standard_position: tuple[float, float, float],
    ) -> _Motion:
        return _motion(
            robot_id,
            label,
            path,
            self.descent_speed,
            payload_handle=handle,
            payload_start_position=self._standard_approach_position(standard_position),
            payload_target_position=[float(value) for value in standard_position],
            payload_orientation=[0.0, 0.0, 0.0],
        )

    def _settle(self, robot_id: str, expected: list[float], label: str) -> None:
        tolerance = math.radians(JOINT_SETTLE_TOLERANCE_DEG)
        self._set_targets(robot_id, expected)
        for _ in range(120):
            current = self._joint_positions(robot_id)
            if max(abs(a - b) for a, b in zip(current, expected)) <= tolerance:
                return
            self.step(f"{label} settle")
        raise RuntimeError(f"{robot_id} did not settle after {label}")

    def _attach(self, object_name: str, robot_id: str) -> int:
        handle = self.bridge.get_object_handle(object_name)
        self.bridge.attach_object(object_name, robot_id)
        self.payloads[robot_id] = handle
        self.step(f"{robot_id} attached {object_name}", force_collision=True)
        return handle

    def _detach_to_parts(
        self,
        handle: int,
        robot_id: str,
        position: tuple[float, float, float],
    ) -> None:
        # Put the visible payload at its canonical release pose before
        # reparenting it. This avoids a rendered parent-change frame at the
        # old gripper-relative pose.
        self.sim.setObjectPosition(handle, -1, list(position))
        self.sim.setObjectOrientation(handle, -1, [0.0, 0.0, 0.0])
        self.bridge.detach_object(handle)
        self.payloads[robot_id] = None
        self.sim.setObjectPosition(handle, -1, list(position))
        self.sim.setObjectOrientation(handle, -1, [0.0, 0.0, 0.0])
        self.step(f"{robot_id} detached payload", force_collision=True)
        self._clear_payload_pose_lock(robot_id)

    def _paths(self) -> dict[str, dict[str, list[list[float]]]]:
        r1 = {name: _rad_path(path) for name, path in self.r1_plan["paths"].items()}
        r2 = {name: _rad_path(path) for name, path in self.r2_plan["paths"].items()}
        r3_module = {
            name: _rad_path(path)
            for name, path in self.r3_plan["paths"][R3_MODULE_PLACED].items()
        }
        r3_product = {
            name: _rad_path(path)
            for name, path in self.r3_plan["paths"][R3_PRODUCT_TO_INSPECTION].items()
        }
        r4 = {
            name: _rad_path(path)
            for name, path in self.r4_plan["paths"].items()
        }
        r5_good = {
            name: _rad_path(path)
            for name, path in self.r5_plan["paths"][R5_SORT_GOOD_DONE].items()
        }
        r5_defect = {
            name: _rad_path(path)
            for name, path in self.r5_plan["paths"][R5_SORT_DEFECT_DONE].items()
        }
        return {
            "R1": r1,
            "R2": r2,
            "R3_MODULE": r3_module,
            "R3_PRODUCT": r3_product,
            "R4": r4,
            "R5_GOOD": r5_good,
            "R5_DEFECT": r5_defect,
        }

    @staticmethod
    def _remaining_after_lift(full_path: list[list[float]], descend_path: list[list[float]]) -> list[list[float]]:
        # lift_and_transfer begins with reversed descend_path. Keep the point at
        # pick APP, then continue to place APP.
        return [list(config) for config in full_path[len(descend_path) - 1 :]]

    @staticmethod
    def _reverse_path(path: list[list[float]]) -> list[list[float]]:
        return [list(config) for config in reversed(path)]

    @staticmethod
    def _split_path_at_config(
        path: list[list[float]],
        config: list[float],
        label: str,
    ) -> tuple[list[list[float]], list[list[float]]]:
        closest_index, closest_gap = min(
            ((index, _joint_gap(point, config)) for index, point in enumerate(path)),
            key=lambda item: item[1],
        )
        if closest_gap > math.radians(R1_MIDPOINT_MATCH_TOLERANCE_DEG):
            raise RuntimeError(
                f"cannot split {label} at configured midpoint; closest gap is "
                f"{math.degrees(closest_gap):.6f} deg"
            )
        return (
            [list(point) for point in path[: closest_index + 1]],
            [list(point) for point in path[closest_index:]],
        )

    def _r1_mid1_config(self) -> list[float]:
        try:
            values = self.r1_plan["validation"]["r1_return_avoidance"]["mid1_rad"]
        except KeyError as exc:
            raise RuntimeError("R1 return avoidance midpoint is missing from the plan") from exc
        if not isinstance(values, list) or len(values) != 6:
            raise RuntimeError("R1 return avoidance midpoint is invalid")
        return [float(value) for value in values]

    def execute(self) -> dict[str, Any]:
        started_wall = time.time()
        paths = self._paths()
        r1 = paths["R1"]
        r2 = paths["R2"]
        r3m = paths["R3_MODULE"]
        r3p = paths["R3_PRODUCT"]
        r4 = paths["R4"]
        r5 = paths["R5_GOOD"]
        r1_mid1 = self._r1_mid1_config()
        r1_box_retreat_to_mid1, r1_mid1_to_terminal_pick_app = self._split_path_at_config(
            r1["box_retreat_and_terminal_approach"],
            r1_mid1,
            "R1 box_retreat_and_terminal_approach",
        )
        r1_terminal_lift_to_mid1, r1_mid1_to_terminal_place_app = self._split_path_at_config(
            r1["terminal_lift_and_transfer"],
            r1_mid1,
            "R1 terminal_lift_and_transfer",
        )
        r3_module_lift_to_pick_app = self._reverse_path(r3m["pick_descend"])

        try:
            self._connect_and_start()

            # R1/R2/R3 pick their private parts at startup. R4 and R5 move
            # into their taught wait points in the same first visible batch.
            self.run_parallel(
                [
                    _motion("R1", "R1 initial_to_box_pick_app", r1["initial_to_box_pick_app"], self.speed),
                    _motion("R2", "R2 initial_to_pick_app", r2["initial_to_pick_app"], self.r2_initial_speed),
                    _motion("R3", "R3 initial_to_module_pick_app", r3m["initial_to_pick_app"], self.speed),
                    _motion(
                        "R4",
                        "R4 home_to_wait",
                        r4["home_to_wait"],
                        self.speed,
                    ),
                    _motion(
                        "R5",
                        "R5 home_to_wait",
                        r5["home_to_wait"],
                        self.speed,
                        start_delay_s=R5_PREAPPROACH_START_DELAY_S,
                    ),
                ]
            )
            self.hold(self.hold_seconds, "hold at pick/pre-approach postures")

            self.run_parallel(
                [
                    _motion("R1", "R1 box_descend", r1["box_descend"], self.descent_speed),
                    _motion("R2", "R2 pick_descend", r2["pick_descend"], self.speed),
                    _motion("R3", "R3 module_pick_descend", r3m["pick_descend"], self.descent_speed),
                ]
            )
            self._set_gripper("R1", False)
            self._set_gripper("R3", False)
            self.hold(0.6, "close R1/R3 grippers and R2 suction settle")
            box = self._attach("BOX_BLANK", "R1")
            pcb = self._attach("PCB_SUPPLY", "R2")
            module = self._attach("CONTROL_MODULE_SUPPLY", "R3")

            # R2 leaves the interference area immediately after PCB suction.
            # R1 starts the box transfer in the same coordinated batch so the
            # front half does not visually stall at the handoff.
            self.run_parallel(
                [
                    _motion("R1", "R1 box_lift_and_transfer", r1["box_lift_and_transfer"], self.speed),
                    _motion("R2", "R2 pick_tcp_to_safe_wait", r2["pick_tcp_to_safe_wait"], self.r2_transfer_speed),
                    _motion("R3", "R3 module_lift_to_pick_app", r3_module_lift_to_pick_app, self.descent_speed),
                ]
            )
            self.hold(self.hold_seconds, "R2 safe wait while R1 places box")

            self._align_payload_above_standard(
                "R1",
                box,
                BOX_ASSEMBLY_POSITION,
                "R1 align box above assembly standard",
            )
            self.run_parallel([
                self._place_descend_motion(
                    "R1",
                    "R1 box_place_descend_to_standard",
                    r1["box_place_descend"],
                    box,
                    BOX_ASSEMBLY_POSITION,
                )
            ])
            self._set_gripper("R1", True)
            self.hold(0.5, "R1 release box")
            self._detach_to_parts(box, "R1", BOX_ASSEMBLY_POSITION)
            self.run_parallel(
                [
                    _motion("R2", "R2 safe_wait_to_place_app", r2["safe_wait_to_place_app"], self.r2_transfer_speed),
                    _motion(
                        "R1",
                        "R1 box_retreat_to_mid1",
                        r1_box_retreat_to_mid1,
                        self.r1_box_retreat_speed,
                    ),
                ]
            )
            self.run_parallel([
                _motion(
                    "R1",
                    "R1 mid1_to_terminal_pick_app",
                    r1_mid1_to_terminal_pick_app,
                    self.speed,
                )
            ])
            self._align_payload_above_standard(
                "R2",
                pcb,
                PCB_ASSEMBLY_POSITION,
                "R2 align PCB above assembly standard",
            )
            self.run_parallel(
                [
                    _motion("R1", "R1 terminal_descend", r1["terminal_descend"], self.descent_speed),
                    self._place_descend_motion(
                        "R2",
                        "R2 place_descend_to_standard",
                        r2["place_descend"],
                        pcb,
                        PCB_ASSEMBLY_POSITION,
                    ),
                ]
            )
            self._detach_to_parts(pcb, "R2", PCB_ASSEMBLY_POSITION)
            self._set_gripper("R1", False)
            self.hold(0.55, "R1 close terminal gripper")
            terminal = self._attach("TERMINAL_BLOCK_SUPPLY", "R1")

            # R3 begins module installation as soon as the PCB is released;
            # R1 retreats with the terminal and R2 leaves the shared area.
            module_transfer_from_app = self._remaining_after_lift(
                r3m["lift_and_transfer"],
                r3m["pick_descend"],
            )
            self.run_parallel(
                [
                    _motion("R1", "R1 terminal_lift_to_mid1", r1_terminal_lift_to_mid1, self.speed),
                    _motion("R2", "R2 return_to_pick_app_standby", r2["return_home"], self.r2_transfer_speed),
                    _motion("R3", "R3 module_pick_app_to_place_app", module_transfer_from_app, self.speed),
                ]
            )

            self._align_payload_above_standard(
                "R3",
                module,
                MODULE_ASSEMBLY_POSITION,
                "R3 align module above assembly standard",
            )
            self.run_parallel([
                self._place_descend_motion(
                    "R3",
                    "R3 module_place_descend_to_standard",
                    r3m["place_descend"],
                    module,
                    MODULE_ASSEMBLY_POSITION,
                )
            ])
            self._set_gripper("R3", True)
            self.hold(0.45, "R3 release module")
            self._detach_to_parts(module, "R3", MODULE_ASSEMBLY_POSITION)

            # R1 starts terminal installation while R3 clears. If this exposes
            # a real collision, keep the overlap and tune that segment next.
            self.run_parallel(
                [
                    _motion("R3", "R3 retreat_to_terminal_clearance", r3m["retreat_to_clear"], self.speed),
                    _motion(
                        "R1",
                        "R1 mid1_to_terminal_place_app",
                        r1_mid1_to_terminal_place_app,
                        self.speed,
                    ),
                ]
            )
            self._align_payload_above_standard(
                "R1",
                terminal,
                TERMINAL_ASSEMBLY_POSITION,
                "R1 align terminal above assembly standard",
            )
            self.run_parallel([
                self._place_descend_motion(
                    "R1",
                    "R1 terminal_place_descend_to_standard",
                    r1["terminal_place_descend"],
                    terminal,
                    TERMINAL_ASSEMBLY_POSITION,
                )
            ])
            self._set_gripper("R1", True)
            self.hold(0.45, "R1 release terminal")
            self._detach_to_parts(terminal, "R1", TERMINAL_ASSEMBLY_POSITION)

            # R3 starts moving to the product pick APP immediately after the
            # terminal is released while R1 clears the assembly side.
            self.run_parallel(
                [
                    _motion(
                        "R1",
                        "R1 return_home",
                        r1["return_home"],
                        self.r1_terminal_return_speed,
                    ),
                    _motion(
                        "R3",
                        "R3 clear_to_product_pick_app",
                        r3p["clear_to_pick_app"],
                        self.r3_product_pick_app_speed,
                    ),
                ]
            )

            r3_controller = R3MotionController(
                self.bridge,
                assembly_lock=None,
                inspection_lock=None,
                speed_deg_s=math.degrees(self.speed),
                hold_seconds=self.hold_seconds,
            )
            r3_controller.set_continuous_stepping(True)
            r3_controller.set_pre_positioned(
                R3_PRODUCT_TO_INSPECTION,
                r3p["clear_to_pick_app"][-1],
            )
            product_result = r3_controller.execute(R3_PRODUCT_TO_INSPECTION)

            return {
                "status": "finished",
                "actions": [
                    R1_BOX_PLACED,
                    R2_PCB_PLACED,
                    R3_MODULE_PLACED,
                    R1_TERMINAL_PLACED,
                    R3_PRODUCT_TO_INSPECTION,
                ],
                "product_transfer": product_result,
                "prepositioned_configs": {
                    "R4": {
                        "point": R4_WAIT_POINT,
                        "action": R4_SCREW_DONE,
                        "config": r4["home_to_wait"][-1],
                    },
                    "R5": {
                        "point": R5_WAIT_POINT,
                        "config": r5["home_to_wait"][-1],
                    },
                },
                "r2_initial_approach_speed_multiplier": (
                    R2_INITIAL_APPROACH_SPEED_MULTIPLIER
                ),
                "r2_coordinated_transfer_speed_multiplier": (
                    R2_COORDINATED_TRANSFER_SPEED_MULTIPLIER
                ),
                "r1_box_retreat_speed_multiplier": (
                    R1_BOX_RETREAT_SPEED_MULTIPLIER
                ),
                "r1_terminal_return_speed_multiplier": (
                    R1_TERMINAL_RETURN_SPEED_MULTIPLIER
                ),
                "r3_product_pick_app_speed_multiplier": (
                    R3_PRODUCT_PICK_APP_SPEED_MULTIPLIER
                ),
                "standard_place_approach_lift_m": STANDARD_PLACE_APPROACH_LIFT_M,
                "descent_speed_cap_deg_s": DESCENT_SPEED_DEG_S,
                "r5_preapproach_start_delay_s": R5_PREAPPROACH_START_DELAY_S,
                "r5_preapproach_skipped_for_r3_clearance": False,
                "r1_return_overlapped_with_r3_product_pick_app": True,
                "r4_wait_point_deg": [
                    round(math.degrees(value), 6)
                    for value in r4["home_to_wait"][-1]
                ],
                "simulation_time_s": float(self.sim.getSimulationTime()),
                "wall_duration_s": time.time() - started_wall,
                "inspection_product_position": list(INSPECTION_PRODUCT_POSITION),
            }
        finally:
            self._restore()


__all__ = ["CoordinatedFrontHalfRunner"]
