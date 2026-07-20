"""Shared runtime Cartesian primitives for the current-scene R3/R5 tasks.

R1, R2, and R4 keep their accepted implementations unchanged.  This module
contains the small amount of common machinery needed by the new R3 and R5
controllers: runtime TCP creation, simIK pose paths, minimum-jerk replay, and
scene-native collision/workspace checks.
"""

from __future__ import annotations

import bisect
import hashlib
import itertools
import math
from pathlib import Path
from typing import Any, Iterable, Optional

from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import ROBOT_BASES, WORKSPACES


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


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def near(first: Iterable[float], second: Iterable[float], tolerance: float) -> bool:
    first_values = list(first)
    second_values = list(second)
    return len(first_values) == len(second_values) and max(
        abs(a - b) for a, b in zip(first_values, second_values)
    ) <= tolerance


def find_unique_alias(sim: Any, root: int, alias: str) -> int:
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


def wrap_near(reference: float, value: float) -> float:
    while value - reference > math.pi:
        value -= 2.0 * math.pi
    while value - reference < -math.pi:
        value += 2.0 * math.pi
    return value


def unwrap_path(configs: list[list[float]]) -> list[list[float]]:
    if not configs:
        return []
    result = [list(configs[0])]
    for config in configs[1:]:
        result.append(
            [
                wrap_near(previous, value)
                for previous, value in zip(result[-1], config)
            ]
        )
    return result


def interpolate_joint_line(
    first: list[float], second: list[float], count: int
) -> list[list[float]]:
    if count < 2:
        raise ValueError("joint-line point count must be at least two")
    return [
        [
            start + (finish - start) * index / (count - 1)
            for start, finish in zip(first, second)
        ]
        for index in range(count)
    ]


def join_paths(*paths: list[list[float]]) -> list[list[float]]:
    result: list[list[float]] = []
    for path in paths:
        if not path:
            raise RuntimeError("cannot join an empty runtime path")
        current = unwrap_path(path)
        if result:
            current = [
                [
                    wrap_near(previous, value)
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
                    "runtime joined-path discontinuity "
                    f"{math.degrees(discontinuity):.3f} deg"
                )
            current = current[1:]
        result.extend(current)
    return result


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


def shape_tree_bounds(
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


def solve_target(
    sim_ik: Any,
    base: int,
    tip: int,
    joints: list[int],
    target: int,
    seed: list[float],
    label: str,
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
                f"IK failed for {label}: "
                f"linear={float(precision[0]) * 1000.0:.3f} mm, "
                f"angular={math.degrees(float(precision[1])):.3f} deg"
            )
        return [
            float(sim_ik.getJointPosition(environment, scene_to_ik[joint]))
            for joint in joints
        ]
    finally:
        sim_ik.eraseEnvironment(environment)


def generate_cartesian_path(
    sim_ik: Any,
    base: int,
    tip: int,
    joints: list[int],
    target: int,
    start: list[float],
    point_count: int,
    label: str,
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
            raise RuntimeError(f"Cartesian path generation failed for {label}")
        return unwrap_path(
            [
                [float(value) for value in flat[index : index + len(joints)]]
                for index in range(0, len(flat), len(joints))
            ]
        )
    finally:
        sim_ik.eraseEnvironment(environment)


def create_pose_dummy(
    sim: Any,
    alias: str,
    position: Iterable[float],
    orientation_deg: Iterable[float],
) -> int:
    target = sim.createDummy(0.004)
    sim.setObjectAlias(target, alias)
    sim.setObjectPosition(target, -1, list(position))
    sim.setObjectOrientation(
        target, -1, [math.radians(value) for value in orientation_deg]
    )
    sim.setObjectInt32Param(target, sim.objintparam_visibility_layer, 0)
    return target


def build_pick_place_paths(
    sim: Any,
    sim_ik: Any,
    robot: int,
    tip: int,
    joints: list[int],
    prefix: str,
    pick_app_position: Iterable[float],
    pick_tcp_position: Iterable[float],
    place_app_position: Iterable[float],
    place_tcp_position: Iterable[float],
    pick_orientation_deg: Iterable[float],
    place_orientation_deg: Iterable[float],
    transfer_waypoints: list[dict[str, Any]],
) -> dict[str, list[list[float]]]:
    """Build a deterministic pose chain while leaving scene targets untouched."""
    base = find_unique_alias(sim, robot, "base_link_respondable")
    temporary: list[int] = []
    try:
        pick_app = create_pose_dummy(
            sim, f"{prefix}Target_Pick_APP", pick_app_position, pick_orientation_deg
        )
        pick_tcp = create_pose_dummy(
            sim, f"{prefix}Target_Pick_TCP", pick_tcp_position, pick_orientation_deg
        )
        place_app = create_pose_dummy(
            sim,
            f"{prefix}Target_Place_APP",
            place_app_position,
            place_orientation_deg,
        )
        place_tcp = create_pose_dummy(
            sim,
            f"{prefix}Target_Place_TCP",
            place_tcp_position,
            place_orientation_deg,
        )
        temporary.extend([pick_app, pick_tcp, place_app, place_tcp])

        pick_app_config = solve_target(
            sim_ik, base, tip, joints, pick_app, [0.0] * 6, "pick APP"
        )
        pick_tcp_config = solve_target(
            sim_ik, base, tip, joints, pick_tcp, pick_app_config, "pick TCP"
        )
        pick_descend = generate_cartesian_path(
            sim_ik,
            base,
            tip,
            joints,
            pick_tcp,
            pick_app_config,
            51,
            "pick descent",
        )
        # generatePath may converge to a numerically different but continuous
        # joint representation of the same pose.  Its endpoint, not the
        # standalone reachability solve, is the seed for the following path.
        pick_tcp_config = pick_descend[-1]

        chain_targets: list[tuple[int, str, int]] = []
        for index, waypoint in enumerate(transfer_waypoints, start=1):
            handle = create_pose_dummy(
                sim,
                f"{prefix}Target_Waypoint_{index}",
                waypoint["position"],
                waypoint["orientation_deg"],
            )
            temporary.append(handle)
            chain_targets.append(
                (handle, str(waypoint.get("name", f"waypoint {index}")), int(waypoint.get("points", 51)))
            )
        chain_targets.append((place_app, "place APP", 81))

        transfer_segments: list[list[list[float]]] = []
        previous_config = pick_app_config
        for handle, label, point_count in chain_targets:
            solved = solve_target(
                sim_ik, base, tip, joints, handle, previous_config, label
            )
            segment = generate_cartesian_path(
                sim_ik,
                base,
                tip,
                joints,
                handle,
                previous_config,
                point_count,
                label,
            )
            transfer_segments.append(segment)
            previous_config = segment[-1]

        place_tcp_config = solve_target(
            sim_ik, base, tip, joints, place_tcp, previous_config, "place TCP"
        )
        place_descend = generate_cartesian_path(
            sim_ik,
            base,
            tip,
            joints,
            place_tcp,
            previous_config,
            51,
            "place descent",
        )
        place_tcp_config = place_descend[-1]

        initial = interpolate_joint_line([0.0] * 6, pick_app_config, 101)
        transfer = join_paths(*transfer_segments)
        result = {
            "initial_to_pick_app": initial,
            "pick_descend": pick_descend,
            "lift_and_transfer": join_paths(
                list(reversed(pick_descend)), transfer
            ),
            "place_descend": place_descend,
            "return_home": join_paths(
                list(reversed(place_descend)),
                list(reversed(transfer)),
                list(reversed(initial)),
            ),
        }
        if transfer_waypoints:
            result["lift_to_pick_app"] = list(reversed(pick_descend))
            result["transfer_to_first_waypoint"] = transfer_segments[0]
            result["transfer_after_first_waypoint"] = join_paths(
                *transfer_segments[1:]
            )
            for index, segment in enumerate(transfer_segments, start=1):
                result[f"transfer_segment_{index}"] = segment
        return result
    finally:
        for handle in reversed(temporary):
            try:
                sim.removeObjects([handle])
            except Exception:
                pass


def create_virtual_tcp(
    sim: Any,
    robot: int,
    alias: str,
    offset_m: float,
) -> int:
    link6 = find_unique_alias(sim, robot, "Link6_visual")
    tip = sim.createDummy(0.004)
    sim.setObjectAlias(tip, alias)
    sim.setObjectParent(tip, link6, False)
    sim.setObjectPose(
        tip,
        link6,
        [0.0, 0.0, float(offset_m), 0.0, 0.0, 0.0, 1.0],
    )
    sim.setObjectInt32Param(tip, sim.objintparam_visibility_layer, 0)
    return tip


def create_command_script(sim: Any, robot: int, alias: str) -> int:
    for handle in sim.getObjectsInTree(robot, sim.object_script_type, 0):
        if sim.getObjectAlias(handle) == alias:
            sim.removeObjects([handle])
    script = sim.createScript(
        sim.scripttype_simulation,
        RUNTIME_BRIDGE_CODE,
        0,
        "lua",
    )
    sim.setObjectAlias(script, alias)
    sim.setObjectParent(script, robot, True)
    return script


def remove_runtime_objects(sim: Any, robot: int, prefix: str) -> None:
    stale = {
        handle
        for handle in sim.getObjectsInTree(robot, sim.handle_all, 0)
        if sim.getObjectAlias(handle).startswith(prefix)
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


def tree_visibility(sim: Any, root: int) -> dict[int, int]:
    return {
        handle: int(
            sim.getObjectInt32Param(handle, sim.objintparam_visibility_layer)
        )
        for handle in sim.getObjectsInTree(root, sim.object_shape_type, 0)
    }


def set_visibility(sim: Any, layers: dict[int, int], visible: bool) -> None:
    layer = 1 if visible else 0
    for handle in layers:
        sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, layer)


def restore_visibility(sim: Any, layers: dict[int, int]) -> None:
    for handle, layer in layers.items():
        sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, layer)


class RobotSafetyGuard:
    """Scene-native environment, self, payload, and workspace checks."""

    def __init__(
        self,
        sim: Any,
        robot: int,
        robot_id: str,
        payload: Optional[int] = None,
        ignored_environment: Optional[set[int]] = None,
        allowed_payload_contacts: Optional[set[int]] = None,
    ):
        self.sim = sim
        self.robot_id = robot_id
        self.payload = payload
        ignored_environment = set(ignored_environment or ())
        self.allowed_payload_contacts = set(allowed_payload_contacts or ())
        self.payload_shapes = (
            set(sim.getObjectsInTree(payload, sim.object_shape_type, 0))
            if payload is not None
            else set()
        )
        self.robot_shapes = set(
            sim.getObjectsInTree(robot, sim.object_shape_type, 0)
        ) - self.payload_shapes
        self.moving_shapes = self.robot_shapes | self.payload_shapes
        self.mover = sim.createCollection(1)
        self.environment = sim.createCollection(1)
        for handle in self.moving_shapes:
            sim.addItemToCollection(self.mover, sim.handle_single, handle, 0)

        self.robot_mover: Optional[int] = None
        self.allowed_contact_environment: Optional[int] = None
        if self.allowed_payload_contacts:
            self.robot_mover = sim.createCollection(1)
            self.allowed_contact_environment = sim.createCollection(1)
            for handle in self.robot_shapes:
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

        robot_base = sim.getObject(ROBOT_BASES[robot_id])
        for handle in sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        ):
            if (
                handle in self.moving_shapes
                or handle == robot_base
                or handle in ignored_environment
            ):
                continue
            if sim.getObjectInt32Param(
                handle, sim.objintparam_visibility_layer
            ) == 0:
                continue
            if handle in self.allowed_payload_contacts:
                continue
            sim.addItemToCollection(
                self.environment, sim.handle_single, handle, 0
            )

        aliases = ["base_link_respondable"] + [
            f"Link{index}_respondable" for index in range(1, 7)
        ]
        by_alias = {
            sim.getObjectAlias(handle): handle for handle in self.robot_shapes
        }
        missing = [alias for alias in aliases if alias not in by_alias]
        if missing:
            raise RuntimeError(f"{robot_id} collision links missing: {missing}")
        self.links = [by_alias[alias] for alias in aliases]

        self.payload_collection: Optional[int] = None
        self.arm_collection: Optional[int] = None
        if self.payload_shapes:
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
            handle: sim.getShapeBB(handle) for handle in self.moving_shapes
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
                            f"{self.robot_id} self collision during {label}: {paths}"
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
                        f"payload-to-{self.robot_id} collision during {label}: {paths}"
                    )

        if not check_workspace:
            return
        lower, upper = shape_tree_bounds(
            self.sim, self.moving_shapes, self.shape_bbs
        )
        allowed = WORKSPACES[self.robot_id]
        tolerance = 0.003
        for axis, actual_low, actual_high, allowed_low, allowed_high in zip(
            "xyz", lower, upper, allowed["lower"], allowed["upper"]
        ):
            if (
                actual_low < allowed_low - tolerance
                or actual_high > allowed_high + tolerance
            ):
                raise RuntimeError(
                    f"{self.robot_id} workspace violation during {label}: "
                    f"axis={axis}, actual=[{actual_low:.4f},{actual_high:.4f}], "
                    f"allowed=[{allowed_low:.4f},{allowed_high:.4f}]"
                )


class SmoothRunner:
    """Minimum-jerk replay with periodic scene-native safety checks."""

    def __init__(
        self,
        bridge: SimBridge,
        robot: int,
        robot_id: str,
        joints: list[int],
        command_script: int,
        payload: int,
        ignored_environment: Optional[set[int]] = None,
        collision_check_interval: int = 5,
        workspace_check_interval: int = 20,
    ):
        self.bridge = bridge
        self.sim = bridge.sim
        self.joints = joints
        self.command_script = command_script
        self.collision_check_interval = max(1, collision_check_interval)
        self.workspace_check_interval = max(1, workspace_check_interval)
        self.dt = float(self.sim.getSimulationTimeStep())
        self.guards = {
            None: RobotSafetyGuard(
                self.sim,
                robot,
                robot_id,
                ignored_environment=ignored_environment,
            ),
            payload: RobotSafetyGuard(
                self.sim,
                robot,
                robot_id,
                payload,
                ignored_environment=ignored_environment,
            ),
        }
        self.guard = self.guards[None]
        self.payload_world_orientation_lock: Optional[list[float]] = None
        self.step_index = 0

    def close(self) -> None:
        for guard in self.guards.values():
            guard.close()

    def set_payload(self, payload: Optional[int]) -> None:
        self.guard = self.guards[payload]

    def lock_payload_world_orientation(
        self, orientation_euler: Optional[Iterable[float]]
    ) -> None:
        self.payload_world_orientation_lock = (
            None if orientation_euler is None else list(orientation_euler)
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
                self.bridge.last_error or "runtime simulation step failed"
            )
        if (
            self.payload_world_orientation_lock is not None
            and self.guard.payload is not None
        ):
            self.sim.setObjectOrientation(
                self.guard.payload,
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
        try:
            self.guard.check(
                label,
                check_workspace=full_due,
                check_internal=full_due,
            )
        except RuntimeError as exc:
            joints_deg = [
                round(math.degrees(value), 3) for value in self.joint_positions()
            ]
            payload_position = (
                [
                    round(float(value), 4)
                    for value in self.sim.getObjectPosition(
                        self.guard.payload, -1
                    )
                ]
                if self.guard.payload is not None
                else None
            )
            payload_pose = (
                [
                    round(float(value), 5)
                    for value in self.sim.getObjectPose(
                        self.guard.payload, -1
                    )
                ]
                if self.guard.payload is not None
                else None
            )
            raise RuntimeError(
                f"{exc}; joints_deg={joints_deg}; "
                f"payload_position={payload_position}; payload_pose={payload_pose}"
            ) from exc

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
            self.step(f"{label} [{index}/{step_count}]")

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

    def animate_world_offset(
        self,
        payload: int,
        offset: Iterable[float],
        steps: int,
        label: str,
    ) -> None:
        start = self.sim.getObjectPosition(payload, -1)
        delta = list(offset)
        for index in range(1, steps + 1):
            self.sim.setObjectPosition(
                payload,
                -1,
                [
                    start[axis] + delta[axis] * index / steps
                    for axis in range(3)
                ],
            )
            self.step(label, force_collision=True)

    def animate_world_orientation(
        self,
        payload: int,
        target_euler: Iterable[float],
        steps: int,
        label: str,
    ) -> None:
        start = list(self.sim.getObjectOrientation(payload, -1))
        target = list(target_euler)
        delta = [wrap_near(first, second) - first for first, second in zip(start, target)]
        for index in range(1, steps + 1):
            self.sim.setObjectOrientation(
                payload,
                -1,
                [
                    start[axis] + delta[axis] * index / steps
                    for axis in range(3)
                ],
            )
            self.step(label, force_collision=True)


__all__ = [
    "RobotSafetyGuard",
    "SmoothRunner",
    "build_pick_place_paths",
    "create_command_script",
    "create_virtual_tcp",
    "find_unique_alias",
    "near",
    "remove_runtime_objects",
    "restore_visibility",
    "set_visibility",
    "sha256_file",
    "tree_visibility",
]
