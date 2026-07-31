#!/usr/bin/env python3
"""CR5 RViz pick-and-place demo using MoveIt2 + OMPL.

Start the simulator first:

    ros2 launch cr5_moveit demo.launch.py

Then run:

    python3 robot_control/pick_and_place.py

Default mode keeps one fixed downward tool orientation and plans the same
deterministic Cartesian sequence used by the step-by-step RViz test:

    approach A -> hold -> down -> lift + straight A-to-B -> hold -> down -> release

If the current tool orientation is unsuitable, OMPL is used once to reach a
fixed HOME state. MoveIt/FCL checks the robot, attached block, and the final
smoothed controller trajectories. The old sampled-IK approach is available
only through --sampled-approach.
"""

from __future__ import annotations

import argparse
import copy
import math
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker


FRAME = "dummy_link"
GROUP = "cr5_group"
EE_LINK = "gripper_base"
CONTROLLER_ACTION = "/cr5_group_controller/follow_joint_trajectory"

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT = "finger_left_joint"
CONTROL_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]

HOME_ARM = [0.0, 0.4337, -1.4695, -0.2602, 1.7175, 0.0]
GRIPPER_OPEN = 0.04
GRIPPER_CLOSED = 0.005

# The target points are expressed as gripper_base positions in dummy_link.
PICK_POSE = (0.40, -0.25, 0.50)
PLACE_POSE = (0.35, 0.30, 0.50)
BLOCK_SIZE = 0.05
BLOCK_GRIPPER_OFFSET_Z = -0.075
# Keep a small clearance from the stand while still collision-checking the payload.
BLOCK_COLLISION_SIZE = 0.045
PAYLOAD_TOUCH_LINKS = [EE_LINK, "finger_left", "finger_right", "Link6"]

DEFAULT_SAFE_ZS = [0.56, 0.62, 0.70]
DEFAULT_PLANNERS = ["RRTstar", "RRTConnect"]
DEFAULT_ENTRY_PLANNERS = ["RRTConnect", "RRTstar"]


@dataclass(frozen=True)
class SegmentTarget:
    name: str
    kind: str
    target: tuple[float, ...]


@dataclass
class PlannedSegment:
    name: str
    trajectory: JointTrajectory
    start_positions: dict[str, float]
    end_positions: dict[str, float]
    length: float


@dataclass
class PlanCandidate:
    planner_id: str
    safe_z: float
    segments: list[PlannedSegment]
    length: float


@dataclass
class CartesianCandidate:
    approach: PlannedSegment
    cartesian_segments: list[PlannedSegment]
    payload_pose: Pose
    length: float


def make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0
    return pose


def make_oriented_pose(x: float, y: float, z: float, orientation: Quaternion) -> Pose:
    pose = make_pose(x, y, z)
    pose.orientation = copy.deepcopy(orientation)
    return pose


def make_payload_pose(ee_orientation: Quaternion) -> Pose:
    norm = math.sqrt(
        ee_orientation.x**2
        + ee_orientation.y**2
        + ee_orientation.z**2
        + ee_orientation.w**2
    )
    if norm <= 1e-9:
        raise ValueError("End-effector orientation is not a valid quaternion")

    x = ee_orientation.x / norm
    y = ee_orientation.y / norm
    z = ee_orientation.z / norm
    w = ee_orientation.w / norm

    # R.T converts the desired world-frame vertical offset into EE_LINK.
    dx, dy, dz = 0.0, 0.0, BLOCK_GRIPPER_OFFSET_Z
    pose = Pose()
    pose.position.x = (1.0 - 2.0 * (y * y + z * z)) * dx + 2.0 * (
        x * y + z * w
    ) * dy + 2.0 * (x * z - y * w) * dz
    pose.position.y = 2.0 * (x * y - z * w) * dx + (
        1.0 - 2.0 * (x * x + z * z)
    ) * dy + 2.0 * (y * z + x * w) * dz
    pose.position.z = 2.0 * (x * z + y * w) * dx + 2.0 * (
        y * z - x * w
    ) * dy + (1.0 - 2.0 * (x * x + y * y)) * dz

    # The block starts axis-aligned in the world, so its EE-relative rotation
    # is the inverse of the end-effector rotation.
    pose.orientation = Quaternion(x=-x, y=-y, z=-z, w=w)
    return pose


def tool_z_direction(orientation: Quaternion) -> tuple[float, float, float]:
    norm = math.sqrt(
        orientation.x**2
        + orientation.y**2
        + orientation.z**2
        + orientation.w**2
    )
    if norm <= 1e-9:
        raise ValueError("End-effector orientation is not a valid quaternion")
    x = orientation.x / norm
    y = orientation.y / norm
    z = orientation.z / norm
    w = orientation.w / norm
    return (
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    )


def make_box_object(
    object_id: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    operation: int = CollisionObject.ADD,
    frame_id: str = FRAME,
) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = frame_id
    obj.operation = operation
    if operation == CollisionObject.ADD:
        obj.primitives.append(
            SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[float(size[0]), float(size[1]), float(size[2])],
            )
        )
        obj.primitive_poses.append(make_pose(*center))
    return obj


def make_sphere_position_constraint(
    x: float,
    y: float,
    z: float,
    tolerance: float,
) -> Constraints:
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(tolerance)])
    pc = PositionConstraint()
    pc.header.frame_id = FRAME
    pc.link_name = EE_LINK
    pc.constraint_region.primitives.append(sphere)
    pc.constraint_region.primitive_poses.append(make_pose(x, y, z))
    pc.weight = 1.0
    return Constraints(position_constraints=[pc])


def make_pose_constraints(
    x: float,
    y: float,
    z: float,
    position_tolerance: float,
    orientation: Optional[Quaternion],
    orientation_tolerance: float,
) -> Constraints:
    constraints = make_sphere_position_constraint(x, y, z, position_tolerance)
    if orientation is not None:
        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = EE_LINK
        oc.orientation = copy.deepcopy(orientation)
        oc.absolute_x_axis_tolerance = orientation_tolerance
        oc.absolute_y_axis_tolerance = orientation_tolerance
        oc.absolute_z_axis_tolerance = orientation_tolerance
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
    return constraints


def make_joint_constraints(target: Iterable[float], tolerance: float = 0.01) -> Constraints:
    constraints = Constraints()
    for joint_name, position in zip(ARM_JOINTS, target):
        jc = JointConstraint()
        jc.joint_name = joint_name
        jc.position = float(position)
        jc.tolerance_above = tolerance
        jc.tolerance_below = tolerance
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    return constraints


def wrap_near(reference: float, value: float) -> float:
    while value - reference > math.pi:
        value -= 2.0 * math.pi
    while value - reference < -math.pi:
        value += 2.0 * math.pi
    return value


def parse_xyz(text: Optional[str], default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not text:
        return default
    parts = [float(part.strip()) for part in text.split(",")]
    if len(parts) != 3:
        raise ValueError("Point must be formatted as x,y,z")
    return parts[0], parts[1], parts[2]


class PickAndPlacePlanner(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("cr5_pick_and_place")
        self.args = args
        self.pick_pose = parse_xyz(args.pick, PICK_POSE)
        self.place_pose = parse_xyz(args.place, PLACE_POSE)
        self._cb_group = ReentrantCallbackGroup()

        self._plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path", callback_group=self._cb_group
        )
        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self._cb_group
        )
        self._fk_client = self.create_client(
            GetPositionFK, "/compute_fk", callback_group=self._cb_group
        )
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self._cb_group
        )
        self._validity_client = self.create_client(
            GetStateValidity, "/check_state_validity", callback_group=self._cb_group
        )
        self._scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene", callback_group=self._cb_group
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            CONTROLLER_ACTION,
            callback_group=self._cb_group,
        )
        self._marker_pub = self.create_publisher(Marker, "/pick_place_block", 10)
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 20
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.positions: dict[str, float] = dict(zip(ARM_JOINTS, HOME_ARM))
        self.positions[GRIPPER_JOINT] = GRIPPER_OPEN
        self._seen_joint_names: set[str] = set()
        self._last_joint_state_time = 0.0
        self.fixed_orientation: Optional[Quaternion] = None

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in CONTROL_JOINTS:
                self.positions[name] = float(position)
                self._seen_joint_names.add(name)
        self._last_joint_state_time = time.monotonic()

    def wait_until_ready(self) -> None:
        self.get_logger().info("Waiting for MoveIt and trajectory controller...")
        for client, name in [
            (self._plan_client, "/plan_kinematic_path"),
            (self._cartesian_client, "/compute_cartesian_path"),
            (self._fk_client, "/compute_fk"),
            (self._ik_client, "/compute_ik"),
            (self._validity_client, "/check_state_validity"),
            (self._scene_client, "/apply_planning_scene"),
        ]:
            if not client.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f"{name} is not available")

        if not self._trajectory_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError(f"{CONTROLLER_ACTION} is not available")

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(joint in self._seen_joint_names for joint in CONTROL_JOINTS):
                break
        missing = [joint for joint in CONTROL_JOINTS if joint not in self._seen_joint_names]
        if missing:
            raise RuntimeError(f"Missing joint states: {missing}")
        self.get_logger().info("MoveIt and controller are ready")

    def update_fixed_orientation(self) -> Quaternion:
        deadline = time.monotonic() + 5.0
        last_error = ""
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self._tf_buffer.lookup_transform(
                    FRAME, EE_LINK, Time()
                )
                self.fixed_orientation = copy.deepcopy(transform.transform.rotation)
                q = self.fixed_orientation
                self.get_logger().info(
                    "Using fixed end-effector orientation "
                    f"xyzw=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})"
                )
                return self.fixed_orientation
            except TransformException as exc:
                last_error = str(exc)
        raise RuntimeError(f"Could not read {FRAME}->{EE_LINK} transform: {last_error}")

    def update_fixed_orientation_from_fk(self, positions: dict[str, float], label: str) -> Quaternion:
        request = GetPositionFK.Request()
        request.header.frame_id = FRAME
        request.fk_link_names = [EE_LINK]
        request.robot_state = self._robot_state_from_positions(positions)
        future = self._fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if (
            response is None
            or response.error_code.val != 1
            or not response.pose_stamped
        ):
            raise RuntimeError(f"Could not compute FK orientation for {label}")
        self.fixed_orientation = copy.deepcopy(response.pose_stamped[0].pose.orientation)
        q = self.fixed_orientation
        self.get_logger().info(
            f"Using FK orientation at {label} "
            f"xyzw=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})"
        )
        return self.fixed_orientation

    def solve_nearby_ik(
        self,
        xyz: tuple[float, float, float],
        orientation: Quaternion,
        seed_positions: dict[str, float],
    ) -> Optional[tuple[float, ...]]:
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = GROUP
        ik_request.robot_state = self._robot_state_from_positions(seed_positions)
        ik_request.avoid_collisions = True
        ik_request.ik_link_name = EE_LINK
        ik_request.pose_stamped.header.frame_id = FRAME
        ik_request.pose_stamped.pose = make_oriented_pose(*xyz, orientation)
        ik_request.timeout = self.seconds_to_duration(self.args.ik_timeout)

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.args.ik_timeout + 2.0
        )
        response = future.result()
        if response is None or response.error_code.val != 1:
            error_code = "no response" if response is None else response.error_code.val
            self.get_logger().warn(f"Nearby IK failed: error={error_code}")
            return None

        solution = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        missing = [joint for joint in ARM_JOINTS if joint not in solution]
        if missing:
            self.get_logger().warn(f"Nearby IK omitted joints: {missing}")
            return None

        target = tuple(
            wrap_near(seed_positions[joint], float(solution[joint]))
            for joint in ARM_JOINTS
        )
        distance = math.sqrt(
            sum(
                (target[index] - seed_positions[joint]) ** 2
                for index, joint in enumerate(ARM_JOINTS)
            )
        )
        self.get_logger().info(f"Nearby IK found, joint_distance={distance:.3f}")
        return target

    def spin_sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

    def apply_scene(self) -> None:
        remove_scene = PlanningScene()
        remove_scene.is_diff = True
        remove_scene.world.collision_objects = [
            make_box_object(
                "middle_fixture",
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                CollisionObject.REMOVE,
            )
        ]
        remove_future = self._scene_client.call_async(
            ApplyPlanningScene.Request(scene=remove_scene)
        )
        rclpy.spin_until_future_complete(self, remove_future, timeout_sec=2.0)

        add_objects = [
            # Keep the floor slightly below the robot base to avoid start-state contact.
            make_box_object("floor", (0.0, 0.0, -0.03), (3.0, 3.0, 0.02)),
            # Small stands under A and B. The gripper target stays above them.
            make_box_object("pick_stand", (self.pick_pose[0], self.pick_pose[1], 0.38), (0.18, 0.18, 0.04)),
            make_box_object("place_stand", (self.place_pose[0], self.place_pose[1], 0.38), (0.18, 0.18, 0.04)),
        ]
        if self.args.include_middle_fixture:
            add_objects.append(
                make_box_object("middle_fixture", (0.375, 0.025, 0.45), (0.13, 0.18, 0.10))
            )
        scene = PlanningScene()
        scene.is_diff = True
        # ADD with an existing id replaces that object in MoveIt. Avoid a prior REMOVE
        # request because MoveIt returns success=false when the object is not present.
        scene.world.collision_objects = add_objects
        request = ApplyPlanningScene.Request(scene=scene)
        future = self._scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if not response or not response.success:
            raise RuntimeError("Failed to apply planning scene")
        self.publish_world_block(*self.pick_pose)
        time.sleep(0.5)
        self.get_logger().info("Planning scene applied")

    def publish_world_block(self, x: float, y: float, gripper_z: float) -> None:
        marker = Marker()
        marker.header.frame_id = FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = make_pose(x, y, gripper_z + BLOCK_GRIPPER_OFFSET_Z)
        marker.scale = Vector3(x=BLOCK_SIZE, y=BLOCK_SIZE, z=BLOCK_SIZE)
        marker.color.r = 1.0
        marker.color.g = 0.52
        marker.color.b = 0.05
        marker.color.a = 1.0
        self._marker_pub.publish(marker)

    def publish_attached_block(self, payload_pose: Pose) -> None:
        marker = Marker()
        marker.header.frame_id = EE_LINK
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(payload_pose)
        marker.frame_locked = True
        marker.scale = Vector3(x=BLOCK_SIZE, y=BLOCK_SIZE, z=BLOCK_SIZE)
        marker.color.r = 1.0
        marker.color.g = 0.52
        marker.color.b = 0.05
        marker.color.a = 1.0
        self._marker_pub.publish(marker)

    @staticmethod
    def _attached_payload(payload_pose: Pose) -> AttachedCollisionObject:
        attached = AttachedCollisionObject()
        attached.link_name = EE_LINK
        attached.object = make_box_object(
            "carried_block",
            (0.0, 0.0, 0.0),
            (BLOCK_COLLISION_SIZE,) * 3,
            frame_id=EE_LINK,
        )
        attached.object.primitive_poses[0] = copy.deepcopy(payload_pose)
        attached.touch_links = list(PAYLOAD_TOUCH_LINKS)
        return attached

    def _robot_state_from_positions(
        self,
        positions: dict[str, float],
        payload_attached: bool = False,
        payload_pose: Optional[Pose] = None,
    ) -> RobotState:
        state = RobotState()
        state.joint_state.name = list(CONTROL_JOINTS)
        state.joint_state.position = [float(positions[j]) for j in CONTROL_JOINTS]
        if payload_attached:
            if payload_pose is None:
                raise ValueError("payload_pose is required when payload_attached is true")
            state.attached_collision_objects.append(self._attached_payload(payload_pose))
        return state

    def _make_motion_request(
        self,
        planner_id: str,
        start_positions: dict[str, float],
        goal_constraints: Constraints,
    ) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = "ompl"
        req.planner_id = planner_id
        req.num_planning_attempts = self.args.attempts
        req.allowed_planning_time = self.args.plan_time
        req.max_velocity_scaling_factor = self.args.velocity_scale
        req.max_acceleration_scaling_factor = self.args.acceleration_scale
        req.workspace_parameters.header.frame_id = FRAME
        req.workspace_parameters.min_corner = Vector3(x=-1.5, y=-1.5, z=-0.05)
        req.workspace_parameters.max_corner = Vector3(x=1.5, y=1.5, z=1.5)
        req.start_state = self._robot_state_from_positions(start_positions)
        req.goal_constraints.append(goal_constraints)
        return req

    def plan_segment(
        self,
        target: SegmentTarget,
        planner_id: str,
        start_positions: dict[str, float],
    ) -> Optional[PlannedSegment]:
        if target.kind == "pose":
            constraints = make_pose_constraints(
                target.target[0],
                target.target[1],
                target.target[2],
                self.args.goal_tolerance,
                self.fixed_orientation if self.args.constrain_orientation else None,
                self.args.orientation_tolerance,
            )
        elif target.kind == "joints":
            constraints = make_joint_constraints(target.target)
        else:
            raise ValueError(f"Unknown target kind: {target.kind}")

        request = GetMotionPlan.Request(
            motion_plan_request=self._make_motion_request(
                planner_id, start_positions, constraints
            )
        )
        future = self._plan_client.call_async(request)
        timeout = self.args.plan_time + 3.0
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result()
        if response is None:
            self.get_logger().warn(f"{target.name}: planner did not respond")
            return None

        mp = response.motion_plan_response
        if mp.error_code.val != 1:
            self.get_logger().warn(
                f"{target.name}: {planner_id} failed with error {mp.error_code.val}"
            )
            return None

        trajectory = mp.trajectory.joint_trajectory
        if not trajectory.points:
            self.get_logger().warn(f"{target.name}: empty trajectory")
            return None

        trajectory = self._wrap_arm_trajectory(trajectory, start_positions)
        if not self.validate_trajectory(trajectory, start_positions, target.name):
            return None

        length = self.trajectory_length(trajectory, start_positions)
        end_positions = self.end_positions(trajectory, start_positions)
        self.get_logger().info(
            f"{target.name}: {planner_id} ok, "
            f"{len(trajectory.points)} points, length={length:.3f}"
        )
        return PlannedSegment(
            name=target.name,
            trajectory=trajectory,
            start_positions=dict(start_positions),
            end_positions=end_positions,
            length=length,
        )

    def cartesian_segment(
        self,
        name: str,
        waypoint_xyz: list[tuple[float, float, float]],
        start_positions: dict[str, float],
        payload_attached: bool = False,
        payload_pose: Optional[Pose] = None,
    ) -> Optional[PlannedSegment]:
        if self.fixed_orientation is None:
            raise RuntimeError("Fixed orientation has not been initialized")

        request = GetCartesianPath.Request()
        request.header.frame_id = FRAME
        request.start_state = self._robot_state_from_positions(
            start_positions,
            payload_attached=payload_attached,
            payload_pose=payload_pose,
        )
        request.group_name = GROUP
        request.link_name = EE_LINK
        request.waypoints = [
            make_oriented_pose(x, y, z, self.fixed_orientation)
            for x, y, z in waypoint_xyz
        ]
        request.max_step = self.args.cartesian_step
        request.jump_threshold = self.args.jump_threshold
        request.avoid_collisions = True

        future = self._cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            self.get_logger().warn(f"{name}: Cartesian planner did not respond")
            return None
        if response.error_code.val != 1 or response.fraction < self.args.min_cartesian_fraction:
            self.get_logger().warn(
                f"{name}: Cartesian path failed, "
                f"fraction={response.fraction:.3f}, error={response.error_code.val}"
            )
            return None

        trajectory = response.solution.joint_trajectory
        if not trajectory.points:
            self.get_logger().warn(f"{name}: empty Cartesian trajectory")
            return None
        trajectory = self._wrap_arm_trajectory(trajectory, start_positions)
        if not self.validate_trajectory(
            trajectory,
            start_positions,
            name,
            payload_attached=payload_attached,
            payload_pose=payload_pose,
        ):
            return None

        length = self.trajectory_length(trajectory, start_positions)
        end_positions = self.end_positions(trajectory, start_positions)
        self.get_logger().info(
            f"{name}: Cartesian ok, fraction={response.fraction:.3f}, "
            f"{len(trajectory.points)} points, length={length:.3f}"
        )
        return PlannedSegment(
            name=name,
            trajectory=trajectory,
            start_positions=dict(start_positions),
            end_positions=end_positions,
            length=length,
        )

    def _wrap_arm_trajectory(
        self, trajectory: JointTrajectory, start_positions: dict[str, float]
    ) -> JointTrajectory:
        wrapped = copy.deepcopy(trajectory)
        previous = {joint: start_positions[joint] for joint in ARM_JOINTS}
        name_to_index = {name: i for i, name in enumerate(wrapped.joint_names)}
        for point in wrapped.points:
            positions = list(point.positions)
            for joint in ARM_JOINTS:
                if joint not in name_to_index:
                    continue
                idx = name_to_index[joint]
                positions[idx] = wrap_near(previous[joint], positions[idx])
                previous[joint] = positions[idx]
            point.positions = positions
        return wrapped

    def validate_trajectory(
        self,
        trajectory: JointTrajectory,
        start_positions: dict[str, float],
        segment_name: str,
        payload_attached: bool = False,
        payload_pose: Optional[Pose] = None,
        sample_count: Optional[int] = None,
    ) -> bool:
        sample_indexes = self._sample_indexes(
            len(trajectory.points),
            self.args.validation_samples if sample_count is None else sample_count,
        )
        name_to_index = {name: i for i, name in enumerate(trajectory.joint_names)}

        for idx in sample_indexes:
            positions = dict(start_positions)
            point = trajectory.points[idx]
            for joint in CONTROL_JOINTS:
                if joint in name_to_index:
                    positions[joint] = float(point.positions[name_to_index[joint]])

            request = GetStateValidity.Request()
            request.robot_state = self._robot_state_from_positions(
                positions,
                payload_attached=payload_attached,
                payload_pose=payload_pose,
            )
            request.group_name = GROUP
            future = self._validity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            response = future.result()
            if response is None or not response.valid:
                contact_text = ""
                if response and response.contacts:
                    first = response.contacts[0]
                    contact_text = f" ({first.contact_body_1} vs {first.contact_body_2})"
                self.get_logger().warn(
                    f"{segment_name}: invalid state at sample {idx}{contact_text}"
                )
                return False
        return True

    @staticmethod
    def _sample_indexes(point_count: int, sample_count: int) -> list[int]:
        if point_count <= 0:
            return []
        if sample_count <= 1:
            return [point_count - 1]
        if point_count <= sample_count:
            return list(range(point_count))
        indexes = {0, point_count - 1}
        for i in range(1, sample_count - 1):
            indexes.add(round(i * (point_count - 1) / (sample_count - 1)))
        return sorted(indexes)

    @staticmethod
    def trajectory_length(
        trajectory: JointTrajectory, start_positions: dict[str, float]
    ) -> float:
        name_to_index = {name: i for i, name in enumerate(trajectory.joint_names)}
        previous = [start_positions[j] for j in ARM_JOINTS]
        total = 0.0
        for point in trajectory.points:
            current = []
            for i, joint in enumerate(ARM_JOINTS):
                if joint in name_to_index:
                    current.append(float(point.positions[name_to_index[joint]]))
                else:
                    current.append(previous[i])
            total += math.sqrt(sum((a - b) ** 2 for a, b in zip(current, previous)))
            previous = current
        return total

    @staticmethod
    def end_positions(
        trajectory: JointTrajectory, start_positions: dict[str, float]
    ) -> dict[str, float]:
        positions = dict(start_positions)
        last = trajectory.points[-1]
        name_to_index = {name: i for i, name in enumerate(trajectory.joint_names)}
        for joint in ARM_JOINTS:
            if joint in name_to_index:
                positions[joint] = float(last.positions[name_to_index[joint]])
        return positions

    def build_targets(self, safe_z: float) -> list[SegmentTarget]:
        px, py, pz = self.pick_pose
        bx, by, bz = self.place_pose
        return [
            SegmentTarget("go_home", "joints", tuple(HOME_ARM)),
            SegmentTarget("approach_A", "pose", (px, py, safe_z)),
            SegmentTarget("grasp_A", "pose", (px, py, pz)),
            SegmentTarget("lift_A", "pose", (px, py, safe_z)),
            SegmentTarget("approach_B", "pose", (bx, by, safe_z)),
            SegmentTarget("place_B", "pose", (bx, by, bz)),
            SegmentTarget("retreat_B", "pose", (bx, by, safe_z)),
            SegmentTarget("return_home", "joints", tuple(HOME_ARM)),
        ]

    def plan_candidate(self, planner_id: str, safe_z: float) -> Optional[PlanCandidate]:
        self.get_logger().info(f"Planning candidate: planner={planner_id}, safe_z={safe_z:.2f}")
        simulated_positions = dict(self.positions)
        simulated_positions[GRIPPER_JOINT] = GRIPPER_OPEN
        segments: list[PlannedSegment] = []

        for target in self.build_targets(safe_z):
            segment = self.plan_segment(target, planner_id, simulated_positions)
            if segment is None:
                self.get_logger().warn(
                    f"Candidate failed: planner={planner_id}, safe_z={safe_z:.2f}, "
                    f"segment={target.name}"
                )
                return None
            segments.append(segment)
            simulated_positions = dict(segment.end_positions)
            if target.name == "grasp_A":
                simulated_positions[GRIPPER_JOINT] = GRIPPER_CLOSED
            elif target.name == "place_B":
                simulated_positions[GRIPPER_JOINT] = GRIPPER_OPEN

        total_length = sum(segment.length for segment in segments)
        self.get_logger().info(
            f"Candidate complete: planner={planner_id}, safe_z={safe_z:.2f}, "
            f"total_length={total_length:.3f}"
        )
        return PlanCandidate(planner_id, safe_z, segments, total_length)

    def choose_best_plan(self) -> PlanCandidate:
        safe_zs = [float(x) for x in self.args.safe_zs.split(",") if x.strip()]
        planners = [x.strip() for x in self.args.planners.split(",") if x.strip()]

        for planner_id in planners:
            candidates: list[PlanCandidate] = []
            for safe_z in safe_zs:
                candidate = self.plan_candidate(planner_id, safe_z)
                if candidate is not None:
                    candidates.append(candidate)
            if candidates:
                best = min(candidates, key=lambda c: c.length)
                self.get_logger().info(
                    f"Selected planner={best.planner_id}, safe_z={best.safe_z:.2f}, "
                    f"length={best.length:.3f}"
                )
                return best

        raise RuntimeError("No collision-free full pick-and-place plan found")

    def first_valid_segment(
        self,
        target: SegmentTarget,
        planner_ids: list[str],
        start_positions: dict[str, float],
    ) -> PlannedSegment:
        for planner_id in planner_ids:
            segment = self.plan_segment(target, planner_id, start_positions)
            if segment is not None:
                return segment
        raise RuntimeError(f"No valid plan for segment {target.name}")

    def finish_segment(
        self,
        segment: PlannedSegment,
        gripper_value: float,
        execute: bool,
        prepared_trajectory: Optional[JointTrajectory] = None,
    ) -> None:
        if execute:
            trajectory = prepared_trajectory or self.prepare_for_controller(
                segment, gripper_value
            )
            if not self.execute_trajectory(trajectory):
                raise RuntimeError(f"Execution failed at segment {segment.name}")
        self.positions.update(segment.end_positions)

    @staticmethod
    def merge_segments(name: str, segments: list[PlannedSegment]) -> PlannedSegment:
        if not segments:
            raise ValueError("No segments to merge")
        merged = JointTrajectory()
        merged.header = copy.deepcopy(segments[0].trajectory.header)
        merged.joint_names = list(segments[0].trajectory.joint_names)
        for segment in segments:
            for point in segment.trajectory.points:
                merged.points.append(copy.deepcopy(point))
        return PlannedSegment(
            name=name,
            trajectory=merged,
            start_positions=dict(segments[0].start_positions),
            end_positions=dict(segments[-1].end_positions),
            length=sum(segment.length for segment in segments),
        )

    def hold(self, label: str, seconds: float, execute: bool) -> None:
        self.get_logger().info(f"Holding {label} for {seconds:.1f}s")
        if execute:
            self.spin_sleep(seconds)

    def build_cartesian_candidate(self, approach: PlannedSegment) -> Optional[CartesianCandidate]:
        safe_z = self.args.hover_z
        px, py, pz = self.pick_pose
        bx, by, bz = self.place_pose
        saved_orientation = copy.deepcopy(self.fixed_orientation)
        saved_positions = dict(self.positions)

        direct_distance = math.sqrt(
            sum(
                (
                    approach.end_positions[joint]
                    - approach.start_positions[joint]
                )
                ** 2
                for joint in ARM_JOINTS
            )
        )
        detour_ratio = approach.length / max(direct_distance, 1e-6)
        per_joint_travel = self.per_joint_travel(
            approach.trajectory, approach.start_positions
        )
        worst_joint, worst_travel = max(
            per_joint_travel.items(), key=lambda item: item[1]
        )
        if worst_travel > self.args.max_approach_joint_travel:
            self.get_logger().warn(
                "Cartesian candidate rejected: approach_A rotates "
                f"{worst_joint} by {worst_travel:.3f}rad in total"
            )
            return None
        if detour_ratio > self.args.max_approach_detour_ratio:
            self.get_logger().warn(
                "Cartesian candidate rejected: approach_A detour ratio "
                f"{detour_ratio:.2f} exceeds "
                f"{self.args.max_approach_detour_ratio:.2f}"
            )
            return None

        sim_positions = dict(approach.end_positions)
        sim_positions[GRIPPER_JOINT] = GRIPPER_OPEN
        try:
            self.update_fixed_orientation_from_fk(sim_positions, "approach_A candidate")
            tool_z = tool_z_direction(self.fixed_orientation)
            downward_alignment = max(-1.0, min(1.0, -tool_z[2]))
            tool_tilt_deg = math.degrees(math.acos(downward_alignment))
            if tool_tilt_deg > self.args.max_tool_tilt_deg:
                raise RuntimeError(
                    f"tool tilt {tool_tilt_deg:.1f}deg exceeds "
                    f"{self.args.max_tool_tilt_deg:.1f}deg"
                )
            self.get_logger().info(
                f"approach_A candidate tool tilt from downward={tool_tilt_deg:.1f}deg"
            )
            payload_pose = make_payload_pose(self.fixed_orientation)
            cartesian_segments: list[PlannedSegment] = []
            for name, waypoints, gripper_value, payload_attached in [
                ("descend_A", [(px, py, pz)], GRIPPER_OPEN, False),
                ("lift_A", [(px, py, safe_z)], GRIPPER_CLOSED, True),
                ("transfer_A_to_B", [(bx, by, safe_z)], GRIPPER_CLOSED, True),
                ("descend_B", [(bx, by, bz)], GRIPPER_CLOSED, True),
                ("retreat_B", [(bx, by, safe_z)], GRIPPER_OPEN, False),
            ]:
                sim_positions[GRIPPER_JOINT] = gripper_value
                segment = self.cartesian_segment(
                    name,
                    waypoints,
                    sim_positions,
                    payload_attached=payload_attached,
                    payload_pose=payload_pose if payload_attached else None,
                )
                if segment is None:
                    raise RuntimeError(f"{name} is not fully Cartesian")
                cartesian_segments.append(segment)
                sim_positions = dict(segment.end_positions)
                sim_positions[GRIPPER_JOINT] = gripper_value

            total = approach.length + sum(segment.length for segment in cartesian_segments)
            self.get_logger().info(
                f"Cartesian candidate accepted: total_joint_length={total:.3f}"
            )
            return CartesianCandidate(approach, cartesian_segments, payload_pose, total)
        except Exception as exc:
            self.get_logger().warn(f"Cartesian candidate rejected: {exc}")
            return None
        finally:
            self.positions = dict(saved_positions)
            self.fixed_orientation = copy.deepcopy(saved_orientation)

    @staticmethod
    def per_joint_travel(
        trajectory: JointTrajectory,
        start_positions: dict[str, float],
    ) -> dict[str, float]:
        name_to_index = {name: index for index, name in enumerate(trajectory.joint_names)}
        previous = {joint: start_positions[joint] for joint in ARM_JOINTS}
        travel = {joint: 0.0 for joint in ARM_JOINTS}
        for point in trajectory.points:
            for joint in ARM_JOINTS:
                if joint not in name_to_index:
                    continue
                current = float(point.positions[name_to_index[joint]])
                travel[joint] += abs(current - previous[joint])
                previous[joint] = current
        return travel

    def build_direct_cartesian_chain(
        self,
        start_positions: dict[str, float],
        orientation: Quaternion,
    ) -> Optional[tuple[dict[str, PlannedSegment], Pose]]:
        tool_z = tool_z_direction(orientation)
        downward_alignment = max(-1.0, min(1.0, -tool_z[2]))
        tool_tilt_deg = math.degrees(math.acos(downward_alignment))
        if tool_tilt_deg > self.args.max_tool_tilt_deg:
            self.get_logger().warn(
                f"Direct Cartesian chain rejected: tool tilt {tool_tilt_deg:.1f}deg "
                f"exceeds {self.args.max_tool_tilt_deg:.1f}deg"
            )
            return None

        self.fixed_orientation = copy.deepcopy(orientation)
        payload_pose = make_payload_pose(orientation)
        px, py, pz = self.pick_pose
        bx, by, bz = self.place_pose
        safe_z = self.args.hover_z
        simulated = dict(start_positions)
        segments: dict[str, PlannedSegment] = {}

        specifications = [
            ("approach_A", [(px, py, safe_z)], GRIPPER_OPEN, False),
            ("descend_A", [(px, py, pz)], GRIPPER_OPEN, False),
            (
                "lift_A_and_transfer_A_to_B",
                [(px, py, safe_z), (bx, by, safe_z)],
                GRIPPER_CLOSED,
                True,
            ),
            ("descend_B", [(bx, by, bz)], GRIPPER_CLOSED, True),
        ]
        if self.args.retreat_after_place:
            specifications.append(
                ("retreat_B", [(bx, by, safe_z)], GRIPPER_OPEN, False)
            )

        for name, waypoints, gripper_value, payload_attached in specifications:
            simulated[GRIPPER_JOINT] = gripper_value
            segment = self.cartesian_segment(
                name,
                waypoints,
                simulated,
                payload_attached=payload_attached,
                payload_pose=payload_pose if payload_attached else None,
            )
            if segment is None:
                self.get_logger().warn(f"Direct Cartesian chain failed at {name}")
                return None
            segments[name] = segment
            simulated = dict(segment.end_positions)

        self.get_logger().info(
            f"Direct Cartesian chain accepted, tool tilt={tool_tilt_deg:.1f}deg, "
            f"joint_length={sum(segment.length for segment in segments.values()):.3f}"
        )
        return segments, payload_pose

    def run_cartesian_pick_place(self, execute: bool) -> None:
        entry_planners = [
            planner.strip()
            for planner in self.args.entry_planners.split(",")
            if planner.strip()
        ]
        actual_start = dict(self.positions)
        actual_start[GRIPPER_JOINT] = GRIPPER_OPEN
        initial_segment: Optional[PlannedSegment] = None

        if self.fixed_orientation is None:
            raise RuntimeError("Fixed orientation has not been initialized")

        direct_plan = self.build_direct_cartesian_chain(
            actual_start, self.fixed_orientation
        )
        if direct_plan is None:
            if self.args.skip_initial_home:
                raise RuntimeError(
                    "Current state cannot start the direct Cartesian chain and "
                    "--skip-initial-home was requested"
                )
            initial_segment = self.first_valid_segment(
                SegmentTarget("initial_home", "joints", tuple(HOME_ARM)),
                entry_planners,
                actual_start,
            )
            chain_start = dict(initial_segment.end_positions)
            chain_start[GRIPPER_JOINT] = GRIPPER_OPEN
            home_orientation = self.update_fixed_orientation_from_fk(
                chain_start, "initial_home"
            )
            direct_plan = self.build_direct_cartesian_chain(
                chain_start, home_orientation
            )
            if direct_plan is None:
                raise RuntimeError(
                    "Fixed HOME pose does not support the complete direct "
                    "Cartesian pick-and-place chain"
                )

        segments, payload_pose = direct_plan
        final_segment = segments.get("retreat_B", segments["descend_B"])

        return_home: Optional[PlannedSegment] = None
        if self.args.return_home:
            return_start = dict(final_segment.end_positions)
            return_start[GRIPPER_JOINT] = GRIPPER_OPEN
            return_home = self.first_valid_segment(
                SegmentTarget("return_home", "joints", tuple(HOME_ARM)),
                entry_planners,
                return_start,
            )

        validation_items = [
            (segments["approach_A"], GRIPPER_OPEN, False),
            (segments["descend_A"], GRIPPER_OPEN, False),
            (segments["lift_A_and_transfer_A_to_B"], GRIPPER_CLOSED, True),
            (segments["descend_B"], GRIPPER_CLOSED, True),
        ]
        if initial_segment is not None:
            validation_items.insert(0, (initial_segment, GRIPPER_OPEN, False))
        if "retreat_B" in segments:
            validation_items.append((segments["retreat_B"], GRIPPER_OPEN, False))
        if return_home is not None:
            validation_items.append((return_home, GRIPPER_OPEN, False))

        prepared: dict[str, JointTrajectory] = {}
        for segment, gripper_value, payload_attached in validation_items:
            trajectory = self.prepare_for_controller(segment, gripper_value)
            if not self.validate_trajectory(
                trajectory,
                segment.start_positions,
                f"{segment.name}_controller",
                payload_attached=payload_attached,
                payload_pose=payload_pose if payload_attached else None,
                sample_count=len(trajectory.points),
            ):
                raise RuntimeError(
                    f"Prepared controller trajectory is invalid: {segment.name}"
                )
            prepared[segment.name] = trajectory
            self.get_logger().info(
                f"{segment.name}: prepared controller trajectory validated, "
                f"{len(trajectory.points)} points"
            )

        if not execute:
            self.get_logger().info(
                "Plan-only direct Cartesian pick-and-place finished successfully"
            )
            return

        self.positions = dict(actual_start)
        self.get_logger().info("Opening gripper")
        if not self.execute_gripper(GRIPPER_OPEN):
            raise RuntimeError("Failed to open gripper")

        if initial_segment is not None:
            self.get_logger().info("Executing initial_home")
            self.finish_segment(
                initial_segment,
                GRIPPER_OPEN,
                execute,
                prepared[initial_segment.name],
            )
            self.hold("at initial point", self.args.initial_hold_seconds, execute)

        self.get_logger().info("Executing approach_A")
        self.finish_segment(
            segments["approach_A"],
            GRIPPER_OPEN,
            execute,
            prepared["approach_A"],
        )
        self.hold("above A", self.args.hold_seconds, execute)

        self.get_logger().info("Executing descend_A")
        self.finish_segment(
            segments["descend_A"],
            GRIPPER_OPEN,
            execute,
            prepared["descend_A"],
        )
        self.hold("at A", self.args.settle_seconds, execute)
        self.get_logger().info("Closing gripper at A")
        if not self.execute_gripper(GRIPPER_CLOSED):
            raise RuntimeError("Failed to close gripper at A")
        self.publish_attached_block(payload_pose)

        self.get_logger().info("Executing lift_A_and_transfer_A_to_B")
        self.finish_segment(
            segments["lift_A_and_transfer_A_to_B"],
            GRIPPER_CLOSED,
            execute,
            prepared["lift_A_and_transfer_A_to_B"],
        )
        self.hold("above B", self.args.hold_seconds, execute)

        self.get_logger().info("Executing descend_B")
        self.finish_segment(
            segments["descend_B"],
            GRIPPER_CLOSED,
            execute,
            prepared["descend_B"],
        )
        self.hold("at B", self.args.settle_seconds, execute)
        self.get_logger().info("Opening gripper at B")
        if not self.execute_gripper(GRIPPER_OPEN):
            raise RuntimeError("Failed to open gripper at B")
        self.publish_world_block(*self.place_pose)

        if "retreat_B" in segments:
            self.get_logger().info("Executing retreat_B")
            self.finish_segment(
                segments["retreat_B"],
                GRIPPER_OPEN,
                execute,
                prepared["retreat_B"],
            )

        if return_home is not None:
            self.get_logger().info("Executing return_home")
            self.finish_segment(
                return_home,
                GRIPPER_OPEN,
                execute,
                prepared[return_home.name],
            )

        self.get_logger().info(
            "Direct Cartesian pick-and-place complete; block released at B"
        )

    def run_sampled_cartesian_pick_place(self, execute: bool) -> None:
        entry_planners = [
            planner.strip() for planner in self.args.entry_planners.split(",") if planner.strip()
        ]
        safe_z = self.args.hover_z
        px, py, pz = self.pick_pose
        bx, by, bz = self.place_pose
        actual_start_positions = dict(self.positions)
        actual_start_positions[GRIPPER_JOINT] = GRIPPER_OPEN
        entry_start_positions = dict(actual_start_positions)

        initial_segment: Optional[PlannedSegment] = None
        if not self.args.skip_initial_home:
            initial_segment = self.first_valid_segment(
                SegmentTarget("initial_home", "joints", tuple(HOME_ARM)),
                entry_planners,
                actual_start_positions,
            )
            entry_start_positions = dict(initial_segment.end_positions)
            entry_start_positions[GRIPPER_JOINT] = GRIPPER_OPEN
            self.positions = dict(entry_start_positions)
            self.update_fixed_orientation_from_fk(entry_start_positions, "initial_home")
        else:
            self.positions = dict(entry_start_positions)

        approach_target = SegmentTarget("approach_A", "pose", (px, py, safe_z))
        approach_segments: list[PlannedSegment] = []
        candidates: list[CartesianCandidate] = []

        if self.fixed_orientation is None:
            raise RuntimeError("Fixed orientation has not been initialized")
        nearby_ik = self.solve_nearby_ik(
            (px, py, safe_z),
            self.fixed_orientation,
            entry_start_positions,
        )
        if nearby_ik is not None:
            for planner_id in entry_planners:
                segment = self.plan_segment(
                    SegmentTarget("approach_A", "joints", nearby_ik),
                    planner_id,
                    entry_start_positions,
                )
                if segment is None:
                    continue
                self.get_logger().info(
                    f"approach_A nearby-IK candidate: planner={planner_id}, "
                    f"length={segment.length:.3f}"
                )
                approach_segments.append(segment)
                candidate = self.build_cartesian_candidate(segment)
                if candidate is not None:
                    candidates.append(candidate)
                    break

        for planner_id in entry_planners:
            for sample_idx in range(self.args.entry_samples):
                segment = self.plan_segment(approach_target, planner_id, entry_start_positions)
                if segment is None:
                    continue
                self.get_logger().info(
                    f"approach_A candidate {len(approach_segments) + 1}: "
                    f"planner={planner_id}, sample={sample_idx + 1}, "
                    f"length={segment.length:.3f}"
                )
                approach_segments.append(segment)
                candidate = self.build_cartesian_candidate(segment)
                if candidate is not None:
                    candidates.append(candidate)
            if candidates and not self.args.optimize_approach:
                break

        if not approach_segments:
            raise RuntimeError("No valid approach_A candidates")

        if not candidates:
            raise RuntimeError("No approach_A candidate supports a full Cartesian A-to-B path")

        best = min(candidates, key=lambda candidate: candidate.length)
        self.get_logger().info(
            f"Selected Cartesian candidate: total_joint_length={best.length:.3f}"
        )

        segments = {segment.name: segment for segment in best.cartesian_segments}
        lift_transfer = self.merge_segments(
            "lift_A_and_transfer_A_to_B",
            [segments["lift_A"], segments["transfer_A_to_B"]],
        )

        return_home: Optional[PlannedSegment] = None
        if self.args.return_home:
            return_start = dict(segments["retreat_B"].end_positions)
            return_start[GRIPPER_JOINT] = GRIPPER_OPEN
            return_home = self.first_valid_segment(
                SegmentTarget("return_home", "joints", tuple(HOME_ARM)),
                entry_planners,
                return_start,
            )

        prepared: dict[str, JointTrajectory] = {}
        validation_items = [
            (best.approach, GRIPPER_OPEN, False),
            (segments["descend_A"], GRIPPER_OPEN, False),
            (lift_transfer, GRIPPER_CLOSED, True),
            (segments["descend_B"], GRIPPER_CLOSED, True),
            (segments["retreat_B"], GRIPPER_OPEN, False),
        ]
        if initial_segment is not None:
            validation_items.insert(0, (initial_segment, GRIPPER_OPEN, False))
        if return_home is not None:
            validation_items.append((return_home, GRIPPER_OPEN, False))

        for segment, gripper_value, payload_attached in validation_items:
            trajectory = self.prepare_for_controller(segment, gripper_value)
            if not self.validate_trajectory(
                trajectory,
                segment.start_positions,
                f"{segment.name}_controller",
                payload_attached=payload_attached,
                payload_pose=best.payload_pose if payload_attached else None,
                sample_count=len(trajectory.points),
            ):
                raise RuntimeError(
                    f"Prepared controller trajectory is invalid: {segment.name}"
                )
            prepared[segment.name] = trajectory
            self.get_logger().info(
                f"{segment.name}: prepared controller trajectory validated, "
                f"{len(trajectory.points)} points"
            )

        if not execute:
            self.get_logger().info("Plan-only mode finished successfully")
            return

        self.positions = dict(actual_start_positions)
        self.get_logger().info("Opening gripper")
        if not self.execute_gripper(GRIPPER_OPEN):
            raise RuntimeError("Failed to open gripper")

        if initial_segment is not None:
            self.get_logger().info("Executing initial_home")
            self.finish_segment(
                initial_segment,
                GRIPPER_OPEN,
                execute,
                prepared[initial_segment.name],
            )
            self.update_fixed_orientation_from_fk(dict(self.positions), "initial_home")
            self.hold("at initial point", self.args.initial_hold_seconds, execute)

        approach_a = best.approach
        self.get_logger().info("Executing approach_A")
        self.finish_segment(
            approach_a, GRIPPER_OPEN, execute, prepared[approach_a.name]
        )
        self.update_fixed_orientation_from_fk(dict(self.positions), "approach_A")
        self.hold("above A", self.args.hold_seconds, execute)

        self.get_logger().info("Executing descend_A")
        self.finish_segment(
            segments["descend_A"],
            GRIPPER_OPEN,
            execute,
            prepared["descend_A"],
        )

        self.hold("at A", self.args.settle_seconds, execute)
        self.get_logger().info("Closing gripper at A")
        if not self.execute_gripper(GRIPPER_CLOSED):
            raise RuntimeError("Failed to close gripper at A")
        self.publish_attached_block(best.payload_pose)
        self.positions[GRIPPER_JOINT] = GRIPPER_CLOSED

        self.get_logger().info("Executing lift_A_and_transfer_A_to_B")
        self.finish_segment(
            lift_transfer,
            GRIPPER_CLOSED,
            execute,
            prepared[lift_transfer.name],
        )

        self.hold("above B", self.args.hold_seconds, execute)

        self.get_logger().info("Executing descend_B")
        self.finish_segment(
            segments["descend_B"],
            GRIPPER_CLOSED,
            execute,
            prepared["descend_B"],
        )

        self.hold("at B", self.args.settle_seconds, execute)
        self.get_logger().info("Opening gripper at B")
        if not self.execute_gripper(GRIPPER_OPEN):
            raise RuntimeError("Failed to open gripper at B")
        self.publish_world_block(bx, by, bz)
        self.positions[GRIPPER_JOINT] = GRIPPER_OPEN

        self.get_logger().info("Executing retreat_B")
        self.finish_segment(
            segments["retreat_B"],
            GRIPPER_OPEN,
            execute,
            prepared["retreat_B"],
        )

        if return_home is not None:
            self.get_logger().info("Executing return_home")
            self.finish_segment(
                return_home,
                GRIPPER_OPEN,
                execute,
                prepared[return_home.name],
            )

        self.get_logger().info(
            f"Cartesian pick-and-place complete, total_joint_length={best.length:.3f}"
        )

    def prepare_for_controller(
        self,
        segment: PlannedSegment,
        gripper_value: float,
    ) -> JointTrajectory:
        source = segment.trajectory
        prepared = JointTrajectory()
        prepared.header = source.header
        prepared.joint_names = list(CONTROL_JOINTS)

        source_index = {name: i for i, name in enumerate(source.joint_names)}
        start_full = [segment.start_positions[j] for j in CONTROL_JOINTS]
        raw_positions = [start_full]

        for source_point in source.points:
            positions = []
            for joint in ARM_JOINTS:
                if joint in source_index:
                    positions.append(float(source_point.positions[source_index[joint]]))
                else:
                    positions.append(raw_positions[-1][CONTROL_JOINTS.index(joint)])
            positions.append(float(gripper_value))
            raw_positions.append(positions)

        if self.args.smooth_execution:
            command_positions = self.smooth_position_series(raw_positions)
        else:
            command_positions = raw_positions

        previous = command_positions[0]
        elapsed = 0.0

        for positions in command_positions[1:]:
            max_delta = max(abs(a - b) for a, b in zip(positions, previous))
            elapsed += max(self.args.min_point_time, max_delta / self.args.joint_speed)

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = []
            point.accelerations = []
            point.effort = []
            point.time_from_start = self.seconds_to_duration(elapsed)
            prepared.points.append(point)
            previous = positions

        self.fill_velocities(prepared)
        return prepared

    def smooth_position_series(self, positions: list[list[float]]) -> list[list[float]]:
        if len(positions) <= 2:
            return positions

        smoothed = [list(point) for point in positions]
        for _ in range(max(0, self.args.smooth_passes)):
            next_points = [list(smoothed[0])]
            for i in range(1, len(smoothed) - 1):
                point = list(smoothed[i])
                for j in range(len(ARM_JOINTS)):
                    point[j] = (
                        0.25 * smoothed[i - 1][j]
                        + 0.50 * smoothed[i][j]
                        + 0.25 * smoothed[i + 1][j]
                    )
                point[-1] = smoothed[i][-1]
                next_points.append(point)
            next_points.append(list(smoothed[-1]))
            smoothed = next_points

        return self.resample_by_joint_distance(smoothed, self.args.max_joint_step)

    @staticmethod
    def resample_by_joint_distance(
        positions: list[list[float]],
        max_joint_step: float,
    ) -> list[list[float]]:
        if len(positions) <= 2:
            return positions

        distances = [0.0]
        for prev, cur in zip(positions, positions[1:]):
            distances.append(
                distances[-1]
                + math.sqrt(
                    sum((cur[j] - prev[j]) ** 2 for j in range(len(ARM_JOINTS)))
                )
            )

        total = distances[-1]
        if total <= 1e-9:
            return [positions[0], positions[-1]]

        step_count = max(2, int(math.ceil(total / max(max_joint_step, 1e-3))) + 1)
        targets = [total * i / (step_count - 1) for i in range(step_count)]
        result: list[list[float]] = []
        seg_idx = 0
        for target in targets:
            while seg_idx < len(distances) - 2 and distances[seg_idx + 1] < target:
                seg_idx += 1
            left = distances[seg_idx]
            right = distances[seg_idx + 1]
            ratio = 0.0 if right <= left else (target - left) / (right - left)
            point = [
                positions[seg_idx][j]
                + ratio * (positions[seg_idx + 1][j] - positions[seg_idx][j])
                for j in range(len(positions[0]))
            ]
            result.append(point)
        result[0] = list(positions[0])
        result[-1] = list(positions[-1])
        return result

    @staticmethod
    def fill_velocities(trajectory: JointTrajectory) -> None:
        points = trajectory.points
        joint_count = len(trajectory.joint_names)
        if not points:
            return
        if len(points) == 1:
            points[0].velocities = [0.0] * joint_count
            points[0].accelerations = [0.0] * joint_count
            return

        times = [
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) / 1_000_000_000.0
            for point in points
        ]
        for i, point in enumerate(points):
            if i == 0 or i == len(points) - 1:
                point.velocities = [0.0] * joint_count
            else:
                dt = max(1e-6, times[i + 1] - times[i - 1])
                point.velocities = [
                    (points[i + 1].positions[j] - points[i - 1].positions[j]) / dt
                    for j in range(joint_count)
                ]
            point.accelerations = [0.0] * joint_count

    @staticmethod
    def seconds_to_duration(seconds: float) -> Duration:
        duration = Duration()
        duration.sec = int(seconds)
        duration.nanosec = int((seconds - duration.sec) * 1_000_000_000)
        return duration

    def execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        timeout = self._trajectory_duration(trajectory) + 10.0
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result()
        if result is None:
            self.get_logger().error("Trajectory execution timed out")
            return False
        if result.result.error_code != 0:
            self.get_logger().error(f"Trajectory failed: error_code={result.result.error_code}")
            return False

        if trajectory.points:
            for name, position in zip(trajectory.joint_names, trajectory.points[-1].positions):
                if name in CONTROL_JOINTS:
                    self.positions[name] = float(position)
        return True

    @staticmethod
    def _trajectory_duration(trajectory: JointTrajectory) -> float:
        if not trajectory.points:
            return 0.0
        duration = trajectory.points[-1].time_from_start
        return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0

    def execute_gripper(self, value: float) -> bool:
        traj = JointTrajectory()
        traj.joint_names = list(CONTROL_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [self.positions[j] for j in ARM_JOINTS] + [float(value)]
        point.time_from_start = self.seconds_to_duration(0.8)
        traj.points.append(point)
        ok = self.execute_trajectory(traj)
        if ok:
            self.positions[GRIPPER_JOINT] = float(value)
        return ok

    def execute_plan(self, candidate: PlanCandidate) -> bool:
        self.get_logger().info("Opening gripper")
        if not self.execute_gripper(GRIPPER_OPEN):
            return False

        gripper_value = GRIPPER_OPEN
        for segment in candidate.segments:
            self.get_logger().info(f"Executing {segment.name}")
            trajectory = self.prepare_for_controller(segment, gripper_value)
            if not self.execute_trajectory(trajectory):
                return False

            if segment.name == "grasp_A":
                self.get_logger().info("Closing gripper at A")
                if not self.execute_gripper(GRIPPER_CLOSED):
                    return False
                gripper_value = GRIPPER_CLOSED
                payload_pose = make_payload_pose(
                    self.fixed_orientation or Quaternion(w=1.0)
                )
                self.publish_attached_block(payload_pose)
            elif segment.name == "place_B":
                self.get_logger().info("Opening gripper at B")
                if not self.execute_gripper(GRIPPER_OPEN):
                    return False
                gripper_value = GRIPPER_OPEN
                self.publish_world_block(*self.place_pose)

            # Let joint_states catch up so the next controller goal starts cleanly.
            end_time = time.monotonic() + 0.2
            while time.monotonic() < end_time:
                rclpy.spin_once(self, timeout_sec=0.02)

        return True


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pick", default=None, help="A point as x,y,z in dummy_link")
    parser.add_argument("--place", default=None, help="B point as x,y,z in dummy_link")
    parser.add_argument("--planners", default=",".join(DEFAULT_PLANNERS))
    parser.add_argument("--entry-planners", default=",".join(DEFAULT_ENTRY_PLANNERS))
    parser.add_argument("--entry-samples", type=int, default=2)
    parser.add_argument("--optimize-approach", action="store_true")
    parser.add_argument("--ik-timeout", type=float, default=0.5)
    parser.add_argument("--max-approach-joint-travel", type=float, default=1.8)
    parser.add_argument("--max-approach-detour-ratio", type=float, default=2.0)
    parser.add_argument("--safe-zs", default=",".join(str(x) for x in DEFAULT_SAFE_ZS))
    parser.add_argument("--hover-z", type=float, default=0.62)
    parser.add_argument("--plan-time", type=float, default=5.0)
    parser.add_argument("--attempts", type=int, default=4)
    parser.add_argument("--goal-tolerance", type=float, default=0.018)
    parser.add_argument("--orientation-tolerance", type=float, default=0.25)
    parser.add_argument("--max-tool-tilt-deg", type=float, default=30.0)
    parser.add_argument("--no-orientation-constraint", dest="constrain_orientation", action="store_false")
    parser.add_argument("--validation-samples", type=int, default=12)
    parser.add_argument("--cartesian-step", type=float, default=0.01)
    parser.add_argument("--jump-threshold", type=float, default=2.0)
    parser.add_argument("--min-cartesian-fraction", type=float, default=0.995)
    parser.add_argument("--include-middle-fixture", action="store_true")
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument("--initial-hold-seconds", type=float, default=0.35)
    parser.add_argument("--settle-seconds", type=float, default=0.25)
    parser.add_argument("--velocity-scale", type=float, default=0.45)
    parser.add_argument("--acceleration-scale", type=float, default=0.45)
    parser.add_argument("--joint-speed", type=float, default=0.60)
    parser.add_argument("--min-point-time", type=float, default=0.05)
    parser.add_argument("--no-smooth-execution", dest="smooth_execution", action="store_false")
    parser.add_argument("--smooth-passes", type=int, default=1)
    parser.add_argument("--max-joint-step", type=float, default=0.08)
    parser.add_argument("--plan-only", action="store_true", help="plan and validate without execution")
    parser.add_argument("--rrt-full", action="store_true", help="use the old all-OMPL segmented mode")
    parser.add_argument(
        "--sampled-approach",
        action="store_true",
        help="use the legacy sampled-IK approach instead of the direct Cartesian chain",
    )
    parser.add_argument(
        "--retreat-after-place",
        action="store_true",
        help="retreat vertically after releasing the block at B",
    )
    parser.add_argument("--return-home", action="store_true", help="return to HOME after placing at B")
    parser.add_argument("--skip-initial-home", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = PickAndPlacePlanner(args)
    try:
        node.wait_until_ready()
        node.apply_scene()
        if args.rrt_full:
            if args.constrain_orientation:
                node.update_fixed_orientation()
            candidate = node.choose_best_plan()
            if args.plan_only:
                node.get_logger().info("Plan-only mode finished successfully")
                return 0
            if not node.execute_plan(candidate):
                return 2
        else:
            node.update_fixed_orientation()
            if args.sampled_approach:
                node.run_sampled_cartesian_pick_place(execute=not args.plan_only)
            else:
                node.run_cartesian_pick_place(execute=not args.plan_only)
        node.get_logger().info("Pick-and-place finished successfully")
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
