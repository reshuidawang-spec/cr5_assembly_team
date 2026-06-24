#!/usr/bin/env python3
"""Shared MoveIt utilities for the CR5 + RMP60 measurement pipeline."""
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from generate_measurement_poses import build_pose_spec


def require_position(spec, key):
    """Require position."""
    value = spec.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{spec.get('name', 'pose')}: {key} must be a 3-value list")
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{spec.get('name', 'pose')}: {key} must contain numbers") from exc
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{spec.get('name', 'pose')}: {key} must contain finite numbers")
    return result


def validate_orientation(spec, orientation):
    """Validate orientation."""
    required = ("x", "y", "z", "w")
    if not isinstance(orientation, dict) or not all(key in orientation for key in required):
        raise ValueError(f"{spec.get('name', 'pose')}: tip_orientation must contain x/y/z/w")
    result = {key: float(orientation[key]) for key in required}
    if not all(math.isfinite(value) for value in result.values()):
        raise ValueError(f"{spec.get('name', 'pose')}: tip_orientation must contain finite numbers")
    norm = math.sqrt(sum(value * value for value in result.values()))
    if norm <= 1e-9:
        raise ValueError(f"{spec.get('name', 'pose')}: tip_orientation cannot be zero")
    return {key: value / norm for key, value in result.items()}


def validate_pose_spec(spec):
    """Validate pose spec."""
    if not isinstance(spec, dict):
        raise ValueError("pose spec must be a JSON object")
    spec["contact"] = require_position(spec, "contact")
    spec["safe_position"] = require_position(spec, "safe_position")
    spec["target_position"] = require_position(spec, "target_position")
    if "approach_vector" in spec:
        spec["approach_vector"] = require_position(spec, "approach_vector")
    if "tip_orientation" in spec:
        spec["tip_orientation"] = validate_orientation(spec, spec["tip_orientation"])
    if "approach_vector" not in spec and "tip_orientation" not in spec:
        raise ValueError(f"{spec.get('name', 'pose')}: provide approach_vector or tip_orientation")
    if "standoff_mm" in spec:
        standoff_mm = float(spec["standoff_mm"])
        if not math.isfinite(standoff_mm) or standoff_mm <= 0:
            raise ValueError(f"{spec.get('name', 'pose')}: standoff_mm must be positive")
    if "travel_mm" in spec:
        travel_mm = float(spec["travel_mm"])
        if not math.isfinite(travel_mm) or travel_mm < 0:
            raise ValueError(f"{spec.get('name', 'pose')}: travel_mm cannot be negative")
    return spec


def load_pose_specs(args):
    """Load pose specs."""
    if args.input:
        data = json.loads(Path(args.input).read_text())
        specs = data if isinstance(data, list) else [data]
        return [validate_pose_spec(spec) for spec in specs]

    if args.contact is None or args.approach is None:
        raise SystemExit("provide --input, or provide both --contact and --approach")
    return [
        validate_pose_spec(
            build_pose_spec(
                args.name,
                list(args.contact),
                list(args.approach),
                args.standoff_mm,
                args.travel_mm,
                reference_up=list(args.reference_up),
            )
        )
    ]


def make_robot_state(joint_state):
    """Make robot state."""
    state = RobotState()
    state.joint_state = joint_state
    return state


def make_pose_stamped(position, orientation, frame_id, scale_factor, stamp=None):
    """Make pose stamped."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    pose.pose.position.x = position[0] * scale_factor
    pose.pose.position.y = position[1] * scale_factor
    pose.pose.position.z = position[2] * scale_factor
    pose.pose.orientation.x = orientation["x"]
    pose.pose.orientation.y = orientation["y"]
    pose.pose.orientation.z = orientation["z"]
    pose.pose.orientation.w = orientation["w"]
    return pose


def make_goal_constraints(label, position, orientation, args):
    """Make goal constraints."""
    pose = Pose()
    pose.position.x = position[0] * args.scale_factor
    pose.position.y = position[1] * args.scale_factor
    pose.position.z = position[2] * args.scale_factor
    pose.orientation.x = orientation["x"]
    pose.orientation.y = orientation["y"]
    pose.orientation.z = orientation["z"]
    pose.orientation.w = orientation["w"]

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [args.position_tolerance_m]

    volume = BoundingVolume()
    volume.primitives.append(sphere)
    volume.primitive_poses.append(pose)

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = args.frame_id
    position_constraint.link_name = args.ik_link
    position_constraint.constraint_region = volume
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = args.frame_id
    orientation_constraint.link_name = args.ik_link
    orientation_constraint.orientation = pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = args.orientation_tolerance_rad
    orientation_constraint.absolute_y_axis_tolerance = args.orientation_tolerance_rad
    orientation_constraint.absolute_z_axis_tolerance = args.orientation_tolerance_rad
    orientation_constraint.weight = 1.0

    constraints = Constraints()
    constraints.name = label
    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)
    return constraints


def trajectory_end_state(start_state, trajectory):
    """Trajectory end state."""
    points = trajectory.joint_trajectory.points
    if not points:
        return start_state
    joint_state = JointState()
    joint_state.header = start_state.joint_state.header
    joint_state.name = list(trajectory.joint_trajectory.joint_names)
    joint_state.position = list(points[-1].positions)
    return make_robot_state(joint_state)


def publish_display(publisher, node, start_state, trajectories):
    """Publish display."""
    display = DisplayTrajectory()
    display.model_id = "cr5_robot"
    display.trajectory_start = start_state
    display.trajectory = list(trajectories)
    for _ in range(3):
        publisher.publish(display)
        rclpy.spin_once(node, timeout_sec=0.1)
