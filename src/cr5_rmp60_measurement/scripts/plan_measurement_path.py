#!/usr/bin/env python3
"""Plan and display a MoveIt path through safe/contact/target probing poses."""
import argparse

import rclpy
from moveit_msgs.msg import (
    DisplayTrajectory,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node
from sensor_msgs.msg import JointState

from generate_measurement_poses import orientation_from_approach
from moveit_utils import (
    load_pose_specs,
    make_goal_constraints,
    make_robot_state,
    publish_display,
    trajectory_end_state,
)
from ros_wait_utils import wait_for_future


class MeasurementPathPlanner(Node):
    def __init__(self):
        super().__init__("rmp60_measurement_path_planner")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.display_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    def wait_ready(self, timeout_sec):
        """Wait ready."""
        if not self.plan_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/plan_kinematic_path service is not available")
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                raise RuntimeError("/joint_states is not available")

    def plan_segment(self, label, start_state, position, orientation, args):
        """Plan segment."""
        req = GetMotionPlan.Request()
        request = req.motion_plan_request
        request.group_name = args.group
        request.pipeline_id = args.pipeline
        request.planner_id = args.planner
        request.num_planning_attempts = args.attempts
        request.allowed_planning_time = args.planning_time
        request.max_velocity_scaling_factor = args.velocity_scale
        request.max_acceleration_scaling_factor = args.acceleration_scale
        request.start_state = start_state
        request.goal_constraints.append(make_goal_constraints(label, position, orientation, args))
        request.workspace_parameters.header.frame_id = args.frame_id
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = -0.5
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0

        future = self.plan_cli.call_async(req)
        result = wait_for_future(self, future, args.planning_time + 2.0, "/plan_kinematic_path")
        response = result.motion_plan_response
        ok = response.error_code.val == MoveItErrorCodes.SUCCESS
        return ok, response

    def publish_display(self, start_state, trajectories):
        """Publish display."""
        publish_display(self.display_pub, self, start_state, trajectories)


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="JSON file from generate_measurement_poses.py --json")
    parser.add_argument("--contact", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--approach", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--reference-up", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("UX", "UY", "UZ"))
    parser.add_argument("--standoff-mm", type=float, default=20.0)
    parser.add_argument("--travel-mm", type=float, default=5.0)
    parser.add_argument("--name", default="probe_pose")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--unit", choices=("mm", "m"), default="mm")
    parser.add_argument("--group", default="cr5_group")
    parser.add_argument("--ik-link", default="rmp60_tip")
    parser.add_argument("--pipeline", default="ompl")
    parser.add_argument("--planner", default="")
    parser.add_argument("--attempts", type=int, default=5)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scale", type=float, default=0.1)
    parser.add_argument("--acceleration-scale", type=float, default=0.1)
    parser.add_argument("--position-tolerance-m", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=0.05)
    parser.add_argument("--no-display", action="store_true")
    args = parser.parse_args()

    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")
    if args.attempts <= 0:
        raise SystemExit("--attempts must be positive")
    if args.planning_time <= 0:
        raise SystemExit("--planning-time must be positive")
    if args.position_tolerance_m <= 0:
        raise SystemExit("--position-tolerance-m must be positive")
    if args.orientation_tolerance_rad <= 0:
        raise SystemExit("--orientation-tolerance-rad must be positive")

    args.scale_factor = 0.001 if args.unit == "mm" else 1.0
    try:
        pose_specs = load_pose_specs(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    rclpy.init()
    node = MeasurementPathPlanner()
    try:
        node.wait_ready(timeout_sec=5.0)
        for spec in pose_specs:
            try:
                orientation = spec.get("tip_orientation") or orientation_from_approach(
                    spec["approach_vector"], reference_up=args.reference_up
                )
            except ValueError as exc:
                raise SystemExit(str(exc)) from exc
            print(f"{spec.get('name', 'probe_pose')}:")
            print(
                "  tip_orientation_xyzw: "
                f"[{orientation['x']:.6f}, {orientation['y']:.6f}, {orientation['z']:.6f}, {orientation['w']:.6f}]"
            )
            start_state = make_robot_state(node.joint_state)
            display_start_state = start_state
            trajectories = []
            for label, key in (("safe", "safe_position"), ("contact", "contact"), ("target", "target_position")):
                ok, response = node.plan_segment(label, start_state, spec[key], orientation, args)
                point_count = len(response.trajectory.joint_trajectory.points)
                status = "OK" if ok else "FAIL"
                print(
                    f"  plan {label}: {status} "
                    f"(moveit_error_code={response.error_code.val}, points={point_count}, "
                    f"planning_time={response.planning_time:.3f}s)"
                )
                if not ok:
                    raise SystemExit(1)
                trajectories.append(response.trajectory)
                start_state = trajectory_end_state(start_state, response.trajectory)
            if not args.no_display:
                node.publish_display(display_start_state, trajectories)
                print("  published: /display_planned_path")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
