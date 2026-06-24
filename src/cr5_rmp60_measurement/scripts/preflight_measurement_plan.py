#!/usr/bin/env python3
"""Preflight validation before turning a simulated probing path into execution."""
import argparse
import math

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from generate_measurement_poses import orientation_from_approach
from moveit_utils import (
    load_pose_specs,
    make_goal_constraints,
    make_pose_stamped,
    make_robot_state,
    publish_display,
)
from ros_wait_utils import wait_for_future


class MeasurementPreflight(Node):
    def __init__(self):
        super().__init__("rmp60_measurement_preflight")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cartesian_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.display_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    def wait_ready(self, timeout_sec):
        """Wait ready."""
        for client, name in (
            (self.ik_cli, "/compute_ik"),
            (self.plan_cli, "/plan_kinematic_path"),
            (self.cartesian_cli, "/compute_cartesian_path"),
        ):
            if not client.wait_for_service(timeout_sec=timeout_sec):
                raise RuntimeError(f"{name} service is not available")

        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                raise RuntimeError("/joint_states is not available")

    def check_tool_tf(self, args):
        """Check tool tf."""
        expected = args.adapter_length + args.probe_body_length + args.stylus_length
        deadline = self.get_clock().now().nanoseconds / 1e9 + args.tf_timeout
        last_error = None
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(args.tool_parent_frame, args.ik_link, rclpy.time.Time())
                t = transform.transform.translation
                length = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
                z_error = abs(t.z - expected)
                lateral = math.sqrt(t.x * t.x + t.y * t.y)
                ok = t.z > 0.0 and z_error <= args.tool_length_tolerance_m and lateral <= args.tool_lateral_tolerance_m
                return ok, (
                    f"translation=[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}], "
                    f"expected_z=+{expected:.4f}, length={length:.4f}"
                )
            except TransformException as exc:
                last_error = exc
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                return False, f"TF {args.tool_parent_frame}->{args.ik_link} unavailable: {last_error}"

    def solve_ik(self, label, position, orientation, seed_state, args):
        """Solve ik."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = args.group
        req.ik_request.ik_link_name = args.ik_link
        req.ik_request.avoid_collisions = args.avoid_collisions
        req.ik_request.timeout = Duration(sec=int(args.ik_timeout), nanosec=int((args.ik_timeout % 1.0) * 1e9))
        req.ik_request.robot_state = seed_state
        req.ik_request.pose_stamped = make_pose_stamped(
            position,
            orientation,
            args.frame_id,
            args.scale_factor,
            self.get_clock().now().to_msg(),
        )

        future = self.ik_cli.call_async(req)
        result = wait_for_future(self, future, args.ik_timeout + 2.0, "/compute_ik")
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        return ok, result.error_code.val, result.solution, label

    def plan_to_safe(self, start_state, safe_position, orientation, args):
        """Plan to safe."""
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
        request.goal_constraints.append(make_goal_constraints("safe", safe_position, orientation, args))
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

    def compute_probe_path(self, safe_state, contact_position, target_position, orientation, args):
        """Compute probe path."""
        req = GetCartesianPath.Request()
        req.header.frame_id = args.frame_id
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state = safe_state
        req.group_name = args.group
        req.link_name = args.ik_link
        req.max_step = args.max_step_m
        req.jump_threshold = args.jump_threshold
        req.avoid_collisions = args.avoid_collisions
        req.waypoints = [
            make_pose_stamped(contact_position, orientation, args.frame_id, args.scale_factor).pose,
            make_pose_stamped(target_position, orientation, args.frame_id, args.scale_factor).pose,
        ]

        future = self.cartesian_cli.call_async(req)
        result = wait_for_future(self, future, max(5.0, args.ik_timeout + 2.0), "/compute_cartesian_path")
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS and result.fraction >= args.min_fraction
        return ok, result

    def publish_display(self, start_state, trajectories):
        """Publish display."""
        publish_display(self.display_pub, self, start_state, trajectories)


def status(ok):
    """Status."""
    return "PASS" if ok else "FAIL"


def print_check(name, ok, detail):
    """Print check."""
    print(f"  [{status(ok)}] {name}: {detail}")


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
    parser.add_argument("--ik-timeout", type=float, default=0.2)
    parser.add_argument("--max-step-m", type=float, default=0.001)
    parser.add_argument("--jump-threshold", type=float, default=0.0)
    parser.add_argument("--min-fraction", type=float, default=1.0)
    parser.add_argument("--no-avoid-collisions", dest="avoid_collisions", action="store_false")
    parser.add_argument("--skip-tool-tf-check", action="store_true")
    parser.add_argument("--tool-parent-frame", default="Link6")
    parser.add_argument("--adapter-length", type=float, default=0.0494)
    parser.add_argument("--probe-body-length", type=float, default=0.076)
    parser.add_argument("--stylus-length", type=float, default=0.075)
    parser.add_argument("--tool-length-tolerance-m", type=float, default=0.003)
    parser.add_argument("--tool-lateral-tolerance-m", type=float, default=0.003)
    parser.add_argument("--tf-timeout", type=float, default=3.0)
    parser.add_argument("--no-display", action="store_true")
    parser.set_defaults(avoid_collisions=True)
    args = parser.parse_args()

    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")
    if args.attempts <= 0:
        raise SystemExit("--attempts must be positive")
    if args.planning_time <= 0:
        raise SystemExit("--planning-time must be positive")
    if args.ik_timeout <= 0:
        raise SystemExit("--ik-timeout must be positive")
    if args.max_step_m <= 0:
        raise SystemExit("--max-step-m must be positive")
    if not 0.0 < args.min_fraction <= 1.0:
        raise SystemExit("--min-fraction must be in (0, 1]")

    args.scale_factor = 0.001 if args.unit == "mm" else 1.0
    try:
        pose_specs = load_pose_specs(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    rclpy.init()
    node = MeasurementPreflight()
    all_ok = True
    try:
        node.wait_ready(timeout_sec=5.0)
        if not args.skip_tool_tf_check:
            tf_ok, tf_detail = node.check_tool_tf(args)
            print_check("tool TF direction", tf_ok, tf_detail)
            all_ok = all_ok and tf_ok

        for spec in pose_specs:
            print(f"{spec.get('name', 'probe_pose')}:")
            try:
                orientation = spec.get("tip_orientation") or orientation_from_approach(
                    spec["approach_vector"], reference_up=args.reference_up
                )
            except ValueError as exc:
                raise SystemExit(str(exc)) from exc
            print(
                "  tip_orientation_xyzw: "
                f"[{orientation['x']:.6f}, {orientation['y']:.6f}, {orientation['z']:.6f}, {orientation['w']:.6f}]"
            )

            start_state = make_robot_state(node.joint_state)
            safe_state = None
            for label, key in (("safe IK", "safe_position"), ("contact IK", "contact"), ("target IK", "target_position")):
                ik_ok, code, solution, _ = node.solve_ik(label, spec[key], orientation, start_state, args)
                print_check(label, ik_ok, f"moveit_error_code={code}, position={spec[key]}")
                all_ok = all_ok and ik_ok
                if key == "safe_position":
                    safe_state = solution

            safe_plan_ok, safe_plan = node.plan_to_safe(start_state, spec["safe_position"], orientation, args)
            safe_plan_points = len(safe_plan.trajectory.joint_trajectory.points)
            print_check(
                "current -> safe plan",
                safe_plan_ok,
                f"moveit_error_code={safe_plan.error_code.val}, points={safe_plan_points}, "
                f"planning_time={safe_plan.planning_time:.3f}s",
            )
            all_ok = all_ok and safe_plan_ok

            cart_ok = False
            cart_result = None
            if safe_state is not None:
                cart_ok, cart_result = node.compute_probe_path(
                    safe_state,
                    spec["contact"],
                    spec["target_position"],
                    orientation,
                    args,
                )
                cart_points = len(cart_result.solution.joint_trajectory.points)
                print_check(
                    "safe -> contact -> target Cartesian",
                    cart_ok,
                    f"moveit_error_code={cart_result.error_code.val}, fraction={cart_result.fraction:.3f}, "
                    f"points={cart_points}",
                )
                all_ok = all_ok and cart_ok

            if not args.no_display and safe_plan_ok and cart_ok and cart_result is not None:
                node.publish_display(start_state, [safe_plan.trajectory, cart_result.solution])
                print("  published: /display_planned_path")

        print(f"preflight result: {status(all_ok)}")
        if not all_ok:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
