#!/usr/bin/env python3
"""Plan and display the straight Cartesian probing segment."""
import argparse

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

from generate_measurement_poses import orientation_from_approach
from moveit_utils import load_pose_specs, make_pose_stamped, publish_display
from ros_wait_utils import wait_for_future


class ProbeCartesianPlanner(Node):
    def __init__(self):
        super().__init__("rmp60_probe_cartesian_planner")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.cartesian_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.display_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    def wait_ready(self, timeout_sec):
        """Wait ready."""
        if not self.ik_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/compute_ik service is not available")
        if not self.cartesian_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/compute_cartesian_path service is not available")
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                raise RuntimeError("/joint_states is not available")

    def solve_safe_state(self, safe_position, orientation, args):
        """Solve safe state."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = args.group
        req.ik_request.ik_link_name = args.ik_link
        req.ik_request.avoid_collisions = args.avoid_collisions
        req.ik_request.timeout = Duration(sec=int(args.ik_timeout), nanosec=int((args.ik_timeout % 1.0) * 1e9))

        seed = RobotState()
        seed.joint_state = self.joint_state
        req.ik_request.robot_state = seed
        req.ik_request.pose_stamped = make_pose_stamped(
            safe_position,
            orientation,
            args.frame_id,
            args.scale_factor,
            self.get_clock().now().to_msg(),
        )

        future = self.ik_cli.call_async(req)
        result = wait_for_future(self, future, args.ik_timeout + 2.0, "/compute_ik")
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        return ok, result.error_code.val, result.solution

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

    def publish_display(self, start_state, trajectory):
        """Publish display."""
        publish_display(self.display_pub, self, start_state, [trajectory])


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
    parser.add_argument("--ik-timeout", type=float, default=0.2)
    parser.add_argument("--max-step-m", type=float, default=0.001)
    parser.add_argument("--jump-threshold", type=float, default=0.0)
    parser.add_argument("--min-fraction", type=float, default=1.0)
    parser.add_argument("--no-avoid-collisions", dest="avoid_collisions", action="store_false")
    parser.add_argument("--no-display", action="store_true")
    parser.set_defaults(avoid_collisions=True)
    args = parser.parse_args()

    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")
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
    node = ProbeCartesianPlanner()
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
            ik_ok, ik_code, safe_state = node.solve_safe_state(spec["safe_position"], orientation, args)
            print(f"  safe IK: {'OK' if ik_ok else 'FAIL'} (moveit_error_code={ik_code})")
            if not ik_ok:
                raise SystemExit(1)

            path_ok, result = node.compute_probe_path(
                safe_state,
                spec["contact"],
                spec["target_position"],
                orientation,
                args,
            )
            point_count = len(result.solution.joint_trajectory.points)
            print(
                f"  cartesian probe: {'OK' if path_ok else 'FAIL'} "
                f"(moveit_error_code={result.error_code.val}, fraction={result.fraction:.3f}, points={point_count})"
            )
            if not path_ok:
                raise SystemExit(1)
            if not args.no_display:
                node.publish_display(safe_state, result.solution)
                print("  published: /display_planned_path")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
