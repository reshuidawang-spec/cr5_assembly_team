#!/usr/bin/env python3
"""Check MoveIt IK reachability for generated measurement positions."""
import argparse

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

from generate_measurement_poses import orientation_from_approach
from moveit_utils import load_pose_specs, make_pose_stamped
from ros_wait_utils import wait_for_future


class ReachabilityChecker(Node):
    def __init__(self):
        super().__init__("rmp60_measurement_reachability_checker")
        self.joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    def wait_ready(self, timeout_sec):
        """Wait ready."""
        if not self.ik_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/compute_ik service is not available")
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                raise RuntimeError("/joint_states is not available")

    def check_pose(self, position, orientation, args):
        """Check pose."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = args.group
        req.ik_request.ik_link_name = args.ik_link
        req.ik_request.avoid_collisions = args.avoid_collisions
        req.ik_request.timeout = Duration(sec=int(args.ik_timeout), nanosec=int((args.ik_timeout % 1.0) * 1e9))

        state = RobotState()
        state.joint_state = self.joint_state
        req.ik_request.robot_state = state

        req.ik_request.pose_stamped = make_pose_stamped(
            position,
            orientation,
            args.frame_id,
            args.scale_factor,
            self.get_clock().now().to_msg(),
        )

        future = self.ik_cli.call_async(req)
        result = wait_for_future(self, future, args.ik_timeout + 2.0, "/compute_ik")
        return result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code.val


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
    parser.add_argument("--manual-orientation", action="store_true", help="use --qx/--qy/--qz/--qw instead of approach-derived orientation")
    parser.add_argument("--avoid-collisions", action="store_true")
    parser.add_argument("--ik-timeout", type=float, default=0.2)
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=1.0)
    args = parser.parse_args()

    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")
    if args.ik_timeout <= 0:
        raise SystemExit("--ik-timeout must be positive")

    args.scale_factor = 0.001 if args.unit == "mm" else 1.0
    try:
        pose_specs = load_pose_specs(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    rclpy.init()
    node = ReachabilityChecker()
    try:
        node.wait_ready(timeout_sec=5.0)
        all_ok = True
        for spec in pose_specs:
            print(f"{spec.get('name', 'probe_pose')}:")
            try:
                orientation = (
                    {"x": args.qx, "y": args.qy, "z": args.qz, "w": args.qw}
                    if args.manual_orientation
                    else spec.get("tip_orientation") or orientation_from_approach(spec["approach_vector"], args.reference_up)
                )
            except ValueError as exc:
                raise SystemExit(str(exc)) from exc
            print(
                "  tip_orientation_xyzw: "
                f"[{orientation['x']:.6f}, {orientation['y']:.6f}, {orientation['z']:.6f}, {orientation['w']:.6f}]"
            )
            for label, key in (("safe", "safe_position"), ("contact", "contact"), ("target", "target_position")):
                ok, code = node.check_pose(spec[key], orientation, args)
                status = "OK" if ok else "FAIL"
                print(f"  {label}: {status} (moveit_error_code={code}) position={spec[key]}")
                all_ok = all_ok and ok
        if not all_ok:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
