#!/usr/bin/env python3
"""Move a short distance along a supplied normal while monitoring DI1."""
import argparse
import math
import time

import rclpy
from dobot_msgs_v4.srv import MovL

from jog_and_record_contacts import current_pose, format_pose, fresh_feed_di1, pose_reached
from probe_touch import PROBE_SPIN_TIMEOUT_SEC, ProbeTouch


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--start-pose", nargs=6, type=float, required=True, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--direction", nargs=3, type=float, required=True, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--distance-mm", type=float, required=True)
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.05)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.08)
    parser.add_argument("--max-start-position-drift-mm", type=float, default=0.3)
    parser.add_argument("--max-start-orientation-drift-deg", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--ack-staged-preapproach", action="store_true")
    return parser.parse_args()


def normalize(vector):
    """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-12:
        raise ValueError("--direction cannot be zero")
    return [value / length for value in vector]


def angle_error_deg(actual, expected):
    """Angle error deg."""
    return max(abs((actual[i] - expected[i] + 180.0) % 360.0 - 180.0) for i in range(3, 6))


def main():
    """Main."""
    args = parse_args()
    if args.distance_mm <= 0:
        raise SystemExit("--distance-mm must be positive")
    if args.distance_mm > 3.0:
        raise SystemExit("--distance-mm must be <= 3.0 for staged pre-approach")
    if not 1 <= args.speed <= 5:
        raise SystemExit("--speed must be between 1 and 5")
    if args.execute and not args.ack_staged_preapproach:
        raise SystemExit("real motion requires --ack-staged-preapproach")

    direction = normalize(args.direction)
    target = list(args.start_pose)
    for index in range(3):
        target[index] += direction[index] * args.distance_mm

    print(f"planned start: {format_pose(args.start_pose)}")
    print(f"direction: [{direction[0]:.6f}, {direction[1]:.6f}, {direction[2]:.6f}]")
    print(f"distance: {args.distance_mm:.4f} mm")
    print(f"target: {format_pose(target)}")
    if not args.execute:
        print("dry-run only; add --execute --ack-staged-preapproach for real motion")
        return

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        node.wait_fresh_feed()
        live_start = current_pose(node, max_age_sec=0.5)
        if fresh_feed_di1(node, max_age_sec=0.5):
            raise RuntimeError("DI1 already triggered before staged pre-approach")

        drift = math.sqrt(sum((live_start[i] - args.start_pose[i]) ** 2 for i in range(3)))
        orientation_drift = angle_error_deg(live_start, args.start_pose)
        if drift > args.max_start_position_drift_mm or orientation_drift > args.max_start_orientation_drift_deg:
            raise RuntimeError(
                "current pose moved since planning; "
                f"live={format_pose(live_start)} planned={format_pose(args.start_pose)}"
            )

        node.set_speed(args.speed)
        req = MovL.Request()
        req.mode = False
        req.a, req.b, req.c, req.d, req.e, req.f = target
        future, immediate = node.call_async_checked(node.movl_cli, req)
        deadline = time.monotonic() + args.timeout_sec
        reached = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
            if immediate is None:
                immediate = node.check_ready_future(node.movl_cli, future)
            if node.last_feed_time is None or time.monotonic() - node.last_feed_time > 0.2:
                node.stop_fast_then_confirm()
                raise RuntimeError("FeedInfo became stale during staged pre-approach; motion stopped")
            snapshot = node.feed_snapshot()
            if snapshot.get("di1"):
                node.stop_fast_then_confirm()
                observed = snapshot.get("pose") or current_pose(node, max_age_sec=0.5)
                raise RuntimeError(f"DI1 triggered during staged pre-approach; stopped at {format_pose(observed)}")
            pose = snapshot.get("pose")
            if pose is not None and pose_reached(
                pose,
                target,
                args.position_tolerance_mm,
                args.orientation_tolerance_deg,
            ):
                reached = True
                break
        if not reached:
            node.stop_fast_then_confirm()
            raise RuntimeError(f"staged pre-approach timed out; stopped at {format_pose(current_pose(node, max_age_sec=0.5))}")

        final_pose = current_pose(node, max_age_sec=0.5)
        print("staged pre-approach reached")
        print(f"final: {format_pose(final_pose)}")
        print(f"DI1: {int(fresh_feed_di1(node, max_age_sec=0.5))}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
