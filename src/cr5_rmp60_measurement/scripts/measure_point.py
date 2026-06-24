#!/usr/bin/env python3
"""Move above one XY point, probe vertically, and record the measured point."""
import argparse
import time
from types import SimpleNamespace

import rclpy

from probe_touch import ProbeTouch, add_vertical_probe_ack_argument, run_probe_cycle, validate_probe_args


def validate_args(args):
    """Validate args."""
    try:
        validate_probe_args(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    parser.add_argument("--x", type=float, required=True, help="target X at the safe height")
    parser.add_argument("--y", type=float, required=True, help="target Y at the safe height")
    parser.add_argument("--safe-z", type=float, required=True, help="safe approach Z")
    parser.add_argument("--approach-mm", type=float, default=20.0, help="vertical probing distance")
    parser.add_argument("--retract-mm", type=float, default=8.0, help="retract distance after trigger")
    parser.add_argument("--speed", type=int, default=1, help="SpeedFactor ratio during probing")
    parser.add_argument("--allow-long-approach", action="store_true", help="allow approach distance > 5 mm")
    parser.add_argument("--flange-mm", type=float, default=125.4, help="robot flange to cross-stylus branch origin")
    parser.add_argument("--stylus-mm", type=float, default=75.0, help="vertical stylus ball-centre distance")
    parser.add_argument("--timeout", type=float, default=20.0, help="probing timeout seconds")
    parser.add_argument(
        "--probe-step-mm",
        type=float,
        default=0.5,
        help="segment probing MovL into this step size; 0 disables segmented probing",
    )
    parser.add_argument("--output", default="data/measured_points.csv", help="CSV output path")
    add_vertical_probe_ack_argument(parser)
    args = parser.parse_args()
    validate_args(args)

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        current_pose = node.get_pose()
        safe_pose = [
            args.x,
            args.y,
            args.safe_z,
            current_pose[3],
            current_pose[4],
            current_pose[5],
        ]
        print(f"current pose: {current_pose}")
        print(f"safe pose:    {safe_pose}")
        print(f"probe target z: {args.safe_z - args.approach_mm:.4f}")

        node.wait_fresh_feed()
        if node.read_di1() or node.di1:
            raise RuntimeError("DI1 is already triggered before moving to the safe pose")

        if not args.execute:
            print("dry-run only; add --execute to move and probe")
            return

        node.set_speed(args.speed)
        node.move_l(safe_pose, timeout_sec=20.0)
        node.wait_until_pose(safe_pose, timeout_sec=20.0)
        time.sleep(0.2)
        node.wait_fresh_feed()
        if node.read_di1() or node.di1:
            raise RuntimeError("DI1 is triggered at the safe pose; refusing to probe")

        run_probe_cycle(node, SimpleNamespace(**vars(args)))

        node.move_l(safe_pose, timeout_sec=20.0)
        node.wait_until_pose(safe_pose, timeout_sec=20.0)
        print("returned to safe pose")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
