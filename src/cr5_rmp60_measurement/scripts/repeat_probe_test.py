#!/usr/bin/env python3
"""Run repeated vertical RMP60 probing cycles and summarize repeatability."""
import argparse
import math
import time

import rclpy

from probe_touch import ProbeTouch, add_vertical_probe_ack_argument, run_probe_cycle, validate_probe_args


def mean(values):
    """Mean."""
    return sum(values) / len(values)


def sample_std(values):
    """Sample std."""
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((v - avg) ** 2 for v in values) / (len(values) - 1))


def validate_args(args):
    """Validate args."""
    if args.cycles <= 0:
        raise SystemExit("--cycles must be positive")
    if args.settle_sec < 0:
        raise SystemExit("--settle-sec cannot be negative")
    try:
        validate_probe_args(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    parser.add_argument("--cycles", type=int, default=5, help="number of probing cycles")
    parser.add_argument("--approach-mm", type=float, default=20.0, help="vertical probing distance")
    parser.add_argument("--retract-mm", type=float, default=8.0, help="retract distance after each cycle")
    parser.add_argument("--speed", type=int, default=1, help="SpeedFactor ratio during probing")
    parser.add_argument("--allow-long-approach", action="store_true", help="allow approach distance > 5 mm")
    parser.add_argument("--settle-sec", type=float, default=1.0, help="delay between cycles")
    parser.add_argument("--flange-mm", type=float, default=125.4, help="robot flange to cross-stylus branch origin")
    parser.add_argument("--stylus-mm", type=float, default=75.0, help="vertical stylus ball-centre distance")
    parser.add_argument("--timeout", type=float, default=20.0, help="probing timeout seconds per cycle")
    parser.add_argument(
        "--probe-step-mm",
        type=float,
        default=0.5,
        help="segment probing MovL into this step size; 0 disables segmented probing",
    )
    parser.add_argument("--output", default="data/repeat_probe_contacts.csv", help="CSV output path")
    parser.add_argument(
        "--fixed-safe-pose",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="explicit safe start pose for every cycle; defaults to the current pose",
    )
    add_vertical_probe_ack_argument(parser)
    args = parser.parse_args()
    validate_args(args)

    rclpy.init()
    node = ProbeTouch()
    rows = []
    missed_cycles = []
    try:
        node.wait_services(10.0)
        current_pose = node.get_pose()
        safe_pose = list(args.fixed_safe_pose) if args.fixed_safe_pose is not None else current_pose
        target_pose = safe_pose.copy()
        target_pose[2] -= args.approach_mm
        retract_pose = target_pose.copy()
        retract_pose[2] += args.retract_mm
        print(f"current flange pose:    {current_pose}")
        print(f"fixed safe start pose: {safe_pose}")
        print(f"nominal per-cycle target from fixed safe pose: {target_pose}")
        print(f"nominal per-cycle retract from fixed safe pose: {retract_pose}")
        if not args.execute:
            print("dry-run only; add --execute to run repeated probing")
            print(f"planned cycles: {args.cycles}")
            return

        for cycle in range(1, args.cycles + 1):
            node.move_l(safe_pose, timeout_sec=10.0)
            node.wait_until_pose(safe_pose, timeout_sec=10.0)
            node.wait_fresh_feed()
            if node.read_di1() or node.di1:
                raise RuntimeError("DI1 is still triggered at the safe start pose; stopping repeat test")
            row = run_probe_cycle(node, args, cycle_index=cycle)
            if row is not None:
                rows.append(row)
            else:
                missed_cycles.append(cycle)
                print(f"[cycle {cycle}] WARNING: no trigger row recorded")
            node.move_l(safe_pose, timeout_sec=10.0)
            node.wait_until_pose(safe_pose, timeout_sec=10.0)
            if cycle != args.cycles:
                time.sleep(args.settle_sec)

        print("repeatability summary")
        print(f"  requested cycles: {args.cycles}")
        print(f"  triggered cycles: {len(rows)}")
        if missed_cycles:
            print(f"  missed cycles: {missed_cycles}")
        if rows:
            for name in ("flange_x", "flange_y", "flange_z", "tip_z_est"):
                values = [float(row[name]) for row in rows]
                print(f"  {name} mean: {mean(values):.4f} mm")
                print(f"  {name} sample std: {sample_std(values):.4f} mm")
                print(f"  {name} range: {min(values):.4f} .. {max(values):.4f} mm")
            print(f"  saved: {args.output}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
