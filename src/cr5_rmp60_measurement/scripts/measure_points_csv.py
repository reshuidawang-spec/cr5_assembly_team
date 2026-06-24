#!/usr/bin/env python3
"""Measure multiple XY points from a CSV file."""
import argparse
import csv
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy

from probe_touch import (
    ProbeTouch,
    VERTICAL_PROBE_ACK_ARG,
    VERTICAL_PROBE_ACK_DEST,
    add_vertical_probe_ack_argument,
    run_probe_cycle,
    validate_probe_motion,
    validate_probe_speed,
)


REQUIRED_COLUMNS = {"name", "x", "y", "safe_z", "approach_mm"}


def validate_common_args(args):
    """Validate common args."""
    try:
        validate_probe_speed(args.speed)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if args.settle_sec < 0:
        raise SystemExit("--settle-sec cannot be negative")
    orientation_args = (args.rx, args.ry, args.rz)
    if any(value is not None for value in orientation_args) and not all(value is not None for value in orientation_args):
        raise SystemExit("--rx --ry --rz must be provided together")
    if getattr(args, "retract_mm", None) is not None and args.retract_mm <= 0:
        raise SystemExit("--retract-mm must be positive")
    if getattr(args, "timeout", None) is not None and args.timeout <= 0:
        raise SystemExit("--timeout must be positive")
    if getattr(args, "probe_step_mm", 0.0) < 0:
        raise SystemExit("--probe-step-mm cannot be negative")
    if args.execute and not getattr(args, VERTICAL_PROBE_ACK_DEST, False):
        raise SystemExit(
            "vertical probing real motion requires explicit on-site confirmation; "
            f"add {VERTICAL_PROBE_ACK_ARG} only after confirming the vertical stylus and this path are safe"
        )


def resolve_orientation(node, args):
    """Resolve orientation."""
    if args.rx is not None:
        print(f"using explicit orientation: rx={args.rx:.4f}, ry={args.ry:.4f}, rz={args.rz:.4f}")
        return args.rx, args.ry, args.rz

    if node is None:
        raise ValueError("resolve_orientation requires a node when --rx is not provided")
    current_pose = node.get_pose()
    rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
    if args.execute and not args.use_current_orientation:
        raise RuntimeError(
            "real execution requires explicit orientation: add --rx --ry --rz, "
            "or add --use-current-orientation after confirming the current robot pose"
        )
    print(f"using current orientation: rx={rx:.4f}, ry={ry:.4f}, rz={rz:.4f}")
    return rx, ry, rz


def read_points(path):
    """Read points."""
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        missing = REQUIRED_COLUMNS - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"missing required columns in {path}: {sorted(missing)}")
        points = list(reader)
    if not points:
        raise SystemExit(f"no points in {path}")
    return points


def point_float(point, key, default=None):
    """Point float."""
    value = point.get(key, "")
    if value == "" and default is not None:
        return default
    return float(value)


def parse_point(point, args):
    """Parse point."""
    name = point["name"]
    try:
        parsed = {
            "name": name,
            "x": point_float(point, "x"),
            "y": point_float(point, "y"),
            "safe_z": point_float(point, "safe_z"),
            "approach_mm": point_float(point, "approach_mm"),
            "retract_mm": point_float(point, "retract_mm", args.retract_mm),
        }
    except ValueError as exc:
        raise ValueError(f"{name}: point fields must be numeric") from exc
    if parsed["retract_mm"] <= 0:
        raise ValueError(f"{name}: retract_mm must be positive")
    validate_probe_motion(
        parsed["approach_mm"],
        args.allow_long_approach,
        args.speed,
        prefix=f"{name}: ",
    )
    return parsed


def validate_points(points, args):
    """Validate points."""
    for point in points:
        parse_point(point, args)


def measure_points(node, points, args):
    """Measure points."""
    failures = []
    rx, ry, rz = resolve_orientation(node, args)
    print(f"loaded points: {len(points)}")

    for index, point in enumerate(points, start=1):
        parsed = parse_point(point, args)
        name = parsed["name"]
        x = parsed["x"]
        y = parsed["y"]
        safe_z = parsed["safe_z"]
        approach_mm = parsed["approach_mm"]
        retract_mm = parsed["retract_mm"]

        safe_pose = [x, y, safe_z, rx, ry, rz]
        print(f"[{index}/{len(points)}] {name}: safe_pose={safe_pose}, target_z={safe_z - approach_mm:.4f}")

        if not args.execute:
            continue

        try:
            node.wait_fresh_feed()
            if node.read_di1() or node.di1:
                raise RuntimeError("DI1 is already triggered before moving to safe pose")
            node.set_speed(args.speed)
            node.move_l(safe_pose, timeout_sec=20.0)
            node.wait_until_pose(safe_pose, timeout_sec=20.0)
            time.sleep(0.2)
            node.wait_fresh_feed()
            if node.read_di1() or node.di1:
                raise RuntimeError("DI1 is triggered at safe pose")

            probe_args = SimpleNamespace(
                execute=True,
                approach_mm=approach_mm,
                retract_mm=retract_mm,
                speed=args.speed,
                allow_long_approach=args.allow_long_approach,
                flange_mm=args.flange_mm,
                stylus_mm=args.stylus_mm,
                timeout=args.timeout,
                probe_step_mm=args.probe_step_mm,
                output=args.output,
                ack_vertical_stylus_ok=args.ack_vertical_stylus_ok,
            )
            row = run_probe_cycle(node, probe_args, cycle_index=name)
            if row is None:
                raise RuntimeError("no trigger row recorded")
            node.move_l(safe_pose, timeout_sec=20.0)
            node.wait_until_pose(safe_pose, timeout_sec=20.0)
            print(f"[{name}] returned to safe pose")
        except Exception as exc:
            failures.append((name, str(exc)))
            print(f"[{name}] ERROR: {exc}")
            if not args.continue_on_error:
                raise
        if index != len(points):
            time.sleep(args.settle_sec)

    if not args.execute:
        print("dry-run only; add --execute to measure all points")
    print(f"completed points: {len(points) - len(failures)} / {len(points)}")
    if failures:
        print("failures:")
        for name, reason in failures:
            print(f"  {name}: {reason}")
    return failures


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    parser.add_argument("--input", required=True, help="CSV with name,x,y,safe_z,approach_mm")
    parser.add_argument("--output", default="data/measured_points.csv", help="CSV output path")
    parser.add_argument("--retract-mm", type=float, default=8.0, help="default retract distance")
    parser.add_argument("--speed", type=int, default=1, help="SpeedFactor ratio during probing")
    parser.add_argument("--allow-long-approach", action="store_true", help="allow approach distance > 5 mm")
    parser.add_argument("--rx", type=float, help="explicit safe-pose RX orientation")
    parser.add_argument("--ry", type=float, help="explicit safe-pose RY orientation")
    parser.add_argument("--rz", type=float, help="explicit safe-pose RZ orientation")
    parser.add_argument(
        "--use-current-orientation",
        action="store_true",
        help="allow real execution with the robot's current RX/RY/RZ",
    )
    parser.add_argument("--continue-on-error", action="store_true", help="continue after a point fails")
    parser.add_argument("--settle-sec", type=float, default=1.0, help="delay between points")
    parser.add_argument("--flange-mm", type=float, default=125.4, help="robot flange to cross-stylus branch origin")
    parser.add_argument("--stylus-mm", type=float, default=75.0, help="vertical stylus ball-centre distance")
    parser.add_argument("--timeout", type=float, default=20.0, help="probing timeout seconds per point")
    parser.add_argument(
        "--probe-step-mm",
        type=float,
        default=0.5,
        help="segment probing MovL into this step size; 0 disables segmented probing",
    )
    add_vertical_probe_ack_argument(parser)
    args = parser.parse_args()
    validate_common_args(args)

    points = read_points(Path(args.input))
    try:
        validate_points(points, args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    if not args.execute and args.rx is not None:
        measure_points(None, points, args)
        return

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        measure_points(node, points, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
