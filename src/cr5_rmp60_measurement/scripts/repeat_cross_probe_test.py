#!/usr/bin/env python3
"""Repeat cross-stylus horizontal probing from one fixed safe start pose."""
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy

from cross_probe_touch import (
    DEFAULT_POSE_CONFIG,
    format_pose,
    orientation_error_deg,
    resolve_orientation,
    run_cross_probe_cycle,
    validate_args,
    wait_until_pose,
)
from geometry_utils import add, normalize, scale
from probe_touch import ProbeTouch
from workpiece_registration import DEFAULT_SETUP_CONFIG, load_approved_face, require_registered_start_pose


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_DIR / "data/cross_probe_contacts.csv"


def mean(values):
    """Mean."""
    return sum(values) / len(values)


def sample_std(values):
    """Sample std."""
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((value - avg) ** 2 for value in values) / (len(values) - 1))


def read_csv_rows(path):
    """Read CSV file and return rows as a list of dicts."""
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def validate_repeat_args(args):
    """Validate repeat args."""
    if args.cycles <= 0:
        raise SystemExit("--cycles must be positive")
    if args.settle_sec < 0:
        raise SystemExit("--settle-sec cannot be negative")
    validate_args(args)


def print_summary(rows, approach):
    """Print summary."""
    print("cross repeat summary")
    print(f"  triggered cycles: {len(rows)}")
    if not rows:
        return

    trigger_x = [float(row["flange_x"]) for row in rows]
    trigger_y = [float(row["flange_y"]) for row in rows]
    trigger_z = [float(row["flange_z"]) for row in rows]
    stop_x = [float(row["stop_flange_x"]) for row in rows]
    stop_y = [float(row["stop_flange_y"]) for row in rows]
    stop_z = [float(row["stop_flange_z"]) for row in rows]
    overtravel = [
        (stop_x[i] - trigger_x[i]) * approach[0]
        + (stop_y[i] - trigger_y[i]) * approach[1]
        + (stop_z[i] - trigger_z[i]) * approach[2]
        for i in range(len(rows))
    ]

    for name, values in (
        ("trigger_flange_x", trigger_x),
        ("trigger_flange_y", trigger_y),
        ("trigger_flange_z", trigger_z),
        ("stop_overtravel_along_approach", overtravel),
    ):
        print(f"  {name} mean: {mean(values):.4f} mm")
        print(f"  {name} sample std: {sample_std(values):.4f} mm")
        print(f"  {name} range: {min(values):.4f} .. {max(values):.4f} mm")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="actually run repeated real probing")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--standard-pose", default="x_neg_y_neg_verified")
    parser.add_argument("--pose-config", default=str(DEFAULT_POSE_CONFIG))
    parser.add_argument("--rx", type=float, help="explicit standard flange RX orientation")
    parser.add_argument("--ry", type=float, help="explicit standard flange RY orientation")
    parser.add_argument("--rz", type=float, help="explicit standard flange RZ orientation")
    parser.add_argument("--use-current-orientation", action="store_true", help="probe with the current robot orientation")
    parser.add_argument("--branch", default="y_neg", choices=("x_pos", "x_neg", "y_pos", "y_neg"))
    parser.add_argument("--approach", nargs=3, type=float, default=[-1.0, 0.0, 0.0], metavar=("DX", "DY", "DZ"))
    parser.add_argument("--distance-mm", type=float, default=5.0)
    parser.add_argument("--allow-over-1mm", action="store_true")
    parser.add_argument("--retract-mm", type=float, default=2.0)
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument(
        "--probe-step-mm",
        type=float,
        default=0.5,
        help="segment probing MovL into this step size; 0 disables segmented probing",
    )
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument(
        "--fixed-safe-pose",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="explicit fixed safe start pose for every cycle; defaults to the current pose",
    )
    parser.add_argument("--max-abs-z", type=float, default=0.05)
    parser.add_argument("--geometry", default=str(PROJECT_DIR / "config/cross_probe_geometry.yaml"))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--allow-orientation-setup", action="store_true")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--session-id", default="", help="measurement session identifier written to CSV")
    parser.add_argument("--setup-config", default=str(DEFAULT_SETUP_CONFIG), help="registered workpiece setup YAML")
    parser.add_argument("--setup-id", default="", help="approved workpiece setup identifier written to CSV")
    parser.add_argument("--workpiece-id", default="", help="workpiece identifier written to CSV")
    parser.add_argument("--face-id", default="", help="measured face/surface identifier written to CSV")
    parser.add_argument("--operator-note", default="", help="free-form note written to CSV")
    args = parser.parse_args()
    args.execution_purpose = "repeat_validation"
    validate_repeat_args(args)
    approach = normalize(args.approach)
    try:
        measurement_orientation, _ = resolve_orientation(args, approach)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if args.fixed_safe_pose is not None and measurement_orientation is not None:
        fixed_orientation_error = orientation_error_deg(args.fixed_safe_pose[3:6], measurement_orientation)
        if fixed_orientation_error > args.orientation_tolerance_deg:
            raise SystemExit(
                "--fixed-safe-pose orientation must match the requested measurement orientation; "
                f"max error is {fixed_orientation_error:.4f} deg"
            )

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        node.wait_fresh_feed()
        current_pose = node.get_pose()
        safe_pose = list(args.fixed_safe_pose) if args.fixed_safe_pose is not None else current_pose
        target_pose = safe_pose.copy()
        target_pose[:3] = add(safe_pose[:3], scale(approach, args.distance_mm))
        retract_pose = safe_pose.copy()
        retract_pose[:3] = add(safe_pose[:3], scale(approach, -args.retract_mm))

        print(f"current flange pose:    {format_pose(current_pose)}", flush=True)
        print(f"fixed safe start pose: {format_pose(safe_pose)}", flush=True)
        print(f"nominal per-cycle target from fixed safe pose: {format_pose(target_pose)}", flush=True)
        print(f"nominal per-cycle retract from fixed safe pose: {format_pose(retract_pose)}", flush=True)
        print(f"planned cycles: {args.cycles}", flush=True)

        if not args.execute:
            print("dry-run only; add --execute to run repeated cross probing")
            return

        try:
            registered_face = load_approved_face(args, approach)
            require_registered_start_pose(
                registered_face,
                safe_pose,
                orientation_tolerance_deg=args.orientation_tolerance_deg,
            )
            require_registered_start_pose(
                registered_face,
                current_pose,
                orientation_tolerance_deg=args.orientation_tolerance_deg,
            )
        except ValueError as exc:
            raise RuntimeError(
                "workpiece registration gate rejected repeated real probing before any positioning move: "
                f"{exc}"
            ) from exc

        node.set_speed(args.speed)
        new_rows = []
        for cycle in range(1, args.cycles + 1):
            print(f"[cycle {cycle}] moving to fixed safe start", flush=True)
            node.move_l(safe_pose, timeout_sec=10.0)
            wait_until_pose(
                node,
                safe_pose,
                orientation_tolerance_deg=args.orientation_tolerance_deg,
                timeout_sec=10.0,
                stop_on_di1=True,
            )
            node.wait_fresh_feed()
            if node.read_di1() or node.di1:
                raise RuntimeError("DI1 is triggered at the fixed safe start pose; stopping repeat test")

            print(f"[cycle {cycle}] probing", flush=True)
            row = run_cross_probe_cycle(node, args, prefix=f"[cycle {cycle}] ")
            if row is not None:
                new_rows.append(row)

            print(f"[cycle {cycle}] returning to fixed safe start", flush=True)
            node.move_l(safe_pose, timeout_sec=10.0)
            wait_until_pose(
                node,
                safe_pose,
                orientation_tolerance_deg=args.orientation_tolerance_deg,
                timeout_sec=10.0,
                stop_on_di1=True,
            )
            node.wait_fresh_feed()
            if cycle != args.cycles:
                time.sleep(args.settle_sec)

        print_summary(new_rows, approach)
        print(f"  saved: {args.output}")
    except (RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
