#!/usr/bin/env python3
"""Measure several points on one cube side with a horizontal cross-stylus touch."""
import argparse
from pathlib import Path
from types import SimpleNamespace

from cross_probe_touch import (
    DEFAULT_POSE_CONFIG,
    format_orientation,
    format_pose,
    resolve_orientation,
    validate_args,
)
from geometry_utils import add, normalize, scale


PROJECT_DIR = Path(__file__).resolve().parents[1]


def generate_safe_poses(args, orientation):
    """Generate safe poses."""
    if args.points <= 0:
        raise SystemExit("--points must be positive")
    if args.axis == "y":
        start = args.y1
        end = args.y2
        base_index = 1
    else:
        start = args.z1
        end = args.z2
        base_index = 2

    if args.points == 1:
        values = [start]
    else:
        values = [start + (end - start) * i / (args.points - 1) for i in range(args.points)]

    poses = []
    for idx, value in enumerate(values, start=1):
        pose = [args.safe_x, args.safe_y, args.safe_z] + orientation
        pose[base_index] = value
        poses.append((f"{args.name_prefix}{idx}", pose))
    return poses


def build_touch_args(args):
    """Build touch args."""
    standard_pose = args.standard_pose
    if isinstance(standard_pose, str) and standard_pose.lower() in ("", "none", "null"):
        standard_pose = None
    return SimpleNamespace(
        execute=args.execute,
        branch=args.branch,
        approach=args.approach,
        distance_mm=args.distance_mm,
        allow_over_1mm=args.allow_over_1mm,
        retract_mm=args.retract_mm,
        speed=args.speed,
        timeout=args.timeout,
        max_abs_z=args.max_abs_z,
        probe_step_mm=args.probe_step_mm,
        geometry=args.geometry,
        pose_config=args.pose_config,
        standard_pose=standard_pose,
        rx=args.rx,
        ry=args.ry,
        rz=args.rz,
        use_current_orientation=False,
        orientation_tolerance_deg=args.orientation_tolerance_deg,
        allow_orientation_setup=False,
        euler_sequence=args.euler_sequence,
        output=args.output,
        session_id=args.session_id,
        workpiece_id=args.workpiece_id,
        face_id=args.face_id,
        operator_note=args.operator_note,
    )


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    parser.add_argument("--safe-x", type=float, required=True, help="safe start X for the side line")
    parser.add_argument("--safe-y", type=float, required=True, help="safe start Y when --axis z")
    parser.add_argument("--safe-z", type=float, required=True, help="safe start Z when --axis y")
    parser.add_argument("--axis", choices=("y", "z"), default="y", help="vary safe poses along Y or Z")
    parser.add_argument("--y1", type=float, help="line start Y for --axis y")
    parser.add_argument("--y2", type=float, help="line end Y for --axis y")
    parser.add_argument("--z1", type=float, help="line start Z for --axis z")
    parser.add_argument("--z2", type=float, help="line end Z for --axis z")
    parser.add_argument("--points", type=int, required=True)
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
    parser.add_argument("--max-abs-z", type=float, default=0.05)
    parser.add_argument("--geometry", default=str(PROJECT_DIR / "config/cross_probe_geometry.yaml"))
    parser.add_argument("--pose-config", default=str(DEFAULT_POSE_CONFIG))
    parser.add_argument("--standard-pose", default="x_neg_y_neg_verified")
    parser.add_argument("--rx", type=float)
    parser.add_argument("--ry", type=float)
    parser.add_argument("--rz", type=float)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--continue-on-error", action="store_true")
    parser.add_argument("--output", default="data/cube_side_line.csv")
    parser.add_argument("--name-prefix", default="side_")
    parser.add_argument("--session-id", default="", help="measurement session identifier written to CSV")
    parser.add_argument("--workpiece-id", default="", help="workpiece identifier written to CSV")
    parser.add_argument("--face-id", default="", help="measured face/surface identifier written to CSV")
    parser.add_argument("--operator-note", default="", help="free-form note written to CSV")
    args = parser.parse_args()

    if args.axis == "y" and (args.y1 is None or args.y2 is None):
        raise SystemExit("--axis y requires --y1 and --y2")
    if args.axis == "z" and (args.z1 is None or args.z2 is None):
        raise SystemExit("--axis z requires --z1 and --z2")
    if args.settle_sec < 0:
        raise SystemExit("--settle-sec cannot be negative")

    approach = normalize(args.approach, "--approach")
    args.approach = approach
    touch_args = build_touch_args(args)
    validate_args(touch_args)
    try:
        orientation, orientation_source = resolve_orientation(touch_args, approach)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    safe_poses = generate_safe_poses(args, orientation)

    print(f"measurement orientation: {format_orientation(orientation)} ({orientation_source})")

    print("side-line safe poses:")
    for name, pose in safe_poses:
        target_pose = pose.copy()
        target_pose[:3] = add(pose[:3], scale(approach, args.distance_mm))
        retract_pose = pose.copy()
        retract_pose[:3] = add(pose[:3], scale(approach, -args.retract_mm))
        print(f"  {name} safe:    {format_pose(pose)}")
        print(f"  {name} target:  {format_pose(target_pose)}")
        print(f"  {name} retract: {format_pose(retract_pose)}")

    if not args.execute:
        print("dry-run only; add --execute after confirming all safe/target/retract poses")
        return

    # Real execution gated behind explicit approval; enable by setting False
    # after every path is generated from approved workpiece face registration records.
    _real_execution_disabled = True
    if _real_execution_disabled:
        raise SystemExit(
            "real side-line execution is disabled until every path is generated from approved "
            "workpiece face registration records"
        )

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        node.wait_fresh_feed()
        for index, (name, safe_pose) in enumerate(safe_poses, start=1):
            print(f"[{name}] moving to safe pose")
            node.move_l(safe_pose, timeout_sec=10.0)
            node.wait_fresh_feed()
            try:
                run_cross_probe_cycle(node, touch_args, prefix=f"[{name}] ")
            except Exception:
                if not args.continue_on_error:
                    raise
                print(f"[{name}] failed; continuing because --continue-on-error is set")
            if index != len(safe_poses):
                time.sleep(args.settle_sec)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
