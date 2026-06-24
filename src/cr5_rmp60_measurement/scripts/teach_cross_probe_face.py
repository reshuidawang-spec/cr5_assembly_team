#!/usr/bin/env python3
"""Capture an operator-taught safe start pose for one cross-probe workpiece face."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import rclpy
import yaml

from cross_probe_touch import DEFAULT_POSE_CONFIG, format_pose, resolve_orientation, validate_args
from geometry_utils import normalize
from probe_touch import ProbeTouch
from workpiece_registration import DEFAULT_SETUP_CONFIG, load_setup_config, pose_errors


def captured_pose(args):
    """Captured pose."""
    if args.pose is not None:
        return list(args.pose)
    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        node.wait_fresh_feed()
        if node.read_di1() or node.di1:
            raise RuntimeError("DI1 is triggered; clear the probe before teaching a safe start pose")
        return node.get_pose()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_SETUP_CONFIG))
    parser.add_argument("--setup-id", required=True)
    parser.add_argument("--workpiece-id", required=True)
    parser.add_argument("--face-id", required=True)
    parser.add_argument("--pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--standard-pose", default="x_neg_y_neg_verified")
    parser.add_argument("--pose-config", default=str(DEFAULT_POSE_CONFIG))
    parser.add_argument("--branch", default="y_neg", choices=("x_pos", "x_neg", "y_pos", "y_neg"))
    parser.add_argument("--approach", nargs=3, type=float, default=[-1.0, 0.0, 0.0], metavar=("DX", "DY", "DZ"))
    parser.add_argument("--max-search-mm", type=float, required=True)
    parser.add_argument("--retract-mm", type=float, default=2.0)
    parser.add_argument("--max-speed", type=int, default=1)
    parser.add_argument("--stop-margin-mm", type=float, default=2.0)
    parser.add_argument("--operator-note", default="")
    parser.add_argument("--record", action="store_true", help="write the taught face into the config file")
    parser.add_argument("--approve-for-execute", action="store_true")
    parser.add_argument("--ack-path-clearance", action="store_true", help="confirm positioning, probing and retract paths are clear")
    parser.add_argument(
        "--replace-existing-face",
        action="store_true",
        help="replace an existing face id while retaining its previous record in history",
    )
    args = parser.parse_args()

    if args.max_search_mm <= 0 or args.retract_mm <= 0 or args.stop_margin_mm <= 0:
        raise SystemExit("--max-search-mm, --retract-mm and --stop-margin-mm must be positive")
    if not 1 <= args.max_speed <= 5:
        raise SystemExit("--max-speed must be between 1 and 5")
    if args.approve_for_execute and not (args.record and args.ack_path_clearance):
        raise SystemExit("--approve-for-execute requires --record and --ack-path-clearance")
    if args.approve_for_execute and args.pose is not None:
        raise SystemExit("--approve-for-execute must capture the current robot pose; do not use --pose")

    approach = normalize(args.approach)
    touch_args = SimpleNamespace(
        execute=False,
        branch=args.branch,
        approach=approach,
        distance_mm=args.max_search_mm,
        allow_over_1mm=True,
        retract_mm=args.retract_mm,
        speed=args.max_speed,
        timeout=10.0,
        max_abs_z=0.05,
        pose_config=args.pose_config,
        standard_pose=args.standard_pose,
        rx=None,
        ry=None,
        rz=None,
        use_current_orientation=False,
    )
    validate_args(touch_args)
    orientation, _ = resolve_orientation(touch_args, approach)
    pose = captured_pose(args)
    _, orientation_error = pose_errors(pose, pose[:3] + orientation)
    if orientation_error > 0.5:
        raise SystemExit(
            "taught pose does not match standard measurement orientation; "
            f"max error is {orientation_error:.4f} deg"
        )

    status = "approved" if args.approve_for_execute else "taught"
    now = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
    face = {
        "status": status,
        "standard_pose": args.standard_pose,
        "branch": args.branch,
        "approach": [float(value) for value in approach],
        "safe_start_pose": [float(value) for value in pose],
        "max_search_mm": float(args.max_search_mm),
        "retract_mm": float(args.retract_mm),
        "max_speed": int(args.max_speed),
        "stop_margin_mm": float(args.stop_margin_mm),
        "operator_note": args.operator_note,
        "updated_at": now,
    }
    print(f"setup/face: {args.setup_id}/{args.face_id}")
    print(f"status: {status}")
    print(f"safe start pose: {format_pose(pose)}")
    print(f"approved X/Y/Z search limit: {args.max_search_mm:.4f} mm along approach {approach}")

    if not args.record:
        print("dry-run only; add --record to write this taught face")
        return

    config_path = Path(args.config)
    config = load_setup_config(config_path)
    setup = config["setups"].setdefault(
        args.setup_id,
        {"workpiece_id": args.workpiece_id, "faces": {}, "created_at": now},
    )
    if setup.get("workpiece_id") != args.workpiece_id:
        raise SystemExit(f"setup {args.setup_id!r} is already assigned to another workpiece")
    faces = setup.setdefault("faces", {})
    previous = faces.get(args.face_id)
    if previous is not None and not args.replace_existing_face:
        raise SystemExit(
            f"face {args.face_id!r} already exists; use a new --face-id or add --replace-existing-face "
            "to preserve the previous record in history"
        )
    if previous is not None:
        previous_history = list(previous.get("history", [])) if isinstance(previous, dict) else []
        previous_snapshot = dict(previous)
        previous_snapshot.pop("history", None)
        face["history"] = previous_history + [previous_snapshot]
    faces[args.face_id] = face
    with config_path.open("w") as f:
        yaml.safe_dump(config, f, sort_keys=False, allow_unicode=False)
    print(f"saved: {config_path}")


if __name__ == "__main__":
    main()
