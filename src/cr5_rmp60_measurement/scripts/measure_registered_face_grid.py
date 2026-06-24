#!/usr/bin/env python3
"""Collect a bounded grid on one approved cross-probe workpiece face."""
import argparse
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy

from cross_probe_touch import format_pose, resolve_orientation, run_cross_probe_cycle, validate_args, wait_until_pose
from geometry_utils import add, scale
from probe_touch import ProbeTouch
from workpiece_registration import (
    DEFAULT_SETUP_CONFIG,
    load_setup_config,
    require_pose_match,
    safe_pose_at_tangent_offsets,
)

PROJECT_DIR = Path(__file__).resolve().parents[1]


def load_face(args):
    """Load face."""
    config = load_setup_config(args.setup_config)
    setup = config["setups"].get(args.setup_id)
    if not isinstance(setup, dict) or setup.get("workpiece_id") != args.workpiece_id:
        raise SystemExit("unknown or mismatched registered workpiece setup")
    face = (setup.get("faces") or {}).get(args.face_id)
    if not isinstance(face, dict) or face.get("status") != "approved":
        raise SystemExit("registered face is missing or not approved")
    return face


def plan_samples(face, y_offsets, z_offsets):
    """Plan samples."""
    samples = []
    for row_index, z_offset in enumerate(z_offsets):
        row_y = y_offsets if row_index % 2 == 0 else list(reversed(y_offsets))
        for y_offset in row_y:
            safe_pose = safe_pose_at_tangent_offsets(face, y_offset, z_offset)
            samples.append((f"y{y_offset:+g}_z{z_offset:+g}", y_offset, z_offset, safe_pose))
    return samples


def clearance_pose(safe_pose, face):
    """Clearance pose."""
    return add(safe_pose[:3], scale(face["approach"], -float(face["retract_mm"]))) + safe_pose[3:6]


def touch_args(args, face, sample_id, y_offset, z_offset, safe_pose):
    """Touch args."""
    return SimpleNamespace(
        execute=args.execute,
        execution_purpose="registered_grid_collection",
        session_id=args.session_id,
        setup_config=args.setup_config,
        setup_id=args.setup_id,
        workpiece_id=args.workpiece_id,
        face_id=args.face_id,
        sample_id=sample_id,
        tangent_y_offset_mm=f"{y_offset:.4f}",
        tangent_z_offset_mm=f"{z_offset:.4f}",
        operator_note=args.operator_note,
        standard_pose=face["standard_pose"],
        pose_config=args.pose_config,
        branch=face["branch"],
        approach=face["approach"],
        distance_mm=float(face["max_search_mm"]),
        allow_over_1mm=True,
        retract_mm=float(face["retract_mm"]),
        speed=int(face["max_speed"]),
        timeout=args.timeout,
        max_abs_z=0.05,
        geometry=args.geometry,
        rx=None,
        ry=None,
        rz=None,
        use_current_orientation=False,
        orientation_tolerance_deg=args.orientation_tolerance_deg,
        allow_orientation_setup=False,
        euler_sequence=args.euler_sequence,
        output=args.output,
        registered_safe_start_pose=safe_pose,
    )


def require_probe_clear(node, label):
    """Require probe clear."""
    node.wait_fresh_feed()
    if node.read_di1() or node.di1:
        raise RuntimeError(f"DI1 is triggered before {label}; refusing to move")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="execute the registered bounded grid")
    parser.add_argument("--setup-config", default=str(DEFAULT_SETUP_CONFIG))
    parser.add_argument("--setup-id", required=True)
    parser.add_argument("--workpiece-id", required=True)
    parser.add_argument("--face-id", required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--y-offsets", nargs="+", type=float, default=[0.0, -5.0, 5.0])
    parser.add_argument("--z-offsets", nargs="+", type=float, default=[0.0, -5.0])
    parser.add_argument("--entry-y-offset", type=float, default=0.0)
    parser.add_argument("--entry-z-offset", type=float, default=0.0)
    parser.add_argument("--settle-sec", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--geometry", default=str(PROJECT_DIR / "config/cross_probe_geometry.yaml"))
    parser.add_argument("--pose-config", default=str(PROJECT_DIR / "config/measurement_poses.yaml"))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--operator-note", default="registered_face_grid_collection")
    parser.add_argument("--output", default="data/registered_face_contacts.csv")
    args = parser.parse_args()
    if args.settle_sec < 0:
        raise SystemExit("--settle-sec cannot be negative")

    face = load_face(args)
    samples = plan_samples(face, args.y_offsets, args.z_offsets)
    entry_safe = safe_pose_at_tangent_offsets(face, args.entry_y_offset, args.entry_z_offset)
    entry_clearance = clearance_pose(entry_safe, face)
    first_sample_args = touch_args(args, face, *samples[0])
    approach = validate_args(first_sample_args)
    try:
        resolve_orientation(first_sample_args, approach)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    print(f"registered face: {args.setup_id}/{args.face_id}")
    print(f"required entry clearance pose: {format_pose(entry_clearance)}")
    for sample_id, y_offset, z_offset, safe_pose in samples:
        print(f"{sample_id} clearance: {format_pose(clearance_pose(safe_pose, face))}")
        print(f"{sample_id} safe:      {format_pose(safe_pose)}")
        target = add(safe_pose[:3], scale(face["approach"], float(face["max_search_mm"]))) + safe_pose[3:6]
        print(f"{sample_id} target:    {format_pose(target)}")
    if not args.execute:
        print("dry-run only; add --execute to collect the approved registered face grid")
        return

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        node.wait_fresh_feed()
        current_pose = node.get_pose()
        require_pose_match(current_pose, entry_clearance, label="registered collection entry clearance pose")
        require_probe_clear(node, "registered collection entry")
        node.set_speed(int(face["max_speed"]))

        rows = []
        for sample_id, y_offset, z_offset, safe_pose in samples:
            sample_clearance = clearance_pose(safe_pose, face)
            print(f"[{sample_id}] moving at retract clearance")
            require_probe_clear(node, f"clearance move for registered sample {sample_id}")
            node.move_l(sample_clearance, timeout_sec=10.0)
            wait_until_pose(node, sample_clearance, timeout_sec=10.0, stop_on_di1=True)
            print(f"[{sample_id}] moving to sample safe start")
            require_probe_clear(node, f"safe-start move for registered sample {sample_id}")
            node.move_l(safe_pose, timeout_sec=10.0)
            wait_until_pose(node, safe_pose, timeout_sec=10.0, stop_on_di1=True)
            row = run_cross_probe_cycle(
                node,
                touch_args(args, face, sample_id, y_offset, z_offset, safe_pose),
                prefix=f"[{sample_id}] ",
            )
            if row is None:
                raise RuntimeError(f"no DI1 trigger at registered sample {sample_id}; collection aborted")
            require_probe_clear(node, f"transition after registered sample {sample_id}")
            rows.append(row)
            time.sleep(args.settle_sec)
        print(f"registered face collection complete: {len(rows)} triggered samples")
        print(f"saved: {args.output}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
