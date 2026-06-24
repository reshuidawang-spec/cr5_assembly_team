#!/usr/bin/env python3
"""Low-risk real-robot validation for one cross-stylus horizontal touch."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from dobot_msgs_v4.srv import MovL
import yaml

from cross_probe_model import compute_branch_point, load_geometry
from geometry_utils import add, normalize, scale
from probe_touch import (
    PROBE_MOVL_ACCEPT_TIMEOUT_SEC,
    PROBE_SEGMENT_POSITION_TOLERANCE_MM,
    PROBE_SPIN_TIMEOUT_SEC,
    ProbeTouch,
    effective_probe_step,
    wait_segment_boundary_trigger,
    write_contact,
)
from workpiece_registration import (
    DEFAULT_SETUP_CONFIG,
    load_approved_face,
    require_pose_match,
    require_registered_start_pose,
    safe_pose_at_tangent_offsets,
)

PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_POSE_CONFIG = PROJECT_DIR / "config/measurement_poses.yaml"
REAL_EXECUTION_POSE_STATUSES = ("verified", "operator_approved_validation")


def angle_delta_deg(a, b):
    """Angle delta deg."""
    return (float(a) - float(b) + 180.0) % 360.0 - 180.0


def load_pose_config(path):
    """Load pose config."""
    with Path(path).open() as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"invalid pose config: {path}")
    return data


def vector_matches(actual, expected, tolerance=1e-6):
    """Vector matches."""
    return all(abs(float(a) - float(e)) <= tolerance for a, e in zip(actual, expected))


def resolve_orientation(args, approach):
    """Resolve orientation."""
    explicit = (args.rx, args.ry, args.rz)
    if any(value is not None for value in explicit):
        if not all(value is not None for value in explicit):
            raise ValueError("--rx --ry --rz must be provided together")
        if args.use_current_orientation:
            raise ValueError("--use-current-orientation cannot be combined with --rx/--ry/--rz")
        if args.standard_pose:
            raise ValueError("--standard-pose cannot be combined with --rx/--ry/--rz")
        return [float(args.rx), float(args.ry), float(args.rz)], "explicit --rx/--ry/--rz"

    if args.use_current_orientation:
        if args.standard_pose:
            raise ValueError("--use-current-orientation cannot be combined with --standard-pose")
        return None, "current robot orientation"

    if not args.standard_pose:
        raise ValueError("provide --standard-pose, --rx/--ry/--rz, or --use-current-orientation")

    data = load_pose_config(args.pose_config)
    poses = data.get("cross_probe", {})
    if args.standard_pose not in poses:
        raise ValueError(f"unknown --standard-pose {args.standard_pose!r}; choices: {', '.join(sorted(poses))}")
    pose = poses[args.standard_pose]
    status = pose.get("status", "unverified")
    if args.execute and status not in REAL_EXECUTION_POSE_STATUSES:
        raise ValueError(
            f"standard pose {args.standard_pose!r} status is {status!r}; "
            "real execution requires status: verified or operator_approved_validation"
        )
    purpose = getattr(args, "execution_purpose", "single_validation")
    if args.execute and status == "operator_approved_validation" and purpose != "single_validation":
        raise ValueError(
            f"standard pose {args.standard_pose!r} is approved only for the single-touch validation entry point; "
            f"{purpose} requires a verified pose"
        )
    if pose.get("branch") != args.branch:
        raise ValueError(f"standard pose {args.standard_pose!r} is for branch {pose.get('branch')!r}, not {args.branch!r}")
    expected_approach = pose.get("approach")
    if not isinstance(expected_approach, list) or len(expected_approach) != 3:
        raise ValueError(f"standard pose {args.standard_pose!r} has invalid approach")
    if not vector_matches(normalize(expected_approach), approach):
        raise ValueError(f"standard pose {args.standard_pose!r} approach does not match --approach")
    orientation = pose.get("flange_orientation_deg")
    if not isinstance(orientation, list) or len(orientation) != 3:
        raise ValueError(f"standard pose {args.standard_pose!r} has invalid flange_orientation_deg")
    return [float(v) for v in orientation], f"standard pose {args.standard_pose} ({status})"


def validate_args(args):
    """Validate args."""
    approach = normalize(args.approach)
    if abs(approach[2]) > args.max_abs_z:
        raise SystemExit("--approach must be horizontal for cross-stylus validation")
    if args.distance_mm <= 0:
        raise SystemExit("--distance-mm must be positive")
    if args.distance_mm > 1.0 and not args.allow_over_1mm:
        raise SystemExit("--distance-mm > 1 requires --allow-over-1mm")
    if args.retract_mm <= 0:
        raise SystemExit("--retract-mm must be positive")
    if args.retract_mm > 5.0:
        raise SystemExit("--retract-mm must be <= 5 for this validation script")
    if not 1 <= args.speed <= 5:
        raise SystemExit("--speed must be between 1 and 5")
    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")
    if getattr(args, "probe_step_mm", 0.0) < 0:
        raise SystemExit("--probe-step-mm cannot be negative")
    return approach


def format_pose(pose):
    """Format pose."""
    return "[" + ", ".join(f"{value:.4f}" for value in pose) + "]"


def format_orientation(orientation):
    """Format orientation."""
    return "[" + ", ".join(f"{value:.4f}" for value in orientation) + "]"


def orientation_error_deg(actual, target):
    """Orientation error deg."""
    return max(abs(angle_delta_deg(current, expected)) for current, expected in zip(actual, target))


def wait_until_pose(
    node,
    target_pose,
    position_tolerance_mm=0.2,
    orientation_tolerance_deg=0.5,
    timeout_sec=10.0,
    stop_on_di1=False,
):
    """Wait until pose."""
    deadline = time.monotonic() + timeout_sec
    last_pose = None
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if stop_on_di1 and node.di1:
            node.stop_repeated()
            raise RuntimeError("DI1 triggered during positioning move; motion stopped")
        if node.feed_pose is not None:
            last_pose = list(node.feed_pose)
            position_error = sum((last_pose[index] - target_pose[index]) ** 2 for index in range(3)) ** 0.5
            orientation_error = orientation_error_deg(last_pose[3:6], target_pose[3:6])
            if position_error <= position_tolerance_mm and orientation_error <= orientation_tolerance_deg:
                return last_pose
    pose = node.get_pose()
    position_error = sum((pose[index] - target_pose[index]) ** 2 for index in range(3)) ** 0.5
    orientation_error = orientation_error_deg(pose[3:6], target_pose[3:6])
    if position_error <= position_tolerance_mm and orientation_error <= orientation_tolerance_deg:
        return pose
    node.stop_repeated()
    observed = last_pose if last_pose is not None else pose
    raise RuntimeError(
        "positioning move did not reach requested pose; "
        f"last observed pose was {format_pose(observed)}"
    )


def branch_result(args, pose, approach):
    """Branch result."""
    geometry = load_geometry(args.geometry)
    return compute_branch_point(
        geometry,
        pose,
        args.branch,
        euler_sequence=args.euler_sequence,
        approach=approach,
    )


def vector_fields(prefix, values, count, digits=4):
    """Vector fields."""
    row = {}
    for index in range(count):
        value = "" if values is None or index >= len(values) else f"{float(values[index]):.{digits}f}"
        row[f"{prefix}_{index + 1}"] = value
    return row


def pose_fields(prefix, pose):
    """Pose fields."""
    names = ("x", "y", "z", "rx", "ry", "rz")
    return {f"{prefix}_{name}": f"{float(pose[index]):.4f}" for index, name in enumerate(names)}


def snapshot_fields(prefix, snapshot):
    """Snapshot fields."""
    snapshot = snapshot or {}
    row = {
        f"{prefix}_feed_sequence": str(snapshot.get("sequence", "")),
        f"{prefix}_feed_wall_time": (
            "" if snapshot.get("wall_time") is None else f"{float(snapshot['wall_time']):.6f}"
        ),
        f"{prefix}_digital_input_bits": (
            "" if snapshot.get("digital_input_bits") is None else str(snapshot["digital_input_bits"])
        ),
        f"{prefix}_di1": "" if snapshot.get("di1") is None else str(int(bool(snapshot["di1"]))),
    }
    row.update(vector_fields(f"{prefix}_joint", snapshot.get("joints"), 6, digits=6))
    return row


def feed_age_ms(snapshot):
    """Feed age ms."""
    if not snapshot or snapshot.get("monotonic_time") is None:
        return ""
    if snapshot.get("age_ms") is not None:
        return f"{float(snapshot['age_ms']):.3f}"
    return f"{(time.monotonic() - float(snapshot['monotonic_time'])) * 1000.0:.3f}"


def make_row(
    args,
    start_pose,
    trigger_pose,
    stop_pose,
    start_branch,
    trigger_branch,
    stop_branch,
    approach,
    orientation_source,
    orientation_error,
    start_snapshot,
    trigger_snapshot,
    stop_snapshot,
):
    """Make row."""
    overtravel = sum((stop_pose[i] - trigger_pose[i]) * approach[i] for i in range(3))
    row = {
        "timestamp": f"{time.time():.3f}",
        "session_id": getattr(args, "session_id", "") or "",
        "setup_id": getattr(args, "setup_id", "") or "",
        "workpiece_id": getattr(args, "workpiece_id", "") or "",
        "face_id": getattr(args, "face_id", "") or "",
        "sample_id": getattr(args, "sample_id", "") or "",
        "tangent_y_offset_mm": getattr(args, "tangent_y_offset_mm", "") or "",
        "tangent_z_offset_mm": getattr(args, "tangent_z_offset_mm", "") or "",
        "operator_note": getattr(args, "operator_note", "") or "",
        "standard_pose": args.standard_pose or "",
        "orientation_source": orientation_source,
        "orientation_error_deg": f"{orientation_error:.6f}",
        "branch": args.branch,
        "approach_x": f"{approach[0]:.6f}",
        "approach_y": f"{approach[1]:.6f}",
        "approach_z": f"{approach[2]:.6f}",
        "distance_mm": f"{args.distance_mm:.4f}",
        "retract_mm": f"{args.retract_mm:.4f}",
        "probe_step_mm": f"{float(getattr(args, 'probe_step_mm', 0.0) or 0.0):.4f}",
        "speed": str(args.speed),
        "euler_sequence": args.euler_sequence,
        "trigger_feed_age_ms": feed_age_ms(trigger_snapshot),
        "stop_overtravel_along_approach_mm": f"{overtravel:.4f}",
        "start_flange_x": f"{start_pose[0]:.4f}",
        "start_flange_y": f"{start_pose[1]:.4f}",
        "start_flange_z": f"{start_pose[2]:.4f}",
        "start_rx": f"{start_pose[3]:.4f}",
        "start_ry": f"{start_pose[4]:.4f}",
        "start_rz": f"{start_pose[5]:.4f}",
        "flange_x": f"{trigger_pose[0]:.4f}",
        "flange_y": f"{trigger_pose[1]:.4f}",
        "flange_z": f"{trigger_pose[2]:.4f}",
        "rx": f"{trigger_pose[3]:.4f}",
        "ry": f"{trigger_pose[4]:.4f}",
        "rz": f"{trigger_pose[5]:.4f}",
        "ball_center_x": f"{trigger_branch['ball_center_mm'][0]:.4f}",
        "ball_center_y": f"{trigger_branch['ball_center_mm'][1]:.4f}",
        "ball_center_z": f"{trigger_branch['ball_center_mm'][2]:.4f}",
        "surface_x_est": f"{trigger_branch['surface_contact_estimate_mm'][0]:.4f}",
        "surface_y_est": f"{trigger_branch['surface_contact_estimate_mm'][1]:.4f}",
        "surface_z_est": f"{trigger_branch['surface_contact_estimate_mm'][2]:.4f}",
        "start_ball_center_x": f"{start_branch['ball_center_mm'][0]:.4f}",
        "start_ball_center_y": f"{start_branch['ball_center_mm'][1]:.4f}",
        "start_ball_center_z": f"{start_branch['ball_center_mm'][2]:.4f}",
        "stop_flange_x": f"{stop_pose[0]:.4f}",
        "stop_flange_y": f"{stop_pose[1]:.4f}",
        "stop_flange_z": f"{stop_pose[2]:.4f}",
        "stop_ball_center_x": f"{stop_branch['ball_center_mm'][0]:.4f}",
        "stop_ball_center_y": f"{stop_branch['ball_center_mm'][1]:.4f}",
        "stop_ball_center_z": f"{stop_branch['ball_center_mm'][2]:.4f}",
    }
    row.update(snapshot_fields("start", start_snapshot))
    row.update(snapshot_fields("trigger", trigger_snapshot))
    row.update(snapshot_fields("stop", stop_snapshot))
    return row


def run_cross_probe_cycle(node, args, prefix=""):
    """Run cross probe cycle."""
    try:
        approach = validate_args(args)
        orientation, orientation_source = resolve_orientation(args, approach)
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    if args.orientation_tolerance_deg < 0:
        raise ValueError("--orientation-tolerance-deg must be non-negative")

    node.wait_fresh_feed()
    if node.read_di1() or node.di1:
        raise RuntimeError("DI1 is already triggered; clear the probe before validation")

    start_snapshot = node.feed_snapshot()
    start_pose = node.get_pose()
    if args.execute:
        try:
            registered_face = load_approved_face(args, approach)
            registered_safe_start = getattr(args, "registered_safe_start_pose", None)
            if registered_safe_start is None:
                require_registered_start_pose(
                    registered_face,
                    start_pose,
                    orientation_tolerance_deg=args.orientation_tolerance_deg,
                )
            else:
                y_offset = getattr(args, "tangent_y_offset_mm", None)
                z_offset = getattr(args, "tangent_z_offset_mm", None)
                if y_offset in (None, "") or z_offset in (None, ""):
                    raise ValueError("registered sample probing requires approved tangent offsets")
                approved_sample_start = safe_pose_at_tangent_offsets(
                    registered_face,
                    float(y_offset),
                    float(z_offset),
                )
                require_pose_match(
                    registered_safe_start,
                    approved_sample_start,
                    position_tolerance_mm=1e-6,
                    orientation_tolerance_deg=1e-6,
                    label="approved registered face sample pose",
                )
                require_pose_match(
                    start_pose,
                    approved_sample_start,
                    orientation_tolerance_deg=args.orientation_tolerance_deg,
                    label="registered face sample safe start pose",
                )
        except ValueError as exc:
            raise RuntimeError(f"workpiece registration gate rejected real probing: {exc}") from exc
    current_orientation = start_pose[3:6]
    measurement_orientation = orientation if orientation is not None else current_orientation
    setup_pose = start_pose.copy()
    setup_pose[3:6] = measurement_orientation
    target_pose = start_pose.copy()
    target_pose[:3] = add(start_pose[:3], scale(approach, args.distance_mm))
    target_pose[3:6] = measurement_orientation
    retract_pose = start_pose.copy()
    retract_pose[:3] = add(start_pose[:3], scale(approach, -args.retract_mm))
    retract_pose[3:6] = measurement_orientation
    start_branch = branch_result(args, start_pose, approach)
    setup_branch = branch_result(args, setup_pose, approach)
    orientation_error = orientation_error_deg(current_orientation, measurement_orientation)

    print(f"{prefix}current flange pose: {format_pose(start_pose)}")
    print(f"{prefix}measurement orientation: {format_orientation(measurement_orientation)} ({orientation_source})")
    print(f"{prefix}orientation max error: {orientation_error:.4f} deg")
    print(f"{prefix}setup flange pose:   {format_pose(setup_pose)}")
    print(f"{prefix}target flange pose:  {format_pose(target_pose)}")
    print(f"{prefix}retract pose:        {format_pose(retract_pose)}")
    print(f"{prefix}branch: {args.branch}")
    print(
        f"{prefix}start ball center: "
        f"[{start_branch['ball_center_mm'][0]:.4f}, "
        f"{start_branch['ball_center_mm'][1]:.4f}, "
        f"{start_branch['ball_center_mm'][2]:.4f}]"
    )
    print(
        f"{prefix}start surface estimate: "
        f"[{start_branch['surface_contact_estimate_mm'][0]:.4f}, "
        f"{start_branch['surface_contact_estimate_mm'][1]:.4f}, "
        f"{start_branch['surface_contact_estimate_mm'][2]:.4f}]"
    )
    print(
        f"{prefix}setup surface estimate: "
        f"[{setup_branch['surface_contact_estimate_mm'][0]:.4f}, "
        f"{setup_branch['surface_contact_estimate_mm'][1]:.4f}, "
        f"{setup_branch['surface_contact_estimate_mm'][2]:.4f}]"
    )
    print(f"{prefix}probe step: {effective_probe_step(args.distance_mm, getattr(args, 'probe_step_mm', 0.0)):.4f} mm")

    if not args.execute:
        print(
            f"{prefix}dry-run only; add --execute"
            f"{' --allow-over-1mm' if args.distance_mm > 1.0 else ''}"
            f" for the {args.distance_mm:g}mm horizontal validation move"
        )
        return None

    node.set_speed(args.speed)
    if orientation_error > args.orientation_tolerance_deg:
        if not args.allow_orientation_setup:
            raise RuntimeError(
                "current orientation differs from the measurement orientation; "
                "run dry-run first and add --allow-orientation-setup only after confirming clearance"
            )
        node.move_l(setup_pose, timeout_sec=10.0)
        wait_until_pose(
            node,
            setup_pose,
            orientation_tolerance_deg=args.orientation_tolerance_deg,
            timeout_sec=10.0,
            stop_on_di1=True,
        )
        node.wait_fresh_feed()
        start_snapshot = node.feed_snapshot()
        start_pose = node.get_pose()
        current_orientation = start_pose[3:6]
        setup_pose = start_pose.copy()
        setup_pose[3:6] = measurement_orientation
        target_pose = start_pose.copy()
        target_pose[:3] = add(start_pose[:3], scale(approach, args.distance_mm))
        target_pose[3:6] = measurement_orientation
        retract_pose = start_pose.copy()
        retract_pose[:3] = add(start_pose[:3], scale(approach, -args.retract_mm))
        retract_pose[3:6] = measurement_orientation
        start_branch = branch_result(args, start_pose, approach)
        orientation_error = orientation_error_deg(current_orientation, measurement_orientation)
        print(f"{prefix}post-setup orientation max error: {orientation_error:.4f} deg")
        if orientation_error > args.orientation_tolerance_deg:
            raise RuntimeError(
                "orientation setup did not reach the requested measurement orientation; "
                "refusing to probe"
            )

    probe_step = effective_probe_step(args.distance_mm, getattr(args, "probe_step_mm", 0.0))
    issued_distance = 0.0
    segment_target = start_pose.copy()
    move_future = None
    immediate_move_result = None

    def issue_next_segment():
        """Issue next segment."""
        nonlocal issued_distance, segment_target, move_future, immediate_move_result
        remaining = args.distance_mm - issued_distance
        step = min(probe_step, remaining)
        issued_distance += step
        segment_target = start_pose.copy()
        segment_target[:3] = add(start_pose[:3], scale(approach, issued_distance))
        segment_target[3:6] = measurement_orientation
        move_req = MovL.Request()
        move_req.mode = False
        move_req.a, move_req.b, move_req.c, move_req.d, move_req.e, move_req.f = segment_target
        move_future, immediate_move_result = node.call_async_checked(
            node.movl_cli,
            move_req,
            accept_timeout_sec=PROBE_MOVL_ACCEPT_TIMEOUT_SEC,
        )
        if immediate_move_result is not None:
            print(f"{prefix}MovL accepted immediately")

    issue_next_segment()

    triggered = False
    trigger_pose = None
    trigger_snapshot = None
    reached_target = False
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
        if immediate_move_result is None:
            immediate_move_result = node.check_ready_future(node.movl_cli, move_future)
        if node.last_feed_time is None or time.monotonic() - node.last_feed_time > 0.2:
            node.stop_repeated()
            raise RuntimeError("FeedInfo became stale during cross probing; motion stopped")
        snapshot = node.feed_snapshot()
        if snapshot.get("di1"):
            triggered = True
            trigger_snapshot = dict(snapshot)
            if trigger_snapshot.get("monotonic_time") is not None:
                trigger_snapshot["age_ms"] = (
                    time.monotonic() - float(trigger_snapshot["monotonic_time"])
                ) * 1000.0
            trigger_pose = trigger_snapshot["pose"] if trigger_snapshot.get("pose") is not None else None
            node.stop_fast_then_confirm()
            break
        if snapshot.get("pose") is not None:
            distance_to_target = sum((snapshot["pose"][i] - segment_target[i]) ** 2 for i in range(3)) ** 0.5
            segment_tolerance = min(PROBE_SEGMENT_POSITION_TOLERANCE_MM, max(0.01, probe_step * 0.1))
            if distance_to_target <= segment_tolerance:
                if issued_distance >= args.distance_mm - 1e-9:
                    reached_target = True
                    break
                boundary_trigger = wait_segment_boundary_trigger(
                    node,
                    deadline,
                    "FeedInfo became stale during cross probing; motion stopped",
                )
                if boundary_trigger is not None:
                    triggered = True
                    trigger_snapshot = dict(boundary_trigger)
                    if trigger_snapshot.get("monotonic_time") is not None:
                        trigger_snapshot["age_ms"] = (
                            time.monotonic() - float(trigger_snapshot["monotonic_time"])
                        ) * 1000.0
                    trigger_pose = trigger_snapshot["pose"] if trigger_snapshot.get("pose") is not None else None
                    node.stop_fast_then_confirm()
                    break
                issue_next_segment()

    row = None
    if not triggered:
        node.stop_repeated()
        stop_pose = node.get_pose()
        reason = "target reached" if reached_target else "timeout"
        print(f"{prefix}no DI1 trigger before {reason}; final flange pose: {format_pose(stop_pose)}")
    else:
        stop_pose = node.get_pose()
        stop_snapshot = node.feed_snapshot()
        measurement_pose = trigger_pose if trigger_pose is not None else stop_pose
        trigger_branch = branch_result(args, measurement_pose, approach)
        stop_branch = branch_result(args, stop_pose, approach)
        row = make_row(
            args,
            start_pose,
            measurement_pose,
            stop_pose,
            start_branch,
            trigger_branch,
            stop_branch,
            approach,
            orientation_source,
            orientation_error,
            start_snapshot,
            trigger_snapshot,
            stop_snapshot,
        )
        print(f"{prefix}triggered at flange pose: {format_pose(measurement_pose)}")
        print(f"{prefix}stopped at flange pose:   {format_pose(stop_pose)}")

    try:
        node.move_l(retract_pose, timeout_sec=10.0)
        wait_until_pose(
            node,
            retract_pose,
            orientation_tolerance_deg=args.orientation_tolerance_deg,
            timeout_sec=10.0,
        )
        print(f"{prefix}retracted opposite approach direction")
    except Exception:
        if triggered:
            node.stop_repeated()
        raise
    node.wait_fresh_feed()
    probe_still_triggered = bool(node.read_di1() or node.di1)
    if row is not None:
        write_contact(Path(args.output), row)
        print(f"{prefix}saved: {args.output}")
    if probe_still_triggered:
        raise RuntimeError("DI1 remains triggered after retract; leave motion stopped and inspect the probe")
    return row


def add_cross_probe_arguments(parser):
    """Add cross probe arguments."""
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    parser.add_argument("--branch", required=True, choices=("x_pos", "x_neg", "y_pos", "y_neg"))
    parser.add_argument("--approach", nargs=3, type=float, required=True, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--distance-mm", type=float, default=1.0)
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
    parser.add_argument("--max-abs-z", type=float, default=0.05)
    parser.add_argument("--geometry", default=str(Path(__file__).resolve().parents[1] / "config/cross_probe_geometry.yaml"))
    parser.add_argument("--pose-config", default=str(DEFAULT_POSE_CONFIG))
    parser.add_argument("--standard-pose", default="x_neg_y_neg_verified")
    parser.add_argument("--rx", type=float, help="explicit standard flange RX orientation")
    parser.add_argument("--ry", type=float, help="explicit standard flange RY orientation")
    parser.add_argument("--rz", type=float, help="explicit standard flange RZ orientation")
    parser.add_argument("--use-current-orientation", action="store_true", help="probe with the current robot orientation")
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--allow-orientation-setup", action="store_true", help="allow a real setup MovL to the standard orientation before probing")
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--output", default="data/cross_probe_contacts.csv")
    parser.add_argument("--session-id", default="", help="measurement session identifier written to CSV")
    parser.add_argument("--setup-config", default=str(DEFAULT_SETUP_CONFIG), help="registered workpiece setup YAML")
    parser.add_argument("--setup-id", default="", help="approved workpiece setup identifier written to CSV")
    parser.add_argument("--workpiece-id", default="", help="workpiece identifier written to CSV")
    parser.add_argument("--face-id", default="", help="measured face/surface identifier written to CSV")
    parser.add_argument("--operator-note", default="", help="free-form note written to CSV")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    add_cross_probe_arguments(parser)
    args = parser.parse_args()

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        run_cross_probe_cycle(node, args)
    except (RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
