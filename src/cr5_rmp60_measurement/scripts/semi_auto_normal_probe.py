#!/usr/bin/env python3
"""Semi-automatic normal probing for one cross-stylus ruby ball.

The operator manually places the selected ruby ball near the calibration
sphere. This script uses a temporary single-branch fit to compute the current
ruby ball centre, searches along the two-sphere centre line for a short
distance, records the trigger row, and optionally refits the branch model.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy

from calibrate_branch_sphere_absolute import read_rows, solve
from cross_probe_model import DEFAULT_GEOMETRY, ball_radius_mm, euler_to_matrix, load_geometry, mat_vec
from jog_and_record_contacts import (
    format_pose,
    current_pose,
    fresh_feed_di1,
    issue_movl,
    pose_fields,
    retract_opposite,
    vector_fields,
    wait_jog,
)
from probe_touch import ProbeTouch


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_FIT = PROJECT_DIR / "data/260603.1/yneg_new_5pts_absolute_fit_20260603.json"
DEFAULT_OUTPUT = PROJECT_DIR / "data/semi_auto_yneg_normal_contacts.csv"
DEFAULT_REFIT_JSON = PROJECT_DIR / "data/semi_auto_yneg_normal_fit.json"
DEFAULT_REFIT_RESIDUALS = PROJECT_DIR / "data/semi_auto_yneg_normal_fit_residuals.csv"


def load_fit(path):
    """Load fit."""
    with Path(path).open() as f:
        data = json.load(f)
    required = ("sphere_center_mm", "local_ball_offset_mm")
    missing = [field for field in required if field not in data]
    if missing:
        raise ValueError(f"fit JSON missing fields: {missing}")
    sphere_center = [float(v) for v in data["sphere_center_mm"]]
    local_offset = [float(v) for v in data["local_ball_offset_mm"]]
    if len(sphere_center) != 3 or len(local_offset) != 3:
        raise ValueError("fit JSON sphere_center_mm and local_ball_offset_mm must contain 3 values")
    return data, sphere_center, local_offset


def vector_norm(values):
    """Vector norm."""
    return math.sqrt(sum(float(v) * float(v) for v in values))


def normalize(values, label):
    """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
    length = vector_norm(values)
    if length <= 1e-12:
        raise ValueError(f"{label} cannot be zero")
    return [float(v) / length for v in values]


def compute_plan(pose, sphere_center, local_offset, euler_sequence, search_mm, approach_override=None):
    """Compute plan."""
    rotation = euler_to_matrix(euler_sequence, pose[3:6])
    offset_base = mat_vec(rotation, local_offset)
    ball_center = [float(pose[index]) + offset_base[index] for index in range(3)]
    to_sphere = [sphere_center[index] - ball_center[index] for index in range(3)]
    approach = (
        normalize(approach_override, "override approach")
        if approach_override is not None
        else normalize(to_sphere, "computed approach")
    )
    target_pose = list(pose)
    for index in range(3):
        target_pose[index] += approach[index] * search_mm
    return {
        "flange_pose": [float(v) for v in pose],
        "rotation": rotation,
        "sphere_center_mm": sphere_center,
        "local_ball_offset_mm": local_offset,
        "ball_center_mm": ball_center,
        "approach": approach,
        "center_distance_mm": vector_norm(to_sphere),
        "target_pose": target_pose,
    }


def print_plan(plan, search_mm, retract_mm):
    """Print plan."""
    print(f"current flange pose: {format_pose(plan['flange_pose'])}")
    print(
        "temporary sphere center C_s: "
        f"[{plan['sphere_center_mm'][0]:.4f}, {plan['sphere_center_mm'][1]:.4f}, {plan['sphere_center_mm'][2]:.4f}]"
    )
    print(
        "current ruby ball center C_p: "
        f"[{plan['ball_center_mm'][0]:.4f}, {plan['ball_center_mm'][1]:.4f}, {plan['ball_center_mm'][2]:.4f}]"
    )
    print(
        "computed approach: "
        f"[{plan['approach'][0]:.6f}, {plan['approach'][1]:.6f}, {plan['approach'][2]:.6f}]"
    )
    print(f"estimated C_p -> C_s distance: {plan['center_distance_mm']:.4f} mm")
    print(f"search distance: {search_mm:.4f} mm")
    print(f"target flange pose: {format_pose(plan['target_pose'])}")
    print(f"opposite retract after trigger: {retract_mm:.4f} mm")


def write_stable_row(path, row):
    """Write stable row."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    fieldnames = list(row.keys())
    if exists:
        with path.open(newline="") as f:
            existing_header = next(csv.reader(f), None)
        if existing_header != fieldnames:
            raise ValueError(
                f"CSV header mismatch for {path}; use a new --output or archive the existing file first"
            )
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def make_contact_row(args, plan, trigger_snapshot, stop_pose):
    """Make contact row."""
    trigger_pose = trigger_snapshot.get("pose") or stop_pose
    row = {
        "timestamp": f"{time.time():.3f}",
        "sample_index": "",
        "session_id": args.session_id,
        "setup_id": args.setup_id,
        "workpiece_id": args.workpiece_id,
        "artifact_id": args.artifact_id,
        "artifact_type": args.artifact_type,
        "physical_ball_id": args.physical_ball_id,
        "branch": args.branch,
        "standard_pose": "semi_auto_normal",
        "face_id": "standard_sphere_semi_auto_normal",
        "operator_note": args.operator_note,
        "source_command": "semi_auto_normal",
        "fit_initial_json": str(args.fit_json),
        "fit_initial_rms_mm": "",
        "fit_initial_max_residual_mm": "",
        "search_mm": f"{args.search_mm:.4f}",
        "speed": str(args.speed),
        "auto_retract_mm": f"{args.retract_mm:.4f}",
        "approach_x": f"{plan['approach'][0]:.6f}",
        "approach_y": f"{plan['approach'][1]:.6f}",
        "approach_z": f"{plan['approach'][2]:.6f}",
        "flange_x": f"{trigger_pose[0]:.4f}",
        "flange_y": f"{trigger_pose[1]:.4f}",
        "flange_z": f"{trigger_pose[2]:.4f}",
        "rx": f"{trigger_pose[3]:.4f}",
        "ry": f"{trigger_pose[4]:.4f}",
        "rz": f"{trigger_pose[5]:.4f}",
        "planned_ball_center_x": f"{plan['ball_center_mm'][0]:.4f}",
        "planned_ball_center_y": f"{plan['ball_center_mm'][1]:.4f}",
        "planned_ball_center_z": f"{plan['ball_center_mm'][2]:.4f}",
        "planned_center_distance_mm": f"{plan['center_distance_mm']:.4f}",
        "trigger_feed_sequence": str(trigger_snapshot.get("sequence", "")),
        "trigger_feed_wall_time": (
            "" if trigger_snapshot.get("wall_time") is None else f"{float(trigger_snapshot['wall_time']):.6f}"
        ),
        "trigger_digital_input_bits": (
            "" if trigger_snapshot.get("digital_input_bits") is None else str(trigger_snapshot["digital_input_bits"])
        ),
        "trigger_di1": "" if trigger_snapshot.get("di1") is None else str(int(bool(trigger_snapshot["di1"]))),
    }
    initial_fit = plan.get("initial_fit", {})
    if initial_fit.get("rms_residual_mm") is not None:
        row["fit_initial_rms_mm"] = f"{float(initial_fit['rms_residual_mm']):.6f}"
    if initial_fit.get("max_residual_mm") is not None:
        row["fit_initial_max_residual_mm"] = f"{float(initial_fit['max_residual_mm']):.6f}"
    row.update(pose_fields("start_flange", plan["flange_pose"]))
    row.update(pose_fields("target_flange", plan["target_pose"]))
    row.update(pose_fields("trigger_flange", trigger_pose))
    row.update(pose_fields("stop_flange", stop_pose))
    row.update(vector_fields("trigger_joint", trigger_snapshot.get("joints"), 6))
    stop_delta = [stop_pose[index] - trigger_pose[index] for index in range(3)]
    row["stop_overtravel_along_approach_mm"] = f"{float(np.dot(stop_delta, plan['approach'])):.6f}"
    return row


def write_residual_csv(path, rows):
    """Write residual csv."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def print_refit(result, branch):
    """Print refit."""
    center = result["sphere_center_mm"]
    offset = result["local_ball_offset_mm"]
    print("updated single-branch fit")
    print(f"  branch: {branch}")
    print(f"  rows: {result['rows']}")
    print(f"  rank: {result['rank']}")
    print(f"  condition: {result['condition']:.6f}" if result["condition"] is not None else "  condition: n/a")
    print(f"  sphere_center_mm: [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]")
    print(f"  local_ball_offset_mm: [{offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}]")
    print(f"  rms_residual_mm: {result['rms_residual_mm']:.6f}")
    print(f"  max_residual_mm: {result['max_residual_mm']:.6f}")


def refit_after_contact(args):
    """Refit after contact."""
    inputs = list(args.refit_input or [])
    if str(args.output) not in inputs:
        inputs.append(str(args.output))
    geometry = load_geometry(args.geometry)
    probe_radius = args.probe_radius_mm if args.probe_radius_mm is not None else ball_radius_mm(geometry)
    rows = read_rows(inputs, args.branch)
    result = solve(rows, args.sphere_radius_mm, probe_radius, args.euler_sequence)
    result["branch"] = args.branch
    result["source_csv"] = inputs
    result["geometry"] = str(args.geometry)
    result["euler_sequence"] = args.euler_sequence
    print_refit(result, args.branch)
    if args.refit_json:
        output = Path(args.refit_json)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(f"saved refit JSON: {output}")
    if args.refit_residual_output:
        write_residual_csv(args.refit_residual_output, result["residual_rows"])
        print(f"saved refit residuals: {args.refit_residual_output}")


def current_pose_from_robot(args):
    """Current pose from robot."""
    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(args.service_timeout_sec)
        node.wait_fresh_feed()
        return node, current_pose(node, max_age_sec=0.5)
    except Exception:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise


def execute_probe(node, args, plan):
    """Execute probe."""
    node.wait_fresh_feed()
    if fresh_feed_di1(node):
        raise RuntimeError("DI1 is already triggered before probing; retract or clear the probe first")
    node.set_speed(args.speed)
    future = issue_movl(node, plan["target_pose"])
    trigger_snapshot, reached_target = wait_jog(
        node,
        plan["target_pose"],
        args.timeout_sec,
        args.position_tolerance_mm,
        args.orientation_tolerance_deg,
    )
    if future.done():
        node.check_ready_future(node.movl_cli, future)
    if trigger_snapshot is None:
        print("reached target; no DI1 trigger")
        if args.no_retract_on_miss:
            final_pose = current_pose(node, max_age_sec=0.5)
            print(f"held at miss target: {format_pose(final_pose)}")
            print(f"DI1 at miss target: {int(fresh_feed_di1(node, max_age_sec=0.5))}")
            return None
        retract_pose = retract_opposite(node, plan["flange_pose"], plan["target_pose"], args.retract_mm, args.timeout_sec)
        if retract_pose is not None:
            print(f"retracted:    {format_pose(retract_pose)}")
        print(f"DI1 after retract: {int(fresh_feed_di1(node, max_age_sec=0.5))}")
        return None
    stop_pose = current_pose(node, max_age_sec=0.5)
    row = make_contact_row(args, plan, trigger_snapshot, stop_pose)
    write_stable_row(args.output, row)
    print(f"DI1 triggered; saved: {args.output}")
    print(f"trigger pose: {format_pose(trigger_snapshot.get('pose') or stop_pose)}")
    print(f"stop pose:    {format_pose(stop_pose)}")
    retract_pose = retract_opposite(node, plan["flange_pose"], plan["target_pose"], args.retract_mm, args.timeout_sec)
    if retract_pose is not None:
        print(f"retracted:    {format_pose(retract_pose)}")
    print(f"DI1 after retract: {int(fresh_feed_di1(node, max_age_sec=0.5))}")
    return row


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--fit-json", default=str(DEFAULT_FIT), help="temporary absolute fit JSON used as C_s / p")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="CSV output for new semi-auto contacts")
    parser.add_argument("--branch", default="y_neg")
    parser.add_argument("--session-id", default="")
    parser.add_argument("--setup-id", default="semi_auto_normal_probe")
    parser.add_argument("--workpiece-id", default="calibration_sphere_20mm")
    parser.add_argument("--artifact-id", default="standard_sphere_20mm")
    parser.add_argument("--artifact-type", default="sphere")
    parser.add_argument("--physical-ball-id", default="1")
    parser.add_argument("--operator-note", default="")
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float)
    parser.add_argument("--search-mm", type=float, default=0.2, help="short normal search distance, max 1.0mm")
    parser.add_argument("--approach", nargs=3, type=float, metavar=("DX", "DY", "DZ"), help="override search direction in base frame")
    parser.add_argument(
        "--allow-search-over-1mm",
        action="store_true",
        help="allow 1-3mm search only after the same line has been operator/empty-path checked",
    )
    parser.add_argument("--retract-mm", type=float, default=1.0)
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.08)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.08)
    parser.add_argument("--service-timeout-sec", type=float, default=10.0)
    parser.add_argument("--pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    parser.add_argument(
        "--ack-safe-normal-probe",
        action="store_true",
        help="confirm the selected ruby ball is near the sphere and the short normal path/retract are clear",
    )
    parser.add_argument(
        "--refit-input",
        nargs="*",
        default=[str(PROJECT_DIR / "data/260603.1/yneg_new_5pts_for_fit.csv")],
        help="existing fit CSV files to combine with --output after a successful trigger",
    )
    parser.add_argument("--refit-json", default=str(DEFAULT_REFIT_JSON))
    parser.add_argument("--refit-residual-output", default=str(DEFAULT_REFIT_RESIDUALS))
    parser.add_argument("--no-refit", action="store_true", help="do not refit after a successful trigger")
    parser.add_argument(
        "--no-retract-on-miss",
        action="store_true",
        help="if no trigger occurs, hold at the reached target instead of retracting; triggers still retract",
    )
    return parser.parse_args()


def validate_args(args):
    """Validate args."""
    if args.search_mm <= 0:
        raise ValueError("--search-mm must be positive")
    if args.search_mm > 1.0 and not args.allow_search_over_1mm:
        raise ValueError("--search-mm > 1.0 requires --allow-search-over-1mm")
    if args.search_mm > 3.0:
        raise ValueError("--search-mm must be <= 3.0 for semi-auto normal probing")
    if args.retract_mm <= 0:
        raise ValueError("--retract-mm must be positive")
    if not 1 <= args.speed <= 5:
        raise ValueError("--speed must be between 1 and 5")
    if args.timeout_sec <= 0:
        raise ValueError("--timeout-sec must be positive")
    if args.execute and args.pose is not None:
        raise ValueError("--pose is for offline dry-run only; do not combine it with --execute")
    if args.execute and not args.ack_safe_normal_probe:
        raise ValueError("real motion requires --ack-safe-normal-probe after on-site path confirmation")
    if args.sphere_radius_mm <= 0:
        raise ValueError("--sphere-radius-mm must be positive")
    if args.probe_radius_mm is not None and args.probe_radius_mm <= 0:
        raise ValueError("--probe-radius-mm must be positive")


def main():
    """Main."""
    args = parse_args()
    node = None
    try:
        validate_args(args)
        fit, sphere_center, local_offset = load_fit(args.fit_json)
        fit_branch = fit.get("branch")
        if fit_branch and fit_branch != args.branch:
            raise ValueError(
                f"fit JSON branch {fit_branch!r} does not match --branch {args.branch!r}; "
                "use the matching initial fit for the selected physical ruby ball"
            )
        pose = args.pose
        if pose is None:
            node, pose = current_pose_from_robot(args)
        plan = compute_plan(pose, sphere_center, local_offset, args.euler_sequence, args.search_mm, args.approach)
        plan["initial_fit"] = fit
        print_plan(plan, args.search_mm, args.retract_mm)
        if not args.execute:
            print("dry-run only; add --execute and --ack-safe-normal-probe for real short probing motion")
            return
        row = execute_probe(node, args, plan)
        if row is not None and not args.no_refit:
            refit_after_contact(args)
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
