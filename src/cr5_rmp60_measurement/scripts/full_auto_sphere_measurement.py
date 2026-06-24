#!/usr/bin/env python3
"""End-to-end automatic standard-sphere probing workflow.

This is an orchestrator around the existing low-level tools:

1. Generate model-based candidate probing poses for each branch.
2. Load the calibration-sphere workcell into MoveIt.
3. Run MoveIt IK/cartesian preflight on the candidate set.
4. Select the requested number of checked rows per branch.
5. Optionally execute the selected plan with guarded short probing.

Real robot motion is disabled unless both --execute and
--ack-full-auto-sphere-measurement are supplied.
"""
import argparse
import copy
import csv
import json
import itertools
import math
import os
import signal
import shlex
import subprocess
import sys
import time
from pathlib import Path

from calibration_registry import (
    DEFAULT_REGISTRY,
    load_registry,
    parse_mapping,
    resolve_branch_styli,
    validate_registry,
)
from cross_probe_model import euler_to_matrix, mat_vec


PROJECT_DIR = Path(__file__).resolve().parents[1]
ROS_PREFIX = "source /opt/ros/humble/setup.bash; source ~/dobot_ws/install/setup.bash; export ROS_LOG_DIR=/tmp"
DEFAULT_REFERENCE_FIT = PROJECT_DIR / "data/2026.6.16/yneg_near_fourth_refit_20260616.json"
DEFAULT_OUTPUT_DIR = PROJECT_DIR / "data/2026.6.21/full_auto_sphere_measurement"
DEFAULT_MOVEIT_LAUNCH = PROJECT_DIR / "launch/cr5_rmp60_moveit_demo.launch.py"
DEFAULT_ANCHOR_CONTACTS = PROJECT_DIR / "data/2026.6.17/manual_teach_cross_12pts.csv"
DEFAULT_BRANCHES = ("y_neg",)
POSE_NAMES = ("x", "y", "z", "rx", "ry", "rz")
DEFAULT_BRANCH_FITS = {
    "y_neg": PROJECT_DIR / "data/2026.6.21/y_neg_fixed_center_distance_fit_20260621.json",
}


def shlex_join(parts):
    """Shlex join."""
    return shlex.join([str(part) for part in parts])


def ros_command(command):
    """Ros command."""
    return f"{ROS_PREFIX}; {command}"


def run_ros(command, *, check=True, capture=False, cwd=PROJECT_DIR):
    """Run ros."""
    printable = command if isinstance(command, str) else shlex_join(command)
    print(f"\n$ {printable}", flush=True)
    shell_command = printable if not isinstance(command, str) else command
    return subprocess.run(
        ["bash", "-lc", ros_command(shell_command)],
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
        check=check,
    )


def service_available(service_name):
    """Service available."""
    completed = subprocess.run(
        ["bash", "-lc", ros_command("ros2 service list")],
        cwd=PROJECT_DIR,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.returncode == 0 and service_name in completed.stdout.splitlines()


def wait_for_service(service_name, timeout_sec):
    """Wait for service."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if service_available(service_name):
            return True
        time.sleep(0.5)
    return False


def stop_process_group(proc):
    """Stop process group."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=6.0)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass


def start_or_reuse_moveit(args):
    """Start or reuse moveit."""
    if service_available("/compute_ik") and service_available("/apply_planning_scene"):
        print("MoveIt services already available; reusing existing move_group", flush=True)
        return None
    command = ros_command(f"ros2 launch {shlex.quote(str(args.moveit_launch))} use_rviz:=false")
    print(f"\n$ ros2 launch {args.moveit_launch} use_rviz:=false", flush=True)
    proc = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=PROJECT_DIR,
        stdout=subprocess.DEVNULL if args.quiet_moveit else None,
        stderr=subprocess.DEVNULL if args.quiet_moveit else None,
        text=True,
        preexec_fn=os.setsid,
    )
    if not wait_for_service("/compute_ik", args.moveit_start_timeout_sec):
        stop_process_group(proc)
        raise RuntimeError("MoveIt /compute_ik did not become available")
    if not wait_for_service("/apply_planning_scene", args.moveit_start_timeout_sec):
        stop_process_group(proc)
        raise RuntimeError("MoveIt /apply_planning_scene did not become available")
    return proc


def read_csv(path):
    """Read csv."""
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not rows:
        raise ValueError(f"no rows to write to {path}")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def row_pose(row, prefix):
    """Row pose."""
    return [float(row[f"{prefix}_{name}"]) for name in POSE_NAMES]


def position_distance_mm(a, b):
    """Position distance mm."""
    return math.sqrt(sum((float(a[index]) - float(b[index])) ** 2 for index in range(3)))


def angle_delta_deg(a, b):
    """Angle delta deg."""
    return (float(a) - float(b) + 180.0) % 360.0 - 180.0


def orientation_delta_deg(a, b):
    """Orientation delta deg."""
    return max(abs(angle_delta_deg(a[index], b[index])) for index in range(3, 6))


def row_approach(row):
    """Row approach."""
    vector = [float(row[f"approach_{axis}"]) for axis in ("x", "y", "z")]
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-12:
        raise ValueError("candidate approach vector cannot be zero")
    return [value / length for value in vector]


def normal_angle_deg(a, b):
    """Normal angle deg."""
    dot = max(-1.0, min(1.0, sum(float(x) * float(y) for x, y in zip(a, b))))
    return math.degrees(math.acos(dot))


def select_local_sequence(rows, anchor_pose, count, args, prior_normals=None):
    """Select local sequence."""
    remaining = list(rows)
    selected = []
    selected_normals = list(prior_normals or [])
    current = list(anchor_pose)
    while remaining and len(selected) < count:
        eligible = []
        step_limit = (
            args.max_initial_transition_mm
            if not selected and not selected_normals
            else args.max_local_step_mm
        )
        orientation_limit = (
            args.max_initial_orientation_step_deg
            if not selected and not selected_normals
            else args.max_local_orientation_step_deg
        )
        for row in remaining:
            transition = row_pose(row, "transition_flange")
            distance = position_distance_mm(current, transition)
            orientation = orientation_delta_deg(current, transition)
            if distance > step_limit or orientation > orientation_limit:
                continue
            approach = row_approach(row)
            if selected_normals and any(
                normal_angle_deg(approach, previous) < args.min_normal_separation_deg
                for previous in selected_normals
            ):
                continue
            cost = distance + args.orientation_cost_mm_per_deg * orientation
            eligible.append((cost, distance, orientation, row, approach, transition))
        if not eligible:
            break
        eligible.sort(key=lambda item: (item[0], item[1], item[2], item[3].get("plan_id", "")))
        _, _, _, row, approach, transition = eligible[0]
        selected.append(row)
        selected_normals.append(approach)
        current = transition
        remaining.remove(row)
    return selected


def parse_branch_fit(items):
    """Parse branch fit."""
    result = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError("--branch-fit entries must be BRANCH=JSON")
        branch, path = item.split("=", 1)
        branch = branch.strip()
        path = path.strip()
        if not branch or not path:
            raise ValueError("--branch-fit entries must be BRANCH=JSON")
        result[branch] = Path(path)
    return result


def parse_branch_pose(items):
    """Parse branch pose."""
    result = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError("--branch-anchor-pose entries must be BRANCH=X,Y,Z,RX,RY,RZ")
        branch, values = item.split("=", 1)
        pose = [float(value.strip()) for value in values.split(",") if value.strip()]
        if not branch.strip() or len(pose) != 6:
            raise ValueError("--branch-anchor-pose entries must be BRANCH=X,Y,Z,RX,RY,RZ")
        result[branch.strip()] = pose
    return result


def branch_fit_map(args):
    """Branch fit map."""
    result = {}
    for branch in args.branches:
        default = DEFAULT_BRANCH_FITS.get(branch)
        if default and default.exists():
            result[branch] = default
    result.update(parse_branch_fit(args.branch_fit))
    return result


def collision_branch_fit_map(args, active_fits):
    """Registry-checked fit in this run.."""
    # Do not silently load historical fits for inactive physical styli. Unknown
    # branches remain on the nominal geometry until they have an explicit,
    # registry-checked fit in this run.
    return dict(active_fits)


def enforce_seed_quality(args, fits):
    """Enforce seed quality."""
    failures = []
    for branch in args.branches:
        path = fits.get(branch)
        if path is None or not Path(path).exists():
            failures.append(f"{branch}: missing branch fit JSON")
            continue
        data = json.loads(Path(path).read_text())
        metric_name = None
        metric = None
        for name in ("rms_residual_mm", "rms_sample_spread_mm"):
            if data.get(name) is not None:
                metric_name = name
                metric = float(data[name])
                break
        if metric is None or not math.isfinite(metric):
            failures.append(f"{branch}: branch fit has no finite RMS quality metric")
            continue
        print(f"seed quality {branch}: {metric_name}={metric:.6f}mm", flush=True)
        if metric > args.max_seed_rms_mm:
            failures.append(
                f"{branch}: {metric_name} {metric:.6f}mm > {args.max_seed_rms_mm:.6f}mm"
            )
    if failures and not args.allow_poor_seed_quality:
        raise RuntimeError("branch seed quality gate failed: " + "; ".join(failures))
    if failures:
        print("seed quality warnings: " + "; ".join(failures), flush=True)


def branch_anchor_map(args, fits):
    """Branch anchor map."""
    result = parse_branch_pose(args.branch_anchor_pose)
    path = Path(args.anchor_contact_csv) if args.anchor_contact_csv else None
    rows = read_csv(path) if path and path.exists() else []
    reference = json.loads(Path(args.reference_fit_json).read_text())
    sphere_center = [float(value) for value in reference["sphere_center_mm"]]
    contact_distance = float(
        reference.get("contact_center_distance_mm", args.sphere_radius_mm + 1.0)
    )
    for branch in args.branches:
        if branch in result:
            continue
        branch_rows = [row for row in rows if row.get("branch") == branch]
        fit_path = fits.get(branch)
        fit_offset = None
        if fit_path and Path(fit_path).exists():
            data = json.loads(Path(fit_path).read_text())
            offset = data.get("local_ball_offset_mm")
            if isinstance(offset, list) and len(offset) == 3:
                fit_offset = [float(value) for value in offset]
        candidates = []
        for row in branch_rows:
            try:
                pose = [float(row[f"trigger_flange_{name}"]) for name in POSE_NAMES]
            except (KeyError, TypeError, ValueError):
                continue
            if fit_offset is not None:
                rotation = euler_to_matrix(args.euler_sequence, pose[3:6])
                rotated_offset = mat_vec(rotation, fit_offset)
                ball_center = [pose[index] + rotated_offset[index] for index in range(3)]
                radial = [sphere_center[index] - ball_center[index] for index in range(3)]
                radial_length = math.sqrt(sum(value * value for value in radial))
                if radial_length <= 1e-12:
                    continue
                approach = [value / radial_length for value in radial]
                offset_error = abs(radial_length - contact_distance)
            else:
                try:
                    approach = row_approach(row)
                except (KeyError, TypeError, ValueError):
                    continue
                offset_error = 0.0
            anchor = list(pose)
            for index in range(3):
                anchor[index] -= approach[index] * args.transition_standoff_mm
            local_distance = (
                position_distance_mm(anchor, args.safe_pose)
                if args.safe_pose is not None
                else 0.0
            )
            candidates.append((local_distance, offset_error, anchor))
        if candidates:
            candidates.sort(key=lambda item: (item[0], item[1]))
            result[branch] = candidates[0][2]
        elif args.safe_pose:
            result[branch] = list(args.safe_pose)
    return result


def optimized_branch_order(branches, anchors, safe_pose, orientation_cost):
    """Optimized branch order."""
    branches = list(branches)
    if len(branches) <= 1:
        return branches
    best = None
    for order in itertools.permutations(branches):
        poses = [safe_pose] + [anchors[branch] for branch in order] + [safe_pose]
        cost = 0.0
        for start, end in zip(poses, poses[1:]):
            cost += position_distance_mm(start, end)
            cost += orientation_cost * orientation_delta_deg(start, end)
        candidate = (cost, order)
        if best is None or candidate < best:
            best = candidate
    return list(best[1])


def selected_anchor_pose(args):
    """Selected anchor pose."""
    if args.anchor_pose:
        return list(args.anchor_pose)
    if args.safe_pose:
        return list(args.safe_pose)
    return None


def build_planner_command(args, branch, branch_fit, branch_plan, branch_all):
    """Build planner command."""
    physical_ball_id = getattr(args, "branch_stylus_map", {}).get(
        branch,
        args.physical_ball_id,
    )
    command = [
        "./scripts/model_based_sphere_point_planner.py",
        "--reference-fit-json",
        args.reference_fit_json,
        "--geometry",
        args.geometry,
        "--euler-sequence",
        args.euler_sequence,
        "--branch",
        branch,
        "--physical-ball-id",
        physical_ball_id,
        "--output",
        branch_plan,
        "--all-output",
        branch_all,
        "--plan-count",
        str(args.candidate_pool_per_branch),
        "--normal-mode",
        args.normal_mode,
        "--orientation-mode",
        args.planner_orientation_mode,
        "--cone-angle-deg",
        str(args.cone_angle_deg),
        "--cone-rings",
        str(args.cone_rings),
        "--cone-samples-per-ring",
        str(args.cone_samples_per_ring),
        "--candidate-count",
        str(args.candidate_count),
        "--roll-angles-deg",
        args.roll_angles_deg,
        "--standoff-mm",
        str(args.standoff_mm),
        "--transition-standoff-mm",
        str(args.transition_standoff_mm),
        "--overtravel-mm",
        str(args.overtravel_mm),
        "--min-approach-z",
        str(args.min_approach_z),
        "--max-approach-z",
        str(args.max_approach_z),
        "--min-probe-clearance-mm",
        str(args.min_probe_clearance_mm),
        "--table-plane-z-mm",
        str(args.table_plane_z_mm),
        "--min-table-clearance-mm",
        str(args.min_table_clearance_mm),
        "--target-stem-exclusion-mm",
        str(args.target_stem_exclusion_mm),
        "--collision-segment-samples",
        str(args.collision_segment_samples),
    ]
    if branch_fit:
        command.extend(["--branch-fit-json", branch_fit])
    collision_fits = getattr(args, "collision_branch_fits", {})
    if collision_fits:
        command.append("--collision-branch-fit")
        command.extend(f"{name}={path}" for name, path in sorted(collision_fits.items()))
    anchor = selected_anchor_pose(args)
    if anchor and args.normal_mode == "anchor-cone":
        command.append("--anchor-pose")
        command.extend(str(value) for value in anchor)
    if args.safe_pose:
        command.append("--safe-pose")
        command.extend(str(value) for value in args.safe_pose)
    if args.disable_table_plane_check:
        command.append("--disable-table-plane-check")
    if args.allow_low_clearance:
        command.append("--allow-low-clearance")
    if args.allow_collision:
        command.append("--allow-collision")
    if args.rod_collision_radius_mm is not None:
        command.extend(["--rod-collision-radius-mm", str(args.rod_collision_radius_mm)])
    return command


def generate_candidate_plans(args, fits, anchors=None):
    """Generate candidate plans."""
    candidate_rows = []
    for branch in args.branches:
        local = copy.copy(args)
        if anchors and branch in anchors:
            local.anchor_pose = list(anchors[branch])
        branch_plan = args.output_dir / f"{branch}_candidate_plan.csv"
        branch_all = args.output_dir / f"{branch}_all_candidates.csv"
        branch_fit = fits.get(branch)
        run_ros(build_planner_command(local, branch, branch_fit, branch_plan, branch_all))
        rows = read_csv(branch_plan)
        for row in rows:
            row["standard_sphere_id"] = args.active_standard_sphere_id
        candidate_rows.extend(rows)
    write_csv(args.candidate_plan, candidate_rows)
    print(f"saved combined candidate plan: {args.candidate_plan}", flush=True)


def apply_moveit_scene(args):
    """Apply moveit scene."""
    command = [
        "./scripts/apply_calibration_moveit_scene.py",
        "--reference-fit-json",
        args.reference_fit_json,
        "--sphere-radius-mm",
        str(args.sphere_radius_mm),
        "--stem-radius-mm",
        str(args.stem_radius_mm),
        "--table-z-mm",
        str(args.table_plane_z_mm),
        "--table-thickness-mm",
        str(args.table_thickness_mm),
        "--table-size-x-mm",
        str(args.table_size_x_mm),
        "--table-size-y-mm",
        str(args.table_size_y_mm),
        "--table-center-x-mm",
        str(args.table_center_x_mm),
        "--table-center-y-mm",
        str(args.table_center_y_mm),
        "--timeout-sec",
        str(args.service_timeout_sec),
    ]
    if args.magnetic_base_size_mm:
        command.append("--magnetic-base-size-mm")
        command.extend(str(value) for value in args.magnetic_base_size_mm)
        command.append("--magnetic-base-center-mm")
        command.extend(str(value) for value in args.magnetic_base_center_mm)
    run_ros(command)


def run_moveit_preflight(args):
    """Run moveit preflight."""
    command = [
        "./scripts/check_five_branch_plan_moveit.py",
        "--plan",
        args.candidate_plan,
        "--checked-output",
        args.checked_plan,
        "--service-timeout-sec",
        str(args.service_timeout_sec),
        "--ik-timeout",
        str(args.moveit_ik_timeout_sec),
        "--euler-sequence",
        args.euler_sequence,
        "--joint-state-topic",
        args.moveit_joint_state_topic,
        "--max-ik-joint-step-deg",
        str(args.max_ik_joint_step_deg),
        "--max-step-m",
        str(args.moveit_max_step_m),
        "--min-fraction",
        str(args.moveit_min_fraction),
    ]
    if args.check_probe_with_moveit_collision:
        command.append("--check-probe-with-moveit-collision")
    completed = run_ros(command, check=False)
    if completed.returncode != 0:
        print(
            "MoveIt preflight reported failures; selecting only rows with moveit_status=OK",
            flush=True,
        )


def preflight_return_to_safe(args, branch, current_pose):
    """Preflight return to safe."""
    if args.skip_moveit_preflight:
        return
    directory = args.output_dir / "adaptive" / branch / "return_to_safe"
    directory.mkdir(parents=True, exist_ok=True)
    plan_path = directory / "return_plan.csv"
    checked_path = directory / "moveit_checked_return_plan.csv"
    row = {
        "plan_id": f"{branch}_return_to_safe",
        "branch": branch,
        "preflight_mode": "transition-only",
    }
    for name, value in zip(POSE_NAMES, current_pose):
        row[f"safe_flange_{name}"] = f"{float(value):.6f}"
    for name, value in zip(POSE_NAMES, args.safe_pose):
        row[f"transition_flange_{name}"] = f"{float(value):.6f}"
    write_csv(plan_path, [row])
    local = copy.copy(args)
    local.candidate_plan = plan_path
    local.checked_plan = checked_path
    run_moveit_preflight(local)
    checked = read_csv(checked_path)
    if len(checked) != 1 or not moveit_ok(checked[0]):
        raise RuntimeError(f"{branch}: MoveIt rejected final transition back to safe pose")
    print(f"{branch}: MoveIt return-to-safe path OK", flush=True)


def locally_ok(row):
    """Locally ok."""
    return row.get("probe_collision_status") == "OK"


def moveit_ok(row):
    """Moveit ok."""
    return row.get("moveit_status") == "OK"


def select_checked_rows(args):
    """Select checked rows."""
    source = args.checked_plan if args.checked_plan.exists() else args.candidate_plan
    rows = read_csv(source)
    selected = []
    missing = []
    for branch in args.branches:
        branch_rows = [row for row in rows if row.get("branch") == branch]
        ok_rows = [row for row in branch_rows if locally_ok(row) and (args.skip_moveit_preflight or moveit_ok(row))]
        anchor = selected_anchor_pose(args)
        if anchor is None and ok_rows:
            anchor = row_pose(ok_rows[0], "transition_flange")
        take = select_local_sequence(ok_rows, anchor, args.samples_per_branch, args) if anchor else []
        selected.extend(take)
        if len(take) < args.samples_per_branch:
            missing.append((branch, len(take), len(branch_rows)))
    if missing:
        detail = ", ".join(f"{branch}: {ok}/{total} usable" for branch, ok, total in missing)
        raise RuntimeError(
            f"not enough checked plan rows for requested samples_per_branch={args.samples_per_branch}: {detail}"
        )
    write_csv(args.selected_plan, selected)
    print(f"saved selected executable plan: {args.selected_plan}", flush=True)
    for branch in args.branches:
        branch_rows = [row for row in selected if row.get("branch") == branch]
        ids = ", ".join(row.get("plan_id", "") for row in branch_rows)
        print(f"  {branch}: {len(branch_rows)} rows -> {ids}", flush=True)


def wait_dobot_services(args):
    """Wait dobot services."""
    for service_name in (
        "/dobot_bringup_ros2/srv/GetPose",
        "/dobot_bringup_ros2/srv/MovL",
        "/dobot_bringup_ros2/srv/MovJ",
        "/dobot_bringup_ros2/srv/DI",
        "/dobot_bringup_ros2/srv/Stop",
        "/dobot_bringup_ros2/srv/SpeedFactor",
    ):
        if not wait_for_service(service_name, args.service_timeout_sec):
            raise RuntimeError(f"Dobot service is not available: {service_name}")


def build_execute_command(
    args,
    execute,
    safe_pose_policy="plan",
    max_local_step_mm=None,
    max_local_orientation_step_deg=None,
):
    """Build execute command."""
    if max_local_step_mm is None:
        max_local_step_mm = args.max_initial_transition_mm
    if max_local_orientation_step_deg is None:
        max_local_orientation_step_deg = args.max_initial_orientation_step_deg
    command = [
        "./scripts/auto_calibrate_five_branch_sphere.py",
        "--reference-fit-json",
        args.reference_fit_json,
        "--euler-sequence",
        args.euler_sequence,
        "--plan-input",
        args.selected_plan,
        "--plan-output",
        args.execution_plan_copy,
        "--branches",
    ]
    command.extend(args.branches)
    command.extend(
        [
            "--physical-ball-id",
            args.physical_ball_id,
            "--session-id",
            args.session_id,
            "--operator-note",
            args.operator_note,
            "--output",
            args.contacts_output,
            "--calibration-output",
            args.measurement_fit_output,
            "--residual-output",
            args.residual_output,
            "--standoff-mm",
            str(args.standoff_mm),
            "--transition-standoff-mm",
            str(args.transition_standoff_mm),
            "--overtravel-mm",
            str(args.overtravel_mm),
            "--max-probe-travel-mm",
            str(args.max_probe_travel_mm),
            "--speed",
            str(args.speed),
            "--timeout-sec",
            str(args.timeout_sec),
            "--positioning-timeout-sec",
            str(args.positioning_timeout_sec),
            "--positioning-trigger-retract-mm",
            str(args.positioning_trigger_retract_mm),
            "--service-timeout-sec",
            str(args.service_timeout_sec),
            "--position-tolerance-mm",
            str(args.position_tolerance_mm),
            "--orientation-tolerance-deg",
            str(args.orientation_tolerance_deg),
            "--safe-transition-move",
            args.safe_transition_move,
            "--safe-pose-policy",
            safe_pose_policy,
            "--max-local-step-mm",
            str(max_local_step_mm),
            "--max-local-orientation-step-deg",
            str(max_local_orientation_step_deg),
            "--table-plane-z-mm",
            str(args.table_plane_z_mm),
            "--min-table-clearance-mm",
            str(args.min_table_clearance_mm),
            "--target-stem-exclusion-mm",
            str(args.target_stem_exclusion_mm),
            "--collision-segment-samples",
            str(args.collision_segment_samples),
            "--refit-after-each-hit",
        ]
    )
    if args.fit_input:
        command.append("--fit-input")
        command.extend(args.fit_input)
    if args.safe_pose:
        command.append("--safe-pose")
        command.extend(str(value) for value in args.safe_pose)
    if args.disable_table_plane_check:
        command.append("--disable-table-plane-check")
    if args.allow_probe_model_collision_risk:
        command.append("--allow-probe-model-collision-risk")
    if args.allow_large_safe_transition_orientation_change:
        command.append("--allow-large-safe-transition-orientation-change")
    if args.allow_probe_travel_over_1mm:
        command.append("--allow-probe-travel-over-1mm")
    if args.rod_collision_radius_mm is not None:
        command.extend(["--rod-collision-radius-mm", str(args.rod_collision_radius_mm)])
    if execute:
        command.extend(["--execute", "--ack-five-branch-calibration-path"])
    return command


def build_move_safe_command(args, branch, max_step_mm, max_orientation_step_deg):
    """Build move safe command."""
    command = [
        "./scripts/auto_calibrate_five_branch_sphere.py",
        "--reference-fit-json",
        args.reference_fit_json,
        "--branches",
        branch,
        "--safe-pose",
    ]
    command.extend(str(value) for value in args.safe_pose)
    command.extend(
        [
            "--speed",
            str(args.speed),
            "--positioning-timeout-sec",
            str(args.positioning_timeout_sec),
            "--service-timeout-sec",
            str(args.service_timeout_sec),
            "--position-tolerance-mm",
            str(args.position_tolerance_mm),
            "--orientation-tolerance-deg",
            str(args.orientation_tolerance_deg),
            "--max-local-step-mm",
            str(max_step_mm),
            "--max-local-orientation-step-deg",
            str(max_orientation_step_deg),
            "--execute",
            "--ack-five-branch-calibration-path",
            "--move-safe-only",
        ]
    )
    return command


def move_robot_to_safe(args, branch, max_step_mm=None, max_orientation_step_deg=None):
    """Move robot to safe."""
    if max_step_mm is None:
        max_step_mm = args.max_safe_entry_mm
    if max_orientation_step_deg is None:
        max_orientation_step_deg = args.max_safe_entry_orientation_deg
    print(f"\nmove to branch-boundary safe pose before/after {branch}", flush=True)
    run_ros(build_move_safe_command(args, branch, max_step_mm, max_orientation_step_deg))


def read_current_robot_pose(args):
    """Read current robot pose."""
    output = args.output_dir / "current_pose_before_execution.json"
    command = [
        "./scripts/auto_calibrate_five_branch_sphere.py",
        "--reference-fit-json",
        args.reference_fit_json,
        "--service-timeout-sec",
        str(args.service_timeout_sec),
        "--report-current-pose-json",
        output,
    ]
    run_ros(command)
    data = json.loads(output.read_text())
    pose = data.get("flange_pose_mm_deg")
    if not isinstance(pose, list) or len(pose) != 6:
        raise RuntimeError("live flange-pose report did not contain six values")
    pose = [float(value) for value in pose]
    if not all(math.isfinite(value) for value in pose):
        raise RuntimeError("live flange pose contains a non-finite value")
    return pose


def hit_count(path, branch):
    """Hit count."""
    if not Path(path).exists():
        return 0
    return sum(
        1
        for row in read_csv(path)
        if row.get("branch") == branch and row.get("status") == "HIT"
    )


def write_updated_branch_seed(measurement_fit_path, branch, output_path):
    """Write updated branch seed."""
    data = json.loads(Path(measurement_fit_path).read_text())
    result = next(
        (item for item in data.get("branches", []) if item.get("branch") == branch and item.get("ok")),
        None,
    )
    if result is None:
        raise RuntimeError(f"updated fit contains no valid result for {branch}")
    offset = result.get("estimated_offset_mm")
    if not isinstance(offset, list) or len(offset) != 3:
        raise RuntimeError(f"updated fit for {branch} has no 3-value estimated_offset_mm")
    seed = {
        "timestamp": time.time(),
        "method": "adaptive_fixed_sphere_center_branch_seed",
        "source_calibration_json": str(measurement_fit_path),
        "branch": branch,
        "physical_ball_id": result.get("physical_ball_id"),
        "sphere_center_mm": data.get("sphere_center_mm"),
        "contact_center_distance_mm": data.get("contact_center_distance_mm"),
        "local_ball_offset_mm": [float(value) for value in offset],
        "rows": result.get("rows"),
        "raw_rows": result.get("raw_rows"),
        "rejected_rows": result.get("rejected_rows"),
        "rms_residual_mm": result.get("rms_residual_mm"),
        "max_residual_mm": result.get("max_residual_mm"),
    }
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(seed, indent=2) + "\n")
    return output


def iteration_paths(args, branch, attempt):
    """Iteration paths."""
    directory = args.output_dir / "adaptive" / branch / f"attempt_{attempt:02d}"
    directory.mkdir(parents=True, exist_ok=True)
    return {
        "directory": directory,
        "candidate": directory / "candidate_plan.csv",
        "all_candidates": directory / "all_candidates.csv",
        "checked": directory / "moveit_checked_plan.csv",
        "selected": directory / "selected_plan.csv",
        "execution_copy": directory / "selected_plan_copy.csv",
        "branch_fit": args.output_dir / "adaptive" / branch / "branch_fit.json",
        "branch_residuals": args.output_dir / "adaptive" / branch / "residuals.csv",
        "current_seed": args.output_dir / "adaptive" / branch / "current_seed.json",
    }


def adaptive_candidate(args, branch, branch_fit, planning_anchor_pose, current_pose, used_normals, attempt):
    """Adaptive candidate."""
    paths = iteration_paths(args, branch, attempt)
    local = copy.copy(args)
    local.branches = [branch]
    local.anchor_pose = list(planning_anchor_pose)
    local.safe_pose = list(current_pose)
    local.max_initial_transition_mm = args.max_branch_transition_mm
    local.max_initial_orientation_step_deg = args.max_branch_orientation_step_deg
    local.candidate_plan = paths["candidate"]
    local.checked_plan = paths["checked"]
    run_ros(
        build_planner_command(
            local,
            branch,
            branch_fit,
            paths["candidate"],
            paths["all_candidates"],
        )
    )
    if args.skip_moveit_preflight:
        rows = read_csv(paths["candidate"])
    else:
        run_moveit_preflight(local)
        rows = read_csv(paths["checked"])
    ok_rows = [
        row
        for row in rows
        if locally_ok(row) and (args.skip_moveit_preflight or moveit_ok(row))
    ]
    selected = []
    if attempt == 1 and not used_normals:
        center_rows = [row for row in ok_rows if row.get("sample_in_branch") == "1"]
        selected = select_local_sequence(center_rows, current_pose, 1, local)
    if not selected:
        selected = select_local_sequence(ok_rows, current_pose, 1, local, prior_normals=used_normals)
    if not selected:
        raise RuntimeError(
            f"{branch} attempt {attempt}: no MoveIt-checked candidate satisfies local motion limits"
        )
    row = dict(selected[0])
    row["plan_id"] = f"{branch}_attempt_{attempt:02d}_{row.get('plan_id', '')}"
    row["standard_sphere_id"] = args.active_standard_sphere_id
    write_csv(paths["selected"], [row])
    return row, paths


def execute_adaptive_candidate(args, branch, row, paths, first_attempt):
    """Execute adaptive candidate."""
    local = copy.copy(args)
    local.branches = [branch]
    local.selected_plan = paths["selected"]
    local.execution_plan_copy = paths["execution_copy"]
    local.measurement_fit_output = paths["branch_fit"]
    local.residual_output = paths["branch_residuals"]
    before = len(read_csv(args.contacts_output)) if args.contacts_output.exists() else 0
    step_limit = args.max_branch_transition_mm if first_attempt else args.max_local_step_mm
    orientation_limit = (
        args.max_branch_orientation_step_deg
        if first_attempt
        else args.max_local_orientation_step_deg
    )
    run_ros(
        build_execute_command(
            local,
            execute=True,
            safe_pose_policy="none",
            max_local_step_mm=step_limit,
            max_local_orientation_step_deg=orientation_limit,
        )
    )
    after_rows = read_csv(args.contacts_output)
    if len(after_rows) != before + 1:
        raise RuntimeError(
            f"expected exactly one new contact row for {row['plan_id']}; got {len(after_rows) - before}"
        )
    result = after_rows[-1]
    if result.get("plan_id") != row.get("plan_id"):
        raise RuntimeError(
            f"contact row plan_id mismatch: expected {row.get('plan_id')}, got {result.get('plan_id')}"
        )
    return result


def run_adaptive_branch(args, branch, initial_fit, branch_anchor_pose, entry_pose):
    """Run adaptive branch."""
    print(f"\n=== adaptive branch calibration: {branch} ===", flush=True)
    current_pose = list(entry_pose)
    planning_anchor_pose = list(branch_anchor_pose)
    current_fit = Path(initial_fit) if initial_fit else None
    used_normals = []
    raw_hits = hit_count(args.contacts_output, branch) if args.resume else 0
    accepted_hits = raw_hits
    attempts = 0
    while accepted_hits < args.samples_per_branch and attempts < args.max_attempts_per_branch:
        attempts += 1
        row, paths = adaptive_candidate(
            args,
            branch,
            current_fit,
            planning_anchor_pose,
            current_pose,
            used_normals,
            attempts,
        )
        result = execute_adaptive_candidate(args, branch, row, paths, first_attempt=(attempts == 1))
        approach = row_approach(row)
        used_normals.append(approach)
        current_pose = row_pose(row, "transition_flange")
        planning_anchor_pose = list(current_pose)
        status = result.get("status", "")
        print(f"{branch} attempt {attempts}: {status}", flush=True)
        if status == "HIT":
            raw_hits += 1
            current_fit = write_updated_branch_seed(paths["branch_fit"], branch, paths["current_seed"])
            seed_data = json.loads(current_fit.read_text())
            accepted_hits = int(seed_data.get("rows") or raw_hits)
            offset = seed_data["local_ball_offset_mm"]
            print(
                f"  updated offset after raw={raw_hits}, inliers={accepted_hits}/"
                f"{args.samples_per_branch}: "
                f"[{offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}]",
                flush=True,
            )
    if accepted_hits < args.samples_per_branch:
        raise RuntimeError(
            f"{branch}: only {accepted_hits}/{args.samples_per_branch} accepted HIT rows after "
            f"{attempts} attempts"
        )
    return current_fit, current_pose


def run_adaptive_planning_dry_run(args, fits, anchors):
    """Run adaptive planning dry run."""
    branch_order = list(args.branches)
    if args.optimize_branch_order:
        branch_order = optimized_branch_order(
            args.branches,
            anchors,
            args.safe_pose,
            args.orientation_cost_mm_per_deg,
        )
    print(f"adaptive dry-run branch order: {' -> '.join(branch_order)}", flush=True)
    current_pose = list(args.safe_pose)
    selected_rows = []
    for branch in branch_order:
        planning_anchor = list(anchors[branch])
        used_normals = []
        for attempt in range(1, args.samples_per_branch + 1):
            row, _ = adaptive_candidate(
                args,
                branch,
                fits.get(branch),
                planning_anchor,
                current_pose,
                used_normals,
                attempt,
            )
            selected_rows.append(row)
            used_normals.append(row_approach(row))
            current_pose = row_pose(row, "transition_flange")
            planning_anchor = list(current_pose)
        current_pose = list(args.safe_pose)
    write_csv(args.selected_plan, selected_rows)
    print(f"saved adaptive dry-run plan: {args.selected_plan}", flush=True)
    run_ros(build_execute_command(args, execute=False))


def run_final_fit(args):
    """Run final fit."""
    command = [
        "./scripts/auto_calibrate_five_branch_sphere.py",
        "--reference-fit-json",
        args.reference_fit_json,
        "--branches",
    ]
    command.extend(args.branches)
    command.extend(
        [
            "--output",
            args.contacts_output,
            "--calibration-output",
            args.measurement_fit_output,
            "--residual-output",
            args.residual_output,
            "--fit-only",
            "--euler-sequence",
            args.euler_sequence,
        ]
    )
    if args.fit_input:
        command.append("--fit-input")
        command.extend(args.fit_input)
    run_ros(command)


def enforce_quality_gate(args):
    """Enforce quality gate."""
    data = json.loads(args.measurement_fit_output.read_text())
    failures = []
    for branch in args.branches:
        result = next((item for item in data.get("branches", []) if item.get("branch") == branch), None)
        if not result or not result.get("ok"):
            failures.append(f"{branch}: no valid fit")
            continue
        rows = int(result.get("rows", 0))
        rms = float(result.get("rms_residual_mm", math.inf))
        maximum = float(result.get("max_residual_mm", math.inf))
        print(
            f"quality {branch}: rows={rows} rms={rms:.6f}mm max={maximum:.6f}mm",
            flush=True,
        )
        if rows < args.samples_per_branch:
            failures.append(f"{branch}: only {rows} inlier rows")
        if rms > args.max_rms_residual_mm:
            failures.append(f"{branch}: RMS {rms:.6f}mm > {args.max_rms_residual_mm:.6f}mm")
        if maximum > args.max_residual_mm:
            failures.append(f"{branch}: max {maximum:.6f}mm > {args.max_residual_mm:.6f}mm")
    if failures and not args.allow_quality_gate_failure:
        raise RuntimeError("calibration quality gate failed: " + "; ".join(failures))
    if failures:
        print("quality gate warnings: " + "; ".join(failures), flush=True)


def validate_args(args):
    """Validate args."""
    if args.samples_per_branch <= 0:
        raise ValueError("--samples-per-branch must be positive")
    if args.candidate_pool_per_branch < args.samples_per_branch:
        raise ValueError("--candidate-pool-per-branch must be >= --samples-per-branch")
    if args.max_initial_transition_mm <= 0 or args.max_local_step_mm <= 0:
        raise ValueError("local translation limits must be positive")
    if args.max_branch_transition_mm <= 0:
        raise ValueError("--max-branch-transition-mm must be positive")
    if args.max_safe_entry_mm <= 0 or args.max_safe_entry_orientation_deg <= 0:
        raise ValueError("safe entry limits must be positive")
    if args.max_local_orientation_step_deg <= 0:
        raise ValueError("--max-local-orientation-step-deg must be positive")
    if args.max_initial_orientation_step_deg <= 0 or args.max_branch_orientation_step_deg <= 0:
        raise ValueError("initial/branch orientation limits must be positive")
    if args.min_normal_separation_deg < 0:
        raise ValueError("--min-normal-separation-deg cannot be negative")
    if args.max_attempts_per_branch < args.samples_per_branch:
        raise ValueError("--max-attempts-per-branch must be >= --samples-per-branch")
    if args.max_rms_residual_mm <= 0 or args.max_residual_mm <= 0:
        raise ValueError("quality gate residual limits must be positive")
    if args.max_seed_rms_mm <= 0:
        raise ValueError("--max-seed-rms-mm must be positive")
    if args.max_ik_joint_step_deg <= 0:
        raise ValueError("--max-ik-joint-step-deg must be positive")
    if args.positioning_trigger_retract_mm <= 0:
        raise ValueError("--positioning-trigger-retract-mm must be positive")
    if args.execute and not args.ack_full_auto_sphere_measurement:
        raise ValueError("real robot motion requires --ack-full-auto-sphere-measurement")
    if args.execute and not args.safe_pose:
        raise ValueError("real robot motion requires --safe-pose")
    if args.execute and args.skip_moveit_preflight and not args.allow_execute_without_moveit:
        raise ValueError("real robot motion requires MoveIt preflight unless --allow-execute-without-moveit is supplied")
    if args.execute and args.adaptive_replan and args.safe_transition_move != "movl":
        raise ValueError("adaptive execution requires --safe-transition-move movl so real motion matches MoveIt preflight")
    if args.planner_orientation_mode == "anchor-fixed" and not selected_anchor_pose(args):
        raise ValueError("small-motion anchor-fixed planning requires --anchor-pose or --safe-pose")
    if args.normal_mode == "anchor-cone" and not selected_anchor_pose(args):
        raise ValueError("--normal-mode anchor-cone requires --anchor-pose or --safe-pose")
    if args.max_probe_travel_mm > 1.0 and not args.allow_probe_travel_over_1mm:
        raise ValueError("--max-probe-travel-mm > 1 requires --allow-probe-travel-over-1mm")
    if args.speed < 1 or args.speed > 5:
        raise ValueError("--speed must be in [1, 5]")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.candidate_plan = args.output_dir / "candidate_plan.csv"
    args.checked_plan = args.output_dir / "moveit_checked_plan.csv"
    args.selected_plan = args.output_dir / "selected_execution_plan.csv"
    args.execution_plan_copy = args.output_dir / "selected_execution_plan_copy.csv"
    args.contacts_output = args.output_dir / "contacts.csv"
    args.measurement_fit_output = args.output_dir / "measurement_fit.json"
    args.residual_output = args.output_dir / "residuals.csv"
    if args.execute and args.adaptive_replan and args.contacts_output.exists() and not args.resume:
        raise ValueError(
            f"{args.contacts_output} already exists; choose a new --output-dir or pass --resume"
        )


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration-registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--reference-fit-json", default=str(DEFAULT_REFERENCE_FIT))
    parser.add_argument("--geometry", default=str(PROJECT_DIR / "config/cross_probe_geometry.yaml"))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--branches", nargs="+", default=list(DEFAULT_BRANCHES))
    parser.add_argument("--branch-fit", nargs="*", default=[], metavar="BRANCH=JSON")
    parser.add_argument(
        "--branch-stylus",
        nargs="*",
        default=[],
        metavar="BRANCH=PHYSICAL_STYLUS_ID",
        help="explicit physical stylus identity; selected fit must match the registry",
    )
    parser.add_argument("--anchor-contact-csv", default=str(DEFAULT_ANCHOR_CONTACTS))
    parser.add_argument(
        "--branch-anchor-pose",
        nargs="*",
        default=[],
        metavar="BRANCH=X,Y,Z,RX,RY,RZ",
    )
    parser.add_argument("--samples-per-branch", type=int, default=5)
    parser.add_argument("--candidate-pool-per-branch", type=int, default=32)
    parser.add_argument("--physical-ball-id", default="cross_probe")
    parser.add_argument("--session-id", default="session_20260621_full_auto_sphere_measurement")
    parser.add_argument("--operator-note", default="full auto standard sphere measurement")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--fit-input", nargs="*", default=[])
    parser.add_argument("--safe-pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--anchor-pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--normal-mode", choices=("anchor-cone", "fibonacci"), default="anchor-cone")
    parser.add_argument(
        "--planner-orientation-mode",
        choices=("anchor-fixed", "align-branch"),
        default="anchor-fixed",
    )
    parser.add_argument("--cone-angle-deg", type=float, default=3.0)
    parser.add_argument("--cone-rings", type=int, default=3)
    parser.add_argument("--cone-samples-per-ring", type=int, default=8)
    parser.add_argument("--candidate-count", type=int, default=240)
    parser.add_argument("--roll-angles-deg", default="0")
    parser.add_argument("--max-initial-transition-mm", type=float, default=15.0)
    parser.add_argument("--max-branch-transition-mm", type=float, default=110.0)
    parser.add_argument("--max-local-step-mm", type=float, default=10.0)
    parser.add_argument("--max-local-orientation-step-deg", type=float, default=4.0)
    parser.add_argument("--max-initial-orientation-step-deg", type=float, default=8.0)
    parser.add_argument("--max-branch-orientation-step-deg", type=float, default=10.0)
    parser.add_argument("--max-safe-entry-mm", type=float, default=20.0)
    parser.add_argument("--max-safe-entry-orientation-deg", type=float, default=8.0)
    parser.add_argument("--min-normal-separation-deg", type=float, default=0.8)
    parser.add_argument("--orientation-cost-mm-per-deg", type=float, default=2.0)
    parser.add_argument(
        "--optimize-branch-order",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--adaptive-replan",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="replan and MoveIt-check after every probing attempt",
    )
    parser.add_argument("--max-attempts-per-branch", type=int, default=8)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--max-rms-residual-mm", type=float, default=0.5)
    parser.add_argument("--max-residual-mm", type=float, default=1.0)
    parser.add_argument("--allow-quality-gate-failure", action="store_true")
    parser.add_argument("--max-seed-rms-mm", type=float, default=1.0)
    parser.add_argument("--allow-poor-seed-quality", action="store_true")
    parser.add_argument("--min-approach-z", type=float, default=-1.0)
    parser.add_argument("--max-approach-z", type=float, default=0.35)
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--stem-radius-mm", type=float, default=4.0)
    parser.add_argument("--standoff-mm", type=float, default=0.3)
    parser.add_argument("--transition-standoff-mm", type=float, default=3.0)
    parser.add_argument("--overtravel-mm", type=float, default=0.2)
    parser.add_argument("--max-probe-travel-mm", type=float, default=1.0)
    parser.add_argument("--allow-probe-travel-over-1mm", action="store_true")
    parser.add_argument("--min-probe-clearance-mm", type=float, default=0.5)
    parser.add_argument("--allow-low-clearance", action="store_true")
    parser.add_argument("--allow-collision", action="store_true")
    parser.add_argument("--allow-probe-model-collision-risk", action="store_true")
    parser.add_argument("--disable-table-plane-check", action="store_true")
    parser.add_argument("--table-plane-z-mm", type=float, default=0.0)
    parser.add_argument("--min-table-clearance-mm", type=float, default=5.0)
    parser.add_argument("--table-thickness-mm", type=float, default=20.0)
    parser.add_argument("--table-size-x-mm", type=float, default=1200.0)
    parser.add_argument("--table-size-y-mm", type=float, default=900.0)
    parser.add_argument("--table-center-x-mm", type=float, default=-400.0)
    parser.add_argument("--table-center-y-mm", type=float, default=120.0)
    parser.add_argument(
        "--magnetic-base-size-mm",
        nargs=3,
        type=float,
        default=[60.0, 60.0, 70.0],
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument("--magnetic-base-center-mm", nargs=3, type=float, default=[-401.9, 126.5, 35.0])
    parser.add_argument("--rod-collision-radius-mm", type=float)
    parser.add_argument("--target-stem-exclusion-mm", type=float, default=3.0)
    parser.add_argument("--collision-segment-samples", type=int, default=9)
    parser.add_argument("--moveit-launch", default=str(DEFAULT_MOVEIT_LAUNCH))
    parser.add_argument("--moveit-start-timeout-sec", type=float, default=25.0)
    parser.add_argument("--moveit-ik-timeout-sec", type=float, default=1.0)
    parser.add_argument("--moveit-joint-state-topic", default="/joint_states_robot")
    parser.add_argument("--max-ik-joint-step-deg", type=float, default=15.0)
    parser.add_argument("--moveit-max-step-m", type=float, default=0.001)
    parser.add_argument("--moveit-min-fraction", type=float, default=1.0)
    parser.add_argument("--check-probe-with-moveit-collision", action="store_true")
    parser.add_argument("--skip-moveit-preflight", action="store_true")
    parser.add_argument("--allow-execute-without-moveit", action="store_true")
    parser.add_argument("--quiet-moveit", action="store_true")
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--positioning-timeout-sec", type=float, default=20.0)
    parser.add_argument("--positioning-trigger-retract-mm", type=float, default=5.0)
    parser.add_argument("--service-timeout-sec", type=float, default=10.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.08)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.10)
    parser.add_argument("--safe-transition-move", choices=("movl", "movj"), default="movl")
    parser.add_argument("--allow-large-safe-transition-orientation-change", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--ack-full-auto-sphere-measurement", action="store_true")
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    moveit_proc = None
    try:
        validate_args(args)
        fits = branch_fit_map(args)
        registry = validate_registry(load_registry(args.calibration_registry))
        args.active_standard_sphere_id = registry["active_standard_sphere_id"]
        explicit_styli = parse_mapping(args.branch_stylus, "--branch-stylus")
        args.branch_stylus_map = resolve_branch_styli(
            registry,
            args.branches,
            fits,
            explicit_styli,
            args.reference_fit_json,
            require_auto_ready=args.execute,
        )
        args.collision_branch_fits = collision_branch_fit_map(args, fits)
        if args.execute:
            enforce_seed_quality(args, fits)
        anchors = branch_anchor_map(args, fits)
        print("branch model inputs:", flush=True)
        for branch in args.branches:
            print(
                f"  {branch}: stylus={args.branch_stylus_map[branch]} "
                f"fit={fits.get(branch, 'nominal_geometry')}",
                flush=True,
            )
        print("branch local anchors:", flush=True)
        for branch in args.branches:
            print(f"  {branch}: {anchors.get(branch, 'missing')}", flush=True)
        if args.execute and args.adaptive_replan:
            missing_anchors = [branch for branch in args.branches if branch not in anchors]
            if missing_anchors:
                raise ValueError(
                    "adaptive execution requires a local anchor for every branch: "
                    + ", ".join(missing_anchors)
                )

        if not args.execute and args.adaptive_replan:
            if not args.skip_moveit_preflight:
                moveit_proc = start_or_reuse_moveit(args)
                apply_moveit_scene(args)
            run_adaptive_planning_dry_run(args, fits, anchors)
        elif not args.execute or not args.adaptive_replan:
            generate_candidate_plans(args, fits, anchors)

            if args.skip_moveit_preflight:
                print("MoveIt preflight skipped; using local collision-filtered candidate rows", flush=True)
            else:
                moveit_proc = start_or_reuse_moveit(args)
                apply_moveit_scene(args)
                run_moveit_preflight(args)

            select_checked_rows(args)
            run_ros(build_execute_command(args, execute=False))

        if not args.execute:
            print(
                "\nfull automatic sphere measurement dry-run complete; "
                "add --execute --ack-full-auto-sphere-measurement for real guarded probing",
                flush=True,
            )
            return 0

        wait_dobot_services(args)
        if args.adaptive_replan:
            if not args.skip_moveit_preflight:
                moveit_proc = start_or_reuse_moveit(args)
                apply_moveit_scene(args)
            branch_order = list(args.branches)
            if args.optimize_branch_order:
                branch_order = optimized_branch_order(
                    args.branches,
                    anchors,
                    args.safe_pose,
                    args.orientation_cost_mm_per_deg,
                )
            print(f"optimized branch order: {' -> '.join(branch_order)}", flush=True)
            current_pose = read_current_robot_pose(args)
            preflight_return_to_safe(args, "initial_entry", current_pose)
            move_robot_to_safe(
                args,
                branch_order[0],
                args.max_safe_entry_mm,
                args.max_safe_entry_orientation_deg,
            )
            for branch in branch_order:
                initial_fit = fits.get(branch)
                resume_seed = args.output_dir / "adaptive" / branch / "current_seed.json"
                if args.resume and resume_seed.exists():
                    initial_fit = resume_seed
                _, final_transition_pose = run_adaptive_branch(
                    args,
                    branch,
                    initial_fit,
                    anchors[branch],
                    args.safe_pose,
                )
                preflight_return_to_safe(args, branch, final_transition_pose)
                move_robot_to_safe(
                    args,
                    branch,
                    args.max_branch_transition_mm,
                    args.max_branch_orientation_step_deg,
                )
            run_final_fit(args)
            enforce_quality_gate(args)
        else:
            run_ros(build_execute_command(args, execute=True))
            enforce_quality_gate(args)
        print(f"\ncompleted automatic calibration; contacts: {args.contacts_output}", flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        stop_process_group(moveit_proc)


if __name__ == "__main__":
    raise SystemExit(main())
