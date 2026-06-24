#!/usr/bin/env python3
"""Plan and optionally execute independent validation probes on a sphere.

This script uses an existing single-branch absolute sphere fit as fixed
geometry. It generates contact poses around the calibration sphere, moves the
robot to a short outside standoff, then probes along the planned normal. New
contacts are recorded as validation observations only; the fit is not updated.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy

from cross_probe_model import (
    DEFAULT_GEOMETRY,
    branch_map,
    euler_to_matrix,
    load_geometry,
    mat_vec,
    nominal_branch_origin_mm,
)
from jog_and_record_contacts import (
    current_pose,
    format_pose,
    fresh_feed_di1,
    issue_movl,
    pose_fields,
    vector_fields,
    wait_jog,
)
from probe_touch import ProbeTouch


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_FIT = PROJECT_DIR / "data/yneg_gui_normal_absolute_fit_20260610.json"
DEFAULT_PLAN = PROJECT_DIR / "data/yneg_sphere_validation_plan_20260612.csv"
DEFAULT_OUTPUT = PROJECT_DIR / "data/yneg_sphere_validation_contacts_20260612.csv"
POSE_NAMES = ("x", "y", "z", "rx", "ry", "rz")


def parse_float(value, label):
    """Parse a string to float, returning a default for empty/missing values."""
    if value in (None, ""):
        raise ValueError(f"missing {label}")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def norm(vector):
    """Norm."""
    return float(np.linalg.norm(np.asarray(vector, dtype=float)))


def normalize(vector, label):
    """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
    length = norm(vector)
    if length <= 1e-12:
        raise ValueError(f"{label} cannot be zero")
    return [float(value) / length for value in vector]


def add(a, b):
    """Return the element-wise sum of two equal-length vectors."""
    return [float(a[index]) + float(b[index]) for index in range(3)]


def sub(a, b):
    """Sub."""
    return [float(a[index]) - float(b[index]) for index in range(3)]


def scale(vector, value):
    """Return a vector scaled by a scalar value (element-wise multiplication)."""
    return [float(component) * float(value) for component in vector]


def load_fit(path):
    """Load fit."""
    data = json.loads(Path(path).read_text())
    for field in ("sphere_center_mm", "local_ball_offset_mm"):
        if field not in data:
            raise ValueError(f"fit JSON missing {field}")
        if not isinstance(data[field], list) or len(data[field]) != 3:
            raise ValueError(f"fit JSON {field} must contain 3 values")
    return data


def residual_orientations(fit):
    """Residual orientations."""
    orientations = []
    for row in fit.get("residual_rows", []):
        try:
            orientations.append(
                [
                    parse_float(row.get("rx"), "residual row rx"),
                    parse_float(row.get("ry"), "residual row ry"),
                    parse_float(row.get("rz"), "residual row rz"),
                ]
            )
        except ValueError:
            continue
    return orientations


def parse_direction_arg(values):
    """Parse direction arg."""
    if len(values) % 3 != 0:
        raise ValueError("--direction values must be provided in groups of 3")
    directions = []
    for index in range(0, len(values), 3):
        directions.append(normalize(values[index : index + 3], f"--direction group {index // 3 + 1}"))
    return directions


def pose_with_position(position, orientation):
    """Pose with position."""
    return [position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]]


def pose_distance(a, b):
    """Pose distance."""
    return norm([float(a[index]) - float(b[index]) for index in range(3)])


def pose_dict(prefix, pose):
    """Pose dict."""
    return {f"{prefix}_{POSE_NAMES[index]}": f"{float(pose[index]):.4f}" for index in range(6)}


def angle_delta_deg(actual, expected):
    """Angle delta deg."""
    return (float(actual) - float(expected) + 180.0) % 360.0 - 180.0


def max_orientation_delta_deg(a, b):
    """Max orientation delta deg."""
    return max(abs(angle_delta_deg(a[index], b[index])) for index in range(3, 6))


def transform_point(pose, rotation, local_point):
    """Transform point."""
    return add(pose[:3], mat_vec(rotation, local_point))


def point_segment_distance(point, start, end):
    """Point segment distance."""
    point = np.asarray(point, dtype=float)
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    segment = end - start
    denom = float(np.dot(segment, segment))
    if denom <= 1e-12:
        return float(np.linalg.norm(point - start))
    t = float(np.dot(point - start, segment) / denom)
    t = max(0.0, min(1.0, t))
    closest = start + t * segment
    return float(np.linalg.norm(point - closest))


def branch_direction(geometry, branch_name):
    """Branch direction."""
    branches = branch_map(geometry)
    if branch_name not in branches:
        raise ValueError(f"unknown branch {branch_name!r} in geometry")
    return normalize(branches[branch_name]["direction"], f"{branch_name} direction")


def probe_dimensions(geometry, args):
    """Probe dimensions."""
    adapter = geometry["flange_adapter"]
    body = geometry["probe_body"]
    stylus = geometry["stylus"]
    configured_rod_radius = float(stylus["rod_radius_mm"])
    ball_radius = float(stylus.get("ball_radius_mm", 0.5 * float(stylus["ball_diameter_mm"])))
    if args.rod_collision_radius_mm is not None:
        rod_collision_radius = float(args.rod_collision_radius_mm)
    elif configured_rod_radius >= ball_radius:
        rod_collision_radius = 0.5 * configured_rod_radius
    else:
        rod_collision_radius = configured_rod_radius
    return {
        "adapter_length_mm": float(adapter["effective_axial_length_mm"]),
        "adapter_radius_mm": float(adapter.get("radius_mm", 30.0)),
        "probe_body_length_mm": float(body["length_mm"]),
        "probe_body_radius_mm": 0.5 * float(body["diameter_mm"]),
        "branch_center_distance_mm": float(stylus["branch_center_distance_mm"]),
        "ball_radius_mm": ball_radius,
        "configured_rod_radius_mm": configured_rod_radius,
        "rod_collision_radius_mm": rod_collision_radius,
    }


def local_collision_primitives(geometry, fit, args):
    """Local collision primitives."""
    dims = probe_dimensions(geometry, args)
    nominal_origin = nominal_branch_origin_mm(geometry)
    local_offset = [float(v) for v in fit["local_ball_offset_mm"]]
    direction = branch_direction(geometry, args.branch)
    calibrated_origin = sub(local_offset, scale(direction, dims["branch_center_distance_mm"]))

    primitives = [
        {
            "name": "adapter_body_nominal",
            "type": "capsule",
            "a": [0.0, 0.0, 0.0],
            "b": [0.0, 0.0, dims["adapter_length_mm"]],
            "radius": dims["adapter_radius_mm"],
            "model": "nominal",
        },
        {
            "name": "rmp60_body_nominal",
            "type": "capsule",
            "a": [0.0, 0.0, dims["adapter_length_mm"]],
            "b": [0.0, 0.0, dims["adapter_length_mm"] + dims["probe_body_length_mm"]],
            "radius": dims["probe_body_radius_mm"],
            "model": "nominal",
        },
    ]

    branch_dirs = {
        name: normalize(branch["direction"], f"{name} direction")
        for name, branch in branch_map(geometry).items()
    }
    origins = [
        ("nominal", nominal_origin),
        ("calibrated_target_origin", calibrated_origin),
    ]
    for model_name, origin in origins:
        distance = dims["branch_center_distance_mm"]
        rod_radius = dims["rod_collision_radius_mm"]
        ball_radius = dims["ball_radius_mm"]
        for branch_name, branch_dir in branch_dirs.items():
            rod_length = distance
            if branch_name == args.branch:
                rod_length = max(0.0, distance - args.target_stem_exclusion_mm)
            primitives.append(
                {
                    "name": f"{model_name}_{branch_name}_stem",
                    "type": "capsule",
                    "a": origin,
                    "b": add(origin, scale(branch_dir, rod_length)),
                    "radius": rod_radius,
                    "model": model_name,
                }
            )
            if model_name == "calibrated_target_origin" and branch_name == args.branch:
                continue
            center = add(origin, scale(branch_dir, distance))
            primitives.append(
                {
                    "name": f"{model_name}_{branch_name}_ball",
                    "type": "sphere",
                    "center": center,
                    "radius": ball_radius,
                    "model": model_name,
                }
            )
    return primitives, dims, calibrated_origin


def primitive_clearance_to_sphere(primitive, pose, sphere_center, sphere_radius, euler_sequence):
    """Primitive clearance to sphere."""
    rotation = euler_to_matrix(euler_sequence, pose[3:6])
    if primitive["type"] == "sphere":
        center = transform_point(pose, rotation, primitive["center"])
        distance = norm(sub(center, sphere_center))
    elif primitive["type"] == "capsule":
        start = transform_point(pose, rotation, primitive["a"])
        end = transform_point(pose, rotation, primitive["b"])
        distance = point_segment_distance(sphere_center, start, end)
    else:
        raise ValueError(f"unknown primitive type {primitive['type']!r}")
    return distance - sphere_radius - float(primitive["radius"])


def probe_collision_summary(pose, primitives, sphere_center, sphere_radius, args):
    """Probe collision summary."""
    clearances = [
        (primitive_clearance_to_sphere(primitive, pose, sphere_center, sphere_radius, args.euler_sequence), primitive)
        for primitive in primitives
    ]
    clearance, primitive = min(clearances, key=lambda item: item[0])
    if clearance < 0.0:
        status = "COLLISION"
    elif clearance < args.min_probe_clearance_mm:
        status = "LOW_CLEARANCE"
    else:
        status = "OK"
    return {
        "status": status,
        "clearance": clearance,
        "primitive": primitive["name"],
        "primitive_model": primitive.get("model", ""),
        "obstacle": "calibration_sphere",
    }


def collision_severity(summary):
    """Collision severity."""
    return {"OK": 0, "LOW_CLEARANCE": 1, "COLLISION": 2}[summary["status"]]


def worst_collision_summary(summaries):
    """Worst collision summary."""
    return max(summaries, key=lambda item: (collision_severity(item), -item["clearance"]))


def primitive_clearance_to_table_plane(primitive, pose, table_z_mm, euler_sequence):
    """Primitive clearance to table plane."""
    rotation = euler_to_matrix(euler_sequence, pose[3:6])
    if primitive["type"] == "sphere":
        center = transform_point(pose, rotation, primitive["center"])
        min_z = center[2] - float(primitive["radius"])
    elif primitive["type"] == "capsule":
        start = transform_point(pose, rotation, primitive["a"])
        end = transform_point(pose, rotation, primitive["b"])
        min_z = min(start[2], end[2]) - float(primitive["radius"])
    else:
        raise ValueError(f"unknown primitive type {primitive['type']!r}")
    return min_z - float(table_z_mm)


def table_plane_collision_summary(pose, primitives, args):
    """Table plane collision summary."""
    clearances = [
        (primitive_clearance_to_table_plane(primitive, pose, args.table_plane_z_mm, args.euler_sequence), primitive)
        for primitive in primitives
    ]
    clearance, primitive = min(clearances, key=lambda item: item[0])
    if clearance < 0.0:
        status = "COLLISION"
    elif clearance < args.min_table_clearance_mm:
        status = "LOW_CLEARANCE"
    else:
        status = "OK"
    return {
        "status": status,
        "clearance": clearance,
        "primitive": f"table_plane:{primitive['name']}",
        "primitive_model": primitive.get("model", ""),
        "obstacle": "table_plane",
    }


def combined_collision_summary(pose, primitives, sphere_center, sphere_radius, args):
    """Combined collision summary."""
    summaries = [probe_collision_summary(pose, primitives, sphere_center, sphere_radius, args)]
    if not args.disable_table_plane_check:
        summaries.append(table_plane_collision_summary(pose, primitives, args))
    return worst_collision_summary(summaries)


def sample_pose_segment_clearance(start_pose, end_pose, primitives, sphere_center, sphere_radius, args):
    """Sample pose segment clearance."""
    samples = max(2, int(args.collision_segment_samples))
    worst = None
    for index in range(samples):
        t = index / float(samples - 1)
        pose = [
            float(start_pose[axis]) + (float(end_pose[axis]) - float(start_pose[axis])) * t
            for axis in range(6)
        ]
        summary = combined_collision_summary(pose, primitives, sphere_center, sphere_radius, args)
        if worst is None or collision_severity(summary) > collision_severity(worst) or (
            summary["status"] == worst["status"] and summary["clearance"] < worst["clearance"]
        ):
            worst = summary
    return worst


def make_plan(args, fit, current_orientation=None):
    """Make plan."""
    branch = fit.get("branch", args.branch)
    if branch and branch != args.branch:
        raise ValueError(f"fit branch {branch!r} does not match --branch {args.branch!r}")
    sphere_center = [float(v) for v in fit["sphere_center_mm"]]
    local_offset = [float(v) for v in fit["local_ball_offset_mm"]]
    geometry = load_geometry(args.geometry)
    primitives, dims, calibrated_origin = local_collision_primitives(geometry, fit, args)
    contact_distance = float(fit.get("contact_center_distance_mm", args.sphere_radius_mm + args.probe_radius_mm))
    if contact_distance <= 0:
        raise ValueError("contact center distance must be positive")

    local_branch_direction = branch_direction(geometry, args.branch)
    explicit_directions = parse_direction_arg(args.direction) if args.direction else None
    if args.orientation:
        orientations = [list(args.orientation[3:6])]
    elif args.orientation_mode == "current":
        if current_orientation is None:
            raise ValueError("--orientation-mode current requires a live robot pose")
        orientations = [list(current_orientation)]
    else:
        orientations = residual_orientations(fit)
        if not orientations:
            raise ValueError("fit JSON has no residual row orientations; provide --orientation X Y Z RX RY RZ")

    rows = []
    previous_start = None
    point_count = len(explicit_directions) if explicit_directions is not None else args.point_count
    for index in range(1, point_count + 1):
        orientation = orientations[(index - 1) % len(orientations)]
        rotation = euler_to_matrix(args.euler_sequence, orientation)
        if explicit_directions is not None:
            direction = explicit_directions[(index - 1) % len(explicit_directions)]
        else:
            direction = normalize(
                mat_vec(rotation, local_branch_direction),
                f"validation point {index} branch-axis approach",
            )
        rotated_offset = mat_vec(rotation, local_offset)
        contact_ball_center = sub(sphere_center, scale(direction, contact_distance))
        transition_ball_center = sub(contact_ball_center, scale(direction, args.transition_standoff_mm))
        start_ball_center = sub(contact_ball_center, scale(direction, args.standoff_mm))
        target_ball_center = add(contact_ball_center, scale(direction, args.overtravel_mm))
        transition_flange_position = sub(transition_ball_center, rotated_offset)
        contact_flange_position = sub(contact_ball_center, rotated_offset)
        start_flange_position = sub(start_ball_center, rotated_offset)
        target_flange_position = sub(target_ball_center, rotated_offset)
        transition_pose = pose_with_position(transition_flange_position, orientation)
        start_pose = pose_with_position(start_flange_position, orientation)
        contact_pose = pose_with_position(contact_flange_position, orientation)
        target_pose = pose_with_position(target_flange_position, orientation)
        probe_travel = args.standoff_mm + args.overtravel_mm
        segment_from_previous = "" if previous_start is None else pose_distance(previous_start, start_pose)
        previous_start = start_pose
        row = {
            "validation_index": str(index),
            "branch": args.branch,
            "physical_ball_id": str(args.physical_ball_id),
            "approach_x": f"{direction[0]:.6f}",
            "approach_y": f"{direction[1]:.6f}",
            "approach_z": f"{direction[2]:.6f}",
            "sphere_center_x": f"{sphere_center[0]:.6f}",
            "sphere_center_y": f"{sphere_center[1]:.6f}",
            "sphere_center_z": f"{sphere_center[2]:.6f}",
            "contact_center_distance_mm": f"{contact_distance:.6f}",
            "standoff_mm": f"{args.standoff_mm:.4f}",
            "transition_standoff_mm": f"{args.transition_standoff_mm:.4f}",
            "overtravel_mm": f"{args.overtravel_mm:.4f}",
            "probe_travel_mm": f"{probe_travel:.4f}",
            "segment_from_previous_start_mm": "" if segment_from_previous == "" else f"{segment_from_previous:.4f}",
            "fit_json": str(args.fit_json),
            "euler_sequence": args.euler_sequence,
            "planned_ball_center_contact_x": f"{contact_ball_center[0]:.4f}",
            "planned_ball_center_contact_y": f"{contact_ball_center[1]:.4f}",
            "planned_ball_center_contact_z": f"{contact_ball_center[2]:.4f}",
            "probe_adapter_length_mm": f"{dims['adapter_length_mm']:.4f}",
            "probe_adapter_radius_mm": f"{dims['adapter_radius_mm']:.4f}",
            "probe_body_length_mm": f"{dims['probe_body_length_mm']:.4f}",
            "probe_body_radius_mm": f"{dims['probe_body_radius_mm']:.4f}",
            "probe_branch_center_distance_mm": f"{dims['branch_center_distance_mm']:.4f}",
            "probe_configured_rod_radius_mm": f"{dims['configured_rod_radius_mm']:.4f}",
            "probe_rod_collision_radius_mm": f"{dims['rod_collision_radius_mm']:.4f}",
            "probe_ball_radius_mm": f"{dims['ball_radius_mm']:.4f}",
            "target_stem_exclusion_mm": f"{args.target_stem_exclusion_mm:.4f}",
            "calibrated_branch_origin_x": f"{calibrated_origin[0]:.4f}",
            "calibrated_branch_origin_y": f"{calibrated_origin[1]:.4f}",
            "calibrated_branch_origin_z": f"{calibrated_origin[2]:.4f}",
        }
        collision_checks = [
            combined_collision_summary(transition_pose, primitives, sphere_center, args.sphere_radius_mm, args),
            combined_collision_summary(start_pose, primitives, sphere_center, args.sphere_radius_mm, args),
            combined_collision_summary(contact_pose, primitives, sphere_center, args.sphere_radius_mm, args),
            combined_collision_summary(target_pose, primitives, sphere_center, args.sphere_radius_mm, args),
            sample_pose_segment_clearance(
                transition_pose,
                start_pose,
                primitives,
                sphere_center,
                args.sphere_radius_mm,
                args,
            ),
            sample_pose_segment_clearance(
                start_pose,
                target_pose,
                primitives,
                sphere_center,
                args.sphere_radius_mm,
                args,
            ),
        ]
        if args.safe_pose is not None:
            collision_checks.append(
                sample_pose_segment_clearance(
                    args.safe_pose,
                    transition_pose,
                    primitives,
                    sphere_center,
                    args.sphere_radius_mm,
                    args,
                )
            )
        worst_collision = worst_collision_summary(collision_checks)
        safe_transition_orientation_delta = None
        if args.safe_pose is not None:
            safe_transition_orientation_delta = max_orientation_delta_deg(transition_pose, args.safe_pose)
        row.update(
            {
                "probe_collision_status": worst_collision["status"],
                "safe_transition_orientation_delta_deg": (
                    "" if safe_transition_orientation_delta is None else f"{safe_transition_orientation_delta:.4f}"
                ),
                "max_safe_transition_orientation_delta_deg": f"{args.max_safe_transition_orientation_delta_deg:.4f}",
                "min_probe_clearance_mm": f"{worst_collision['clearance']:.4f}",
                "closest_probe_part": worst_collision["primitive"],
                "closest_probe_model": worst_collision["primitive_model"],
                "closest_obstacle": worst_collision["obstacle"],
                "collision_model_note": "target branch uses calibrated ball offset; other probe parts are nominal or target-origin estimates",
                "collision_segment_samples": str(args.collision_segment_samples),
                "table_plane_check": str(int(not args.disable_table_plane_check)),
                "table_plane_z_mm": f"{args.table_plane_z_mm:.4f}",
                "min_table_clearance_mm": f"{args.min_table_clearance_mm:.4f}",
            }
        )
        if args.safe_pose is not None:
            row.update(pose_dict("safe_flange", args.safe_pose))
        row.update(pose_dict("transition_flange", transition_pose))
        row.update(pose_dict("start_flange", start_pose))
        row.update(pose_dict("contact_flange", contact_pose))
        row.update(pose_dict("target_flange", target_pose))
        rows.append(
            {
                "row": row,
                "approach": direction,
                "orientation": orientation,
                "transition_pose": transition_pose,
                "start_pose": start_pose,
                "contact_pose": contact_pose,
                "target_pose": target_pose,
                "sphere_center": sphere_center,
                "local_offset": local_offset,
                "contact_distance": contact_distance,
            }
        )
    return rows


def write_csv(path, rows, key="row"):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not rows:
        raise ValueError("no rows to write")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    dict_rows = [item[key] if key else item for item in rows]
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(dict_rows[0].keys()))
        writer.writeheader()
        writer.writerows(dict_rows)


def append_stable_row(path, row):
    """Append stable row."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    exists = output.exists()
    if exists:
        with output.open(newline="") as f:
            header = next(csv.reader(f), None)
        if header != list(row.keys()):
            raise ValueError(f"CSV header mismatch for {output}; choose a new --output")
    with output.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def print_plan_summary(plan):
    """Print plan summary."""
    print(f"planned validation points: {len(plan)}")
    for item in plan:
        row = item["row"]
        safe = ""
        if "safe_flange_x" in row:
            safe_pose = [row[f"safe_flange_{name}"] for name in POSE_NAMES]
            safe = f"safe=[{', '.join(safe_pose)}] "
        print(
            f"  #{row['validation_index']} approach="
            f"[{row['approach_x']}, {row['approach_y']}, {row['approach_z']}] "
            f"{safe}"
            f"transition={format_pose(item['transition_pose'])} "
            f"start={format_pose(item['start_pose'])} "
            f"target={format_pose(item['target_pose'])} "
            f"probe_travel={row['probe_travel_mm']}mm "
            f"collision={row['probe_collision_status']} "
            f"clearance={row['min_probe_clearance_mm']}mm "
            f"part={row['closest_probe_part']}"
        )


def validate_args(args):
    """Validate args."""
    if args.sphere_radius_mm <= 0 or args.probe_radius_mm <= 0:
        raise ValueError("sphere/probe radius must be positive")
    if args.point_count <= 0:
        raise ValueError("--point-count must be positive")
    if args.standoff_mm <= 0:
        raise ValueError("--standoff-mm must be positive")
    if args.transition_standoff_mm <= args.standoff_mm:
        raise ValueError("--transition-standoff-mm must be greater than --standoff-mm")
    if args.overtravel_mm <= 0:
        raise ValueError("--overtravel-mm must be positive")
    if args.standoff_mm + args.overtravel_mm > args.max_probe_travel_mm:
        raise ValueError("--standoff-mm + --overtravel-mm exceeds --max-probe-travel-mm")
    if args.min_probe_clearance_mm < 0:
        raise ValueError("--min-probe-clearance-mm cannot be negative")
    if args.max_safe_transition_orientation_delta_deg < 0:
        raise ValueError("--max-safe-transition-orientation-delta-deg cannot be negative")
    if args.min_table_clearance_mm < 0:
        raise ValueError("--min-table-clearance-mm cannot be negative")
    if not math.isfinite(args.table_plane_z_mm):
        raise ValueError("--table-plane-z-mm must be finite")
    if args.target_stem_exclusion_mm < 0:
        raise ValueError("--target-stem-exclusion-mm cannot be negative")
    if args.collision_segment_samples < 2:
        raise ValueError("--collision-segment-samples must be at least 2")
    if args.rod_collision_radius_mm is not None and args.rod_collision_radius_mm <= 0:
        raise ValueError("--rod-collision-radius-mm must be positive")
    if args.max_probe_travel_mm > 1.0 and not args.allow_probe_travel_over_1mm:
        raise ValueError("--max-probe-travel-mm > 1 requires --allow-probe-travel-over-1mm")
    if args.timeout_sec <= 0 or args.positioning_timeout_sec <= 0:
        raise ValueError("timeouts must be positive")
    if not 1 <= args.speed <= 5:
        raise ValueError("--speed must be between 1 and 5")
    if args.execute and not args.ack_sphere_validation_path:
        raise ValueError("real robot motion requires --ack-sphere-validation-path")
    if args.execute and args.safe_pose is None and not args.allow_no_safe_pose:
        raise ValueError("real robot motion requires --safe-pose, or explicitly pass --allow-no-safe-pose")
    if args.execute and args.orientation is not None and args.orientation_mode == "current":
        raise ValueError("--orientation and --orientation-mode current are mutually exclusive")
    if args.safe_pose is not None and len(args.safe_pose) != 6:
        raise ValueError("--safe-pose must contain X Y Z RX RY RZ")


def current_pose_from_robot(args):
    """Current pose from robot."""
    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(args.service_timeout_sec)
        node.wait_fresh_feed()
        pose = current_pose(node, max_age_sec=0.5)
        return node, pose
    except Exception:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise


def validation_row(args, item, trigger_snapshot, stop_pose, status):
    """Validation row."""
    trigger_pose = trigger_snapshot.get("pose") if trigger_snapshot else None
    stop_delta = None if trigger_pose is None else sub(stop_pose[:3], trigger_pose[:3])
    ball_center = center_from_row = residual = None
    residual_norm = None
    if trigger_snapshot is not None:
        rotation = euler_to_matrix(args.euler_sequence, trigger_pose[3:6])
        ball_center = add(trigger_pose[:3], mat_vec(rotation, item["local_offset"]))
        center_from_row = add(ball_center, scale(item["approach"], item["contact_distance"]))
        residual = sub(center_from_row, item["sphere_center"])
        residual_norm = norm(residual)
    row = {
        "timestamp": f"{time.time():.3f}",
        "validation_index": item["row"]["validation_index"],
        "status": status,
        "session_id": args.session_id,
        "workpiece_id": args.workpiece_id,
        "artifact_id": args.artifact_id,
        "artifact_type": args.artifact_type,
        "physical_ball_id": str(args.physical_ball_id),
        "branch": args.branch,
        "operator_note": args.operator_note,
        "fit_json": str(args.fit_json),
        "euler_sequence": args.euler_sequence,
        "approach_x": f"{item['approach'][0]:.6f}",
        "approach_y": f"{item['approach'][1]:.6f}",
        "approach_z": f"{item['approach'][2]:.6f}",
        "sphere_center_x": f"{item['sphere_center'][0]:.6f}",
        "sphere_center_y": f"{item['sphere_center'][1]:.6f}",
        "sphere_center_z": f"{item['sphere_center'][2]:.6f}",
        "fitted_ball_center_x": "" if ball_center is None else f"{ball_center[0]:.4f}",
        "fitted_ball_center_y": "" if ball_center is None else f"{ball_center[1]:.4f}",
        "fitted_ball_center_z": "" if ball_center is None else f"{ball_center[2]:.4f}",
        "sphere_center_from_row_x": "" if center_from_row is None else f"{center_from_row[0]:.4f}",
        "sphere_center_from_row_y": "" if center_from_row is None else f"{center_from_row[1]:.4f}",
        "sphere_center_from_row_z": "" if center_from_row is None else f"{center_from_row[2]:.4f}",
        "residual_x": "" if residual is None else f"{residual[0]:.6f}",
        "residual_y": "" if residual is None else f"{residual[1]:.6f}",
        "residual_z": "" if residual is None else f"{residual[2]:.6f}",
        "residual_norm_mm": "" if residual_norm is None else f"{residual_norm:.6f}",
        "stop_overtravel_along_approach_mm": (
            "" if stop_delta is None else f"{float(np.dot(stop_delta, item['approach'])):.6f}"
        ),
        "trigger_feed_sequence": "" if trigger_snapshot is None else str(trigger_snapshot.get("sequence", "")),
        "trigger_feed_wall_time": (
            "" if trigger_snapshot is None or trigger_snapshot.get("wall_time") is None
            else f"{float(trigger_snapshot['wall_time']):.6f}"
        ),
        "trigger_digital_input_bits": (
            "" if trigger_snapshot is None or trigger_snapshot.get("digital_input_bits") is None
            else str(trigger_snapshot["digital_input_bits"])
        ),
        "trigger_di1": (
            "" if trigger_snapshot is None or trigger_snapshot.get("di1") is None
            else str(int(bool(trigger_snapshot["di1"])))
        ),
    }
    row.update(pose_fields("start_flange", item["start_pose"]))
    row.update(pose_fields("target_flange", item["target_pose"]))
    row.update(pose_fields("trigger_flange", trigger_pose))
    row.update(pose_fields("stop_flange", stop_pose))
    row.update(vector_fields("trigger_joint", None if trigger_snapshot is None else trigger_snapshot.get("joints"), 6))
    return row


def move_to_pose_guarded(node, pose, args, label):
    """Move to pose guarded."""
    future = issue_movl(node, pose)
    trigger_snapshot, reached_target = wait_jog(
        node,
        pose,
        args.positioning_timeout_sec,
        args.position_tolerance_mm,
        args.orientation_tolerance_deg,
    )
    if future.done():
        node.check_ready_future(node.movl_cli, future)
    if trigger_snapshot is not None:
        raise RuntimeError(f"DI1 triggered while moving to {label}; motion stopped")
    if not reached_target:
        raise RuntimeError(f"did not reach {label}")


def move_to_pose_guarded_and_confirm_clear(node, pose, args, label):
    """Move to pose guarded and confirm clear."""
    move_to_pose_guarded(node, pose, args, label)
    node.wait_fresh_feed()
    if fresh_feed_di1(node, max_age_sec=0.5):
        raise RuntimeError(f"DI1 is triggered after moving to {label}; path is not clear")


def execute_plan(node, args, plan):
    """Execute plan."""
    large_orientation = [
        item["row"]
        for item in plan
        if item["row"].get("safe_transition_orientation_delta_deg") not in ("", None)
        and float(item["row"]["safe_transition_orientation_delta_deg"]) > args.max_safe_transition_orientation_delta_deg
    ]
    if large_orientation and not args.allow_large_safe_transition_orientation_change:
        detail = ", ".join(
            f"#{row['validation_index']} {row['safe_transition_orientation_delta_deg']}deg"
            for row in large_orientation
        )
        raise RuntimeError(
            "safe->transition orientation change is too large; this can hit joint limits. "
            f"Use a closer taught safe pose, choose another orientation, or pass "
            f"--allow-large-safe-transition-orientation-change after manual review: {detail}"
        )
    risky = [
        item["row"]
        for item in plan
        if item["row"]["probe_collision_status"] != "OK"
    ]
    if risky and not args.allow_probe_model_collision_risk:
        detail = ", ".join(
            f"#{row['validation_index']} {row['probe_collision_status']} "
            f"{row['min_probe_clearance_mm']}mm {row['closest_probe_part']}"
            for row in risky
        )
        raise RuntimeError(
            "probe model collision check is not OK; inspect plan or pass "
            f"--allow-probe-model-collision-risk after manual review: {detail}"
        )
    node.wait_fresh_feed()
    if fresh_feed_di1(node, max_age_sec=0.5):
        raise RuntimeError("DI1 is already triggered before validation; retract or clear the probe first")
    node.set_speed(args.speed)
    residuals = []
    for item in plan:
        index = item["row"]["validation_index"]
        print(f"\nvalidation point #{index}")
        if args.safe_pose is not None:
            print(f"move to safe pose: {format_pose(args.safe_pose)}")
            move_to_pose_guarded_and_confirm_clear(node, args.safe_pose, args, f"safe pose before #{index}")
        print(f"move to transition: {format_pose(item['transition_pose'])}")
        move_to_pose_guarded_and_confirm_clear(node, item["transition_pose"], args, f"transition #{index}")
        if fresh_feed_di1(node, max_age_sec=0.5):
            raise RuntimeError(f"DI1 triggered at validation transition #{index}; path is not clear")
        print(f"move to start: {format_pose(item['start_pose'])}")
        move_to_pose_guarded_and_confirm_clear(node, item["start_pose"], args, f"start #{index}")
        node.wait_fresh_feed()
        if fresh_feed_di1(node, max_age_sec=0.5):
            raise RuntimeError(f"DI1 triggered at validation start #{index}; path is not clear")
        print(f"short probe target: {format_pose(item['target_pose'])}")
        future = issue_movl(node, item["target_pose"])
        trigger_snapshot, reached_target = wait_jog(
            node,
            item["target_pose"],
            args.timeout_sec,
            args.position_tolerance_mm,
            args.orientation_tolerance_deg,
        )
        if future.done():
            node.check_ready_future(node.movl_cli, future)
        stop_pose = current_pose(node, max_age_sec=0.5)
        if trigger_snapshot is None:
            print("  no DI1 trigger; retracting and recording MISS")
            row = validation_row(args, item, None, stop_pose, "MISS")
            append_stable_row(args.output, row)
            move_to_pose_guarded_and_confirm_clear(
                node,
                item["transition_pose"],
                args,
                f"retract transition after MISS #{index}",
            )
            if args.safe_pose is not None:
                print(f"  returning to safe pose: {format_pose(args.safe_pose)}")
                move_to_pose_guarded_and_confirm_clear(node, args.safe_pose, args, f"safe pose after MISS #{index}")
            continue
        row = validation_row(args, item, trigger_snapshot, stop_pose, "HIT")
        append_stable_row(args.output, row)
        residuals.append(float(row["residual_norm_mm"]))
        print(f"  HIT residual_norm_mm={row['residual_norm_mm']}")
        move_to_pose_guarded_and_confirm_clear(
            node,
            item["transition_pose"],
            args,
            f"retract transition after HIT #{index}",
        )
        retract_pose = current_pose(node, max_age_sec=0.5)
        print(f"  retracted to transition: {format_pose(retract_pose)}")
        print(f"  DI1 after retract: {int(fresh_feed_di1(node, max_age_sec=0.5))}")
        if args.safe_pose is not None:
            print(f"  returning to safe pose: {format_pose(args.safe_pose)}")
            move_to_pose_guarded_and_confirm_clear(node, args.safe_pose, args, f"safe pose after HIT #{index}")
    return residuals


def print_validation_summary(residuals):
    """Print validation summary."""
    if not residuals:
        print("no HIT rows recorded; no residual summary")
        return
    arr = np.asarray(residuals, dtype=float)
    print("\nvalidation residual summary")
    print(f"  hits: {len(residuals)}")
    print(f"  rms_residual_mm: {math.sqrt(float(np.mean(arr * arr))):.6f}")
    print(f"  max_residual_mm: {float(np.max(arr)):.6f}")
    print(f"  mean_residual_mm: {float(np.mean(arr)):.6f}")


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--fit-json", default=str(DEFAULT_FIT))
    parser.add_argument("--plan-output", default=str(DEFAULT_PLAN))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--branch", default="y_neg")
    parser.add_argument("--physical-ball-id", default="1")
    parser.add_argument("--session-id", default="session_20260612_yneg_sphere_validation")
    parser.add_argument("--workpiece-id", default="calibration_sphere_20mm")
    parser.add_argument("--artifact-id", default="standard_sphere_20mm")
    parser.add_argument("--artifact-type", default="sphere")
    parser.add_argument("--operator-note", default="independent sphere validation")
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float, default=1.0)
    parser.add_argument("--point-count", type=int, default=6)
    parser.add_argument("--direction", nargs="*", type=float, help="custom directions as repeated DX DY DZ groups")
    parser.add_argument(
        "--orientation-mode",
        choices=("fit-cycled", "current"),
        default="fit-cycled",
        help="use fit residual row orientations cyclically, or current robot orientation for every point",
    )
    parser.add_argument(
        "--orientation",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="offline orientation source; only RX RY RZ are used",
    )
    parser.add_argument("--standoff-mm", type=float, default=0.5)
    parser.add_argument("--transition-standoff-mm", type=float, default=6.0)
    parser.add_argument("--overtravel-mm", type=float, default=0.3)
    parser.add_argument("--max-probe-travel-mm", type=float, default=1.0)
    parser.add_argument("--allow-probe-travel-over-1mm", action="store_true")
    parser.add_argument("--min-probe-clearance-mm", type=float, default=2.0)
    parser.add_argument("--max-safe-transition-orientation-delta-deg", type=float, default=90.0)
    parser.add_argument("--allow-large-safe-transition-orientation-change", action="store_true")
    parser.add_argument("--disable-table-plane-check", action="store_true")
    parser.add_argument("--table-plane-z-mm", type=float, default=0.0)
    parser.add_argument("--min-table-clearance-mm", type=float, default=5.0)
    parser.add_argument(
        "--rod-collision-radius-mm",
        type=float,
        help="override stylus rod collision radius; defaults to half of configured rod_radius_mm when it exceeds ball radius",
    )
    parser.add_argument("--target-stem-exclusion-mm", type=float, default=3.0)
    parser.add_argument("--collision-segment-samples", type=int, default=9)
    parser.add_argument(
        "--allow-probe-model-collision-risk",
        action="store_true",
        help="allow execution even if the simplified probe/sphere collision model reports low clearance",
    )
    parser.add_argument("--allow-no-safe-pose", action="store_true")
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--positioning-timeout-sec", type=float, default=15.0)
    parser.add_argument("--service-timeout-sec", type=float, default=10.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.08)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.08)
    parser.add_argument(
        "--safe-pose",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="operator-approved global safe waypoint used before and after each validation point",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--ack-sphere-validation-path",
        action="store_true",
        help="confirm every planned start/probe/retract path is safe for real robot motion",
    )
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    node = None
    try:
        validate_args(args)
        fit = load_fit(args.fit_json)
        current_orientation = None
        if args.execute or args.orientation_mode == "current":
            node, pose = current_pose_from_robot(args)
            current_orientation = pose[3:6]
        plan = make_plan(args, fit, current_orientation=current_orientation)
        write_csv(args.plan_output, plan)
        print_plan_summary(plan)
        print(f"saved plan: {args.plan_output}")
        if not args.execute:
            print("dry-run only; inspect the plan, then add --execute --ack-sphere-validation-path for real motion")
            return
        residuals = execute_plan(node, args, plan)
        print_validation_summary(residuals)
        print(f"saved validation contacts: {args.output}")
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
