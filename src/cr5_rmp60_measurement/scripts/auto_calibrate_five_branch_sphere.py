#!/usr/bin/env python3
"""Plan and optionally execute five-branch cross-stylus sphere calibration.

The script treats one trusted branch fit as the reference for the calibration
sphere centre. It then plans short normal probes for each requested branch,
records trigger rows, and fits each branch local ruby-ball offset with the
sphere centre held fixed.

Default mode is dry-run plan generation only. Real robot motion requires
--execute and --ack-five-branch-calibration-path.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from dobot_msgs_v4.srv import MovJ

from cross_probe_model import (
    DEFAULT_GEOMETRY,
    branch_local_offset_mm,
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
DEFAULT_REFERENCE_FIT = PROJECT_DIR / "data/yneg_gui_normal_absolute_fit_20260610.json"
DEFAULT_PLAN = PROJECT_DIR / "data/five_branch_sphere_calibration_plan_20260612.csv"
DEFAULT_OUTPUT = PROJECT_DIR / "data/five_branch_sphere_calibration_contacts_20260612.csv"
DEFAULT_CALIBRATION_OUTPUT = PROJECT_DIR / "data/five_branch_sphere_calibration_20260612.json"
DEFAULT_RESIDUAL_OUTPUT = PROJECT_DIR / "data/five_branch_sphere_calibration_residuals_20260612.csv"
DEFAULT_BRANCHES = ("z", "x_pos", "x_neg", "y_pos", "y_neg")
POSE_NAMES = ("x", "y", "z", "rx", "ry", "rz")


class PositioningTriggerError(RuntimeError):
    def __init__(self, label, trigger_snapshot, stop_pose, recovery_pose):
        super().__init__(f"DI1 triggered while moving to {label}; motion stopped and retracted")
        self.label = label
        self.trigger_snapshot = trigger_snapshot
        self.stop_pose = stop_pose
        self.recovery_pose = recovery_pose


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


def transpose_mat_vec(matrix, vector):
    """Transpose mat vec."""
    return [sum(float(matrix[row][col]) * float(vector[row]) for row in range(3)) for col in range(3)]


def pose_with_position(position, orientation):
    """Pose with position."""
    return [position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]]


def pose_dict(prefix, pose):
    """Pose dict."""
    return {f"{prefix}_{POSE_NAMES[index]}": f"{float(pose[index]):.4f}" for index in range(6)}


def angle_delta_deg(actual, expected):
    """Angle delta deg."""
    return (float(actual) - float(expected) + 180.0) % 360.0 - 180.0


def max_orientation_delta_deg(a, b):
    """Max orientation delta deg."""
    return max(abs(angle_delta_deg(a[index], b[index])) for index in range(3, 6))


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


def transform_point(pose, rotation, local_point):
    """Transform point."""
    return add(pose[:3], mat_vec(rotation, local_point))


def load_reference_fit(path):
    """Load reference fit."""
    data = json.loads(Path(path).read_text())
    for field in ("sphere_center_mm", "local_ball_offset_mm"):
        if field not in data:
            raise ValueError(f"reference fit missing {field}")
        if not isinstance(data[field], list) or len(data[field]) != 3:
            raise ValueError(f"reference fit {field} must contain 3 values")
    return data


def residual_orientations(fit):
    """Residual orientations."""
    orientations = []
    for row in fit.get("residual_rows", []):
        try:
            orientations.append(
                [
                    parse_float(row.get("rx"), "reference residual rx"),
                    parse_float(row.get("ry"), "reference residual ry"),
                    parse_float(row.get("rz"), "reference residual rz"),
                ]
            )
        except ValueError:
            continue
    return orientations


def branch_direction(geometry, branch_name):
    """Branch direction."""
    branches = branch_map(geometry)
    if branch_name not in branches:
        raise ValueError(f"unknown branch {branch_name!r}; choices: {', '.join(sorted(branches))}")
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


def offset_source_for_branch(branch, reference_fit, geometry, seed_offsets=None):
    """Offset source for branch."""
    if seed_offsets and branch in seed_offsets:
        return seed_offsets[branch], "branch_seed_fit"
    if reference_fit.get("branch") == branch:
        return [float(v) for v in reference_fit["local_ball_offset_mm"]], "reference_fit"
    return branch_local_offset_mm(geometry, branch), "nominal_geometry"


def load_branch_seed_offsets(items):
    """Load branch seed offsets."""
    result = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError("--branch-seed-fit entries must be BRANCH=PATH")
        branch, path = item.split("=", 1)
        branch = branch.strip()
        path = path.strip()
        if not branch or not path:
            raise ValueError("--branch-seed-fit entries must be BRANCH=PATH")
        data = json.loads(Path(path).read_text())
        offset = data.get("local_ball_offset_mm")
        if not isinstance(offset, list) or len(offset) != 3:
            fit_result = next(
                (
                    branch_result
                    for branch_result in data.get("branches", [])
                    if branch_result.get("branch") == branch and branch_result.get("ok", True)
                ),
                None,
            )
            if fit_result is not None:
                offset = fit_result.get("estimated_offset_mm")
        if not isinstance(offset, list) or len(offset) != 3:
            raise ValueError(
                f"{path}: missing 3-value local_ball_offset_mm "
                f"or branches[].estimated_offset_mm for {branch}"
            )
        result[branch] = [float(value) for value in offset]
    return result


def local_collision_primitives(geometry, active_branch, active_offset, args, collision_offsets=None):
    """Local collision primitives."""
    dims = probe_dimensions(geometry, args)
    nominal_origin = nominal_branch_origin_mm(geometry)
    calibrated_active_vector = sub(active_offset, nominal_origin)
    calibrated_active_length = norm(calibrated_active_vector)
    if calibrated_active_length <= 1e-9:
        raise ValueError(f"{active_branch} calibrated offset coincides with the branch origin")
    calibrated_active_direction = normalize(
        calibrated_active_vector,
        f"{active_branch} calibrated stem direction",
    )
    active_origin = list(nominal_origin)
    branch_dirs = {
        name: normalize(branch["direction"], f"{name} direction")
        for name, branch in branch_map(geometry).items()
    }

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

    for branch_name, direction in branch_dirs.items():
        if branch_name == active_branch:
            continue
        calibrated_offset = (collision_offsets or {}).get(branch_name)
        if calibrated_offset is not None:
            center = [float(value) for value in calibrated_offset]
            vector = sub(center, nominal_origin)
            if norm(vector) <= 1e-9:
                raise ValueError(f"{branch_name} collision offset coincides with the branch origin")
            primitives.append(
                {
                    "name": f"calibrated_{branch_name}_stem",
                    "type": "capsule",
                    "a": nominal_origin,
                    "b": center,
                    "radius": dims["rod_collision_radius_mm"],
                    "model": "calibrated_other_branch",
                }
            )
            primitives.append(
                {
                    "name": f"calibrated_{branch_name}_ball",
                    "type": "sphere",
                    "center": center,
                    "radius": dims["ball_radius_mm"],
                    "model": "calibrated_other_branch",
                }
            )
            continue
        primitives.append(
            {
                "name": f"nominal_{branch_name}_stem",
                "type": "capsule",
                "a": nominal_origin,
                "b": add(nominal_origin, scale(direction, dims["branch_center_distance_mm"])),
                "radius": dims["rod_collision_radius_mm"],
                "model": "nominal_other_branch",
            }
        )
        center = add(nominal_origin, scale(direction, dims["branch_center_distance_mm"]))
        primitives.append(
            {
                "name": f"nominal_{branch_name}_ball",
                "type": "sphere",
                "center": center,
                "radius": dims["ball_radius_mm"],
                "model": "nominal_other_branch",
            }
        )

    active_rod_length = max(0.0, calibrated_active_length - args.target_stem_exclusion_mm)
    primitives.append(
        {
            "name": f"calibrated_{active_branch}_stem",
            "type": "capsule",
            "a": nominal_origin,
            "b": add(nominal_origin, scale(calibrated_active_direction, active_rod_length)),
            "radius": dims["rod_collision_radius_mm"],
            "model": "calibrated_active_branch",
        }
    )
    return primitives, dims, active_origin


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


def collision_summary(pose, primitives, sphere_center, sphere_radius, args):
    """Collision summary."""
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
    summaries = [collision_summary(pose, primitives, sphere_center, sphere_radius, args)]
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


def choose_orientations(args, reference_fit, current_orientation=None):
    """Choose orientations."""
    if args.orientation:
        return [list(args.orientation[3:6])]
    if args.orientation_mode == "current":
        if current_orientation is None:
            raise ValueError("--orientation-mode current requires live robot pose")
        return [list(current_orientation)]
    orientations = residual_orientations(reference_fit)
    if not orientations:
        raise ValueError("reference fit has no residual row orientations; provide --orientation")
    return orientations


def make_plan(args, reference_fit, geometry, current_orientation=None, seed_offsets=None):
    """Make plan."""
    sphere_center = [float(v) for v in reference_fit["sphere_center_mm"]]
    contact_distance = float(
        reference_fit.get("contact_center_distance_mm", args.sphere_radius_mm + args.probe_radius_mm)
    )
    orientations = choose_orientations(args, reference_fit, current_orientation=current_orientation)
    rows = []

    for branch in args.branches:
        active_offset, offset_source = offset_source_for_branch(branch, reference_fit, geometry, seed_offsets)
        local_direction = branch_direction(geometry, branch)
        primitives, dims, active_origin = local_collision_primitives(geometry, branch, active_offset, args)
        for sample_index in range(1, args.samples_per_branch + 1):
            orientation = orientations[(sample_index - 1) % len(orientations)]
            rotation = euler_to_matrix(args.euler_sequence, orientation)
            approach = normalize(mat_vec(rotation, local_direction), f"{branch} sample {sample_index} approach")
            rotated_offset = mat_vec(rotation, active_offset)
            contact_ball_center = sub(sphere_center, scale(approach, contact_distance))
            transition_ball_center = sub(contact_ball_center, scale(approach, args.transition_standoff_mm))
            start_ball_center = sub(contact_ball_center, scale(approach, args.standoff_mm))
            target_ball_center = add(contact_ball_center, scale(approach, args.overtravel_mm))
            transition_pose = pose_with_position(sub(transition_ball_center, rotated_offset), orientation)
            start_pose = pose_with_position(sub(start_ball_center, rotated_offset), orientation)
            contact_pose = pose_with_position(sub(contact_ball_center, rotated_offset), orientation)
            target_pose = pose_with_position(sub(target_ball_center, rotated_offset), orientation)
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
            plan_id = f"{branch}_{sample_index}"
            safe_transition_orientation_delta = None
            if args.safe_pose is not None:
                safe_transition_orientation_delta = max_orientation_delta_deg(transition_pose, args.safe_pose)
            row = {
                "plan_id": plan_id,
                "branch": branch,
                "sample_in_branch": str(sample_index),
                "physical_ball_id": str(args.physical_ball_id),
                "approach_x": f"{approach[0]:.6f}",
                "approach_y": f"{approach[1]:.6f}",
                "approach_z": f"{approach[2]:.6f}",
                "sphere_center_x": f"{sphere_center[0]:.6f}",
                "sphere_center_y": f"{sphere_center[1]:.6f}",
                "sphere_center_z": f"{sphere_center[2]:.6f}",
                "contact_center_distance_mm": f"{contact_distance:.6f}",
                "offset_source": offset_source,
                "initial_offset_x": f"{active_offset[0]:.6f}",
                "initial_offset_y": f"{active_offset[1]:.6f}",
                "initial_offset_z": f"{active_offset[2]:.6f}",
                "estimated_branch_origin_x": f"{active_origin[0]:.4f}",
                "estimated_branch_origin_y": f"{active_origin[1]:.4f}",
                "estimated_branch_origin_z": f"{active_origin[2]:.4f}",
                "standoff_mm": f"{args.standoff_mm:.4f}",
                "transition_standoff_mm": f"{args.transition_standoff_mm:.4f}",
                "overtravel_mm": f"{args.overtravel_mm:.4f}",
                "probe_travel_mm": f"{args.standoff_mm + args.overtravel_mm:.4f}",
                "safe_transition_orientation_delta_deg": (
                    "" if safe_transition_orientation_delta is None else f"{safe_transition_orientation_delta:.4f}"
                ),
                "max_safe_transition_orientation_delta_deg": f"{args.max_safe_transition_orientation_delta_deg:.4f}",
                "probe_collision_status": worst_collision["status"],
                "min_probe_clearance_mm": f"{worst_collision['clearance']:.4f}",
                "closest_probe_part": worst_collision["primitive"],
                "closest_probe_model": worst_collision["primitive_model"],
                "closest_obstacle": worst_collision["obstacle"],
                "probe_adapter_length_mm": f"{dims['adapter_length_mm']:.4f}",
                "probe_adapter_radius_mm": f"{dims['adapter_radius_mm']:.4f}",
                "probe_body_length_mm": f"{dims['probe_body_length_mm']:.4f}",
                "probe_body_radius_mm": f"{dims['probe_body_radius_mm']:.4f}",
                "probe_branch_center_distance_mm": f"{dims['branch_center_distance_mm']:.4f}",
                "probe_configured_rod_radius_mm": f"{dims['configured_rod_radius_mm']:.4f}",
                "probe_rod_collision_radius_mm": f"{dims['rod_collision_radius_mm']:.4f}",
                "probe_ball_radius_mm": f"{dims['ball_radius_mm']:.4f}",
                "target_stem_exclusion_mm": f"{args.target_stem_exclusion_mm:.4f}",
                "collision_segment_samples": str(args.collision_segment_samples),
                "table_plane_check": str(int(not args.disable_table_plane_check)),
                "table_plane_z_mm": f"{args.table_plane_z_mm:.4f}",
                "min_table_clearance_mm": f"{args.min_table_clearance_mm:.4f}",
                "reference_fit_json": str(args.reference_fit_json),
                "euler_sequence": args.euler_sequence,
            }
            if args.safe_pose is not None:
                row.update(pose_dict("safe_flange", args.safe_pose))
            row.update(pose_dict("transition_flange", transition_pose))
            row.update(pose_dict("start_flange", start_pose))
            row.update(pose_dict("contact_flange", contact_pose))
            row.update(pose_dict("target_flange", target_pose))
            rows.append(
                {
                    "row": row,
                    "branch": branch,
                    "approach": approach,
                    "sphere_center": sphere_center,
                    "contact_distance": contact_distance,
                    "initial_offset": active_offset,
                    "transition_pose": transition_pose,
                    "start_pose": start_pose,
                    "contact_pose": contact_pose,
                    "target_pose": target_pose,
                }
            )
    return rows


def write_csv(path, dict_rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not dict_rows:
        raise ValueError("no rows to write")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(dict_rows[0].keys()))
        writer.writeheader()
        writer.writerows(dict_rows)


def pose_from_plan_row(row, prefix):
    """Pose from plan row."""
    return [parse_float(row.get(f"{prefix}_{name}"), f"{prefix}_{name}") for name in POSE_NAMES]


def plan_from_csv(path):
    """Plan from csv."""
    rows = []
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            branch = row.get("branch", "")
            approach = normalize(
                [
                    parse_float(row.get("approach_x"), "approach_x"),
                    parse_float(row.get("approach_y"), "approach_y"),
                    parse_float(row.get("approach_z"), "approach_z"),
                ],
                "approach",
            )
            sphere_center = [
                parse_float(row.get("sphere_center_x"), "sphere_center_x"),
                parse_float(row.get("sphere_center_y"), "sphere_center_y"),
                parse_float(row.get("sphere_center_z"), "sphere_center_z"),
            ]
            initial_offset = [
                parse_float(row.get("initial_offset_x"), "initial_offset_x"),
                parse_float(row.get("initial_offset_y"), "initial_offset_y"),
                parse_float(row.get("initial_offset_z"), "initial_offset_z"),
            ]
            rows.append(
                {
                    "row": row,
                    "branch": branch,
                    "approach": approach,
                    "sphere_center": sphere_center,
                    "contact_distance": parse_float(row.get("contact_center_distance_mm"), "contact_center_distance_mm"),
                    "initial_offset": initial_offset,
                    "transition_pose": pose_from_plan_row(row, "transition_flange"),
                    "start_pose": pose_from_plan_row(row, "start_flange"),
                    "contact_pose": pose_from_plan_row(row, "contact_flange"),
                    "target_pose": pose_from_plan_row(row, "target_flange"),
                }
            )
    if not rows:
        raise ValueError(f"{path}: no plan rows")
    return rows


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
    print(f"planned branch samples: {len(plan)}")
    for item in plan:
        row = item["row"]
        print(
            f"  {row['plan_id']} branch={row['branch']} "
            f"approach=[{row['approach_x']}, {row['approach_y']}, {row['approach_z']}] "
            f"start={format_pose(item['start_pose'])} "
            f"target={format_pose(item['target_pose'])} "
            f"collision={row['probe_collision_status']} "
            f"clearance={row['min_probe_clearance_mm']}mm "
            f"part={row['closest_probe_part']}"
        )


def current_pose_from_robot(args):
    """Current pose from robot."""
    rclpy.init()
    node = ProbeTouch()
    node.movj_cli = node.create_client(MovJ, "/dobot_bringup_ros2/srv/MovJ")
    try:
        node.wait_services(args.service_timeout_sec)
        if not node.movj_cli.wait_for_service(timeout_sec=args.service_timeout_sec):
            raise RuntimeError(f"service not available: {node.movj_cli.srv_name}")
        node.wait_fresh_feed()
        pose = current_pose(node, max_age_sec=0.5)
        return node, pose
    except Exception:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise


def move_to_pose_guarded(node, pose, args, label):
    """Move to pose guarded."""
    previous_pose = current_pose(node, max_age_sec=0.5)
    use_movj = args.safe_transition_move == "movj" and (
        label.startswith("transition ") or label.startswith("safe pose after ")
    )
    future = issue_movj(node, pose) if use_movj else issue_movl(node, pose)
    trigger_snapshot, reached_target = wait_jog(
        node,
        pose,
        args.positioning_timeout_sec,
        args.position_tolerance_mm,
        args.orientation_tolerance_deg,
    )
    if future.done():
        node.check_ready_future(node.movj_cli if use_movj else node.movl_cli, future)
    if trigger_snapshot is not None:
        stop_pose = trigger_snapshot.get("pose") or current_pose(node, max_age_sec=0.5)
        reverse = sub(previous_pose[:3], stop_pose[:3])
        reverse_length = norm(reverse)
        if reverse_length <= 1e-9:
            raise RuntimeError(f"DI1 triggered while moving to {label}; no reverse path is available")
        retract_distance = min(float(args.positioning_trigger_retract_mm), reverse_length)
        recovery_pose = list(stop_pose)
        for index in range(3):
            recovery_pose[index] += reverse[index] / reverse_length * retract_distance
        print(
            f"DI1 triggered during positioning; retracting {retract_distance:.4f}mm "
            f"along the verified reverse path to {format_pose(recovery_pose)}"
        )
        recovery_future = issue_movl(node, recovery_pose)
        node.wait_until_pose(
            recovery_pose,
            position_tolerance_mm=args.position_tolerance_mm,
            orientation_tolerance_deg=args.orientation_tolerance_deg,
            timeout_sec=args.positioning_timeout_sec,
        )
        if recovery_future.done():
            node.check_ready_future(node.movl_cli, recovery_future)
        node.wait_fresh_feed()
        if fresh_feed_di1(node, max_age_sec=0.5):
            raise RuntimeError(
                f"DI1 remains triggered after reverse-path recovery from {label}; manual clearance required"
            )
        raise PositioningTriggerError(label, trigger_snapshot, stop_pose, recovery_pose)
    if not reached_target:
        raise RuntimeError(f"did not reach {label}")


def issue_movj(node, pose):
    """Issue movj."""
    request = MovJ.Request()
    request.mode = False
    request.a, request.b, request.c, request.d, request.e, request.f = [float(value) for value in pose]
    future, _ = node.call_async_checked(node.movj_cli, request)
    return future


def move_to_pose_guarded_and_confirm_clear(node, pose, args, label):
    """Move to pose guarded and confirm clear."""
    move_to_pose_guarded(node, pose, args, label)
    node.wait_fresh_feed()
    if fresh_feed_di1(node, max_age_sec=0.5):
        raise RuntimeError(f"DI1 is triggered after moving to {label}; path is not clear")


def retract_to_pose_after_probe(node, pose, args, label):
    """Retract after a probe hit; DI1 may still be active at the beginning."""
    future = issue_movl(node, pose)
    node.wait_until_pose(
        pose,
        position_tolerance_mm=args.position_tolerance_mm,
        orientation_tolerance_deg=args.orientation_tolerance_deg,
        timeout_sec=args.positioning_timeout_sec,
    )
    if future.done():
        node.check_ready_future(node.movl_cli, future)
    node.wait_fresh_feed()
    if fresh_feed_di1(node, max_age_sec=0.5):
        raise RuntimeError(f"DI1 remains triggered after {label}; manual clearance required")


def contact_row(args, item, trigger_snapshot, stop_pose, status):
    """Contact row."""
    trigger_pose = trigger_snapshot.get("pose") if trigger_snapshot else None
    row = {
        "timestamp": f"{time.time():.3f}",
        "plan_id": item["row"]["plan_id"],
        "status": status,
        "session_id": args.session_id,
        "workpiece_id": args.workpiece_id,
        "artifact_id": args.artifact_id,
        "artifact_type": args.artifact_type,
        "standard_sphere_id": str(item["row"].get("standard_sphere_id", "")),
        "physical_ball_id": str(
            item["row"].get("physical_ball_id") or args.physical_ball_id
        ),
        "branch": item["branch"],
        "operator_note": args.operator_note,
        "reference_fit_json": str(args.reference_fit_json),
        "euler_sequence": args.euler_sequence,
        "approach_x": f"{item['approach'][0]:.6f}",
        "approach_y": f"{item['approach'][1]:.6f}",
        "approach_z": f"{item['approach'][2]:.6f}",
        "sphere_center_x": f"{item['sphere_center'][0]:.6f}",
        "sphere_center_y": f"{item['sphere_center'][1]:.6f}",
        "sphere_center_z": f"{item['sphere_center'][2]:.6f}",
        "contact_center_distance_mm": f"{item['contact_distance']:.6f}",
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
            f"{row['plan_id']} {row['safe_transition_orientation_delta_deg']}deg"
            for row in large_orientation
        )
        raise RuntimeError(
            "safe->transition orientation change is too large; this can hit joint limits. "
            f"Use a closer taught safe pose, choose another orientation, or pass "
            f"--allow-large-safe-transition-orientation-change after manual review: {detail}"
        )
    risky = [item["row"] for item in plan if item["row"]["probe_collision_status"] != "OK"]
    if risky and not args.allow_probe_model_collision_risk:
        detail = ", ".join(
            f"{row['plan_id']} {row['probe_collision_status']} "
            f"{row['min_probe_clearance_mm']}mm {row['closest_probe_part']}"
            for row in risky
        )
        raise RuntimeError(
            "probe model collision check is not OK; inspect plan or use "
            f"--allow-probe-model-collision-risk after manual review: {detail}"
        )
    node.wait_fresh_feed()
    if fresh_feed_di1(node, max_age_sec=0.5):
        raise RuntimeError("DI1 is already triggered before calibration; retract or clear the probe first")
    node.set_speed(args.speed)
    if args.safe_pose is not None and args.safe_pose_policy in ("plan", "initial-only"):
        print(f"move to initial safe pose: {format_pose(args.safe_pose)}")
        move_to_pose_guarded_and_confirm_clear(node, args.safe_pose, args, "safe pose before plan")
    for item in plan:
        plan_id = item["row"]["plan_id"]
        print(f"\ncalibration sample {plan_id}")
        if args.safe_pose is not None and args.safe_pose_policy == "each-sample":
            print(f"move to safe pose: {format_pose(args.safe_pose)}")
            move_to_pose_guarded_and_confirm_clear(node, args.safe_pose, args, f"safe pose before {plan_id}")
        current = current_pose(node, max_age_sec=0.5)
        transition_distance = norm(sub(item["transition_pose"][:3], current[:3]))
        transition_orientation_delta = max_orientation_delta_deg(item["transition_pose"], current)
        if args.max_local_step_mm > 0 and transition_distance > args.max_local_step_mm:
            raise RuntimeError(
                f"local move to transition {plan_id} is {transition_distance:.4f}mm; "
                f"limit is {args.max_local_step_mm:.4f}mm"
            )
        if (
            args.max_local_orientation_step_deg > 0
            and transition_orientation_delta > args.max_local_orientation_step_deg
        ):
            raise RuntimeError(
                f"local orientation move to transition {plan_id} is {transition_orientation_delta:.4f}deg; "
                f"limit is {args.max_local_orientation_step_deg:.4f}deg"
            )
        print(f"move to transition: {format_pose(item['transition_pose'])}")
        try:
            move_to_pose_guarded_and_confirm_clear(node, item["transition_pose"], args, f"transition {plan_id}")
        except PositioningTriggerError as exc:
            append_stable_row(
                args.output,
                contact_row(args, item, exc.trigger_snapshot, exc.stop_pose, "EARLY_TRIGGER_TRANSITION"),
            )
            raise
        print(f"move to start: {format_pose(item['start_pose'])}")
        try:
            move_to_pose_guarded_and_confirm_clear(node, item["start_pose"], args, f"start {plan_id}")
        except PositioningTriggerError as exc:
            append_stable_row(
                args.output,
                contact_row(args, item, exc.trigger_snapshot, exc.stop_pose, "EARLY_TRIGGER_START"),
            )
            raise
        if fresh_feed_di1(node, max_age_sec=0.5):
            raise RuntimeError(f"DI1 triggered at calibration start {plan_id}; path is not clear")
        print(f"short probe target: {format_pose(item['target_pose'])}")
        future = issue_movl(node, item["target_pose"])
        trigger_snapshot, _ = wait_jog(
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
            print("  MISS")
            row = contact_row(args, item, None, stop_pose, "MISS")
            append_stable_row(args.output, row)
        else:
            print("  HIT")
            row = contact_row(args, item, trigger_snapshot, stop_pose, "HIT")
            append_stable_row(args.output, row)
            if args.refit_after_each_hit:
                write_calibration(args, args._reference_fit, args._geometry)
        retract_to_pose_after_probe(node, item["transition_pose"], args, f"retract transition {plan_id}")
        if args.safe_pose is not None and args.safe_pose_policy == "each-sample":
            move_to_pose_guarded_and_confirm_clear(node, args.safe_pose, args, f"safe pose after {plan_id}")
    if args.safe_pose is not None and args.safe_pose_policy in ("plan", "final-only"):
        print(f"move to final safe pose: {format_pose(args.safe_pose)}")
        move_to_pose_guarded_and_confirm_clear(node, args.safe_pose, args, "safe pose after plan")


def supported_pose_fields(row):
    """Supported pose fields."""
    if all(row.get(f"trigger_flange_{name}") not in (None, "") for name in POSE_NAMES):
        return [parse_float(row[f"trigger_flange_{name}"], f"trigger_flange_{name}") for name in POSE_NAMES]
    if all(row.get(name) not in (None, "") for name in ("flange_x", "flange_y", "flange_z", "rx", "ry", "rz")):
        return [parse_float(row[name], name) for name in ("flange_x", "flange_y", "flange_z", "rx", "ry", "rz")]
    raise ValueError("row missing supported pose fields")


def is_hit_row(row):
    """Is hit row."""
    status = row.get("status")
    if status not in (None, ""):
        return status == "HIT"
    trigger_di1 = str(row.get("trigger_di1", "")).strip()
    if trigger_di1 in ("1", "true", "True"):
        return True
    if trigger_di1 == "0":
        return False
    return row.get("source_command", "") == "semi_auto_normal"


def robust_inlier_mask(samples, args):
    """Robust inlier mask."""
    count = len(samples)
    if args.disable_outlier_rejection or count < args.outlier_min_samples:
        return [True] * count, [0.0] * count
    arr = np.asarray(samples, dtype=float)
    center = np.median(arr, axis=0)
    distances = np.linalg.norm(arr - center, axis=1)
    mask = [float(distance) <= args.outlier_threshold_mm for distance in distances]
    if sum(mask) < args.outlier_min_inliers:
        return [True] * count, [float(value) for value in distances]
    return mask, [float(value) for value in distances]


def fit_offsets_from_contacts(paths, branches, sphere_center, contact_distance, euler_sequence, args):
    """Fit offsets from contacts."""
    branch_samples = {branch: [] for branch in branches}
    branch_physical_ids = {branch: set() for branch in branches}
    residual_rows = []
    source_rows = {branch: [] for branch in branches}
    for path in paths:
        path = Path(path)
        if not path.exists():
            continue
        with path.open(newline="") as f:
            for row_index, row in enumerate(csv.DictReader(f), start=1):
                if not is_hit_row(row):
                    continue
                branch = row.get("branch", "")
                if branch not in branch_samples:
                    continue
                physical_ball_id = str(row.get("physical_ball_id", "")).strip()
                if physical_ball_id:
                    branch_physical_ids[branch].add(physical_ball_id)
                pose = supported_pose_fields(row)
                approach = normalize(
                    [parse_float(row.get("approach_x"), "approach_x"),
                     parse_float(row.get("approach_y"), "approach_y"),
                     parse_float(row.get("approach_z"), "approach_z")],
                    "approach",
                )
                rotation = euler_to_matrix(euler_sequence, pose[3:6])
                rhs = sub(sub(sphere_center, scale(approach, contact_distance)), pose[:3])
                p_sample = transpose_mat_vec(rotation, rhs)
                branch_samples[branch].append(p_sample)
                source_rows[branch].append((str(path), row_index, row, pose, approach, p_sample))

    results = []
    for branch in branches:
        samples = branch_samples[branch]
        if not samples:
            results.append({"branch": branch, "ok": False, "reason": "no HIT samples"})
            continue
        physical_ids = branch_physical_ids[branch]
        if len(physical_ids) > 1:
            raise ValueError(
                f"{branch}: contact inputs mix physical_ball_id values: "
                + ", ".join(sorted(physical_ids))
            )
        physical_ball_id = next(iter(physical_ids), str(args.physical_ball_id))
        inlier_mask, sample_distances = robust_inlier_mask(samples, args)
        inlier_samples = [sample for sample, keep in zip(samples, inlier_mask) if keep]
        estimate = np.mean(np.asarray(inlier_samples, dtype=float), axis=0)
        residual_norms = []
        inlier_residual_norms = []
        for sample_index, (path, row_index, row, pose, approach, p_sample) in enumerate(source_rows[branch]):
            rotation = euler_to_matrix(euler_sequence, pose[3:6])
            ball_center = add(pose[:3], mat_vec(rotation, estimate.tolist()))
            center_from_row = add(ball_center, scale(approach, contact_distance))
            residual = sub(center_from_row, sphere_center)
            residual_norm = norm(residual)
            residual_norms.append(residual_norm)
            if inlier_mask[sample_index]:
                inlier_residual_norms.append(residual_norm)
            residual_rows.append(
                {
                    "source_csv": path,
                    "source_row": str(row_index),
                    "plan_id": row.get("plan_id", ""),
                    "branch": branch,
                    "physical_ball_id": str(row.get("physical_ball_id", "")),
                    "outlier_status": "inlier" if inlier_mask[sample_index] else "outlier",
                    "p_sample_distance_to_median_mm": f"{sample_distances[sample_index]:.6f}",
                    "approach_x": f"{approach[0]:.6f}",
                    "approach_y": f"{approach[1]:.6f}",
                    "approach_z": f"{approach[2]:.6f}",
                    "p_sample_x": f"{p_sample[0]:.6f}",
                    "p_sample_y": f"{p_sample[1]:.6f}",
                    "p_sample_z": f"{p_sample[2]:.6f}",
                    "estimated_offset_x": f"{estimate[0]:.6f}",
                    "estimated_offset_y": f"{estimate[1]:.6f}",
                    "estimated_offset_z": f"{estimate[2]:.6f}",
                    "sphere_center_from_row_x": f"{center_from_row[0]:.4f}",
                    "sphere_center_from_row_y": f"{center_from_row[1]:.4f}",
                    "sphere_center_from_row_z": f"{center_from_row[2]:.4f}",
                    "residual_x": f"{residual[0]:.6f}",
                    "residual_y": f"{residual[1]:.6f}",
                    "residual_z": f"{residual[2]:.6f}",
                    "residual_norm_mm": f"{residual_norm:.6f}",
                }
            )
        arr = np.asarray(inlier_residual_norms or residual_norms, dtype=float)
        results.append(
            {
                "branch": branch,
                "physical_ball_id": physical_ball_id,
                "ok": True,
                "rows": len(inlier_samples),
                "raw_rows": len(samples),
                "rejected_rows": int(len(samples) - len(inlier_samples)),
                "estimated_offset_mm": estimate.tolist(),
                "rms_residual_mm": float(math.sqrt(float(np.mean(arr * arr)))),
                "max_residual_mm": float(np.max(arr)),
                "mean_residual_mm": float(np.mean(arr)),
            }
        )
    return results, residual_rows


def write_calibration(args, reference_fit, geometry):
    """Write calibration."""
    sphere_center = [float(v) for v in reference_fit["sphere_center_mm"]]
    contact_distance = float(
        reference_fit.get("contact_center_distance_mm", args.sphere_radius_mm + args.probe_radius_mm)
    )
    branches, residual_rows = fit_offsets_from_contacts(
        [args.output] + list(args.fit_input or []),
        args.branches,
        sphere_center,
        contact_distance,
        args.euler_sequence,
        args,
    )
    result = {
        "timestamp": time.time(),
        "method": "fixed_sphere_center_five_branch_offset_fit",
        "method_warning": (
            "Sphere centre is held fixed from the reference branch fit. "
            "Branches without HIT rows remain uncalibrated."
        ),
        "reference_fit_json": str(args.reference_fit_json),
        "sphere_center_mm": sphere_center,
        "contact_center_distance_mm": contact_distance,
        "geometry": str(args.geometry),
        "euler_sequence": args.euler_sequence,
        "outlier_rejection": {
            "enabled": not args.disable_outlier_rejection,
            "threshold_mm": args.outlier_threshold_mm,
            "min_samples": args.outlier_min_samples,
            "min_inliers": args.outlier_min_inliers,
        },
        "branches": branches,
    }
    output = Path(args.calibration_output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(f"saved calibration JSON: {output}")
    if residual_rows:
        write_csv(args.residual_output, residual_rows)
        print(f"saved residual CSV: {args.residual_output}")
    for branch in branches:
        if not branch.get("ok"):
            print(f"  {branch['branch']}: {branch['reason']}")
            continue
        offset = branch["estimated_offset_mm"]
        print(
            f"  {branch['branch']}: rows={branch['rows']}/{branch.get('raw_rows', branch['rows'])} "
            f"rejected={branch.get('rejected_rows', 0)} "
            f"offset=[{offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}] "
            f"rms={branch['rms_residual_mm']:.6f} max={branch['max_residual_mm']:.6f}"
        )


def validate_args(args):
    """Validate args."""
    if args.samples_per_branch <= 0:
        raise ValueError("--samples-per-branch must be positive")
    if args.sphere_radius_mm <= 0 or args.probe_radius_mm <= 0:
        raise ValueError("sphere/probe radius must be positive")
    if args.standoff_mm <= 0:
        raise ValueError("--standoff-mm must be positive")
    if args.transition_standoff_mm <= args.standoff_mm:
        raise ValueError("--transition-standoff-mm must be greater than --standoff-mm")
    if args.overtravel_mm <= 0:
        raise ValueError("--overtravel-mm must be positive")
    if args.standoff_mm + args.overtravel_mm > args.max_probe_travel_mm:
        raise ValueError("--standoff-mm + --overtravel-mm exceeds --max-probe-travel-mm")
    if args.max_probe_travel_mm > 1.0 and not args.allow_probe_travel_over_1mm:
        raise ValueError("--max-probe-travel-mm > 1 requires --allow-probe-travel-over-1mm")
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
    if args.outlier_threshold_mm <= 0:
        raise ValueError("--outlier-threshold-mm must be positive")
    if args.outlier_min_samples < 1:
        raise ValueError("--outlier-min-samples must be positive")
    if args.outlier_min_inliers < 1:
        raise ValueError("--outlier-min-inliers must be positive")
    if args.max_local_step_mm < 0:
        raise ValueError("--max-local-step-mm cannot be negative")
    if args.max_local_orientation_step_deg < 0:
        raise ValueError("--max-local-orientation-step-deg cannot be negative")
    if not 1 <= args.speed <= 5:
        raise ValueError("--speed must be between 1 and 5")
    if args.timeout_sec <= 0 or args.positioning_timeout_sec <= 0:
        raise ValueError("timeouts must be positive")
    if args.positioning_trigger_retract_mm <= 0:
        raise ValueError("--positioning-trigger-retract-mm must be positive")
    if args.execute and not args.ack_five_branch_calibration_path:
        raise ValueError("real robot motion requires --ack-five-branch-calibration-path")
    if args.execute and args.safe_pose is None and not args.allow_no_safe_pose:
        raise ValueError("real robot motion requires --safe-pose, or explicitly pass --allow-no-safe-pose")
    if args.execute and args.orientation is not None and args.orientation_mode == "current":
        raise ValueError("--orientation and --orientation-mode current are mutually exclusive")
    if args.safe_pose is not None and len(args.safe_pose) != 6:
        raise ValueError("--safe-pose must contain X Y Z RX RY RZ")


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-fit-json", default=str(DEFAULT_REFERENCE_FIT))
    parser.add_argument("--plan-input", help="execute an existing compatible plan CSV instead of generating a new plan")
    parser.add_argument("--plan-output", default=str(DEFAULT_PLAN))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--calibration-output", default=str(DEFAULT_CALIBRATION_OUTPUT))
    parser.add_argument("--residual-output", default=str(DEFAULT_RESIDUAL_OUTPUT))
    parser.add_argument("--fit-input", nargs="*", default=[])
    parser.add_argument(
        "--branch-seed-fit",
        nargs="*",
        default=[],
        metavar="BRANCH=JSON",
        help="per-branch coarse fit seed, for example x_neg=data/seed.json",
    )
    parser.add_argument("--branches", nargs="+", default=list(DEFAULT_BRANCHES))
    parser.add_argument("--samples-per-branch", type=int, default=3)
    parser.add_argument("--physical-ball-id", default="five_branch_probe")
    parser.add_argument("--session-id", default="session_20260612_five_branch_sphere_calibration")
    parser.add_argument("--workpiece-id", default="calibration_sphere_20mm")
    parser.add_argument("--artifact-id", default="standard_sphere_20mm")
    parser.add_argument("--artifact-type", default="sphere")
    parser.add_argument("--operator-note", default="five branch sphere calibration")
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float, default=1.0)
    parser.add_argument(
        "--orientation-mode",
        choices=("reference-fit-cycled", "current"),
        default="reference-fit-cycled",
    )
    parser.add_argument(
        "--orientation",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="offline orientation source; only RX RY RZ are used",
    )
    parser.add_argument("--safe-pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument(
        "--safe-pose-policy",
        choices=("each-sample", "plan", "initial-only", "final-only", "none"),
        default="each-sample",
        help="when to visit --safe-pose during plan execution",
    )
    parser.add_argument(
        "--move-safe-only",
        action="store_true",
        help="move to --safe-pose and exit without loading or executing a probing plan",
    )
    parser.add_argument(
        "--report-current-pose-json",
        help="read the live flange pose, write it as JSON, and exit without motion",
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
    parser.add_argument("--rod-collision-radius-mm", type=float)
    parser.add_argument("--target-stem-exclusion-mm", type=float, default=3.0)
    parser.add_argument("--collision-segment-samples", type=int, default=9)
    parser.add_argument("--allow-probe-model-collision-risk", action="store_true")
    parser.add_argument("--allow-no-safe-pose", action="store_true")
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--safe-transition-move", choices=("movl", "movj"), default="movl")
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--positioning-timeout-sec", type=float, default=15.0)
    parser.add_argument("--positioning-trigger-retract-mm", type=float, default=5.0)
    parser.add_argument("--service-timeout-sec", type=float, default=10.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.08)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.08)
    parser.add_argument(
        "--max-local-step-mm",
        type=float,
        default=0.0,
        help="hard limit from the current pose to each transition pose; 0 disables",
    )
    parser.add_argument(
        "--max-local-orientation-step-deg",
        type=float,
        default=0.0,
        help="hard orientation limit from the current pose to each transition pose; 0 disables",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fit-only", action="store_true")
    parser.add_argument("--no-fit-after-execute", action="store_true")
    parser.add_argument(
        "--refit-after-each-hit",
        action="store_true",
        help="write updated calibration/residual files after every successful contact",
    )
    parser.add_argument("--disable-outlier-rejection", action="store_true")
    parser.add_argument("--outlier-threshold-mm", type=float, default=0.6)
    parser.add_argument("--outlier-min-samples", type=int, default=4)
    parser.add_argument("--outlier-min-inliers", type=int, default=3)
    parser.add_argument("--ack-five-branch-calibration-path", action="store_true")
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    node = None
    try:
        validate_args(args)
        reference_fit = load_reference_fit(args.reference_fit_json)
        geometry = load_geometry(args.geometry)
        seed_offsets = load_branch_seed_offsets(args.branch_seed_fit)
        args._reference_fit = reference_fit
        args._geometry = geometry
        if args.report_current_pose_json:
            node, pose = current_pose_from_robot(args)
            output = Path(args.report_current_pose_json)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(
                json.dumps(
                    {
                        "timestamp": time.time(),
                        "flange_pose_mm_deg": [float(value) for value in pose],
                    },
                    indent=2,
                )
                + "\n"
            )
            print(f"current flange pose: {format_pose(pose)}")
            print(f"saved current pose JSON: {output}")
            return
        if args.move_safe_only:
            if not args.execute:
                raise ValueError("--move-safe-only requires --execute")
            if args.safe_pose is None:
                raise ValueError("--move-safe-only requires --safe-pose")
            node, current = current_pose_from_robot(args)
            if fresh_feed_di1(node, max_age_sec=0.5):
                raise RuntimeError("DI1 is already triggered; refusing safe-pose motion")
            safe_distance = norm(sub(args.safe_pose[:3], current[:3]))
            safe_orientation_delta = max_orientation_delta_deg(args.safe_pose, current)
            if args.max_local_step_mm > 0 and safe_distance > args.max_local_step_mm:
                raise ValueError(
                    f"safe-only move is {safe_distance:.4f}mm; limit is {args.max_local_step_mm:.4f}mm"
                )
            if (
                args.max_local_orientation_step_deg > 0
                and safe_orientation_delta > args.max_local_orientation_step_deg
            ):
                raise ValueError(
                    f"safe-only orientation move is {safe_orientation_delta:.4f}deg; "
                    f"limit is {args.max_local_orientation_step_deg:.4f}deg"
                )
            node.set_speed(args.speed)
            move_to_pose_guarded_and_confirm_clear(node, args.safe_pose, args, "safe pose move only")
            print(f"reached safe pose: {format_pose(args.safe_pose)}")
            return
        if args.fit_only:
            write_calibration(args, reference_fit, geometry)
            return
        current_orientation = None
        if (args.execute or args.orientation_mode == "current") and not args.plan_input:
            node, pose = current_pose_from_robot(args)
            current_orientation = pose[3:6]
        if args.plan_input:
            plan = plan_from_csv(args.plan_input)
            if args.execute:
                node, _ = current_pose_from_robot(args)
            if args.plan_output and str(args.plan_output) != str(args.plan_input):
                write_csv(args.plan_output, [item["row"] for item in plan])
        else:
            plan = make_plan(args, reference_fit, geometry, current_orientation=current_orientation, seed_offsets=seed_offsets)
            write_csv(args.plan_output, [item["row"] for item in plan])
        print_plan_summary(plan)
        if args.plan_input:
            print(f"loaded plan: {args.plan_input}")
            if args.plan_output and str(args.plan_output) != str(args.plan_input):
                print(f"saved plan copy: {args.plan_output}")
        else:
            print(f"saved plan: {args.plan_output}")
        if not args.execute:
            print("dry-run only; inspect the plan before real motion")
            return
        execute_plan(node, args, plan)
        print(f"saved contacts: {args.output}")
        if not args.no_fit_after_execute:
            write_calibration(args, reference_fit, geometry)
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
