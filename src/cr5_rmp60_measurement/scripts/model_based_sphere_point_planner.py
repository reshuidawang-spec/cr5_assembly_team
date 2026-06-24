#!/usr/bin/env python3
"""Model-based calibration-sphere probing point planner.

This planner does not reuse measured contact poses. It generates sphere-normal
candidates from the standard sphere and cross-stylus model, converts each
candidate into flange poses, and writes a plan CSV compatible with
check_five_branch_plan_moveit.py and the existing execution scripts.
"""
import argparse
import csv
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from auto_calibrate_five_branch_sphere import (
    DEFAULT_REFERENCE_FIT,
    POSE_NAMES,
    add,
    branch_direction,
    combined_collision_summary,
    local_collision_primitives,
    load_branch_seed_offsets,
    max_orientation_delta_deg,
    norm,
    normalize,
    pose_dict,
    probe_dimensions,
    sample_pose_segment_clearance,
    scale,
    sub,
    worst_collision_summary,
)
from cross_probe_model import (
    DEFAULT_GEOMETRY,
    branch_local_offset_mm,
    euler_to_matrix,
    load_geometry,
    mat_vec,
)


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_DIR / "data/model_based_sphere_plan.csv"


def dot(a, b):
    """Dot."""
    return float(sum(float(x) * float(y) for x, y in zip(a, b)))


def cross(a, b):
    """Cross."""
    return [
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ]


def mat_mult(a, b):
    """Multiply two 3x3 matrices."""
    return [[sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)] for row in range(3)]


def transpose(m):
    """Transpose."""
    return [[m[col][row] for col in range(3)] for row in range(3)]


def columns_to_matrix(columns):
    """Columns to matrix."""
    return [[columns[col][row] for col in range(3)] for row in range(3)]


def axis_angle_matrix(axis, angle_deg):
    """Axis angle matrix."""
    axis = normalize(axis, "axis")
    x, y, z = axis
    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return [
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ]


def matrix_to_euler_xyz(matrix):
    """Inverse of cross_probe_model.euler_to_matrix('xyz', angles)."""
    r20 = max(-1.0, min(1.0, float(matrix[2][0])))
    ry = math.asin(-r20)
    cy = math.cos(ry)
    if abs(cy) > 1e-9:
        rx = math.atan2(matrix[2][1], matrix[2][2])
        rz = math.atan2(matrix[1][0], matrix[0][0])
    else:
        rx = 0.0
        rz = math.atan2(-matrix[0][1], matrix[1][1])
    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def local_basis(direction):
    """Local basis."""
    e1 = normalize(direction, "branch direction")
    q = [0.0, 0.0, 1.0]
    if abs(dot(e1, q)) > 0.95:
        q = [0.0, 1.0, 0.0]
    e2 = normalize(sub(q, scale(e1, dot(e1, q))), "local secondary")
    e3 = normalize(cross(e1, e2), "local tertiary")
    return [e1, e2, e3]


def arbitrary_world_basis(normal):
    """Arbitrary world basis."""
    e1 = normalize(normal, "normal")
    q = [0.0, 0.0, 1.0]
    if abs(dot(e1, q)) > 0.95:
        q = [1.0, 0.0, 0.0]
    e2 = normalize(sub(q, scale(e1, dot(e1, q))), "world secondary")
    e3 = normalize(cross(e1, e2), "world tertiary")
    return [e1, e2, e3]


def rotation_for_normal(branch_local_direction, normal, anchor_rotation=None, roll_deg=0.0):
    """Rotation for normal."""
    local = local_basis(branch_local_direction)
    normal = normalize(normal, "candidate normal")
    if anchor_rotation is not None:
        reference_secondary = mat_vec(anchor_rotation, local[1])
        projected = sub(reference_secondary, scale(normal, dot(reference_secondary, normal)))
        if norm(projected) <= 1e-9:
            world = arbitrary_world_basis(normal)
        else:
            world = [normal, normalize(projected, "projected secondary"), None]
            world[2] = normalize(cross(world[0], world[1]), "projected tertiary")
    else:
        world = arbitrary_world_basis(normal)
    base = mat_mult(columns_to_matrix(world), transpose(columns_to_matrix(local)))
    return mat_mult(axis_angle_matrix(normal, roll_deg), base)


def fibonacci_normals(count):
    """Fibonacci normals."""
    normals = []
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    for index in range(max(1, count)):
        z = 1.0 - 2.0 * (index + 0.5) / float(count)
        radius = math.sqrt(max(0.0, 1.0 - z * z))
        theta = golden_angle * index
        normals.append([math.cos(theta) * radius, math.sin(theta) * radius, z])
    return normals


def cone_normals(anchor_normal, cone_angle_deg, rings, samples_per_ring):
    """Cone normals."""
    anchor = normalize(anchor_normal, "anchor normal")
    basis = arbitrary_world_basis(anchor)
    normals = [anchor]
    for ring in range(1, max(1, rings) + 1):
        angle = math.radians(float(cone_angle_deg) * ring / float(max(1, rings)))
        samples = max(1, samples_per_ring * ring)
        for index in range(samples):
            azimuth = 2.0 * math.pi * index / float(samples)
            tangent = add(
                scale(basis[1], math.cos(azimuth)),
                scale(basis[2], math.sin(azimuth)),
            )
            normals.append(normalize(add(scale(anchor, math.cos(angle)), scale(tangent, math.sin(angle))), "cone normal"))
    return normals


def load_reference_sphere(path):
    """Load reference sphere."""
    data = json.loads(Path(path).read_text())
    center = data.get("sphere_center_mm")
    if not isinstance(center, list) or len(center) != 3:
        raise ValueError(f"{path}: missing 3-value sphere_center_mm")
    return data, [float(value) for value in center]


def load_branch_offset(args, geometry):
    """Load branch offset."""
    if args.branch_offset_mm:
        return [float(value) for value in args.branch_offset_mm], "branch_offset_mm"
    if args.branch_fit_json:
        data = json.loads(Path(args.branch_fit_json).read_text())
        offset = data.get("local_ball_offset_mm")
        if not isinstance(offset, list) or len(offset) != 3:
            result = next(
                (
                    item
                    for item in data.get("branches", [])
                    if item.get("branch") == args.branch and item.get("ok", True)
                ),
                None,
            )
            if result is not None:
                offset = result.get("estimated_offset_mm")
        if not isinstance(offset, list) or len(offset) != 3:
            raise ValueError(
                f"{args.branch_fit_json}: missing 3-value local_ball_offset_mm "
                f"or branches[].estimated_offset_mm for {args.branch}"
            )
        return [float(value) for value in offset], "branch_fit_json"
    return branch_local_offset_mm(geometry, args.branch), "nominal_geometry"


def pose_with_position(position, orientation):
    """Pose with position."""
    return [position[0], position[1], position[2], orientation[0], orientation[1], orientation[2]]


def plan_candidate(args, geometry, dims, primitives, active_origin, sphere_center, contact_distance, offset, normal, roll_deg, index):
    """Plan candidate."""
    branch_dir = branch_direction(geometry, args.branch)
    if args.orientation_mode == "anchor-fixed":
        if not args.anchor_pose:
            raise ValueError("--orientation-mode anchor-fixed requires --anchor-pose")
        rotation = euler_to_matrix(args.euler_sequence, args.anchor_pose[3:6])
        orientation = [float(value) for value in args.anchor_pose[3:6]]
        approach = normalize(normal, "planned approach")
    else:
        anchor_rotation = None
        if args.anchor_pose:
            anchor_rotation = euler_to_matrix(args.euler_sequence, args.anchor_pose[3:6])
        rotation = rotation_for_normal(branch_dir, normal, anchor_rotation=anchor_rotation, roll_deg=roll_deg)
        orientation = matrix_to_euler_xyz(rotation)
        approach = normalize(mat_vec(rotation, branch_dir), "planned approach")
    rotated_offset = mat_vec(rotation, offset)
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
        sample_pose_segment_clearance(transition_pose, start_pose, primitives, sphere_center, args.sphere_radius_mm, args),
        sample_pose_segment_clearance(start_pose, target_pose, primitives, sphere_center, args.sphere_radius_mm, args),
    ]
    safe_delta = ""
    safe_distance = ""
    if args.safe_pose:
        collision_checks.append(
            sample_pose_segment_clearance(args.safe_pose, transition_pose, primitives, sphere_center, args.sphere_radius_mm, args)
        )
        safe_delta_value = max_orientation_delta_deg(args.safe_pose, transition_pose)
        safe_delta = f"{safe_delta_value:.4f}"
        safe_distance = f"{norm(sub(args.safe_pose[:3], transition_pose[:3])):.4f}"
    else:
        safe_delta_value = 0.0
    worst = worst_collision_summary(collision_checks)
    anchor_delta = ""
    if args.anchor_pose:
        anchor_delta = f"{max_orientation_delta_deg(args.anchor_pose, transition_pose):.4f}"
    clearance_score = max(-1000.0, float(worst["clearance"]))
    score = clearance_score - 0.02 * safe_delta_value
    if args.anchor_pose:
        score -= 0.01 * float(anchor_delta)
    row = {
        "plan_id": f"{args.branch}_{index:03d}",
        "branch": args.branch,
        "sample_in_branch": str(index),
        "physical_ball_id": str(args.physical_ball_id),
        "candidate_source": args.normal_mode,
        "candidate_roll_deg": f"{float(roll_deg):.4f}",
        "candidate_score": f"{score:.6f}",
        "candidate_anchor_orientation_delta_deg": anchor_delta,
        "candidate_safe_transition_distance_mm": safe_distance,
        "approach_x": f"{approach[0]:.6f}",
        "approach_y": f"{approach[1]:.6f}",
        "approach_z": f"{approach[2]:.6f}",
        "sphere_center_x": f"{sphere_center[0]:.6f}",
        "sphere_center_y": f"{sphere_center[1]:.6f}",
        "sphere_center_z": f"{sphere_center[2]:.6f}",
        "contact_center_distance_mm": f"{contact_distance:.6f}",
        "offset_source": args._offset_source,
        "initial_offset_x": f"{offset[0]:.6f}",
        "initial_offset_y": f"{offset[1]:.6f}",
        "initial_offset_z": f"{offset[2]:.6f}",
        "estimated_branch_origin_x": f"{active_origin[0]:.4f}",
        "estimated_branch_origin_y": f"{active_origin[1]:.4f}",
        "estimated_branch_origin_z": f"{active_origin[2]:.4f}",
        "standoff_mm": f"{args.standoff_mm:.4f}",
        "transition_standoff_mm": f"{args.transition_standoff_mm:.4f}",
        "overtravel_mm": f"{args.overtravel_mm:.4f}",
        "probe_travel_mm": f"{args.standoff_mm + args.overtravel_mm:.4f}",
        "safe_transition_orientation_delta_deg": safe_delta,
        "max_safe_transition_orientation_delta_deg": f"{args.max_safe_transition_orientation_delta_deg:.4f}",
        "probe_collision_status": worst["status"],
        "min_probe_clearance_mm": f"{worst['clearance']:.4f}",
        "closest_probe_part": worst["primitive"],
        "closest_probe_model": worst["primitive_model"],
        "closest_obstacle": worst["obstacle"],
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
    if args.safe_pose:
        row.update(pose_dict("safe_flange", args.safe_pose))
    row.update(pose_dict("transition_flange", transition_pose))
    row.update(pose_dict("start_flange", start_pose))
    row.update(pose_dict("contact_flange", contact_pose))
    row.update(pose_dict("target_flange", target_pose))
    return row


def write_csv(path, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def candidate_normals(args, geometry):
    """Candidate normals."""
    if args.normal_mode == "anchor-cone":
        if args.anchor_normal:
            anchor = args.anchor_normal
        elif getattr(args, "_anchor_approach", None) is not None:
            anchor = args._anchor_approach
        elif args.anchor_pose:
            anchor = mat_vec(euler_to_matrix(args.euler_sequence, args.anchor_pose[3:6]), branch_direction(geometry, args.branch))
        else:
            raise ValueError("--normal-mode anchor-cone requires --anchor-pose or --anchor-normal")
        return cone_normals(anchor, args.cone_angle_deg, args.cone_rings, args.cone_samples_per_ring)
    return fibonacci_normals(args.candidate_count)


def filter_normal(args, normal):
    """Filter normal."""
    if normal[2] < args.min_approach_z:
        return False
    if normal[2] > args.max_approach_z:
        return False
    return True


def sort_rows(rows):
    """Sort rows."""
    status_rank = {"OK": 0, "LOW_CLEARANCE": 1, "COLLISION": 2}
    return sorted(
        rows,
        key=lambda row: (
            status_rank.get(row["probe_collision_status"], 9),
            -float(row["candidate_score"]),
            float(row.get("candidate_anchor_orientation_delta_deg") or 0.0),
            float(row.get("candidate_safe_transition_distance_mm") or 0.0),
        ),
    )


def parse_roll_angles(value):
    """Parse roll angles."""
    result = []
    for item in value.split(","):
        item = item.strip()
        if item:
            result.append(float(item))
    if not result:
        raise argparse.ArgumentTypeError("roll angle list cannot be empty")
    return result


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-fit-json", default=str(DEFAULT_REFERENCE_FIT))
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--branch", required=True)
    parser.add_argument("--branch-fit-json", help="JSON containing local_ball_offset_mm for this branch")
    parser.add_argument(
        "--collision-branch-fit",
        nargs="*",
        default=[],
        metavar="BRANCH=JSON",
        help="calibrated offsets used to model all non-active stylus branches",
    )
    parser.add_argument("--branch-offset-mm", nargs=3, type=float, metavar=("PX", "PY", "PZ"))
    parser.add_argument(
        "--orientation-mode",
        choices=("align-branch", "anchor-fixed"),
        default="align-branch",
        help="anchor-fixed keeps flange orientation fixed and varies only sphere contact normals",
    )
    parser.add_argument("--physical-ball-id", default="model_based_probe")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--all-output", help="optional CSV with every generated candidate before top-N filtering")
    parser.add_argument("--plan-count", type=int, default=6)
    parser.add_argument("--normal-mode", choices=("anchor-cone", "fibonacci"), default="anchor-cone")
    parser.add_argument("--anchor-pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--anchor-normal", nargs=3, type=float, metavar=("NX", "NY", "NZ"))
    parser.add_argument("--cone-angle-deg", type=float, default=12.0)
    parser.add_argument("--cone-rings", type=int, default=3)
    parser.add_argument("--cone-samples-per-ring", type=int, default=8)
    parser.add_argument("--candidate-count", type=int, default=240)
    parser.add_argument("--roll-angles-deg", type=parse_roll_angles, default=parse_roll_angles("0,90,180,270"))
    parser.add_argument("--safe-pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float, default=1.0)
    parser.add_argument("--standoff-mm", type=float, default=0.5)
    parser.add_argument("--transition-standoff-mm", type=float, default=6.0)
    parser.add_argument("--overtravel-mm", type=float, default=0.3)
    parser.add_argument("--max-safe-transition-orientation-delta-deg", type=float, default=90.0)
    parser.add_argument("--min-approach-z", type=float, default=-1.0)
    parser.add_argument("--max-approach-z", type=float, default=0.35)
    parser.add_argument("--min-probe-clearance-mm", type=float, default=2.0)
    parser.add_argument("--disable-table-plane-check", action="store_true")
    parser.add_argument("--table-plane-z-mm", type=float, default=0.0)
    parser.add_argument("--min-table-clearance-mm", type=float, default=5.0)
    parser.add_argument("--rod-collision-radius-mm", type=float)
    parser.add_argument("--target-stem-exclusion-mm", type=float, default=3.0)
    parser.add_argument("--collision-segment-samples", type=int, default=9)
    parser.add_argument("--allow-low-clearance", action="store_true")
    parser.add_argument("--allow-collision", action="store_true")
    return parser.parse_args()


def validate_args(args):
    """Validate args."""
    if args.plan_count <= 0:
        raise ValueError("--plan-count must be positive")
    if args.candidate_count <= 0:
        raise ValueError("--candidate-count must be positive")
    if args.cone_rings < 1 or args.cone_samples_per_ring < 1:
        raise ValueError("--cone-rings and --cone-samples-per-ring must be positive")
    if args.sphere_radius_mm <= 0 or args.probe_radius_mm <= 0:
        raise ValueError("sphere/probe radii must be positive")
    if args.standoff_mm <= 0 or args.transition_standoff_mm <= args.standoff_mm:
        raise ValueError("--transition-standoff-mm must be greater than --standoff-mm")
    if args.overtravel_mm <= 0:
        raise ValueError("--overtravel-mm must be positive")
    if args.min_approach_z > args.max_approach_z:
        raise ValueError("--min-approach-z cannot exceed --max-approach-z")
    if args.orientation_mode == "anchor-fixed" and not args.anchor_pose:
        raise ValueError("--orientation-mode anchor-fixed requires --anchor-pose")
    if args.orientation_mode == "anchor-fixed" and any(abs(value) > 1e-9 for value in args.roll_angles_deg):
        raise ValueError("--orientation-mode anchor-fixed requires --roll-angles-deg 0")


def main():
    """Main."""
    args = parse_args()
    try:
        validate_args(args)
        reference_fit, sphere_center = load_reference_sphere(args.reference_fit_json)
        geometry = load_geometry(args.geometry)
        offset, offset_source = load_branch_offset(args, geometry)
        collision_offsets = load_branch_seed_offsets(args.collision_branch_fit)
        collision_offsets[args.branch] = list(offset)
        args._offset_source = offset_source
        args._anchor_approach = None
        if args.orientation_mode == "anchor-fixed" and args.anchor_pose:
            anchor_rotation = euler_to_matrix(args.euler_sequence, args.anchor_pose[3:6])
            anchor_ball_center = add(args.anchor_pose[:3], mat_vec(anchor_rotation, offset))
            args._anchor_approach = normalize(
                sub(sphere_center, anchor_ball_center),
                "anchor ball centre to sphere centre",
            )
        contact_distance = float(reference_fit.get("contact_center_distance_mm", args.sphere_radius_mm + args.probe_radius_mm))
        primitives, dims, active_origin = local_collision_primitives(
            geometry,
            args.branch,
            offset,
            args,
            collision_offsets=collision_offsets,
        )
        normals = [normal for normal in candidate_normals(args, geometry) if filter_normal(args, normal)]
        rows = []
        index = 1
        for normal in normals:
            for roll in args.roll_angles_deg:
                rows.append(
                    plan_candidate(
                        args,
                        geometry,
                        dims,
                        primitives,
                        active_origin,
                        sphere_center,
                        contact_distance,
                        offset,
                        normal,
                        roll,
                        index,
                    )
                )
                index += 1
        if not rows:
            raise ValueError("no candidates generated after normal filtering")
        rows = sort_rows(rows)
        if not args.allow_collision:
            rows = [row for row in rows if row["probe_collision_status"] != "COLLISION"]
        if not args.allow_low_clearance:
            rows = [row for row in rows if row["probe_collision_status"] == "OK"]
        if not rows:
            raise ValueError("no candidates left after collision filtering")
        selected = rows[: args.plan_count]
        write_csv(args.output, selected)
        if args.all_output:
            write_csv(args.all_output, rows)
        print(f"generated candidates: {index - 1}")
        print(f"selected plan rows: {len(selected)}")
        print(f"saved plan: {args.output}")
        for row in selected:
            print(
                f"  {row['plan_id']} status={row['probe_collision_status']} "
                f"clearance={row['min_probe_clearance_mm']}mm "
                f"approach=[{row['approach_x']}, {row['approach_y']}, {row['approach_z']}] "
                f"start=[{row['start_flange_x']}, {row['start_flange_y']}, {row['start_flange_z']}, "
                f"{row['start_flange_rx']}, {row['start_flange_ry']}, {row['start_flange_rz']}]"
            )
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
