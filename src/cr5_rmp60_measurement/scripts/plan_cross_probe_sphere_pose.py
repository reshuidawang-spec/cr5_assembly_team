#!/usr/bin/env python3
"""Plan a cross-stylus standard-sphere touch for an arbitrary flange attitude.

The planner is deliberately offline-only. It uses the calibrated branch ball
offset to plan the path of the selected ball centre against the calibration
sphere, then derives the corresponding flange poses. This avoids treating a
rotated flange pose as "just move X/Y", which can select the wrong physical
contact geometry.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path

import yaml

from cross_probe_model import (
    DEFAULT_GEOMETRY,
    ball_radius_mm,
    branch_map,
    euler_to_matrix,
    load_geometry,
    mat_vec,
)
from geometry_utils import add, normalize, scale


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = PROJECT_DIR / "data/cross_probe_sphere_calibration_low_overtravel_20260601.json"


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        """Ignore aliases."""
        return True


def subtract(a, b):
    """Subtract."""
    return [x - y for x, y in zip(a, b)]


def dot(a, b):
    """Dot."""
    return sum(x * y for x, y in zip(a, b))


def norm(vector):
    """Norm."""
    return math.sqrt(dot(vector, vector))


def distance(a, b):
    """Distance."""
    return norm(subtract(a, b))


def clamp(value, low=-1.0, high=1.0):
    """Clamp."""
    return max(low, min(high, value))


def angle_deg(a, b):
    """Angle deg."""
    return math.degrees(math.acos(clamp(dot(normalize(a), normalize(b)))))


def pose(position, orientation):
    """Pose."""
    return [float(position[0]), float(position[1]), float(position[2]), *[float(v) for v in orientation]]


def load_calibration(path):
    """Load calibration."""
    with Path(path).open() as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"invalid calibration JSON: {path}")
    return data


def branch_calibration(calibration, branch_name):
    """Branch calibration."""
    for branch in calibration.get("branches", []):
        if branch.get("branch") == branch_name:
            offset = branch.get("estimated_offset_mm")
            if not isinstance(offset, list) or len(offset) != 3:
                raise ValueError(f"branch {branch_name!r} has invalid estimated_offset_mm")
            return [float(v) for v in offset]
    choices = sorted(branch.get("branch", "") for branch in calibration.get("branches", []))
    raise ValueError(f"branch {branch_name!r} not found in calibration; choices: {', '.join(choices)}")


def branch_nominal_direction(geometry, branch_name):
    """Branch nominal direction."""
    branches = branch_map(geometry)
    if branch_name not in branches:
        raise ValueError(f"unknown branch {branch_name!r}; choices: {', '.join(sorted(branches))}")
    direction = branches[branch_name].get("direction")
    if not isinstance(direction, list) or len(direction) != 3:
        raise ValueError(f"branch {branch_name!r} has invalid direction")
    return normalize([float(v) for v in direction], f"{branch_name} direction")


def mm3(values):
    """Mm3."""
    return [round(float(v), 4) for v in values]


def pose6(values):
    """Pose6."""
    return [round(float(v), 4) for v in values]


def ball_center_for(flange_pose, offset_base):
    """Ball center for."""
    return add([float(v) for v in flange_pose[:3]], offset_base)


def transpose_mat_vec(matrix, vector):
    """Transpose mat vec."""
    return [sum(matrix[row][col] * vector[row] for row in range(3)) for col in range(3)]


def mean_vector(vectors):
    """Mean vector."""
    return [statistics.fmean(vector[index] for vector in vectors) for index in range(3)]


def sample_std(values):
    """Sample std."""
    if len(values) < 2:
        return 0.0
    return statistics.stdev(values)


def vector_stats(vectors):
    """Vector stats."""
    if not vectors:
        return {}
    return {
        "mean_mm": mm3(mean_vector(vectors)),
        "sample_std_mm": mm3([sample_std([vector[index] for vector in vectors]) for index in range(3)]),
        "min_mm": mm3([min(vector[index] for vector in vectors) for index in range(3)]),
        "max_mm": mm3([max(vector[index] for vector in vectors) for index in range(3)]),
    }


def parse_row_float(row, field, source):
    """Parse row float."""
    value = row.get(field, "")
    if value in (None, ""):
        raise ValueError(f"{source}: missing {field}")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{source}: {field} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{source}: {field} must be finite")
    return result


def read_anchor_rows(paths, branch_name):
    """Read anchor rows."""
    rows = []
    for path in paths or []:
        with Path(path).open(newline="") as f:
            for index, row in enumerate(csv.DictReader(f), start=1):
                if row.get("branch") == branch_name:
                    row = dict(row)
                    row["_source_csv"] = str(path)
                    row["_source_row"] = index
                    rows.append(row)
    if not rows:
        raise ValueError(f"no anchor rows found for branch {branch_name!r}")
    return rows


def pose_from_contact_row(row):
    """Pose from contact row."""
    source = f"{row['_source_csv']} row {row['_source_row']}"
    fields = ("flange_x", "flange_y", "flange_z", "rx", "ry", "rz")
    return [parse_row_float(row, field, source) for field in fields]


def approach_from_contact_row(row):
    """Approach from contact row."""
    source = f"{row['_source_csv']} row {row['_source_row']}"
    fields = ("approach_x", "approach_y", "approach_z")
    return normalize([parse_row_float(row, field, source) for field in fields], f"{source} approach")


def anchored_local_offset(anchor_rows, sphere_center, contact_center_distance, euler_sequence):
    """Anchored local offset."""
    offsets = []
    trigger_ball_centers = []
    for row in anchor_rows:
        trigger_pose = pose_from_contact_row(row)
        approach = approach_from_contact_row(row)
        rotation = euler_to_matrix(euler_sequence, trigger_pose[3:6])
        trigger_ball_center = add(sphere_center, scale(approach, -contact_center_distance))
        local_offset = transpose_mat_vec(rotation, subtract(trigger_ball_center, trigger_pose[:3]))
        offsets.append(local_offset)
        trigger_ball_centers.append(trigger_ball_center)
    return mean_vector(offsets), {
        "rows": len(anchor_rows),
        "source_csv": sorted({row["_source_csv"] for row in anchor_rows}),
        "source_rows": [row["_source_row"] for row in anchor_rows],
        "local_offset_stats_mm": vector_stats(offsets),
        "trigger_ball_center_stats_mm": vector_stats(trigger_ball_centers),
    }


def make_yaml_snippets(args, plan):
    """Make yaml snippets."""
    if not args.plan_id:
        return {}
    standard_pose = {
        args.plan_id: {
            "description": (
                "Planned cross-stylus standard-sphere multi-attitude touch. "
                "Generated from calibrated branch ball-centre path; not operator approved."
            ),
            "status": "planned",
            "branch": args.branch,
            "approach": plan["approach_unit_base"],
            "pose": plan["safe_flange_pose_mm_deg"],
            "source": f"generated by {Path(__file__).name}; verify clearance before execution",
        }
    }
    snippets = {"measurement_pose": {"cross_probe": {"standard_poses": standard_pose}}}
    if args.setup_id and args.face_id:
        snippets["workpiece_face"] = {
            "setups": {
                args.setup_id: {
                    "workpiece_id": args.workpiece_id or "calibration_sphere_20mm",
                    "faces": {
                        args.face_id: {
                            "status": "planned",
                            "branch": args.branch,
                            "standard_pose": args.plan_id,
                            "approach": plan["approach_unit_base"],
                            "safe_start_pose": plan["safe_flange_pose_mm_deg"],
                            "max_search_mm": plan["target_travel_from_start_mm"],
                            "retract_mm": args.retract_mm,
                            "max_speed": args.max_speed,
                            "validation_result": (
                                "planned only; path is derived from the selected ball-centre line "
                                "against the calibration sphere"
                            ),
                        }
                    },
                }
            }
        }
    return snippets


def plan_sphere_touch(args):
    """Plan sphere touch."""
    geometry = load_geometry(args.geometry)
    calibration = load_calibration(args.calibration)
    euler_sequence = args.euler_sequence or calibration.get("euler_sequence", "xyz")

    sphere_center = args.sphere_center
    if sphere_center is None:
        sphere_center = calibration.get("common_sphere_center_mm")
    if not isinstance(sphere_center, list) or len(sphere_center) != 3:
        raise ValueError("provide --sphere-center or use calibration JSON with common_sphere_center_mm")
    sphere_center = [float(v) for v in sphere_center]

    sphere_radius = args.sphere_radius_mm
    if sphere_radius is None:
        sphere_radius = float(calibration.get("calibration_sphere_radius_mm", 0.0))
    if sphere_radius <= 0:
        raise ValueError("--sphere-radius-mm must be positive")

    probe_radius = args.probe_radius_mm
    if probe_radius is None:
        probe_radius = float(calibration.get("ball_radius_mm", ball_radius_mm(geometry)))
    if probe_radius <= 0:
        raise ValueError("--probe-radius-mm must be positive")
    if args.standoff_mm <= 0 or args.search_mm <= 0 or args.retract_mm < 0:
        raise ValueError("--standoff-mm and --search-mm must be positive; --retract-mm cannot be negative")

    if args.rotated_start_pose is not None:
        rotated_start_pose = [float(v) for v in args.rotated_start_pose]
        orientation = [float(v) for v in (args.orientation or rotated_start_pose[3:6])]
    else:
        rotated_start_pose = None
        if args.orientation is None:
            raise ValueError("provide --orientation, or provide --rotated-start-pose for anchored planning")
        orientation = [float(v) for v in args.orientation]
    rotation = euler_to_matrix(euler_sequence, orientation)
    contact_center_distance = sphere_radius + probe_radius
    anchor_context = None
    if args.anchor_csv:
        anchor_rows = read_anchor_rows(args.anchor_csv, args.branch)
        local_offset, anchor_context = anchored_local_offset(
            anchor_rows,
            sphere_center,
            contact_center_distance,
            euler_sequence,
        )
    else:
        local_offset = branch_calibration(calibration, args.branch)
    offset_base = mat_vec(rotation, local_offset)
    branch_axis_local = branch_nominal_direction(geometry, args.branch)
    branch_axis_base = normalize(mat_vec(rotation, branch_axis_local), f"{args.branch} branch axis in base")
    if rotated_start_pose is not None and args.anchor_csv:
        if args.approach or args.approach_from_branch_axis:
            raise ValueError("anchored --rotated-start-pose planning derives approach from current ruby ball centre to sphere centre")
        current_ball_center = ball_center_for(rotated_start_pose, offset_base)
        center_vector = subtract(sphere_center, current_ball_center)
        center_distance = norm(center_vector)
        if center_distance <= contact_center_distance:
            raise ValueError(
                "rotated start ball centre is already at or inside the contact sphere; "
                "move to a safer start before planning"
            )
        approach = normalize(center_vector, "current ruby ball centre to sphere centre")
        distance_to_trigger = center_distance - contact_center_distance
        target_travel = distance_to_trigger + args.past_contact_mm
        trigger_ball_center = add(current_ball_center, scale(approach, distance_to_trigger))
        trigger_position = add(rotated_start_pose[:3], scale(approach, distance_to_trigger))
        safe_position = rotated_start_pose[:3]
        target_position = add(rotated_start_pose[:3], scale(approach, target_travel))
        retract_position = add(trigger_position, scale(approach, -args.retract_mm))
        safe_pose = pose(safe_position, orientation)
        trigger_pose = pose(trigger_position, orientation)
        target_pose = pose(target_position, orientation)
        retract_pose = pose(retract_position, orientation)
        penetration = args.past_contact_mm
        approach_source = "anchored_current_ball_to_sphere_center"
    elif args.approach_from_branch_axis and args.approach:
        raise ValueError("use only one of --approach or --approach-from-branch-axis")
    else:
        if args.approach_from_branch_axis:
            multiplier = 1.0 if args.approach_from_branch_axis == "same" else -1.0
            approach = scale(branch_axis_base, multiplier)
        elif args.approach:
            approach = normalize([float(v) for v in args.approach], "approach")
        else:
            raise ValueError("provide --approach or --approach-from-branch-axis")
        trigger_ball_center = add(sphere_center, scale(approach, -contact_center_distance))
        trigger_position = subtract(trigger_ball_center, offset_base)
        safe_position = add(trigger_position, scale(approach, -args.standoff_mm))
        target_position = add(safe_position, scale(approach, args.search_mm))
        retract_position = add(trigger_position, scale(approach, -args.retract_mm))
        safe_pose = pose(safe_position, orientation)
        trigger_pose = pose(trigger_position, orientation)
        target_pose = pose(target_position, orientation)
        retract_pose = pose(retract_position, orientation)
        penetration = args.search_mm - args.standoff_mm
        approach_source = "branch_axis_" + args.approach_from_branch_axis if args.approach_from_branch_axis else "manual"
        distance_to_trigger = args.standoff_mm
        target_travel = args.search_mm
        current_ball_center = ball_center_for(safe_pose, offset_base)

    angle_to_motion = angle_deg(branch_axis_base, approach)
    angle_to_reverse_motion = angle_deg(branch_axis_base, scale(approach, -1.0))
    best_axis_angle = min(angle_to_motion, angle_to_reverse_motion)

    path_points = {}
    for label, flange_pose in (
        ("safe", safe_pose),
        ("trigger", trigger_pose),
        ("target", target_pose),
        ("retract", retract_pose),
    ):
        center = ball_center_for(flange_pose, offset_base)
        clearance = distance(center, sphere_center) - contact_center_distance
        signed_to_trigger = dot(subtract(center, trigger_ball_center), scale(approach, -1.0))
        path_points[label] = {
            "flange_pose_mm_deg": pose6(flange_pose),
            "ball_center_mm": mm3(center),
            "sphere_clearance_mm": round(clearance, 4),
            "signed_distance_before_trigger_mm": round(signed_to_trigger, 4),
        }

    warnings = []
    target_clearance = path_points["target"]["signed_distance_before_trigger_mm"]
    if target_clearance >= 0:
        warnings.append("search target does not pass the theoretical trigger point; increase --search-mm or reduce --standoff-mm")
    if args.anchor_csv and rotated_start_pose is not None and target_travel > args.long_search_warning_mm:
        warnings.append(
            f"anchored search distance is {target_travel:.3f}mm; this is much longer than fixed-pose probing"
        )
    if penetration > 1.5:
        warnings.append(
            f"search endpoint is {penetration:.3f}mm past theoretical contact; keep low speed and segmented probing"
        )
    if best_axis_angle > args.max_axis_angle_deg:
        warnings.append(
            f"selected branch axis is {best_axis_angle:.2f}deg from the motion line; verify the chosen branch cannot collide"
        )

    result = {
        "planner": Path(__file__).name,
        "status": "planned_only",
        "branch": args.branch,
        "calibration": str(args.calibration),
        "geometry": str(args.geometry),
        "euler_sequence": euler_sequence,
        "sphere_center_mm": mm3(sphere_center),
        "sphere_radius_mm": round(sphere_radius, 4),
        "probe_radius_mm": round(probe_radius, 4),
        "contact_center_distance_mm": round(contact_center_distance, 4),
        "orientation_rx_ry_rz_deg": pose6(orientation),
        "approach_source": approach_source,
        "approach_unit_base": [round(v, 6) for v in approach],
        "calibrated_local_ball_offset_mm": mm3(local_offset),
        "calibrated_base_ball_offset_mm": mm3(offset_base),
        "anchor_context": anchor_context,
        "rotated_start_flange_pose_mm_deg": pose6(rotated_start_pose) if rotated_start_pose is not None else None,
        "rotated_start_ball_center_mm": mm3(current_ball_center),
        "rotated_start_ball_to_sphere_center_mm": round(distance(current_ball_center, sphere_center), 4),
        "distance_to_theoretical_trigger_mm": round(distance_to_trigger, 4),
        "target_travel_from_start_mm": round(target_travel, 4),
        "branch_axis_base": [round(v, 6) for v in branch_axis_base],
        "branch_axis_angle_to_motion_deg": round(angle_to_motion, 3),
        "branch_axis_angle_to_reverse_motion_deg": round(angle_to_reverse_motion, 3),
        "branch_axis_min_angle_to_motion_line_deg": round(best_axis_angle, 3),
        "standoff_mm": round(args.standoff_mm, 4),
        "search_mm": round(args.search_mm, 4),
        "planned_endpoint_past_contact_mm": round(penetration, 4),
        "retract_mm": round(args.retract_mm, 4),
        "safe_flange_pose_mm_deg": pose6(safe_pose),
        "trigger_flange_pose_mm_deg": pose6(trigger_pose),
        "target_flange_pose_mm_deg": pose6(target_pose),
        "retract_flange_pose_mm_deg": pose6(retract_pose),
        "path_points": path_points,
        "warnings": warnings,
        "yaml_snippets": {},
    }
    result["yaml_snippets"] = make_yaml_snippets(args, result)
    return result


def print_summary(result):
    """Print summary."""
    print("cross-probe standard-sphere pose plan")
    print(f"  status: {result['status']}")
    print(f"  branch: {result['branch']}")
    print(f"  sphere_center_mm: {result['sphere_center_mm']}")
    print(f"  orientation_rx_ry_rz_deg: {result['orientation_rx_ry_rz_deg']}")
    if result.get("rotated_start_flange_pose_mm_deg") is not None:
        print(f"  rotated_start_flange_pose_mm_deg: {result['rotated_start_flange_pose_mm_deg']}")
        print(f"  rotated_start_ball_center_mm: {result['rotated_start_ball_center_mm']}")
        print(f"  rotated_start_ball_to_sphere_center_mm: {result['rotated_start_ball_to_sphere_center_mm']:.4f}")
        print(f"  distance_to_theoretical_trigger_mm: {result['distance_to_theoretical_trigger_mm']:.4f}")
        print(f"  target_travel_from_start_mm: {result['target_travel_from_start_mm']:.4f}")
    print(f"  approach_unit_base: {result['approach_unit_base']}")
    print(f"  branch_axis_base: {result['branch_axis_base']}")
    print(
        "  branch_axis_min_angle_to_motion_line_deg: "
        f"{result['branch_axis_min_angle_to_motion_line_deg']:.3f}"
    )
    print(f"  safe_flange_pose_mm_deg: {result['safe_flange_pose_mm_deg']}")
    print(f"  trigger_flange_pose_mm_deg: {result['trigger_flange_pose_mm_deg']}")
    print(f"  target_flange_pose_mm_deg: {result['target_flange_pose_mm_deg']}")
    print(f"  retract_flange_pose_mm_deg: {result['retract_flange_pose_mm_deg']}")
    print("  ball-centre path:")
    for label in ("safe", "trigger", "target", "retract"):
        point = result["path_points"][label]
        print(
            f"    {label}: ball={point['ball_center_mm']} "
            f"clearance={point['sphere_clearance_mm']:.4f}mm "
            f"signed_before_trigger={point['signed_distance_before_trigger_mm']:.4f}mm"
        )
    if result["warnings"]:
        print("  warnings:")
        for warning in result["warnings"]:
            print(f"    - {warning}")
    else:
        print("  warnings: none")
    print("  note: this script does not execute robot motion")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--branch", required=True)
    parser.add_argument("--orientation", nargs=3, type=float, metavar=("RX", "RY", "RZ"))
    parser.add_argument(
        "--anchor-csv",
        nargs="+",
        help=(
            "real trigger CSV rows for this branch; derive the ruby-ball local offset "
            "from previously touched standard-sphere poses"
        ),
    )
    parser.add_argument(
        "--rotated-start-pose",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="rotated flange start pose; with --anchor-csv, approach is current ruby ball centre to sphere centre",
    )
    parser.add_argument("--approach", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument(
        "--approach-from-branch-axis",
        choices=("same", "opposite"),
        help=(
            "derive the ball-centre approach from the selected branch axis in base frame; "
            "'same' uses R*branch_direction, 'opposite' uses the reverse"
        ),
    )
    parser.add_argument("--sphere-center", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--sphere-radius-mm", type=float)
    parser.add_argument("--probe-radius-mm", type=float)
    parser.add_argument("--euler-sequence")
    parser.add_argument("--standoff-mm", type=float, default=3.45)
    parser.add_argument("--search-mm", type=float, default=5.0)
    parser.add_argument("--past-contact-mm", type=float, default=1.0)
    parser.add_argument("--retract-mm", type=float, default=2.0)
    parser.add_argument("--long-search-warning-mm", type=float, default=10.0)
    parser.add_argument("--max-speed", type=float, default=1.0)
    parser.add_argument("--max-axis-angle-deg", type=float, default=20.0)
    parser.add_argument("--plan-id", help="include planned measurement_poses YAML snippet")
    parser.add_argument("--setup-id", help="include planned workpiece setup snippet")
    parser.add_argument("--workpiece-id")
    parser.add_argument("--face-id")
    parser.add_argument("--json-output")
    parser.add_argument("--yaml-output", help="write YAML snippets only")
    args = parser.parse_args()

    try:
        result = plan_sphere_touch(args)
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc

    print_summary(result)
    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(f"saved: {output}")
    if args.yaml_output:
        output = Path(args.yaml_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.dump(result["yaml_snippets"], Dumper=NoAliasDumper, sort_keys=False, allow_unicode=False))
        print(f"saved: {output}")


if __name__ == "__main__":
    main()
