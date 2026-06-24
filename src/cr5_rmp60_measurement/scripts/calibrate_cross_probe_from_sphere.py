#!/usr/bin/env python3
"""Estimate relative cross-stylus branch offsets from a calibration sphere.

This fits the four horizontal branch offsets to a common sphere centre using
trigger rows collected around a known-diameter calibration sphere. With all
rows taken at nearly the same flange orientation, the result is a relative
four-direction consistency calibration, not an absolute TCP calibration.
"""
import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

from cross_probe_model import (
    DEFAULT_GEOMETRY,
    ball_radius_mm,
    branch_local_offset_mm,
    compute_branch_point,
    euler_to_matrix,
    load_geometry,
    mat_vec,
)
from geometry_utils import add, normalize, scale


POSE_FIELDS = ("flange_x", "flange_y", "flange_z", "rx", "ry", "rz")
APPROACH_FIELDS = ("approach_x", "approach_y", "approach_z")


def parse_float(row, key, row_number):
    """Parse a string to float, returning a default for empty/missing values."""
    value = row.get(key, "")
    if value in (None, ""):
        raise ValueError(f"row {row_number}: missing {key}")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"row {row_number}: {key} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"row {row_number}: {key} must be finite")
    return result


def read_rows(paths):
    """Read rows."""
    rows = []
    row_number = 1
    for path in paths:
        with Path(path).open(newline="") as f:
            for local_index, row in enumerate(csv.DictReader(f), start=1):
                row = dict(row)
                row["_source_csv"] = str(path)
                row["_source_row"] = str(local_index)
                row["_global_row"] = str(row_number)
                rows.append(row)
                row_number += 1
    return rows


def write_rows(path, rows):
    """Write rows."""
    if not rows:
        raise ValueError("no rows to write")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def pose_from_row(row, row_number):
    """Extract a 6-element flange pose list from a CSV row dict."""
    return [parse_float(row, field, row_number) for field in POSE_FIELDS]


def approach_from_row(row, row_number):
    """Approach from row."""
    return normalize([parse_float(row, field, row_number) for field in APPROACH_FIELDS], f"row {row_number}: approach")


def transpose_mat_vec(matrix, vector):
    """Transpose mat vec."""
    return [sum(matrix[row][col] * vector[row] for row in range(3)) for col in range(3)]


def subtract(a, b):
    """Subtract."""
    return [a[index] - b[index] for index in range(3)]


def mean_vector(vectors):
    """Mean vector."""
    return [statistics.fmean(vector[index] for vector in vectors) for index in range(3)]


def sample_std(values):
    """Sample std."""
    if len(values) < 2:
        return 0.0
    return statistics.stdev(values)


def vector_norm(vector):
    """Vector norm."""
    return math.sqrt(sum(value * value for value in vector))


def axis_stats(vectors):
    """Axis stats."""
    if not vectors:
        return {}
    return {
        "mean_mm": mean_vector(vectors),
        "sample_std_mm": [sample_std([vector[index] for vector in vectors]) for index in range(3)],
        "min_mm": [min(vector[index] for vector in vectors) for index in range(3)],
        "max_mm": [max(vector[index] for vector in vectors) for index in range(3)],
    }


def row_to_observation(row, geometry, probe_radius, sphere_radius, euler_sequence):
    """Row to observation."""
    row_number = int(row["_global_row"])
    pose = pose_from_row(row, row_number)
    branch = row.get("branch")
    if not branch:
        raise ValueError(f"row {row_number}: missing branch")
    approach = approach_from_row(row, row_number)
    branch_result = compute_branch_point(geometry, pose, branch, euler_sequence=euler_sequence, approach=approach)
    nominal_ball_center = branch_result["ball_center_mm"]
    nominal_sphere_center = add(nominal_ball_center, scale(approach, sphere_radius + probe_radius))
    rotation = euler_to_matrix(euler_sequence, pose[3:6])
    return {
        "source_csv": row["_source_csv"],
        "source_row": int(row["_source_row"]),
        "global_row": row_number,
        "timestamp": row.get("timestamp", ""),
        "session_id": row.get("session_id", ""),
        "setup_id": row.get("setup_id", ""),
        "workpiece_id": row.get("workpiece_id", ""),
        "face_id": row.get("face_id", ""),
        "branch": branch,
        "standard_pose": row.get("standard_pose", ""),
        "operator_note": row.get("operator_note", ""),
        "pose": pose,
        "approach": approach,
        "rotation": rotation,
        "nominal_local_offset_mm": branch_result["local_ball_center_offset_mm"],
        "nominal_ball_center_mm": nominal_ball_center,
        "nominal_sphere_center_mm": nominal_sphere_center,
        "stop_overtravel_along_approach_mm": row.get("stop_overtravel_along_approach_mm", ""),
    }


def make_combined_row(observation, common_center, branch_delta):
    """Make combined row."""
    corrected_sphere_center = add(observation["nominal_sphere_center_mm"], mat_vec(observation["rotation"], branch_delta))
    residual = subtract(corrected_sphere_center, common_center)
    pose = observation["pose"]
    approach = observation["approach"]
    nominal_ball = observation["nominal_ball_center_mm"]
    nominal_center = observation["nominal_sphere_center_mm"]
    return {
        "source_csv": observation["source_csv"],
        "source_row": observation["source_row"],
        "global_row": observation["global_row"],
        "timestamp": observation["timestamp"],
        "session_id": observation["session_id"],
        "setup_id": observation["setup_id"],
        "workpiece_id": observation["workpiece_id"],
        "face_id": observation["face_id"],
        "branch": observation["branch"],
        "standard_pose": observation["standard_pose"],
        "flange_x": f"{pose[0]:.4f}",
        "flange_y": f"{pose[1]:.4f}",
        "flange_z": f"{pose[2]:.4f}",
        "rx": f"{pose[3]:.4f}",
        "ry": f"{pose[4]:.4f}",
        "rz": f"{pose[5]:.4f}",
        "approach_x": f"{approach[0]:.6f}",
        "approach_y": f"{approach[1]:.6f}",
        "approach_z": f"{approach[2]:.6f}",
        "nominal_ball_center_x": f"{nominal_ball[0]:.4f}",
        "nominal_ball_center_y": f"{nominal_ball[1]:.4f}",
        "nominal_ball_center_z": f"{nominal_ball[2]:.4f}",
        "nominal_sphere_center_x": f"{nominal_center[0]:.4f}",
        "nominal_sphere_center_y": f"{nominal_center[1]:.4f}",
        "nominal_sphere_center_z": f"{nominal_center[2]:.4f}",
        "branch_delta_local_x": f"{branch_delta[0]:.6f}",
        "branch_delta_local_y": f"{branch_delta[1]:.6f}",
        "branch_delta_local_z": f"{branch_delta[2]:.6f}",
        "corrected_sphere_center_x": f"{corrected_sphere_center[0]:.4f}",
        "corrected_sphere_center_y": f"{corrected_sphere_center[1]:.4f}",
        "corrected_sphere_center_z": f"{corrected_sphere_center[2]:.4f}",
        "residual_x": f"{residual[0]:.6f}",
        "residual_y": f"{residual[1]:.6f}",
        "residual_z": f"{residual[2]:.6f}",
        "residual_norm_mm": f"{vector_norm(residual):.6f}",
        "stop_overtravel_along_approach_mm": observation["stop_overtravel_along_approach_mm"],
        "operator_note": observation["operator_note"],
    }


def calibrate(observations, geometry, probe_radius, sphere_radius, euler_sequence):
    """Calibrate."""
    if not observations:
        raise ValueError("no observations")
    common_center = mean_vector([observation["nominal_sphere_center_mm"] for observation in observations])
    by_branch = {}
    for observation in observations:
        by_branch.setdefault(observation["branch"], []).append(observation)

    branches = []
    combined_rows = []
    for branch in sorted(by_branch):
        branch_observations = by_branch[branch]
        delta_observations = []
        for observation in branch_observations:
            correction_base = subtract(common_center, observation["nominal_sphere_center_mm"])
            delta_observations.append(transpose_mat_vec(observation["rotation"], correction_base))
        delta_mean = mean_vector(delta_observations)
        nominal_offset = branch_local_offset_mm(geometry, branch)
        estimated_offset = add(nominal_offset, delta_mean)

        residuals = []
        for observation in branch_observations:
            row = make_combined_row(observation, common_center, delta_mean)
            combined_rows.append(row)
            residuals.append(float(row["residual_norm_mm"]))

        branches.append(
            {
                "branch": branch,
                "ok": True,
                "method": "relative_common_sphere_center",
                "row_numbers": [observation["global_row"] for observation in branch_observations],
                "rows": len(branch_observations),
                "nominal_offset_mm": nominal_offset,
                "estimated_offset_mm": estimated_offset,
                "delta_from_nominal_mm": delta_mean,
                "delta_observation_stats": axis_stats(delta_observations),
                "nominal_sphere_center_stats": axis_stats(
                    [observation["nominal_sphere_center_mm"] for observation in branch_observations]
                ),
                "rms_all_constraints_mm": math.sqrt(statistics.fmean(value * value for value in residuals)),
                "max_abs_all_constraints_mm": max(residuals),
                "condition": None,
            }
        )

    all_residuals = [float(row["residual_norm_mm"]) for row in combined_rows]
    return {
        "timestamp": time.time(),
        "method": "relative_four_direction_sphere_fit",
        "method_warning": (
            "Rows were collected at nearly the same flange orientation. The common sphere centre and branch offsets "
            "are gauge-coupled; this output is a relative four-direction consistency calibration, not an absolute TCP."
        ),
        "source_csv": sorted({observation["source_csv"] for observation in observations}),
        "geometry": str(DEFAULT_GEOMETRY),
        "euler_sequence": euler_sequence,
        "ball_radius_mm": probe_radius,
        "calibration_sphere_radius_mm": sphere_radius,
        "common_sphere_center_mm": common_center,
        "nominal_sphere_center_stats": axis_stats([observation["nominal_sphere_center_mm"] for observation in observations]),
        "overall_rms_residual_mm": math.sqrt(statistics.fmean(value * value for value in all_residuals)),
        "overall_max_residual_mm": max(all_residuals),
        "branches": branches,
    }, combined_rows


def print_summary(result):
    """Print summary."""
    center = result["common_sphere_center_mm"]
    print("cross probe calibration sphere fit")
    print(f"  method: {result['method']}")
    print(f"  common_sphere_center_mm: [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]")
    print(f"  probe_ball_radius_mm: {result['ball_radius_mm']:.4f}")
    print(f"  calibration_sphere_radius_mm: {result['calibration_sphere_radius_mm']:.4f}")
    print(f"  overall_rms_residual_mm: {result['overall_rms_residual_mm']:.6f}")
    print(f"  overall_max_residual_mm: {result['overall_max_residual_mm']:.6f}")
    print("  warning: relative four-direction fit; collect varied orientations for absolute TCP calibration")
    for branch in result["branches"]:
        estimate = branch["estimated_offset_mm"]
        delta = branch["delta_from_nominal_mm"]
        print(f"  [PASS] {branch['branch']} rows={branch['rows']}")
        print(f"    estimated_offset_mm: [{estimate[0]:.4f}, {estimate[1]:.4f}, {estimate[2]:.4f}]")
        print(f"    delta_from_nominal_mm: [{delta[0]:.4f}, {delta[1]:.4f}, {delta[2]:.4f}]")
        print(f"    rms_residual_mm: {branch['rms_all_constraints_mm']:.6f}")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", nargs="+", required=True, help="repeat contact CSV files")
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--json-output", help="write calibration result JSON")
    parser.add_argument("--combined-output", help="write combined observation/residual CSV")
    args = parser.parse_args()

    if args.sphere_radius_mm <= 0:
        raise SystemExit("--sphere-radius-mm must be positive")

    try:
        geometry = load_geometry(args.geometry)
        probe_radius = ball_radius_mm(geometry)
        rows = read_rows(args.input)
        observations = [
            row_to_observation(row, geometry, probe_radius, args.sphere_radius_mm, args.euler_sequence)
            for row in rows
        ]
        result, combined_rows = calibrate(observations, geometry, probe_radius, args.sphere_radius_mm, args.euler_sequence)
        result["geometry"] = str(args.geometry)
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc

    print_summary(result)
    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(f"saved: {output}")
    if args.combined_output:
        write_rows(args.combined_output, combined_rows)
        print(f"saved: {args.combined_output}")


if __name__ == "__main__":
    main()
