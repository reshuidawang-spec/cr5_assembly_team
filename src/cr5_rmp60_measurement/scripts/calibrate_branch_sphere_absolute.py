#!/usr/bin/env python3
"""Fit one physical cross-stylus ball and a calibration sphere centre.

For every trigger row, the selected ruby ball centre at trigger satisfies:

    flange_position + R(flange_orientation) * local_ball_offset
        = sphere_center - approach * (sphere_radius + probe_radius)

The script solves this linear least-squares system for one branch:

    [I, -R] [sphere_center, local_ball_offset] = flange + approach * distance

At least two sufficiently different flange orientations are required for a
well-conditioned absolute estimate. A single fixed orientation remains
gauge-coupled and is not an absolute TCP calibration.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np

from cross_probe_model import DEFAULT_GEOMETRY, ball_radius_mm, euler_to_matrix, load_geometry


POSE_FIELDS = ("flange_x", "flange_y", "flange_z", "rx", "ry", "rz")
APPROACH_FIELDS = ("approach_x", "approach_y", "approach_z")


def parse_float(row, field, label):
    """Parse a string to float, returning a default for empty/missing values."""
    value = row.get(field, "")
    if value in (None, ""):
        raise ValueError(f"{label}: missing {field}")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}: {field} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label}: {field} must be finite")
    return result


def normalize(vector, label):
    """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-12:
        raise ValueError(f"{label} cannot be zero")
    return [value / length for value in vector]


def read_rows(paths, branch):
    """Read rows."""
    rows = []
    for path in paths:
        with Path(path).open(newline="") as f:
            for row_index, row in enumerate(csv.DictReader(f), start=1):
                if row.get("branch") != branch:
                    continue
                row = dict(row)
                row["_source_csv"] = str(path)
                row["_source_row"] = row_index
                rows.append(row)
    if not rows:
        raise ValueError(f"no rows found for branch {branch!r}")
    return rows


def pose_from_row(row):
    """Extract a 6-element flange pose list from a CSV row dict."""
    label = f"{row['_source_csv']} row {row['_source_row']}"
    return [parse_float(row, field, label) for field in POSE_FIELDS]


def approach_from_row(row):
    """Approach from row."""
    label = f"{row['_source_csv']} row {row['_source_row']}"
    return normalize([parse_float(row, field, label) for field in APPROACH_FIELDS], f"{label} approach")


def vector_norm(vector):
    """Vector norm."""
    return float(np.linalg.norm(np.asarray(vector, dtype=float)))


def solve(rows, sphere_radius, probe_radius, euler_sequence):
    """Solve."""
    contact_center_distance = sphere_radius + probe_radius
    a_rows = []
    b_rows = []
    observations = []

    for row in rows:
        pose = pose_from_row(row)
        flange = np.asarray(pose[:3], dtype=float)
        rotation = np.asarray(euler_to_matrix(euler_sequence, pose[3:6]), dtype=float)
        approach = np.asarray(approach_from_row(row), dtype=float)
        rhs = flange + approach * contact_center_distance
        block = np.zeros((3, 6), dtype=float)
        block[:, 0:3] = np.eye(3)
        block[:, 3:6] = -rotation
        a_rows.append(block)
        b_rows.append(rhs)
        observations.append(
            {
                "source_csv": row["_source_csv"],
                "source_row": row["_source_row"],
                "timestamp": row.get("timestamp", ""),
                "standard_pose": row.get("standard_pose", ""),
                "face_id": row.get("face_id", ""),
                "pose": pose,
                "approach": approach.tolist(),
                "rhs": rhs.tolist(),
                "stop_overtravel_along_approach_mm": row.get("stop_overtravel_along_approach_mm", ""),
            }
        )

    a = np.vstack(a_rows)
    b = np.concatenate(b_rows)
    estimate, residuals, rank, singular_values = np.linalg.lstsq(a, b, rcond=None)
    sphere_center = estimate[0:3]
    local_offset = estimate[3:6]
    predicted = a @ estimate
    residual_vector = predicted - b
    residual_norms = []
    residual_rows = []
    for index, observation in enumerate(observations):
        residual = residual_vector[index * 3 : index * 3 + 3]
        residual_norm = vector_norm(residual)
        residual_norms.append(residual_norm)
        pose = observation["pose"]
        rotation = np.asarray(euler_to_matrix(euler_sequence, pose[3:6]), dtype=float)
        ball_center = np.asarray(pose[:3], dtype=float) + rotation @ local_offset
        center_from_row = ball_center + np.asarray(observation["approach"], dtype=float) * contact_center_distance
        residual_rows.append(
            {
                "source_csv": observation["source_csv"],
                "source_row": observation["source_row"],
                "timestamp": observation["timestamp"],
                "standard_pose": observation["standard_pose"],
                "face_id": observation["face_id"],
                "flange_x": f"{pose[0]:.4f}",
                "flange_y": f"{pose[1]:.4f}",
                "flange_z": f"{pose[2]:.4f}",
                "rx": f"{pose[3]:.4f}",
                "ry": f"{pose[4]:.4f}",
                "rz": f"{pose[5]:.4f}",
                "approach_x": f"{observation['approach'][0]:.6f}",
                "approach_y": f"{observation['approach'][1]:.6f}",
                "approach_z": f"{observation['approach'][2]:.6f}",
                "fitted_ball_center_x": f"{ball_center[0]:.4f}",
                "fitted_ball_center_y": f"{ball_center[1]:.4f}",
                "fitted_ball_center_z": f"{ball_center[2]:.4f}",
                "sphere_center_from_row_x": f"{center_from_row[0]:.4f}",
                "sphere_center_from_row_y": f"{center_from_row[1]:.4f}",
                "sphere_center_from_row_z": f"{center_from_row[2]:.4f}",
                "residual_x": f"{residual[0]:.6f}",
                "residual_y": f"{residual[1]:.6f}",
                "residual_z": f"{residual[2]:.6f}",
                "residual_norm_mm": f"{residual_norm:.6f}",
                "stop_overtravel_along_approach_mm": observation["stop_overtravel_along_approach_mm"],
            }
        )

    condition = None
    if len(singular_values) and float(singular_values[-1]) > 1e-12:
        condition = float(singular_values[0] / singular_values[-1])
    return {
        "timestamp": time.time(),
        "method": "single_branch_absolute_sphere_fit",
        "method_warning": (
            "This solves one branch local ruby-ball offset and one sphere centre from mixed-orientation trigger rows. "
            "With only one non-fixed-orientation row, treat the result as a first estimate and validate with more poses."
        ),
        "rows": len(rows),
        "rank": int(rank),
        "condition": condition,
        "singular_values": singular_values.tolist(),
        "sphere_radius_mm": sphere_radius,
        "probe_radius_mm": probe_radius,
        "contact_center_distance_mm": contact_center_distance,
        "sphere_center_mm": sphere_center.tolist(),
        "local_ball_offset_mm": local_offset.tolist(),
        "rms_residual_mm": float(math.sqrt(np.mean(np.asarray(residual_norms) ** 2))),
        "max_residual_mm": float(max(residual_norms)),
        "residual_rows": residual_rows,
    }


def write_csv(path, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not rows:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def print_summary(result, branch):
    """Print summary."""
    center = result["sphere_center_mm"]
    offset = result["local_ball_offset_mm"]
    print("single-branch sphere absolute fit")
    print(f"  branch: {branch}")
    print(f"  rows: {result['rows']}")
    print(f"  rank: {result['rank']}")
    print(f"  condition: {result['condition']:.6f}" if result["condition"] is not None else "  condition: n/a")
    print(f"  sphere_center_mm: [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]")
    print(f"  local_ball_offset_mm: [{offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}]")
    print(f"  rms_residual_mm: {result['rms_residual_mm']:.6f}")
    print(f"  max_residual_mm: {result['max_residual_mm']:.6f}")
    print("  warning: validate with more rotated poses before treating this as final geometry")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", nargs="+", required=True, help="trigger CSV files")
    parser.add_argument("--branch", required=True)
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float)
    parser.add_argument("--json-output")
    parser.add_argument("--residual-output")
    args = parser.parse_args()

    try:
        geometry = load_geometry(args.geometry)
        probe_radius = args.probe_radius_mm if args.probe_radius_mm is not None else ball_radius_mm(geometry)
        if args.sphere_radius_mm <= 0 or probe_radius <= 0:
            raise ValueError("sphere and probe radii must be positive")
        rows = read_rows(args.input, args.branch)
        result = solve(rows, args.sphere_radius_mm, probe_radius, args.euler_sequence)
        result["branch"] = args.branch
        result["source_csv"] = args.input
        result["geometry"] = str(args.geometry)
        result["euler_sequence"] = args.euler_sequence
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc

    print_summary(result, args.branch)
    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(f"saved: {output}")
    if args.residual_output:
        write_csv(args.residual_output, result["residual_rows"])
        print(f"saved: {args.residual_output}")


if __name__ == "__main__":
    main()
