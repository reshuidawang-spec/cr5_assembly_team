#!/usr/bin/env python3
"""Fit cross-stylus branch offsets from contact constraints.

This is an offline calibration scaffold. It supports two constraint styles:

- known_surface_x/y/z: direct known contact point for a row.
- plane_nx/ny/nz + plane_d: contact point lies on n dot p + d = 0.

Rows must include branch, flange_x/y/z, rx/ry/rz, and approach_x/y/z.
"""
import argparse
import csv
import json
import math
import random
import time
from pathlib import Path

import numpy as np

from cross_probe_model import (
    DEFAULT_GEOMETRY,
    ball_radius_mm,
    branch_local_offset_mm,
    euler_to_matrix,
    load_geometry,
    mat_vec,
)
from geometry_utils import add, normalize, scale


BRANCH_APPROACH = {
    "x_pos": [1.0, 0.0, 0.0],
    "x_neg": [-1.0, 0.0, 0.0],
    "y_pos": [0.0, 1.0, 0.0],
    "y_neg": [0.0, -1.0, 0.0],
}


def parse_float(row, key, row_number, required=True, default=None):
    """Parse a string to float, returning a default for empty/missing values."""
    value = row.get(key, "")
    if value in (None, ""):
        if required:
            raise ValueError(f"row {row_number}: missing {key}")
        return default
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"row {row_number}: {key} must be a number") from exc
    if not math.isfinite(result):
        raise ValueError(f"row {row_number}: {key} must be finite")
    return result


def read_rows(path):
    """Read rows."""
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def write_rows(path, rows):
    """Write rows."""
    if not rows:
        raise ValueError("no synthetic rows generated")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def pose_from_row(row, row_number):
    """Extract a 6-element flange pose list from a CSV row dict."""
    return [
        parse_float(row, "flange_x", row_number),
        parse_float(row, "flange_y", row_number),
        parse_float(row, "flange_z", row_number),
        parse_float(row, "rx", row_number),
        parse_float(row, "ry", row_number),
        parse_float(row, "rz", row_number),
    ]


def approach_from_row(row, row_number):
    """Approach from row."""
    return normalize(
        [
            parse_float(row, "approach_x", row_number),
            parse_float(row, "approach_y", row_number),
            parse_float(row, "approach_z", row_number),
        ],
        f"row {row_number}: approach",
    )


def surface_from_row(row, row_number):
    """Surface from row."""
    fields = ("known_surface_x", "known_surface_y", "known_surface_z")
    if not all(row.get(field) not in (None, "") for field in fields):
        return None
    return [parse_float(row, field, row_number) for field in fields]


def plane_from_row(row, row_number):
    """Plane from row."""
    fields = ("plane_nx", "plane_ny", "plane_nz", "plane_d")
    if not all(row.get(field) not in (None, "") for field in fields):
        return None
    raw_normal = [parse_float(row, field, row_number) for field in fields[:3]]
    normal_length = math.sqrt(sum(value * value for value in raw_normal))
    if normal_length <= 1e-12:
        raise ValueError(f"row {row_number}: plane normal cannot be zero")
    normal = [value / normal_length for value in raw_normal]
    d = parse_float(row, "plane_d", row_number) / normal_length
    return normal, d


def rotation_from_pose(pose, euler_sequence):
    """Rotation from pose."""
    return np.asarray(euler_to_matrix(euler_sequence, pose[3:6]), dtype=float)


def build_branch_system(rows, branch, radius, euler_sequence):
    """Build branch system."""
    vector_observations = []
    plane_a = []
    plane_b = []
    used_rows = []

    for row_number, row in rows:
        if row.get("branch") != branch:
            continue
        pose = pose_from_row(row, row_number)
        flange = np.asarray(pose[:3], dtype=float)
        rotation = rotation_from_pose(pose, euler_sequence)
        approach = np.asarray(approach_from_row(row, row_number), dtype=float)
        surface = surface_from_row(row, row_number)
        plane = plane_from_row(row, row_number)

        if surface is not None:
            target = np.asarray(surface, dtype=float) - flange - approach * radius
            vector_observations.append(rotation.T @ target)
            used_rows.append(row_number)

        if plane is not None:
            normal, d = plane
            normal = np.asarray(normal, dtype=float)
            plane_a.append(normal @ rotation)
            plane_b.append(-(float(normal @ flange) + float(normal @ approach) * radius + d))
            used_rows.append(row_number)

    return vector_observations, plane_a, plane_b, used_rows


def solve_branch(rows, branch, nominal_offset, radius, euler_sequence):
    """Solve branch."""
    vector_observations, plane_a, plane_b, used_rows = build_branch_system(rows, branch, radius, euler_sequence)
    a_rows = []
    b_rows = []

    for observation in vector_observations:
        a_rows.extend(np.eye(3))
        b_rows.extend(observation.tolist())
    for coeffs, value in zip(plane_a, plane_b):
        a_rows.append(coeffs.tolist())
        b_rows.append(float(value))

    if len(a_rows) < 3:
        return {
            "branch": branch,
            "ok": False,
            "reason": "not enough constraints to solve 3D offset",
            "row_numbers": sorted(set(used_rows)),
            "vector_observations": len(vector_observations),
            "plane_observations": len(plane_a),
            "nominal_offset_mm": nominal_offset,
        }

    a = np.asarray(a_rows, dtype=float)
    b = np.asarray(b_rows, dtype=float)
    estimate, residuals, rank, singular_values = np.linalg.lstsq(a, b, rcond=None)
    predicted = a @ estimate
    all_residuals = predicted - b

    vector_residuals = []
    for observation in vector_observations:
        vector_residuals.append(float(np.linalg.norm(estimate - observation)))

    plane_residuals = []
    for coeffs, value in zip(plane_a, plane_b):
        plane_residuals.append(float(np.dot(coeffs, estimate) - value))

    nominal = np.asarray(nominal_offset, dtype=float)
    condition = None
    if len(singular_values) and float(singular_values[-1]) > 1e-12:
        condition = float(singular_values[0] / singular_values[-1])

    return {
        "branch": branch,
        "ok": bool(rank == 3),
        "rank": int(rank),
        "condition": condition,
        "row_numbers": sorted(set(used_rows)),
        "vector_observations": len(vector_observations),
        "plane_observations": len(plane_a),
        "nominal_offset_mm": nominal.tolist(),
        "estimated_offset_mm": estimate.tolist(),
        "delta_from_nominal_mm": (estimate - nominal).tolist(),
        "rms_all_constraints_mm": float(math.sqrt(np.mean(all_residuals * all_residuals))),
        "max_abs_all_constraints_mm": float(np.max(np.abs(all_residuals))),
        "rms_vector_residual_mm": (
            None if not vector_residuals else float(math.sqrt(np.mean(np.asarray(vector_residuals) ** 2)))
        ),
        "rms_plane_residual_mm": (
            None if not plane_residuals else float(math.sqrt(np.mean(np.asarray(plane_residuals) ** 2)))
        ),
    }


def perturb_offset(offset, branch_index):
    """Perturb offset."""
    return [
        offset[0] + 0.35 * (branch_index + 1),
        offset[1] - 0.20 * (branch_index + 1),
        offset[2] + 0.15 * (branch_index + 1),
    ]


def random_pose(rng, base_position, sample_index):
    """Random pose."""
    return [
        base_position[0] + rng.uniform(-15.0, 15.0),
        base_position[1] + rng.uniform(-15.0, 15.0),
        base_position[2] + rng.uniform(-10.0, 10.0),
        -175.0 + rng.uniform(-8.0, 8.0) + sample_index * 0.2,
        rng.uniform(-5.0, 5.0),
        120.0 + rng.uniform(-10.0, 10.0),
    ]


def make_synthetic_rows(geometry, args):
    """Make synthetic rows."""
    rng = random.Random(args.seed)
    radius = ball_radius_mm(geometry)
    rows = []
    branches = args.branches or list(BRANCH_APPROACH)
    for branch_index, branch in enumerate(branches):
        nominal_offset = branch_local_offset_mm(geometry, branch)
        true_offset = perturb_offset(nominal_offset, branch_index)
        approach = BRANCH_APPROACH.get(branch)
        if approach is None:
            raise ValueError(f"synthetic generation has no default approach for branch {branch!r}")
        for sample in range(args.synthetic_samples_per_branch):
            pose = random_pose(rng, [320.0, 120.0, 210.0], sample)
            rotation = euler_to_matrix(args.euler_sequence, pose[3:6])
            ball_center = add(pose[:3], mat_vec(rotation, true_offset))
            surface = add(ball_center, scale(approach, radius))
            noisy_surface = [value + rng.gauss(0.0, args.synthetic_noise_mm) for value in surface]
            normal = normalize(approach)
            plane_d = -sum(n * p for n, p in zip(normal, noisy_surface))
            rows.append(
                {
                    "timestamp": f"{time.time():.3f}",
                    "branch": branch,
                    "approach_x": f"{approach[0]:.6f}",
                    "approach_y": f"{approach[1]:.6f}",
                    "approach_z": f"{approach[2]:.6f}",
                    "flange_x": f"{pose[0]:.6f}",
                    "flange_y": f"{pose[1]:.6f}",
                    "flange_z": f"{pose[2]:.6f}",
                    "rx": f"{pose[3]:.6f}",
                    "ry": f"{pose[4]:.6f}",
                    "rz": f"{pose[5]:.6f}",
                    "known_surface_x": f"{noisy_surface[0]:.6f}",
                    "known_surface_y": f"{noisy_surface[1]:.6f}",
                    "known_surface_z": f"{noisy_surface[2]:.6f}",
                    "plane_nx": f"{normal[0]:.6f}",
                    "plane_ny": f"{normal[1]:.6f}",
                    "plane_nz": f"{normal[2]:.6f}",
                    "plane_d": f"{plane_d:.6f}",
                    "true_offset_x": f"{true_offset[0]:.6f}",
                    "true_offset_y": f"{true_offset[1]:.6f}",
                    "true_offset_z": f"{true_offset[2]:.6f}",
                }
            )
    return rows


def run_calibration(rows, geometry, args):
    """Run calibration."""
    numbered_rows = list(enumerate(rows, start=1))
    radius = ball_radius_mm(geometry)
    branches = args.branches or sorted({row.get("branch") for row in rows if row.get("branch")})
    results = []
    for branch in branches:
        nominal_offset = branch_local_offset_mm(geometry, branch)
        results.append(solve_branch(numbered_rows, branch, nominal_offset, radius, args.euler_sequence))
    return {
        "timestamp": time.time(),
        "source_csv": args.input,
        "geometry": str(args.geometry),
        "euler_sequence": args.euler_sequence,
        "ball_radius_mm": radius,
        "branches": results,
    }


def print_summary(result):
    """Print summary."""
    print("cross probe geometry calibration")
    print(f"  euler_sequence: {result['euler_sequence']}")
    print(f"  ball_radius_mm: {result['ball_radius_mm']:.4f}")
    for branch in result["branches"]:
        status = "PASS" if branch["ok"] else "FAIL"
        print(f"  [{status}] {branch['branch']}")
        print(f"    rows: {branch.get('row_numbers', [])}")
        print(
            f"    constraints: vector={branch.get('vector_observations', 0)}, "
            f"plane={branch.get('plane_observations', 0)}"
        )
        if not branch["ok"]:
            print(f"    reason: {branch.get('reason', 'rank deficient')}")
            continue
        estimate = branch["estimated_offset_mm"]
        delta = branch["delta_from_nominal_mm"]
        print(f"    estimated_offset_mm: [{estimate[0]:.4f}, {estimate[1]:.4f}, {estimate[2]:.4f}]")
        print(f"    delta_from_nominal_mm: [{delta[0]:.4f}, {delta[1]:.4f}, {delta[2]:.4f}]")
        print(f"    rank: {branch['rank']}, condition: {branch['condition']:.3f}")
        print(f"    rms_all_constraints_mm: {branch['rms_all_constraints_mm']:.6f}")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="CSV with contact constraints")
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--branches", nargs="+", choices=tuple(BRANCH_APPROACH))
    parser.add_argument("--json-output", help="write calibration result JSON")
    parser.add_argument("--generate-synthetic", help="write synthetic calibration CSV before fitting it")
    parser.add_argument("--synthetic-samples-per-branch", type=int, default=8)
    parser.add_argument("--synthetic-noise-mm", type=float, default=0.01)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    if not args.input and not args.generate_synthetic:
        raise SystemExit("provide --input or --generate-synthetic")
    if args.synthetic_samples_per_branch <= 0:
        raise SystemExit("--synthetic-samples-per-branch must be positive")
    if args.synthetic_noise_mm < 0:
        raise SystemExit("--synthetic-noise-mm cannot be negative")

    try:
        geometry = load_geometry(args.geometry)
        if args.generate_synthetic:
            rows = make_synthetic_rows(geometry, args)
            write_rows(args.generate_synthetic, rows)
            print(f"saved synthetic CSV: {args.generate_synthetic}")
            if not args.input:
                args.input = args.generate_synthetic
        rows = read_rows(args.input)
        if not rows:
            raise ValueError(f"no rows in: {args.input}")
        result = run_calibration(rows, geometry, args)
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc

    print_summary(result)
    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(f"saved: {output}")


if __name__ == "__main__":
    main()
