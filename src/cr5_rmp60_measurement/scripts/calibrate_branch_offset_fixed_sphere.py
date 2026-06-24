#!/usr/bin/env python3
"""Fit one ruby-ball offset with a trusted calibration-sphere centre fixed.

The fit uses only trigger flange poses and the two-sphere centre distance:

    ||F_i + R_i p - C_s|| = sphere_radius + probe_radius

It intentionally does not use commanded or estimated approach vectors.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from calibrate_branch_sphere_distance_only import make_observations, read_rows, write_csv
from cross_probe_model import DEFAULT_GEOMETRY, ball_radius_mm, branch_local_offset_mm, load_geometry


def load_sphere(path):
    """Load sphere."""
    data = json.loads(Path(path).read_text())
    center = data.get("sphere_center_mm")
    if not isinstance(center, list) or len(center) != 3:
        raise ValueError(f"{path}: missing 3-value sphere_center_mm")
    return data, np.asarray(center, dtype=float)


def residual_and_jacobian(offset, observations, sphere_center, contact_distance):
    """Residual and jacobian."""
    residuals = []
    jacobian = []
    ball_centers = []
    for obs in observations:
        ball_center = obs["flange"] + obs["rotation"] @ offset
        delta = ball_center - sphere_center
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-12:
            raise ValueError("fitted ruby centre coincides with the sphere centre")
        unit = delta / distance
        residuals.append(distance - contact_distance)
        jacobian.append(unit @ obs["rotation"])
        ball_centers.append(ball_center)
    return np.asarray(residuals), np.vstack(jacobian), ball_centers


def solve(observations, sphere_center, contact_distance, initial_offset, max_iterations, tolerance):
    """Solve."""
    offset = np.asarray(initial_offset, dtype=float).copy()
    damping = 1e-3
    converged = False
    iterations = 0
    for iterations in range(1, max_iterations + 1):
        residuals, jacobian, _ = residual_and_jacobian(
            offset, observations, sphere_center, contact_distance
        )
        current_cost = 0.5 * float(residuals @ residuals)
        lhs = jacobian.T @ jacobian + damping * np.eye(3)
        rhs = -jacobian.T @ residuals
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
        if float(np.linalg.norm(step)) <= tolerance:
            converged = True
            break
        candidate = offset + step
        candidate_residuals, _, _ = residual_and_jacobian(
            candidate, observations, sphere_center, contact_distance
        )
        candidate_cost = 0.5 * float(candidate_residuals @ candidate_residuals)
        if candidate_cost < current_cost:
            offset = candidate
            damping = max(1e-12, damping * 0.3)
            if abs(current_cost - candidate_cost) <= tolerance:
                converged = True
                break
        else:
            damping = min(1e12, damping * 10.0)

    residuals, jacobian, ball_centers = residual_and_jacobian(
        offset, observations, sphere_center, contact_distance
    )
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    rank = int(np.linalg.matrix_rank(jacobian))
    condition = None
    if len(singular_values) and float(singular_values[-1]) > 1e-12:
        condition = float(singular_values[0] / singular_values[-1])
    return {
        "offset": offset,
        "residuals": residuals,
        "ball_centers": ball_centers,
        "rank": rank,
        "singular_values": singular_values,
        "condition": condition,
        "iterations": iterations,
        "converged": converged,
    }


def build_result(rows, observations, solution, sphere_data, sphere_center, contact_distance, args):
    """Build result."""
    residual_rows = []
    for obs, residual, ball_center in zip(
        observations, solution["residuals"], solution["ball_centers"]
    ):
        row = obs["row"]
        pose = obs["pose"]
        distance = float(np.linalg.norm(ball_center - sphere_center))
        residual_rows.append(
            {
                "source_csv": row["_source_csv"],
                "source_row": row["_source_row"],
                "timestamp": row.get("timestamp", ""),
                "branch": args.branch,
                "physical_ball_id": row.get("physical_ball_id", ""),
                "flange_x": f"{pose[0]:.4f}",
                "flange_y": f"{pose[1]:.4f}",
                "flange_z": f"{pose[2]:.4f}",
                "rx": f"{pose[3]:.4f}",
                "ry": f"{pose[4]:.4f}",
                "rz": f"{pose[5]:.4f}",
                "fitted_ball_center_x": f"{ball_center[0]:.6f}",
                "fitted_ball_center_y": f"{ball_center[1]:.6f}",
                "fitted_ball_center_z": f"{ball_center[2]:.6f}",
                "center_distance_mm": f"{distance:.6f}",
                "radial_residual_mm": f"{float(residual):.6f}",
                "operator_note": row.get("operator_note", ""),
            }
        )
    residuals = solution["residuals"]
    return {
        "timestamp": time.time(),
        "method": "fixed_sphere_center_distance_only_branch_fit",
        "method_warning": (
            "Sphere centre is fixed from an independently calibrated branch. "
            "The fit uses trigger poses only and does not assume manual jog directions are contact normals."
        ),
        "branch": args.branch,
        "rows": len(rows),
        "rank": solution["rank"],
        "degrees_of_freedom": len(rows) - solution["rank"],
        "condition": solution["condition"],
        "singular_values": solution["singular_values"].tolist(),
        "converged": solution["converged"],
        "iterations": solution["iterations"],
        "reference_fit_json": str(args.reference_fit_json),
        "sphere_center_mm": sphere_center.tolist(),
        "sphere_radius_mm": args.sphere_radius_mm,
        "probe_radius_mm": args.probe_radius_mm,
        "contact_center_distance_mm": contact_distance,
        "initial_local_ball_offset_mm": [float(value) for value in args.initial_offset],
        "local_ball_offset_mm": solution["offset"].tolist(),
        "rms_residual_mm": float(math.sqrt(float(np.mean(residuals * residuals)))),
        "max_residual_mm": float(np.max(np.abs(residuals))),
        "source_csv": [str(path) for path in args.input],
        "geometry": str(args.geometry),
        "euler_sequence": args.euler_sequence,
        "residual_rows": residual_rows,
    }


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", nargs="+", required=True)
    parser.add_argument("--branch", required=True)
    parser.add_argument("--physical-ball-id")
    parser.add_argument("--reference-fit-json", required=True)
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float)
    parser.add_argument("--initial-offset", nargs=3, type=float, metavar=("PX", "PY", "PZ"))
    parser.add_argument("--max-iterations", type=int, default=200)
    parser.add_argument("--tolerance", type=float, default=1e-12)
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--residual-output", required=True)
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    try:
        geometry = load_geometry(args.geometry)
        sphere_data, sphere_center = load_sphere(args.reference_fit_json)
        args.probe_radius_mm = (
            args.probe_radius_mm
            if args.probe_radius_mm is not None
            else ball_radius_mm(geometry)
        )
        if args.sphere_radius_mm <= 0 or args.probe_radius_mm <= 0:
            raise ValueError("sphere/probe radii must be positive")
        rows = read_rows(args.input, args.branch, physical_ball_id=args.physical_ball_id)
        observations = make_observations(rows, args.euler_sequence)
        if args.initial_offset is None:
            args.initial_offset = branch_local_offset_mm(geometry, args.branch)
        contact_distance = float(
            sphere_data.get(
                "contact_center_distance_mm",
                args.sphere_radius_mm + args.probe_radius_mm,
            )
        )
        solution = solve(
            observations,
            sphere_center,
            contact_distance,
            args.initial_offset,
            args.max_iterations,
            args.tolerance,
        )
        result = build_result(
            rows,
            observations,
            solution,
            sphere_data,
            sphere_center,
            contact_distance,
            args,
        )
    except (OSError, ValueError, KeyError, np.linalg.LinAlgError) as exc:
        raise SystemExit(str(exc)) from exc

    output = Path(args.json_output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    write_csv(args.residual_output, result["residual_rows"])
    offset = result["local_ball_offset_mm"]
    print(f"fixed sphere centre: {result['sphere_center_mm']}")
    print(f"offset: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}]")
    condition = (
        "n/a" if result["condition"] is None else f"{result['condition']:.6f}"
    )
    print(
        f"rows={result['rows']} rank={result['rank']} dof={result['degrees_of_freedom']} "
        f"condition={condition} rms={result['rms_residual_mm']:.6f}mm "
        f"max={result['max_residual_mm']:.6f}mm"
    )
    print(f"saved: {output}")
    print(f"saved: {args.residual_output}")


if __name__ == "__main__":
    main()
