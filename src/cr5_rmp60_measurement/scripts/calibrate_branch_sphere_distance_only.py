#!/usr/bin/env python3
"""Coarse single-branch calibration from sphere distance constraints.

This script is intended as the bootstrap step before semi-automatic normal
probing. It reads raw GUI/jog trigger CSV rows and fits one calibration sphere
centre C_s plus one local ruby-ball offset p from:

    || F_i + R_i * p - C_s || = sphere_radius + probe_radius

Unlike calibrate_branch_sphere_absolute.py, this does not require an approach
vector. It is therefore suitable for early manual contacts where the operator
cannot guarantee that the commanded jog direction equals the true two-sphere
normal.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np

from cross_probe_model import (
    DEFAULT_GEOMETRY,
    ball_radius_mm,
    branch_local_offset_mm,
    euler_to_matrix,
    load_geometry,
)


POSE_FIELDS = ("flange_x", "flange_y", "flange_z", "rx", "ry", "rz")
TRIGGER_POSE_FIELDS = (
    "trigger_flange_x",
    "trigger_flange_y",
    "trigger_flange_z",
    "trigger_flange_rx",
    "trigger_flange_ry",
    "trigger_flange_rz",
)


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


def row_has_fields(row, fields):
    """Row has fields."""
    return all(row.get(field, "") not in (None, "") for field in fields)


def pose_from_row(row):
    """Extract a 6-element flange pose list from a CSV row dict."""
    label = f"{row['_source_csv']} row {row['_source_row']}"
    if row_has_fields(row, POSE_FIELDS):
        return [parse_float(row, field, label) for field in POSE_FIELDS]
    if row_has_fields(row, TRIGGER_POSE_FIELDS):
        return [parse_float(row, field, label) for field in TRIGGER_POSE_FIELDS]
    raise ValueError(f"{label}: missing supported pose fields")


def command_from_row(row):
    """Command from row."""
    return row.get("command") or row.get("source_command") or ""


def read_rows(
    paths,
    branch,
    physical_ball_id=None,
    include_untriggered=False,
    exclude_commands=None,
    exclude_sample_indices=None,
):
    """Read rows."""
    rows = []
    excluded = {item.strip() for item in (exclude_commands or []) if item.strip()}
    excluded_samples = {str(item).strip() for item in (exclude_sample_indices or []) if str(item).strip()}
    for path in paths:
        with Path(path).open(newline="") as f:
            for row_index, row in enumerate(csv.DictReader(f), start=1):
                if row.get("branch") != branch:
                    continue
                if physical_ball_id is not None and row.get("physical_ball_id", "") != str(physical_ball_id):
                    continue
                trigger_di1 = row.get("trigger_di1", "")
                if not include_untriggered and trigger_di1 not in ("", "1", "1.0", "True", "true"):
                    continue
                command = command_from_row(row)
                if command in excluded:
                    continue
                sample_index = row.get("sample_index") or row.get("gui_sample_index") or ""
                if sample_index in excluded_samples:
                    continue
                row = dict(row)
                row["_source_csv"] = str(path)
                row["_source_row"] = row_index
                rows.append(row)
    if not rows:
        raise ValueError(f"no usable rows found for branch {branch!r}")
    return rows


def make_observations(rows, euler_sequence):
    """Make observations."""
    observations = []
    for row in rows:
        pose = pose_from_row(row)
        observations.append(
            {
                "row": row,
                "pose": pose,
                "flange": np.asarray(pose[:3], dtype=float),
                "rotation": np.asarray(euler_to_matrix(euler_sequence, pose[3:6]), dtype=float),
            }
        )
    return observations


def distances_for_params(params, observations):
    """Distances for params."""
    sphere_center = params[:3]
    local_offset = params[3:6]
    distances = []
    ball_centers = []
    for obs in observations:
        ball_center = obs["flange"] + obs["rotation"] @ local_offset
        ball_centers.append(ball_center)
        distances.append(float(np.linalg.norm(ball_center - sphere_center)))
    return np.asarray(distances, dtype=float), ball_centers


def residual_and_jacobian(params, observations, contact_distance):
    """Residual and jacobian."""
    sphere_center = params[:3]
    local_offset = params[3:6]
    residuals = []
    jacobian = []
    for obs in observations:
        ball_center = obs["flange"] + obs["rotation"] @ local_offset
        delta = ball_center - sphere_center
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-12:
            unit = np.zeros(3, dtype=float)
        else:
            unit = delta / distance
        residuals.append(distance - contact_distance)
        row = np.zeros(6, dtype=float)
        row[0:3] = -unit
        row[3:6] = unit @ obs["rotation"]
        jacobian.append(row)
    return np.asarray(residuals, dtype=float), np.vstack(jacobian)


def cost(residuals):
    """Cost."""
    return 0.5 * float(np.dot(residuals, residuals))


def solve_distance_only(
    observations,
    contact_distance,
    initial_offset,
    max_iterations=200,
    tolerance=1e-10,
):
    """Solve distance only."""
    ball_centers0 = [obs["flange"] + obs["rotation"] @ initial_offset for obs in observations]
    initial_center = np.mean(np.vstack(ball_centers0), axis=0)
    params = np.concatenate([initial_center, np.asarray(initial_offset, dtype=float)])
    damping = 1e-3
    converged = False
    iterations = 0

    for iterations in range(1, max_iterations + 1):
        residuals, jacobian = residual_and_jacobian(params, observations, contact_distance)
        current_cost = cost(residuals)
        lhs = jacobian.T @ jacobian + damping * np.eye(6)
        rhs = -jacobian.T @ residuals
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
        if float(np.linalg.norm(step)) <= tolerance:
            converged = True
            break
        candidate = params + step
        candidate_residuals, _ = residual_and_jacobian(candidate, observations, contact_distance)
        if cost(candidate_residuals) < current_cost:
            params = candidate
            damping = max(damping * 0.3, 1e-12)
            if abs(current_cost - cost(candidate_residuals)) <= tolerance:
                converged = True
                break
        else:
            damping = min(damping * 10.0, 1e12)

    residuals, jacobian = residual_and_jacobian(params, observations, contact_distance)
    _, singular_values, _ = np.linalg.svd(jacobian, full_matrices=False)
    rank = int(np.linalg.matrix_rank(jacobian))
    condition = None
    if len(singular_values) and float(singular_values[-1]) > 1e-12:
        condition = float(singular_values[0] / singular_values[-1])
    return {
        "params": params,
        "residuals": residuals,
        "jacobian": jacobian,
        "singular_values": singular_values,
        "rank": rank,
        "condition": condition,
        "iterations": iterations,
        "converged": converged,
        "final_damping": damping,
        "initial_center": initial_center,
    }


def build_result(rows, observations, solution, contact_distance, branch, args, initial_offset):
    """Build result."""
    params = solution["params"]
    sphere_center = params[:3]
    local_offset = params[3:6]
    residuals = solution["residuals"]
    distances, ball_centers = distances_for_params(params, observations)
    residual_rows = []
    for obs, row_residual, distance, ball_center in zip(observations, residuals, distances, ball_centers):
        row = obs["row"]
        pose = obs["pose"]
        residual_rows.append(
            {
                "source_csv": row["_source_csv"],
                "source_row": row["_source_row"],
                "timestamp": row.get("timestamp", ""),
                "sample_index": row.get("sample_index", row.get("gui_sample_index", "")),
                "command": command_from_row(row),
                "physical_ball_id": row.get("physical_ball_id", ""),
                "branch": row.get("branch", ""),
                "flange_x": f"{pose[0]:.4f}",
                "flange_y": f"{pose[1]:.4f}",
                "flange_z": f"{pose[2]:.4f}",
                "rx": f"{pose[3]:.4f}",
                "ry": f"{pose[4]:.4f}",
                "rz": f"{pose[5]:.4f}",
                "fitted_ball_center_x": f"{ball_center[0]:.4f}",
                "fitted_ball_center_y": f"{ball_center[1]:.4f}",
                "fitted_ball_center_z": f"{ball_center[2]:.4f}",
                "sphere_center_x": f"{sphere_center[0]:.4f}",
                "sphere_center_y": f"{sphere_center[1]:.4f}",
                "sphere_center_z": f"{sphere_center[2]:.4f}",
                "center_distance_mm": f"{distance:.6f}",
                "target_center_distance_mm": f"{contact_distance:.6f}",
                "radial_residual_mm": f"{float(row_residual):.6f}",
                "abs_radial_residual_mm": f"{abs(float(row_residual)):.6f}",
            }
        )
    abs_residuals = np.abs(residuals)
    return {
        "timestamp": time.time(),
        "method": "single_branch_distance_only_sphere_fit",
        "method_warning": (
            "This coarse bootstrap fit uses only the distance-to-sphere constraint. "
            "It does not need commanded approach vectors, but it still requires that every row is the same physical "
            "ruby ball touching the same calibration sphere. Use the result as an initial fit for semi-auto normal "
            "probing, not as final tool geometry."
        ),
        "branch": branch,
        "rows": len(rows),
        "rank": solution["rank"],
        "degrees_of_freedom": len(rows) - solution["rank"],
        "condition": solution["condition"],
        "singular_values": solution["singular_values"].tolist(),
        "converged": solution["converged"],
        "iterations": solution["iterations"],
        "sphere_radius_mm": args.sphere_radius_mm,
        "probe_radius_mm": args.probe_radius_mm,
        "contact_center_distance_mm": contact_distance,
        "initial_local_ball_offset_mm": np.asarray(initial_offset, dtype=float).tolist(),
        "initial_sphere_center_mm": solution["initial_center"].tolist(),
        "sphere_center_mm": sphere_center.tolist(),
        "local_ball_offset_mm": local_offset.tolist(),
        "rms_radial_residual_mm": float(math.sqrt(np.mean(residuals**2))),
        "max_abs_radial_residual_mm": float(np.max(abs_residuals)),
        "mean_abs_radial_residual_mm": float(np.mean(abs_residuals)),
        "source_csv": args.input,
        "geometry": str(args.geometry),
        "euler_sequence": args.euler_sequence,
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


def print_summary(result):
    """Print summary."""
    center = result["sphere_center_mm"]
    offset = result["local_ball_offset_mm"]
    print("single-branch distance-only coarse fit")
    print(f"  branch: {result['branch']}")
    print(f"  rows: {result['rows']}")
    print(f"  rank: {result['rank']}")
    print(f"  degrees_of_freedom: {result['degrees_of_freedom']}")
    print(f"  condition: {result['condition']:.6f}" if result["condition"] is not None else "  condition: n/a")
    print(f"  converged: {result['converged']} in {result['iterations']} iterations")
    print(f"  sphere_center_mm: [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]")
    print(f"  local_ball_offset_mm: [{offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}]")
    print(f"  rms_radial_residual_mm: {result['rms_radial_residual_mm']:.6f}")
    print(f"  max_abs_radial_residual_mm: {result['max_abs_radial_residual_mm']:.6f}")
    if result["degrees_of_freedom"] <= 0:
        print("  warning: rows do not exceed fitted rank; residuals can be zero by interpolation")
    print("  warning: use only as a bootstrap fit for semi-auto normal probing")


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", nargs="+", required=True, help="raw GUI/jog trigger CSV files")
    parser.add_argument("--branch", required=True)
    parser.add_argument("--physical-ball-id")
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float)
    parser.add_argument("--initial-offset", nargs=3, type=float, metavar=("PX", "PY", "PZ"))
    parser.add_argument("--include-untriggered", action="store_true")
    parser.add_argument(
        "--exclude-command",
        action="append",
        default=["capture_pose"],
        help="exclude exact command text; can be repeated",
    )
    parser.add_argument(
        "--exclude-sample-index",
        action="append",
        default=[],
        help="exclude a sample_index/gui_sample_index value; can be repeated",
    )
    parser.add_argument("--max-iterations", type=int, default=200)
    parser.add_argument("--json-output")
    parser.add_argument("--residual-output")
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    try:
        geometry = load_geometry(args.geometry)
        args.probe_radius_mm = args.probe_radius_mm if args.probe_radius_mm is not None else ball_radius_mm(geometry)
        if args.sphere_radius_mm <= 0 or args.probe_radius_mm <= 0:
            raise ValueError("sphere and probe radii must be positive")
        rows = read_rows(
            args.input,
            args.branch,
            physical_ball_id=args.physical_ball_id,
            include_untriggered=args.include_untriggered,
            exclude_commands=args.exclude_command,
            exclude_sample_indices=args.exclude_sample_index,
        )
        observations = make_observations(rows, args.euler_sequence)
        initial_offset = (
            np.asarray(args.initial_offset, dtype=float)
            if args.initial_offset is not None
            else np.asarray(branch_local_offset_mm(geometry, args.branch), dtype=float)
        )
        if len(initial_offset) != 3:
            raise ValueError("initial offset must contain three values")
        contact_distance = args.sphere_radius_mm + args.probe_radius_mm
        solution = solve_distance_only(
            observations,
            contact_distance,
            initial_offset,
            max_iterations=args.max_iterations,
        )
        result = build_result(rows, observations, solution, contact_distance, args.branch, args, initial_offset)
    except (OSError, ValueError, KeyError, np.linalg.LinAlgError) as exc:
        raise SystemExit(str(exc)) from exc

    print_summary(result)
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
