#!/usr/bin/env python3
"""Convert cross-probe trigger rows to calibrated contact points."""
import argparse
import csv
import json
import math
from pathlib import Path

from cross_probe_model import euler_to_matrix, mat_vec
from geometry_utils import add, normalize, scale


POSE_FIELDS = ("flange_x", "flange_y", "flange_z", "rx", "ry", "rz")
APPROACH_FIELDS = ("approach_x", "approach_y", "approach_z")


def parse_float(row, key, row_number, required=True):
    """Parse a string to float, returning a default for empty/missing values."""
    value = row.get(key, "")
    if value in (None, ""):
        if required:
            raise ValueError(f"row {row_number}: missing {key}")
        return None
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"row {row_number}: {key} must be a number") from exc
    if not math.isfinite(result):
        raise ValueError(f"row {row_number}: {key} must be finite")
    return result


def read_csv(path):
    """Read csv."""
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not rows:
        raise ValueError("no converted rows")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_calibration(path):
    """Load calibration."""
    data = json.loads(Path(path).read_text())
    branches = {}
    for branch in data.get("branches", []):
        if not branch.get("ok"):
            continue
        name = branch.get("branch")
        offset = branch.get("estimated_offset_mm")
        if not name or not isinstance(offset, list) or len(offset) != 3:
            continue
        branches[name] = {
            "offset": [float(value) for value in offset],
            "rms_all_constraints_mm": branch.get("rms_all_constraints_mm"),
            "condition": branch.get("condition"),
        }
    if not branches:
        raise ValueError(f"calibration has no usable branch estimates: {path}")
    return {
        "source": str(path),
        "euler_sequence": data.get("euler_sequence", "xyz"),
        "ball_radius_mm": float(data["ball_radius_mm"]),
        "branches": branches,
    }


def row_pose(row, row_number):
    """Row pose."""
    return [parse_float(row, field, row_number) for field in POSE_FIELDS]


def row_approach(row, row_number):
    """Row approach."""
    return normalize([parse_float(row, field, row_number) for field in APPROACH_FIELDS], f"row {row_number}: approach")


def optional_point(row, prefix, row_number):
    """Optional point."""
    fields = (f"{prefix}_x", f"{prefix}_y", f"{prefix}_z")
    if not all(row.get(field) not in (None, "") for field in fields):
        return None
    return [parse_float(row, field, row_number) for field in fields]


def distance(a, b):
    """Distance."""
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def overtravel(row, trigger_pose, approach, row_number):
    """Overtravel."""
    stop = optional_point(row, "stop_flange", row_number)
    if stop is None:
        return ""
    return sum((stop[i] - trigger_pose[i]) * approach[i] for i in range(3))


def convert_row(row, row_number, calibration):
    """Convert row."""
    branch_name = row.get("branch", "")
    branch = calibration["branches"].get(branch_name)
    base = {
        "source_row": str(row_number),
        "quality_status": "PASS" if branch else "FAIL",
        "quality_reason": "" if branch else f"missing calibration for branch {branch_name!r}",
        "timestamp": row.get("timestamp", ""),
        "session_id": row.get("session_id", ""),
        "setup_id": row.get("setup_id", ""),
        "workpiece_id": row.get("workpiece_id", ""),
        "face_id": row.get("face_id", ""),
        "sample_id": row.get("sample_id", ""),
        "tangent_y_offset_mm": row.get("tangent_y_offset_mm", ""),
        "tangent_z_offset_mm": row.get("tangent_z_offset_mm", ""),
        "branch": branch_name,
        "standard_pose": row.get("standard_pose", ""),
        "operator_note": row.get("operator_note", ""),
        "calibration_source": calibration["source"],
        "calibration_euler_sequence": calibration["euler_sequence"],
    }
    if branch is None:
        return add_blank_result_fields(base)

    try:
        pose = row_pose(row, row_number)
        approach = row_approach(row, row_number)
    except ValueError as exc:
        base["quality_status"] = "FAIL"
        base["quality_reason"] = str(exc)
        return add_blank_result_fields(base)

    rotation = euler_to_matrix(calibration["euler_sequence"], pose[3:6])
    offset = branch["offset"]
    ball_center = add(pose[:3], mat_vec(rotation, offset))
    surface = add(ball_center, scale(approach, calibration["ball_radius_mm"]))
    old_surface = optional_point(row, "surface", row_number)
    old_surface_delta = "" if old_surface is None else distance(surface, old_surface)
    stop_overtravel = overtravel(row, pose[:3], approach, row_number)

    base.update(
        {
            "flange_x": f"{pose[0]:.4f}",
            "flange_y": f"{pose[1]:.4f}",
            "flange_z": f"{pose[2]:.4f}",
            "rx": f"{pose[3]:.4f}",
            "ry": f"{pose[4]:.4f}",
            "rz": f"{pose[5]:.4f}",
            "approach_x": f"{approach[0]:.6f}",
            "approach_y": f"{approach[1]:.6f}",
            "approach_z": f"{approach[2]:.6f}",
            "calibrated_offset_x": f"{offset[0]:.6f}",
            "calibrated_offset_y": f"{offset[1]:.6f}",
            "calibrated_offset_z": f"{offset[2]:.6f}",
            "calibrated_ball_center_x": f"{ball_center[0]:.4f}",
            "calibrated_ball_center_y": f"{ball_center[1]:.4f}",
            "calibrated_ball_center_z": f"{ball_center[2]:.4f}",
            "calibrated_surface_x": f"{surface[0]:.4f}",
            "calibrated_surface_y": f"{surface[1]:.4f}",
            "calibrated_surface_z": f"{surface[2]:.4f}",
            "ball_radius_mm": f"{calibration['ball_radius_mm']:.6f}",
            "stop_overtravel_along_approach_mm": (
                "" if stop_overtravel == "" else f"{float(stop_overtravel):.4f}"
            ),
            "old_surface_delta_mm": "" if old_surface_delta == "" else f"{old_surface_delta:.4f}",
            "branch_calibration_rms_mm": (
                "" if branch["rms_all_constraints_mm"] is None else f"{float(branch['rms_all_constraints_mm']):.6f}"
            ),
            "branch_calibration_condition": (
                "" if branch["condition"] is None else f"{float(branch['condition']):.6f}"
            ),
        }
    )
    return base


def add_blank_result_fields(row):
    """Add blank result fields."""
    for key in (
        "flange_x",
        "flange_y",
        "flange_z",
        "rx",
        "ry",
        "rz",
        "approach_x",
        "approach_y",
        "approach_z",
        "calibrated_offset_x",
        "calibrated_offset_y",
        "calibrated_offset_z",
        "calibrated_ball_center_x",
        "calibrated_ball_center_y",
        "calibrated_ball_center_z",
        "calibrated_surface_x",
        "calibrated_surface_y",
        "calibrated_surface_z",
        "ball_radius_mm",
        "stop_overtravel_along_approach_mm",
        "old_surface_delta_mm",
        "branch_calibration_rms_mm",
        "branch_calibration_condition",
    ):
        row.setdefault(key, "")
    return row


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="cross probe contact CSV")
    parser.add_argument("--calibration", required=True, help="JSON from calibrate_cross_probe_geometry.py")
    parser.add_argument("--output", required=True, help="converted contact point CSV")
    args = parser.parse_args()

    try:
        calibration = load_calibration(args.calibration)
        rows = read_csv(args.input)
        if not rows:
            raise ValueError(f"no rows in: {args.input}")
        converted = [convert_row(row, index, calibration) for index, row in enumerate(rows, start=1)]
        write_csv(args.output, converted)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc

    pass_count = sum(1 for row in converted if row["quality_status"] == "PASS")
    print(f"loaded rows: {len(rows)}")
    print(f"converted rows: {len(converted)}")
    print(f"PASS: {pass_count}, FAIL: {len(converted) - pass_count}")
    print(f"saved: {args.output}")


if __name__ == "__main__":
    main()
