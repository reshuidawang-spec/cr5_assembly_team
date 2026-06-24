#!/usr/bin/env python3
"""Convert GUI normal-probe rows to absolute sphere-fit rows.

The probe calibration GUI records raw trigger rows with trigger_flange_* fields
and stores the computed probing direction inside the command field, for example:

    normal 0.976009 -0.205816 -0.071039 0.5000
    normal_jog 0.630295 0.136551 -0.764253 0.5000

calibrate_branch_sphere_absolute.py expects flange_x/y/z, rx/ry/rz, and
approach_x/y/z columns. This script bridges those two schemas while preserving
the source metadata needed for review.
"""
import argparse
import csv
import math
import re
from pathlib import Path


TRIGGER_POSE_FIELDS = (
    "trigger_flange_x",
    "trigger_flange_y",
    "trigger_flange_z",
    "trigger_flange_rx",
    "trigger_flange_ry",
    "trigger_flange_rz",
)


OUTPUT_FIELDS = (
    "timestamp",
    "gui_row",
    "gui_sample_index",
    "standard_pose",
    "setup_id",
    "session_id",
    "workpiece_id",
    "face_id",
    "artifact_id",
    "artifact_type",
    "branch",
    "approach_x",
    "approach_y",
    "approach_z",
    "flange_x",
    "flange_y",
    "flange_z",
    "rx",
    "ry",
    "rz",
    "stop_overtravel_along_approach_mm",
    "source_command",
    "source_branch",
    "physical_ball_id",
    "operator_note",
)


COMMAND_RE = re.compile(
    r"^(normal|normal_jog)\s+"
    r"([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)\s+"
    r"([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)\s+"
    r"([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)\s+"
    r"([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)"
)


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


def normalize(vector, label):
    """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-12:
        raise ValueError(f"{label} cannot be zero")
    return [value / length for value in vector]


def parse_normal_command(command):
    """Parse normal command."""
    match = COMMAND_RE.match((command or "").strip())
    if not match:
        return None
    values = [float(match.group(index)) for index in range(2, 5)]
    return normalize(values, f"command {command!r} approach")


def row_trigger_pose(row, row_number):
    """Row trigger pose."""
    label = f"row {row_number}"
    return [parse_float(row.get(field), f"{label} {field}") for field in TRIGGER_POSE_FIELDS]


def optional_stop_overtravel(row, approach):
    """Optional stop overtravel."""
    fields = ("stop_flange_x", "stop_flange_y", "stop_flange_z")
    if not all(row.get(field) not in (None, "") for field in fields):
        return ""
    trigger = [parse_float(row.get(field), field) for field in TRIGGER_POSE_FIELDS[:3]]
    stop = [parse_float(row.get(field), field) for field in fields]
    return sum((stop[index] - trigger[index]) * approach[index] for index in range(3))


def convert_row(row, row_number):
    """Convert row."""
    approach = parse_normal_command(row.get("command", ""))
    if approach is None:
        return None
    pose = row_trigger_pose(row, row_number)
    overtravel = optional_stop_overtravel(row, approach)
    return {
        "timestamp": row.get("timestamp", ""),
        "gui_row": str(row_number),
        "gui_sample_index": row.get("sample_index") or row.get("gui_sample_index") or "",
        "standard_pose": row.get("standard_pose", "") or "gui_normal_probe",
        "setup_id": row.get("setup_id", "") or "gui_probe_calibration",
        "session_id": row.get("session_id", ""),
        "workpiece_id": row.get("workpiece_id", ""),
        "face_id": row.get("face_id", "") or "standard_sphere_gui_normal",
        "artifact_id": row.get("artifact_id", ""),
        "artifact_type": row.get("artifact_type", ""),
        "branch": row.get("branch", ""),
        "approach_x": f"{approach[0]:.6f}",
        "approach_y": f"{approach[1]:.6f}",
        "approach_z": f"{approach[2]:.6f}",
        "flange_x": f"{pose[0]:.4f}",
        "flange_y": f"{pose[1]:.4f}",
        "flange_z": f"{pose[2]:.4f}",
        "rx": f"{pose[3]:.4f}",
        "ry": f"{pose[4]:.4f}",
        "rz": f"{pose[5]:.4f}",
        "stop_overtravel_along_approach_mm": "" if overtravel == "" else f"{overtravel:.4f}",
        "source_command": row.get("command", ""),
        "source_branch": row.get("branch", ""),
        "physical_ball_id": row.get("physical_ball_id", ""),
        "operator_note": row.get("operator_note", ""),
    }


def read_and_convert(args):
    """Read and convert."""
    converted = []
    skipped = 0
    with Path(args.input).open(newline="") as f:
        for row_number, row in enumerate(csv.DictReader(f), start=1):
            if args.branch and row.get("branch") != args.branch:
                skipped += 1
                continue
            if args.physical_ball_id is not None and row.get("physical_ball_id", "") != str(args.physical_ball_id):
                skipped += 1
                continue
            trigger_di1 = row.get("trigger_di1", "")
            if not args.include_untriggered and trigger_di1 not in ("", "1", "1.0", "true", "True"):
                skipped += 1
                continue
            converted_row = convert_row(row, row_number)
            if converted_row is None:
                skipped += 1
                continue
            converted.append(converted_row)
    return converted, skipped


def write_csv(path, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not rows:
        raise ValueError("no normal GUI rows were converted")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="raw GUI contact CSV")
    parser.add_argument("--output", required=True, help="absolute-fit CSV to write")
    parser.add_argument("--branch", default="y_neg")
    parser.add_argument("--physical-ball-id", type=int)
    parser.add_argument("--include-untriggered", action="store_true")
    args = parser.parse_args()

    try:
        converted, skipped = read_and_convert(args)
        write_csv(args.output, converted)
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc

    print(f"converted normal rows: {len(converted)}")
    print(f"skipped rows: {skipped}")
    print(f"saved: {args.output}")


if __name__ == "__main__":
    main()
