#!/usr/bin/env python3
"""Generate a multi-pose measurement plan from contact points and approach vectors."""
import argparse
import csv
import json
import math
from pathlib import Path

from generate_measurement_poses import build_pose_spec


REQUIRED_COLUMNS = ("x", "y", "z", "dx", "dy", "dz")


def parse_float(row, key, row_number):
    """Parse a string to float, returning a default for empty/missing values."""
    value = row.get(key, "")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"row {row_number}: {key} must be a number") from exc
    if not math.isfinite(result):
        raise ValueError(f"row {row_number}: {key} must be finite")
    return result


def optional_positive_float(row, key, default, row_number):
    """Optional positive float."""
    value = row.get(key, "")
    if value is None or str(value).strip() == "":
        return default
    result = parse_float(row, key, row_number)
    if result <= 0:
        raise ValueError(f"row {row_number}: {key} must be positive")
    return result


def optional_nonnegative_float(row, key, default, row_number):
    """Optional nonnegative float."""
    value = row.get(key, "")
    if value is None or str(value).strip() == "":
        return default
    result = parse_float(row, key, row_number)
    if result < 0:
        raise ValueError(f"row {row_number}: {key} cannot be negative")
    return result


def load_rows(path):
    """Load rows."""
    with Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV header is missing")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV missing required columns: {', '.join(missing)}")
        return list(reader)


def build_plan(rows, args):
    """Build plan."""
    specs = []
    seen_names = set()
    for index, row in enumerate(rows, start=1):
        name = (row.get("name") or f"pose_{index:03d}").strip()
        if not name:
            name = f"pose_{index:03d}"
        if name in seen_names:
            raise ValueError(f"row {index}: duplicate name: {name}")
        seen_names.add(name)

        contact = [parse_float(row, key, index) for key in ("x", "y", "z")]
        approach = [parse_float(row, key, index) for key in ("dx", "dy", "dz")]
        standoff_mm = optional_positive_float(row, "standoff_mm", args.standoff_mm, index)
        travel_mm = optional_nonnegative_float(row, "travel_mm", args.travel_mm, index)

        specs.append(
            build_pose_spec(
                name,
                contact,
                approach,
                standoff_mm,
                travel_mm,
                reference_up=list(args.reference_up),
            )
        )
    return specs


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="CSV with name,x,y,z,dx,dy,dz columns")
    parser.add_argument("--output", required=True, help="output JSON file for visualization/preflight scripts")
    parser.add_argument("--reference-up", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("UX", "UY", "UZ"))
    parser.add_argument("--standoff-mm", type=float, default=20.0)
    parser.add_argument("--travel-mm", type=float, default=5.0)
    args = parser.parse_args()

    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")

    try:
        rows = load_rows(args.input)
        if not rows:
            raise SystemExit(f"no data rows in: {args.input}")
        specs = build_plan(rows, args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(specs, indent=2) + "\n")

    print(f"loaded CSV rows: {len(rows)}")
    print(f"generated pose specs: {len(specs)}")
    print(f"saved: {output}")
    for spec in specs:
        approach = spec["approach_vector"]
        print(
            f"  {spec['name']}: contact={spec['contact']} "
            f"approach=[{approach[0]:.6f}, {approach[1]:.6f}, {approach[2]:.6f}]"
        )


if __name__ == "__main__":
    main()
