#!/usr/bin/env python3
"""Analyze repeated RMP60 probing CSV results."""
import argparse
import csv
import math
from pathlib import Path


def mean(values):
    """Mean."""
    return sum(values) / len(values)


def sample_std(values):
    """Sample std."""
    if len(values) < 2:
        return 0.0
    avg = mean(values)
    return math.sqrt(sum((v - avg) ** 2 for v in values) / (len(values) - 1))


def values(rows, field):
    """Values."""
    return [float(row[field]) for row in rows if row.get(field) not in (None, "")]


def print_stats(name, vals, unit="mm"):
    """Print stats."""
    if not vals:
        print(f"{name}: no data")
        return
    print(f"{name}:")
    print(f"  count: {len(vals)}")
    print(f"  mean: {mean(vals):.4f} {unit}")
    print(f"  sample std: {sample_std(vals):.4f} {unit}")
    print(f"  range: {min(vals):.4f} .. {max(vals):.4f} {unit}")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="data/repeat_probe_contacts.csv", help="CSV file to analyze")
    args = parser.parse_args()

    path = Path(args.input)
    if not path.exists():
        raise SystemExit(f"file not found: {path}")

    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise SystemExit(f"no rows in: {path}")

    print(f"file: {path}")
    print(f"rows: {len(rows)}")

    required = {"flange_x", "flange_y", "flange_z", "tip_z_est"}
    missing = sorted(required - set(rows[0].keys()))
    if missing:
        raise SystemExit(f"missing required columns: {missing}")

    print_stats("trigger flange_x", values(rows, "flange_x"))
    print_stats("trigger flange_y", values(rows, "flange_y"))
    print_stats("trigger flange_z", values(rows, "flange_z"))
    print_stats("estimated tip_z", values(rows, "tip_z_est"))

    stop_fields = {"stop_flange_x", "stop_flange_y", "stop_flange_z"}
    if stop_fields.issubset(rows[0].keys()):
        over_x = [float(row["stop_flange_x"]) - float(row["flange_x"]) for row in rows]
        over_y = [float(row["stop_flange_y"]) - float(row["flange_y"]) for row in rows]
        over_z = [float(row["stop_flange_z"]) - float(row["flange_z"]) for row in rows]
        print_stats("stop overtravel_x", over_x)
        print_stats("stop overtravel_y", over_y)
        print_stats("stop overtravel_z", over_z)
    else:
        print("stop overtravel: no stop_flange_* columns; rerun repeat_probe_test.py with the current code")

    cycles = [row.get("cycle", "") for row in rows]
    if any(cycles):
        print(f"cycles recorded: {', '.join(cycles)}")


if __name__ == "__main__":
    main()
