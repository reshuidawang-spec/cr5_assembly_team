#!/usr/bin/env python3
"""Fit one plane from vertical probing CSV contacts.

The flatness value reported here is the signed-residual span around a
least-squares plane. It is a practical process-control estimate, not a full
ISO minimum-zone flatness implementation.
"""
import argparse
import csv
import json
from pathlib import Path

from plane_fit_utils import fit_plane, orient_normal


POINT_FIELDS = {
    "tip": ("tip_x_est", "tip_y_est", "tip_z_est"),
    "flange": ("flange_x", "flange_y", "flange_z"),
}


def read_rows(path):
    """Read rows."""
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def load_points(rows, point_source):
    """Load points."""
    fields = POINT_FIELDS[point_source]
    points = []
    names = []
    for index, row in enumerate(rows, start=1):
        if not all(row.get(field) not in (None, "") for field in fields):
            continue
        points.append([float(row[field]) for field in fields])
        names.append(row.get("cycle") or row.get("name") or f"row_{index}")
    return names, points


def write_residuals(path, names, points, residuals):
    """Write residuals."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["name", "x", "y", "z", "plane_residual_mm"])
        writer.writeheader()
        for name, point, residual in zip(names, points, residuals):
            writer.writerow(
                {
                    "name": name,
                    "x": f"{point[0]:.4f}",
                    "y": f"{point[1]:.4f}",
                    "z": f"{point[2]:.4f}",
                    "plane_residual_mm": f"{residual:.6f}",
                }
            )


def print_plane(plane):
    """Print plane."""
    n = plane["normal"]
    c = plane["centroid_mm"]
    print(f"points: {plane['point_count']}")
    print(f"centroid_mm: [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]")
    print(f"normal: [{n[0]:.6f}, {n[1]:.6f}, {n[2]:.6f}]")
    print(f"plane_d: {plane['d']:.6f}")
    print(f"rms_residual_mm: {plane['rms_residual_mm']:.6f}")
    print(f"max_abs_residual_mm: {plane['max_abs_residual_mm']:.6f}")
    print(f"flatness_estimate_ls_mm: {plane['residual_span_mm']:.6f}")
    print("note: flatness_estimate_ls_mm is residual max-min around the least-squares plane")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="vertical probing CSV with estimated contact points")
    parser.add_argument("--point-source", choices=sorted(POINT_FIELDS), default="tip")
    parser.add_argument("--preferred-normal", nargs=3, type=float, default=[0.0, 0.0, 1.0])
    parser.add_argument("--json-output", help="write plane reconstruction JSON")
    parser.add_argument("--residual-output", help="write per-point residual CSV")
    args = parser.parse_args()

    rows = read_rows(args.input)
    names, points = load_points(rows, args.point_source)
    plane = orient_normal(fit_plane(points), args.preferred_normal)
    result = {
        "input": args.input,
        "point_source": args.point_source,
        "preferred_normal": args.preferred_normal,
        "flatness_method": "least_squares_residual_span",
        "plane": plane,
    }

    print_plane(plane)

    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(f"saved: {output}")

    if args.residual_output:
        write_residuals(args.residual_output, names, points, plane["residuals_mm"])
        print(f"saved: {args.residual_output}")


if __name__ == "__main__":
    main()
