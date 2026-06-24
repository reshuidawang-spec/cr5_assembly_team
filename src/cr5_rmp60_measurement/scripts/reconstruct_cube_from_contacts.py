#!/usr/bin/env python3
"""Fit simple cube planes from vertical and cross-stylus contact CSV files."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from plane_fit_utils import fit_plane, orient_normal


def read_rows(path):
    """Read rows."""
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def vertical_points(rows):
    """Vertical points."""
    points = []
    for row in rows:
        if all(row.get(field) not in (None, "") for field in ("tip_x_est", "tip_y_est", "tip_z_est")):
            points.append([float(row["tip_x_est"]), float(row["tip_y_est"]), float(row["tip_z_est"])])
    return points


def cross_surface_points(rows, point_source):
    """Cross surface points."""
    points = []
    fields = (
        ("surface_x_est", "surface_y_est", "surface_z_est")
        if point_source == "nominal"
        else ("calibrated_surface_x", "calibrated_surface_y", "calibrated_surface_z")
    )
    for row in rows:
        if all(row.get(field) not in (None, "") for field in fields):
            points.append([float(row[field]) for field in fields])
    return points


def plane_angle_deg(a, b):
    """Plane angle deg."""
    na = np.asarray(a["normal"], dtype=float)
    nb = np.asarray(b["normal"], dtype=float)
    value = float(np.dot(na, nb) / (np.linalg.norm(na) * np.linalg.norm(nb)))
    value = max(-1.0, min(1.0, abs(value)))
    return math.degrees(math.acos(value))


def plane_distance_at_point(plane, point):
    """Plane distance at point."""
    normal = np.asarray(plane["normal"], dtype=float)
    return float(np.dot(normal, np.asarray(point, dtype=float)) + plane["d"])


def print_plane(name, plane):
    """Print plane."""
    n = plane["normal"]
    c = plane["centroid_mm"]
    print(f"{name}:")
    print(f"  points: {plane['point_count']}")
    print(f"  centroid_mm: [{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}]")
    print(f"  normal: [{n[0]:.6f}, {n[1]:.6f}, {n[2]:.6f}]")
    print(f"  rms_residual_mm: {plane['rms_residual_mm']:.4f}")
    print(f"  max_abs_residual_mm: {plane['max_abs_residual_mm']:.4f}")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--top-csv", required=True, help="vertical probing CSV with tip_x/y/z_est")
    parser.add_argument("--side-csv", required=True, help="cross probing CSV with surface_x/y/z_est")
    parser.add_argument("--side-point-source", choices=("nominal", "calibrated"), default="nominal")
    parser.add_argument("--top-preferred-normal", nargs=3, type=float, default=[0.0, 0.0, 1.0])
    parser.add_argument("--side-preferred-normal", nargs=3, type=float, default=[-1.0, 0.0, 0.0])
    parser.add_argument("--json-output", help="write reconstruction JSON")
    args = parser.parse_args()

    top = orient_normal(fit_plane(vertical_points(read_rows(args.top_csv))), args.top_preferred_normal)
    side = orient_normal(
        fit_plane(cross_surface_points(read_rows(args.side_csv), args.side_point_source)),
        args.side_preferred_normal,
    )
    angle = plane_angle_deg(top, side)
    top_to_side_centroid_signed_mm = plane_distance_at_point(top, side["centroid_mm"])
    side_to_top_centroid_signed_mm = plane_distance_at_point(side, top["centroid_mm"])

    result = {
        "top_csv": args.top_csv,
        "side_csv": args.side_csv,
        "side_point_source": args.side_point_source,
        "top_plane": top,
        "side_plane": side,
        "plane_angle_deg": angle,
        "top_to_side_centroid_signed_mm": top_to_side_centroid_signed_mm,
        "side_to_top_centroid_signed_mm": side_to_top_centroid_signed_mm,
    }

    print_plane("top_plane", top)
    print_plane("side_plane", side)
    print(f"plane_angle_deg: {angle:.4f}")
    print(f"top_to_side_centroid_signed_mm: {top_to_side_centroid_signed_mm:.4f}")
    print(f"side_to_top_centroid_signed_mm: {side_to_top_centroid_signed_mm:.4f}")

    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        print(f"saved: {output}")


if __name__ == "__main__":
    main()
