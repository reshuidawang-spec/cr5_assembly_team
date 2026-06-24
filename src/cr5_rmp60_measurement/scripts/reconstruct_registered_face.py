#!/usr/bin/env python3
"""Fit one registered workpiece face from cross-probe contact samples."""
import argparse
import csv
import json
import math
from pathlib import Path

from plane_fit_utils import fit_plane as fit_least_squares_plane
from plane_fit_utils import orient_normal


def read_points(path, setup_id, face_id, point_source):
    """Read points."""
    points = []
    rows = []
    prefix = "surface" if point_source == "nominal" else "calibrated_surface"
    fields = tuple(f"{prefix}_{axis}{'_est' if point_source == 'nominal' else ''}" for axis in ("x", "y", "z"))
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            if row.get("setup_id") != setup_id or row.get("face_id") != face_id:
                continue
            if not all(row.get(field) not in (None, "") for field in fields):
                continue
            points.append([float(row[field]) for field in fields])
            rows.append(
                {
                    "sample_id": row.get("sample_id", ""),
                    "tangent_y_offset_mm": row.get("tangent_y_offset_mm", ""),
                    "tangent_z_offset_mm": row.get("tangent_z_offset_mm", ""),
                    "surface_point_mm": points[-1],
                    "stop_overtravel_mm": row.get("stop_overtravel_along_approach_mm", ""),
                }
            )
    return points, rows


def fit_plane(points, preferred_normal):
    """Fit a plane to 3D points via SVD, returning normal, centroid, and residuals."""
    plane = fit_least_squares_plane(
        points,
        min_points_message="at least 3 registered contact points are required to fit a face",
    )
    return orient_normal(plane, preferred_normal)


def statistics(values):
    """Statistics."""
    if not values:
        return None
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / max(1, len(values) - 1)
    return {
        "count": len(values),
        "mean": mean,
        "sample_std": math.sqrt(variance),
        "min": min(values),
        "max": max(values),
    }


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="registered cross-probe contacts CSV")
    parser.add_argument("--setup-id", required=True)
    parser.add_argument("--face-id", required=True)
    parser.add_argument("--point-source", choices=("nominal", "calibrated"), default="nominal")
    parser.add_argument("--preferred-normal", nargs=3, type=float, default=[-1.0, 0.0, 0.0])
    parser.add_argument("--json-output")
    args = parser.parse_args()

    points, rows = read_points(args.input, args.setup_id, args.face_id, args.point_source)
    plane = fit_plane(points, args.preferred_normal)
    overtravel_stats = statistics(
        [float(row["stop_overtravel_mm"]) for row in rows if row["stop_overtravel_mm"] not in ("", None)]
    )
    result = {
        "setup_id": args.setup_id,
        "face_id": args.face_id,
        "input": args.input,
        "point_source": args.point_source,
        "point_model": (
            "surface_x/y/z_est using nominal cross-probe geometry; absolute plane location is provisional until tool calibration"
            if args.point_source == "nominal"
            else "calibrated_surface_x/y/z using the supplied calibrated cross-probe conversion"
        ),
        "plane": plane,
        "stop_overtravel_statistics_mm": overtravel_stats,
        "samples": rows,
    }
    normal = plane["normal"]
    centroid = plane["centroid_mm"]
    print(f"registered face: {args.setup_id}/{args.face_id}")
    print(f"points: {plane['point_count']}")
    print(f"centroid_mm: [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}]")
    print(f"normal: [{normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f}]")
    print(f"rms_residual_mm: {plane['rms_residual_mm']:.4f}")
    print(f"max_abs_residual_mm: {plane['max_abs_residual_mm']:.4f}")
    if overtravel_stats:
        print(
            "stop_overtravel_mm: "
            f"mean={overtravel_stats['mean']:.4f}, "
            f"std={overtravel_stats['sample_std']:.4f}, "
            f"range={overtravel_stats['min']:.4f}..{overtravel_stats['max']:.4f}"
        )
    if args.point_source == "nominal":
        print("note: absolute plane location remains provisional until cross-probe geometry calibration")
    else:
        print("note: absolute plane location depends on the supplied calibration quality and traceability")
    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w") as f:
            json.dump(result, f, indent=2)
        print(f"saved: {output}")


if __name__ == "__main__":
    main()
