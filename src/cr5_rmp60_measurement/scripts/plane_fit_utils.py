#!/usr/bin/env python3
"""Shared least-squares plane fitting helpers."""
import math

import numpy as np


def fit_plane(points, min_points_message="at least 3 points are required to fit a plane"):
    """Fit a plane to 3D points via SVD, returning normal, centroid, and residuals."""
    if len(points) < 3:
        raise ValueError(min_points_message)
    matrix = np.asarray(points, dtype=float)
    centroid = matrix.mean(axis=0)
    _, singular_values, vh = np.linalg.svd(matrix - centroid)
    if singular_values[1] <= max(1e-9, singular_values[0] * 1e-9):
        raise ValueError("contact points are collinear or duplicated; cannot fit a plane")
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    d = -float(np.dot(normal, centroid))
    residuals = matrix @ normal + d
    return {
        "point_count": len(points),
        "singular_values_mm": singular_values.tolist(),
        "centroid_mm": centroid.tolist(),
        "normal": normal.tolist(),
        "d": d,
        "rms_residual_mm": float(math.sqrt(np.mean(residuals * residuals))),
        "max_abs_residual_mm": float(np.max(np.abs(residuals))),
        "residual_span_mm": float(np.max(residuals) - np.min(residuals)),
        "residuals_mm": residuals.tolist(),
    }


def orient_normal(plane, preferred):
    """Flip the plane normal to align with a preferred direction if needed."""
    normal = np.asarray(plane["normal"], dtype=float)
    preferred = np.asarray(preferred, dtype=float)
    if np.linalg.norm(preferred) <= 1e-12:
        return plane
    if float(np.dot(normal, preferred)) >= 0:
        return plane
    flipped = dict(plane)
    flipped["normal"] = (-normal).tolist()
    flipped["d"] = -float(plane["d"])
    flipped["residuals_mm"] = [-value for value in plane["residuals_mm"]]
    return flipped
