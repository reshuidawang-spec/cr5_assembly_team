#!/usr/bin/env python3
"""Validation helpers for operator-taught workpiece face probing."""
import math
from pathlib import Path

import yaml

PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SETUP_CONFIG = PROJECT_DIR / "config/workpiece_setups.yaml"


def load_setup_config(path):
    """Load setup config."""
    source = Path(path)
    if not source.exists():
        raise ValueError(f"workpiece setup config does not exist: {source}")
    with source.open() as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict) or not isinstance(data.get("setups", {}), dict):
        raise ValueError(f"invalid workpiece setup config: {source}")
    data.setdefault("version", 1)
    data.setdefault("setups", {})
    return data


def angle_delta_deg(actual, expected):
    """Angle delta deg."""
    return (float(actual) - float(expected) + 180.0) % 360.0 - 180.0


def pose_errors(actual, expected):
    """Pose errors."""
    if len(actual) != 6 or len(expected) != 6:
        raise ValueError("poses must contain six values")
    position_error = math.sqrt(sum((float(actual[i]) - float(expected[i])) ** 2 for i in range(3)))
    orientation_error = max(abs(angle_delta_deg(actual[i], expected[i])) for i in range(3, 6))
    return position_error, orientation_error


def approach_matches(actual, expected, tolerance=1e-6):
    """Approach matches."""
    return len(actual) == 3 and len(expected) == 3 and all(
        abs(float(actual[i]) - float(expected[i])) <= tolerance for i in range(3)
    )


def load_approved_face(args, approach):
    """Load approved face."""
    setup_id = getattr(args, "setup_id", "") or ""
    workpiece_id = getattr(args, "workpiece_id", "") or ""
    face_id = getattr(args, "face_id", "") or ""
    if not setup_id or not workpiece_id or not face_id:
        raise ValueError(
            "real probing requires --setup-id, --workpiece-id and --face-id "
            "from an approved taught workpiece face"
        )

    config = load_setup_config(getattr(args, "setup_config", DEFAULT_SETUP_CONFIG))
    setup = config["setups"].get(setup_id)
    if not isinstance(setup, dict):
        raise ValueError(f"unknown workpiece setup {setup_id!r}; teach and approve a face before real probing")
    if setup.get("workpiece_id") != workpiece_id:
        raise ValueError(f"setup {setup_id!r} is registered for workpiece {setup.get('workpiece_id')!r}")
    face = (setup.get("faces") or {}).get(face_id)
    if not isinstance(face, dict):
        raise ValueError(f"unknown face {face_id!r} in setup {setup_id!r}")
    if face.get("status") != "approved":
        raise ValueError(f"face {face_id!r} in setup {setup_id!r} is not approved for real probing")
    if face.get("standard_pose") != getattr(args, "standard_pose", None):
        raise ValueError("requested --standard-pose does not match the approved face record")
    if face.get("branch") != getattr(args, "branch", None):
        raise ValueError("requested --branch does not match the approved face record")
    if not approach_matches(approach, face.get("approach", [])):
        raise ValueError("requested --approach does not match the approved face record")

    for field in ("max_search_mm", "max_speed", "retract_mm", "safe_start_pose"):
        if field not in face:
            raise ValueError(
                f"approved face {setup_id!r}/{face_id!r} is missing required field {field!r}; "
                "re-teach the face or repair config/workpiece_setups.yaml"
            )

    approved_search = float(face["max_search_mm"])
    if float(args.distance_mm) > approved_search + 1e-6:
        raise ValueError(
            f"requested --distance-mm {float(args.distance_mm):g} exceeds approved "
            f"max_search_mm {approved_search:g}"
        )
    approved_speed = int(face["max_speed"])
    if int(args.speed) > approved_speed:
        raise ValueError(f"requested --speed {int(args.speed)} exceeds approved max_speed {approved_speed}")
    approved_retract = float(face["retract_mm"])
    if abs(float(args.retract_mm) - approved_retract) > 1e-6:
        raise ValueError(
            f"requested --retract-mm {float(args.retract_mm):g} must equal approved "
            f"retract_mm {approved_retract:g}"
        )
    safe_pose = face.get("safe_start_pose")
    if not isinstance(safe_pose, list) or len(safe_pose) != 6:
        raise ValueError("approved face has no valid safe_start_pose")
    return face


def require_registered_start_pose(face, current_pose, position_tolerance_mm=0.2, orientation_tolerance_deg=0.5):
    """Require registered start pose."""
    return require_pose_match(
        current_pose,
        face["safe_start_pose"],
        position_tolerance_mm=position_tolerance_mm,
        orientation_tolerance_deg=orientation_tolerance_deg,
        label="approved taught safe start pose",
    )


def require_pose_match(actual_pose, expected_pose, position_tolerance_mm=0.2, orientation_tolerance_deg=0.5, label="pose"):
    """Require pose match."""
    position_error, orientation_error = pose_errors(actual_pose, expected_pose)
    if position_error > position_tolerance_mm or orientation_error > orientation_tolerance_deg:
        raise ValueError(
            f"robot is not at the {label}; "
            f"position error {position_error:.4f} mm, orientation error {orientation_error:.4f} deg"
        )
    return position_error, orientation_error


def safe_pose_at_tangent_offsets(face, y_offset_mm, z_offset_mm):
    """Safe pose at tangent offsets."""
    bounds = face.get("collection_bounds_mm")
    if not isinstance(bounds, dict):
        raise ValueError("approved face has no registered collection_bounds_mm")
    y_bounds = bounds.get("y_offset")
    z_bounds = bounds.get("z_offset")
    if not isinstance(y_bounds, list) or len(y_bounds) != 2:
        raise ValueError("approved face has invalid y_offset collection bounds")
    if not isinstance(z_bounds, list) or len(z_bounds) != 2:
        raise ValueError("approved face has invalid z_offset collection bounds")
    if not float(y_bounds[0]) - 1e-6 <= float(y_offset_mm) <= float(y_bounds[1]) + 1e-6:
        raise ValueError(f"Y offset {y_offset_mm:g} mm is outside approved collection bounds")
    if not float(z_bounds[0]) - 1e-6 <= float(z_offset_mm) <= float(z_bounds[1]) + 1e-6:
        raise ValueError(f"Z offset {z_offset_mm:g} mm is outside approved collection bounds")
    tangent_vectors = face.get("collection_tangent_vectors")
    if not isinstance(tangent_vectors, dict):
        raise ValueError("approved face has no registered collection_tangent_vectors")
    y_vector = tangent_vectors.get("y_offset")
    z_vector = tangent_vectors.get("z_offset")
    if not isinstance(y_vector, list) or len(y_vector) != 3:
        raise ValueError("approved face has invalid y_offset tangent vector")
    if not isinstance(z_vector, list) or len(z_vector) != 3:
        raise ValueError("approved face has invalid z_offset tangent vector")
    approach = face.get("approach")
    if not isinstance(approach, list) or len(approach) != 3:
        raise ValueError("approved face has invalid approach vector")
    normal_length = math.sqrt(sum(float(value) ** 2 for value in approach))
    if normal_length <= 1e-12:
        raise ValueError("approved face approach vector cannot be zero")
    approach_unit = [float(value) / normal_length for value in approach]
    y_vector = [float(value) for value in y_vector]
    z_vector = [float(value) for value in z_vector]
    for label, vector in (("y_offset", y_vector), ("z_offset", z_vector)):
        length = math.sqrt(sum(value * value for value in vector))
        if abs(length - 1.0) > 1e-6:
            raise ValueError(f"approved face {label} tangent vector must be unit length")
        if abs(sum(vector[i] * approach_unit[i] for i in range(3))) > 1e-6:
            raise ValueError(f"approved face {label} tangent vector must be perpendicular to approach")
    if abs(sum(y_vector[i] * z_vector[i] for i in range(3))) > 1e-6:
        raise ValueError("approved face tangent vectors must be perpendicular")
    pose = list(face["safe_start_pose"])
    for index in range(3):
        pose[index] += y_vector[index] * float(y_offset_mm) + z_vector[index] * float(z_offset_mm)
    return pose
