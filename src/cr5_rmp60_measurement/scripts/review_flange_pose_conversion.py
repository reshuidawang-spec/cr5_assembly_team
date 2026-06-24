#!/usr/bin/env python3
"""Review candidate CR5 flange poses derived from rmp60_tip target poses."""
import argparse
import json
import math
from pathlib import Path

from generate_measurement_poses import orientation_from_approach
from moveit_utils import load_pose_specs


def quaternion_to_matrix(orientation):
    """Quaternion to matrix."""
    x = float(orientation["x"])
    y = float(orientation["y"])
    z = float(orientation["z"])
    w = float(orientation["w"])
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("orientation quaternion cannot be zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def matrix_to_rpy_deg(matrix):
    """Candidate ROS-style RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).."""
    # Candidate ROS-style RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    sy = -matrix[2][0]
    if abs(sy) < 1.0 - 1e-9:
        pitch = math.asin(sy)
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        pitch = math.copysign(math.pi / 2.0, sy)
        roll = math.atan2(-matrix[0][1], matrix[1][1])
        yaw = 0.0
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def mat_vec(matrix, vector):
    """Multiply a 3x3 matrix by a 3-element vector."""
    return [
        sum(matrix[row][col] * vector[col] for col in range(3))
        for row in range(3)
    ]


def add(a, b):
    """Return the element-wise sum of two equal-length vectors."""
    return [x + y for x, y in zip(a, b)]


def sub(a, b):
    """Sub."""
    return [x - y for x, y in zip(a, b)]


def norm(vector):
    """Norm."""
    return math.sqrt(sum(value * value for value in vector))


def dot(a, b):
    """Dot."""
    return sum(x * y for x, y in zip(a, b))


def normalize(vector, label):
    """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
    length = norm(vector)
    if length <= 1e-12:
        raise ValueError(f"{label} cannot be zero")
    return [value / length for value in vector]


def convert_tip_to_flange(tip_position_mm, orientation, tool_length_mm):
    """Convert tip to flange."""
    rotation = quaternion_to_matrix(orientation)
    tip_offset_base = mat_vec(rotation, [0.0, 0.0, tool_length_mm])
    flange_position_mm = sub(tip_position_mm, tip_offset_base)
    rpy_deg = matrix_to_rpy_deg(rotation)
    reconstructed_tip_mm = add(flange_position_mm, tip_offset_base)
    return {
        "flange_position_mm": flange_position_mm,
        "candidate_rpy_deg": rpy_deg,
        "candidate_movl_pose": flange_position_mm + rpy_deg,
        "reconstructed_tip_mm": reconstructed_tip_mm,
        "reconstruction_error_mm": norm(sub(reconstructed_tip_mm, tip_position_mm)),
        "tool_axis_base": [rotation[0][2], rotation[1][2], rotation[2][2]],
    }


def resolve_orientation(spec, reference_up):
    """Resolve orientation."""
    return spec.get("tip_orientation") or orientation_from_approach(
        spec["approach_vector"], reference_up=reference_up
    )


def review_spec(spec, args):
    """Review spec."""
    orientation = resolve_orientation(spec, args.reference_up)
    approach = normalize(spec["approach_vector"], "approach vector")
    tool_length_mm = (args.adapter_length + args.probe_body_length + args.stylus_length) * 1000.0
    results = {}
    for label, key in (("safe", "safe_position"), ("contact", "contact"), ("target", "target_position")):
        converted = convert_tip_to_flange(spec[key], orientation, tool_length_mm)
        axis = normalize(converted["tool_axis_base"], "tool axis")
        dot_value = max(-1.0, min(1.0, dot(axis, approach)))
        converted["tool_axis_vs_approach_angle_deg"] = math.degrees(math.acos(dot_value))
        results[label] = converted
    return {
        "name": spec.get("name", "probe_pose"),
        "tool_length_mm": tool_length_mm,
        "approach_vector": approach,
        "tip_orientation": orientation,
        "convention_warning": (
            "candidate_rpy_deg uses ROS-style RPY with R=Rz(yaw)*Ry(pitch)*Rx(roll); "
            "verify against CR5 GetPose before real execution"
        ),
        "poses": results,
    }


def print_review(review):
    """Print review."""
    print(f"{review['name']}:")
    print(f"  tool_length_mm: {review['tool_length_mm']:.3f}")
    print(
        "  tip_orientation_xyzw: "
        f"[{review['tip_orientation']['x']:.6f}, {review['tip_orientation']['y']:.6f}, "
        f"{review['tip_orientation']['z']:.6f}, {review['tip_orientation']['w']:.6f}]"
    )
    print("  convention: candidate ROS RPY; verify with CR5 GetPose before real execution")
    for label, converted in review["poses"].items():
        pose = converted["candidate_movl_pose"]
        print(
            f"  {label} candidate MovL: "
            f"x={pose[0]:.4f}, y={pose[1]:.4f}, z={pose[2]:.4f}, "
            f"rx={pose[3]:.4f}, ry={pose[4]:.4f}, rz={pose[5]:.4f}"
        )
        print(
            f"    reconstruction_error_mm={converted['reconstruction_error_mm']:.6f}, "
            f"tool_axis_vs_approach_angle_deg={converted['tool_axis_vs_approach_angle_deg']:.6f}"
        )


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="JSON file from generate_measurement_poses.py --json")
    parser.add_argument("--contact", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--approach", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--reference-up", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("UX", "UY", "UZ"))
    parser.add_argument("--standoff-mm", type=float, default=20.0)
    parser.add_argument("--travel-mm", type=float, default=5.0)
    parser.add_argument("--name", default="probe_pose")
    parser.add_argument("--unit", choices=("mm", "m"), default="mm")
    parser.add_argument("--adapter-length", type=float, default=0.0494)
    parser.add_argument("--probe-body-length", type=float, default=0.076)
    parser.add_argument("--stylus-length", type=float, default=0.075)
    parser.add_argument("--json-output", help="write full review JSON to this path")
    args = parser.parse_args()

    if args.unit != "mm":
        raise SystemExit("only --unit mm is supported for CR5 MovL candidate output")
    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")
    if args.adapter_length <= 0 or args.probe_body_length <= 0 or args.stylus_length <= 0:
        raise SystemExit("tool lengths must be positive")

    try:
        specs = load_pose_specs(args)
        reviews = [review_spec(spec, args) for spec in specs]
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    for review in reviews:
        print_review(review)

    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(reviews, indent=2) + "\n")
        print(f"saved: {output}")


if __name__ == "__main__":
    main()
