#!/usr/bin/env python3
"""Generate safe/target positions for arbitrary-direction probing."""
import argparse
import json
import math


def normalize(vector, label="vector"):
    """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
    norm = math.sqrt(sum(v * v for v in vector))
    if norm <= 1e-9:
        raise ValueError(f"{label} cannot be zero")
    return [v / norm for v in vector]


def add(a, b):
    """Return the element-wise sum of two equal-length vectors."""
    return [x + y for x, y in zip(a, b)]


def scale(v, s):
    """Return a vector scaled by a scalar value (element-wise multiplication)."""
    return [x * s for x in v]


def dot(a, b):
    """Dot."""
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    """Cross."""
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def matrix_to_quaternion(x_axis, y_axis, z_axis):
    """Matrix to quaternion."""
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    quat = normalize([qx, qy, qz, qw], "quaternion")
    return {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}


def orientation_from_approach(approach_vector, reference_up=None):
    """The mounted probe/stylus extends along rmp60_tip local +Z.."""
    approach = normalize(approach_vector, "approach vector")
    reference = normalize(reference_up or [0.0, 0.0, 1.0], "reference-up vector")
    # The mounted probe/stylus extends along rmp60_tip local +Z.
    z_axis = approach
    if abs(dot(reference, z_axis)) > 0.98:
        reference = [0.0, 1.0, 0.0]
    x_axis = normalize(cross(reference, z_axis), "orientation x axis")
    y_axis = cross(z_axis, x_axis)
    return matrix_to_quaternion(x_axis, y_axis, z_axis)


def build_pose_spec(name, contact, approach, standoff_mm, travel_mm, reference_up=None):
    """Build pose spec."""
    approach_vector = normalize(approach, "approach vector")
    return {
        "name": name,
        "contact": contact,
        "approach_vector": approach_vector,
        "safe_position": add(contact, scale(approach_vector, -standoff_mm)),
        "target_position": add(contact, scale(approach_vector, travel_mm)),
        "probe_axis": approach_vector,
        "tip_orientation": orientation_from_approach(approach_vector, reference_up=reference_up),
        "standoff_mm": standoff_mm,
        "travel_mm": travel_mm,
    }


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--contact", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--approach", nargs=3, type=float, required=True, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--reference-up", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("UX", "UY", "UZ"))
    parser.add_argument("--standoff-mm", type=float, default=20.0, help="distance before contact")
    parser.add_argument("--travel-mm", type=float, default=5.0, help="distance past contact along approach vector")
    parser.add_argument("--name", default="probe_pose", help="pose name")
    parser.add_argument("--json", action="store_true", help="print JSON instead of text")
    args = parser.parse_args()

    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")

    contact = list(args.contact)
    try:
        result = build_pose_spec(
            args.name,
            contact,
            list(args.approach),
            args.standoff_mm,
            args.travel_mm,
            reference_up=list(args.reference_up),
        )
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    if args.json:
        print(json.dumps(result, indent=2))
        return

    print(f"name: {args.name}")
    print(f"contact: [{contact[0]:.4f}, {contact[1]:.4f}, {contact[2]:.4f}]")
    approach = result["approach_vector"]
    safe_position = result["safe_position"]
    target_position = result["target_position"]
    orientation = result["tip_orientation"]
    print(f"approach_vector: [{approach[0]:.6f}, {approach[1]:.6f}, {approach[2]:.6f}]")
    print(f"safe_position: [{safe_position[0]:.4f}, {safe_position[1]:.4f}, {safe_position[2]:.4f}]")
    print(f"target_position: [{target_position[0]:.4f}, {target_position[1]:.4f}, {target_position[2]:.4f}]")
    print(f"probe_axis: [{approach[0]:.6f}, {approach[1]:.6f}, {approach[2]:.6f}]")
    print(
        "tip_orientation_xyzw: "
        f"[{orientation['x']:.6f}, {orientation['y']:.6f}, {orientation['z']:.6f}, {orientation['w']:.6f}]"
    )


if __name__ == "__main__":
    main()
