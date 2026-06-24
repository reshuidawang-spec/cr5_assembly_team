#!/usr/bin/env python3
"""Validate which Euler convention matches CR5 rx/ry/rz readings."""
import argparse
import json
import math
import re
import time
from pathlib import Path

import rclpy
from dobot_msgs_v4.srv import GetPose
from rclpy.node import Node

from generate_measurement_poses import matrix_to_quaternion, normalize


CONVENTIONS = [
    ("xyz", "extrinsic X-Y-Z; ROS RPY candidate, R=Rz*Ry*Rx"),
    ("zyx", "extrinsic Z-Y-X"),
    ("xzy", "extrinsic X-Z-Y"),
    ("yxz", "extrinsic Y-X-Z"),
    ("yzx", "extrinsic Y-Z-X"),
    ("zxy", "extrinsic Z-X-Y"),
    ("XYZ", "intrinsic X-Y-Z"),
    ("ZYX", "intrinsic Z-Y-X"),
    ("XZY", "intrinsic X-Z-Y"),
    ("YXZ", "intrinsic Y-X-Z"),
    ("YZX", "intrinsic Y-Z-X"),
    ("ZXY", "intrinsic Z-X-Y"),
]


def parse_pose(text):
    """Parse pose."""
    match = re.search(r"\{([^}]*)\}", text)
    if not match:
        raise ValueError(f"cannot parse pose from: {text}")
    values = [float(item.strip()) for item in match.group(1).split(",")]
    if len(values) != 6:
        raise ValueError(f"expected 6 pose values, got {len(values)}: {text}")
    return values


def angle_between(a, b):
    """Angle between."""
    a = normalize(a, "axis a")
    b = normalize(b, "axis b")
    value = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    return math.degrees(math.acos(value))


def _rot_x(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def _rot_y(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _rot_z(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


_AXIS_ROT = {"x": _rot_x, "y": _rot_y, "z": _rot_z}


def _mat_mult(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_vec(matrix, vector):
    return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]


def euler_to_matrix(sequence, angles_deg):
    """Convert Euler angles to a 3x3 rotation matrix (scipy-compatible convention).

    Intrinsic (uppercase): R = R_first @ R_second @ R_third.
    Extrinsic (lowercase): R = R_third @ R_second @ R_first.
    """
    angles_rad = [math.radians(a) for a in angles_deg]
    matrices = [_AXIS_ROT[axis.lower()](angle) for axis, angle in zip(sequence, angles_rad)]
    if sequence.isupper():
        result = matrices[0]
        for m in matrices[1:]:
            result = _mat_mult(result, m)
    else:
        result = matrices[-1]
        for m in reversed(matrices[:-1]):
            result = _mat_mult(result, m)
    return result


def rotation_matrix_to_quat_xyzw(matrix):
    """Convert a 3x3 rotation matrix to a [x, y, z, w] quaternion list."""
    cols = [[matrix[0][i], matrix[1][i], matrix[2][i]] for i in range(3)]
    q = matrix_to_quaternion(cols[0], cols[1], cols[2])
    return [q["x"], q["y"], q["z"], q["w"]]


def analyze_pose(pose, expected_axis=None, local_tool_axis=None):
    """Analyze pose."""
    local_axis = normalize(local_tool_axis or [0.0, 0.0, 1.0], "local tool axis")
    results = []
    angles = pose[3:6]
    for sequence, description in CONVENTIONS:
        matrix = euler_to_matrix(sequence, angles)
        quat = rotation_matrix_to_quat_xyzw(matrix)
        tool_axis = _mat_vec(matrix, local_axis)
        angle = None
        if expected_axis is not None:
            angle = angle_between(tool_axis, expected_axis)
        results.append(
            {
                "sequence": sequence,
                "description": description,
                "quat_xyzw": quat,
                "tool_axis_base": tool_axis,
                "axis_error_deg": angle,
                "rotation_matrix": matrix,
            }
        )
    if expected_axis is not None:
        results.sort(key=lambda item: item["axis_error_deg"])
    return results


class PoseReader(Node):
    def __init__(self):
        super().__init__("cr5_pose_convention_validator")
        self.pose_cli = self.create_client(GetPose, "/dobot_bringup_ros2/srv/GetPose")

    def read_pose(self, timeout_sec):
        """Read pose."""
        if not self.pose_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/dobot_bringup_ros2/srv/GetPose is not available")
        req = GetPose.Request()
        req.user = 0
        req.tool = 0
        future = self.pose_cli.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return parse_pose(future.result().robot_return)
        raise TimeoutError("GetPose timeout")


def read_robot_pose(timeout_sec):
    """Read robot pose."""
    rclpy.init()
    node = PoseReader()
    try:
        return node.read_pose(timeout_sec)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def print_results(pose, results, expected_axis):
    """Print results."""
    print(f"source pose: x={pose[0]:.4f}, y={pose[1]:.4f}, z={pose[2]:.4f}, rx={pose[3]:.4f}, ry={pose[4]:.4f}, rz={pose[5]:.4f}")
    if expected_axis is not None:
        print(f"expected tool axis in base: [{expected_axis[0]:.6f}, {expected_axis[1]:.6f}, {expected_axis[2]:.6f}]")
    print("candidate conventions:")
    for item in results:
        axis = item["tool_axis_base"]
        quat = item["quat_xyzw"]
        angle_text = "N/A" if item["axis_error_deg"] is None else f"{item['axis_error_deg']:.6f} deg"
        print(
            f"  {item['sequence']:>3}  axis_error={angle_text}  "
            f"tool_axis=[{axis[0]:.6f}, {axis[1]:.6f}, {axis[2]:.6f}]  "
            f"quat=[{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]"
        )
        print(f"       {item['description']}")
    print("note: choose the convention only after checking a known physical tool direction on the real CR5.")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"), help="offline CR5 pose to analyze")
    parser.add_argument("--expected-axis", nargs=3, type=float, metavar=("DX", "DY", "DZ"), help="known physical direction of Link6/tool local +Z in base")
    parser.add_argument("--local-tool-axis", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("LX", "LY", "LZ"))
    parser.add_argument("--timeout", type=float, default=5.0, help="GetPose wait timeout when --pose is omitted")
    parser.add_argument("--json-output", help="write full analysis JSON to this path")
    args = parser.parse_args()

    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")

    pose = list(args.pose) if args.pose is not None else read_robot_pose(args.timeout)
    expected_axis = None
    if args.expected_axis is not None:
        try:
            expected_axis = normalize(list(args.expected_axis), "expected axis")
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc

    try:
        results = analyze_pose(pose, expected_axis=expected_axis, local_tool_axis=list(args.local_tool_axis))
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    print_results(pose, results, expected_axis)

    if args.json_output:
        output = Path(args.json_output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(
                {
                    "pose": pose,
                    "expected_axis": expected_axis,
                    "local_tool_axis": list(args.local_tool_axis),
                    "results": results,
                },
                indent=2,
            )
            + "\n"
        )
        print(f"saved: {output}")


if __name__ == "__main__":
    main()
