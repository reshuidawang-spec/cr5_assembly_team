#!/usr/bin/env python3
"""Convert CR5 flange poses to nominal cross-stylus ball centres."""
import argparse
import csv
import json
import math
from pathlib import Path

import yaml

from geometry_utils import add, normalize, scale


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_GEOMETRY = PROJECT_DIR / "config/cross_probe_geometry.yaml"


def load_geometry(path):
    """Load cross-probe geometry parameters from a YAML file."""
    with Path(path).open() as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"invalid geometry file: {path}")
    return data


def _rot_x(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rot_y(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rot_z(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


_AXIS_ROT = {"x": _rot_x, "y": _rot_y, "z": _rot_z}


def mat_mult(a, b):
    """Multiply two 3x3 matrices."""
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(matrix, vector):
    """Multiply a 3x3 matrix by a 3-element vector."""
    return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]


def euler_to_matrix(sequence, angles_deg):
    """Convert Euler angles to a 3x3 rotation matrix.

    Lowercase sequences are extrinsic. The default `xyz` matches the current
    project convention used for CR5 pose review: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    Uppercase sequences are intrinsic.
    """
    angles_rad = [math.radians(angle) for angle in angles_deg]
    matrices = [_AXIS_ROT[axis.lower()](angle) for axis, angle in zip(sequence, angles_rad)]
    if sequence.isupper():
        result = matrices[0]
        for matrix in matrices[1:]:
            result = mat_mult(result, matrix)
        return result
    result = matrices[-1]
    for matrix in reversed(matrices[:-1]):
        result = mat_mult(result, matrix)
    return result


def branch_map(geometry):
    """Build a dictionary mapping branch names to their geometry config entries."""
    branches = geometry.get("branches", [])
    if not isinstance(branches, list):
        raise ValueError("geometry branches must be a list")
    result = {}
    for branch in branches:
        name = branch.get("name")
        if not name:
            raise ValueError("branch without name")
        result[name] = branch
    return result


def nominal_branch_origin_mm(geometry):
    """Compute the nominal flange-to-branch-origin offset along the Z axis."""
    derived = geometry.get("derived_nominal", {})
    if "flange_to_branch_origin_mm" in derived:
        return [0.0, 0.0, float(derived["flange_to_branch_origin_mm"])]
    adapter = float(geometry["flange_adapter"]["effective_axial_length_mm"])
    body = float(geometry["probe_body"]["length_mm"])
    return [0.0, 0.0, adapter + body]


def branch_local_offset_mm(geometry, branch_name):
    """Return the 3D offset of a named branch in the flange coordinate frame."""
    branches = branch_map(geometry)
    if branch_name not in branches:
        raise ValueError(f"unknown branch {branch_name!r}; choices: {', '.join(sorted(branches))}")
    branch = branches[branch_name]
    offset = branch.get("nominal_offset_mm")
    if not isinstance(offset, list) or len(offset) != 3:
        raise ValueError(f"branch {branch_name!r} has invalid nominal_offset_mm")
    return add(nominal_branch_origin_mm(geometry), [float(v) for v in offset])


def ball_radius_mm(geometry):
    """Extract the stylus ball radius from the geometry config."""
    stylus = geometry.get("stylus", {})
    if "ball_radius_mm" in stylus:
        return float(stylus["ball_radius_mm"])
    if "ball_diameter_mm" in stylus:
        return 0.5 * float(stylus["ball_diameter_mm"])
    raise ValueError("geometry stylus must define ball_radius_mm or ball_diameter_mm")


def compute_branch_point(geometry, pose, branch_name, euler_sequence="xyz", approach=None):
    """Compute the ball-centre position of a probe branch given a flange pose."""
    if len(pose) != 6:
        raise ValueError("pose must contain x y z rx ry rz")
    flange_position = [float(v) for v in pose[:3]]
    rotation = euler_to_matrix(euler_sequence, [float(v) for v in pose[3:6]])
    local_offset = branch_local_offset_mm(geometry, branch_name)
    offset_base = mat_vec(rotation, local_offset)
    ball_center = add(flange_position, offset_base)
    result = {
        "branch": branch_name,
        "flange_pose_mm_deg": [float(v) for v in pose],
        "euler_sequence": euler_sequence,
        "local_ball_center_offset_mm": local_offset,
        "ball_center_mm": ball_center,
        "ball_radius_mm": ball_radius_mm(geometry),
        "convention_warning": (
            "default euler sequence is current ROS RPY candidate xyz; "
            "verify against CR5 GetPose before using for real calibration"
        ),
    }
    if approach is not None:
        approach_unit = normalize([float(v) for v in approach], "approach vector")
        result["approach_vector"] = approach_unit
        result["surface_contact_estimate_mm"] = add(
            ball_center,
            scale(approach_unit, result["ball_radius_mm"]),
        )
    return result


def print_result(result):
    """Print a human-readable summary of a branch-point computation result."""
    center = result["ball_center_mm"]
    print(f"branch: {result['branch']}")
    print(f"euler_sequence: {result['euler_sequence']}")
    print(
        "local_ball_center_offset_mm: "
        f"[{result['local_ball_center_offset_mm'][0]:.4f}, "
        f"{result['local_ball_center_offset_mm'][1]:.4f}, "
        f"{result['local_ball_center_offset_mm'][2]:.4f}]"
    )
    print(f"ball_center_mm: [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}]")
    print(f"ball_radius_mm: {result['ball_radius_mm']:.4f}")
    if "surface_contact_estimate_mm" in result:
        contact = result["surface_contact_estimate_mm"]
        print(f"surface_contact_estimate_mm: [{contact[0]:.4f}, {contact[1]:.4f}, {contact[2]:.4f}]")
    print("warning: verify CR5 rx/ry/rz convention before using these points for final calibration")


def read_csv_rows(path):
    """Read CSV file and return rows as a list of dicts."""
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def pose_from_row(row):
    """Extract a 6-element flange pose list from a CSV row dict."""
    fields = ["flange_x", "flange_y", "flange_z", "rx", "ry", "rz"]
    missing = [field for field in fields if row.get(field) in (None, "")]
    if missing:
        raise ValueError(f"CSV row missing pose fields: {missing}")
    return [float(row[field]) for field in fields]


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--branch", default="z", help="branch name from geometry config")
    parser.add_argument("--pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--input-csv", help="CSV with flange_x/flange_y/flange_z/rx/ry/rz columns")
    parser.add_argument("--row-index", type=int, default=0, help="zero-based CSV data row index")
    parser.add_argument("--approach", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--json-output", help="write result JSON")
    args = parser.parse_args()

    if args.pose is None and args.input_csv is None:
        raise SystemExit("provide --pose or --input-csv")
    if args.pose is not None and args.input_csv is not None:
        raise SystemExit("provide only one of --pose or --input-csv")

    try:
        geometry = load_geometry(args.geometry)
        pose = args.pose
        row_context = None
        if args.input_csv:
            rows = read_csv_rows(args.input_csv)
            if not 0 <= args.row_index < len(rows):
                raise ValueError(f"--row-index out of range; CSV rows: {len(rows)}")
            row_context = rows[args.row_index]
            pose = pose_from_row(row_context)
        result = compute_branch_point(
            geometry,
            pose,
            args.branch,
            euler_sequence=args.euler_sequence,
            approach=args.approach,
        )
        if row_context is not None:
            result["source_csv"] = args.input_csv
            result["source_row_index"] = args.row_index
            result["source_cycle"] = row_context.get("cycle", "")
            result["source_timestamp"] = row_context.get("timestamp", "")
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc

    if args.json or args.json_output:
        text = json.dumps(result, indent=2)
        if args.json:
            print(text)
        if args.json_output:
            output = Path(args.json_output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(text + "\n")
            print(f"saved: {output}")
        return

    print_result(result)


if __name__ == "__main__":
    main()
