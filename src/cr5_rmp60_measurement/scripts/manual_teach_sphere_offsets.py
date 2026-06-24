#!/usr/bin/env python3
"""Record manual standard-sphere touches and update branch offset seeds.

The operator manually brings one ruby ball into contact with the calibration
sphere. This script records the live flange pose at DI1 trigger and estimates
the branch local ball offset by projecting the current seed ball centre onto
the known sphere-contact shell.

It does not command robot motion.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy

from auto_calibrate_five_branch_sphere import (
    DEFAULT_REFERENCE_FIT,
    POSE_NAMES,
    add,
    branch_direction,
    load_reference_fit,
    norm,
    normalize,
    scale,
    sub,
    transpose_mat_vec,
)
from cross_probe_model import DEFAULT_GEOMETRY, branch_local_offset_mm, euler_to_matrix, load_geometry, mat_vec
from jog_and_record_contacts import current_pose, pose_fields, vector_fields
from probe_touch import PROBE_SPIN_TIMEOUT_SEC, ProbeTouch


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_DIR / "data/manual_teach_sphere_offsets.csv"
DEFAULT_CALIBRATION_OUTPUT = PROJECT_DIR / "data/manual_teach_sphere_offsets.json"


def parse_seed_fits(items):
    """Parse seed fits."""
    result = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError("--branch-seed-fit entries must be BRANCH=JSON")
        branch, path = item.split("=", 1)
        branch = branch.strip()
        path = path.strip()
        if not branch or not path:
            raise ValueError("--branch-seed-fit entries must be BRANCH=JSON")
        data = json.loads(Path(path).read_text())
        offset = data.get("local_ball_offset_mm")
        if not isinstance(offset, list) or len(offset) != 3:
            raise ValueError(f"{path}: missing 3-value local_ball_offset_mm")
        result[branch] = [float(value) for value in offset]
    return result


def seed_offset_for_branch(branch, reference_fit, geometry, seed_fits):
    """Seed offset for branch."""
    if branch in seed_fits:
        return list(seed_fits[branch]), "branch_seed_fit"
    if reference_fit.get("branch") == branch:
        return [float(value) for value in reference_fit["local_ball_offset_mm"]], "reference_fit"
    return branch_local_offset_mm(geometry, branch), "nominal_geometry"


def correction_from_pose(pose, branch, seed_offset, sphere_center, contact_distance, euler_sequence):
    """Correction from pose."""
    rotation = euler_to_matrix(euler_sequence, pose[3:6])
    seed_ball_center = add(pose[:3], mat_vec(rotation, seed_offset))
    sphere_to_seed = sub(seed_ball_center, sphere_center)
    seed_distance = norm(sphere_to_seed)
    if seed_distance <= 1e-9:
        raise ValueError("seed ball centre coincides with sphere centre; cannot infer contact normal")
    sphere_to_probe = normalize(sphere_to_seed, "sphere-to-probe")
    approach = scale(sphere_to_probe, -1.0)
    corrected_ball_center = add(sphere_center, scale(sphere_to_probe, contact_distance))
    corrected_offset = transpose_mat_vec(rotation, sub(corrected_ball_center, pose[:3]))
    offset_delta = sub(corrected_offset, seed_offset)
    return {
        "branch": branch,
        "seed_ball_center": seed_ball_center,
        "seed_center_distance_mm": seed_distance,
        "radial_error_mm": seed_distance - contact_distance,
        "approach": approach,
        "corrected_ball_center": corrected_ball_center,
        "corrected_offset": corrected_offset,
        "offset_delta": offset_delta,
        "offset_delta_norm_mm": norm(offset_delta),
    }


def stable_append_row(path, row):
    """Stable append row."""
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    exists = output.exists()
    if exists:
        with output.open(newline="") as f:
            header = next(csv.reader(f), None)
        if header != list(row.keys()):
            raise ValueError(f"CSV header mismatch for {output}; choose a new --output")
    with output.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def make_row(args, sample_index, branch, seed_source, seed_offset, snapshot, pose, result):
    """Make row."""
    row = {
        "timestamp": f"{time.time():.3f}",
        "sample_index": str(sample_index),
        "session_id": args.session_id,
        "workpiece_id": args.workpiece_id,
        "artifact_id": args.artifact_id,
        "artifact_type": args.artifact_type,
        "physical_ball_id": args.physical_ball_id,
        "branch": branch,
        "operator_note": args.operator_note,
        "reference_fit_json": str(args.reference_fit_json),
        "geometry": str(args.geometry),
        "euler_sequence": args.euler_sequence,
        "seed_source": seed_source,
        "sphere_center_x": f"{args._sphere_center[0]:.6f}",
        "sphere_center_y": f"{args._sphere_center[1]:.6f}",
        "sphere_center_z": f"{args._sphere_center[2]:.6f}",
        "contact_center_distance_mm": f"{args._contact_distance:.6f}",
        "trigger_feed_sequence": str(snapshot.get("sequence", "")),
        "trigger_feed_wall_time": "" if snapshot.get("wall_time") is None else f"{float(snapshot['wall_time']):.6f}",
        "trigger_digital_input_bits": (
            "" if snapshot.get("digital_input_bits") is None else str(snapshot["digital_input_bits"])
        ),
        "trigger_di1": "" if snapshot.get("di1") is None else str(int(bool(snapshot["di1"]))),
        "seed_offset_x": f"{seed_offset[0]:.6f}",
        "seed_offset_y": f"{seed_offset[1]:.6f}",
        "seed_offset_z": f"{seed_offset[2]:.6f}",
        "seed_ball_center_x": f"{result['seed_ball_center'][0]:.4f}",
        "seed_ball_center_y": f"{result['seed_ball_center'][1]:.4f}",
        "seed_ball_center_z": f"{result['seed_ball_center'][2]:.4f}",
        "seed_center_distance_mm": f"{result['seed_center_distance_mm']:.6f}",
        "radial_error_mm": f"{result['radial_error_mm']:.6f}",
        "approach_x": f"{result['approach'][0]:.6f}",
        "approach_y": f"{result['approach'][1]:.6f}",
        "approach_z": f"{result['approach'][2]:.6f}",
        "corrected_ball_center_x": f"{result['corrected_ball_center'][0]:.4f}",
        "corrected_ball_center_y": f"{result['corrected_ball_center'][1]:.4f}",
        "corrected_ball_center_z": f"{result['corrected_ball_center'][2]:.4f}",
        "corrected_offset_x": f"{result['corrected_offset'][0]:.6f}",
        "corrected_offset_y": f"{result['corrected_offset'][1]:.6f}",
        "corrected_offset_z": f"{result['corrected_offset'][2]:.6f}",
        "offset_delta_x": f"{result['offset_delta'][0]:.6f}",
        "offset_delta_y": f"{result['offset_delta'][1]:.6f}",
        "offset_delta_z": f"{result['offset_delta'][2]:.6f}",
        "offset_delta_norm_mm": f"{result['offset_delta_norm_mm']:.6f}",
    }
    row.update(pose_fields("trigger_flange", pose))
    row.update(vector_fields("trigger_joint", snapshot.get("joints"), 6))
    return row


def existing_max_sample_index(path):
    """Existing max sample index."""
    path = Path(path)
    if not path.exists():
        return 0
    result = 0
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            try:
                result = max(result, int(float(row.get("sample_index", "0"))))
            except ValueError:
                pass
    return result


def wait_for_manual_trigger(node, args, branch):
    """Wait for manual trigger."""
    node.wait_fresh_feed(timeout_sec=args.service_timeout_sec)
    if not args.capture_now:
        print(
            f"\nbranch {branch}: release DI1, then manually touch the standard sphere until DI1 triggers",
            flush=True,
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            snapshot = node.feed_snapshot()
            if not snapshot.get("di1"):
                break
        deadline = None if args.wait_timeout_sec <= 0 else time.monotonic() + args.wait_timeout_sec
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
            snapshot = node.feed_snapshot()
            if snapshot.get("di1"):
                trigger_snapshot = dict(snapshot)
                if args.stop_on_trigger:
                    node.stop_fast_then_confirm()
                return trigger_snapshot
            if deadline is not None and time.monotonic() > deadline:
                raise TimeoutError(f"timed out waiting for DI1 trigger for {branch}")
    snapshot = node.feed_snapshot()
    if not snapshot.get("di1") and not args.allow_not_triggered:
        raise RuntimeError("--capture-now requires DI1=1; pass --allow-not-triggered only for diagnostics")
    return snapshot


def read_corrected_rows(paths, branches, physical_ball_id=None):
    """Read corrected rows."""
    rows = {branch: [] for branch in branches}
    for path in paths:
        if not path or not Path(path).exists():
            continue
        with Path(path).open(newline="") as f:
            for row in csv.DictReader(f):
                branch = row.get("branch", "")
                if branch not in rows:
                    continue
                if physical_ball_id and row.get("physical_ball_id") not in ("", physical_ball_id):
                    continue
                try:
                    rows[branch].append(
                        {
                            "source_csv": str(path),
                            "source_row": row,
                            "offset": [
                                float(row["corrected_offset_x"]),
                                float(row["corrected_offset_y"]),
                                float(row["corrected_offset_z"]),
                            ],
                            "radial_error_mm": float(row.get("radial_error_mm", "nan")),
                            "offset_delta_norm_mm": float(row.get("offset_delta_norm_mm", "nan")),
                        }
                    )
                except (KeyError, TypeError, ValueError):
                    continue
    return rows


def fit_offsets(args, branches):
    """Fit offsets."""
    rows = read_corrected_rows([args.output] + list(args.fit_input or []), branches, args.physical_ball_id)
    results = []
    residual_rows = []
    for branch in branches:
        samples = rows.get(branch, [])
        if not samples:
            results.append({"branch": branch, "ok": False, "reason": "no manual teach samples"})
            continue
        arr = np.asarray([sample["offset"] for sample in samples], dtype=float)
        estimate = np.mean(arr, axis=0)
        spread = np.linalg.norm(arr - estimate, axis=1)
        for sample, distance in zip(samples, spread):
            row = sample["source_row"]
            residual_rows.append(
                {
                    "source_csv": sample["source_csv"],
                    "sample_index": row.get("sample_index", ""),
                    "branch": branch,
                    "physical_ball_id": row.get("physical_ball_id", ""),
                    "estimated_offset_x": f"{estimate[0]:.6f}",
                    "estimated_offset_y": f"{estimate[1]:.6f}",
                    "estimated_offset_z": f"{estimate[2]:.6f}",
                    "sample_spread_norm_mm": f"{float(distance):.6f}",
                    "radial_error_mm": row.get("radial_error_mm", ""),
                    "offset_delta_norm_mm": row.get("offset_delta_norm_mm", ""),
                }
            )
        results.append(
            {
                "branch": branch,
                "ok": True,
                "rows": len(samples),
                "estimated_offset_mm": [float(value) for value in estimate.tolist()],
                "rms_sample_spread_mm": float(math.sqrt(float(np.mean(spread * spread)))),
                "max_sample_spread_mm": float(np.max(spread)),
                "mean_abs_radial_error_mm": float(np.nanmean([abs(sample["radial_error_mm"]) for sample in samples])),
                "max_offset_delta_norm_mm": float(np.nanmax([sample["offset_delta_norm_mm"] for sample in samples])),
            }
        )
    return results, residual_rows


def write_csv(path, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not rows:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_calibration(args, branches):
    """Write calibration."""
    results, residual_rows = fit_offsets(args, branches)
    result = {
        "timestamp": time.time(),
        "method": "manual_sphere_touch_projected_offset_fit",
        "method_warning": (
            "Manual trigger poses are converted to offsets by projecting the seed ball centre "
            "onto the known standard-sphere contact shell. Use as a seed, then validate with guarded probes."
        ),
        "reference_fit_json": str(args.reference_fit_json),
        "geometry": str(args.geometry),
        "sphere_center_mm": args._sphere_center,
        "contact_center_distance_mm": args._contact_distance,
        "physical_ball_id": args.physical_ball_id,
        "branches": results,
    }
    output = Path(args.calibration_output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    if args.residual_output:
        write_csv(args.residual_output, residual_rows)
    if args.branch_output_dir:
        branch_dir = Path(args.branch_output_dir)
        branch_dir.mkdir(parents=True, exist_ok=True)
        for branch in results:
            if not branch.get("ok"):
                continue
            path = branch_dir / f"{args.physical_ball_id}_{branch['branch']}_manual_seed.json"
            path.write_text(
                json.dumps(
                    {
                        "timestamp": time.time(),
                        "method": "manual_sphere_touch_projected_seed",
                        "source_calibration_json": str(output),
                        "source_contact_csv": str(args.output),
                        "physical_ball_id": args.physical_ball_id,
                        "branch": branch["branch"],
                        "sphere_center_mm": args._sphere_center,
                        "contact_center_distance_mm": args._contact_distance,
                        "local_ball_offset_mm": branch["estimated_offset_mm"],
                        "sample_count": branch["rows"],
                        "rms_sample_spread_mm": branch["rms_sample_spread_mm"],
                        "max_sample_spread_mm": branch["max_sample_spread_mm"],
                    },
                    indent=2,
                )
                + "\n"
            )
    print(f"saved calibration JSON: {args.calibration_output}")
    for branch in results:
        if not branch.get("ok"):
            print(f"  {branch['branch']}: {branch['reason']}")
            continue
        offset = branch["estimated_offset_mm"]
        print(
            f"  {branch['branch']}: rows={branch['rows']} "
            f"offset=[{offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}] "
            f"rms_spread={branch['rms_sample_spread_mm']:.6f} "
            f"max_delta={branch['max_offset_delta_norm_mm']:.6f}"
        )


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-fit-json", default=str(DEFAULT_REFERENCE_FIT))
    parser.add_argument("--geometry", default=str(DEFAULT_GEOMETRY))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--calibration-output", default=str(DEFAULT_CALIBRATION_OUTPUT))
    parser.add_argument("--residual-output")
    parser.add_argument("--branch-output-dir")
    parser.add_argument("--fit-input", nargs="*", default=[])
    parser.add_argument("--branch-seed-fit", nargs="*", default=[], metavar="BRANCH=JSON")
    parser.add_argument("--branch", help="single branch to capture")
    parser.add_argument("--branches", nargs="+", help="branches to capture in sequence")
    parser.add_argument("--samples-per-branch", type=int, default=1)
    parser.add_argument("--physical-ball-id", default="manual_probe")
    parser.add_argument("--session-id", default="session_manual_teach_sphere_offsets")
    parser.add_argument("--workpiece-id", default="calibration_sphere_20mm")
    parser.add_argument("--artifact-id", default="standard_sphere_20mm")
    parser.add_argument("--artifact-type", default="sphere")
    parser.add_argument("--operator-note", default="manual teach sphere offset")
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float, default=1.0)
    parser.add_argument("--capture-now", action="store_true")
    parser.add_argument("--allow-not-triggered", action="store_true")
    parser.add_argument("--stop-on-trigger", action="store_true")
    parser.add_argument("--wait-timeout-sec", type=float, default=0.0, help="0 means wait forever")
    parser.add_argument("--service-timeout-sec", type=float, default=10.0)
    parser.add_argument(
        "--max-radial-error-mm",
        type=float,
        default=10.0,
        help="reject manual samples whose seed ball centre is this far from the sphere contact shell",
    )
    parser.add_argument("--allow-large-radial-error", action="store_true")
    parser.add_argument("--fit-only", action="store_true")
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    node = None
    try:
        if args.samples_per_branch <= 0:
            raise ValueError("--samples-per-branch must be positive")
        if args.sphere_radius_mm <= 0 or args.probe_radius_mm <= 0:
            raise ValueError("sphere/probe radii must be positive")
        reference_fit = load_reference_fit(args.reference_fit_json)
        geometry = load_geometry(args.geometry)
        seed_fits = parse_seed_fits(args.branch_seed_fit)
        args._sphere_center = [float(value) for value in reference_fit["sphere_center_mm"]]
        args._contact_distance = float(
            reference_fit.get("contact_center_distance_mm", args.sphere_radius_mm + args.probe_radius_mm)
        )
        branches = args.branches or ([args.branch] if args.branch else None)
        if not branches:
            raise ValueError("provide --branch or --branches")
        if args.fit_only:
            write_calibration(args, branches)
            return
        rclpy.init()
        node = ProbeTouch()
        node.wait_services(args.service_timeout_sec)
        sample_index = existing_max_sample_index(args.output)
        for branch in branches:
            branch_direction(geometry, branch)
            seed_offset, seed_source = seed_offset_for_branch(branch, reference_fit, geometry, seed_fits)
            for _ in range(args.samples_per_branch):
                snapshot = wait_for_manual_trigger(node, args, branch)
                pose = snapshot.get("pose")
                if pose is None:
                    pose = current_pose(node, max_age_sec=0.5)
                result = correction_from_pose(
                    pose,
                    branch,
                    seed_offset,
                    args._sphere_center,
                    args._contact_distance,
                    args.euler_sequence,
                )
                if (
                    not args.allow_large_radial_error
                    and abs(result["radial_error_mm"]) > args.max_radial_error_mm
                ):
                    raise RuntimeError(
                        f"rejected sample for {branch}: radial_error={result['radial_error_mm']:.4f}mm "
                        f"exceeds --max-radial-error-mm={args.max_radial_error_mm:.4f}. "
                        "This usually means wrong branch/seed, false trigger, or the probe was not touching "
                        "the standard sphere."
                    )
                sample_index += 1
                row = make_row(args, sample_index, branch, seed_source, seed_offset, snapshot, pose, result)
                stable_append_row(args.output, row)
                print(
                    f"recorded sample {sample_index} branch={branch} "
                    f"radial_error={result['radial_error_mm']:.4f}mm "
                    f"offset_delta={result['offset_delta_norm_mm']:.4f}mm",
                    flush=True,
                )
                write_calibration(args, branches)
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
