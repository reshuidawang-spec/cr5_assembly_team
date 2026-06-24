#!/usr/bin/env python3
"""Record manually jogged probe contacts from DI1 rising edges.

This script does not command robot motion. Keep jogging from the teach pendant
or another operator-controlled interface; each DI1 OFF->ON transition records
the current FeedInfo pose and joints to CSV.
"""
import argparse
import csv
import time
from pathlib import Path

import rclpy

from probe_touch import ProbeTouch


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_DIR / "data/manual_probe_contacts.csv"


def pose_fields(prefix, pose):
    """Pose fields."""
    names = ("x", "y", "z", "rx", "ry", "rz")
    return {
        f"{prefix}_{name}": "" if pose is None else f"{float(pose[index]):.4f}"
        for index, name in enumerate(names)
    }


def vector_fields(prefix, values, count, digits=6):
    """Vector fields."""
    return {
        f"{prefix}_{index + 1}": "" if values is None or index >= len(values) else f"{float(values[index]):.{digits}f}"
        for index in range(count)
    }


def write_row(path, row):
    """Write row."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def make_row(args, sample_index, snapshot):
    """Make row."""
    pose = snapshot.get("pose")
    row = {
        "timestamp": f"{time.time():.3f}",
        "sample_index": str(sample_index),
        "session_id": args.session_id,
        "workpiece_id": args.workpiece_id,
        "artifact_id": args.artifact_id,
        "artifact_type": args.artifact_type,
        "physical_ball_id": args.physical_ball_id,
        "branch": args.branch,
        "operator_direction_note": args.operator_direction_note,
        "operator_pose_note": args.operator_pose_note,
        "feed_sequence": str(snapshot.get("sequence", "")),
        "feed_wall_time": "" if snapshot.get("wall_time") is None else f"{float(snapshot['wall_time']):.6f}",
        "digital_input_bits": "" if snapshot.get("digital_input_bits") is None else str(snapshot["digital_input_bits"]),
        "di1": "" if snapshot.get("di1") is None else str(int(bool(snapshot["di1"]))),
    }
    row.update(pose_fields("flange", pose))
    row.update(vector_fields("joint", snapshot.get("joints"), 6))
    return row


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--session-id", default="")
    parser.add_argument("--workpiece-id", default="calibration_sphere_20mm")
    parser.add_argument("--artifact-id", default="calibration_sphere")
    parser.add_argument("--artifact-type", default="sphere")
    parser.add_argument("--physical-ball-id", default="1")
    parser.add_argument("--branch", default="y_neg")
    parser.add_argument("--operator-direction-note", default="")
    parser.add_argument("--operator-pose-note", default="")
    parser.add_argument("--max-samples", type=int, default=0, help="0 means keep recording until Ctrl-C")
    parser.add_argument("--min-release-sec", type=float, default=0.2, help="require DI1 to be OFF this long before next sample")
    parser.add_argument("--services-timeout-sec", type=float, default=10.0)
    args = parser.parse_args()

    if args.max_samples < 0:
        raise SystemExit("--max-samples cannot be negative")
    if args.min_release_sec < 0:
        raise SystemExit("--min-release-sec cannot be negative")

    rclpy.init()
    node = ProbeTouch()
    sample_index = 0
    last_di = None
    last_off_time = None
    try:
        node.wait_services(args.services_timeout_sec)
        node.wait_fresh_feed()
        initial_di = bool(node.read_di1() or node.di1)
        last_di = initial_di
        if not initial_di:
            last_off_time = time.monotonic()
        print(f"manual contact recorder ready; output={args.output}", flush=True)
        print("jog the robot manually; each DI1 rising edge records one row", flush=True)
        print(f"initial DI1: {int(bool(initial_di))}", flush=True)
        if initial_di:
            print("DI1 is already ON; release the probe before the first sample", flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            snapshot = node.feed_snapshot()
            current_di = bool(snapshot.get("di1"))
            now = time.monotonic()
            if not current_di:
                last_off_time = now
            rising = current_di and last_di is False
            released_long_enough = last_off_time is not None and now - last_off_time >= args.min_release_sec
            if rising and released_long_enough:
                sample_index += 1
                row = make_row(args, sample_index, snapshot)
                write_row(args.output, row)
                pose = snapshot.get("pose")
                pose_text = "unknown" if pose is None else "[" + ", ".join(f"{float(v):.4f}" for v in pose[:6]) + "]"
                print(f"sample {sample_index}: recorded DI1 rising edge at pose {pose_text}", flush=True)
                if args.max_samples and sample_index >= args.max_samples:
                    break
            last_di = current_di
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"recorded samples: {sample_index}", flush=True)


if __name__ == "__main__":
    main()
