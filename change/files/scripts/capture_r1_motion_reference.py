#!/usr/bin/env python3
"""Capture R1 swept reference geometry from CoppeliaSim for RViz.

The output is a robot-relative JSON file consumed by
``publish_r1_motion_reference.py``. It samples the validated R1 joint replay
plan, queries CoppeliaSim for selected R1 collision/tool shape bounding boxes,
and transforms those boxes into the selected planning robot's local frame
(R2 by default).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Iterable

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.r1_motion import PLAN_PATH, load_r1_plan
from sim_bridge.coppelia_client import SimBridge


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "data"
    / "manual_waypoints"
    / "r1_motion_reference"
    / "r1_box_transfer_reference.json"
)
DEFAULT_SEGMENTS = (
    "initial_to_box_pick_app",
    "box_descend",
    "box_lift_and_transfer",
    "box_place_descend",
    "box_retreat_and_terminal_approach",
)
DEFAULT_SHAPE_ALIASES = (
    "Link2_respondable",
    "Link3_respondable",
    "Link4_respondable",
    "Link5_respondable",
    "Link6_respondable",
    "R1T_main_body",
    "R1T_top_rail",
    "R1T_bottom_rail",
    "R1T_wide_guide_top",
    "R1T_wide_guide_bottom",
    "R1T_wide_back_bridge",
    "R1T_left_front_vertical_jaw",
    "R1T_right_front_vertical_jaw",
)


def _quat_multiply(
    first: tuple[float, float, float, float],
    second: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    rotated = _quat_multiply(
        _quat_multiply((x, y, z, w), (*vector, 0.0)),
        (-x, -y, -z, w),
    )
    return rotated[:3]


def _pose_multiply(first: list[float], second: list[float]) -> list[float]:
    translated = _rotate_vector(tuple(first[3:]), tuple(second[:3]))
    return [
        first[index] + translated[index] for index in range(3)
    ] + list(_quat_multiply(tuple(first[3:]), tuple(second[3:])))


def _pose_inverse(pose: list[float]) -> list[float]:
    q_inv = [-pose[3], -pose[4], -pose[5], pose[6]]
    rotated = _rotate_vector(tuple(q_inv), tuple(-value for value in pose[:3]))
    return list(rotated) + q_inv


def _find_alias(sim, root: int, alias: str) -> int:
    matches = [
        handle
        for handle in sim.getObjectsInTree(root, sim.handle_all, 0)
        if sim.getObjectAlias(handle) == alias
    ]
    if len(matches) != 1:
        root_path = sim.getObjectAlias(root, 1)
        raise RuntimeError(f"expected one {alias} below {root_path}, found {len(matches)}")
    return int(matches[0])


def _parse_csv(values: str) -> tuple[str, ...]:
    return tuple(value.strip() for value in values.split(",") if value.strip())


def _sample_plan(
    plan: dict,
    segments: Iterable[str],
    stride: int,
) -> list[tuple[str, int, list[float]]]:
    result: list[tuple[str, int, list[float]]] = []
    for segment in segments:
        configs = plan["paths"][segment]
        for index, config in enumerate(configs):
            if index % stride != 0 and index != len(configs) - 1:
                continue
            if result and max(abs(a - b) for a, b in zip(result[-1][2], config)) < 1e-9:
                continue
            result.append((segment, index, [float(value) for value in config]))
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23001)
    parser.add_argument("--plan", type=Path, default=PLAN_PATH)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--reference-robot", default="R2")
    parser.add_argument("--frame-id", default="dummy_link")
    parser.add_argument("--sample-stride", type=int, default=1)
    parser.add_argument(
        "--segments",
        type=_parse_csv,
        default=DEFAULT_SEGMENTS,
        help="Comma-separated R1 plan segment names to sample.",
    )
    parser.add_argument(
        "--shape-aliases",
        type=_parse_csv,
        default=DEFAULT_SHAPE_ALIASES,
        help="Comma-separated R1 shape aliases to capture.",
    )
    parser.add_argument(
        "--stop-simulation",
        action="store_true",
        help="Stop CoppeliaSim first so setJointPosition is deterministic.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.sample_stride < 1:
        raise ValueError("--sample-stride must be >= 1")

    plan = load_r1_plan(args.plan)
    samples = _sample_plan(plan, args.segments, args.sample_stride)
    bridge = SimBridge(args.host, args.port)
    if not bridge.connect(bridge.host, bridge.port):
        raise RuntimeError(bridge.last_error or "cannot connect to CoppeliaSim")
    sim = bridge.sim
    if args.stop_simulation:
        bridge.stop_simulation()
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError(
            "capture requires a stopped simulation; rerun with --stop-simulation"
        )

    r1 = bridge.get_object_handle("R1")
    reference = bridge.get_object_handle(args.reference_robot.upper())
    joints = bridge.get_robot_joint_handles("R1")
    original_joints = [float(sim.getJointPosition(joint)) for joint in joints]
    reference_pose = [float(value) for value in sim.getObjectPose(reference, -1)]
    world_to_reference = _pose_inverse(reference_pose)
    shape_handles = {
        alias: _find_alias(sim, r1, alias) for alias in args.shape_aliases
    }
    shape_sizes = {}
    shape_bb_poses = {}
    for alias, handle in shape_handles.items():
        size, bb_pose = sim.getShapeBB(handle)
        shape_sizes[alias] = [float(value) for value in size]
        shape_bb_poses[alias] = [float(value) for value in bb_pose]

    output_samples = []
    try:
        for sample_index, (segment, path_index, config) in enumerate(samples):
            for joint, value in zip(joints, config):
                sim.setJointPosition(joint, float(value))
            sample_shapes = {}
            for alias, handle in shape_handles.items():
                object_pose = [
                    float(value) for value in sim.getObjectPose(handle, -1)
                ]
                world_bb_pose = _pose_multiply(object_pose, shape_bb_poses[alias])
                reference_bb_pose = _pose_multiply(world_to_reference, world_bb_pose)
                sample_shapes[alias] = {
                    "pose_xyzw": [round(value, 9) for value in reference_bb_pose],
                    "scale_xyz": [round(value, 9) for value in shape_sizes[alias]],
                }
            output_samples.append(
                {
                    "sample_index": sample_index,
                    "segment": segment,
                    "path_index": path_index,
                    "joint_positions_rad": [round(value, 12) for value in config],
                    "joint_positions_deg": [
                        round(math.degrees(value), 6) for value in config
                    ],
                    "shapes": sample_shapes,
                }
            )
    finally:
        for joint, value in zip(joints, original_joints):
            sim.setJointPosition(joint, value)
        bridge.disconnect()

    payload = {
        "captured_at": datetime.now().isoformat(timespec="seconds"),
        "source_plan": str(args.plan.resolve()),
        "reference_robot": args.reference_robot.upper(),
        "frame_id": args.frame_id,
        "reference_pose_world_xyzw": [round(value, 9) for value in reference_pose],
        "segments": list(args.segments),
        "shape_aliases": list(args.shape_aliases),
        "sample_stride": args.sample_stride,
        "samples": output_samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "output": str(args.output),
                "samples": len(output_samples),
                "shapes": len(args.shape_aliases),
                "frame_id": args.frame_id,
                "reference_robot": args.reference_robot.upper(),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
