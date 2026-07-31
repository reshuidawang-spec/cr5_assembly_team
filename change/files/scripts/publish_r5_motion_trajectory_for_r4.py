#!/usr/bin/env python3
"""Publish the taught R5 sorting motion as visible-only markers in R4 RViz.

The generated data is intentionally marker-only.  It does not publish
CollisionObject or PlanningScene messages, so R5 remains visible but ignored by
MoveIt collision checking while teaching R4's ready posture.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sim_bridge.coppelia_client import SimBridge


DEFAULT_SCENE = REPO_ROOT / "scenes" / "compact_cell1ttt.ttt"
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "data"
    / "rviz_obstacles"
    / "r5_sort_motion_trajectory_for_r4.json"
)
DEFAULT_CAPTURES = (
    REPO_ROOT
    / "data"
    / "manual_waypoints"
    / "rviz_plan_captures"
    / "r5_sort_new_layout"
    / "rviz_plan_capture_20260728_162347_01.json",
    REPO_ROOT
    / "data"
    / "manual_waypoints"
    / "rviz_plan_captures"
    / "r5_sort_new_layout"
    / "rviz_plan_capture_20260728_162812_03.json",
)

FRAME_ID = "dummy_link"
DEFAULT_FRAME_OBJECT = "/R4"
DEFAULT_ROBOT_OBJECT = "/R5"
R5_TIP_CANDIDATES = (
    "/R5/R5T/R5_gripper_tip",
    "/R5/R5_gripper_tip",
)
ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def _quaternion_multiply(
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
    rotated = _quaternion_multiply(
        _quaternion_multiply((x, y, z, w), (*vector, 0.0)),
        (-x, -y, -z, w),
    )
    return rotated[:3]


def _compose_poses(first: list[float], second: list[float]) -> list[float]:
    translated = _rotate_vector(tuple(first[3:]), tuple(second[:3]))
    return [
        first[index] + translated[index] for index in range(3)
    ] + list(_quaternion_multiply(tuple(first[3:]), tuple(second[3:])))


def _inverse_pose(pose: list[float]) -> list[float]:
    qx, qy, qz, qw = [float(value) for value in pose[3:]]
    inverse_q = (-qx, -qy, -qz, qw)
    inverse_t = _rotate_vector(
        inverse_q,
        tuple(-float(value) for value in pose[:3]),
    )
    return list(inverse_t) + list(inverse_q)


def _transform_point(
    pose: list[float],
    point: Iterable[float],
) -> tuple[float, float, float]:
    rotated = _rotate_vector(tuple(pose[3:]), tuple(float(value) for value in point))
    return tuple(float(pose[index]) + rotated[index] for index in range(3))


def _world_to_frame(
    frame_pose: list[float],
    point: Iterable[float],
) -> tuple[float, float, float]:
    return _transform_point(_inverse_pose(frame_pose), point)


def _sanitize_id(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", text).strip("_").lower()


def _load_capture_samples(
    capture_paths: tuple[Path, ...],
    sample_stride: int,
) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    stride = max(1, int(sample_stride))
    previous: list[float] | None = None

    for capture_path in capture_paths:
        data = json.loads(capture_path.read_text(encoding="utf-8"))
        trajectories = data["display_trajectory"]["trajectories"]
        for trajectory in trajectories:
            joint_names = list(trajectory["joint_names"])
            missing = [name for name in ARM_JOINT_NAMES if name not in joint_names]
            if missing:
                raise ValueError(f"{capture_path} is missing joints: {missing}")
            joint_indices = [joint_names.index(name) for name in ARM_JOINT_NAMES]
            points = list(trajectory["points"])
            for point_index, point in enumerate(points):
                is_endpoint = point_index == len(points) - 1
                if point_index % stride != 0 and not is_endpoint:
                    continue
                positions = [float(point["positions_rad"][index]) for index in joint_indices]
                if previous is not None and all(
                    abs(a - b) < 1e-9 for a, b in zip(previous, positions)
                ):
                    continue
                previous = positions
                samples.append(
                    {
                        "capture": str(capture_path),
                        "capture_name": capture_path.stem,
                        "trajectory_index": int(trajectory.get("index", 0)),
                        "point_index": point_index,
                        "time_from_start_s": float(point.get("time_from_start_s", 0.0)),
                        "positions_rad": positions,
                        "positions_deg": [math.degrees(value) for value in positions],
                    }
                )

    if not samples:
        raise ValueError("no R5 motion samples selected")
    return samples


def _shape_box_in_frame(
    sim: Any,
    shape: int,
    frame_pose: list[float],
    inflate: float,
    prefix: str,
) -> dict[str, Any]:
    size, bb_pose = sim.getShapeBB(shape)
    world_bb_pose = _compose_poses(sim.getObjectPose(shape, -1), bb_pose)
    lower = [math.inf, math.inf, math.inf]
    upper = [-math.inf, -math.inf, -math.inf]
    for signs in itertools.product((-0.5, 0.5), repeat=3):
        local = tuple(float(size[index]) * signs[index] for index in range(3))
        rotated = _rotate_vector(tuple(world_bb_pose[3:]), local)
        world_point = [
            world_bb_pose[index] + rotated[index] for index in range(3)
        ]
        frame_point = _world_to_frame(frame_pose, world_point)
        lower = [min(a, b) for a, b in zip(lower, frame_point)]
        upper = [max(a, b) for a, b in zip(upper, frame_point)]
    inflated_lower = [value - inflate for value in lower]
    inflated_upper = [value + inflate for value in upper]
    center = [
        (inflated_lower[index] + inflated_upper[index]) / 2.0
        for index in range(3)
    ]
    dimensions = [
        inflated_upper[index] - inflated_lower[index]
        for index in range(3)
    ]
    alias = sim.getObjectAlias(shape, 1)
    return {
        "id": f"{prefix}_{_sanitize_id(alias)}",
        "alias": alias,
        "center": center,
        "size": dimensions,
        "lower": inflated_lower,
        "upper": inflated_upper,
    }


def _merge_boxes(boxes: list[dict[str, Any]], prefix: str) -> list[dict[str, Any]]:
    by_alias: dict[str, dict[str, Any]] = {}
    for box in boxes:
        key = box["alias"]
        current = by_alias.get(key)
        if current is None:
            by_alias[key] = {
                "id": f"{prefix}_{_sanitize_id(key)}",
                "alias": key,
                "lower": list(box["lower"]),
                "upper": list(box["upper"]),
            }
            continue
        current["lower"] = [
            min(a, b) for a, b in zip(current["lower"], box["lower"])
        ]
        current["upper"] = [
            max(a, b) for a, b in zip(current["upper"], box["upper"])
        ]

    merged = []
    for box in by_alias.values():
        lower = box["lower"]
        upper = box["upper"]
        center = [(lower[index] + upper[index]) / 2.0 for index in range(3)]
        size = [upper[index] - lower[index] for index in range(3)]
        merged.append(
            {
                "id": box["id"],
                "alias": box["alias"],
                "center": center,
                "size": size,
            }
        )
    return sorted(merged, key=lambda item: item["id"])


def _set_joints(sim: Any, joints: list[int], config: list[float]) -> None:
    for joint, value in zip(joints, config):
        sim.setJointPosition(joint, float(value))
        sim.setJointTargetPosition(joint, float(value))


def _find_tip(sim: Any) -> int | None:
    for path in R5_TIP_CANDIDATES:
        try:
            return int(sim.getObject(path))
        except Exception:
            continue
    return None


def _tip_position(sim: Any, tip: int | None, frame_pose: list[float]) -> list[float] | None:
    if tip is None:
        return None
    return list(_world_to_frame(frame_pose, sim.getObjectPosition(tip, -1)))


def generate_trajectory(
    scene_path: Path,
    capture_paths: tuple[Path, ...],
    output_path: Path,
    host: str,
    port: int,
    inflate: float,
    sample_stride: int,
    frame_object: str,
    robot_object: str,
) -> dict[str, Any]:
    bridge = SimBridge(host=host, port=port)
    if not bridge.connect(host, port):
        raise RuntimeError(bridge.last_error or "cannot connect to CoppeliaSim")
    sim = bridge.sim
    try:
        if sim.getSimulationState() != sim.simulation_stopped:
            if not bridge.stop_simulation():
                raise RuntimeError(bridge.last_error or "cannot stop CoppeliaSim")
        sim.loadScene(str(scene_path.resolve()))

        samples = _load_capture_samples(capture_paths, sample_stride)
        frame = sim.getObject(frame_object)
        robot = sim.getObject(robot_object)
        frame_pose = sim.getObjectPose(frame, -1)
        joints = bridge.get_robot_joint_handles("R5")
        shapes = list(sim.getObjectsInTree(robot, sim.object_shape_type, 0))
        tip = _find_tip(sim)

        if not bridge.start_simulation():
            raise RuntimeError(bridge.last_error or "cannot start CoppeliaSim")
        for _ in range(3):
            if not bridge.step():
                raise RuntimeError(bridge.last_error or "cannot step CoppeliaSim")

        sampled_boxes: list[dict[str, Any]] = []
        sample_boxes: list[dict[str, Any]] = []
        tip_trace: list[dict[str, Any]] = []
        for sample_index, sample in enumerate(samples):
            _set_joints(sim, joints, sample["positions_rad"])
            if not bridge.step():
                raise RuntimeError(bridge.last_error or "cannot step CoppeliaSim")
            sample_prefix = f"r5_motion_sample_{sample_index:03d}"
            current_boxes = [
                _shape_box_in_frame(sim, shape, frame_pose, inflate, sample_prefix)
                for shape in shapes
            ]
            sampled_boxes.extend(current_boxes)
            sample_boxes.extend(
                {
                    **box,
                    "sample_index": sample_index,
                    "capture_name": sample["capture_name"],
                    "source_point_index": sample["point_index"],
                }
                for box in current_boxes
            )
            tip_position = _tip_position(sim, tip, frame_pose)
            if tip_position is not None:
                tip_trace.append(
                    {
                        "sample_index": sample_index,
                        "capture_name": sample["capture_name"],
                        "source_point_index": sample["point_index"],
                        "position": tip_position,
                    }
                )

        final_config = samples[-1]["positions_rad"]
        _set_joints(sim, joints, final_config)
        if not bridge.step():
            raise RuntimeError(bridge.last_error or "cannot step CoppeliaSim")
        final_boxes = [
            _shape_box_in_frame(sim, shape, frame_pose, inflate, "r5_motion_final")
            for shape in shapes
        ]
        first_capture_name = capture_paths[0].stem
        mid_config = next(
            (
                sample["positions_rad"]
                for sample in reversed(samples)
                if sample["capture_name"] == first_capture_name
            ),
            samples[0]["positions_rad"],
        )
        payload = {
            "frame_id": FRAME_ID,
            "frame_object": frame_object,
            "robot_object": robot_object,
            "frame_pose_world": [float(value) for value in frame_pose],
            "source_scene": str(scene_path.resolve()),
            "source_captures": [str(path.resolve()) for path in capture_paths],
            "sample_stride": max(1, int(sample_stride)),
            "sample_count": len(samples),
            "shape_count": len(shapes),
            "inflate_m": float(inflate),
            "start_r5_joints_rad": samples[0]["positions_rad"],
            "mid_r5_joints_rad": mid_config,
            "final_r5_joints_rad": final_config,
            "samples": samples,
            "sweep_boxes": _merge_boxes(sampled_boxes, "r5_motion_sweep"),
            "sample_boxes": sample_boxes,
            "final_boxes": _merge_boxes(final_boxes, "r5_motion_final"),
            "tip_trace": tip_trace,
            "collision_objects": 0,
        }
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return payload
    finally:
        try:
            if sim.getSimulationState() != sim.simulation_stopped:
                bridge.stop_simulation()
        except Exception:
            pass
        bridge.disconnect()


def _load_trajectory_file(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if "sweep_boxes" not in data or "sample_boxes" not in data:
        raise ValueError(f"{path} has no R5 motion marker boxes")
    if "tip_trace" not in data:
        data["tip_trace"] = []
    return data


def _publish_trajectory(
    path: Path,
    rate_hz: float,
    clear: bool,
    show_samples: bool,
    show_final: bool,
) -> None:
    import rclpy
    from geometry_msgs.msg import Point, Pose
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    data = _load_trajectory_file(path)
    frame_id = data.get("frame_id", FRAME_ID)
    marker_boxes = data["sweep_boxes"]
    sample_marker_boxes = data["sample_boxes"] if show_samples else []
    final_marker_boxes = data["final_boxes"] if show_final else []

    def pose(center: list[float]) -> Pose:
        result = Pose()
        result.position.x = float(center[0])
        result.position.y = float(center[1])
        result.position.z = float(center[2])
        result.orientation.w = 1.0
        return result

    def point(values: list[float]) -> Point:
        return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))

    def box_marker(
        box: dict[str, Any],
        index: int,
        namespace: str,
        color: ColorRGBA,
    ) -> Marker:
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.ns = namespace
        msg.id = index
        msg.type = Marker.CUBE
        msg.action = Marker.DELETE if clear else Marker.ADD
        msg.pose = pose(box["center"])
        msg.scale.x = float(box["size"][0])
        msg.scale.y = float(box["size"][1])
        msg.scale.z = float(box["size"][2])
        msg.color = color
        msg.lifetime.sec = 0
        return msg

    def tip_trace_marker(marker_id: int) -> Marker:
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.ns = "r5_sort_tip_trace_for_r4"
        msg.id = marker_id
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.DELETE if clear else Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.010
        msg.color = ColorRGBA(r=0.10, g=0.95, b=0.80, a=0.95)
        msg.points = [point(item["position"]) for item in data["tip_trace"]]
        msg.lifetime.sec = 0
        return msg

    class R5MotionTrajectoryPublisher(Node):
        def __init__(self) -> None:
            super().__init__("r5_motion_trajectory_for_r4_publisher")
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.marker_pub = self.create_publisher(
                MarkerArray,
                "/r5_motion_trajectory_for_r4_markers",
                qos,
            )
            self.visual_marker_pub = self.create_publisher(
                MarkerArray,
                "/visualization_marker_array",
                qos,
            )
            self.timer = self.create_timer(max(0.1, 1.0 / rate_hz), self.publish_all)
            self.publish_all()

        def publish_all(self) -> None:
            markers = [
                box_marker(
                    box,
                    index,
                    "r5_sort_motion_sweep_for_r4",
                    ColorRGBA(r=0.12, g=0.70, b=1.0, a=0.18),
                )
                for index, box in enumerate(marker_boxes)
            ]
            offset = len(markers)
            markers.extend(
                box_marker(
                    box,
                    offset + index,
                    "r5_sort_motion_samples_for_r4",
                    ColorRGBA(r=0.45, g=0.78, b=1.0, a=0.075),
                )
                for index, box in enumerate(sample_marker_boxes)
            )
            offset = len(markers)
            markers.extend(
                box_marker(
                    box,
                    offset + index,
                    "r5_sort_motion_final_for_r4",
                    ColorRGBA(r=0.15, g=1.0, b=0.50, a=0.22),
                )
                for index, box in enumerate(final_marker_boxes)
            )
            if data["tip_trace"]:
                markers.append(tip_trace_marker(len(markers)))
            marker_array = MarkerArray(markers=markers)
            self.marker_pub.publish(marker_array)
            self.visual_marker_pub.publish(marker_array)
            self.get_logger().info(
                f"{'removed' if clear else 'published'} R5 motion trajectory for R4: "
                f"collision_objects=0, sweep_markers={len(marker_boxes)}, "
                f"sample_markers={len(sample_marker_boxes)}, "
                f"final_markers={len(final_marker_boxes)}, "
                f"trace_points={len(data['tip_trace'])}"
            )

    rclpy.init()
    node = R5MotionTrajectoryPublisher()
    try:
        if clear:
            rclpy.spin_once(node, timeout_sec=1.0)
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _parse_capture_list(text: str) -> tuple[Path, ...]:
    paths = tuple(Path(part.strip()) for part in text.split(",") if part.strip())
    if not paths:
        raise argparse.ArgumentTypeError("expected at least one capture path")
    return paths


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--captures", type=_parse_capture_list, default=DEFAULT_CAPTURES)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--inflate", type=float, default=0.0)
    parser.add_argument("--sample-stride", type=int, default=3)
    parser.add_argument("--frame-object", default=DEFAULT_FRAME_OBJECT)
    parser.add_argument("--robot-object", default=DEFAULT_ROBOT_OBJECT)
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--hide-samples", action="store_true")
    parser.add_argument("--show-final", action="store_true")
    args = parser.parse_args()

    capture_paths = tuple(path.resolve() for path in args.captures)
    if args.generate_only:
        data = generate_trajectory(
            args.scene,
            capture_paths,
            args.output,
            args.host,
            args.port,
            args.inflate,
            args.sample_stride,
            args.frame_object,
            args.robot_object,
        )
        print(
            f"wrote R5 motion trajectory to {args.output}: "
            f"samples={data['sample_count']}, shapes={data['shape_count']}, "
            f"sweep_boxes={len(data['sweep_boxes'])}, "
            f"sample_boxes={len(data['sample_boxes'])}, "
            f"final_boxes={len(data['final_boxes'])}, "
            f"collision_objects=0"
        )
        return

    if not args.output.exists():
        raise SystemExit(
            f"{args.output} does not exist. Run with --generate-only first."
        )
    _publish_trajectory(
        args.output,
        args.rate,
        args.clear,
        show_samples=not args.hide_samples,
        show_final=args.show_final,
    )


if __name__ == "__main__":
    main()
