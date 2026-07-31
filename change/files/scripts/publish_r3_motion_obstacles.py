#!/usr/bin/env python3
"""Publish R3 module-placement motion as R2 MoveIt/RViz obstacles."""

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
DEFAULT_R3_PLAN = REPO_ROOT / "robot_control" / "plans" / "r3_gripper_cycle_plan.json"
DEFAULT_OUTPUT = REPO_ROOT / "data" / "rviz_obstacles" / "r3_module_motion_obstacles_for_r2.json"

FRAME_ID = "dummy_link"
DEFAULT_ACTION = "R3_MODULE_PLACED"
DEFAULT_SEGMENTS = ("lift_and_transfer", "place_descend")
DEFAULT_FRAME_OBJECT = "/R2"
R3_TIP_PATH = "/R3/R3T/R3_gripper_tip"


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


def _transform_point(pose: list[float], point: Iterable[float]) -> tuple[float, float, float]:
    rotated = _rotate_vector(tuple(pose[3:]), tuple(float(value) for value in point))
    return tuple(float(pose[index]) + rotated[index] for index in range(3))


def _world_to_frame(frame_pose: list[float], point: Iterable[float]) -> tuple[float, float, float]:
    return _transform_point(_inverse_pose(frame_pose), point)


def _sanitize_id(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", text).strip("_").lower()


def _load_action_paths(plan_path: Path, action: str) -> dict[str, list[list[float]]]:
    data = json.loads(plan_path.read_text(encoding="utf-8"))
    try:
        paths = data["paths"][action]
    except KeyError as exc:
        raise ValueError(f"{plan_path} has no action {action}") from exc
    return {
        name: [[float(value) for value in config] for config in path]
        for name, path in paths.items()
    }


def _load_plan_samples(
    paths: dict[str, list[list[float]]],
    segments: tuple[str, ...],
    sample_stride: int,
    trim_pick_lift: bool,
) -> list[tuple[str, list[float]]]:
    samples: list[tuple[str, list[float]]] = []
    stride = max(1, int(sample_stride))
    for segment in segments:
        if segment not in paths:
            raise ValueError(f"R3 plan has no path segment {segment}")
        segment_path = paths[segment]
        if trim_pick_lift and segment == "lift_and_transfer":
            lift_count = len(paths["pick_descend"])
            segment_path = segment_path[lift_count - 1 :]
        for index, config in enumerate(segment_path):
            is_endpoint = index == len(segment_path) - 1
            if index % stride == 0 or is_endpoint:
                samples.append((segment, list(config)))
    if not samples:
        raise ValueError("no R3 motion samples selected")
    return samples


def _shape_box_in_frame(
    sim: Any,
    shape: int,
    frame_pose: list[float],
    inflate: float,
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
        inflated_upper[index] - inflated_lower[index] for index in range(3)
    ]
    alias = sim.getObjectAlias(shape, 1)
    return {
        "id": f"r3_motion_{_sanitize_id(alias)}",
        "alias": alias,
        "lower": inflated_lower,
        "upper": inflated_upper,
        "center": center,
        "size": dimensions,
    }


def _merge_boxes(boxes: list[dict[str, Any]]) -> list[dict[str, Any]]:
    by_id: dict[str, dict[str, Any]] = {}
    for box in boxes:
        current = by_id.get(box["id"])
        if current is None:
            by_id[box["id"]] = {
                "id": box["id"],
                "alias": box["alias"],
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
    for box in by_id.values():
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
        sim.setJointPosition(joint, value)
        sim.setJointTargetPosition(joint, value)


def _tip_position(sim: Any, tip: int, frame_pose: list[float]) -> list[float]:
    return list(_world_to_frame(frame_pose, sim.getObjectPosition(tip, -1)))


def generate_obstacles(
    scene_path: Path,
    r3_plan_path: Path,
    output_path: Path,
    host: str,
    port: int,
    inflate: float,
    sample_stride: int,
    segments: tuple[str, ...],
    trim_pick_lift: bool,
    frame_object: str,
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
        paths = _load_action_paths(r3_plan_path, DEFAULT_ACTION)
        samples = _load_plan_samples(
            paths,
            segments,
            sample_stride,
            trim_pick_lift,
        )
        frame = sim.getObject(frame_object)
        frame_pose = sim.getObjectPose(frame, -1)
        joints = bridge.get_robot_joint_handles("R3")
        r3 = bridge.get_object_handle("R3")
        module = bridge.get_object_handle("CONTROL_MODULE_SUPPLY")
        tip = sim.getObject(R3_TIP_PATH)

        # Attach the module with R3 at the actual pick TCP so the payload sweep
        # follows the same tool-relative offset used during execution.
        _set_joints(sim, joints, paths["pick_descend"][-1])
        if not bridge.start_simulation():
            raise RuntimeError(bridge.last_error or "cannot start CoppeliaSim")
        for _ in range(3):
            if not bridge.step():
                raise RuntimeError(bridge.last_error or "cannot step CoppeliaSim")
        bridge.attach_object("CONTROL_MODULE_SUPPLY", "R3")
        if not bridge.step():
            raise RuntimeError(bridge.last_error or "cannot step CoppeliaSim")

        shapes = list(sim.getObjectsInTree(r3, sim.object_shape_type, 0))
        for shape in sim.getObjectsInTree(module, sim.object_shape_type, 0):
            if shape not in shapes:
                shapes.append(shape)

        sampled_boxes: list[dict[str, Any]] = []
        tip_trace: list[dict[str, Any]] = []
        for index, (segment, config) in enumerate(samples):
            _set_joints(sim, joints, config)
            if not bridge.step():
                raise RuntimeError(bridge.last_error or "cannot step CoppeliaSim")
            sampled_boxes.extend(
                _shape_box_in_frame(sim, shape, frame_pose, inflate)
                for shape in shapes
            )
            tip_trace.append(
                {
                    "sample_index": index,
                    "segment": segment,
                    "position": _tip_position(sim, tip, frame_pose),
                }
            )
        final_config = samples[-1][1]
        _set_joints(sim, joints, final_config)
        if not bridge.step():
            raise RuntimeError(bridge.last_error or "cannot step CoppeliaSim")
        final_boxes = [
            _shape_box_in_frame(sim, shape, frame_pose, inflate) for shape in shapes
        ]
        payload = {
            "frame_id": FRAME_ID,
            "frame_object": frame_object,
            "frame_pose_world": [float(value) for value in frame_pose],
            "source_scene": str(scene_path.resolve()),
            "source_plan": str(r3_plan_path.resolve()),
            "source_action": DEFAULT_ACTION,
            "segments": list(segments),
            "trim_pick_lift": bool(trim_pick_lift),
            "sample_stride": max(1, int(sample_stride)),
            "sample_count": len(samples),
            "shape_count": len(shapes),
            "inflate_m": float(inflate),
            "final_r3_joints_rad": final_config,
            "sweep_boxes": _merge_boxes(sampled_boxes),
            "final_boxes": _merge_boxes(final_boxes),
            "tip_trace": tip_trace,
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


def _load_obstacle_file(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if "sweep_boxes" not in data or "final_boxes" not in data:
        raise ValueError(f"{path} has no sweep/final boxes")
    return data


def _publish_obstacles(
    path: Path,
    collision_mode: str,
    rate_hz: float,
    clear: bool,
) -> None:
    import rclpy
    from geometry_msgs.msg import Point, Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    data = _load_obstacle_file(path)
    frame_id = data.get("frame_id", FRAME_ID)
    box_key = "final_boxes" if collision_mode == "final" else "sweep_boxes"
    collision_boxes = [] if collision_mode == "none" else data[box_key]
    marker_boxes = data["sweep_boxes"]

    def pose(center: list[float]) -> Pose:
        result = Pose()
        result.position.x = float(center[0])
        result.position.y = float(center[1])
        result.position.z = float(center[2])
        result.orientation.w = 1.0
        return result

    def point(values: list[float]) -> Point:
        return Point(x=float(values[0]), y=float(values[1]), z=float(values[2]))

    def collision_object(box: dict[str, Any], operation: int) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = str(box["id"])
        obj.operation = operation
        if operation == CollisionObject.ADD:
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(value) for value in box["size"]]
            obj.primitives = [primitive]
            obj.primitive_poses = [pose(box["center"])]
        return obj

    def box_marker(box: dict[str, Any], index: int) -> Marker:
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.ns = "r3_module_motion_sweep"
        msg.id = index
        msg.type = Marker.CUBE
        msg.action = Marker.DELETE if clear else Marker.ADD
        msg.pose = pose(box["center"])
        msg.scale.x = float(box["size"][0])
        msg.scale.y = float(box["size"][1])
        msg.scale.z = float(box["size"][2])
        msg.color = ColorRGBA(r=0.20, g=0.85, b=0.25, a=0.30)
        msg.lifetime.sec = 0
        return msg

    def tip_trace_marker(marker_id: int) -> Marker:
        msg = Marker()
        msg.header.frame_id = frame_id
        msg.ns = "r3_module_tip_trace"
        msg.id = marker_id
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.DELETE if clear else Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.012
        msg.color = ColorRGBA(r=1.0, g=0.80, b=0.05, a=0.95)
        msg.points = [point(item["position"]) for item in data["tip_trace"]]
        msg.lifetime.sec = 0
        return msg

    class R3MotionObstaclePublisher(Node):
        def __init__(self) -> None:
            super().__init__("r3_module_motion_obstacle_publisher")
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.scene_pub = self.create_publisher(
                PlanningScene, "/planning_scene", qos
            )
            self.object_pub = self.create_publisher(
                CollisionObject, "/collision_object", qos
            )
            self.marker_pub = self.create_publisher(
                MarkerArray, "/r3_module_motion_obstacle_markers", qos
            )
            self.visual_marker_pub = self.create_publisher(
                MarkerArray, "/visualization_marker_array", qos
            )
            self.timer = self.create_timer(max(0.1, 1.0 / rate_hz), self.publish_all)
            self.publish_all()

        def publish_all(self) -> None:
            operation = CollisionObject.REMOVE if clear else CollisionObject.ADD
            objects = [collision_object(box, operation) for box in collision_boxes]
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects = objects
            self.scene_pub.publish(scene)
            for obj in objects:
                self.object_pub.publish(obj)

            markers = [box_marker(box, index) for index, box in enumerate(marker_boxes)]
            markers.append(tip_trace_marker(len(markers)))
            marker_array = MarkerArray(markers=markers)
            self.marker_pub.publish(marker_array)
            self.visual_marker_pub.publish(marker_array)
            self.get_logger().info(
                f"{'removed' if clear else 'published'} R3 module motion obstacles: "
                f"collision_mode={collision_mode}, collision_objects={len(objects)}, "
                f"sweep_markers={len(marker_boxes)}, trace_points={len(data['tip_trace'])}"
            )

    rclpy.init()
    node = R3MotionObstaclePublisher()
    try:
        if clear:
            rclpy.spin_once(node, timeout_sec=1.0)
        else:
            rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--r3-plan", type=Path, default=DEFAULT_R3_PLAN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--inflate", type=float, default=0.015)
    parser.add_argument("--sample-stride", type=int, default=3)
    parser.add_argument(
        "--segments",
        default=",".join(DEFAULT_SEGMENTS),
        help="Comma-separated R3 module path names to sample.",
    )
    parser.add_argument(
        "--frame-object",
        default=DEFAULT_FRAME_OBJECT,
        help="CoppeliaSim object that corresponds to RViz dummy_link.",
    )
    parser.add_argument(
        "--include-pick-lift",
        action="store_true",
        help="Include the pick TCP->APP lift portion of lift_and_transfer.",
    )
    parser.add_argument("--generate-only", action="store_true")
    parser.add_argument(
        "--collision-mode",
        choices=("sweep", "final", "none"),
        default="sweep",
        help="Which R3 geometry is added to MoveIt collision objects.",
    )
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--clear", action="store_true")
    args = parser.parse_args()
    segments = tuple(part.strip() for part in args.segments.split(",") if part.strip())

    if args.generate_only:
        data = generate_obstacles(
            args.scene,
            args.r3_plan,
            args.output,
            args.host,
            args.port,
            args.inflate,
            args.sample_stride,
            segments,
            trim_pick_lift=not args.include_pick_lift,
            frame_object=args.frame_object,
        )
        print(
            f"wrote R3 module motion obstacles to {args.output}: "
            f"samples={data['sample_count']}, shapes={data['shape_count']}, "
            f"sweep_boxes={len(data['sweep_boxes'])}, final_boxes={len(data['final_boxes'])}"
        )
        return

    if not args.output.exists():
        raise SystemExit(
            f"{args.output} does not exist. Run with --generate-only first."
        )
    _publish_obstacles(args.output, args.collision_mode, args.rate, args.clear)


if __name__ == "__main__":
    main()
