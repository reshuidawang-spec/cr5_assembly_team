#!/usr/bin/env python3
"""Publish animated R1 reference geometry and sweep markers for RViz."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REFERENCE_FILE = (
    REPO_ROOT
    / "data"
    / "manual_waypoints"
    / "r1_motion_reference"
    / "r1_box_transfer_reference.json"
)
DEFAULT_SWEEP_SEGMENTS = ("box_lift_and_transfer",)
BOX_PAYLOAD_ALIAS = "Box_Blank_payload"
BOX_PAYLOAD_SIZE_XYZ = (0.23, 0.17, 0.09)
BOX_PAYLOAD_OFFSET_FROM_R1T_MAIN_BODY_XYZ = (-0.004, -0.004, -0.359)


def _color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


COLORS = {
    "current_link": _color(0.16, 0.66, 1.0, 0.45),
    "current_tool": _color(1.0, 0.78, 0.18, 0.58),
    "current_payload": _color(0.72, 0.72, 0.72, 0.55),
    "sweep_link": _color(1.0, 0.22, 0.10, 0.075),
    "sweep_tool": _color(1.0, 0.52, 0.06, 0.105),
    "sweep_payload": _color(0.80, 0.80, 0.80, 0.16),
    "path": _color(1.0, 1.0, 1.0, 0.75),
    "label": _color(0.95, 0.95, 0.95, 0.95),
}


def _parse_csv(text: str) -> tuple[str, ...]:
    return tuple(item.strip() for item in text.split(",") if item.strip())


def _point(pose_xyzw: list[float]) -> Point:
    return Point(x=float(pose_xyzw[0]), y=float(pose_xyzw[1]), z=float(pose_xyzw[2]))


def _quat(pose_xyzw: list[float]) -> Quaternion:
    return Quaternion(
        x=float(pose_xyzw[3]),
        y=float(pose_xyzw[4]),
        z=float(pose_xyzw[5]),
        w=float(pose_xyzw[6]),
    )


def _pose(pose_xyzw: list[float]) -> Pose:
    pose = Pose()
    pose.position = _point(pose_xyzw)
    pose.orientation = _quat(pose_xyzw)
    return pose


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


def _scale(size: list[float], inflate_m: float) -> tuple[float, float, float]:
    return (
        max(0.001, float(size[0]) + 2.0 * inflate_m),
        max(0.001, float(size[1]) + 2.0 * inflate_m),
        max(0.001, float(size[2]) + 2.0 * inflate_m),
    )


def _sanitize_id(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", text).strip("_").lower()


def _primitive(size: list[float] | tuple[float, float, float]) -> SolidPrimitive:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in size]
    return primitive


def _shape_bounds(shape: dict, inflate_m: float) -> tuple[list[float], list[float]]:
    pose = [float(value) for value in shape["pose_xyzw"]]
    size = _scale(shape["scale_xyz"], inflate_m)
    lower = [math.inf, math.inf, math.inf]
    upper = [-math.inf, -math.inf, -math.inf]
    for x_sign in (-0.5, 0.5):
        for y_sign in (-0.5, 0.5):
            for z_sign in (-0.5, 0.5):
                rotated = _rotate_vector(
                    tuple(pose[3:]),
                    (size[0] * x_sign, size[1] * y_sign, size[2] * z_sign),
                )
                point = [pose[index] + rotated[index] for index in range(3)]
                lower = [min(a, b) for a, b in zip(lower, point)]
                upper = [max(a, b) for a, b in zip(upper, point)]
    return lower, upper


def _center_size(lower: list[float], upper: list[float]) -> tuple[list[float], list[float]]:
    center = [(lower[index] + upper[index]) / 2.0 for index in range(3)]
    size = [upper[index] - lower[index] for index in range(3)]
    return center, size


class R1MotionReferencePublisher(Node):
    def __init__(
        self,
        reference_file: Path,
        frame_id: str | None,
        rate_hz: float,
        animation_speed: float,
        sweep_segments: tuple[str, ...],
        sweep_stride: int,
        inflate_m: float,
        topics: tuple[str, ...],
        collision_mode: str,
        include_box_payload: bool,
    ) -> None:
        super().__init__("r1_motion_reference_publisher")
        self._reference_file = Path(reference_file)
        data = json.loads(self._reference_file.read_text(encoding="utf-8"))
        self._samples = data["samples"]
        if not self._samples:
            raise ValueError(f"{self._reference_file} has no samples")
        self._shape_aliases = tuple(data["shape_aliases"])
        self._frame_id = frame_id or data.get("frame_id", "dummy_link")
        self._animation_speed = max(0.05, float(animation_speed))
        self._sweep_segments = set(sweep_segments)
        self._sweep_stride = max(1, int(sweep_stride))
        self._inflate_m = max(0.0, float(inflate_m))
        self._collision_mode = collision_mode
        self._include_box_payload = include_box_payload
        self._pubs = [self.create_publisher(MarkerArray, topic, 1) for topic in topics]
        self._collision_objects = self._build_collision_objects()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._scene_pub = self.create_publisher(PlanningScene, "/planning_scene", qos)
        self._object_pub = self.create_publisher(CollisionObject, "/collision_object", qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)
        self._tick = 0
        self.get_logger().info(
            f"publishing animated R1 reference from {self._reference_file} "
            f"in frame {self._frame_id} to {', '.join(topics)}; "
            f"collision_mode={collision_mode}, "
            f"collision_objects={len(self._collision_objects)}, "
            f"include_box_payload={include_box_payload}"
        )

    def _stamp(self, marker: Marker, namespace: str, marker_id: int) -> Marker:
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = False
        return marker

    def _shape_marker(
        self,
        namespace: str,
        marker_id: int,
        alias: str,
        shape: dict,
        color: ColorRGBA,
        inflate_m: float,
    ) -> Marker:
        marker = self._stamp(Marker(), namespace, marker_id)
        marker.type = Marker.CUBE
        marker.pose.position = _point(shape["pose_xyzw"])
        marker.pose.orientation = _quat(shape["pose_xyzw"])
        sx, sy, sz = _scale(shape["scale_xyz"], inflate_m)
        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz
        marker.color = color
        marker.text = alias
        return marker

    def _sample_shapes(self, sample: dict, include_payload: bool) -> list[tuple[str, dict]]:
        shapes = [(alias, sample["shapes"][alias]) for alias in self._shape_aliases]
        if (
            include_payload
            and self._include_box_payload
            and sample["segment"] in self._sweep_segments
            and "R1T_main_body" in sample["shapes"]
        ):
            main_body = sample["shapes"]["R1T_main_body"]
            pose = [float(value) for value in main_body["pose_xyzw"]]
            pose[0] += BOX_PAYLOAD_OFFSET_FROM_R1T_MAIN_BODY_XYZ[0]
            pose[1] += BOX_PAYLOAD_OFFSET_FROM_R1T_MAIN_BODY_XYZ[1]
            pose[2] += BOX_PAYLOAD_OFFSET_FROM_R1T_MAIN_BODY_XYZ[2]
            shapes.append(
                (
                    BOX_PAYLOAD_ALIAS,
                    {
                        "pose_xyzw": pose,
                        "scale_xyz": list(BOX_PAYLOAD_SIZE_XYZ),
                    },
                )
            )
        return shapes

    def _selected_sweep_samples(self) -> list[dict]:
        selected = []
        for sample_index, sample in enumerate(self._samples):
            if sample["segment"] not in self._sweep_segments:
                continue
            if sample_index % self._sweep_stride != 0:
                continue
            selected.append(sample)
        return selected

    def _collision_object(
        self,
        object_id: str,
        size: list[float] | tuple[float, float, float],
        pose: Pose,
    ) -> CollisionObject:
        obj = CollisionObject()
        obj.header.frame_id = self._frame_id
        obj.id = object_id
        obj.operation = CollisionObject.ADD
        obj.primitives = [_primitive(size)]
        obj.primitive_poses = [pose]
        return obj

    def _build_sample_collision_objects(self) -> list[CollisionObject]:
        objects = []
        for sample in self._selected_sweep_samples():
            for alias, shape in self._sample_shapes(sample, include_payload=True):
                objects.append(
                    self._collision_object(
                        "r1_motion_sample_"
                        f"{sample['sample_index']:03d}_{_sanitize_id(alias)}",
                        _scale(shape["scale_xyz"], self._inflate_m),
                        _pose(shape["pose_xyzw"]),
                    )
                )
        return objects

    def _build_sweep_collision_objects(self) -> list[CollisionObject]:
        bounds_by_alias: dict[str, tuple[list[float], list[float]]] = {}
        for sample in self._selected_sweep_samples():
            for alias, shape in self._sample_shapes(sample, include_payload=True):
                lower, upper = _shape_bounds(shape, self._inflate_m)
                if alias not in bounds_by_alias:
                    bounds_by_alias[alias] = (lower, upper)
                    continue
                current_lower, current_upper = bounds_by_alias[alias]
                bounds_by_alias[alias] = (
                    [min(a, b) for a, b in zip(current_lower, lower)],
                    [max(a, b) for a, b in zip(current_upper, upper)],
                )

        objects = []
        for alias, (lower, upper) in bounds_by_alias.items():
            center, size = _center_size(lower, upper)
            pose = Pose()
            pose.position.x = center[0]
            pose.position.y = center[1]
            pose.position.z = center[2]
            pose.orientation.w = 1.0
            objects.append(
                self._collision_object(
                    f"r1_motion_sweep_{_sanitize_id(alias)}",
                    size,
                    pose,
                )
            )
        return objects

    def _build_collision_objects(self) -> list[CollisionObject]:
        if self._collision_mode == "none":
            return []
        if self._collision_mode == "samples":
            return self._build_sample_collision_objects()
        return self._build_sweep_collision_objects()

    def _current_markers(self, sample: dict, start_id: int) -> tuple[list[Marker], int]:
        markers: list[Marker] = []
        marker_id = start_id
        for alias, shape in self._sample_shapes(sample, include_payload=True):
            if alias == BOX_PAYLOAD_ALIAS:
                color = COLORS["current_payload"]
            elif alias.startswith("R1T_"):
                color = COLORS["current_tool"]
            else:
                color = COLORS["current_link"]
            markers.append(
                self._shape_marker(
                    "r1_reference_current",
                    marker_id,
                    alias,
                    shape,
                    color,
                    0.0,
                )
            )
            marker_id += 1

        label = self._stamp(Marker(), "r1_reference_label", marker_id)
        marker_id += 1
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.x = -0.15
        label.pose.position.y = 0.36
        label.pose.position.z = 0.62
        label.scale.z = 0.045
        label.color = COLORS["label"]
        label.text = (
            "R1 motion reference: "
            f"{sample['segment']}[{sample['path_index']}]"
        )
        markers.append(label)
        return markers, marker_id

    def _sweep_markers(self, start_id: int) -> tuple[list[Marker], int]:
        markers: list[Marker] = []
        marker_id = start_id
        path_points: list[Point] = []
        for sample_index, sample in enumerate(self._samples):
            if sample["segment"] not in self._sweep_segments:
                continue
            if sample_index % self._sweep_stride != 0:
                continue
            for alias, shape in self._sample_shapes(sample, include_payload=True):
                if alias == BOX_PAYLOAD_ALIAS:
                    color = COLORS["sweep_payload"]
                elif alias.startswith("R1T_"):
                    color = COLORS["sweep_tool"]
                else:
                    color = COLORS["sweep_link"]
                markers.append(
                    self._shape_marker(
                        "r1_reference_sweep",
                        marker_id,
                        alias,
                        shape,
                        color,
                        self._inflate_m,
                    )
                )
                marker_id += 1
            if "R1T_main_body" in sample["shapes"]:
                path_points.append(_point(sample["shapes"]["R1T_main_body"]["pose_xyzw"]))

        if len(path_points) >= 2:
            path = self._stamp(Marker(), "r1_reference_path", marker_id)
            marker_id += 1
            path.type = Marker.LINE_STRIP
            path.pose.orientation.w = 1.0
            path.scale.x = 0.008
            path.color = COLORS["path"]
            path.points = path_points
            markers.append(path)
        return markers, marker_id

    def _publish(self) -> None:
        sample_index = int(self._tick * self._animation_speed) % len(self._samples)
        self._tick += 1
        markers: list[Marker] = []

        delete_all = self._stamp(Marker(), "r1_reference", 0)
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)

        current, next_id = self._current_markers(self._samples[sample_index], 1)
        markers.extend(current)
        sweep, _ = self._sweep_markers(next_id)
        markers.extend(sweep)

        message = MarkerArray(markers=markers)
        for pub in self._pubs:
            pub.publish(message)
        if self._collision_objects:
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects = self._collision_objects
            self._scene_pub.publish(scene)
            for obj in self._collision_objects:
                self._object_pub.publish(obj)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-file", type=Path, default=DEFAULT_REFERENCE_FILE)
    parser.add_argument("--frame", default=None)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--animation-speed", type=float, default=0.65)
    parser.add_argument("--sweep-segments", type=_parse_csv, default=DEFAULT_SWEEP_SEGMENTS)
    parser.add_argument("--sweep-stride", type=int, default=4)
    parser.add_argument("--inflate-m", type=float, default=0.025)
    parser.add_argument(
        "--collision-mode",
        choices=("none", "sweep", "samples"),
        default="none",
        help="Publish selected R1 sweep geometry into MoveIt's planning scene.",
    )
    parser.add_argument(
        "--include-box-payload",
        action="store_true",
        help="Add an approximate carried Box_Blank volume to the selected R1 sweep.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=None,
        help="MarkerArray topic to publish. Can be specified more than once.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    topics = args.topic or ["/visualization_marker_array", "/r1_motion_reference"]
    rclpy.init()
    node = R1MotionReferencePublisher(
        args.reference_file,
        args.frame,
        args.rate,
        args.animation_speed,
        tuple(args.sweep_segments),
        args.sweep_stride,
        args.inflate_m,
        tuple(dict.fromkeys(topics)),
        args.collision_mode,
        args.include_box_payload,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
