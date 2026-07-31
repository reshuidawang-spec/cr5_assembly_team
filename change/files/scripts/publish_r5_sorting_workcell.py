#!/usr/bin/env python3
"""Publish the R5 sorting targets and conveyors in the R5 RViz frame."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POINTS_FILE = REPO_ROOT / "configs" / "points.yaml"
DEFAULT_FRAME_ID = "dummy_link"
DEFAULT_R5_ROOT_POSITION = (0.35, -0.50, 0.19)
DEFAULT_R5_ROOT_RPY_DEG = (0.0, 0.0, 110.0)

WORKPIECE_SCALE = 0.60
BOX_L = 0.35 * WORKPIECE_SCALE
BOX_W = 0.25 * WORKPIECE_SCALE
PRODUCT_ON_BELT_Z = 0.270

R5_TARGET_NAMES = (
    "R5_PRODUCT_PICK_APP",
    "R5_PRODUCT_PICK_TCP",
    "R5_GOOD_PLACE_APP",
    "R5_GOOD_PLACE_TCP",
    "R5_DEFECT_PLACE_APP",
    "R5_DEFECT_PLACE_TCP",
)


@dataclass(frozen=True)
class BoxSpec:
    name: str
    world_position: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float, float]
    collision: bool = False
    world_yaw_deg: float = 0.0


@dataclass(frozen=True)
class LineSpec:
    name: str
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    color: tuple[float, float, float, float]
    width: float = 0.012
    arrow: bool = False


@dataclass(frozen=True)
class ConveyorSpec:
    name: str
    target_name: str
    center: tuple[float, float, float]
    length: float
    width: float
    direction: str
    color: tuple[float, float, float, float]


CONVEYORS = (
    ConveyorSpec(
        name="Good_Conveyor",
        target_name="R5_GOOD_PLACE_TCP",
        center=(0.85, -1.72, 0.18),
        length=1.25,
        width=0.36,
        direction="Y",
        color=(0.10, 0.78, 0.22, 0.55),
    ),
    ConveyorSpec(
        name="Defect_Conveyor",
        target_name="R5_DEFECT_PLACE_TCP",
        center=(-0.75, -1.12, 0.18),
        length=1.20,
        width=0.36,
        direction="X",
        color=(1.00, 0.34, 0.10, 0.55),
    ),
)


def _color(values: tuple[float, float, float, float]) -> ColorRGBA:
    return ColorRGBA(r=values[0], g=values[1], b=values[2], a=values[3])


def _yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _parse_vector(text: str) -> tuple[float, float, float]:
    values = [float(part.strip()) for part in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated values")
    return values[0], values[1], values[2]


def _load_points(points_file: Path) -> dict[str, tuple[float, float, float]]:
    with points_file.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{points_file} did not contain a YAML mapping")
    points: dict[str, tuple[float, float, float]] = {}
    missing: list[str] = []
    for name in R5_TARGET_NAMES:
        entry = data.get(name)
        position = entry.get("position") if isinstance(entry, dict) else None
        if position is None:
            missing.append(name)
            continue
        if len(position) != 3:
            raise ValueError(f"{name} in {points_file} does not have three coordinates")
        points[name] = (float(position[0]), float(position[1]), float(position[2]))
    if missing:
        raise ValueError(f"missing R5 target positions in {points_file}: {', '.join(missing)}")
    return points


def _relative_point(
    world_position: Iterable[float],
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> Point:
    if abs(robot_root_rpy_deg[0]) > 1e-9 or abs(robot_root_rpy_deg[1]) > 1e-9:
        raise ValueError("R5 workcell transform supports zero root roll/pitch only")
    x, y, z = [float(value) for value in world_position]
    dx = x - robot_root_position[0]
    dy = y - robot_root_position[1]
    dz = z - robot_root_position[2]
    inverse_yaw = math.radians(-robot_root_rpy_deg[2])
    cos_yaw = math.cos(inverse_yaw)
    sin_yaw = math.sin(inverse_yaw)
    return Point(
        x=cos_yaw * dx - sin_yaw * dy,
        y=sin_yaw * dx + cos_yaw * dy,
        z=dz,
    )


def _relative_pose(
    world_position: tuple[float, float, float],
    world_yaw_deg: float,
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> Pose:
    point = _relative_point(world_position, robot_root_position, robot_root_rpy_deg)
    local_yaw = math.radians(world_yaw_deg - robot_root_rpy_deg[2])
    qx, qy, qz, qw = _yaw_quaternion(local_yaw)
    pose = Pose()
    pose.position = point
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _conveyor_boxes(conveyor: ConveyorSpec) -> list[BoxSpec]:
    x, y, z = conveyor.center
    frame_h = 0.12
    leg_w = 0.035
    leg_h = max(z - frame_h / 2.0, 0.02)
    leg_z = leg_h / 2.0
    boxes: list[BoxSpec] = []
    if conveyor.direction == "Y":
        boxes.extend(
            [
                BoxSpec(
                    f"{conveyor.name}_Frame",
                    (x, y, z),
                    (conveyor.width + 0.10, conveyor.length, frame_h),
                    (0.42, 0.42, 0.42, 0.38),
                ),
                BoxSpec(
                    f"{conveyor.name}_Belt",
                    (x, y, z + 0.075),
                    (conveyor.width, conveyor.length - 0.08, 0.030),
                    (0.02, 0.02, 0.02, 0.65),
                ),
            ]
        )
        for index, (lx, ly) in enumerate(
            (
                (x - conveyor.width / 2.0, y - conveyor.length / 2.0 + 0.12),
                (x + conveyor.width / 2.0, y - conveyor.length / 2.0 + 0.12),
                (x - conveyor.width / 2.0, y + conveyor.length / 2.0 - 0.12),
                (x + conveyor.width / 2.0, y + conveyor.length / 2.0 - 0.12),
            ),
            start=1,
        ):
            boxes.append(
                BoxSpec(
                    f"{conveyor.name}_Leg_{index}",
                    (lx, ly, leg_z),
                    (leg_w, leg_w, leg_h),
                    (0.42, 0.42, 0.42, 0.35),
                )
            )
    elif conveyor.direction == "X":
        boxes.extend(
            [
                BoxSpec(
                    f"{conveyor.name}_Frame",
                    (x, y, z),
                    (conveyor.length, conveyor.width + 0.10, frame_h),
                    (0.42, 0.42, 0.42, 0.38),
                ),
                BoxSpec(
                    f"{conveyor.name}_Belt",
                    (x, y, z + 0.075),
                    (conveyor.length - 0.08, conveyor.width, 0.030),
                    (0.02, 0.02, 0.02, 0.65),
                ),
            ]
        )
        for index, (lx, ly) in enumerate(
            (
                (x - conveyor.length / 2.0 + 0.12, y - conveyor.width / 2.0),
                (x - conveyor.length / 2.0 + 0.12, y + conveyor.width / 2.0),
                (x + conveyor.length / 2.0 - 0.12, y - conveyor.width / 2.0),
                (x + conveyor.length / 2.0 - 0.12, y + conveyor.width / 2.0),
            ),
            start=1,
        ):
            boxes.append(
                BoxSpec(
                    f"{conveyor.name}_Leg_{index}",
                    (lx, ly, leg_z),
                    (leg_w, leg_w, leg_h),
                    (0.42, 0.42, 0.42, 0.35),
                )
            )
    else:
        raise ValueError(f"unknown conveyor direction: {conveyor.direction}")
    return boxes


def _conveyor_lines(conveyor: ConveyorSpec) -> list[LineSpec]:
    x, y, z = conveyor.center
    top_z = z + 0.092
    half_l = conveyor.length / 2.0
    half_w = conveyor.width / 2.0
    if conveyor.direction == "Y":
        start = (x, y + half_l, top_z)
        end = (x, y - half_l, top_z)
        left_start = (x - half_w, y + half_l, top_z)
        left_end = (x - half_w, y - half_l, top_z)
        right_start = (x + half_w, y + half_l, top_z)
        right_end = (x + half_w, y - half_l, top_z)
    else:
        start = (x + half_l, y, top_z)
        end = (x - half_l, y, top_z)
        left_start = (x + half_l, y - half_w, top_z)
        left_end = (x - half_l, y - half_w, top_z)
        right_start = (x + half_l, y + half_w, top_z)
        right_end = (x - half_l, y + half_w, top_z)
    return [
        LineSpec(
            f"{conveyor.name}_long_centerline",
            start,
            end,
            conveyor.color,
            width=0.018,
            arrow=True,
        ),
        LineSpec(
            f"{conveyor.name}_left_edge",
            left_start,
            left_end,
            (0.92, 0.92, 0.92, 0.72),
            width=0.008,
        ),
        LineSpec(
            f"{conveyor.name}_right_edge",
            right_start,
            right_end,
            (0.92, 0.92, 0.92, 0.72),
            width=0.008,
        ),
    ]


def _target_lines(targets: dict[str, tuple[float, float, float]]) -> list[LineSpec]:
    lines: list[LineSpec] = []
    for name, color in (
        ("R5_GOOD_PLACE_TCP", (0.10, 1.00, 0.22, 0.95)),
        ("R5_DEFECT_PLACE_TCP", (1.00, 0.30, 0.10, 0.95)),
    ):
        x, y, _ = targets[name]
        z = PRODUCT_ON_BELT_Z + 0.010
        lines.extend(
            [
                LineSpec(
                    f"{name}_cross_x",
                    (x - 0.12, y, z),
                    (x + 0.12, y, z),
                    color,
                    width=0.010,
                ),
                LineSpec(
                    f"{name}_cross_y",
                    (x, y - 0.12, z),
                    (x, y + 0.12, z),
                    color,
                    width=0.010,
                ),
                LineSpec(
                    f"{name}_vertical",
                    (x, y, z - 0.09),
                    (x, y, z + 0.18),
                    color,
                    width=0.008,
                ),
            ]
        )
    return lines


def _target_footprints(targets: dict[str, tuple[float, float, float]]) -> list[BoxSpec]:
    return [
        BoxSpec(
            "GOOD_target_box_footprint_long_edge_parallel_to_belt",
            (targets["R5_GOOD_PLACE_TCP"][0], targets["R5_GOOD_PLACE_TCP"][1], PRODUCT_ON_BELT_Z + 0.006),
            (BOX_L, BOX_W, 0.012),
            (0.10, 1.00, 0.22, 0.22),
            collision=False,
            world_yaw_deg=90.0,
        ),
        BoxSpec(
            "DEFECT_target_box_footprint_long_edge_parallel_to_belt",
            (targets["R5_DEFECT_PLACE_TCP"][0], targets["R5_DEFECT_PLACE_TCP"][1], PRODUCT_ON_BELT_Z + 0.006),
            (BOX_L, BOX_W, 0.012),
            (1.00, 0.30, 0.10, 0.22),
            collision=False,
            world_yaw_deg=0.0,
        ),
    ]


def _collision_object(
    frame_id: str,
    box: BoxSpec,
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> CollisionObject:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(box.size)
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = f"r5_sorting_{box.name}"
    obj.primitives = [primitive]
    obj.primitive_poses = [
        _relative_pose(
            box.world_position,
            box.world_yaw_deg,
            robot_root_position,
            robot_root_rpy_deg,
        )
    ]
    obj.operation = CollisionObject.ADD
    return obj


def _box_marker(
    frame_id: str,
    box: BoxSpec,
    marker_id: int,
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "r5_sorting_workcell_boxes"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = _relative_pose(
        box.world_position,
        box.world_yaw_deg,
        robot_root_position,
        robot_root_rpy_deg,
    )
    marker.scale.x = box.size[0]
    marker.scale.y = box.size[1]
    marker.scale.z = box.size[2]
    marker.color = _color(box.color)
    marker.text = box.name
    marker.lifetime.sec = 0
    return marker


def _line_marker(
    frame_id: str,
    line: LineSpec,
    marker_id: int,
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "r5_sorting_workcell_lines"
    marker.id = marker_id
    marker.type = Marker.ARROW if line.arrow else Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = line.width
    marker.scale.y = line.width * 2.5 if line.arrow else 0.0
    marker.scale.z = line.width * 4.0 if line.arrow else 0.0
    marker.color = _color(line.color)
    marker.points = [
        _relative_point(line.start, robot_root_position, robot_root_rpy_deg),
        _relative_point(line.end, robot_root_position, robot_root_rpy_deg),
    ]
    marker.text = line.name
    marker.lifetime.sec = 0
    return marker


def _sphere_marker(
    frame_id: str,
    name: str,
    world_position: tuple[float, float, float],
    marker_id: int,
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "r5_sorting_target_centers"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = _relative_point(world_position, robot_root_position, robot_root_rpy_deg)
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.030
    marker.scale.y = 0.030
    marker.scale.z = 0.030
    marker.color = _color((0.20, 0.75, 1.00, 0.95))
    if "GOOD" in name:
        marker.color = _color((0.10, 1.00, 0.22, 0.95))
    elif "DEFECT" in name:
        marker.color = _color((1.00, 0.30, 0.10, 0.95))
    marker.text = name
    marker.lifetime.sec = 0
    return marker


def _label_marker(
    frame_id: str,
    name: str,
    world_position: tuple[float, float, float],
    marker_id: int,
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = "r5_sorting_labels"
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position = _relative_point(
        (world_position[0], world_position[1], world_position[2] + 0.055),
        robot_root_position,
        robot_root_rpy_deg,
    )
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.045
    marker.color = _color((0.95, 0.95, 0.95, 1.0))
    marker.text = name
    marker.lifetime.sec = 0
    return marker


class R5SortingWorkcellPublisher(Node):
    def __init__(
        self,
        frame_id: str,
        points_file: Path,
        robot_root_position: tuple[float, float, float],
        robot_root_rpy_deg: tuple[float, float, float],
        rate_hz: float,
        collision_mode: str,
    ) -> None:
        super().__init__("r5_sorting_workcell_publisher")
        self._frame_id = frame_id
        self._targets = _load_points(points_file)
        self._robot_root_position = robot_root_position
        self._robot_root_rpy_deg = robot_root_rpy_deg
        self._collision_mode = collision_mode
        self._boxes = self._build_boxes()
        self._lines = self._build_lines()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            "/r5_sorting_workcell_markers",
            qos,
        )
        self._visual_pub = self.create_publisher(
            MarkerArray,
            "/visualization_marker_array",
            qos,
        )
        self._scene_pub = self.create_publisher(PlanningScene, "/planning_scene", qos)
        self._collision_pub = self.create_publisher(CollisionObject, "/collision_object", qos)
        self._timer = self.create_timer(1.0 / rate_hz, self.publish_all)
        self.publish_all()
        self.get_logger().info(
            "publishing R5 sorting workcell: "
            f"targets={len(self._targets)}, boxes={len(self._boxes)}, "
            f"lines={len(self._lines)}, collision_mode={collision_mode}"
        )

    def _build_boxes(self) -> list[BoxSpec]:
        boxes: list[BoxSpec] = []
        for conveyor in CONVEYORS:
            boxes.extend(_conveyor_boxes(conveyor))
        boxes.extend(_target_footprints(self._targets))
        return boxes

    def _build_lines(self) -> list[LineSpec]:
        lines: list[LineSpec] = []
        for conveyor in CONVEYORS:
            lines.extend(_conveyor_lines(conveyor))
        lines.extend(_target_lines(self._targets))
        return lines

    def _collision_boxes(self) -> list[BoxSpec]:
        if self._collision_mode == "visual":
            return []
        if self._collision_mode == "belt":
            return [box for box in self._boxes if box.name.endswith("_Belt")]
        if self._collision_mode == "full":
            return [box for box in self._boxes if not box.name.endswith("_footprint")]
        raise ValueError(f"unknown collision mode: {self._collision_mode}")

    def publish_all(self) -> None:
        markers: list[Marker] = []
        marker_id = 0
        for box in self._boxes:
            markers.append(
                _box_marker(
                    self._frame_id,
                    box,
                    marker_id,
                    self._robot_root_position,
                    self._robot_root_rpy_deg,
                )
            )
            marker_id += 1
        for line in self._lines:
            markers.append(
                _line_marker(
                    self._frame_id,
                    line,
                    marker_id,
                    self._robot_root_position,
                    self._robot_root_rpy_deg,
                )
            )
            marker_id += 1
        for name in R5_TARGET_NAMES:
            point = self._targets[name]
            markers.append(
                _sphere_marker(
                    self._frame_id,
                    name,
                    point,
                    marker_id,
                    self._robot_root_position,
                    self._robot_root_rpy_deg,
                )
            )
            marker_id += 1
            markers.append(
                _label_marker(
                    self._frame_id,
                    name,
                    point,
                    marker_id,
                    self._robot_root_position,
                    self._robot_root_rpy_deg,
                )
            )
            marker_id += 1
        marker_array = MarkerArray(markers=markers)
        self._marker_pub.publish(marker_array)
        self._visual_pub.publish(marker_array)

        collision_objects = [
            _collision_object(
                self._frame_id,
                box,
                self._robot_root_position,
                self._robot_root_rpy_deg,
            )
            for box in self._collision_boxes()
        ]
        if collision_objects:
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects = collision_objects
            self._scene_pub.publish(scene)
            for obj in collision_objects:
                self._collision_pub.publish(obj)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frame", default=DEFAULT_FRAME_ID)
    parser.add_argument("--points-file", type=Path, default=DEFAULT_POINTS_FILE)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument(
        "--scene-root-position",
        type=_parse_vector,
        default=DEFAULT_R5_ROOT_POSITION,
        help="Measured R5 root world position as x,y,z.",
    )
    parser.add_argument(
        "--scene-root-rpy-deg",
        type=_parse_vector,
        default=DEFAULT_R5_ROOT_RPY_DEG,
        help="Measured R5 root world orientation as roll,pitch,yaw degrees.",
    )
    parser.add_argument(
        "--collision-mode",
        choices=("visual", "belt", "full"),
        default="visual",
        help="visual publishes markers only; belt/full also add conveyor collision objects.",
    )
    args = parser.parse_args()
    rclpy.init()
    node = R5SortingWorkcellPublisher(
        frame_id=args.frame,
        points_file=args.points_file,
        robot_root_position=args.scene_root_position,
        robot_root_rpy_deg=args.scene_root_rpy_deg,
        rate_hz=args.rate,
        collision_mode=args.collision_mode,
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
