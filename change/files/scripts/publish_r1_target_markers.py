#!/usr/bin/env python3
"""Publish process targets as RViz MarkerArray markers.

The marker positions are transformed from Coppelia world coordinates into the
selected robot's local RViz frame.  This is important for manual teaching:
targets shown in RViz must be relative to the robot base, not raw scene-world
coordinates.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POINTS_FILE = REPO_ROOT / "configs" / "points.yaml"
DEFAULT_ROBOTS_FILE = REPO_ROOT / "configs" / "robots.yaml"
DEFAULT_SCENE_ROOTS = {
    "R1": {
        "position": (-1.6, 0.65, 0.19),
        "rpy_deg": (0.0, 0.0, -35.0),
    },
    "R2": {
        "position": (-1.56, -0.22, 0.19),
        "rpy_deg": (0.0, 0.0, 20.0),
    },
    "R3": {
        "position": (-0.62, 0.40, 0.19),
        "rpy_deg": (0.0, 0.0, 15.0),
    },
    "R4": {
        "position": (0.78, 0.25, 0.19),
        "rpy_deg": (0.0, 0.0, -150.0),
    },
    "R5": {
        "position": (0.35, -0.50, 0.19),
        "rpy_deg": (0.0, 0.0, 110.0),
    },
}
TARGETS_BY_ROBOT = {
    "R1": (
        "R1_BOX_PICK_APP",
        "R1_BOX_PICK_TCP",
        "R1_BOX_PLACE_APP",
        "R1_BOX_PLACE_TCP",
        "R1_TERMINAL_PICK_APP",
        "R1_TERMINAL_PICK_TCP",
        "R1_TERMINAL_PLACE_APP",
        "R1_TERMINAL_PLACE_TCP",
    ),
    "R2": (
        "R2_PCB_PICK_APP",
        "R2_PCB_PICK_TCP",
        "R2_PCB_PLACE_APP",
        "R2_PCB_PLACE_TCP",
    ),
    "R3": (
        "R3_MODULE_PICK_APP",
        "R3_MODULE_PICK_TCP",
        "R3_MODULE_PLACE_APP",
        "R3_MODULE_PLACE_TCP",
        "R3_PRODUCT_PICK_APP",
        "R3_PRODUCT_PICK_TCP",
        "R3_PRODUCT_PLACE_INSPECTION_APP",
        "R3_PRODUCT_PLACE_INSPECTION_TCP",
    ),
    "R4": (
        "R4_HOME_REF",
        "R4_SCREW_APP",
        "R4_SCREW_TCP",
        "R4_SCREW_PRESS",
    ),
    "R5": (
        "R5_HOME_REF",
        "R5_PRODUCT_PICK_APP",
        "R5_PRODUCT_PICK_TCP",
        "R5_GOOD_PLACE_APP",
        "R5_GOOD_PLACE_TCP",
        "R5_DEFECT_PLACE_APP",
        "R5_DEFECT_PLACE_TCP",
    ),
}
TARGET_LINES_BY_ROBOT = {
    "R1": (
        ("box_pick_descent", "R1_BOX_PICK_APP", "R1_BOX_PICK_TCP"),
        ("box_transfer", "R1_BOX_PICK_APP", "R1_BOX_PLACE_APP"),
        ("box_place_descent", "R1_BOX_PLACE_APP", "R1_BOX_PLACE_TCP"),
        ("terminal_pick_descent", "R1_TERMINAL_PICK_APP", "R1_TERMINAL_PICK_TCP"),
        ("terminal_transfer", "R1_TERMINAL_PICK_APP", "R1_TERMINAL_PLACE_APP"),
        ("terminal_place_descent", "R1_TERMINAL_PLACE_APP", "R1_TERMINAL_PLACE_TCP"),
    ),
    "R2": (
        ("pcb_pick_descent", "R2_PCB_PICK_APP", "R2_PCB_PICK_TCP"),
        ("pcb_transfer", "R2_PCB_PICK_APP", "R2_PCB_PLACE_APP"),
        ("pcb_place_descent", "R2_PCB_PLACE_APP", "R2_PCB_PLACE_TCP"),
    ),
    "R3": (
        ("module_pick_descent", "R3_MODULE_PICK_APP", "R3_MODULE_PICK_TCP"),
        ("module_transfer", "R3_MODULE_PICK_APP", "R3_MODULE_PLACE_APP"),
        ("module_place_descent", "R3_MODULE_PLACE_APP", "R3_MODULE_PLACE_TCP"),
        ("product_pick_descent", "R3_PRODUCT_PICK_APP", "R3_PRODUCT_PICK_TCP"),
        (
            "product_transfer",
            "R3_PRODUCT_PICK_APP",
            "R3_PRODUCT_PLACE_INSPECTION_APP",
        ),
        (
            "product_place_descent",
            "R3_PRODUCT_PLACE_INSPECTION_APP",
            "R3_PRODUCT_PLACE_INSPECTION_TCP",
        ),
    ),
    "R4": (
        ("home_to_screw_app", "R4_HOME_REF", "R4_SCREW_APP"),
        ("screw_descent", "R4_SCREW_APP", "R4_SCREW_TCP"),
        ("screw_press", "R4_SCREW_TCP", "R4_SCREW_PRESS"),
    ),
    "R5": (
        ("home_to_product_pick_app", "R5_HOME_REF", "R5_PRODUCT_PICK_APP"),
        ("product_pick_descent", "R5_PRODUCT_PICK_APP", "R5_PRODUCT_PICK_TCP"),
        ("good_transfer", "R5_PRODUCT_PICK_APP", "R5_GOOD_PLACE_APP"),
        ("good_place_descent", "R5_GOOD_PLACE_APP", "R5_GOOD_PLACE_TCP"),
        ("defect_transfer", "R5_PRODUCT_PICK_APP", "R5_DEFECT_PLACE_APP"),
        ("defect_place_descent", "R5_DEFECT_PLACE_APP", "R5_DEFECT_PLACE_TCP"),
    ),
}


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


COLORS = {
    "box_app": _color(0.10, 0.55, 1.00, 1.0),
    "box_tcp": _color(0.00, 0.95, 0.85, 1.0),
    "terminal_app": _color(1.00, 0.72, 0.10, 1.0),
    "terminal_tcp": _color(1.00, 0.30, 0.12, 1.0),
    "pcb_app": _color(0.20, 0.85, 0.35, 1.0),
    "pcb_tcp": _color(0.05, 0.95, 0.95, 1.0),
    "module_app": _color(0.68, 0.55, 1.00, 1.0),
    "module_tcp": _color(0.95, 0.45, 1.00, 1.0),
    "product_app": _color(0.30, 0.75, 1.00, 1.0),
    "product_tcp": _color(0.10, 0.95, 0.55, 1.0),
    "screw_home": _color(0.85, 0.85, 0.85, 1.0),
    "screw_app": _color(1.00, 0.78, 0.18, 1.0),
    "screw_tcp": _color(1.00, 0.30, 0.12, 1.0),
    "screw_press": _color(0.90, 0.05, 0.05, 1.0),
    "sort_home": _color(0.85, 0.85, 0.85, 1.0),
    "good_app": _color(0.20, 0.90, 0.25, 1.0),
    "good_tcp": _color(0.05, 0.65, 0.12, 1.0),
    "defect_app": _color(1.00, 0.45, 0.15, 1.0),
    "defect_tcp": _color(0.95, 0.10, 0.08, 1.0),
    "label": _color(0.95, 0.95, 0.95, 1.0),
    "line": _color(0.65, 0.85, 1.00, 0.9),
}


def _target_color(name: str) -> ColorRGBA:
    if name == "R5_HOME_REF":
        return COLORS["sort_home"]
    if "GOOD" in name and name.endswith("_TCP"):
        return COLORS["good_tcp"]
    if "GOOD" in name:
        return COLORS["good_app"]
    if "DEFECT" in name and name.endswith("_TCP"):
        return COLORS["defect_tcp"]
    if "DEFECT" in name:
        return COLORS["defect_app"]
    if name == "R4_HOME_REF":
        return COLORS["screw_home"]
    if name.endswith("_PRESS"):
        return COLORS["screw_press"]
    if "SCREW" in name and name.endswith("_TCP"):
        return COLORS["screw_tcp"]
    if "SCREW" in name:
        return COLORS["screw_app"]
    if "MODULE" in name and name.endswith("_TCP"):
        return COLORS["module_tcp"]
    if "MODULE" in name:
        return COLORS["module_app"]
    if "PRODUCT" in name and name.endswith("_TCP"):
        return COLORS["product_tcp"]
    if "PRODUCT" in name:
        return COLORS["product_app"]
    if "PCB" in name and name.endswith("_TCP"):
        return COLORS["pcb_tcp"]
    if "PCB" in name:
        return COLORS["pcb_app"]
    if "TERMINAL" in name and name.endswith("_TCP"):
        return COLORS["terminal_tcp"]
    if "TERMINAL" in name:
        return COLORS["terminal_app"]
    if name.endswith("_TCP"):
        return COLORS["box_tcp"]
    return COLORS["box_app"]


def _point(values: Iterable[float]) -> Point:
    x, y, z = values
    return Point(x=float(x), y=float(y), z=float(z))


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return data


def _load_robot_base(robots_file: Path, robot_id: str) -> Point:
    data = _load_yaml(robots_file)
    position = data.get("robots", {}).get(robot_id, {}).get("position")
    if position is None:
        raise ValueError(f"missing robots.{robot_id}.position in {robots_file}")
    return _point(position)


def _relative_to_config_base(point: Point, base: Point) -> Point:
    return Point(x=point.x - base.x, y=point.y - base.y, z=point.z - base.z)


def _parse_vector(text: str) -> tuple[float, float, float]:
    values = [float(part.strip()) for part in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated values")
    return values[0], values[1], values[2]


def _relative_to_scene_root(
    point: Point,
    base_position: tuple[float, float, float],
    base_rpy_deg: tuple[float, float, float],
) -> Point:
    if abs(base_rpy_deg[0]) > 1e-9 or abs(base_rpy_deg[1]) > 1e-9:
        raise ValueError("RViz marker transform currently supports zero base roll/pitch only")
    dx = point.x - base_position[0]
    dy = point.y - base_position[1]
    dz = point.z - base_position[2]
    inverse_yaw = math.radians(-base_rpy_deg[2])
    cos_yaw = math.cos(inverse_yaw)
    sin_yaw = math.sin(inverse_yaw)
    return Point(
        x=cos_yaw * dx - sin_yaw * dy,
        y=sin_yaw * dx + cos_yaw * dy,
        z=dz,
    )


def _load_targets(
    points_file: Path,
    robots_file: Path,
    robot_id: str,
    coordinate_mode: str,
    scene_root_position: tuple[float, float, float],
    scene_root_rpy_deg: tuple[float, float, float],
) -> dict[str, Point]:
    data = _load_yaml(points_file)
    base = _load_robot_base(robots_file, robot_id)
    target_names = TARGETS_BY_ROBOT[robot_id]

    targets: dict[str, Point] = {}
    missing: list[str] = []
    for name in target_names:
        entry = data.get(name) if isinstance(data, dict) else None
        position = entry.get("position") if isinstance(entry, dict) else None
        if position is None:
            missing.append(name)
            continue
        world_point = _point(position)
        if coordinate_mode in {"scene-root", "r1-scene-root"}:
            targets[name] = _relative_to_scene_root(
                world_point,
                scene_root_position,
                scene_root_rpy_deg,
            )
        elif coordinate_mode == "config-translation":
            targets[name] = _relative_to_config_base(world_point, base)
        else:
            targets[name] = world_point

    if missing:
        raise ValueError(
            f"missing {robot_id} target positions in {points_file}: {', '.join(missing)}"
        )
    return targets


class R1TargetMarkerPublisher(Node):
    def __init__(
        self,
        points_file: Path,
        robots_file: Path,
        robot_id: str,
        coordinate_mode: str,
        frame_id: str,
        rate_hz: float,
        topics: tuple[str, ...],
        scene_root_position: tuple[float, float, float],
        scene_root_rpy_deg: tuple[float, float, float],
    ) -> None:
        self._robot_id = robot_id
        self._target_names = TARGETS_BY_ROBOT[robot_id]
        self._target_lines = TARGET_LINES_BY_ROBOT[robot_id]
        super().__init__(f"{robot_id.lower()}_target_marker_publisher")
        self._targets = _load_targets(
            points_file,
            robots_file,
            robot_id,
            coordinate_mode,
            scene_root_position,
            scene_root_rpy_deg,
        )
        self._frame_id = frame_id
        self._pubs = [self.create_publisher(MarkerArray, topic, 1) for topic in topics]
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f"publishing {len(self._targets)} {robot_id} target markers from {points_file} "
            f"in {frame_id} using {coordinate_mode} coordinates to {', '.join(topics)}"
        )

    def _stamp(self, marker: Marker) -> Marker:
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self._robot_id.lower()}_targets"
        marker.action = Marker.ADD
        marker.frame_locked = False
        return marker

    def _sphere_marker(self, marker_id: int, name: str, point: Point) -> Marker:
        marker = self._stamp(Marker())
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025
        marker.color = _target_color(name)
        return marker

    def _label_marker(self, marker_id: int, name: str, point: Point) -> Marker:
        marker = self._stamp(Marker())
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z + 0.04
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.04
        marker.color = COLORS["label"]
        marker.text = name
        return marker

    def _line_marker(self, marker_id: int, name: str, start: Point, end: Point) -> Marker:
        marker = self._stamp(Marker())
        marker.id = marker_id
        marker.ns = f"{self._robot_id.lower()}_target_lines"
        marker.type = Marker.LINE_STRIP
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.008
        marker.color = COLORS["line"]
        marker.points = [start, end]
        marker.text = name
        return marker

    def _publish(self) -> None:
        markers: list[Marker] = []
        marker_id = 0
        for name in self._target_names:
            point = self._targets[name]
            markers.append(self._sphere_marker(marker_id, name, point))
            marker_id += 1
            markers.append(self._label_marker(marker_id, name, point))
            marker_id += 1

        for line_name, start_name, end_name in self._target_lines:
            markers.append(
                self._line_marker(
                    marker_id,
                    line_name,
                    self._targets[start_name],
                    self._targets[end_name],
                )
            )
            marker_id += 1

        message = MarkerArray(markers=markers)
        for pub in self._pubs:
            pub.publish(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=sorted(TARGETS_BY_ROBOT), default="R1")
    parser.add_argument("--points-file", type=Path, default=DEFAULT_POINTS_FILE)
    parser.add_argument("--robots-file", type=Path, default=DEFAULT_ROBOTS_FILE)
    parser.add_argument(
        "--coordinate-mode",
        choices=("scene-root", "r1-scene-root", "config-translation", "coppelia-world"),
        default="scene-root",
        help=(
            "scene-root applies the measured robot scene-root inverse transform; "
            "config-translation only subtracts configs/robots.yaml robot position"
        ),
    )
    parser.add_argument(
        "--scene-root-position",
        type=_parse_vector,
        default=None,
        help="Measured robot world position as x,y,z.",
    )
    parser.add_argument(
        "--scene-root-rpy-deg",
        type=_parse_vector,
        default=None,
        help="Measured robot world orientation as roll,pitch,yaw degrees.",
    )
    parser.add_argument("--frame", default="dummy_link")
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument(
        "--topic",
        action="append",
        default=None,
        help="MarkerArray topic to publish. Can be specified more than once.",
    )
    args = parser.parse_args()
    scene_root = DEFAULT_SCENE_ROOTS[args.robot]
    scene_root_position = args.scene_root_position or scene_root["position"]
    scene_root_rpy_deg = args.scene_root_rpy_deg or scene_root["rpy_deg"]
    topics = args.topic or [
        "/visualization_marker_array",
        f"/{args.robot.lower()}_target_markers",
    ]

    rclpy.init()
    node = R1TargetMarkerPublisher(
        args.points_file,
        args.robots_file,
        args.robot,
        args.coordinate_mode,
        args.frame,
        args.rate,
        tuple(dict.fromkeys(topics)),
        scene_root_position,
        scene_root_rpy_deg,
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
