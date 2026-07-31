#!/usr/bin/env python3
"""Publish the fixed inspection camera station in a robot-local RViz frame."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_FRAME_ID = "dummy_link"
DEFAULT_SCENE_ROOTS = {
    "R1": {
        "position": (-1.60, 0.65, 0.19),
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


@dataclass(frozen=True)
class BoxSpec:
    name: str
    world_position: tuple[float, float, float]
    size: tuple[float, float, float]
    collision: bool = True


CAMERA_BOXES = (
    BoxSpec("camera_column_base", (0.10, 0.55, 0.08), (0.110, 0.110, 0.080)),
    BoxSpec("camera_column", (0.10, 0.55, 0.48), (0.044, 0.044, 0.800)),
    BoxSpec("camera_bracket_x", (0.23, 0.55, 0.86), (0.300, 0.035, 0.035)),
    BoxSpec("camera_bracket_y", (0.35, 0.33, 0.86), (0.035, 0.450, 0.035)),
    BoxSpec("fixed_camera_body", (0.35, 0.15, 0.82), (0.080, 0.060, 0.060)),
    BoxSpec("fixed_camera_lens", (0.35, 0.15, 0.775), (0.035, 0.035, 0.035)),
    BoxSpec(
        "camera_view_area_marker",
        (0.35, 0.05, 0.218),
        (0.300, 0.220, 0.005),
        collision=False,
    ),
)


def _color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    return ColorRGBA(r=r, g=g, b=b, a=a)


def _yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _parse_vector(text: str) -> tuple[float, float, float]:
    values = [float(part.strip()) for part in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated values")
    return values[0], values[1], values[2]


def _relative_pose(
    world_position: tuple[float, float, float],
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> Pose:
    if abs(robot_root_rpy_deg[0]) > 1e-9 or abs(robot_root_rpy_deg[1]) > 1e-9:
        raise ValueError("camera obstacle transform supports zero roll/pitch only")
    dx = world_position[0] - robot_root_position[0]
    dy = world_position[1] - robot_root_position[1]
    dz = world_position[2] - robot_root_position[2]
    inverse_yaw = math.radians(-robot_root_rpy_deg[2])
    cos_yaw = math.cos(inverse_yaw)
    sin_yaw = math.sin(inverse_yaw)
    qx, qy, qz, qw = _yaw_quaternion(inverse_yaw)

    pose = Pose()
    pose.position.x = cos_yaw * dx - sin_yaw * dy
    pose.position.y = sin_yaw * dx + cos_yaw * dy
    pose.position.z = dz
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _collision_object(
    robot_id: str,
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
    obj.id = f"{robot_id.lower()}_{box.name}"
    obj.primitives = [primitive]
    obj.primitive_poses = [
        _relative_pose(box.world_position, robot_root_position, robot_root_rpy_deg)
    ]
    obj.operation = CollisionObject.ADD
    return obj


def _marker(
    robot_id: str,
    frame_id: str,
    box: BoxSpec,
    index: int,
    robot_root_position: tuple[float, float, float],
    robot_root_rpy_deg: tuple[float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = f"{robot_id.lower()}_fixed_camera_obstacles"
    marker.id = index
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = _relative_pose(
        box.world_position,
        robot_root_position,
        robot_root_rpy_deg,
    )
    marker.scale.x = box.size[0]
    marker.scale.y = box.size[1]
    marker.scale.z = box.size[2]
    marker.color = (
        _color(1.0, 0.20, 0.12, 0.55)
        if box.collision
        else _color(0.25, 0.75, 1.0, 0.28)
    )
    marker.lifetime.sec = 0
    return marker


class FixedCameraObstaclePublisher(Node):
    def __init__(
        self,
        robot_id: str,
        frame_id: str,
        robot_root_position: tuple[float, float, float],
        robot_root_rpy_deg: tuple[float, float, float],
        rate_hz: float,
    ) -> None:
        super().__init__(f"{robot_id.lower()}_fixed_camera_obstacle_publisher")
        self.robot_id = robot_id
        self.frame_id = frame_id
        self.robot_root_position = robot_root_position
        self.robot_root_rpy_deg = robot_root_rpy_deg
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", qos)
        self.collision_pub = self.create_publisher(
            CollisionObject,
            "/collision_object",
            qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            f"/{robot_id.lower()}_fixed_camera_obstacle_markers",
            qos,
        )
        self.visual_marker_pub = self.create_publisher(
            MarkerArray,
            "/visualization_marker_array",
            qos,
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_all)
        self.publish_all()
        self.get_logger().info(
            f"publishing fixed camera obstacles for {robot_id} in {frame_id}; "
            f"collision boxes={sum(1 for box in CAMERA_BOXES if box.collision)}, "
            "camera_view_area_marker visual-only"
        )

    def publish_all(self) -> None:
        collision_objects = [
            _collision_object(
                self.robot_id,
                self.frame_id,
                box,
                self.robot_root_position,
                self.robot_root_rpy_deg,
            )
            for box in CAMERA_BOXES
            if box.collision
        ]
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        self.scene_pub.publish(scene)
        for obj in collision_objects:
            self.collision_pub.publish(obj)

        markers = MarkerArray(
            markers=[
                _marker(
                    self.robot_id,
                    self.frame_id,
                    box,
                    index,
                    self.robot_root_position,
                    self.robot_root_rpy_deg,
                )
                for index, box in enumerate(CAMERA_BOXES)
            ]
        )
        self.marker_pub.publish(markers)
        self.visual_marker_pub.publish(markers)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", choices=sorted(DEFAULT_SCENE_ROOTS), default="R4")
    parser.add_argument("--frame", default=DEFAULT_FRAME_ID)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument(
        "--scene-root-position",
        type=_parse_vector,
        default=None,
        help="Measured robot root world position as x,y,z.",
    )
    parser.add_argument(
        "--scene-root-rpy-deg",
        type=_parse_vector,
        default=None,
        help="Measured robot root world orientation as roll,pitch,yaw degrees.",
    )
    args = parser.parse_args()

    scene_root = DEFAULT_SCENE_ROOTS[args.robot]
    robot_root_position = args.scene_root_position or scene_root["position"]
    robot_root_rpy_deg = args.scene_root_rpy_deg or scene_root["rpy_deg"]

    rclpy.init()
    node = FixedCameraObstaclePublisher(
        args.robot,
        args.frame,
        robot_root_position,
        robot_root_rpy_deg,
        args.rate,
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
