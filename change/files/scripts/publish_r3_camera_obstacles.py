#!/usr/bin/env python3
"""Publish the fixed inspection camera as R3 MoveIt collision objects."""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


FRAME_ID = "dummy_link"
R3_ROOT_POSITION = (-0.62, 0.40, 0.19)
R3_ROOT_YAW_DEG = 15.0


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


def _relative_pose(world_position: tuple[float, float, float]) -> Pose:
    dx = world_position[0] - R3_ROOT_POSITION[0]
    dy = world_position[1] - R3_ROOT_POSITION[1]
    dz = world_position[2] - R3_ROOT_POSITION[2]
    inverse_yaw = math.radians(-R3_ROOT_YAW_DEG)
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


def _collision_object(box: BoxSpec) -> CollisionObject:
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(box.size)

    obj = CollisionObject()
    obj.header.frame_id = FRAME_ID
    obj.id = f"r3_{box.name}"
    obj.primitives = [primitive]
    obj.primitive_poses = [_relative_pose(box.world_position)]
    obj.operation = CollisionObject.ADD
    return obj


def _marker(box: BoxSpec, index: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.ns = "r3_camera_obstacles"
    marker.id = index
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = _relative_pose(box.world_position)
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


class R3CameraObstaclePublisher(Node):
    def __init__(self) -> None:
        super().__init__("r3_camera_obstacle_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", qos
        )
        self.collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", qos
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/r3_camera_obstacle_markers", qos
        )
        self.visual_marker_pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", qos
        )
        self.timer = self.create_timer(1.0, self.publish_all)
        self.publish_all()

    def publish_all(self) -> None:
        collision_objects = [
            _collision_object(box) for box in CAMERA_BOXES if box.collision
        ]
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = collision_objects
        self.scene_pub.publish(scene)
        for obj in collision_objects:
            self.collision_pub.publish(obj)

        markers = MarkerArray(
            markers=[_marker(box, index) for index, box in enumerate(CAMERA_BOXES)]
        )
        self.marker_pub.publish(markers)
        self.visual_marker_pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = R3CameraObstaclePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
