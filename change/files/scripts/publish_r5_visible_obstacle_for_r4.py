#!/usr/bin/env python3
"""Publish R5 as visible-only RViz markers in the R4 teaching frame."""

from __future__ import annotations

import argparse
import itertools
import math
import re
import sys
from pathlib import Path
from typing import Any, Iterable

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sim_bridge.coppelia_client import SimBridge


DEFAULT_SCENE = REPO_ROOT / "scenes" / "compact_cell1ttt.ttt"
FRAME_ID = "dummy_link"
DEFAULT_FRAME_OBJECT = "/R4"
DEFAULT_ROBOT_OBJECT = "/R5"


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
    return [first[index] + translated[index] for index in range(3)] + list(
        _quaternion_multiply(tuple(first[3:]), tuple(second[3:]))
    )


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
        world_point = [world_bb_pose[index] + rotated[index] for index in range(3)]
        frame_point = _world_to_frame(frame_pose, world_point)
        lower = [min(a, b) for a, b in zip(lower, frame_point)]
        upper = [max(a, b) for a, b in zip(upper, frame_point)]
    inflated_lower = [value - inflate for value in lower]
    inflated_upper = [value + inflate for value in upper]
    center = [(inflated_lower[index] + inflated_upper[index]) / 2.0 for index in range(3)]
    dimensions = [inflated_upper[index] - inflated_lower[index] for index in range(3)]
    alias = sim.getObjectAlias(shape, 1)
    return {
        "id": f"r5_visible_{_sanitize_id(alias)}",
        "alias": alias,
        "center": center,
        "size": dimensions,
    }


def _set_joints(sim: Any, joints: list[int], config: list[float]) -> None:
    for joint, value in zip(joints, config):
        sim.setJointPosition(joint, float(value))
        sim.setJointTargetPosition(joint, float(value))


def _parse_joints_deg(text: str) -> list[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if len(values) != 6:
        raise argparse.ArgumentTypeError("expected six comma-separated joint values in degrees")
    return [math.radians(value) for value in values]


def load_r5_boxes(
    scene_path: Path,
    host: str,
    port: int,
    frame_object: str,
    robot_object: str,
    inflate: float,
    joints_rad: list[float] | None,
) -> list[dict[str, Any]]:
    bridge = SimBridge(host=host, port=port)
    if not bridge.connect(host, port):
        raise RuntimeError(bridge.last_error or "cannot connect to CoppeliaSim")
    sim = bridge.sim
    try:
        if sim.getSimulationState() != sim.simulation_stopped:
            if not bridge.stop_simulation():
                raise RuntimeError(bridge.last_error or "cannot stop CoppeliaSim")
        sim.loadScene(str(scene_path.resolve()))
        if joints_rad is not None:
            _set_joints(sim, bridge.get_robot_joint_handles("R5"), joints_rad)
        frame = sim.getObject(frame_object)
        robot = sim.getObject(robot_object)
        frame_pose = sim.getObjectPose(frame, -1)
        return [
            _shape_box_in_frame(sim, shape, frame_pose, inflate)
            for shape in sim.getObjectsInTree(robot, sim.object_shape_type, 0)
        ]
    finally:
        bridge.disconnect()


def _pose(center: list[float]) -> Pose:
    pose = Pose()
    pose.position.x = float(center[0])
    pose.position.y = float(center[1])
    pose.position.z = float(center[2])
    pose.orientation.w = 1.0
    return pose


def _color() -> ColorRGBA:
    return ColorRGBA(r=0.35, g=0.65, b=1.0, a=0.30)


class R5VisibleObstaclePublisher(Node):
    def __init__(self, boxes: list[dict[str, Any]], rate_hz: float) -> None:
        super().__init__("r5_visible_obstacle_for_r4_publisher")
        self._boxes = boxes
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray,
            "/r5_visible_obstacle_for_r4_markers",
            qos,
        )
        self._visual_pub = self.create_publisher(
            MarkerArray,
            "/visualization_marker_array",
            qos,
        )
        self._timer = self.create_timer(1.0 / rate_hz, self.publish_all)
        self.publish_all()
        self.get_logger().info(
            f"publishing R5 visible-only obstacle for R4: boxes={len(boxes)}, collision_objects=0"
        )

    def publish_all(self) -> None:
        markers: list[Marker] = []
        for index, box in enumerate(self._boxes):
            marker = Marker()
            marker.header.frame_id = FRAME_ID
            marker.ns = "r5_visible_obstacle_for_r4"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = _pose(box["center"])
            marker.scale.x = float(box["size"][0])
            marker.scale.y = float(box["size"][1])
            marker.scale.z = float(box["size"][2])
            marker.color = _color()
            marker.text = box["alias"]
            marker.lifetime.sec = 0
            markers.append(marker)
        message = MarkerArray(markers=markers)
        self._marker_pub.publish(message)
        self._visual_pub.publish(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--frame-object", default=DEFAULT_FRAME_OBJECT)
    parser.add_argument("--robot-object", default=DEFAULT_ROBOT_OBJECT)
    parser.add_argument("--inflate", type=float, default=0.0)
    parser.add_argument("--r5-joints-deg", type=_parse_joints_deg, default=None)
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()

    boxes = load_r5_boxes(
        args.scene,
        args.host,
        args.port,
        args.frame_object,
        args.robot_object,
        args.inflate,
        args.r5_joints_deg,
    )
    rclpy.init()
    node = R5VisibleObstaclePublisher(boxes, args.rate)
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
