#!/usr/bin/env python3
"""Publish RViz markers for arbitrary-direction probing poses."""
import argparse
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from moveit_utils import load_pose_specs


def color(r, g, b, a=1.0):
    """Color."""
    msg = ColorRGBA()
    msg.r = r
    msg.g = g
    msg.b = b
    msg.a = a
    return msg


def set_point(point, xyz, scale_factor):
    """Set point."""
    point.x = xyz[0] * scale_factor
    point.y = xyz[1] * scale_factor
    point.z = xyz[2] * scale_factor


def make_marker(frame_id, marker_id, namespace, marker_type, position, scale_xyz, marker_color, scale_factor):
    """Make marker."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    set_point(marker.pose.position, position, scale_factor)
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale_xyz[0]
    marker.scale.y = scale_xyz[1]
    marker.scale.z = scale_xyz[2]
    marker.color = marker_color
    return marker


def make_text(frame_id, marker_id, namespace, position, text, scale_factor):
    """Make text."""
    marker = make_marker(
        frame_id,
        marker_id,
        namespace,
        Marker.TEXT_VIEW_FACING,
        position,
        (0.0, 0.0, 0.035),
        color(1.0, 1.0, 1.0, 1.0),
        scale_factor,
    )
    marker.pose.position.z += 0.04
    marker.text = text
    return marker


def make_arrow(frame_id, marker_id, namespace, start, end, marker_color, scale_factor):
    """Make arrow."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.008
    marker.scale.y = 0.018
    marker.scale.z = 0.03
    marker.color = marker_color
    marker.points.append(Point())
    marker.points.append(Point())
    set_point(marker.points[0], start, scale_factor)
    set_point(marker.points[1], end, scale_factor)
    return marker


def make_markers(pose_specs, frame_id, scale_factor):
    """Make markers."""
    markers = MarkerArray()
    marker_id = 0
    for spec in pose_specs:
        name = spec.get("name", f"pose_{marker_id}")
        namespace = f"rmp60_{name}"
        contact = spec["contact"]
        safe = spec["safe_position"]
        target = spec["target_position"]

        markers.markers.append(
            make_marker(frame_id, marker_id, namespace, Marker.SPHERE, contact, (0.03, 0.03, 0.03), color(1.0, 0.8, 0.1), scale_factor)
        )
        marker_id += 1
        markers.markers.append(
            make_marker(frame_id, marker_id, namespace, Marker.CUBE, safe, (0.025, 0.025, 0.025), color(0.1, 0.45, 1.0), scale_factor)
        )
        marker_id += 1
        markers.markers.append(
            make_marker(frame_id, marker_id, namespace, Marker.CUBE, target, (0.025, 0.025, 0.025), color(1.0, 0.15, 0.15), scale_factor)
        )
        marker_id += 1
        markers.markers.append(
            make_arrow(frame_id, marker_id, namespace, safe, target, color(0.1, 1.0, 0.25), scale_factor)
        )
        marker_id += 1
        markers.markers.append(make_text(frame_id, marker_id, namespace, contact, f"{name}: contact", scale_factor))
        marker_id += 1
        markers.markers.append(make_text(frame_id, marker_id, namespace, safe, f"{name}: safe", scale_factor))
        marker_id += 1
        markers.markers.append(make_text(frame_id, marker_id, namespace, target, f"{name}: target", scale_factor))
        marker_id += 1
    return markers


class MeasurementPoseMarkerNode(Node):
    def __init__(self, markers, topic, publish_hz):
        super().__init__("rmp60_measurement_pose_markers")
        self.markers = markers
        self.publisher = self.create_publisher(MarkerArray, topic, 10)
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_markers)

    def publish_markers(self):
        """Publish markers."""
        now = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = now
        self.publisher.publish(self.markers)


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", help="JSON file from generate_measurement_poses.py --json")
    parser.add_argument("--contact", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--approach", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--reference-up", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("UX", "UY", "UZ"))
    parser.add_argument("--standoff-mm", type=float, default=20.0)
    parser.add_argument("--travel-mm", type=float, default=5.0)
    parser.add_argument("--name", default="probe_pose")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--unit", choices=("mm", "m"), default="mm", help="unit used by input positions")
    parser.add_argument("--topic", default="/rmp60_measurement_markers")
    parser.add_argument("--publish-hz", type=float, default=2.0)
    parser.add_argument("--once", action="store_true", help="publish once and exit")
    args = parser.parse_args()

    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")
    if args.publish_hz <= 0 or not math.isfinite(args.publish_hz):
        raise SystemExit("--publish-hz must be positive")

    try:
        pose_specs = load_pose_specs(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    scale_factor = 0.001 if args.unit == "mm" else 1.0
    markers = make_markers(pose_specs, args.frame_id, scale_factor)

    print(f"loaded pose specs: {len(pose_specs)}")
    print(f"publishing markers: {len(markers.markers)} on {args.topic}, frame={args.frame_id}, unit={args.unit}")

    rclpy.init()
    node = MeasurementPoseMarkerNode(markers, args.topic, args.publish_hz)
    try:
        node.publish_markers()
        if not args.once:
            rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
