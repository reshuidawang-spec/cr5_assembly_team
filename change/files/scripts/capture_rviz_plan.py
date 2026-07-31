#!/usr/bin/env python3
"""Capture the next RViz/MoveIt display trajectory as JSON."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "data" / "manual_waypoints"


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _duration_to_float(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0


def _rad_to_deg(values: list[float]) -> list[float]:
    return [math.degrees(float(value)) for value in values]


def _round_list(values: list[float], digits: int = 9) -> list[float]:
    return [round(float(value), digits) for value in values]


def _round_deg(values: list[float]) -> list[float]:
    return [round(value, 6) for value in _rad_to_deg(values)]


def _joint_state_record(msg: JointState | None) -> dict[str, Any] | None:
    if msg is None:
        return None
    return {
        "name": list(msg.name),
        "position_rad": _round_list(list(msg.position)),
        "position_deg": _round_deg(list(msg.position)),
        "velocity": _round_list(list(msg.velocity)) if msg.velocity else [],
        "effort": _round_list(list(msg.effort)) if msg.effort else [],
    }


def _trajectory_record(msg: DisplayTrajectory) -> dict[str, Any]:
    trajectories = []
    for index, robot_trajectory in enumerate(msg.trajectory):
        joint_trajectory = robot_trajectory.joint_trajectory
        points = []
        for point in joint_trajectory.points:
            positions = list(point.positions)
            points.append(
                {
                    "time_from_start_s": round(_duration_to_float(point.time_from_start), 9),
                    "positions_rad": _round_list(positions),
                    "positions_deg": _round_deg(positions),
                    "velocities": _round_list(list(point.velocities)) if point.velocities else [],
                    "accelerations": _round_list(list(point.accelerations))
                    if point.accelerations
                    else [],
                }
            )

        first = points[0] if points else None
        last = points[-1] if points else None
        max_step_deg = 0.0
        if len(points) >= 2:
            for before, after in zip(points, points[1:]):
                step = max(
                    abs(a - b)
                    for a, b in zip(before["positions_deg"], after["positions_deg"])
                )
                max_step_deg = max(max_step_deg, step)

        trajectories.append(
            {
                "index": index,
                "joint_names": list(joint_trajectory.joint_names),
                "point_count": len(points),
                "duration_s": last["time_from_start_s"] if last else 0.0,
                "first_point": first,
                "last_point": last,
                "max_adjacent_joint_step_deg": round(max_step_deg, 6),
                "points": points,
            }
        )

    return {
        "model_id": msg.model_id,
        "trajectory_start": _joint_state_record(msg.trajectory_start.joint_state),
        "trajectory_count": len(trajectories),
        "trajectories": trajectories,
    }


class RvizPlanCapture(Node):
    def __init__(
        self,
        output_dir: Path,
        output_file: Path | None,
        keep_going: bool,
    ) -> None:
        super().__init__("rviz_plan_capture")
        self._output_dir = output_dir
        self._output_file = output_file
        self._keep_going = keep_going
        self._capture_count = 0
        self._latest_joint_state: JointState | None = None
        self._start_wall_time = time.time()
        self._done = False
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            self._on_display_trajectory,
            5,
        )
        self.get_logger().info("waiting for next /display_planned_path message")

    @property
    def done(self) -> bool:
        return self._done

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _on_display_trajectory(self, msg: DisplayTrajectory) -> None:
        if self._done:
            return
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._capture_count += 1
        if self._output_file is not None and not self._keep_going:
            output = self._output_file
        else:
            output = self._output_dir / (
                f"rviz_plan_capture_{_stamp()}_{self._capture_count:02d}.json"
            )
        record = {
            "captured_at": datetime.now().isoformat(timespec="seconds"),
            "capture_index": self._capture_count,
            "capture_wait_wall_s": round(time.time() - self._start_wall_time, 6),
            "topics": {
                "display_trajectory": "/display_planned_path",
                "joint_states": "/joint_states",
            },
            "latest_joint_state_at_capture": _joint_state_record(self._latest_joint_state),
            "display_trajectory": _trajectory_record(msg),
        }
        output.write_text(json.dumps(record, indent=2), encoding="utf-8")
        self.get_logger().info(f"captured RViz plan to {output}")
        self._done = not self._keep_going


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--keep-going", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = RvizPlanCapture(args.output_dir, args.output, args.keep_going)
    deadline = time.time() + args.timeout
    try:
        while rclpy.ok() and not node.done and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        timed_out = not node.done
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if timed_out:
        raise SystemExit(f"timed out waiting for /display_planned_path after {args.timeout}s")


if __name__ == "__main__":
    main()
