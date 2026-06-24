#!/usr/bin/env python3
"""RMP60 vertical probing helper.

The first version assumes vertical probing along robot Z- and only uses DI1.
It does not configure the robot Tool frame; the flange-to-tip length is only
used when recording the estimated contact point.
"""
import argparse
import csv
import json
import os
import re
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dobot_msgs_v4.srv import DI, GetPose, MovL, SpeedFactor, Stop

VERTICAL_PROBE_ACK_ARG = "--ack-vertical-stylus-ok"
VERTICAL_PROBE_ACK_DEST = "ack_vertical_stylus_ok"
PROBE_SPIN_TIMEOUT_SEC = 0.001
PROBE_MOVL_ACCEPT_TIMEOUT_SEC = 0.02
PROBE_SEGMENT_SETTLE_SEC = 0.03
PROBE_SEGMENT_POSITION_TOLERANCE_MM = 0.03


def parse_pose(text):
    """Parse pose."""
    match = re.search(r"\{([^}]*)\}", text)
    if not match:
        raise ValueError(f"cannot parse pose from: {text}")
    values = [float(item.strip()) for item in match.group(1).split(",")]
    if len(values) != 6:
        raise ValueError(f"expected 6 pose values, got {len(values)}: {text}")
    return values


def check_service_result(client, result):
    """Check service result."""
    if result is None:
        raise RuntimeError(f"{client.srv_name} returned no result")
    if hasattr(result, "res") and int(result.res) != 0:
        detail = getattr(result, "robot_return", "")
        suffix = f": {detail}" if detail else ""
        raise RuntimeError(f"{client.srv_name} returned res={result.res}{suffix}")
    return result


class ProbeTouch(Node):
    def __init__(self):
        super().__init__("probe_touch")
        self.di1 = None
        self.last_feed_time = None
        self.last_feed_wall_time = None
        self.feed_sequence = 0
        self.feed_pose = None
        self.feed_joints = None
        self.feed_digital_input_bits = None
        self.create_subscription(String, "/dobot_bringup_ros2/msg/FeedInfo", self._feed_cb, 10)
        self.di_cli = self.create_client(DI, "/dobot_bringup_ros2/srv/DI")
        self.pose_cli = self.create_client(GetPose, "/dobot_bringup_ros2/srv/GetPose")
        self.movl_cli = self.create_client(MovL, "/dobot_bringup_ros2/srv/MovL")
        self.speed_cli = self.create_client(SpeedFactor, "/dobot_bringup_ros2/srv/SpeedFactor")
        self.stop_cli = self.create_client(Stop, "/dobot_bringup_ros2/srv/Stop")

    def _feed_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.feed_sequence += 1
        digital_input_bits = int(data.get("digital_input_bits", 0))
        self.feed_digital_input_bits = digital_input_bits
        self.di1 = bool(digital_input_bits & 0x01)
        pose = data.get("tool_vector_actual")
        if isinstance(pose, list) and len(pose) >= 6:
            self.feed_pose = [float(v) for v in pose[:6]]
        joints = data.get("q_actual")
        if isinstance(joints, list) and len(joints) >= 6:
            self.feed_joints = [float(v) for v in joints[:6]]
        self.last_feed_time = time.monotonic()
        self.last_feed_wall_time = time.time()

    def feed_snapshot(self):
        """Feed snapshot."""
        return {
            "sequence": self.feed_sequence,
            "monotonic_time": self.last_feed_time,
            "wall_time": self.last_feed_wall_time,
            "pose": list(self.feed_pose) if self.feed_pose is not None else None,
            "joints": list(self.feed_joints) if self.feed_joints is not None else None,
            "digital_input_bits": self.feed_digital_input_bits,
            "di1": self.di1,
        }

    def wait_services(self, timeout_sec):
        """Wait services."""
        deadline = time.monotonic() + timeout_sec
        for client in (self.di_cli, self.pose_cli, self.movl_cli, self.speed_cli, self.stop_cli):
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
                raise RuntimeError(f"service not available: {client.srv_name}")

    def call(self, client, request, timeout_sec=10.0):
        """Call."""
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return check_service_result(client, future.result())
        raise TimeoutError(f"service timeout: {client.srv_name}")

    def call_async_checked(self, client, request, accept_timeout_sec=PROBE_MOVL_ACCEPT_TIMEOUT_SEC):
        """Call async checked."""
        future = client.call_async(request)
        deadline = time.monotonic() + accept_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
            if future.done():
                result = check_service_result(client, future.result())
                return future, result
        return future, None

    def check_ready_future(self, client, future):
        """Check ready future."""
        if future is not None and future.done():
            return check_service_result(client, future.result())
        return None

    def get_pose(self):
        """Get pose."""
        req = GetPose.Request()
        req.user = 0
        req.tool = 0
        return parse_pose(self.call(self.pose_cli, req).robot_return)

    def read_di1(self):
        """Read di1."""
        req = DI.Request()
        req.index = 1
        try:
            result = self.call(self.di_cli, req)
        except TimeoutError:
            if self.di1 is not None and self.last_feed_time is not None and time.monotonic() - self.last_feed_time <= 0.2:
                self.get_logger().warning("DI service timed out; using fresh FeedInfo DI1 state")
                return bool(self.di1)
            raise
        match = re.search(r"\{([^}]*)\}", result.robot_return)
        if match:
            return bool(int(match.group(1).split(",", 1)[0]))
        self.get_logger().warning("DI service returned no robot_return; using FeedInfo DI1 state")
        return bool(self.di1)

    def wait_fresh_feed(self, timeout_sec=3.0, max_age_sec=0.2):
        """Wait fresh feed."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.last_feed_time is not None and time.monotonic() - self.last_feed_time <= max_age_sec:
                return
        raise RuntimeError("FeedInfo is not fresh; refusing to move")

    def wait_until_pose(
        self,
        target_pose,
        position_tolerance_mm=0.2,
        orientation_tolerance_deg=0.5,
        timeout_sec=10.0,
    ):
        """Wait until pose."""
        deadline = time.monotonic() + timeout_sec
        last_pose = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.feed_pose is None:
                continue
            last_pose = list(self.feed_pose)
            position_error = sum((last_pose[index] - target_pose[index]) ** 2 for index in range(3)) ** 0.5
            orientation_error = max(
                abs((last_pose[index] - target_pose[index] + 180.0) % 360.0 - 180.0)
                for index in range(3, 6)
            )
            if position_error <= position_tolerance_mm and orientation_error <= orientation_tolerance_deg:
                return last_pose
        pose = self.get_pose()
        position_error = sum((pose[index] - target_pose[index]) ** 2 for index in range(3)) ** 0.5
        orientation_error = max(
            abs((pose[index] - target_pose[index] + 180.0) % 360.0 - 180.0)
            for index in range(3, 6)
        )
        if position_error <= position_tolerance_mm and orientation_error <= orientation_tolerance_deg:
            return pose
        self.stop_repeated()
        observed = last_pose if last_pose is not None else pose
        raise RuntimeError(f"positioning move did not reach requested pose; last observed pose was {observed}")

    def move_l(self, pose, timeout_sec):
        """Move l."""
        req = MovL.Request()
        req.mode = False
        req.a, req.b, req.c, req.d, req.e, req.f = pose
        return self.call(self.movl_cli, req, timeout_sec=timeout_sec)

    def set_speed(self, ratio):
        """Set speed."""
        req = SpeedFactor.Request()
        req.ratio = int(ratio)
        return self.call(self.speed_cli, req)

    def stop(self, timeout_sec=0.5):
        """Stop."""
        return self.call(self.stop_cli, Stop.Request(), timeout_sec=timeout_sec)

    def stop_repeated(self, count=3, interval_sec=0.05):
        """Stop repeated."""
        last = None
        for _ in range(count):
            try:
                last = self.stop()
            except Exception as exc:
                self.get_logger().warning(f"Stop() failed once: {exc}")
            time.sleep(interval_sec)
        return last

    def stop_fast_then_confirm(self, confirm_count=1, confirm_interval_sec=0.0):
        """Stop fast then confirm."""
        future = self.stop_cli.call_async(Stop.Request())
        last = self.stop_repeated(count=confirm_count, interval_sec=confirm_interval_sec)
        if future.done():
            try:
                check_service_result(self.stop_cli, future.result())
            except RuntimeError as exc:
                self.get_logger().warning(
                    f"first async Stop returned an error but robot is already stopped via backup: {exc}"
                )
        return last


def write_contact(path, row):
    """Write contact."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(row.keys())
    exists = path.exists()
    if exists:
        with path.open(newline="") as f:
            reader = csv.reader(f)
            header = next(reader, None)
        if header != fieldnames:
            backup = path.with_name(f"{path.stem}_{int(time.time())}{path.suffix}")
            os.replace(path, backup)
            print(f"CSV header changed; moved old file to {backup}")
            exists = False
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def validate_probe_motion(approach_mm, allow_long_approach, speed=None, prefix=""):
    """Validate probe motion."""
    if approach_mm <= 0:
        raise ValueError(f"{prefix}--approach-mm must be positive")
    if approach_mm > 5.0 and not allow_long_approach:
        raise ValueError(f"{prefix}--approach-mm > 5 requires --allow-long-approach")
    if speed is not None:
        validate_probe_speed(speed, prefix=prefix)


def validate_probe_speed(speed, prefix=""):
    """Validate probe speed."""
    if not 1 <= speed <= 5:
        raise ValueError(f"{prefix}--speed must be between 1 and 5 for probing")


def validate_probe_args(args, prefix=""):
    """Validate probe args."""
    if getattr(args, "execute", False) and not getattr(args, VERTICAL_PROBE_ACK_DEST, False):
        raise ValueError(
            f"{prefix}vertical probing real motion requires explicit on-site confirmation; "
            f"add {VERTICAL_PROBE_ACK_ARG} only after confirming the vertical stylus and this path are safe"
        )
    validate_probe_motion(
        args.approach_mm,
        getattr(args, "allow_long_approach", False),
        getattr(args, "speed", None),
        prefix=prefix,
    )
    retract_mm = getattr(args, "retract_mm", None)
    if retract_mm is not None and retract_mm <= 0:
        raise ValueError(f"{prefix}--retract-mm must be positive")
    timeout = getattr(args, "timeout", None)
    if timeout is not None and timeout <= 0:
        raise ValueError(f"{prefix}--timeout must be positive")
    probe_step_mm = getattr(args, "probe_step_mm", 0.0)
    if probe_step_mm is not None and probe_step_mm < 0:
        raise ValueError(f"{prefix}--probe-step-mm cannot be negative")


def add_vertical_probe_ack_argument(parser):
    """Add vertical probe ack argument."""
    parser.add_argument(
        VERTICAL_PROBE_ACK_ARG,
        dest=VERTICAL_PROBE_ACK_DEST,
        action="store_true",
        help="permit real vertical probing after confirming the vertical stylus is installed and safe",
    )


def effective_probe_step(total_distance_mm, probe_step_mm):
    """Effective probe step."""
    step = float(probe_step_mm or 0.0)
    if step <= 0.0:
        return float(total_distance_mm)
    return min(float(total_distance_mm), step)


def wait_segment_boundary_trigger(node, deadline, stale_message):
    """Wait segment boundary trigger."""
    settle_until = min(time.monotonic() + PROBE_SEGMENT_SETTLE_SEC, deadline)
    while rclpy.ok() and time.monotonic() < settle_until:
        rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
        if node.last_feed_time is None or time.monotonic() - node.last_feed_time > 0.2:
            node.stop_repeated()
            raise RuntimeError(stale_message)
        snapshot = node.feed_snapshot()
        if snapshot.get("di1"):
            return snapshot
    return None


def run_probe_cycle(node, args, cycle_index=None):
    """Run probe cycle."""
    validate_probe_args(args)
    tool_length = args.flange_mm + args.stylus_mm
    start_pose = node.get_pose()
    target_pose = start_pose.copy()
    target_pose[2] -= args.approach_mm
    prefix = f"[cycle {cycle_index}] " if cycle_index is not None else ""
    print(f"{prefix}current flange pose: {start_pose}")
    print(f"{prefix}probe target pose:   {target_pose}")
    print(f"{prefix}assumed flange-to-tip length: {tool_length:.1f} mm")
    print(f"{prefix}probe step: {effective_probe_step(args.approach_mm, getattr(args, 'probe_step_mm', 0.0)):.4f} mm")

    node.wait_fresh_feed()
    if node.read_di1() or node.di1:
        raise RuntimeError("DI1 is already triggered before probing; retract or clear the probe first")

    if not args.execute:
        print(f"{prefix}dry-run only; add --execute to start the probing motion")
        return None

    node.set_speed(args.speed)
    probe_step = effective_probe_step(args.approach_mm, getattr(args, "probe_step_mm", 0.0))
    issued_distance = 0.0
    segment_target = start_pose.copy()
    move_future = None
    immediate_move_result = None

    def issue_next_segment():
        """Issue next segment."""
        nonlocal issued_distance, segment_target, move_future, immediate_move_result
        remaining = args.approach_mm - issued_distance
        step = min(probe_step, remaining)
        issued_distance += step
        segment_target = start_pose.copy()
        segment_target[2] -= issued_distance
        move_req = MovL.Request()
        move_req.mode = False
        move_req.a, move_req.b, move_req.c, move_req.d, move_req.e, move_req.f = segment_target
        move_future, immediate_move_result = node.call_async_checked(
            node.movl_cli,
            move_req,
            accept_timeout_sec=PROBE_MOVL_ACCEPT_TIMEOUT_SEC,
        )
        if immediate_move_result is not None:
            print(f"{prefix}MovL accepted immediately")

    issue_next_segment()

    triggered = False
    trigger_pose = None
    reached_target = False
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
        if immediate_move_result is None:
            immediate_move_result = node.check_ready_future(node.movl_cli, move_future)
        if node.last_feed_time is None or time.monotonic() - node.last_feed_time > 0.2:
            node.stop_repeated()
            raise RuntimeError("FeedInfo became stale during probing; motion stopped")
        snapshot = node.feed_snapshot()
        if snapshot.get("di1"):
            triggered = True
            trigger_pose = snapshot["pose"] if snapshot.get("pose") is not None else None
            node.stop_fast_then_confirm()
            break
        if snapshot.get("pose") is not None:
            segment_tolerance = min(PROBE_SEGMENT_POSITION_TOLERANCE_MM, max(0.01, probe_step * 0.1))
            if abs(snapshot["pose"][2] - segment_target[2]) <= segment_tolerance:
                if issued_distance >= args.approach_mm - 1e-9:
                    reached_target = True
                    break
                boundary_trigger = wait_segment_boundary_trigger(
                    node,
                    deadline,
                    "FeedInfo became stale during probing; motion stopped",
                )
                if boundary_trigger is not None:
                    triggered = True
                    trigger_pose = boundary_trigger["pose"] if boundary_trigger.get("pose") is not None else None
                    node.stop_fast_then_confirm()
                    break
                issue_next_segment()

    row = None
    if triggered:
        contact_pose = node.get_pose()
        measurement_pose = trigger_pose if trigger_pose is not None else contact_pose
        tip_z = measurement_pose[2] - tool_length
        row = {
            "timestamp": f"{time.time():.3f}",
            "cycle": "" if cycle_index is None else str(cycle_index),
            "flange_x": f"{measurement_pose[0]:.4f}",
            "flange_y": f"{measurement_pose[1]:.4f}",
            "flange_z": f"{measurement_pose[2]:.4f}",
            "rx": f"{measurement_pose[3]:.4f}",
            "ry": f"{measurement_pose[4]:.4f}",
            "rz": f"{measurement_pose[5]:.4f}",
            "tip_x_est": f"{measurement_pose[0]:.4f}",
            "tip_y_est": f"{measurement_pose[1]:.4f}",
            "tip_z_est": f"{tip_z:.4f}",
            "tool_length_mm": f"{tool_length:.1f}",
            "probe_step_mm": f"{float(getattr(args, 'probe_step_mm', 0.0) or 0.0):.4f}",
            "stop_flange_x": f"{contact_pose[0]:.4f}",
            "stop_flange_y": f"{contact_pose[1]:.4f}",
            "stop_flange_z": f"{contact_pose[2]:.4f}",
        }
        print(f"{prefix}triggered at flange pose: {measurement_pose}")
        print(f"{prefix}stopped at flange pose: {contact_pose}")
        print(f"{prefix}estimated tip contact xyz: [{measurement_pose[0]:.4f}, {measurement_pose[1]:.4f}, {tip_z:.4f}]")
    else:
        node.stop_repeated()
        contact_pose = node.get_pose()
        reason = "target reached" if reached_target else "timeout"
        print(f"{prefix}no trigger before {reason}; final flange pose: {contact_pose}")

    retract_pose = contact_pose.copy()
    retract_pose[2] += args.retract_mm
    node.move_l(retract_pose, timeout_sec=10.0)
    node.wait_until_pose(retract_pose, timeout_sec=10.0)
    print(f"{prefix}retracted to z + {args.retract_mm:.1f} mm")
    node.wait_fresh_feed()
    probe_still_triggered = bool(node.read_di1() or node.di1)
    if row is not None:
        write_contact(Path(args.output), row)
        print(f"{prefix}saved: {args.output}")
    if probe_still_triggered:
        raise RuntimeError("DI1 remains triggered after retract; leave motion stopped and inspect the probe")
    return row


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    parser.add_argument("--approach-mm", type=float, default=3.0, help="vertical probing distance")
    parser.add_argument("--retract-mm", type=float, default=3.0, help="retract distance after trigger")
    parser.add_argument("--speed", type=int, default=1, help="SpeedFactor ratio during probing")
    parser.add_argument("--allow-long-approach", action="store_true", help="allow approach distance > 5 mm")
    parser.add_argument("--flange-mm", type=float, default=125.4, help="robot flange to cross-stylus branch origin")
    parser.add_argument("--stylus-mm", type=float, default=75.0, help="vertical stylus ball-centre distance")
    parser.add_argument("--timeout", type=float, default=20.0, help="probing timeout seconds")
    parser.add_argument(
        "--probe-step-mm",
        type=float,
        default=0.5,
        help="segment probing MovL into this step size; 0 disables segmented probing",
    )
    parser.add_argument("--output", default="data/probe_contacts.csv", help="CSV output path")
    add_vertical_probe_ack_argument(parser)
    args = parser.parse_args()
    try:
        validate_probe_args(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        run_probe_cycle(node, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
