#!/usr/bin/env python3
"""MoveIt preflight gate for five-branch sphere calibration plans."""
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

from cross_probe_model import euler_to_matrix
from ros_wait_utils import wait_for_future


POSE_NAMES = ("x", "y", "z", "rx", "ry", "rz")


def parse_float(row, field):
    """Parse a string to float, returning a default for empty/missing values."""
    value = row.get(field, "")
    if value in (None, ""):
        raise ValueError(f"missing {field}")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def pose_from_row(row, prefix):
    """Extract a 6-element flange pose list from a CSV row dict."""
    return [parse_float(row, f"{prefix}_{name}") for name in POSE_NAMES]


def quat_from_matrix(m):
    """Quat from matrix."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return {
            "w": 0.25 * s,
            "x": (m[2][1] - m[1][2]) / s,
            "y": (m[0][2] - m[2][0]) / s,
            "z": (m[1][0] - m[0][1]) / s,
        }
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return {
            "w": (m[2][1] - m[1][2]) / s,
            "x": 0.25 * s,
            "y": (m[0][1] + m[1][0]) / s,
            "z": (m[0][2] + m[2][0]) / s,
        }
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return {
            "w": (m[0][2] - m[2][0]) / s,
            "x": (m[0][1] + m[1][0]) / s,
            "y": 0.25 * s,
            "z": (m[1][2] + m[2][1]) / s,
        }
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return {
        "w": (m[1][0] - m[0][1]) / s,
        "x": (m[0][2] + m[2][0]) / s,
        "y": (m[1][2] + m[2][1]) / s,
        "z": 0.25 * s,
    }


def pose_stamped_from_flange_pose(pose, frame_id, euler_sequence):
    """Pose stamped from flange pose."""
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = pose[0] * 0.001
    msg.pose.position.y = pose[1] * 0.001
    msg.pose.position.z = pose[2] * 0.001
    quat = quat_from_matrix(euler_to_matrix(euler_sequence, pose[3:6]))
    norm = math.sqrt(sum(value * value for value in quat.values()))
    msg.pose.orientation.x = quat["x"] / norm
    msg.pose.orientation.y = quat["y"] / norm
    msg.pose.orientation.z = quat["z"] / norm
    msg.pose.orientation.w = quat["w"] / norm
    return msg


class PlanMoveItChecker(Node):
    def __init__(self, joint_state_topic):
        super().__init__("five_branch_plan_moveit_checker")
        self.joint_state_topic = joint_state_topic
        self.joint_state = None
        self.joint_state_wall_time = None
        self.create_subscription(JointState, joint_state_topic, self._joint_state_cb, 10)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.cartesian_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")

    def _joint_state_cb(self, msg):
        self.joint_state = msg
        self.joint_state_wall_time = time.monotonic()

    def wait_ready(self, timeout_sec):
        """Wait ready."""
        if not self.ik_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/compute_ik service is not available")
        if not self.cartesian_cli.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/compute_cartesian_path service is not available")
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self.joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                raise RuntimeError(f"{self.joint_state_topic} is not available")
        if self.joint_state_wall_time is None or time.monotonic() - self.joint_state_wall_time > 1.0:
            raise RuntimeError("robot joint state is stale")
        required = {f"joint{index}" for index in range(1, 7)}
        missing = sorted(required - set(self.joint_state.name))
        if missing:
            raise RuntimeError("robot joint state is missing: " + ", ".join(missing))

    def solve_ik(self, pose, seed_state, args, avoid_collisions=True):
        """Solve ik."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = args.group
        req.ik_request.ik_link_name = args.ik_link
        req.ik_request.avoid_collisions = bool(avoid_collisions)
        req.ik_request.robot_state = seed_state
        req.ik_request.pose_stamped = pose_stamped_from_flange_pose(pose, args.frame_id, args.euler_sequence)
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.timeout = Duration(sec=int(args.ik_timeout), nanosec=int((args.ik_timeout % 1.0) * 1e9))
        future = self.ik_cli.call_async(req)
        result = wait_for_future(self, future, args.ik_timeout + 2.0, "/compute_ik")
        return result.error_code.val == MoveItErrorCodes.SUCCESS, result.error_code.val, result.solution

    def compute_path(self, start_state, waypoints, args, avoid_collisions=True):
        """Compute path."""
        req = GetCartesianPath.Request()
        req.header.frame_id = args.frame_id
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state = start_state
        req.group_name = args.group
        req.link_name = args.ik_link
        req.max_step = args.max_step_m
        req.jump_threshold = args.jump_threshold
        req.avoid_collisions = bool(avoid_collisions)
        req.waypoints = [
            pose_stamped_from_flange_pose(pose, args.frame_id, args.euler_sequence).pose
            for pose in waypoints
        ]
        future = self.cartesian_cli.call_async(req)
        result = wait_for_future(self, future, max(5.0, args.ik_timeout + 2.0), "/compute_cartesian_path")
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS and result.fraction >= args.min_fraction
        return ok, result.error_code.val, result.fraction, result.solution


def make_seed_state(joint_state):
    """Make seed state."""
    state = RobotState()
    state.joint_state = joint_state
    return state


def max_joint_delta_deg(start_state, end_state):
    """Max joint delta deg."""
    start = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
    end = dict(zip(end_state.joint_state.name, end_state.joint_state.position))
    names = [f"joint{index}" for index in range(1, 7)]
    if any(name not in start or name not in end for name in names):
        return math.inf
    deltas = []
    for name in names:
        delta = (float(end[name]) - float(start[name]) + math.pi) % (2.0 * math.pi) - math.pi
        deltas.append(abs(math.degrees(delta)))
    return max(deltas)


def write_csv(path, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not path or not rows:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", required=True, help="CSV from auto_calibrate_five_branch_sphere.py --plan-output")
    parser.add_argument("--checked-output")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--group", default="cr5_group")
    parser.add_argument("--ik-link", default="Link6", help="use Link6 for Dobot flange-pose plans")
    parser.add_argument("--joint-state-topic", default="/joint_states_robot")
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--ik-timeout", type=float, default=0.2)
    parser.add_argument("--max-step-m", type=float, default=0.001)
    parser.add_argument("--jump-threshold", type=float, default=0.0)
    parser.add_argument("--min-fraction", type=float, default=1.0)
    parser.add_argument("--service-timeout-sec", type=float, default=5.0)
    parser.add_argument("--max-ik-joint-step-deg", type=float, default=15.0)
    parser.add_argument(
        "--check-probe-with-moveit-collision",
        action="store_true",
        help=(
            "also require the short probe segment to be MoveIt collision-free. "
            "This usually fails if the calibration sphere is loaded as a collision object, "
            "because the active ruby ball is expected to contact it."
        ),
    )
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    if (
        args.ik_timeout <= 0
        or args.max_step_m <= 0
        or args.service_timeout_sec <= 0
        or args.max_ik_joint_step_deg <= 0
    ):
        raise SystemExit("timeouts and --max-step-m must be positive")
    if not 0.0 < args.min_fraction <= 1.0:
        raise SystemExit("--min-fraction must be in (0, 1]")

    try:
        with Path(args.plan).open(newline="") as f:
            rows = list(csv.DictReader(f))
    except OSError as exc:
        raise SystemExit(str(exc)) from exc

    rclpy.init()
    node = PlanMoveItChecker(args.joint_state_topic)
    checked_rows = []
    failures = 0
    try:
        node.wait_ready(args.service_timeout_sec)
        for row in rows:
            checked = dict(row)
            plan_id = row.get("plan_id", "")
            try:
                transition_only = row.get("preflight_mode", "").strip() == "transition-only"
                transition = pose_from_row(row, "transition_flange")
                start = None if transition_only else pose_from_row(row, "start_flange")
                target = None if transition_only else pose_from_row(row, "target_flange")
                safe = None
                if all(row.get(f"safe_flange_{name}") not in (None, "") for name in POSE_NAMES):
                    safe = pose_from_row(row, "safe_flange")
                seed = make_seed_state(node.joint_state)
                safe_ok = True
                safe_code = ""
                safe_state = seed
                if safe is not None:
                    safe_ok, safe_code, safe_state = node.solve_ik(safe, seed, args, True)
                transition_ok, transition_code, transition_state = node.solve_ik(
                    transition,
                    safe_state if safe_ok else seed,
                    args,
                    True,
                )
                start_ok = transition_only
                start_code = ""
                start_state = transition_state
                target_ok = transition_only
                target_code = ""
                if not transition_only:
                    start_ok, start_code, start_state = node.solve_ik(
                        start,
                        transition_state if transition_ok else seed,
                        args,
                        True,
                    )
                    target_ok, target_code, target_state = node.solve_ik(
                        target,
                        start_state if start_ok else seed,
                        args,
                        args.check_probe_with_moveit_collision,
                    )
                else:
                    target_state = start_state
                safe_joint_delta = 0.0 if safe is None else max_joint_delta_deg(seed, safe_state)
                transition_joint_delta = max_joint_delta_deg(
                    safe_state if safe is not None and safe_ok else seed,
                    transition_state,
                )
                start_joint_delta = 0.0 if transition_only else max_joint_delta_deg(transition_state, start_state)
                target_joint_delta = 0.0 if transition_only else max_joint_delta_deg(start_state, target_state)
                joint_continuity_ok = max(
                    safe_joint_delta,
                    transition_joint_delta,
                    start_joint_delta,
                    target_joint_delta,
                ) <= args.max_ik_joint_step_deg
                approach_ok = transition_only
                probe_ok = transition_only
                safe_transition_ok = safe is None
                safe_transition_fraction = "" if safe is None else "0.000000"
                approach_fraction = 0.0
                probe_fraction = 0.0
                safe_transition_code = "" if safe is None else ""
                approach_code = ""
                probe_code = ""
                if safe is not None and safe_ok:
                    safe_transition_ok, safe_transition_code, safe_transition_fraction_raw, _ = node.compute_path(
                        safe_state,
                        [transition],
                        args,
                        True,
                    )
                    safe_transition_fraction = f"{safe_transition_fraction_raw:.6f}"
                if transition_ok and not transition_only:
                    approach_ok, approach_code, approach_fraction, _ = node.compute_path(
                        transition_state,
                        [start],
                        args,
                        True,
                    )
                if start_ok and not transition_only:
                    probe_ok, probe_code, probe_fraction, _ = node.compute_path(
                        start_state,
                        [target],
                        args,
                        args.check_probe_with_moveit_collision,
                    )
                ok = (
                    safe_ok
                    and transition_ok
                    and start_ok
                    and target_ok
                    and safe_transition_ok
                    and approach_ok
                    and probe_ok
                    and joint_continuity_ok
                )
                checked.update(
                    {
                        "moveit_status": "OK" if ok else "FAIL",
                        "moveit_safe_ik_code": str(safe_code),
                        "moveit_transition_ik_code": str(transition_code),
                        "moveit_start_ik_code": str(start_code),
                        "moveit_target_ik_code": str(target_code),
                        "moveit_safe_transition_path_code": str(safe_transition_code),
                        "moveit_safe_transition_fraction": str(safe_transition_fraction),
                        "moveit_approach_path_code": str(approach_code),
                        "moveit_approach_fraction": f"{approach_fraction:.6f}",
                        "moveit_probe_path_code": str(probe_code),
                        "moveit_probe_fraction": f"{probe_fraction:.6f}",
                        "moveit_probe_collision_checked": str(int(args.check_probe_with_moveit_collision)),
                        "moveit_joint_continuity_ok": str(int(joint_continuity_ok)),
                        "moveit_safe_joint_delta_deg": f"{safe_joint_delta:.6f}",
                        "moveit_transition_joint_delta_deg": f"{transition_joint_delta:.6f}",
                        "moveit_start_joint_delta_deg": f"{start_joint_delta:.6f}",
                        "moveit_target_joint_delta_deg": f"{target_joint_delta:.6f}",
                    }
                )
                if not ok:
                    failures += 1
                print(f"{plan_id}: {checked['moveit_status']}")
            except (ValueError, RuntimeError) as exc:
                failures += 1
                checked.update({"moveit_status": "FAIL", "moveit_error": str(exc)})
                print(f"{plan_id}: FAIL ({exc})")
            checked_rows.append(checked)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    write_csv(args.checked_output, checked_rows)
    if args.checked_output:
        print(f"saved checked plan: {args.checked_output}")
    if failures:
        raise SystemExit(f"MoveIt preflight failed for {failures}/{len(rows)} plan rows")
    print(f"MoveIt preflight OK for {len(rows)} plan rows")


if __name__ == "__main__":
    main()
