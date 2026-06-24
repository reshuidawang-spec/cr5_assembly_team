#!/usr/bin/env python3
"""Interactive Cartesian jog tool with DI1 contact recording.

This is a terminal replacement for a simple pendant-style Cartesian jog panel.
It commands small MovL increments, monitors DI1 during each move, records the
trigger pose on a rising edge, and optionally retracts opposite the last jog.
"""
import argparse
import csv
import math
import shlex
import time
from pathlib import Path

import rclpy
from dobot_msgs_v4.srv import MovL

from probe_touch import PROBE_SPIN_TIMEOUT_SEC, ProbeTouch


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_DIR / "data/jog_probe_contacts.csv"
POST_REACH_TRIGGER_GRACE_SEC = 0.25

AXIS_INDEX = {
    "x": 0,
    "y": 1,
    "z": 2,
    "rx": 3,
    "ry": 4,
    "rz": 5,
}


def angle_delta_deg(actual, expected):
    """Angle delta deg."""
    return (float(actual) - float(expected) + 180.0) % 360.0 - 180.0


def format_pose(pose):
    """Format pose."""
    return "[" + ", ".join(f"{float(value):.4f}" for value in pose) + "]"


def pose_fields(prefix, pose):
    """Pose fields."""
    names = ("x", "y", "z", "rx", "ry", "rz")
    return {
        f"{prefix}_{name}": "" if pose is None else f"{float(pose[index]):.4f}"
        for index, name in enumerate(names)
    }


def vector_fields(prefix, values, count, digits=6):
    """Vector fields."""
    return {
        f"{prefix}_{index + 1}": "" if values is None or index >= len(values) else f"{float(values[index]):.{digits}f}"
        for index in range(count)
    }


def write_row(path, row):
    """Write row."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def fresh_feed_di1(node, max_age_sec=0.2):
    """Fresh feed di1."""
    if node.last_feed_time is None:
        raise RuntimeError("FeedInfo is not available; cannot read DI1")
    age = time.monotonic() - node.last_feed_time
    if age > max_age_sec:
        raise RuntimeError(f"FeedInfo is stale ({age:.3f}s); cannot read DI1")
    return bool(node.di1)


def current_pose(node, max_age_sec=0.2):
    """Return a fresh FeedInfo pose, falling back to GetPose only if needed."""
    if node.last_feed_time is not None and time.monotonic() - node.last_feed_time <= max_age_sec:
        snapshot = node.feed_snapshot()
        pose = snapshot.get("pose")
        if pose is not None:
            return list(pose)
    return node.get_pose()


def make_contact_row(args, state, command, start_pose, target_pose, trigger_snapshot, stop_pose):
    """Make contact row."""
    trigger_pose = trigger_snapshot.get("pose")
    row = {
        "timestamp": f"{time.time():.3f}",
        "sample_index": str(state["sample_index"]),
        "session_id": args.session_id,
        "workpiece_id": args.workpiece_id,
        "artifact_id": args.artifact_id,
        "artifact_type": args.artifact_type,
        "physical_ball_id": state["physical_ball_id"],
        "branch": state["branch"],
        "operator_note": state["operator_note"],
        "command": command,
        "linear_step_mm": f"{state['linear_step_mm']:.4f}",
        "angular_step_deg": f"{state['angular_step_deg']:.4f}",
        "speed": str(state["speed"]),
        "auto_retract_mm": f"{state['auto_retract_mm']:.4f}",
        "trigger_feed_sequence": str(trigger_snapshot.get("sequence", "")),
        "trigger_feed_wall_time": (
            "" if trigger_snapshot.get("wall_time") is None else f"{float(trigger_snapshot['wall_time']):.6f}"
        ),
        "trigger_digital_input_bits": (
            "" if trigger_snapshot.get("digital_input_bits") is None else str(trigger_snapshot["digital_input_bits"])
        ),
        "trigger_di1": "" if trigger_snapshot.get("di1") is None else str(int(bool(trigger_snapshot["di1"]))),
    }
    row.update(pose_fields("start_flange", start_pose))
    row.update(pose_fields("target_flange", target_pose))
    row.update(pose_fields("trigger_flange", trigger_pose))
    row.update(pose_fields("stop_flange", stop_pose))
    row.update(vector_fields("trigger_joint", trigger_snapshot.get("joints"), 6))
    return row


def jog_target(current_pose, axis, sign, state):
    """Jog target."""
    target = list(current_pose)
    index = AXIS_INDEX[axis]
    step = state["angular_step_deg"] if axis.startswith("r") else state["linear_step_mm"]
    target[index] += sign * step
    return target


def vector_jog_target(current_pose, direction, distance_mm):
    """Vector jog target."""
    length = math.sqrt(sum(value * value for value in direction))
    if length <= 1e-9:
        raise ValueError("vector direction cannot be zero")
    if distance_mm <= 0:
        raise ValueError("vector distance must be positive")
    target = list(current_pose)
    for index in range(3):
        target[index] += direction[index] / length * distance_mm
    return target


def pose_reached(pose, target, position_tolerance_mm, orientation_tolerance_deg):
    """Pose reached."""
    position_error = math.sqrt(sum((pose[index] - target[index]) ** 2 for index in range(3)))
    orientation_error = max(abs(angle_delta_deg(pose[index], target[index])) for index in range(3, 6))
    return position_error <= position_tolerance_mm and orientation_error <= orientation_tolerance_deg


def issue_movl(node, target_pose):
    """Issue movl."""
    req = MovL.Request()
    req.mode = False
    req.a, req.b, req.c, req.d, req.e, req.f = target_pose
    future, _ = node.call_async_checked(node.movl_cli, req)
    return future


def wait_jog(node, target_pose, timeout_sec, position_tolerance_mm, orientation_tolerance_deg):
    """Wait jog."""
    deadline = time.monotonic() + timeout_sec
    trigger_snapshot = None
    reached_target = False
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
        if node.last_feed_time is None or time.monotonic() - node.last_feed_time > 0.2:
            node.stop_fast_then_confirm()
            raise RuntimeError("FeedInfo became stale during jog; motion stopped")
        snapshot = node.feed_snapshot()
        if snapshot.get("di1"):
            trigger_snapshot = dict(snapshot)
            node.stop_fast_then_confirm()
            break
        pose = snapshot.get("pose")
        if pose is not None and pose_reached(pose, target_pose, position_tolerance_mm, orientation_tolerance_deg):
            reached_target = True
            grace_deadline = time.monotonic() + POST_REACH_TRIGGER_GRACE_SEC
            while rclpy.ok() and time.monotonic() < grace_deadline:
                rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
                late_snapshot = node.feed_snapshot()
                if late_snapshot.get("di1"):
                    trigger_snapshot = dict(late_snapshot)
                    node.stop_fast_then_confirm()
                    break
            break
    if trigger_snapshot is None and not reached_target:
        node.stop_fast_then_confirm()
        raise RuntimeError("jog command timed out; motion stopped")
    return trigger_snapshot, reached_target


def retract_opposite(node, start_pose, target_pose, retract_mm, timeout_sec):
    """Retract opposite."""
    if retract_mm <= 0:
        return None
    direction = [target_pose[index] - start_pose[index] for index in range(3)]
    length = math.sqrt(sum(value * value for value in direction))
    if length <= 1e-9:
        return None
    pose = current_pose(node, max_age_sec=0.5)
    retract_pose = list(pose)
    for index in range(3):
        retract_pose[index] -= direction[index] / length * retract_mm
    node.move_l(retract_pose, timeout_sec=timeout_sec)
    node.wait_until_pose(retract_pose, timeout_sec=timeout_sec)
    return retract_pose


def print_status(node, state):
    """Print status."""
    pose = current_pose(node, max_age_sec=0.5)
    di1 = fresh_feed_di1(node, max_age_sec=0.5)
    print(f"pose: {format_pose(pose)}")
    print(f"DI1: {int(di1)}")
    print(
        "state: "
        f"linear_step={state['linear_step_mm']:.4f}mm "
        f"angular_step={state['angular_step_deg']:.4f}deg "
        f"speed={state['speed']} "
        f"ball={state['physical_ball_id']} "
        f"branch={state['branch']} "
        f"auto_retract={state['auto_retract_mm']:.4f}mm"
    )


def print_help():
    """Print help."""
    print(
        """
commands:
  x+ x- y+ y- z+ z-       jog linear axes by current step
  vec <dx> <dy> <dz> [mm]  jog along a normalized base-frame vector
  rx+ rx- ry+ ry- rz+ rz- jog orientation axes by current angular step
  step <mm>               set linear jog step, e.g. step 0.5
  rstep <deg>             set angular jog step, e.g. rstep 1
  speed <1..5>            set SpeedFactor used before jogs
  retract <mm>            set automatic opposite linear retract after trigger
  ball <id>               set physical ruby ball id
  branch <name>           set branch label written to CSV
  note <text>             set operator note written to CSV
  pose                    print current pose and DI1
  di                      print DI1
  stop                    send Stop()
  help                    show this help
  quit                    exit
""".strip()
    )


def parse_jog_command(command):
    """Parse jog command."""
    cmd = command.lower()
    for axis in ("rx", "ry", "rz", "x", "y", "z"):
        if cmd == axis + "+":
            return axis, 1.0
        if cmd == axis + "-":
            return axis, -1.0
    return None, None


def execute_jog(node, args, state, command, target_from_start):
    """Execute jog."""
    node.wait_fresh_feed()
    if fresh_feed_di1(node):
        print("DI1 is already ON; release probe before jogging")
        return

    start_pose = current_pose(node)
    target_pose = target_from_start(start_pose)
    print(f"target: {format_pose(target_pose)}")
    node.set_speed(state["speed"])
    future = issue_movl(node, target_pose)
    trigger_snapshot, reached_target = wait_jog(
        node,
        target_pose,
        args.timeout_sec,
        args.position_tolerance_mm,
        args.orientation_tolerance_deg,
    )
    if future.done():
        node.check_ready_future(node.movl_cli, future)
    if trigger_snapshot is None:
        print("reached target; no DI1 trigger")
        return

    stop_pose = current_pose(node, max_age_sec=0.5)
    state["sample_index"] += 1
    row = make_contact_row(args, state, command, start_pose, target_pose, trigger_snapshot, stop_pose)
    write_row(args.output, row)
    print(f"DI1 triggered; recorded sample {state['sample_index']}")
    print(f"trigger pose: {format_pose(trigger_snapshot.get('pose') or stop_pose)}")
    print(f"stop pose:    {format_pose(stop_pose)}")
    retract_pose = retract_opposite(node, start_pose, target_pose, state["auto_retract_mm"], args.timeout_sec)
    if retract_pose is not None:
        print(f"retracted:    {format_pose(retract_pose)}")
    print(f"DI1 after retract: {int(fresh_feed_di1(node, max_age_sec=0.5))}")


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--session-id", default="")
    parser.add_argument("--workpiece-id", default="calibration_sphere_20mm")
    parser.add_argument("--artifact-id", default="standard_sphere_20mm")
    parser.add_argument("--artifact-type", default="sphere")
    parser.add_argument("--physical-ball-id", default="1")
    parser.add_argument("--branch", default="y_neg")
    parser.add_argument("--operator-note", default="")
    parser.add_argument("--linear-step-mm", type=float, default=0.5)
    parser.add_argument("--angular-step-deg", type=float, default=1.0)
    parser.add_argument("--auto-retract-mm", type=float, default=1.0)
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.08)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.08)
    args = parser.parse_args()

    if args.linear_step_mm <= 0 or args.angular_step_deg <= 0:
        raise SystemExit("step sizes must be positive")
    if not 1 <= args.speed <= 5:
        raise SystemExit("--speed must be between 1 and 5")
    if args.auto_retract_mm < 0:
        raise SystemExit("--auto-retract-mm cannot be negative")

    state = {
        "linear_step_mm": float(args.linear_step_mm),
        "angular_step_deg": float(args.angular_step_deg),
        "auto_retract_mm": float(args.auto_retract_mm),
        "speed": int(args.speed),
        "physical_ball_id": args.physical_ball_id,
        "branch": args.branch,
        "operator_note": args.operator_note,
        "sample_index": 0,
    }

    rclpy.init()
    node = ProbeTouch()
    try:
        node.wait_services(10.0)
        node.wait_fresh_feed()
        print("interactive jog recorder ready")
        print(f"output: {args.output}")
        print_help()
        print_status(node, state)

        while True:
            try:
                line = input("jog> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print()
                break
            if not line:
                continue
            parts = shlex.split(line)
            command = parts[0].lower()

            if command in ("quit", "exit", "q"):
                break
            if command == "help":
                print_help()
                continue
            if command == "pose":
                print_status(node, state)
                continue
            if command == "di":
                print(f"DI1: {int(fresh_feed_di1(node, max_age_sec=0.5))}")
                continue
            if command == "stop":
                node.stop_fast_then_confirm()
                print("Stop sent")
                continue
            if command == "step" and len(parts) == 2:
                value = float(parts[1])
                if value <= 0:
                    print("step must be positive")
                else:
                    state["linear_step_mm"] = value
                continue
            if command == "rstep" and len(parts) == 2:
                value = float(parts[1])
                if value <= 0:
                    print("rstep must be positive")
                else:
                    state["angular_step_deg"] = value
                continue
            if command == "speed" and len(parts) == 2:
                value = int(parts[1])
                if not 1 <= value <= 5:
                    print("speed must be 1..5")
                else:
                    state["speed"] = value
                    node.set_speed(value)
                continue
            if command == "retract" and len(parts) == 2:
                value = float(parts[1])
                if value < 0:
                    print("retract cannot be negative")
                else:
                    state["auto_retract_mm"] = value
                continue
            if command == "ball" and len(parts) == 2:
                state["physical_ball_id"] = parts[1]
                continue
            if command == "branch" and len(parts) == 2:
                state["branch"] = parts[1]
                continue
            if command == "note":
                state["operator_note"] = line.partition(" ")[2]
                continue
            if command in ("vec", "vector", "line") and len(parts) in (4, 5):
                direction = [float(parts[index]) for index in range(1, 4)]
                distance_mm = float(parts[4]) if len(parts) == 5 else state["linear_step_mm"]
                label = f"vec {direction[0]:g} {direction[1]:g} {direction[2]:g} {distance_mm:g}"
                execute_jog(
                    node,
                    args,
                    state,
                    label,
                    lambda start_pose, direction=direction, distance_mm=distance_mm: vector_jog_target(
                        start_pose, direction, distance_mm
                    ),
                )
                continue

            axis, sign = parse_jog_command(command)
            if axis is None:
                print("unknown command; type help")
                continue
            execute_jog(
                node,
                args,
                state,
                command,
                lambda start_pose, axis=axis, sign=sign: jog_target(start_pose, axis, sign, state),
            )
    except (RuntimeError, ValueError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
