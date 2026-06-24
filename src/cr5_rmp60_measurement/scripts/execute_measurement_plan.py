#!/usr/bin/env python3
"""Dry-run scaffold for future arbitrary-direction probing execution."""
import argparse
import json
import time
from pathlib import Path

import rclpy

from generate_measurement_poses import orientation_from_approach
from moveit_utils import load_pose_specs, make_robot_state
from preflight_measurement_plan import (
    MeasurementPreflight,
    status,
)
from review_flange_pose_conversion import review_spec


STATE_MACHINE = [
    "PRECHECK",
    "MOVE_TO_SAFE",
    "ARM_PROBE_MONITOR",
    "PROBE_ALONG_APPROACH",
    "DI_TRIGGER_STOP",
    "RETRACT",
    "DONE",
]


def validate_execution_args(args):
    """Validate execution args."""
    if args.execute:
        raise SystemExit(
            "--execute is intentionally disabled in this scaffold; "
            "this script currently supports dry-run planning only"
        )
    if args.speed < 1 or args.speed > 5:
        raise SystemExit("--speed must be between 1 and 5 for probing")
    if args.travel_mm > 5.0 and not args.allow_long_travel:
        raise SystemExit("--travel-mm > 5 requires --allow-long-travel")
    if args.retract_mm <= 0:
        raise SystemExit("--retract-mm must be positive")


def print_state_machine():
    """Print state machine."""
    print("execution state machine:")
    for index, state in enumerate(STATE_MACHINE, start=1):
        print(f"  {index}. {state}")


def print_future_actions(spec, orientation, flange_review, args):
    """Print future actions."""
    print("dry-run future robot actions:")
    print(f"  set SpeedFactor: {args.speed}")
    print(f"  move to safe_position: {spec['safe_position']}")
    safe_movl = flange_review["poses"]["safe"]["candidate_movl_pose"]
    print(
        "  candidate safe MovL flange pose: "
        f"[{safe_movl[0]:.4f}, {safe_movl[1]:.4f}, {safe_movl[2]:.4f}, "
        f"{safe_movl[3]:.4f}, {safe_movl[4]:.4f}, {safe_movl[5]:.4f}]"
    )
    print("  start DI1 monitor: required before probing motion")
    print(f"  probe Cartesian segment: safe_position -> contact -> target_position")
    print(f"  stop condition: DI1 trigger OR target_position reached OR timeout {args.timeout:.1f}s")
    print(f"  retract distance: {args.retract_mm:.3f} mm opposite approach direction")
    print(
        "  target orientation xyzw: "
        f"[{orientation['x']:.6f}, {orientation['y']:.6f}, {orientation['z']:.6f}, {orientation['w']:.6f}]"
    )


def write_plan(path, plan):
    """Write plan."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(plan, indent=2) + "\n")


def run_dry_plan(node, pose_specs, args):
    """Run dry plan."""
    all_ok = True
    plan_specs = []

    tf_ok = None
    tf_detail = "skipped"
    if not args.skip_tool_tf_check:
        tf_ok, tf_detail = node.check_tool_tf(args)
        print(f"  [{status(tf_ok)}] tool TF direction: {tf_detail}")
        all_ok = all_ok and tf_ok

    for spec in pose_specs:
        name = spec.get("name", "probe_pose")
        print(f"{name}:")
        orientation = spec.get("tip_orientation") or orientation_from_approach(
            spec["approach_vector"], reference_up=args.reference_up
        )
        print(
            "  tip_orientation_xyzw: "
            f"[{orientation['x']:.6f}, {orientation['y']:.6f}, {orientation['z']:.6f}, {orientation['w']:.6f}]"
        )

        checks = []
        start_state = make_robot_state(node.joint_state)
        safe_state = None
        for label, key in (("safe IK", "safe_position"), ("contact IK", "contact"), ("target IK", "target_position")):
            ik_ok, code, solution, _ = node.solve_ik(label, spec[key], orientation, start_state, args)
            print(f"  [{status(ik_ok)}] {label}: moveit_error_code={code}, position={spec[key]}")
            checks.append({"name": label, "ok": ik_ok, "moveit_error_code": code})
            all_ok = all_ok and ik_ok
            if key == "safe_position":
                safe_state = solution

        safe_plan_ok, safe_plan = node.plan_to_safe(start_state, spec["safe_position"], orientation, args)
        safe_plan_points = len(safe_plan.trajectory.joint_trajectory.points)
        print(
            f"  [{status(safe_plan_ok)}] current -> safe plan: "
            f"moveit_error_code={safe_plan.error_code.val}, points={safe_plan_points}, "
            f"planning_time={safe_plan.planning_time:.3f}s"
        )
        checks.append(
            {
                "name": "current -> safe plan",
                "ok": safe_plan_ok,
                "moveit_error_code": safe_plan.error_code.val,
                "points": safe_plan_points,
                "planning_time": safe_plan.planning_time,
            }
        )
        all_ok = all_ok and safe_plan_ok

        cart_ok = False
        cart_result = None
        if safe_state is not None:
            cart_ok, cart_result = node.compute_probe_path(
                safe_state,
                spec["contact"],
                spec["target_position"],
                orientation,
                args,
            )
            cart_points = len(cart_result.solution.joint_trajectory.points)
            print(
                f"  [{status(cart_ok)}] safe -> contact -> target Cartesian: "
                f"moveit_error_code={cart_result.error_code.val}, fraction={cart_result.fraction:.3f}, "
                f"points={cart_points}"
            )
            checks.append(
                {
                    "name": "safe -> contact -> target Cartesian",
                    "ok": cart_ok,
                    "moveit_error_code": cart_result.error_code.val,
                    "fraction": cart_result.fraction,
                    "points": cart_points,
                }
            )
            all_ok = all_ok and cart_ok

        flange_review = review_spec(spec, args)
        print("  candidate flange MovL poses use ROS RPY convention; verify with CR5 GetPose before real execution")
        for pose_label, converted in flange_review["poses"].items():
            movl_pose = converted["candidate_movl_pose"]
            print(
                f"    {pose_label}: "
                f"[{movl_pose[0]:.4f}, {movl_pose[1]:.4f}, {movl_pose[2]:.4f}, "
                f"{movl_pose[3]:.4f}, {movl_pose[4]:.4f}, {movl_pose[5]:.4f}]"
            )

        print_future_actions(spec, orientation, flange_review, args)
        plan_specs.append(
            {
                "name": name,
                "contact": spec["contact"],
                "safe_position": spec["safe_position"],
                "target_position": spec["target_position"],
                "approach_vector": spec["approach_vector"],
                "tip_orientation": orientation,
                "flange_pose_conversion": flange_review,
                "checks": checks,
            }
        )

        if not args.no_display and safe_plan_ok and cart_ok and cart_result is not None:
            node.publish_display(start_state, [safe_plan.trajectory, cart_result.solution])
            print("  published: /display_planned_path")

    plan = {
        "timestamp": time.time(),
        "dry_run": True,
        "execute_requested": False,
        "overall_status": status(all_ok),
        "state_machine": STATE_MACHINE,
        "tool_tf_check": {"ok": tf_ok, "detail": tf_detail},
        "safety_policy": {
            "real_execution_enabled": False,
            "di_port": "DI1",
            "speed": args.speed,
            "timeout_sec": args.timeout,
            "retract_mm": args.retract_mm,
            "requires_preflight_pass": True,
            "requires_fresh_feedinfo": True,
            "requires_di1_clear_before_motion": True,
        },
        "poses": plan_specs,
    }
    return all_ok, plan


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="reserved for future real execution; disabled now")
    parser.add_argument("--input", help="JSON file from generate_measurement_poses.py --json")
    parser.add_argument("--contact", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--approach", nargs=3, type=float, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--reference-up", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("UX", "UY", "UZ"))
    parser.add_argument("--standoff-mm", type=float, default=20.0)
    parser.add_argument("--travel-mm", type=float, default=5.0)
    parser.add_argument("--allow-long-travel", action="store_true", help="allow travel distance > 5 mm")
    parser.add_argument("--retract-mm", type=float, default=5.0)
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--name", default="probe_pose")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--unit", choices=("mm", "m"), default="mm")
    parser.add_argument("--group", default="cr5_group")
    parser.add_argument("--ik-link", default="rmp60_tip")
    parser.add_argument("--pipeline", default="ompl")
    parser.add_argument("--planner", default="")
    parser.add_argument("--attempts", type=int, default=5)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scale", type=float, default=0.1)
    parser.add_argument("--acceleration-scale", type=float, default=0.1)
    parser.add_argument("--position-tolerance-m", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=0.05)
    parser.add_argument("--ik-timeout", type=float, default=0.2)
    parser.add_argument("--max-step-m", type=float, default=0.001)
    parser.add_argument("--jump-threshold", type=float, default=0.0)
    parser.add_argument("--min-fraction", type=float, default=1.0)
    parser.add_argument("--no-avoid-collisions", dest="avoid_collisions", action="store_false")
    parser.add_argument("--skip-tool-tf-check", action="store_true")
    parser.add_argument("--tool-parent-frame", default="Link6")
    parser.add_argument("--adapter-length", type=float, default=0.0494)
    parser.add_argument("--probe-body-length", type=float, default=0.076)
    parser.add_argument("--stylus-length", type=float, default=0.075)
    parser.add_argument("--tool-length-tolerance-m", type=float, default=0.003)
    parser.add_argument("--tool-lateral-tolerance-m", type=float, default=0.003)
    parser.add_argument("--tf-timeout", type=float, default=3.0)
    parser.add_argument("--plan-output", default="data/measurement_plan_dry_run.json")
    parser.add_argument("--no-display", action="store_true")
    parser.set_defaults(avoid_collisions=True)
    args = parser.parse_args()

    validate_execution_args(args)
    if args.standoff_mm <= 0:
        raise SystemExit("--standoff-mm must be positive")
    if args.travel_mm < 0:
        raise SystemExit("--travel-mm cannot be negative")
    if args.attempts <= 0:
        raise SystemExit("--attempts must be positive")
    if args.planning_time <= 0:
        raise SystemExit("--planning-time must be positive")
    if args.ik_timeout <= 0:
        raise SystemExit("--ik-timeout must be positive")
    if args.max_step_m <= 0:
        raise SystemExit("--max-step-m must be positive")
    if not 0.0 < args.min_fraction <= 1.0:
        raise SystemExit("--min-fraction must be in (0, 1]")

    args.scale_factor = 0.001 if args.unit == "mm" else 1.0
    try:
        pose_specs = load_pose_specs(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    print("dry-run only: no Dobot motion service will be called")
    print_state_machine()

    rclpy.init()
    node = MeasurementPreflight()
    try:
        node.wait_ready(timeout_sec=5.0)
        ok, plan = run_dry_plan(node, pose_specs, args)
        write_plan(Path(args.plan_output), plan)
        print(f"saved dry-run plan: {args.plan_output}")
        print(f"dry-run result: {status(ok)}")
        if not ok:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
