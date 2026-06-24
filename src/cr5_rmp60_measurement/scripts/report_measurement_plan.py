#!/usr/bin/env python3
"""Batch MoveIt preflight report for a measurement plan JSON file.

Produces:
  data/measurement_plan_report.json  — full details per pose
  data/measurement_plan_report.csv   — flat summary for spreadsheet review
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy

from generate_measurement_poses import orientation_from_approach
from moveit_utils import load_pose_specs, make_robot_state
from preflight_measurement_plan import MeasurementPreflight, status
from review_flange_pose_conversion import review_spec


def flattened(results):
    """Build a flat dict suitable for CSV writing from the nested report structure."""
    flat = {}
    for spec in results["specs"]:
        row = {"name": spec["name"]}
        for prefix, checks in [
            ("ik_safe", spec["ik"]["safe"]),
            ("ik_contact", spec["ik"]["contact"]),
            ("ik_target", spec["ik"]["target"]),
        ]:
            row[f"{prefix}"] = status(checks["ok"])
            row[f"{prefix}_moveit_error_code"] = checks["moveit_error_code"]
        row["plan_to_safe"] = status(spec["plan_current_to_safe"]["ok"])
        row["plan_to_safe_moveit_error_code"] = spec["plan_current_to_safe"]["moveit_error_code"]
        row["plan_to_safe_points"] = spec["plan_current_to_safe"]["points"]
        row["plan_to_safe_time"] = f"{spec['plan_current_to_safe']['planning_time']:.3f}"
        row["cartesian_probe"] = status(spec["cartesian_probe"]["ok"])
        row["cartesian_moveit_error_code"] = spec["cartesian_probe"]["moveit_error_code"]
        row["cartesian_fraction"] = f"{spec['cartesian_probe']['fraction']:.3f}"
        row["cartesian_points"] = spec["cartesian_probe"]["points"]
        for pose_label in ("safe", "contact", "target"):
            converted = spec["flange_conversion"]["poses"][pose_label]
            movl = converted["candidate_movl_pose"]
            for axis_idx, axis_name in enumerate("xyz"):
                row[f"flange_{pose_label}_{axis_name}"] = f"{movl[axis_idx]:.4f}"
            row[f"flange_{pose_label}_rx"] = f"{movl[3]:.4f}"
            row[f"flange_{pose_label}_ry"] = f"{movl[4]:.4f}"
            row[f"flange_{pose_label}_rz"] = f"{movl[5]:.4f}"
            row[f"flange_{pose_label}_tool_axis_vs_approach_deg"] = (
                f"{converted['tool_axis_vs_approach_angle_deg']:.6f}"
            )
        row["status"] = status(spec["ok"])
        flat[spec["name"]] = row
    return flat


def write_csv(path, results):
    """Write a list of dicts to a CSV file with given fieldnames."""
    flat = flattened(results)
    if not flat:
        return
    fieldnames = list(next(iter(flat.values())).keys())
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in flat.values():
            writer.writerow(row)


def validate_report_args(args):
    """Validate report args."""
    if args.ik_timeout <= 0:
        raise SystemExit("--ik-timeout must be positive")
    if args.attempts <= 0:
        raise SystemExit("--attempts must be positive")
    if args.planning_time <= 0:
        raise SystemExit("--planning-time must be positive")
    if args.max_step_m <= 0:
        raise SystemExit("--max-step-m must be positive")
    if not 0.0 < args.min_fraction <= 1.0:
        raise SystemExit("--min-fraction must be in (0, 1]")
    if args.position_tolerance_m <= 0:
        raise SystemExit("--position-tolerance-m must be positive")
    if args.orientation_tolerance_rad <= 0:
        raise SystemExit("--orientation-tolerance-rad must be positive")


def run_report(node, pose_specs, args):
    """Run report."""
    overall_ok = True
    spec_results = []

    tf_ok, tf_detail = True, "skipped"
    if not args.skip_tool_tf_check:
        tf_ok, tf_detail = node.check_tool_tf(args)
        print(f"  [{status(tf_ok)}] tool TF: {tf_detail}")
        overall_ok = overall_ok and tf_ok

    for spec in pose_specs:
        name = spec.get("name", "probe_pose")
        orientation = spec.get("tip_orientation") or orientation_from_approach(
            spec["approach_vector"], reference_up=args.reference_up
        )
        spec_ok = True

        ik_results = {}
        start_state = make_robot_state(node.joint_state)
        safe_state = None
        for label, key in (("safe", "safe_position"), ("contact", "contact"), ("target", "target_position")):
            ik_ok, code, solution, _ = node.solve_ik(label, spec[key], orientation, start_state, args)
            ik_results[label] = {"ok": ik_ok, "moveit_error_code": code}
            if not ik_ok:
                spec_ok = False
            if key == "safe_position":
                safe_state = solution if ik_ok else None

        plan_ok, plan_result = node.plan_to_safe(start_state, spec["safe_position"], orientation, args)
        plan_info = {
            "ok": plan_ok,
            "moveit_error_code": plan_result.error_code.val,
            "points": len(plan_result.trajectory.joint_trajectory.points),
            "planning_time": plan_result.planning_time,
        }
        if not plan_ok:
            spec_ok = False

        cart_info = {
            "ok": False,
            "moveit_error_code": -1,
            "fraction": 0.0,
            "points": 0,
        }
        if safe_state is not None:
            cart_ok, cart_result = node.compute_probe_path(
                safe_state,
                spec["contact"],
                spec["target_position"],
                orientation,
                args,
            )
            cart_info = {
                "ok": cart_ok,
                "moveit_error_code": cart_result.error_code.val,
                "fraction": cart_result.fraction,
                "points": len(cart_result.solution.joint_trajectory.points),
            }
            if not cart_ok:
                spec_ok = False
        else:
            spec_ok = False

        flange_review = review_spec(spec, args)

        prefix = f"[{status(spec_ok)}]"
        print(
            f"  {prefix} {name}: "
            f"IK safe={status(ik_results['safe']['ok'])} "
            f"contact={status(ik_results['contact']['ok'])} "
            f"target={status(ik_results['target']['ok'])} | "
            f"plan={status(plan_info['ok'])} "
            f"cartesian={status(cart_info['ok'])} "
            f"(fraction={cart_info['fraction']:.3f})"
        )

        spec_results.append(
            {
                "name": name,
                "ik": ik_results,
                "plan_current_to_safe": plan_info,
                "cartesian_probe": cart_info,
                "flange_conversion": flange_review,
                "ok": spec_ok,
            }
        )
        overall_ok = overall_ok and spec_ok

    return {
        "timestamp": time.time(),
        "plan_source": args.input,
        "overall_status": status(overall_ok),
        "summary": {
            "total": len(spec_results),
            "pass": sum(1 for s in spec_results if s["ok"]),
            "fail": sum(1 for s in spec_results if not s["ok"]),
        },
        "tool_tf_check": {"ok": tf_ok, "detail": tf_detail},
        "args": {
            "frame_id": args.frame_id,
            "unit": args.unit,
            "group": args.group,
            "ik_link": args.ik_link,
            "pipeline": args.pipeline,
            "planner": args.planner,
            "attempts": args.attempts,
            "planning_time": args.planning_time,
            "velocity_scale": args.velocity_scale,
            "acceleration_scale": args.acceleration_scale,
            "position_tolerance_m": args.position_tolerance_m,
            "orientation_tolerance_rad": args.orientation_tolerance_rad,
            "ik_timeout": args.ik_timeout,
            "max_step_m": args.max_step_m,
            "min_fraction": args.min_fraction,
            "avoid_collisions": args.avoid_collisions,
        },
        "specs": spec_results,
    }


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="JSON measurement plan file")
    parser.add_argument("--reference-up", nargs=3, type=float, default=[0.0, 0.0, 1.0], metavar=("UX", "UY", "UZ"))
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
    parser.add_argument("--json-output", default="data/measurement_plan_report.json")
    parser.add_argument("--csv-output", default="data/measurement_plan_report.csv")
    parser.set_defaults(avoid_collisions=True)
    args = parser.parse_args()

    validate_report_args(args)
    args.scale_factor = 0.001 if args.unit == "mm" else 1.0
    args.standoff_mm = None
    args.travel_mm = None
    args.contact = None
    args.approach = None
    args.name = None
    args.reference_up = list(args.reference_up)
    args.no_display = True

    try:
        pose_specs = load_pose_specs(args)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if not pose_specs:
        raise SystemExit(f"no poses in: {args.input}")

    print(f"loaded: {args.input}")
    print(f"poses: {len(pose_specs)}")

    rclpy.init()
    node = MeasurementPreflight()
    try:
        node.wait_ready(timeout_sec=5.0)
        results = run_report(node, pose_specs, args)

        json_path = Path(args.json_output)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(results, indent=2, default=str) + "\n")
        print(f"saved: {json_path}")

        csv_path = Path(args.csv_output)
        write_csv(csv_path, results)
        print(f"saved: {csv_path}")

        summary = results["summary"]
        print(
            f"report: {summary['total']} poses, "
            f"{summary['pass']} PASS, {summary['fail']} FAIL, "
            f"overall: {results['overall_status']}"
        )
        if summary["fail"]:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
