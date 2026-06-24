#!/usr/bin/env python3
"""Controlled end-to-end branch calibration driver.

This script connects the existing pieces into one guarded workflow:

1. Generate a local probe plan with auto_calibrate_five_branch_sphere.py.
2. Start MoveIt, load the calibration workcell scene, and preflight the plan.
3. Optionally execute the same checked plan on the real robot.

The real robot motion still requires an explicit --execute plus
--ack-controlled-auto-calibration.
"""
import argparse
import os
import signal
import shlex
import subprocess
import sys
import time
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
ROS_PREFIX = "source /opt/ros/humble/setup.bash; source ~/dobot_ws/install/setup.bash; export ROS_LOG_DIR=/tmp"
DEFAULT_REFERENCE_FIT = PROJECT_DIR / "data/2026.6.16/yneg_near_fourth_refit_20260616.json"
DEFAULT_MOVEIT_LAUNCH = PROJECT_DIR / "launch/cr5_rmp60_moveit_demo.launch.py"


def shlex_join(parts):
    """Shlex join."""
    return shlex.join([str(part) for part in parts])


def ros_command(command):
    """Ros command."""
    return f"{ROS_PREFIX}; {command}"


def run_ros(command, *, cwd=PROJECT_DIR, check=True):
    """Run ros."""
    printable = command if isinstance(command, str) else shlex_join(command)
    print(f"\n$ {printable}", flush=True)
    if isinstance(command, str):
        completed = subprocess.run(
            ["bash", "-lc", ros_command(command)],
            cwd=cwd,
            text=True,
            check=check,
        )
    else:
        completed = subprocess.run(
            ["bash", "-lc", ros_command(shlex_join(command))],
            cwd=cwd,
            text=True,
            check=check,
        )
    return completed


def service_available(service_name):
    """Service available."""
    completed = subprocess.run(
        ["bash", "-lc", ros_command("ros2 service list")],
        cwd=PROJECT_DIR,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return completed.returncode == 0 and service_name in completed.stdout.splitlines()


def wait_for_service(service_name, timeout_sec):
    """Wait for service."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if service_available(service_name):
            return True
        time.sleep(0.5)
    return False


def start_moveit(args):
    """Start moveit."""
    command = ros_command(f"ros2 launch {args.moveit_launch} use_rviz:=false")
    print(f"\n$ ros2 launch {args.moveit_launch} use_rviz:=false", flush=True)
    proc = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=PROJECT_DIR,
        stdout=subprocess.PIPE if args.quiet_moveit else None,
        stderr=subprocess.STDOUT if args.quiet_moveit else None,
        text=True,
        preexec_fn=os.setsid,
    )
    if not wait_for_service("/compute_ik", args.moveit_start_timeout_sec):
        stop_process_group(proc)
        raise RuntimeError("MoveIt /compute_ik did not become available")
    if not wait_for_service("/apply_planning_scene", args.moveit_start_timeout_sec):
        stop_process_group(proc)
        raise RuntimeError("MoveIt /apply_planning_scene did not become available")
    return proc


def stop_process_group(proc):
    """Stop process group."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=6.0)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass


def build_auto_args(args, execute=False):
    """Build auto args."""
    command = [
        "./scripts/auto_calibrate_five_branch_sphere.py",
        "--reference-fit-json",
        args.reference_fit_json,
        "--branches",
        args.branch,
        "--samples-per-branch",
        str(args.samples),
        "--physical-ball-id",
        args.physical_ball_id,
        "--session-id",
        args.session_id,
        "--operator-note",
        args.operator_note,
        "--plan-output",
        args.plan_output,
        "--output",
        args.output,
        "--calibration-output",
        args.calibration_output,
        "--residual-output",
        args.residual_output,
        "--standoff-mm",
        str(args.standoff_mm),
        "--transition-standoff-mm",
        str(args.transition_standoff_mm),
        "--overtravel-mm",
        str(args.overtravel_mm),
        "--max-probe-travel-mm",
        str(args.max_probe_travel_mm),
        "--speed",
        str(args.speed),
        "--timeout-sec",
        str(args.timeout_sec),
        "--positioning-timeout-sec",
        str(args.positioning_timeout_sec),
        "--service-timeout-sec",
        str(args.service_timeout_sec),
        "--position-tolerance-mm",
        str(args.position_tolerance_mm),
        "--orientation-tolerance-deg",
        str(args.orientation_tolerance_deg),
        "--table-plane-z-mm",
        str(args.table_plane_z_mm),
    ]
    if args.branch_seed_fit:
        command.extend(["--branch-seed-fit", f"{args.branch}={args.branch_seed_fit}"])
    if args.fit_input:
        command.append("--fit-input")
        command.extend(args.fit_input)
    if args.orientation:
        command.append("--orientation")
        command.extend(str(value) for value in args.orientation)
    if args.safe_pose:
        command.append("--safe-pose")
        command.extend(str(value) for value in args.safe_pose)
    if args.refit_after_each_hit:
        command.append("--refit-after-each-hit")
    if args.safe_transition_move:
        command.extend(["--safe-transition-move", args.safe_transition_move])
    if args.allow_large_safe_transition_orientation_change:
        command.append("--allow-large-safe-transition-orientation-change")
    if args.allow_probe_model_collision_risk:
        command.append("--allow-probe-model-collision-risk")
    if execute:
        command.extend(["--execute", "--ack-five-branch-calibration-path"])
    return command


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-fit-json", default=str(DEFAULT_REFERENCE_FIT))
    parser.add_argument("--branch", default="x_neg")
    parser.add_argument("--branch-seed-fit", help="coarse branch seed JSON for the selected branch")
    parser.add_argument("--samples", type=int, default=6)
    parser.add_argument("--physical-ball-id", default="needle3")
    parser.add_argument("--session-id", default="session_controlled_auto_branch_calibration")
    parser.add_argument("--operator-note", default="controlled auto branch calibration")
    parser.add_argument("--fit-input", nargs="*", default=[])
    parser.add_argument("--orientation", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--safe-pose", nargs=6, type=float, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--plan-output", default="data/controlled_auto_branch_plan.csv")
    parser.add_argument("--checked-output", default="data/controlled_auto_branch_plan_moveit_checked.csv")
    parser.add_argument("--output", default="data/controlled_auto_branch_contacts.csv")
    parser.add_argument("--calibration-output", default="data/controlled_auto_branch_fit.json")
    parser.add_argument("--residual-output", default="data/controlled_auto_branch_residuals.csv")
    parser.add_argument("--standoff-mm", type=float, default=0.5)
    parser.add_argument("--transition-standoff-mm", type=float, default=6.0)
    parser.add_argument("--overtravel-mm", type=float, default=0.3)
    parser.add_argument("--max-probe-travel-mm", type=float, default=1.0)
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--positioning-timeout-sec", type=float, default=15.0)
    parser.add_argument("--service-timeout-sec", type=float, default=10.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.08)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.08)
    parser.add_argument("--table-plane-z-mm", type=float, default=0.0)
    parser.add_argument("--moveit-launch", default=str(DEFAULT_MOVEIT_LAUNCH))
    parser.add_argument("--moveit-start-timeout-sec", type=float, default=20.0)
    parser.add_argument("--moveit-ik-timeout-sec", type=float, default=3.0)
    parser.add_argument("--skip-moveit-preflight", action="store_true")
    parser.add_argument("--quiet-moveit", action="store_true")
    parser.add_argument("--refit-after-each-hit", action="store_true")
    parser.add_argument("--safe-transition-move", choices=("movl", "movj"), default="movj")
    parser.add_argument("--allow-large-safe-transition-orientation-change", action="store_true")
    parser.add_argument("--allow-probe-model-collision-risk", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--ack-controlled-auto-calibration", action="store_true")
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.execute and not args.ack_controlled_auto_calibration:
        raise SystemExit("real robot motion requires --ack-controlled-auto-calibration")
    if args.execute and not args.safe_pose:
        raise SystemExit("real robot motion requires --safe-pose")

    run_ros(build_auto_args(args, execute=False))

    moveit_proc = None
    try:
        if not args.skip_moveit_preflight:
            moveit_proc = start_moveit(args)
            run_ros(
                [
                    "./scripts/apply_calibration_moveit_scene.py",
                    "--reference-fit-json",
                    args.reference_fit_json,
                    "--timeout-sec",
                    "10",
                ]
            )
            run_ros(
                [
                    "./scripts/check_five_branch_plan_moveit.py",
                    "--plan",
                    args.plan_output,
                    "--checked-output",
                    args.checked_output,
                    "--service-timeout-sec",
                    "10",
                    "--ik-timeout",
                    str(args.moveit_ik_timeout_sec),
                ]
            )
    finally:
        stop_process_group(moveit_proc)

    if not args.execute:
        print("\nplan and MoveIt preflight complete; add --execute --ack-controlled-auto-calibration for real motion")
        return 0

    for service_name in (
        "/dobot_bringup_ros2/srv/GetPose",
        "/dobot_bringup_ros2/srv/MovL",
        "/dobot_bringup_ros2/srv/DI",
        "/dobot_bringup_ros2/srv/Stop",
    ):
        if not wait_for_service(service_name, args.service_timeout_sec):
            raise SystemExit(f"Dobot service is not available: {service_name}")

    run_ros(build_auto_args(args, execute=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
