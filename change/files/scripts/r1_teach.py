#!/usr/bin/env python3
"""Manual R1 teaching helper for CoppeliaSim.

This script lets a human jog R1 joints in a stopped CoppeliaSim scene and record
the current six-joint pose as a named waypoint. It deliberately does not save or
modify the scene file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sim_bridge.coppelia_client import SimBridge
from robot_control.r1_motion import R1SafetyGuard


DEFAULT_OUTPUT = REPO_ROOT / "data" / "manual_waypoints" / "r1_teach_waypoints.json"
WAYPOINT_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
DRIVE_POSITIVE_KEYS = {"1": 0, "2": 1, "3": 2, "4": 3, "5": 4, "6": 5}
DRIVE_NEGATIVE_KEYS = {"q": 0, "w": 1, "e": 2, "r": 3, "t": 4, "y": 5}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _round(values: list[float], digits: int = 9) -> list[float]:
    return [round(float(value), digits) for value in values]


def _joints_deg(joints_rad: list[float]) -> list[float]:
    return _round([math.degrees(value) for value in joints_rad], 6)


def _print_pose(joints_rad: list[float]) -> None:
    print("R1 current joints:")
    print("  deg:", json.dumps(_joints_deg(joints_rad)))
    print("  rad:", json.dumps(_round(joints_rad)))


def _check_guard(
    guard: R1SafetyGuard,
    label: str = "manual teach pose",
    check_workspace: bool = True,
) -> tuple[bool, str]:
    try:
        guard.check(label, check_workspace=check_workspace)
    except Exception as exc:
        return False, str(exc)
    return True, "collision-free"


def _check_pose(
    bridge: SimBridge,
    label: str = "manual teach pose",
    check_workspace: bool = True,
) -> tuple[bool, str]:
    robot = bridge.get_object_handle("R1")
    guard = R1SafetyGuard(bridge.sim, robot)
    try:
        return _check_guard(guard, label, check_workspace=check_workspace)
    finally:
        guard.close()


def _print_check_result(ok: bool, message: str) -> bool:
    if ok:
        print("collision check: OK")
    else:
        print(f"collision check: COLLISION / INVALID - {message}")
    return ok


def _print_check(bridge: SimBridge) -> bool:
    return _print_check_result(*_check_pose(bridge))


def _load_waypoints(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"robot_id": "R1", "waypoints": {}}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"cannot read waypoint file {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"waypoint file {path} must contain a JSON object")
    waypoints = data.setdefault("waypoints", {})
    if not isinstance(waypoints, dict):
        raise RuntimeError(f"waypoint file {path} has invalid 'waypoints'")
    data.setdefault("robot_id", "R1")
    return data


def _write_waypoint(
    bridge: SimBridge,
    output: Path,
    name: str,
    joints: list[float],
) -> None:
    scene = Path(bridge.scene_path())
    data = _load_waypoints(output)
    data["scene"] = str(scene)
    data["scene_fingerprint"] = {
        "size": scene.stat().st_size,
        "sha256": _sha256(scene),
    }
    data["updated_at"] = datetime.now(timezone.utc).isoformat()
    data["waypoints"][name] = {
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "joints_rad": _round(joints),
        "joints_deg": _joints_deg(joints),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(data, indent=2, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )


def _next_waypoint_name(output: Path, prefix: str) -> str:
    data = _load_waypoints(output)
    used = set(data.get("waypoints", {}))
    pattern = re.compile(rf"^{re.escape(prefix)}_(\d+)$")
    highest = 0
    for name in used:
        match = pattern.match(name)
        if match:
            highest = max(highest, int(match.group(1)))
    index = highest + 1
    while True:
        candidate = f"{prefix}_{index}"
        if candidate not in used:
            return candidate
        index += 1


def _connect(args: argparse.Namespace) -> SimBridge:
    bridge = SimBridge()
    if not bridge.connect(host=args.host, port=args.port):
        raise RuntimeError(
            bridge.last_error
            or f"cannot connect to CoppeliaSim at {args.host}:{args.port}"
        )
    return bridge


def _require_stopped(bridge: SimBridge, allow_running: bool) -> None:
    sim = bridge.sim
    if allow_running:
        return
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError(
            "CoppeliaSim simulation is running. Stop it first, or pass "
            "--allow-running to command joint targets instead of fixed joint poses."
        )


def cmd_show(args: argparse.Namespace) -> None:
    bridge = _connect(args)
    try:
        _print_pose(bridge.get_robot_joint_positions("R1"))
        if args.check:
            _print_check_result(
                *_check_pose(bridge, check_workspace=not args.ignore_workspace)
            )
    finally:
        bridge.disconnect()


def cmd_check(args: argparse.Namespace) -> None:
    bridge = _connect(args)
    try:
        _print_pose(bridge.get_robot_joint_positions("R1"))
        ok = _print_check_result(
            *_check_pose(bridge, check_workspace=not args.ignore_workspace)
        )
        if not ok:
            raise RuntimeError("current R1 pose is not collision-free")
    finally:
        bridge.disconnect()


def cmd_jog(args: argparse.Namespace) -> None:
    bridge = _connect(args)
    try:
        _require_stopped(bridge, args.allow_running)
        joints = bridge.get_robot_joint_positions("R1")
        index = args.joint - 1
        if args.deg is None and args.rad is None:
            raise RuntimeError("provide either --deg or --rad")
        if args.deg is not None and args.rad is not None:
            raise RuntimeError("use only one of --deg or --rad")
        delta = math.radians(args.deg) if args.deg is not None else float(args.rad)
        joints[index] += delta
        if not bridge.move_robot_joints("R1", joints):
            raise RuntimeError(bridge.last_error or "failed to move R1")
        _print_pose(bridge.get_robot_joint_positions("R1"))
        if not args.no_check:
            _print_check_result(
                *_check_pose(bridge, check_workspace=not args.ignore_workspace)
            )
    finally:
        bridge.disconnect()


def cmd_set(args: argparse.Namespace) -> None:
    bridge = _connect(args)
    try:
        _require_stopped(bridge, args.allow_running)
        if args.deg is None and args.rad is None:
            raise RuntimeError("provide either --deg or --rad")
        if args.deg is not None and args.rad is not None:
            raise RuntimeError("use only one of --deg or --rad")
        joints = (
            [math.radians(value) for value in args.deg]
            if args.deg is not None
            else [float(value) for value in args.rad]
        )
        if len(joints) != 6:
            raise RuntimeError("set requires exactly six joint values")
        if not bridge.move_robot_joints("R1", joints):
            raise RuntimeError(bridge.last_error or "failed to move R1")
        _print_pose(bridge.get_robot_joint_positions("R1"))
        if not args.no_check:
            _print_check_result(
                *_check_pose(bridge, check_workspace=not args.ignore_workspace)
            )
    finally:
        bridge.disconnect()


def cmd_record(args: argparse.Namespace) -> None:
    if not WAYPOINT_NAME_RE.match(args.name):
        raise RuntimeError(
            "waypoint name must start with a letter and contain only letters, "
            "numbers, and underscores"
        )
    bridge = _connect(args)
    try:
        joints = bridge.get_robot_joint_positions("R1")
        if not args.allow_collision:
            ok, message = _check_pose(
                bridge,
                check_workspace=not args.ignore_workspace,
            )
            if not ok:
                raise RuntimeError(
                    "refusing to record a colliding/invalid pose: "
                    f"{message}. Use --allow-collision only for diagnostics."
                )
        _write_waypoint(bridge, args.output, args.name, joints)
        print(f"recorded {args.name} -> {args.output}")
        _print_pose(joints)
    finally:
        bridge.disconnect()


def cmd_list(args: argparse.Namespace) -> None:
    data = _load_waypoints(args.output)
    waypoints = data.get("waypoints", {})
    if not waypoints:
        print(f"no waypoints recorded in {args.output}")
        return
    print(f"waypoints in {args.output}:")
    for name, waypoint in waypoints.items():
        print(f"  {name}: {json.dumps(waypoint['joints_deg'])} deg")


def _print_drive_help(step_deg: float) -> None:
    print("")
    print("R1 keyboard drive")
    print(f"  step: {step_deg:g} deg")
    print("  + joints: 1 2 3 4 5 6")
    print("  - joints: q w e r t y")
    print("  a: auto-record waypoint")
    print("  c: collision check")
    print("  s: show joints")
    print("  u: undo last move")
    print("  0: reset to zero")
    print("  +/-: change step")
    print("  h: help")
    print("  x or Esc: exit")
    print("  option: --ignore-workspace still checks collisions but allows workspace overrun")
    print("")


def _read_key() -> str:
    if not sys.stdin.isatty():
        raise RuntimeError("drive mode requires an interactive terminal")
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def cmd_drive(args: argparse.Namespace) -> None:
    if not WAYPOINT_NAME_RE.match(args.prefix):
        raise RuntimeError(
            "waypoint prefix must start with a letter and contain only letters, "
            "numbers, and underscores"
        )
    bridge = _connect(args)
    guard: R1SafetyGuard | None = None
    try:
        _require_stopped(bridge, False)
        robot = bridge.get_object_handle("R1")
        guard = R1SafetyGuard(bridge.sim, robot)
        step_deg = float(args.step_deg)
        previous: list[float] | None = None
        print("Connected to CoppeliaSim. Keep the simulation stopped.")
        _print_pose(bridge.get_robot_joint_positions("R1"))
        check_workspace = not args.ignore_workspace
        _print_check_result(*_check_guard(guard, check_workspace=check_workspace))
        _print_drive_help(step_deg)
        while True:
            key = _read_key()
            if key in {"x", "X", "\x1b"}:
                print("exit drive mode")
                return
            if key == "\x03":
                raise KeyboardInterrupt
            if key in {"h", "H", "?"}:
                _print_drive_help(step_deg)
                continue
            if key in {"+", "="}:
                step_deg = min(45.0, step_deg + 1.0)
                print(f"step: {step_deg:g} deg")
                continue
            if key in {"-", "_"}:
                step_deg = max(0.25, step_deg - 1.0)
                print(f"step: {step_deg:g} deg")
                continue
            if key in {"s", "S"}:
                _print_pose(bridge.get_robot_joint_positions("R1"))
                continue
            if key in {"c", "C"}:
                _print_check_result(
                    *_check_guard(guard, check_workspace=check_workspace)
                )
                continue
            if key in {"u", "U"}:
                if previous is None:
                    print("nothing to undo")
                    continue
                if not bridge.move_robot_joints("R1", previous):
                    raise RuntimeError(bridge.last_error or "failed to undo R1 move")
                print("undo")
                _print_pose(bridge.get_robot_joint_positions("R1"))
                _print_check_result(
                    *_check_guard(guard, check_workspace=check_workspace)
                )
                continue
            if key == "0":
                previous = bridge.get_robot_joint_positions("R1")
                if not bridge.move_robot_joints("R1", [0.0] * 6):
                    raise RuntimeError(bridge.last_error or "failed to reset R1")
                print("reset to zero")
                _print_pose(bridge.get_robot_joint_positions("R1"))
                _print_check_result(
                    *_check_guard(guard, check_workspace=check_workspace)
                )
                continue
            if key in {"a", "A"}:
                ok, message = _check_guard(guard, check_workspace=check_workspace)
                if not _print_check_result(ok, message):
                    print("not recorded")
                    continue
                name = _next_waypoint_name(args.output, args.prefix)
                joints = bridge.get_robot_joint_positions("R1")
                _write_waypoint(bridge, args.output, name, joints)
                print(f"recorded {name} -> {args.output}")
                continue

            sign = 0
            joint_index = None
            if key in DRIVE_POSITIVE_KEYS:
                sign = 1
                joint_index = DRIVE_POSITIVE_KEYS[key]
            elif key in DRIVE_NEGATIVE_KEYS:
                sign = -1
                joint_index = DRIVE_NEGATIVE_KEYS[key]
            if joint_index is None:
                print(f"unknown key: {key!r}; press h for help")
                continue

            previous = bridge.get_robot_joint_positions("R1")
            target = list(previous)
            target[joint_index] += math.radians(sign * step_deg)
            if not bridge.move_robot_joints("R1", target):
                raise RuntimeError(bridge.last_error or "failed to move R1")
            direction = "+" if sign > 0 else "-"
            print(f"joint{joint_index + 1} {direction}{step_deg:g} deg")
            _print_pose(bridge.get_robot_joint_positions("R1"))
            if args.no_check:
                continue
            ok, message = _check_guard(guard, check_workspace=check_workspace)
            _print_check_result(ok, message)
            if not ok and not args.keep_collision:
                if not bridge.move_robot_joints("R1", previous):
                    raise RuntimeError(
                        bridge.last_error or "failed to revert colliding R1 pose"
                    )
                print("collision detected; reverted to previous pose")
                _print_pose(bridge.get_robot_joint_positions("R1"))
    finally:
        if guard is not None:
            guard.close()
        bridge.disconnect()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Jog and record R1 joint waypoints in CoppeliaSim."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="JSON waypoint file for record/list commands.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    show = subparsers.add_parser("show", help="Print current R1 joints.")
    show.add_argument("--check", action="store_true", help="Also run collision check.")
    show.add_argument("--ignore-workspace", action="store_true")
    show.set_defaults(func=cmd_show)

    check = subparsers.add_parser("check", help="Check current R1 pose for collision.")
    check.add_argument("--ignore-workspace", action="store_true")
    check.set_defaults(func=cmd_check)

    jog = subparsers.add_parser("jog", help="Move one joint by a relative delta.")
    jog.add_argument("--joint", type=int, choices=range(1, 7), required=True)
    jog.add_argument("--deg", type=float, help="Relative joint delta in degrees.")
    jog.add_argument("--rad", type=float, help="Relative joint delta in radians.")
    jog.add_argument("--allow-running", action="store_true")
    jog.add_argument("--no-check", action="store_true")
    jog.add_argument("--ignore-workspace", action="store_true")
    jog.set_defaults(func=cmd_jog)

    set_cmd = subparsers.add_parser("set", help="Set all six joints.")
    set_cmd.add_argument("--deg", type=float, nargs=6)
    set_cmd.add_argument("--rad", type=float, nargs=6)
    set_cmd.add_argument("--allow-running", action="store_true")
    set_cmd.add_argument("--no-check", action="store_true")
    set_cmd.add_argument("--ignore-workspace", action="store_true")
    set_cmd.set_defaults(func=cmd_set)

    record = subparsers.add_parser("record", help="Record current R1 pose.")
    record.add_argument("name")
    record.add_argument("--allow-collision", action="store_true")
    record.add_argument("--ignore-workspace", action="store_true")
    record.set_defaults(func=cmd_record)

    list_cmd = subparsers.add_parser("list", help="List recorded waypoints.")
    list_cmd.set_defaults(func=cmd_list)

    drive = subparsers.add_parser("drive", help="Interactive keyboard joint drive.")
    drive.add_argument("--step-deg", type=float, default=5.0)
    drive.add_argument("--prefix", default="initial_escape")
    drive.add_argument("--no-check", action="store_true")
    drive.add_argument("--keep-collision", action="store_true")
    drive.add_argument("--ignore-workspace", action="store_true")
    drive.set_defaults(func=cmd_drive)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.func(args)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
