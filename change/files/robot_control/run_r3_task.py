#!/usr/bin/env python3
"""Run one R3 action through the official IRobotExecutor contract."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from interfaces.types import Task, TaskStatus
from robot_control.r3_motion import R3_ACTIONS, R3_MODULE_PLACED
from robot_control.robot_executor import RobotExecutor
from sim_bridge.coppelia_client import SimBridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=sorted(R3_ACTIONS))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--speed-deg-s", type=float, default=50.0)
    parser.add_argument("--hold-seconds", type=float, default=0.8)
    parser.add_argument("--task-id")
    parser.add_argument("--order-id", default="R3-MANUAL")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    task = Task(
        task_id=args.task_id or args.action,
        order_id=args.order_id,
        product_type="A",
        process="assemble",
        target_area=(
            "assembly_area"
            if args.action == R3_MODULE_PLACED
            else "inspection_screw_area"
        ),
        target_point=args.action,
        available_robots=["R3"],
    )
    result = RobotExecutor(
        sim_bridge=SimBridge(args.host, args.port),
        speed_deg_s=args.speed_deg_s,
        hold_seconds=args.hold_seconds,
    ).execute_task(task)
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
    return 0 if result.status == TaskStatus.FINISHED.value else 1


if __name__ == "__main__":
    raise SystemExit(main())
