#!/usr/bin/env python3
"""Run one complete five-CR5A visual assembly cycle in the current scene."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_control.five_arm_coordinator import FiveArmCoordinator
from sim_bridge.coppelia_client import SimBridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quality", choices=("good", "defect"), default="good")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--speed-deg-s", type=float, default=50.0)
    parser.add_argument("--hold-seconds", type=float, default=0.8)
    parser.add_argument("--order-id", default="FIVE-ARM-DEMO")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    coordinator = FiveArmCoordinator(
        bridge=SimBridge(args.host, args.port),
        speed_deg_s=args.speed_deg_s,
        hold_seconds=args.hold_seconds,
    )
    evidence = coordinator.execute_cycle(args.quality, args.order_id)
    rendered = json.dumps(evidence, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if evidence["status"] == "finished" else 1


if __name__ == "__main__":
    raise SystemExit(main())
