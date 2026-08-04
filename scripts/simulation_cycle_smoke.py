#!/usr/bin/env python3
"""Run one scheduler-driven order against the open CoppeliaSim scene."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from interfaces.types import Order
from orchestration.cell_orchestrator import CellOrchestrator
from robot_control.simulation_executor import SimulationCellExecutor
from scheduler.scheduler import Scheduler
from sim_bridge.coppelia_client import SimBridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--quality", choices=("OK", "NG"), default="OK")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--stop-after", type=int, default=0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bridge = SimBridge(host=args.host, port=args.port, request_timeout=20.0)
    try:
        if not bridge.connect(args.host, args.port):
            raise RuntimeError(bridge.last_error or "CoppeliaSim connection failed")
        if bridge.sim.getSimulationState() != bridge.sim.simulation_stopped:
            if not bridge.stop_simulation():
                raise RuntimeError(bridge.last_error or "cannot stop simulation")

        order = Order("SIM-ACCEPT", "A", priority=1, quantity=1)
        motion = SimulationCellExecutor(
            bridge,
            quality_by_order={order.order_id: args.quality},
        )
        evidence = motion.prepare_cycle()
        print(
            "SIMULATION READY: "
            f"path_points={evidence.get('path_points_total')} "
            f"prepare_wall_s={evidence.get('prepare_wall_s', 0.0):.3f}",
            flush=True,
        )
        if args.prepare_only:
            return 0

        orchestrator = CellOrchestrator(Scheduler(), motion)
        completed_count = 0

        def report(event):
            nonlocal completed_count
            if event.kind == "task_dispatched":
                print(f"DISPATCH {event.task_id}", flush=True)
            elif event.kind == "task_completed" and event.result is not None:
                completed_count += 1
                result = event.result
                suffix = (
                    f" quality={result.quality_result}"
                    if result.quality_result
                    else ""
                )
                print(
                    f"RESULT {result.task_id} {result.robot_id} "
                    f"{result.status}{suffix}: {result.message}",
                    flush=True,
                )
                if args.stop_after and completed_count >= args.stop_after:
                    orchestrator.stop()
            elif event.kind in {"finished", "failed", "stopped"}:
                print(f"ORCHESTRATOR {event.kind}: {event.message}", flush=True)

        orchestrator.add_event_callback(report)
        orchestrator.start([order])
        status = orchestrator.wait(args.timeout)
        if status not in {"finished", "failed", "stopped"}:
            orchestrator.stop()
            raise RuntimeError(f"simulation cycle timed out with status {status}")
        finished = sum(task.status == "finished" for task in orchestrator.tasks)
        sim = bridge.sim
        visual_paths = {
            "actual_pcb": "/FiveCR5A_Cell/Parts/PCB_Supply",
            "template_pcb": (
                "/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product/"
                "Assembly_ControlBox_Product_PCB"
            ),
        }
        visual_counts = {}
        for name, path in visual_paths.items():
            root = sim.getObject(path)
            shapes = sim.getObjectsInTree(root, sim.object_shape_type, 0)
            visual_counts[name] = sum(
                sim.getObjectInt32Param(shape, sim.objintparam_visibility_layer) != 0
                for shape in shapes
            )
        print(
            f"VISUAL OWNER={bridge.get_visual_owner()} VISIBLE={visual_counts}",
            flush=True,
        )
        print(
            f"SIMULATION CYCLE {status.upper()}: "
            f"tasks={finished}/{len(orchestrator.tasks)}",
            flush=True,
        )
        return 0 if status == "finished" else 1
    finally:
        if bridge.is_connected():
            bridge.stop_simulation()
        bridge.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
