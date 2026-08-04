#!/usr/bin/env python3
"""Measure planned pick endpoints against visible TCPs and workpieces."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim_bridge.coppelia_client import SimBridge
from sim_bridge.process_manager import CoppeliaProcessManager
from sim_bridge.scene_objects import ROBOT_TIPS


CASES = (
    ("R1", "r1_complete_cycle_plan.json", None, "box_descend", "BOX_BLANK"),
    ("R1", "r1_complete_cycle_plan.json", None, "terminal_descend", "TERMINAL_BLOCK_SUPPLY"),
    ("R2", "r2_pcb_cycle_plan.json", None, "pick_descend", "PCB_SUPPLY"),
    ("R3", "r3_gripper_cycle_plan.json", "R3_MODULE_PLACED", "pick_descend", "CONTROL_MODULE_SUPPLY"),
    ("R3", "r3_gripper_cycle_plan.json", "R3_PRODUCT_TO_INSPECTION", "pick_descend", "ASSEMBLY_PRODUCT"),
    ("R5", "r5_sort_cycle_plan.json", "R5_SORT_GOOD_DONE", "pick_descend", "INSPECTION_PRODUCT"),
)


def _find_alias(sim, root: int, alias: str) -> int:
    matches = [
        handle
        for handle in sim.getObjectsInTree(root, sim.handle_all, 0)
        if sim.getObjectAlias(handle) == alias
    ]
    if len(matches) != 1:
        raise RuntimeError(f"expected one {alias} below robot, found {len(matches)}")
    return matches[0]


def _connect_or_launch() -> tuple[SimBridge, CoppeliaProcessManager]:
    manager = CoppeliaProcessManager()
    bridge = SimBridge(request_timeout=20.0)
    if manager.endpoint_reachable() and bridge.connect():
        return bridge, manager
    manager.launch()
    deadline = time.monotonic() + manager.startup_timeout
    while time.monotonic() < deadline:
        time.sleep(0.5)
        if manager.endpoint_reachable() and bridge.connect():
            return bridge, manager
    raise RuntimeError("CoppeliaSim did not become ready")


def main() -> int:
    bridge, manager = _connect_or_launch()
    try:
        sim = bridge.sim
        if sim.getSimulationState() != sim.simulation_stopped:
            if not bridge.stop_simulation():
                raise RuntimeError(bridge.last_error or "cannot stop simulation")
        plans = ROOT / "robot_control" / "plans"
        for robot_id, filename, action, path_name, part_name in CASES:
            plan = json.loads((plans / filename).read_text(encoding="utf-8"))
            paths = plan["paths"] if action is None else plan["paths"][action]
            endpoint = [float(value) for value in paths[path_name][-1]]
            robot = bridge.get_object_handle(robot_id)
            joints = bridge.get_robot_joint_handles(robot_id)
            original = [float(sim.getJointPosition(joint)) for joint in joints]
            for joint, value in zip(joints, endpoint):
                sim.setJointPosition(joint, value)
            tip = _find_alias(sim, robot, ROBOT_TIPS[robot_id])
            part = bridge.get_object_handle(part_name)
            tip_position = [float(value) for value in sim.getObjectPosition(tip, -1)]
            part_position = [float(value) for value in sim.getObjectPosition(part, -1)]
            delta = [tip_position[index] - part_position[index] for index in range(3)]
            distance = math.sqrt(sum(value * value for value in delta))
            print(
                f"{robot_id} {part_name}: tip={tip_position} part={part_position} "
                f"tip-part={delta} distance={distance:.6f}m"
            )
            if robot_id == "R2":
                for handle in sim.getObjectsInTree(
                    robot, sim.object_shape_type, 0
                ):
                    alias = sim.getObjectAlias(handle)
                    if "R2T_plate" in alias or "R2T_cup_" in alias:
                        print(
                            f"  {alias}: "
                            f"{[float(value) for value in sim.getObjectPosition(handle, -1)]}"
                        )
            for joint, value in zip(joints, original):
                sim.setJointPosition(joint, value)
        return 0
    finally:
        bridge.disconnect()
        manager.terminate_owned_process()


if __name__ == "__main__":
    raise SystemExit(main())
