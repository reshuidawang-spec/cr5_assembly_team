#!/usr/bin/env python3
"""Drive a coarse CoppeliaSim display from the CR5 scheduler.

This script is intentionally visual-first: it uses the existing scheduler to
produce a dynamic schedule, then moves the five CR5 arm joints approximately
and updates scene stage signals. It is not an IK/trajectory controller.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parents[1]

sys.path.insert(0, str(ROOT))

from interfaces.types import Order  # noqa: E402
from scheduler.config_loader import load_yaml  # noqa: E402
from scheduler.dynamic_order_sequence import (  # noqa: E402
    DynamicOrderInput,
    QualityOverrideExperiment,
    expand_order_inputs_as_units,
    expand_quality_overrides,
    to_orders,
)
from scheduler.experiment import DiscreteEventExperiment, ScheduleRecord  # noqa: E402


def safe_print(message: str) -> None:
    try:
        print(message, flush=True)
    except OSError:
        pass


def _candidate_coppeliasim_roots(coppeliasim_root: Path | None = None) -> list[Path]:
    roots: list[Path] = []
    if coppeliasim_root:
        roots.append(coppeliasim_root)
    env_root = os.environ.get("COPPELIASIM_ROOT")
    if env_root:
        roots.append(Path(env_root).expanduser())
    roots.extend(
        [
            Path(r"D:\CoppeliaSim\CoppeliaSim_Edu_V4_10_0_rev0_Win"),
            Path("/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04"),
            Path("/opt/CoppeliaSim"),
            Path("~/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04").expanduser(),
            Path("~/CoppeliaSim").expanduser(),
        ]
    )
    unique_roots: list[Path] = []
    seen: set[str] = set()
    for root in roots:
        key = str(root)
        if key not in seen:
            unique_roots.append(root)
            seen.add(key)
    return unique_roots


def configure_coppelia_zmq_client(coppeliasim_root: Path | None = None) -> Path | None:
    for root in _candidate_coppeliasim_roots(coppeliasim_root):
        client_path = root / "programming" / "zmqRemoteApi" / "clients" / "python" / "src"
        if client_path.exists():
            client_path_str = str(client_path)
            if client_path_str not in sys.path:
                sys.path.insert(0, client_path_str)
            return client_path
    return None


def load_remote_api_client(coppeliasim_root: Path | None = None):
    configure_coppelia_zmq_client(coppeliasim_root)
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    except ModuleNotFoundError as exc:
        searched = "\n".join(f"- {root}" for root in _candidate_coppeliasim_roots(coppeliasim_root))
        raise RuntimeError(
            "未找到 CoppeliaSim ZMQ Remote API Python 客户端。\n"
            "请确认已安装 CoppeliaSim，并设置环境变量 COPPELIASIM_ROOT，例如：\n"
            "  export COPPELIASIM_ROOT=/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04\n"
            "或运行脚本时加入：--coppeliasim-root /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04\n"
            f"已搜索路径：\n{searched}"
        ) from exc
    return RemoteAPIClient


PROCESS_TO_POINT = {
    "box_feed": "R1_BOX_PLACE_TCP",
    "pcb_install": "R2_PCB_PLACE_TCP",
    "module_install": "R3_MODULE_PLACE_TCP",
    "terminal_install": "R1_TERMINAL_PLACE_TCP",
    "transfer_to_inspection": "R3_PRODUCT_PLACE_INSPECTION_TCP",
    "inspect": "CAMERA_INSPECTION_CENTER",
    "screw": "R4_SCREW_PRESS",
    "sort_good": "R5_GOOD_PLACE_TCP",
    "sort_defect": "R5_DEFECT_PLACE_TCP",
}

PROCESS_TO_STAGE_SIGNAL = {
    "box_feed": ("cell_product_state", "assembly_shell"),
    "pcb_install": ("cell_product_state", "assembly_pcb"),
    "module_install": ("cell_product_state", "assembly_module"),
    "terminal_install": ("cell_product_state", "assembly_full"),
    "transfer_to_inspection": ("cell_product_state", "inspection_full"),
    "screw": ("cell_screw_state", "done"),
    "sort_good": ("cell_conveyor_state", "good"),
    "sort_defect": ("cell_conveyor_state", "defect"),
}

ROBOT_COLORS = {
    "R1": [0.95, 0.28, 0.20],
    "R2": [0.20, 0.55, 1.00],
    "R3": [0.35, 0.85, 0.35],
    "R4": [1.00, 0.72, 0.18],
    "R5": [0.82, 0.42, 1.00],
    "CAMERA": [0.1, 0.9, 0.9],
}

PRODUCT_COLORS = {
    "A": [1.00, 0.86, 0.05],  # yellow
    "B": [0.20, 0.90, 0.25],  # green
    "C": [0.20, 0.55, 1.00],  # blue
    "URGENT": [1.00, 0.05, 0.05],  # red
    "COMPLETED": [0.55, 0.58, 0.62],  # grey
}

PRODUCT_SHELL_GROUP_ALIASES = {
    "assembly": ("Assembly_ControlBox_Product_Shell",),
    "inspection": ("Inspection_ControlBox_Product_Shell",),
}

PROCESS_TO_PRODUCT_SHELL_GROUPS = {
    "box_feed": ("assembly",),
    "pcb_install": ("assembly",),
    "module_install": ("assembly",),
    "terminal_install": ("assembly",),
    "transfer_to_inspection": ("assembly", "inspection"),
    "inspect": ("inspection",),
    "screw": ("inspection",),
    "sort_good": ("inspection",),
    "sort_defect": ("inspection",),
}

ROBOT_HOME_POINTS = {
    "R1": "R1_HOME_REF",
    "R2": "R2_HOME_REF",
    "R3": "R3_HOME_REF",
    "R4": "R4_HOME_REF",
    "R5": "R5_HOME_REF",
    "CAMERA": "CAMERA_INSPECTION_CENTER",
}

ROBOT_BASE_JOINT_POSES = {
    ("R1", "box_feed"): [-0.65, 0.48, 0.70, 0.0, 0.55, 0.0],
    ("R1", "terminal_install"): [-1.05, 0.42, 0.95, 0.0, 0.62, 0.0],
    ("R2", "pcb_install"): [0.68, 0.55, 0.82, 0.0, 0.52, 0.0],
    ("R3", "module_install"): [-0.88, 0.48, 0.88, 0.0, 0.55, 0.0],
    ("R3", "transfer_to_inspection"): [0.95, 0.45, 0.78, 0.0, 0.60, 0.0],
    ("R4", "screw"): [-0.72, 0.60, 0.95, 0.0, 0.40, 0.0],
    ("R5", "sort_good"): [-1.05, 0.42, 0.72, 0.0, 0.60, 0.0],
    ("R5", "sort_defect"): [1.00, 0.42, 0.72, 0.0, 0.60, 0.0],
}


@dataclass(frozen=True)
class DemoTask:
    record: ScheduleRecord
    point_name: str
    target_position: list[float]
    quality_result: str


def quality_for(order_id: str, quality_overrides: dict[str, str] | None = None) -> str:
    if quality_overrides and quality_overrides.get(order_id) in ("OK", "NG"):
        return quality_overrides[order_id]
    checksum = sum(ord(char) for char in order_id)
    return "NG" if checksum % 3 == 0 else "OK"


def product_color_for(product_type: str, priority: int) -> list[float]:
    color_key = "URGENT" if priority >= 5 else product_type
    return list(PRODUCT_COLORS.get(color_key, [1.0, 1.0, 1.0]))


def shell_groups_for_process(process: str) -> tuple[str, ...]:
    return PROCESS_TO_PRODUCT_SHELL_GROUPS.get(process, ())


def build_orders(urgent_type: str, insert_time: float) -> list[Order]:
    orders = [
        Order("A001", "A", 1, due_time=260),
        Order("A002", "A", 1, due_time=280),
        Order("A003", "A", 1, due_time=300),
        Order("B001", "B", 2, due_time=340),
        Order("C001", "C", 2, due_time=380),
        Order("C002", "C", 2, due_time=410),
        Order(
            f"URGENT_{urgent_type}",
            urgent_type,
            5,
            due_time=insert_time + 95,
            arrival_time=insert_time,
        ),
    ]
    return orders


def build_demo_tasks(
    orders: list[Order],
    quality_overrides: dict[str, str] | None = None,
    scoring_weights: dict[str, float] | None = None,
) -> tuple[list[DemoTask], dict]:
    experiment = (
        QualityOverrideExperiment(quality_overrides, scoring_weights)
        if quality_overrides or scoring_weights
        else DiscreteEventExperiment()
    )
    result = experiment.run_proposed(orders)
    points = load_yaml(ROOT / "configs" / "points.yaml")
    demo_tasks = []
    for record in sorted(result.records, key=lambda item: (item.start_time, item.end_time)):
        point_name = PROCESS_TO_POINT.get(record.process, "")
        target_position = list(points.get(point_name, {}).get("position", [0.0, 0.0, 0.6]))
        demo_tasks.append(
            DemoTask(
                record=record,
                point_name=point_name,
                target_position=target_position,
                quality_result=quality_for(record.order_id, quality_overrides),
            )
        )
    return demo_tasks, result.summary_dict()


def load_orders_json(path: Path) -> tuple[list[Order], dict[str, str], float, dict[str, float]]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    raw_orders = payload.get("orders", payload)
    raw_scoring = payload.get("scoring", {})
    inputs = [
        DynamicOrderInput(
            order_id=str(item["order_id"]),
            product_type=str(item["product_type"]).upper(),
            quantity=int(item.get("quantity", 1)),
            priority=int(item.get("priority", 1)),
            due_time=float(item.get("due_time", 0.0)),
            arrival_time=float(item.get("arrival_time", 0.0)),
            quality=str(item.get("quality", "AUTO")).upper(),
        )
        for item in raw_orders
    ]
    expanded_inputs = expand_order_inputs_as_units(inputs)
    orders = to_orders(expanded_inputs)
    quality_overrides = expand_quality_overrides(expanded_inputs)
    urgent_arrivals = [order.arrival_time for order in orders if order.priority >= 5]
    urgent_insert_time = min(urgent_arrivals) if urgent_arrivals else 0.0
    scoring_weights = {
        str(key): float(value)
        for key, value in raw_scoring.items()
        if isinstance(value, (int, float))
    }
    return orders, quality_overrides, urgent_insert_time, scoring_weights


class CoppeliaSchedulerDisplay:
    def __init__(self, coppeliasim_root: Path | None = None) -> None:
        remote_api_client = load_remote_api_client(coppeliasim_root)
        self.client = remote_api_client(host="localhost", port=23000)
        self.sim = self.client.getObject("sim")
        self.points = load_yaml(ROOT / "configs" / "points.yaml")
        self.robot_joints = self._find_robot_joints()
        self.robot_markers: dict[str, int] = {}
        self.order_markers: dict[str, int] = {}
        self.product_shell_handles = self._find_product_shell_handles()
        self.urgent_marker: int | None = None

    def setup(self, orders: Iterable[Order]) -> None:
        self.cleanup()
        self._ensure_simulation_running()
        self.sim.setStringSignal("cell_product_state", "reset")
        time.sleep(0.2)
        self._make_resource_markers()
        self._make_order_markers(orders)
        self._log("SCHED-DEMO: orders loaded. New orders can be synchronized in watch mode.")

    def cleanup(self) -> None:
        handles = []
        for handle in self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.handle_all, 0):
            try:
                alias = self.sim.getObjectAlias(handle, 1)
            except Exception:
                continue
            if "/sched_demo_" in alias:
                handles.append(handle)
        if handles:
            self.sim.removeObjects(handles)

    def play(
        self,
        tasks: list[DemoTask],
        summary: dict,
        speed: float,
        urgent_insert_time: float,
        watch_orders_path: Path | None = None,
        quality_overrides: dict[str, str] | None = None,
    ) -> None:
        if not tasks:
            return
        start_wall = time.monotonic()
        makespan = max(task.record.end_time for task in tasks)
        started: set[str] = set()
        finished: set[str] = set()
        urgent_announced = False
        last_orders_signature = self._orders_file_signature(watch_orders_path)

        while True:
            current_time = (time.monotonic() - start_wall) * speed
            if watch_orders_path:
                signature = self._orders_file_signature(watch_orders_path)
                if signature and signature != last_orders_signature:
                    try:
                        orders, quality_overrides, urgent_insert_time, scoring_weights = load_orders_json(watch_orders_path)
                        tasks, summary = build_demo_tasks(orders, quality_overrides, scoring_weights)
                        makespan = max(task.record.end_time for task in tasks)
                        self._sync_order_markers(orders)
                        last_orders_signature = signature
                        self._log(
                            f"ORDER UPDATE t={current_time:.1f}: {len(orders)} units synchronized; "
                            f"new makespan={summary.get('makespan', 0.0):.1f}s"
                        )
                    except Exception as exc:
                        self._log(f"ORDER UPDATE FAILED: {exc}")

            if not urgent_announced and current_time >= urgent_insert_time:
                urgent_announced = True
                self._announce_urgent_order(urgent_insert_time)

            active_by_robot: dict[str, DemoTask] = {}
            for task in tasks:
                record = task.record
                key = f"{record.task_id}:{record.robot_id}"
                if record.start_time <= current_time < record.end_time:
                    active_by_robot[record.robot_id] = task
                if current_time >= record.start_time and key not in started:
                    started.add(key)
                    self._on_task_start(task)

            for task in tasks:
                record = task.record
                key = f"{record.task_id}:{record.robot_id}"
                if current_time >= record.end_time and key not in finished:
                    finished.add(key)
                    self._on_task_finish(task)

            for robot_id in ["R1", "R2", "R3", "R4", "R5", "CAMERA"]:
                task = active_by_robot.get(robot_id)
                if task:
                    ratio = (current_time - task.record.start_time) / max(
                        task.record.end_time - task.record.start_time,
                        0.001,
                    )
                    self._show_active_task(robot_id, task, ratio)
                else:
                    self._show_idle_resource(robot_id)

            if current_time > makespan + 2:
                break
            time.sleep(0.05)

        for robot_id in ["R1", "R2", "R3", "R4", "R5", "CAMERA"]:
            self._show_idle_resource(robot_id)
        self._log(
            "SCHED-DEMO finished: makespan={makespan:.1f}s, conflicts={conflicts}, urgent_completion={urgent:.1f}s".format(
                makespan=summary.get("makespan", 0.0),
                conflicts=summary.get("conflict_count", 0),
                urgent=summary.get("urgent_completion_time", 0.0),
            )
        )

    def _ensure_simulation_running(self) -> None:
        state = self.sim.getSimulationState()
        if state == self.sim.simulation_stopped:
            self.sim.startSimulation()
            time.sleep(0.5)

    def _find_robot_joints(self) -> dict[str, list[int]]:
        joints = {}
        for robot_id in ["R1", "R2", "R3", "R4", "R5"]:
            robot_joints = []
            for index in range(1, 7):
                try:
                    robot_joints.append(self.sim.getObject(f"/{robot_id}/joint{index}"))
                except Exception:
                    pass
            joints[robot_id] = robot_joints
        return joints

    def _find_product_shell_handles(self) -> dict[str, list[int]]:
        handles: dict[str, list[int]] = {
            group: []
            for group in PRODUCT_SHELL_GROUP_ALIASES
        }
        try:
            scene_handles = self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.handle_all, 0)
        except Exception:
            return handles
        for handle in scene_handles:
            try:
                alias = self.sim.getObjectAlias(handle, 1)
            except Exception:
                continue
            for group, aliases in PRODUCT_SHELL_GROUP_ALIASES.items():
                if any(name in alias for name in aliases):
                    handles[group].append(handle)
        return handles

    def _set_product_shell_color(self, group: str, color: list[float]) -> None:
        if not self.product_shell_handles.get(group):
            self.product_shell_handles = self._find_product_shell_handles()
        for handle in self.product_shell_handles.get(group, []):
            self._set_shape_color(handle, color)

    def _set_product_shell_color_for_task(self, task: DemoTask) -> None:
        color = product_color_for(task.record.product_type, task.record.priority)
        for group in shell_groups_for_process(task.record.process):
            self._set_product_shell_color(group, color)

    def _set_shape_color(self, handle: int, color: list[float]) -> None:
        try:
            self.sim.setShapeColor(handle, "", self.sim.colorcomponent_ambient_diffuse, color)
        except Exception:
            pass

    def _make_resource_markers(self) -> None:
        for robot_id, point_name in ROBOT_HOME_POINTS.items():
            position = self._point(point_name)
            position[2] += 0.35
            marker = self._make_box(
                f"sched_demo_resource_{robot_id}",
                position,
                [0.07, 0.07, 0.07],
                ROBOT_COLORS[robot_id],
            )
            self.robot_markers[robot_id] = marker

    def _make_order_markers(self, orders: Iterable[Order]) -> None:
        y = 1.08
        for index, order in enumerate(orders):
            color = product_color_for(order.product_type, order.priority)
            marker = self._make_box(
                f"sched_demo_order_{order.order_id}",
                [1.28, y - 0.12 * index, 0.55],
                [0.10, 0.10, 0.06],
                color,
            )
            self.order_markers[order.order_id] = marker
        self.urgent_marker = self.order_markers.get(next((o.order_id for o in orders if o.priority >= 5), ""), None)
        if self.urgent_marker is not None:
            self.sim.setObjectPosition(self.urgent_marker, -1, [1.28, 1.08, 1.8])

    def _sync_order_markers(self, orders: Iterable[Order]) -> None:
        y = 1.08
        for index, order in enumerate(orders):
            marker = self.order_markers.get(order.order_id)
            if marker is None:
                color = product_color_for(order.product_type, order.priority)
                marker = self._make_box(
                    f"sched_demo_order_{order.order_id}",
                    [1.28, y - 0.12 * index, 0.55],
                    [0.10, 0.10, 0.06],
                    color,
                )
                self.order_markers[order.order_id] = marker
            else:
                try:
                    self.sim.setObjectPosition(marker, -1, [1.28, y - 0.12 * index, 0.55])
                    self._set_shape_color(marker, product_color_for(order.product_type, order.priority))
                except Exception:
                    pass

    def _orders_file_signature(self, path: Path | None) -> tuple[float, int] | None:
        if not path:
            return None
        try:
            stat = path.stat()
            return stat.st_mtime, stat.st_size
        except OSError:
            return None

    def _make_box(self, alias: str, position: list[float], size: list[float], color: list[float]) -> int:
        handle = self.sim.createPureShape(0, 16, size, 0.001)
        self.sim.setObjectAlias(handle, alias)
        self.sim.setObjectPosition(handle, -1, position)
        self.sim.setShapeColor(handle, "", self.sim.colorcomponent_ambient_diffuse, color)
        return handle

    def _on_task_start(self, task: DemoTask) -> None:
        record = task.record
        self._set_product_shell_color_for_task(task)
        marker = self.order_markers.get(record.order_id)
        if marker is not None:
            self._set_shape_color(marker, product_color_for(record.product_type, record.priority))
        self._log(
            f"START t={record.start_time:.1f}: {record.order_id} {record.product_type} "
            f"{record.process} -> {record.robot_id}"
        )
        self.sim.setStringSignal(f"{record.robot_id.lower()}_ros_cmd", record.process)

    def _on_task_finish(self, task: DemoTask) -> None:
        record = task.record
        if record.process == "inspect":
            signal_value = "camera_good" if task.quality_result == "OK" else "camera_defect"
            self.sim.setStringSignal("cell_product_state", signal_value)
        elif record.process in PROCESS_TO_STAGE_SIGNAL:
            signal_name, signal_value = PROCESS_TO_STAGE_SIGNAL[record.process]
            self.sim.setStringSignal(signal_name, signal_value)
        self._log(
            f"FINISH t={record.end_time:.1f}: {record.order_id} {record.process} "
            f"({task.quality_result if record.process == 'inspect' else 'done'})"
        )
        if record.process in ("sort_good", "sort_defect"):
            completed_color = PRODUCT_COLORS["COMPLETED"]
            marker = self.order_markers.get(record.order_id)
            if marker is not None:
                self._set_shape_color(marker, completed_color)
            self._set_product_shell_color("inspection", completed_color)

    def _show_active_task(self, robot_id: str, task: DemoTask, ratio: float) -> None:
        target = list(task.target_position)
        target[2] += 0.48
        if robot_id in self.robot_markers:
            pulse = 0.04 * math.sin(ratio * math.pi * 6)
            self.sim.setObjectPosition(self.robot_markers[robot_id], -1, [target[0], target[1], target[2] + pulse])
        if robot_id != "CAMERA":
            self._pose_robot(robot_id, task.record.process, ratio)
        marker = self.order_markers.get(task.record.order_id)
        if marker is not None:
            self.sim.setObjectPosition(marker, -1, [target[0], target[1], target[2] + 0.12])

    def _show_idle_resource(self, robot_id: str) -> None:
        if robot_id in self.robot_markers:
            position = self._point(ROBOT_HOME_POINTS[robot_id])
            position[2] += 0.35
            self.sim.setObjectPosition(self.robot_markers[robot_id], -1, position)
        if robot_id != "CAMERA":
            self._pose_robot(robot_id, "", 0.0)

    def _pose_robot(self, robot_id: str, process: str, ratio: float) -> None:
        joints = self.robot_joints.get(robot_id, [])
        if len(joints) != 6:
            return
        target_pose = ROBOT_BASE_JOINT_POSES.get((robot_id, process), [0.0] * 6)
        motion = math.sin(max(0.0, min(ratio, 1.0)) * math.pi)
        for index, joint in enumerate(joints):
            value = target_pose[index] * motion
            if robot_id == "R4" and process == "screw" and index == 5:
                value += ratio * math.pi * 5.0
            self.sim.setJointPosition(joint, value)

    def _announce_urgent_order(self, insert_time: float) -> None:
        if self.urgent_marker is not None:
            self.sim.setObjectPosition(self.urgent_marker, -1, [1.28, 1.08, 0.80])
        self.sim.setStringSignal("cell_product_state", "color_next")
        self._log(f"URGENT INSERT t={insert_time:.1f}: priority=5 order enters the queue")

    def _point(self, point_name: str) -> list[float]:
        return list(self.points.get(point_name, {}).get("position", [0.0, 0.0, 0.6]))

    def _log(self, message: str) -> None:
        safe_print(message)
        try:
            self.sim.addLog(self.sim.verbosity_scriptinfos, message)
        except Exception:
            pass


def print_schedule(tasks: list[DemoTask], summary: dict) -> None:
    safe_print("Generated dynamic schedule:")
    for task in tasks:
        record = task.record
        urgent = " URGENT" if record.priority >= 5 else ""
        safe_print(
            f"{record.start_time:6.1f}-{record.end_time:6.1f}  "
            f"{record.robot_id:6s}  {record.order_id:10s}  "
            f"{record.product_type}  {record.process:24s}{urgent}"
        )
    safe_print(
        "Summary: makespan={makespan:.1f}s, conflicts={conflict_count}, "
        "urgent_completion={urgent_completion_time:.1f}s, urgent_response={urgent_response_time:.1f}s, "
        "weighted_tardiness={weighted_tardiness:.1f}, "
        "platform_avg={inspection_platform_avg_residency_time:.1f}s, "
        "clearance_wait_avg={post_inspection_avg_clearance_wait:.1f}s".format(
            **summary
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a visual CoppeliaSim demo for the CR5 scheduler.")
    parser.add_argument("--urgent-type", choices=["A", "B", "C"], default="C")
    parser.add_argument("--insert-time", type=float, default=20.0)
    parser.add_argument("--speed", type=float, default=2.0, help="simulation seconds per real second")
    parser.add_argument("--orders-json", type=Path, help="orders exported from dynamic_order_window.py")
    parser.add_argument("--watch-orders", action="store_true", help="watch --orders-json for runtime updates")
    parser.add_argument("--preview-only", action="store_true", help="print the schedule without driving CoppeliaSim")
    parser.add_argument(
        "--coppeliasim-root",
        type=Path,
        help="CoppeliaSim installation root. Overrides COPPELIASIM_ROOT when connecting to the simulator.",
    )
    args = parser.parse_args()

    quality_overrides: dict[str, str] = {}
    scoring_weights: dict[str, float] = {}
    insert_time = args.insert_time
    if args.orders_json:
        orders, quality_overrides, insert_time, scoring_weights = load_orders_json(args.orders_json)
    else:
        orders = build_orders(args.urgent_type, args.insert_time)
    tasks, summary = build_demo_tasks(orders, quality_overrides, scoring_weights)
    print_schedule(tasks, summary)
    if args.preview_only:
        return

    display = CoppeliaSchedulerDisplay(args.coppeliasim_root)
    display.setup(orders)
    display.play(
        tasks,
        summary,
        speed=max(args.speed, 0.1),
        urgent_insert_time=insert_time,
        watch_orders_path=args.orders_json if args.watch_orders else None,
        quality_overrides=quality_overrides,
    )


if __name__ == "__main__":
    main()
