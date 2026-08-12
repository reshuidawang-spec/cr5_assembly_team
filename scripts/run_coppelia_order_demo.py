#!/usr/bin/env python3
"""Drive a coarse CoppeliaSim display from the CR5 scheduler.

This script is intentionally visual-first: it uses the existing scheduler to
produce a dynamic schedule, then follows the repository work-step point table
to move the five CR5 arm joints approximately and update scene stage signals.
It is not a full IK/trajectory controller, but it now follows pick/approach/
place/sort waypoint sequences from the project configs.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parents[1]

sys.path.insert(0, str(ROOT))

from interfaces.types import Order, Task, TaskStatus  # noqa: E402
from robot_control.motion_control import WorkstepMotionPlanner  # noqa: E402
from scheduler.config_loader import load_yaml  # noqa: E402
from scheduler.dynamic_order_sequence import (  # noqa: E402
    DynamicOrderInput,
    QualityOverrideExperiment,
    expand_order_inputs_as_units,
    expand_quality_overrides_with_policy,
    to_orders,
)
from scheduler.experiment import DiscreteEventExperiment, ScheduleRecord  # noqa: E402
from sim_bridge.scene_objects import PARTS  # noqa: E402


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
            os.environ.setdefault("COPPELIASIM_ROOT", str(root))
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
    "box_feed": ("cell_process_state", "assembly_shell"),
    "pcb_install": ("cell_process_state", "assembly_pcb"),
    "module_install": ("cell_process_state", "assembly_module"),
    "terminal_install": ("cell_process_state", "assembly_full"),
    "transfer_to_inspection": ("cell_process_state", "inspection_full"),
    "screw": ("cell_screw_state", "done"),
    "sort_good": ("cell_conveyor_state", "good"),
    "sort_defect": ("cell_conveyor_state", "defect"),
}

PROCESS_TO_SCENE_COMMAND = {
    "box_feed": "R1_BOX_PLACED",
    "pcb_install": "R2_PCB_PLACED",
    "terminal_install": "R1_TERMINAL_PLACED",
    "module_install": "R3_MODULE_PLACED",
    "transfer_to_inspection": "R3_PRODUCT_TO_INSPECTION",
    "screw": "R4_SCREW_DONE",
    "sort_good": "R5_SORT_GOOD_DONE",
    "sort_defect": "R5_SORT_DEFECT_DONE",
}

PROCESS_TO_TARGET_AREA = {
    "box_feed": "box_supply_area",
    "pcb_install": "pcb_supply_area",
    "terminal_install": "terminal_supply_area",
    "module_install": "module_supply_area",
    "transfer_to_inspection": "transfer_area",
    "inspect": "camera_area",
    "screw": "inspection_screw_area",
    "sort_good": "good_conveyor_area",
    "sort_defect": "defect_conveyor_area",
}

PROCESS_TO_TOOL_START_CMD = {
    "box_feed": "R1_ATTACH_BOX",
    "pcb_install": "R2_ATTACH_PCB",
    "module_install": "R3_ATTACH_MODULE",
    "terminal_install": "R1_ATTACH_TERMINAL",
    "transfer_to_inspection": "R3_ATTACH_ASSEMBLY_PRODUCT",
    "screw": "R4_SCREW_START",
    "sort_good": "R5_ATTACH_INSPECTION_PRODUCT",
    "sort_defect": "R5_ATTACH_INSPECTION_PRODUCT",
}

PROCESS_TO_TOOL_FINISH_CMD = {
    "box_feed": "R1_RELEASE_BOX_ASSEMBLY",
    "pcb_install": "R2_RELEASE_PCB_ASSEMBLY",
    "module_install": "R3_RELEASE_MODULE_ASSEMBLY",
    "terminal_install": "R1_RELEASE_TERMINAL_ASSEMBLY",
    "transfer_to_inspection": "R3_RELEASE_PRODUCT_INSPECTION",
    "screw": "R4_SCREW_STOP",
    "sort_good": "R5_RELEASE_GOOD",
    "sort_defect": "R5_RELEASE_DEFECT",
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
    "A": [0.62, 0.62, 0.62],  # library COLOR_BOX / A shell
    "B": [0.15, 0.20, 0.55],  # library CartB blue shell
    "C": [0.20, 0.55, 1.00],  # blue
    "URGENT": [1.00, 0.05, 0.05],  # red
    "COMPLETED": [0.55, 0.58, 0.62],  # grey
}

MATERIAL_CART_COLORS = {
    "A": [1.00, 0.50, 0.20],
    "B": [0.20, 0.60, 1.00],
}

B_VARIANT_COLOR = [0.95, 0.40, 0.10]

LIBRARY_PART_COLORS = {
    "A_BOX": [0.60, 0.60, 0.60],
    "A_PCB": [0.00, 0.45, 0.18],
    "A_MODULE": [0.15, 0.15, 0.20],
    "A_TERMINAL": [0.92, 0.82, 0.35],
    "B_BOX": [0.15, 0.20, 0.55],
    "B_BOX_ACCENT": [0.95, 0.40, 0.10],
    "B_BASE": [0.12, 0.12, 0.14],
    "B_PCB": [0.35, 0.05, 0.35],
    "B_MODULE": [0.95, 0.25, 0.15],
    "B_TERMINAL": [0.10, 0.70, 0.25],
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


def material_color_for(product_type: str) -> list[float]:
    """Physical part color: keep the material color identical to its cart."""
    return list(
        MATERIAL_CART_COLORS.get(
            product_type,
            PRODUCT_COLORS.get(product_type, [1.0, 1.0, 1.0]),
        )
    )


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


def quality_evaluation_from_tasks(
    tasks: list[DemoTask],
    summary: dict,
    defects_per_100: float | None = None,
) -> dict[str, float]:
    good_count = sum(1 for task in tasks if task.record.process == "sort_good")
    defect_count = sum(1 for task in tasks if task.record.process == "sort_defect")
    total = good_count + defect_count
    success_rate = (good_count / total * 100.0) if total else 0.0
    configured_defects = max(float(defects_per_100 or 0.0), 0.0)
    return {
        "total_products": float(total),
        "good_count": float(good_count),
        "defect_count": float(defect_count),
        "success_rate": success_rate,
        "configured_defects_per_100": configured_defects,
        "configured_good_rate": max(0.0, 100.0 - configured_defects),
        "makespan": float(summary.get("makespan", 0.0)),
        "conflict_count": float(summary.get("conflict_count", 0.0)),
        "weighted_tardiness": float(summary.get("weighted_tardiness", 0.0)),
        "post_inspection_avg_clearance_wait": float(summary.get("post_inspection_avg_clearance_wait", 0.0)),
    }


def write_evaluation_report(
    tasks: list[DemoTask],
    summary: dict,
    defects_per_100: float | None,
    path: Path | None = None,
) -> Path:
    report_path = path or (ROOT / "output" / "dynamic_order_evaluation.json")
    report_path.parent.mkdir(exist_ok=True)
    evaluation = quality_evaluation_from_tasks(tasks, summary, defects_per_100)
    type_counts: dict[str, int] = {}
    for task in tasks:
        if task.record.process in ("sort_good", "sort_defect"):
            type_counts[task.record.product_type] = type_counts.get(task.record.product_type, 0) + 1
    payload = {
        "generated_at": time.time(),
        "evaluation": evaluation,
        "product_type_counts": type_counts,
        "summary": summary,
    }
    report_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return report_path


def load_orders_json(path: Path) -> tuple[list[Order], dict[str, str], float, dict[str, float], dict]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    raw_orders = payload.get("orders", payload)
    raw_scoring = payload.get("scoring", {})
    quality_policy = payload.get("quality_policy", {}) if isinstance(payload, dict) else {}
    material_switch = payload.get("material_switch", {}) if isinstance(payload, dict) else {}
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
    defects_per_100 = quality_policy.get("defects_per_100")
    quality_overrides = expand_quality_overrides_with_policy(expanded_inputs, defects_per_100)
    urgent_arrivals = [order.arrival_time for order in orders if order.priority >= 5]
    urgent_insert_time = min(urgent_arrivals) if urgent_arrivals else 0.0
    scoring_weights = {
        str(key): float(value)
        for key, value in raw_scoring.items()
        if isinstance(value, (int, float))
    }
    demo_policy = {
        "quality_policy": quality_policy,
        "material_switch": material_switch,
    }
    return orders, quality_overrides, urgent_insert_time, scoring_weights, demo_policy


class CoppeliaSchedulerDisplay:
    def __init__(
        self,
        coppeliasim_root: Path | None = None,
        motion_mode: str = "library",
        speed: float = 2.0,
    ) -> None:
        self.motion_mode = motion_mode
        self.coppeliasim_root = coppeliasim_root
        self.library_speed = float(speed)
        self.bridge = None
        self.executor = None
        if motion_mode == "library":
            configure_coppelia_zmq_client(coppeliasim_root)
            from robot_control.coordinated_engine import CoordinatedEngine
            from sim_bridge.coppelia_client import SimBridge

            os.environ["CR5_SKIP_SCENE_FINGERPRINT"] = "1"
            os.environ["CR5_SKIP_R1_READY_PREMOTION"] = "1"
            os.environ.setdefault("CR5_SCENE_PATH", str(ROOT / "scenes" / "compact_cell.ttt"))
            self.bridge = SimBridge(
                validate_contract=False,
                request_timeout=180.0,
            )
            if not self.bridge.connect("127.0.0.1", 23000):
                first_error = self.bridge.last_error or "cannot connect to CoppeliaSim"
                self._load_library_scene(coppeliasim_root)
                self.bridge = SimBridge(
                    validate_contract=False,
                    request_timeout=180.0,
                )
                if not self.bridge.connect("127.0.0.1", 23000):
                    raise RuntimeError(self.bridge.last_error or first_error)
            try:
                self.bridge.set_stepping(False)
            except Exception:
                self.bridge.disconnect()
                self.bridge = SimBridge(validate_contract=False, request_timeout=180.0)
                if not self.bridge.connect("127.0.0.1", 23000):
                    raise RuntimeError(self.bridge.last_error or "cannot reconnect to CoppeliaSim")
            self.sim = self.bridge.sim
            self.client = None
            self.executor = CoordinatedEngine(bridge=self.bridge)
            try:
                self.bridge.set_visual_owner("executor")
            except Exception:
                pass
            self._ensure_demo_carts()
            self._align_good_conveyor_layout()
            self._align_camera_view_area()
        else:
            remote_api_client = load_remote_api_client(coppeliasim_root)
            self.client = remote_api_client(host="localhost", port=23000)
            self.sim = self.client.getObject("sim")
        self.points = load_yaml(ROOT / "configs" / "points.yaml")
        self.motion_planner = WorkstepMotionPlanner(ROOT)
        self.robot_joints = self._find_robot_joints()
        self.robot_markers: dict[str, int] = {}
        self.order_markers: dict[str, int] = {}
        self.material_cart_markers: dict[str, int] = {}
        self.product_variant_marker: int | None = None
        self.delivery_preview_marker: int | None = None
        self.current_material_type: str | None = None
        self.product_shell_handles = self._find_product_shell_handles()
        self.urgent_marker: int | None = None

    def _load_library_scene(self, coppeliasim_root: Path | None = None) -> None:
        scene_path = ROOT / "scenes" / "compact_cell.ttt"
        cache_name = scene_path.name
        if not scene_path.exists():
            raise RuntimeError(f"library scene is missing: {scene_path}")
        load_path = self._scene_path_for_coppeliasim(scene_path, cache_name=cache_name)
        os.environ["CR5_SCENE_PATH"] = str(load_path)
        remote_api_client = load_remote_api_client(coppeliasim_root)
        client = remote_api_client(host="127.0.0.1", port=23000)
        sim = client.require("sim") if hasattr(client, "require") else client.getObject("sim")
        try:
            if sim.getSimulationState() != sim.simulation_stopped:
                sim.stopSimulation()
                deadline = time.monotonic() + 10.0
                while sim.getSimulationState() != sim.simulation_stopped:
                    if time.monotonic() >= deadline:
                        raise RuntimeError("CoppeliaSim did not stop before loading scene")
                    time.sleep(0.2)
            sim.loadScene(str(load_path))
            time.sleep(3.0)
        finally:
            try:
                client.setStepping(False)
            except Exception:
                pass

    def _scene_path_for_coppeliasim(self, scene_path: Path, cache_name: str | None = None) -> Path:
        scene_text = str(scene_path)
        if cache_name is None:
            try:
                scene_text.encode("ascii")
                return scene_path
            except UnicodeEncodeError:
                pass
        cache_dir_env = os.environ.get("CR5_SCENE_CACHE_DIR")
        if cache_dir_env:
            cache_dir = Path(cache_dir_env)
        else:
            cache_dir = Path("/tmp/cr5_scene_cache")
        cache_dir.mkdir(parents=True, exist_ok=True)
        cached_scene = cache_dir / (cache_name or scene_path.name)
        if (
            not cached_scene.exists()
            or cached_scene.stat().st_size != scene_path.stat().st_size
            or cached_scene.stat().st_mtime < scene_path.stat().st_mtime
        ):
            shutil.copy2(scene_path, cached_scene)
        return cached_scene

    def _reconnect_library_executor(self) -> None:
        from robot_control.coordinated_engine import CoordinatedEngine
        from sim_bridge.coppelia_client import SimBridge

        try:
            if self.bridge is not None:
                self.bridge.disconnect()
        except Exception:
            pass
        self._load_library_scene(self.coppeliasim_root)
        self.bridge = SimBridge(
            validate_contract=False,
            request_timeout=180.0,
        )
        if not self.bridge.connect("127.0.0.1", 23000):
            raise RuntimeError(self.bridge.last_error or "cannot reconnect to CoppeliaSim")
        self.sim = self.bridge.sim
        self.executor = CoordinatedEngine(bridge=self.bridge)
        try:
            self.bridge.set_visual_owner("executor")
        except Exception:
            pass
        self._ensure_demo_carts()
        self._align_good_conveyor_layout()
        self._align_camera_view_area()

    def setup(self, orders: Iterable[Order]) -> None:
        self.cleanup()
        if self.motion_mode == "library":
            self._ensure_simulation_stopped()
            self.sim.setStringSignal("cell_product_state", "reset")
            self._ensure_demo_carts()
            self._align_good_conveyor_layout()
            self._align_camera_view_area()
            time.sleep(0.2)
            self._log(
                "SCHED-DEMO: library trajectory mode loaded; visual markers are disabled "
                "to avoid interfering with collision-checked robot paths."
            )
            return
        else:
            self._ensure_simulation_running()
        self.sim.setStringSignal("cell_product_state", "reset")
        time.sleep(0.2)
        self._make_resource_markers()
        self._make_material_cart_markers()
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
        defects_per_100: float | None = None,
        changeover_seconds: float = 3.0,
    ) -> None:
        if self.motion_mode == "library":
            self.play_library_motion(
                tasks,
                summary,
                speed,
                urgent_insert_time,
                defects_per_100,
                changeover_seconds,
            )
            return
        if not tasks:
            return
        start_wall = time.monotonic()
        self.changeover_seconds = max(float(changeover_seconds), 0.0)
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
                        orders, quality_overrides, urgent_insert_time, scoring_weights, demo_policy = load_orders_json(watch_orders_path)
                        tasks, summary = build_demo_tasks(orders, quality_overrides, scoring_weights)
                        defects_per_100 = demo_policy.get("quality_policy", {}).get("defects_per_100", defects_per_100)
                        changeover_seconds = float(
                            demo_policy.get("material_switch", {}).get("changeover_seconds", changeover_seconds)
                        )
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
        evaluation = quality_evaluation_from_tasks(tasks, summary, defects_per_100)
        self._log(
            "EVALUATION: total={total_products:.0f}, good={good_count:.0f}, defect={defect_count:.0f}, "
            "success_rate={success_rate:.1f}%, configured_defects_per_100={configured_defects_per_100:.1f}".format(
                **evaluation
            )
        )
        report_path = write_evaluation_report(tasks, summary, defects_per_100)
        self._log(f"EVALUATION REPORT: {report_path}")

    def play_library_motion(
        self,
        tasks: list[DemoTask],
        summary: dict,
        speed: float,
        urgent_insert_time: float,
        defects_per_100: float | None = None,
        changeover_seconds: float = 3.0,
    ) -> None:
        if not tasks:
            return
        if self.executor is None:
            raise RuntimeError("library motion executor is not initialized")
        self.changeover_seconds = max(float(changeover_seconds), 0.0)
        self._log(
            "COORDINATED MOTION MODE: using docs/五臂协同.md "
            "CoordinatedEngine + data/captured_paths replay"
        )
        os.environ["CR5_SKIP_SCENE_FINGERPRINT"] = "1"
        os.environ.setdefault("CR5_SCENE_PATH", str(ROOT / "scenes" / "compact_cell.ttt"))

        unit_groups = self._group_tasks_by_order_unit(tasks)
        urgent_announced = False
        for unit_index, unit_tasks in enumerate(unit_groups, start=1):
            first = unit_tasks[0]
            record = first.record
            if unit_index > 1:
                self._log(
                    f"COORDINATED CONTINUOUS NEXT UNIT: reusing current scene for {record.order_id}"
                )
                assert self.executor is not None
            if not urgent_announced and record.start_time >= urgent_insert_time and urgent_insert_time > 0:
                urgent_announced = True
                self._announce_urgent_order(urgent_insert_time)
            self._set_material_type(record.product_type, record.start_time, record.priority)
            self._sync_library_part_colors(record.product_type)
            marker = self.order_markers.get(record.order_id)
            if marker is not None:
                self._set_shape_color(marker, product_color_for(record.product_type, record.priority))
            self._log(
                f"COORDINATED UNIT {unit_index}/{len(unit_groups)}: {record.order_id} "
                f"type={record.product_type} quality={first.quality_result}"
            )
            self._set_product_shell_color_for_task(first)
            quality = "defect" if first.quality_result == "NG" else "good"
            self._log(
                f"COORDINATED EXEC START: {record.order_id} "
                f"run_cycle quality={quality}"
            )
            result = self.executor.run_cycle(
                quality=quality,
                start_from_wait=unit_index > 1,
                keep_running=unit_index < len(unit_groups),
                reuse_running=unit_index > 1,
                timeout_s=900,
            )
            if result.get("status") != "ok":
                message = str(result.get("message", "coordinated cycle failed"))
                self._log(f"COORDINATED EXEC FAILED: {message}")
                raise RuntimeError(message)
            signal_value = "camera_good" if first.quality_result == "OK" else "camera_defect"
            self.sim.setStringSignal("cell_quality_state", signal_value)
            self._log(
                f"COORDINATED EXEC DONE: {record.order_id}; "
                f"camera={first.quality_result}"
            )
            if first.quality_result == "NG":
                self._log(
                    "COORDINATED NOTE: current coordinated_front.py mainly "
                    "implements the GOOD visual route; NG is kept in schedule/KPI."
                )
            marker = self.order_markers.get(record.order_id)
            if marker is not None:
                self._set_shape_color(marker, product_color_for(record.product_type, record.priority))
            self._set_product_shell_color_for_task(first)
        self._log(
            "SCHED-DEMO finished with library trajectories: makespan={makespan:.1f}s, conflicts={conflicts}, urgent_completion={urgent:.1f}s".format(
                makespan=summary.get("makespan", 0.0),
                conflicts=summary.get("conflict_count", 0),
                urgent=summary.get("urgent_completion_time", 0.0),
            )
        )
        evaluation = quality_evaluation_from_tasks(tasks, summary, defects_per_100)
        self._log(
            "EVALUATION: total={total_products:.0f}, good={good_count:.0f}, defect={defect_count:.0f}, "
            "success_rate={success_rate:.1f}%, configured_defects_per_100={configured_defects_per_100:.1f}".format(
                **evaluation
            )
        )
        report_path = write_evaluation_report(tasks, summary, defects_per_100)
        self._log(f"EVALUATION REPORT: {report_path}")

    def _group_tasks_by_order_unit(self, tasks: list[DemoTask]) -> list[list[DemoTask]]:
        grouped: dict[str, list[DemoTask]] = {}
        for task in tasks:
            grouped.setdefault(task.record.order_id, []).append(task)
        process_order = {
            "box_feed": 0,
            "pcb_install": 1,
            "terminal_install": 2,
            "module_install": 3,
            "transfer_to_inspection": 4,
            "inspect": 5,
            "screw": 6,
            "sort_good": 7,
            "sort_defect": 7,
        }
        units = []
        for rows in grouped.values():
            rows.sort(key=lambda item: (process_order.get(item.record.process, 99), item.record.start_time))
            units.append(rows)
        units.sort(key=lambda rows: min(item.record.start_time for item in rows))
        return units

    def _to_executor_task(self, task: DemoTask, scene_command: str) -> Task:
        record = task.record
        return Task(
            task_id=record.task_id,
            order_id=record.order_id,
            product_type=record.product_type,
            process=record.process,
            target_area=PROCESS_TO_TARGET_AREA.get(record.process, record.process),
            target_point=task.point_name,
            available_robots=[record.robot_id],
            duration=max(record.end_time - record.start_time, 0.1),
            predecessors=[],
            priority=record.priority,
            status=TaskStatus.PENDING.value,
            required_areas=[],
            scene_command=scene_command,
        )

    def _ensure_simulation_running(self) -> None:
        state = self.sim.getSimulationState()
        if state == self.sim.simulation_stopped:
            self.sim.startSimulation()
            time.sleep(0.5)

    def _ensure_simulation_stopped(self) -> None:
        state = self.sim.getSimulationState()
        if state != self.sim.simulation_stopped:
            self.sim.stopSimulation()
            deadline = time.monotonic() + 10.0
            while self.sim.getSimulationState() != self.sim.simulation_stopped:
                if time.monotonic() >= deadline:
                    raise RuntimeError("CoppeliaSim did not stop before library motion preparation")
                time.sleep(0.1)

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
        color = material_color_for(task.record.product_type)
        for group in shell_groups_for_process(task.record.process):
            self._set_product_shell_color(group, color)

    def _set_shape_color(self, handle: int, color: list[float]) -> None:
        try:
            self.sim.setShapeColor(handle, "", self.sim.colorcomponent_ambient_diffuse, color)
        except Exception:
            pass

    def _sync_library_part_colors(self, product_type: str) -> None:
        """Force all real scene parts to match the active material cart color."""
        color = material_color_for(product_type)
        for part_key in (
            "BOX_BLANK",
            "PCB_SUPPLY",
            "CONTROL_MODULE_SUPPLY",
            "TERMINAL_BLOCK_SUPPLY",
            "INSPECTION_PRODUCT",
        ):
            try:
                handle = self.sim.getObject(PARTS[part_key])
                for shape in self.sim.getObjectsInTree(handle, self.sim.object_shape_type, 0):
                    self._set_shape_color(shape, color)
            except Exception:
                continue
        self._log(
            f"MATERIAL COLOR SYNC: {product_type} parts match cart color "
            f"{tuple(round(value, 2) for value in color)}"
        )

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

    def _make_material_cart_markers(self) -> None:
        base = self._point("R1_BOX_PICK_TCP")
        positions = {
            "A": [base[0] - 0.35, base[1] + 0.35, base[2] + 0.12],
            "B": [base[0] - 0.35, base[1] + 0.65, base[2] + 0.12],
        }
        for product_type, position in positions.items():
            cart = self._make_box(
                f"sched_demo_material_cart_{product_type}",
                position,
                [0.28, 0.18, 0.10],
                MATERIAL_CART_COLORS[product_type],
            )
            self.material_cart_markers[product_type] = cart
        self.product_variant_marker = self._make_box(
            "sched_demo_B_variant_badge",
            [base[0], base[1], base[2] - 1.0],
            [0.16, 0.035, 0.035],
            B_VARIANT_COLOR,
        )

    def _set_material_type(self, product_type: str, schedule_time: float, priority: int = 1) -> None:
        if product_type not in MATERIAL_CART_COLORS:
            return
        previous = self.current_material_type
        self.current_material_type = product_type
        base = self._point("R1_BOX_PICK_TCP")
        product_signal = f"product_{product_type.lower()}"
        cart_signal = f"cart_{product_type.lower()}_supply"
        self.sim.setStringSignal("cell_material_state", f"changeover_{product_type}")
        self.sim.setStringSignal("cell_event_state", f"material_switching_{product_type}")
        self.sim.setStringSignal("cart_order", cart_signal)
        if previous is None:
            self._log(f"MATERIAL READY t={schedule_time:.1f}: {product_type} material cart is active")
        elif previous != product_type:
            self._log(
                f"MATERIAL CHANGEOVER t={schedule_time:.1f}: {previous} -> {product_type}; "
                f"waiting {self.changeover_seconds:.1f}s"
            )
        else:
            self._log(f"MATERIAL READY t={schedule_time:.1f}: {product_type} material cart remains active")
        self._position_scene_carts(
            product_type,
            animate=bool(previous is None or previous != product_type),
        )
        for cart_type, marker in self.material_cart_markers.items():
            try:
                if cart_type == product_type:
                    position = [base[0] - 0.12, base[1] + 0.30, base[2] + 0.16]
                else:
                    position = [base[0] - 0.45, base[1] + (0.35 if cart_type == "A" else 0.65), base[2] + 0.12]
                self.sim.setObjectPosition(marker, -1, position)
            except Exception:
                pass
        if previous is not None and previous != product_type and self.changeover_seconds > 0:
            time.sleep(self.changeover_seconds)
        self._animate_material_delivery(product_type, priority)
        self.sim.setStringSignal("cell_material_state", f"ready_{product_type}")
        self.sim.setStringSignal("cell_product_state", product_signal)
        self.sim.setStringSignal("cart_order", cart_signal)
        self._log(f"MATERIAL SIGNAL {product_signal}, {cart_signal}; delivered to feed position")

    def _animate_material_delivery(self, product_type: str, priority: int = 1) -> None:
        """Show a simple cart-to-feed delivery before robot assembly starts."""
        # Match coordinated_front.py's actual BOX_BLANK reset/grasp supply
        # pose so the cart-delivered preview and R1's real pick object read as
        # one continuous handoff.
        approach_end = [-1.86, 0.22, 0.30]
        real_box_position = [-1.86, 0.22, 0.156]
        cart_name = {"A": "CartA", "B": "CartB"}.get(product_type)
        start = [approach_end[0] - 0.45, approach_end[1] - 0.25, approach_end[2] + 0.05]
        if cart_name:
            try:
                cart = self.sim.getObject(f"/{cart_name}")
                cart_pos = self.sim.getObjectPosition(cart, -1)
                start = [cart_pos[0], cart_pos[1], cart_pos[2] + 0.24]
            except Exception:
                pass
        color = material_color_for(product_type)
        if self.delivery_preview_marker is None:
            self.delivery_preview_marker = self._safe_get_object(
                "/sched_demo_delivered_material_preview"
            )
        if self.delivery_preview_marker is None:
            self.delivery_preview_marker = self._make_box(
                "sched_demo_delivered_material_preview",
                start,
                [0.16, 0.11, 0.06],
                color,
            )
        else:
            self.sim.setObjectPosition(self.delivery_preview_marker, -1, start)
            self._set_shape_color(self.delivery_preview_marker, color)
            try:
                self.sim.setObjectInt32Param(
                    self.delivery_preview_marker,
                    self.sim.objintparam_visibility_layer,
                    1,
                )
            except Exception:
                pass
        steps = 16
        for index in range(1, steps + 1):
            ratio = index / steps
            position = [
                start[axis] + (approach_end[axis] - start[axis]) * ratio
                for axis in range(3)
            ]
            try:
                self.sim.setObjectPosition(self.delivery_preview_marker, -1, position)
            except Exception:
                pass
            time.sleep(0.025)
        for index in range(1, 9):
            ratio = index / 8
            position = [
                approach_end[axis]
                + (real_box_position[axis] - approach_end[axis]) * ratio
                for axis in range(3)
            ]
            try:
                self.sim.setObjectPosition(self.delivery_preview_marker, -1, position)
            except Exception:
                pass
            time.sleep(0.02)
        self._handoff_delivered_material_to_real_box(
            product_type,
            real_box_position,
            priority,
        )
        self._log(
            f"MATERIAL DELIVERY: {product_type} cart -> feed position "
            f"{tuple(round(value, 3) for value in real_box_position)}"
        )

    def _handoff_delivered_material_to_real_box(
        self,
        product_type: str,
        box_position: list[float],
        priority: int = 1,
    ) -> None:
        """Turn the cart preview into the actual object R1 will pick."""
        color = material_color_for(product_type)
        try:
            box = self.sim.getObject("/FiveCR5A_Cell/Parts/Box_Blank")
            self.sim.setObjectPosition(box, -1, box_position)
            self.sim.setObjectQuaternion(box, -1, [1, 0, 0, 0])
            for shape in self.sim.getObjectsInTree(box, self.sim.object_shape_type, 0):
                self.sim.setObjectInt32Param(shape, self.sim.objintparam_visibility_layer, 1)
                self._set_shape_color(shape, color)
        except Exception as exc:
            self._log(f"MATERIAL HANDOFF WARNING: BOX_BLANK not synced: {exc}")
        if self.delivery_preview_marker is not None:
            try:
                self.sim.setObjectInt32Param(
                    self.delivery_preview_marker,
                    self.sim.objintparam_visibility_layer,
                    0,
                )
                self.sim.setObjectPosition(self.delivery_preview_marker, -1, [3.0, 3.0, 0.5])
            except Exception:
                pass

    def _position_scene_carts(self, product_type: str, animate: bool = False) -> None:
        self._ensure_demo_carts()
        targets = {
            "A": {
                "CartA": "/CartA_SupplyPose",
                "CartB": "/CartB_WaitPose",
            },
            "B": {
                "CartA": "/CartA_WaitPose",
                "CartB": "/CartB_SupplyPose",
            },
        }.get(product_type)
        if not targets:
            return
        cart_handles: dict[str, int] = {}
        target_positions: dict[str, list[float]] = {}
        start_positions: dict[str, list[float]] = {}
        for cart_name, target_path in targets.items():
            try:
                cart = self.sim.getObject(f"/{cart_name}")
                target = self.sim.getObject(target_path)
                cart_handles[cart_name] = cart
                target_positions[cart_name] = [
                    float(value) for value in self.sim.getObjectPosition(target, -1)
                ]
                start_positions[cart_name] = [
                    float(value) for value in self.sim.getObjectPosition(cart, -1)
                ]
            except Exception:
                return
        if animate and self.changeover_seconds > 0:
            steps = max(1, int(min(30, max(6, self.changeover_seconds / 0.05))))
            for index in range(1, steps + 1):
                ratio = index / steps
                for cart_name, cart in cart_handles.items():
                    start = start_positions[cart_name]
                    target = target_positions[cart_name]
                    pos = [
                        start[axis] + (target[axis] - start[axis]) * ratio
                        for axis in range(3)
                    ]
                    try:
                        self.sim.setObjectPosition(cart, -1, pos)
                    except Exception:
                        pass
                time.sleep(self.changeover_seconds / steps)
        else:
            for cart_name, cart in cart_handles.items():
                try:
                    self.sim.setObjectPosition(cart, -1, target_positions[cart_name])
                except Exception:
                    pass
        self._log(
            "CART POSITION: "
            + ", ".join(
                f"{name}={tuple(round(value, 3) for value in target_positions[name])}"
                for name in sorted(target_positions)
            )
        )

    def _ensure_demo_carts(self) -> None:
        cart_targets = {
            "CartA_SupplyPose": [-2.30, -0.90, 0.05],
            "CartA_WaitPose": [-2.30, -1.55, 0.05],
            "CartB_SupplyPose": [-1.80, -0.90, 0.05],
            "CartB_WaitPose": [-1.80, -1.55, 0.05],
        }
        cart_initial = {
            "CartA": cart_targets["CartA_WaitPose"],
            "CartB": cart_targets["CartB_WaitPose"],
        }
        for name, position in cart_targets.items():
            if self._safe_get_object(f"/{name}") is not None:
                continue
            try:
                dummy = self.sim.createDummy(0.035)
                self.sim.setObjectAlias(dummy, name)
                self.sim.setObjectPosition(dummy, -1, position)
            except Exception:
                pass
        for cart_name, position in cart_initial.items():
            if self._safe_get_object(f"/{cart_name}") is not None:
                continue
            color = MATERIAL_CART_COLORS["A" if cart_name == "CartA" else "B"]
            try:
                cart = self._make_box(
                    cart_name,
                    position,
                    [0.38, 0.24, 0.10],
                    color,
                )
                self._make_library_cart_cargo(
                    cart,
                    cart_name,
                    "A" if cart_name == "CartA" else "B",
                    position,
                )
            except Exception:
                pass

    def _make_library_cart_cargo(
        self,
        cart_handle: int,
        cart_name: str,
        product_type: str,
        cart_position: list[float],
    ) -> None:
        def add(local_pos: list[float], size: list[float], color: list[float], suffix: str) -> None:
            world = [
                cart_position[0] + local_pos[0],
                cart_position[1] + local_pos[1],
                cart_position[2] + local_pos[2],
            ]
            part = self._make_box(f"{cart_name}_{suffix}", world, size, color)
            self.sim.setObjectParent(part, cart_handle, True)

        if product_type == "A":
            add([0.0, 0.0, 0.100], [0.21, 0.15, 0.072], LIBRARY_PART_COLORS["A_BOX"], "A_Box")
            add([0.0, -0.072, 0.110], [0.19, 0.006, 0.050], LIBRARY_PART_COLORS["A_BOX"], "A_Box_Front")
            add([0.0, 0.020, 0.142], [0.14, 0.09, 0.006], LIBRARY_PART_COLORS["A_PCB"], "A_PCB")
            add([-0.020, 0.020, 0.148], [0.04, 0.04, 0.010], [0.10, 0.10, 0.10], "A_Chip")
            add([0.025, 0.020, 0.150], [0.05, 0.04, 0.020], LIBRARY_PART_COLORS["A_MODULE"], "A_Module")
            add([0.045, -0.030, 0.140], [0.10, 0.020, 0.020], LIBRARY_PART_COLORS["A_TERMINAL"], "A_Terminal")
            return

        add([0.0, 0.0, 0.100], [0.21, 0.15, 0.072], LIBRARY_PART_COLORS["B_BOX"], "B_Box")
        add([0.0, -0.072, 0.110], [0.19, 0.006, 0.050], LIBRARY_PART_COLORS["B_BOX_ACCENT"], "B_Box_Front")
        add([0.0, 0.0, 0.062], [0.23, 0.17, 0.006], LIBRARY_PART_COLORS["B_BASE"], "B_Base")
        for index, offset in enumerate(([-0.075, -0.045], [0.075, -0.045], [-0.075, 0.045], [0.075, 0.045]), start=1):
            add([offset[0], offset[1], 0.060], [0.016, 0.016, 0.025], LIBRARY_PART_COLORS["B_BOX_ACCENT"], f"B_Foot_{index}")
        add([0.0, 0.020, 0.142], [0.14, 0.09, 0.006], LIBRARY_PART_COLORS["B_PCB"], "B_PCB")
        add([-0.020, 0.020, 0.148], [0.04, 0.04, 0.010], [0.85, 0.85, 0.90], "B_Chip")
        add([0.025, 0.020, 0.150], [0.05, 0.04, 0.020], LIBRARY_PART_COLORS["B_MODULE"], "B_Module")
        add([0.027, 0.020, 0.160], [0.05, 0.02, 0.010], [1.0, 1.0, 1.0], "B_Module_Label")
        add([0.045, -0.030, 0.140], [0.10, 0.020, 0.020], LIBRARY_PART_COLORS["B_TERMINAL"], "B_Terminal")
        for index, y in enumerate((-0.04, 0.0, 0.04), start=1):
            add([-0.107, y, 0.100], [0.004, 0.015, 0.055], [0.08, 0.12, 0.45], f"B_Left_Stripe_{index}")
            add([0.107, y, 0.100], [0.004, 0.015, 0.055], [0.08, 0.12, 0.45], f"B_Right_Stripe_{index}")

    def _safe_get_object(self, path: str) -> int | None:
        try:
            return int(self.sim.getObject(path))
        except Exception:
            return None

    def _align_camera_view_area(self) -> None:
        try:
            station = self.sim.getObject(
                "/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station"
            )
            view = self.sim.getObject(
                "/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area"
            )
            body = self.sim.getObject(
                "/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Fixed_Camera_Body"
            )
            self.sim.setObjectPosition(station, -1, [1.80, 1.50, 0.0])
            self.sim.setObjectPosition(view, -1, [1.95, 1.55, 0.218])
            self.sim.setObjectPosition(body, -1, [1.95, 1.65, 0.82])
        except Exception:
            pass

    def _align_good_conveyor_layout(self) -> None:
        """Keep the loaded scene at the AGV-guide GOOD conveyor layout.

        The cached demonstration scene can still contain the old good
        conveyor position.  Align it immediately after scene load/connection
        so the user sees the modified layout before the first order starts,
        and so CoppeliaSim's stop/reset cycle does not visually jump back to
        an old conveyor coordinate.
        """
        try:
            conveyor = self.sim.getObject(
                "/FiveCR5A_Cell/Conveyors/Good_Conveyor"
            )
            conveyor_pos = list(self.sim.getObjectPosition(conveyor, -1))
            conveyor_pos[0] = 0.48
            self.sim.setObjectPosition(conveyor, -1, conveyor_pos)
            for target_name in ("R5_GOOD_PLACE_APP", "R5_GOOD_PLACE_TCP"):
                target = self.sim.getObject(
                    f"/FiveCR5A_Cell/Targets/R5_Targets/{target_name}"
                )
                target_pos = list(self.sim.getObjectPosition(target, -1))
                target_pos[0] = 0.35
                self.sim.setObjectPosition(target, -1, target_pos)
            belt = self.sim.getObject(
                "/FiveCR5A_Cell/Conveyors/Good_Conveyor/"
                "Good_Conveyor_Belt_Black"
            )
            belt_pos = self.sim.getObjectPosition(belt, -1)
            self._log(
                "SCENE ALIGNMENT: Good_Conveyor_Belt X={:.2f}, "
                "R5_GOOD_PLACE X=0.35".format(float(belt_pos[0]))
            )
        except Exception as exc:
            self._log(f"SCENE ALIGNMENT WARNING: good conveyor not aligned: {exc}")

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
        try:
            self.sim.setObjectInt32Param(handle, self.sim.shapeintparam_static, 1)
            self.sim.setObjectInt32Param(handle, self.sim.shapeintparam_respondable, 0)
        except Exception:
            pass
        return handle

    def _on_task_start(self, task: DemoTask) -> None:
        record = task.record
        if record.process == "box_feed":
            self._set_material_type(record.product_type, record.start_time, record.priority)
        self._set_product_shell_color_for_task(task)
        marker = self.order_markers.get(record.order_id)
        if marker is not None:
            self._set_shape_color(marker, product_color_for(record.product_type, record.priority))
        self._log(
            f"START t={record.start_time:.1f}: {record.order_id} {record.product_type} "
            f"{record.process} -> {record.robot_id}"
        )
        step_labels = [
            step.label
            for step in self.motion_planner.worksteps.get(record.process, [])
        ]
        if step_labels:
            self._log(f"MOTION PLAN {record.process}: " + " -> ".join(step_labels))
        tool_cmd = PROCESS_TO_TOOL_START_CMD.get(record.process)
        if tool_cmd:
            self._send_tool_command(tool_cmd)
        self.sim.setStringSignal(f"{record.robot_id.lower()}_ros_cmd", record.process)

    def _on_task_finish(self, task: DemoTask) -> None:
        record = task.record
        tool_cmd = PROCESS_TO_TOOL_FINISH_CMD.get(record.process)
        if tool_cmd:
            self._send_tool_command(tool_cmd)
        if record.process == "inspect":
            signal_value = "camera_good" if task.quality_result == "OK" else "camera_defect"
            self.sim.setStringSignal("cell_quality_state", signal_value)
        elif record.process in PROCESS_TO_STAGE_SIGNAL:
            signal_name, signal_value = PROCESS_TO_STAGE_SIGNAL[record.process]
            self.sim.setStringSignal(signal_name, signal_value)
        self._log(
            f"FINISH t={record.end_time:.1f}: {record.order_id} {record.process} "
            f"({task.quality_result if record.process == 'inspect' else 'done'})"
        )
        if record.process in ("sort_good", "sort_defect"):
            marker = self.order_markers.get(record.order_id)
            if marker is not None:
                self._set_shape_color(marker, product_color_for(record.product_type, record.priority))
            self._set_product_shell_color_for_task(task)

    def _show_active_task(self, robot_id: str, task: DemoTask, ratio: float) -> None:
        frame = self.motion_planner.frame_for(task.record.process, ratio, task.point_name)
        target = list(frame.position)
        target[2] += 0.48
        if robot_id in self.robot_markers:
            pulse = 0.04 * math.sin(ratio * math.pi * 6)
            self.sim.setObjectPosition(self.robot_markers[robot_id], -1, [target[0], target[1], target[2] + pulse])
        if robot_id != "CAMERA":
            self._pose_robot(robot_id, task.record.process, ratio)
        marker = self.order_markers.get(task.record.order_id)
        if marker is not None:
            self.sim.setObjectPosition(marker, -1, [target[0], target[1], target[2] + 0.12])
        self._show_product_variant(task.record.product_type, [target[0], target[1], target[2] + 0.22])

    def _show_idle_resource(self, robot_id: str) -> None:
        if robot_id in self.robot_markers:
            position = self._point(ROBOT_HOME_POINTS[robot_id])
            position[2] += 0.35
            self.sim.setObjectPosition(self.robot_markers[robot_id], -1, position)
        if robot_id != "CAMERA":
            self._pose_robot(robot_id, "", 0.0)

    def _show_product_variant(self, product_type: str, position: list[float]) -> None:
        if self.product_variant_marker is None:
            return
        try:
            if product_type == "B":
                self.sim.setObjectPosition(self.product_variant_marker, -1, position)
            else:
                self.sim.setObjectPosition(self.product_variant_marker, -1, [0.0, 0.0, -1.0])
        except Exception:
            pass

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

    def _send_tool_command(self, command: str) -> None:
        self.sim.setStringSignal("tool_cmd", command)
        self._log(f"TOOL_CMD {command}")

    def _announce_urgent_order(self, insert_time: float) -> None:
        if self.urgent_marker is not None:
            self.sim.setObjectPosition(self.urgent_marker, -1, [1.28, 1.08, 0.80])
        self.sim.setStringSignal("cell_event_state", "urgent_inserted")
        self._log(f"URGENT INSERT t={insert_time:.1f}: priority=5 order enters the queue")

    def _point(self, point_name: str) -> list[float]:
        return list(self.points.get(point_name, {}).get("position", [0.0, 0.0, 0.6]))

    def _log(self, message: str) -> None:
        safe_print(message)
        try:
            self.sim.addLog(self.sim.verbosity_scriptinfos, message)
        except Exception:
            pass


def print_schedule(tasks: list[DemoTask], summary: dict, defects_per_100: float | None = None) -> None:
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
    evaluation = quality_evaluation_from_tasks(tasks, summary, defects_per_100)
    safe_print(
        "Evaluation: total={total_products:.0f}, good={good_count:.0f}, defect={defect_count:.0f}, "
        "success_rate={success_rate:.1f}%, configured_defects_per_100={configured_defects_per_100:.1f}".format(
            **evaluation
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
        "--motion-mode",
        choices=["library", "markers"],
        default="library",
        help="library uses robot_control/plans trajectories; markers uses the old lightweight visual playback",
    )
    parser.add_argument(
        "--coppeliasim-root",
        type=Path,
        help="CoppeliaSim installation root. Overrides COPPELIASIM_ROOT when connecting to the simulator.",
    )
    args = parser.parse_args()

    quality_overrides: dict[str, str] = {}
    scoring_weights: dict[str, float] = {}
    demo_policy: dict = {"quality_policy": {}, "material_switch": {}}
    insert_time = args.insert_time
    if args.orders_json:
        orders, quality_overrides, insert_time, scoring_weights, demo_policy = load_orders_json(args.orders_json)
    else:
        orders = build_orders(args.urgent_type, args.insert_time)
    defects_per_100 = demo_policy.get("quality_policy", {}).get("defects_per_100")
    changeover_seconds = float(demo_policy.get("material_switch", {}).get("changeover_seconds", 3.0))
    tasks, summary = build_demo_tasks(orders, quality_overrides, scoring_weights)
    print_schedule(tasks, summary, defects_per_100)
    write_evaluation_report(tasks, summary, defects_per_100)
    if args.preview_only:
        return

    display = CoppeliaSchedulerDisplay(
        args.coppeliasim_root,
        motion_mode=args.motion_mode,
        speed=max(args.speed, 0.1),
    )
    display.setup(orders)
    display.play(
        tasks,
        summary,
        speed=max(args.speed, 0.1),
        urgent_insert_time=insert_time,
        watch_orders_path=args.orders_json if args.watch_orders else None,
        quality_overrides=quality_overrides,
        defects_per_100=defects_per_100,
        changeover_seconds=changeover_seconds,
    )


if __name__ == "__main__":
    main()
