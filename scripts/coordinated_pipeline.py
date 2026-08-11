#!/usr/bin/env python3
"""Run multiple orders as an overlapped five-arm CoppeliaSim pipeline.

R1/R2/R3 build the next unit as soon as the assembly fixture is clear while
R4/R5 finish the previous unit at the inspection/sorting station.  A fresh set
of visible supply parts is presented for every unit and each finished product
uses its own cloned scene object.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_control.runtime_cartesian import find_unique_alias
from scripts.coordinated_front import Arm
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import ROBOT_TIPS


SUPPLY_POSES = {
    "box": ([-1.86, 0.22, 0.156], [1, 0, 0, 0]),
    "term": ([-1.82, -0.02, 0.1665], [1, 0, 0, 0]),
    "pcb": ([-1.22, -0.42, 0.1584], [1, 0, 0, 0]),
    "module": ([-0.78, -0.20, 0.1665], [1, 0, 0, 0]),
}
B_CABINET_RED = [0.85, 0.05, 0.05]
B_PCB_BOARD = [0.35, 0.05, 0.35]
B_PCB_CHIP = [0.85, 0.85, 0.90]
B_PCB_HOLE = [1.0, 0.75, 0.15]
B_PCB_CONNECTOR = [1.0, 0.90, 0.10]
B_MODULE_BODY = [0.95, 0.25, 0.15]
B_MODULE_LABEL = [1.0, 1.0, 1.0]
B_TERMINAL_BODY = [0.10, 0.70, 0.25]
B_TERMINAL_SLOT = [0.02, 0.15, 0.05]
B_TERMINAL_SCREW = [0.90, 0.90, 0.92]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orders", type=int, default=1)
    parser.add_argument(
        "--defect-order",
        type=int,
        default=0,
        help="1-based A unit index routed to the defect conveyor; 0 means all good",
    )
    parser.add_argument(
        "--urgent-file",
        type=Path,
        help="JSON-lines control file used for one live B-type urgent unit",
    )
    return parser.parse_args()


class UrgentOrderReader:
    """Read append-only urgent requests without blocking the motion loop."""

    def __init__(self, path: Path | None):
        self.path = path
        self.offset = 0

    def poll(self) -> list[dict[str, str]]:
        if self.path is None or not self.path.exists():
            return []
        with self.path.open("r", encoding="utf-8") as stream:
            stream.seek(self.offset)
            lines = stream.readlines()
            self.offset = stream.tell()
        requests = []
        for line in lines:
            if not line.strip():
                continue
            value = json.loads(line)
            requests.append(
                {
                    "order_id": str(value["order_id"]),
                    "product_type": str(value["product_type"]).upper(),
                }
            )
        return requests


def set_visible(sim, root: int, visible: bool) -> None:
    layer = 1 if visible else 0
    for shape in sim.getObjectsInTree(root, sim.object_shape_type, 0):
        sim.setObjectInt32Param(shape, sim.objintparam_visibility_layer, layer)


def hide_teaching_targets(sim) -> int:
    root = sim.getObject("/FiveCR5A_Cell/Targets")
    objects = list(sim.getObjectsInTree(root, sim.handle_all, 0))
    for handle in objects:
        sim.setObjectInt32Param(
            handle,
            sim.objintparam_visibility_layer,
            0,
        )
    return len(objects)


def set_tree_color(sim, root: int, color: list[float]) -> None:
    for shape in sim.getObjectsInTree(root, sim.object_shape_type, 0):
        sim.setShapeColor(
            shape,
            None,
            sim.colorcomponent_ambient_diffuse,
            color,
        )


def shape_minimum_z(sim, shape: int) -> float:
    return float(sim.getObjectPosition(shape, -1)[2]) + float(
        sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_z)
    )


def shape_maximum_z(sim, shape: int) -> float:
    return float(sim.getObjectPosition(shape, -1)[2]) + float(
        sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_z)
    )


def tree_minimum_z(sim, root: int) -> float:
    shapes = list(sim.getObjectsInTree(root, sim.object_shape_type, 0))
    if not shapes:
        raise RuntimeError(f"对象 {root} 下没有可测量形状")
    return min(shape_minimum_z(sim, shape) for shape in shapes)


def cabinet_opening_is_up(sim, box: int) -> tuple[bool, float, float]:
    """Return whether the cabinet walls extend upward from its bottom plate."""
    shapes = list(sim.getObjectsInTree(box, sim.object_shape_type, 0))
    bottom_shapes = [
        shape
        for shape in shapes
        if str(sim.getObjectAlias(shape)).endswith("_Bottom")
    ]
    wall_shapes = [
        shape
        for shape in shapes
        if "_Wall" in str(sim.getObjectAlias(shape))
    ]
    if not bottom_shapes or not wall_shapes:
        raise RuntimeError("B型箱体缺少底板或侧壁，无法验证开口方向")
    bottom_center_z = sum(
        float(sim.getObjectPosition(shape, -1)[2]) for shape in bottom_shapes
    ) / len(bottom_shapes)
    wall_center_z = sum(
        float(sim.getObjectPosition(shape, -1)[2]) for shape in wall_shapes
    ) / len(wall_shapes)
    return wall_center_z > bottom_center_z, bottom_center_z, wall_center_z


def _b_assembled_shape_color(alias: str) -> list[float] | None:
    """Return the saved B-supply appearance for an assembled-product shape."""
    if "_Shell" in alias:
        return B_CABINET_RED
    if "_PCB" in alias:
        if "Chip" in alias:
            return B_PCB_CHIP
        if "Hole" in alias:
            return B_PCB_HOLE
        if "Connector" in alias:
            return B_PCB_CONNECTOR
        if "Board" in alias:
            return B_PCB_BOARD
    if "Control_Module" in alias:
        if "Label" in alias:
            return B_MODULE_LABEL
        if "Body" in alias:
            return B_MODULE_BODY
    if "Terminal_Block" in alias:
        if "Slot" in alias:
            return B_TERMINAL_SLOT
        if "Screw" in alias:
            return B_TERMINAL_SCREW
        if "Body" in alias:
            return B_TERMINAL_BODY
    return None


def recolor_b_cabinet(sim, box_b: int, assembled_products: list[int]) -> None:
    """Keep the loose and preassembled B visuals consistent at handoff."""
    set_tree_color(sim, box_b, B_CABINET_RED)
    for product in assembled_products:
        for handle in sim.getObjectsInTree(product, sim.handle_all, 0):
            alias = str(sim.getObjectAlias(handle))
            color = _b_assembled_shape_color(alias)
            if color is not None:
                try:
                    sim.setShapeColor(
                        handle,
                        None,
                        sim.colorcomponent_ambient_diffuse,
                        color,
                    )
                except Exception:
                    # Component roots are dummies; their descendant shapes
                    # receive their own matching color in the same traversal.
                    pass


def attach_handle(bridge: SimBridge, sim, child: int, robot_id: str) -> None:
    robot = bridge.get_object_handle(robot_id)
    tip = find_unique_alias(sim, robot, ROBOT_TIPS[robot_id])
    pose = list(sim.getObjectPose(child, -1))
    sim.setObjectParent(child, tip, True)
    sim.setObjectPose(child, -1, pose)


def clone_product(sim, source: int, parent: int, unit_number: int) -> int:
    source_tree = sim.getObjectsInTree(source, sim.handle_all, 0)
    copies = list(sim.copyPasteObjects(source_tree, 0))
    copied = set(copies)
    roots = [handle for handle in copies if sim.getObjectParent(handle) not in copied]
    if len(roots) != 1:
        raise RuntimeError(
            f"cannot identify cloned product root for unit {unit_number}: {roots}"
        )
    root = int(roots[0])
    sim.setObjectAlias(root, f"Pipeline_Product_{unit_number:03d}")
    sim.setObjectParent(root, parent, True)
    return root


def front_sequences(first: bool) -> dict[str, list[tuple[str, int]]]:
    r1 = [] if not first else [("r1_initial_to_box_pick_app", 1)]
    r1 += [
        ("r1_box_descend", 1),
        ("r1_box_grasp", 1),
        ("r1_box_grasp", -1),
        ("r1_box_lift_to_mid2", 1),
        ("r1_mid2_to_mid1", 1),
        ("r1_mid1_to_place_app", 1),
        ("r1_box_place_descend", 1),
        ("r1_box_place_descend", -1),
        ("r1_box_to_term_transition", 1),
        ("r1_mid1_to_mid2", 1),
        ("r1_mid2_to_pick_app", 1),
        ("r1_terminal_descend", 1),
        ("r1_terminal_descend", -1),
        ("r1_terminal_mid_transfer", 1),
        ("r1_terminal_mid_to_place_app", 1),
        ("r1_terminal_place_descend", 1),
        ("r1_terminal_place_descend", -1),
        ("r1_return_home", 1),
    ]
    r2 = [
        ("r2_initial_to_pick_app", 1)
        if first
        else ("r2_safe_wait_to_pick_app", 1),
        ("r2_pick_descend", 1),
        ("r2_pick_to_safe_wait", 1),
        ("r2_safe_wait_to_place_app", 1),
        ("r2_place_descend", 1),
        ("r2_place_descend", -1),
        ("r2_place_to_safe_wait", 1),
    ]
    r3 = [] if not first else [("r3_initial_to_module_pick_app", 1)]
    r3 += [
        ("r3_module_pick_descend", 1),
        ("r3_module_lift_transfer", 1),
        ("r3_module_place_descend", 1),
        ("r3_module_place_descend", -1),
        ("r3_module_to_product_pick_app", 1),
        ("r3_product_pick_descend", 1),
        ("r3_product_pick_descend", -1),
        ("r3_product_transfer", 1),
        ("r3_product_place_descend", 1),
        ("r3_product_place_descend", -1),
        ("r3_place_to_module_pick_app", 1),
    ]
    return {"R1": r1, "R2": r2, "R3": r3}


def back_sequences(defect: bool = False) -> dict[str, list[tuple[str, int]]]:
    r5_transfer = (
        [
            ("pick_to_defect_high", 1),
            ("defect_high_to_place_final", 1),
            ("defect_place_to_wait_new", 1),
        ]
        if defect
        else [
            ("pick_to_good_app_avoid_r4wait", 1),
            ("good_app_to_place_zfixed2", 1),
            ("good_place_to_wait_new", 1),
        ]
    )
    return {
        "R4": [
            ("r4_wait_to_app", 1),
            ("r4_app_to_tcp", 1),
            ("r4_tcp_to_press", 1),
            ("r4_press_to_app", 1),
            ("r4_app_to_wait", 1),
        ],
        "R5": [
            ("r5_wait_to_pick_app", 1),
            ("r5_pick_descend", 1),
            *r5_transfer,
        ],
    }


def main() -> int:
    args = parse_args()
    if args.orders < 1 or args.orders > 20:
        raise ValueError("--orders must be between 1 and 20")
    if args.defect_order < 0 or args.defect_order > args.orders:
        raise ValueError("--defect-order must be 0 or a valid A unit index")

    bridge = SimBridge(request_timeout=20.0)
    for _ in range(10):
        if bridge.connect():
            break
        time.sleep(2)
    if not bridge.is_connected():
        raise RuntimeError(bridge.last_error or "CoppeliaSim connection failed")

    sim = bridge.sim
    template_states: dict[int, tuple[list[float], dict[int, int]]] = {}
    created_products: list[int] = []
    try:
        if sim.getSimulationState() != sim.simulation_stopped:
            if not bridge.stop_simulation():
                raise RuntimeError(bridge.last_error or "cannot stop simulation")
        hidden_target_count = hide_teaching_targets(sim)
        print(f"TEACHING TARGETS HIDDEN: {hidden_target_count}", flush=True)

        supply_by_type = {
            "A": {
                "box": bridge.get_object_handle("BOX_BLANK"),
                "term": bridge.get_object_handle("TERMINAL_BLOCK_SUPPLY"),
                "pcb": bridge.get_object_handle("PCB_SUPPLY"),
                "module": bridge.get_object_handle("CONTROL_MODULE_SUPPLY"),
            },
            "B": {
                "box": sim.getObject("/FiveCR5A_Cell/PartsB/Box_Blank_B"),
                "term": sim.getObject(
                    "/FiveCR5A_Cell/PartsB/Terminal_Block_Supply_B"
                ),
                "pcb": sim.getObject("/FiveCR5A_Cell/PartsB/PCB_Supply_B"),
                "module": sim.getObject(
                    "/FiveCR5A_Cell/PartsB/Control_Module_Supply_B"
                ),
            },
        }
        template_product = bridge.get_object_handle("INSPECTION_PRODUCT")
        product_b = sim.getObject(
            "/FiveCR5A_Cell/PartsB/Inspection_ControlBox_Product_B"
        )
        assembled_b_products = [product_b]
        try:
            assembled_b_products.append(
                sim.getObject(
                    "/FiveCR5A_Cell/PartsB/Assembly_ControlBox_Product_B"
                )
            )
        except Exception:
            pass
        box_b = supply_by_type["B"]["box"]
        recolor_b_cabinet(sim, box_b, assembled_b_products)
        parts_roots = {
            "A": sim.getObject("/FiveCR5A_Cell/Parts"),
            "B": sim.getObject("/FiveCR5A_Cell/PartsB"),
        }
        assembly_fixture = sim.getObject(
            "/FiveCR5A_Cell/Areas/Assembly_Fixture"
        )
        for template in (template_product, product_b):
            template_states[template] = (
                list(sim.getObjectPose(template, -1)),
                {
                    int(shape): int(
                        sim.getObjectInt32Param(
                            shape, sim.objintparam_visibility_layer
                        )
                    )
                    for shape in sim.getObjectsInTree(
                        template, sim.object_shape_type, 0
                    )
                },
            )

        products = [template_product]
        for unit_number in range(2, args.orders + 1):
            clone = clone_product(
                sim, template_product, parts_roots["A"], unit_number
            )
            products.append(clone)
            created_products.append(clone)
        for index, product in enumerate(products):
            sim.setObjectPosition(product, -1, [3.0, 3.0 + index * 0.2, 0.5])
            set_visible(sim, product, False)

        sim.setObjectPosition(product_b, -1, [4.0, 4.0, 0.5])
        set_visible(sim, product_b, False)
        for type_supply in supply_by_type.values():
            for handle in type_supply.values():
                set_visible(sim, handle, False)

        joints = {
            robot_id: bridge.get_robot_joint_handles(robot_id)
            for robot_id in ("R1", "R2", "R3", "R4", "R5")
        }
        roots = {
            robot_id: bridge.get_object_handle(robot_id)
            for robot_id in ("R1", "R2", "R3", "R4", "R5")
        }
        for robot_joints in joints.values():
            for joint in robot_joints:
                sim.setJointPosition(joint, 0.0)
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, math.radians(500.0)
                )

        arms = {
            "R1": Arm(bridge, sim, "R1", joints["R1"]),
            "R2": Arm(bridge, sim, "R2", joints["R2"], step_deg=8.0),
            "R3": Arm(bridge, sim, "R3", joints["R3"]),
            "R4": Arm(bridge, sim, "R4", joints["R4"]),
            "R5": Arm(bridge, sim, "R5", joints["R5"]),
        }
        # Move downstream robots to their persistent wait positions while the
        # first product is assembled.
        arms["R4"].set_sequence([("r4_home_to_wait", 1)])
        arms["R5"].set_sequence([("r5_home_to_wait", 1)])

        events: set[str] = {"INSPECTION_FREE"}
        # The recorded HOME->WAIT routes for R4 and R5 briefly intersect if
        # started on the same frame.  Stage them once at batch startup; all
        # later cycles begin from their already-safe wait poses.
        arms["R5"].wait_event = "R4_INITIAL_WAIT"
        attached = {
            "box": False,
            "term": False,
            "pcb": False,
            "module": False,
            "product": False,
            "r5_product": False,
        }
        inspection_queue: deque[int] = deque()
        front_index = -1
        back_index = -1
        next_front = 0
        front_active = False
        back_active = False
        inspection_occupied = False
        completed = 0
        completion_order: list[str] = []
        b_fixture_correction_mm = 0.0
        b_fixture_clearance_mm = float("nan")
        b_opening_up = False
        defect_product_position: list[float] | None = None
        urgent_reader = UrgentOrderReader(args.urgent_file)
        urgent_index = args.orders
        urgent_order_id = ""
        urgent_latched = False
        urgent_started = False
        urgent_done = False

        def unit_type(unit: int) -> str:
            return "B" if unit == urgent_index else "A"

        def unit_label(unit: int) -> str:
            if unit == urgent_index:
                return urgent_order_id or "B急单"
            return f"A{unit + 1}"

        def product_for(unit: int) -> int:
            return product_b if unit == urgent_index else products[unit]

        def supply_for(unit: int) -> dict[str, int]:
            return supply_by_type[unit_type(unit)]

        def event(name: str, unit: int) -> str:
            return f"{name}:{unit + 1}"

        def fire(name: str, unit: int) -> None:
            value = event(name, unit)
            events.add(value)
            print(f"[订单 {unit_label(unit)}] 事件 {name}", flush=True)

        def present_supply(unit: int) -> None:
            product_type = unit_type(unit)
            supply = supply_for(unit)
            parts_root = parts_roots[product_type]
            for type_supply in supply_by_type.values():
                for handle in type_supply.values():
                    set_visible(sim, handle, False)
            for name, handle in supply.items():
                position, quaternion = SUPPLY_POSES[name]
                if product_type == "B" and name == "box":
                    # The checked-in B branch has the opposite local shell
                    # orientation from A.  Identity keeps its opening upward;
                    # applying A's 180-degree X rotation turns it upside down.
                    quaternion = [0, 0, 0, 1]
                sim.setObjectParent(handle, parts_root, True)
                sim.setObjectPosition(handle, -1, list(position))
                sim.setObjectQuaternion(handle, -1, list(quaternion))
                set_visible(sim, handle, True)
            product = product_for(unit)
            sim.setObjectParent(product, parts_root, True)
            sim.setObjectPosition(product, -1, [-1.08, 0.12, 0.2160])
            sim.setObjectQuaternion(product, -1, [0, 0, 0, 1])
            set_visible(sim, product, False)
            print(
                f"[订单 {unit_label(unit)}] {product_type}型箱体与配套物料已生成",
                flush=True,
            )

        def start_front(unit: int) -> None:
            nonlocal front_index, front_active
            present_supply(unit)
            attached.update(
                box=False,
                term=False,
                pcb=False,
                module=False,
                product=False,
            )
            sequences = front_sequences(first=unit == 0)
            for robot_id in ("R1", "R2", "R3"):
                arms[robot_id].set_sequence(sequences[robot_id])
            front_index = unit
            front_active = True
            print(
                f"[订单 {unit_label(unit)}] R1/R2/R3 开始前段装配",
                flush=True,
            )

        def start_back(unit: int) -> None:
            nonlocal back_index, back_active
            is_defect = unit < args.orders and unit + 1 == args.defect_order
            sequences = back_sequences(defect=is_defect)
            arms["R4"].set_sequence(sequences["R4"])
            arms["R5"].set_sequence(sequences["R5"])
            # R5 enters the shared inspection envelope only after R4 has
            # completely returned to its wait pose.  Starting at screw-done
            # allowed a brief R4/R5 path crossing on repeated cycles.
            arms["R5"].wait_event = event("R4_AT_WAIT", unit)
            attached["r5_product"] = False
            back_index = unit
            back_active = True
            print(
                f"[订单 {unit_label(unit)}] R4/R5 开始锁付与"
                f"{'不良品' if is_defect else '良品'}分拣",
                flush=True,
            )

        def handle_front_end(arm: Arm) -> None:
            nonlocal inspection_occupied
            nonlocal b_fixture_correction_mm, b_fixture_clearance_mm
            nonlocal b_opening_up
            unit = front_index
            supply = supply_for(unit)
            name = arm.segment_name()
            direction = arm.segment_dir()
            if arm.robot_id == "R1":
                if name == "r1_box_grasp" and direction == 1 and not attached["box"]:
                    bridge.set_gripper_gap("R1", 0.150)
                    attach_handle(bridge, sim, supply["box"], "R1")
                    attached["box"] = True
                elif name == "r1_box_place_descend" and attached["box"]:
                    bridge.set_gripper_gap("R1", 0.158)
                    bridge.detach_object(supply["box"])
                    if unit_type(unit) == "B":
                        b_opening_up, bottom_z, wall_z = cabinet_opening_is_up(
                            sim, supply["box"]
                        )
                        if not b_opening_up:
                            raise RuntimeError(
                                "B型箱体姿态错误：开口仍朝下 "
                                f"(bottom_z={bottom_z:.4f}, wall_z={wall_z:.4f})"
                            )
                        fixture_top = shape_maximum_z(sim, assembly_fixture)
                        desired_bottom = fixture_top + 0.001
                        cabinet_bottom = tree_minimum_z(sim, supply["box"])
                        correction = desired_bottom - cabinet_bottom
                        if abs(correction) > 1e-5:
                            position = list(
                                sim.getObjectPosition(supply["box"], -1)
                            )
                            position[2] += correction
                            sim.setObjectPosition(supply["box"], -1, position)
                        b_fixture_correction_mm = correction * 1000
                        b_fixture_clearance_mm = (
                            tree_minimum_z(sim, supply["box"])
                            - fixture_top
                        ) * 1000
                        print(
                            "B ORIENTATION: opening=UP; B FIXTURE ALIGNMENT: "
                            f"adjusted={b_fixture_correction_mm:+.1f}mm "
                            f"clearance={b_fixture_clearance_mm:.1f}mm",
                            flush=True,
                        )
                    attached["box"] = False
                    fire("BOX_PLACED", unit)
                elif name == "r1_terminal_descend" and direction == 1 and not attached["term"]:
                    bridge.set_gripper_gap("R1", 0.046)
                    attach_handle(bridge, sim, supply["term"], "R1")
                    attached["term"] = True
                elif name == "r1_terminal_mid_to_place_app" and direction == 1:
                    arm.wait_event = event("PCB_PLACED", unit)
                elif name == "r1_terminal_place_descend" and direction == 1 and attached["term"]:
                    bridge.detach_object(supply["term"])
                    attached["term"] = False
                    bridge.set_gripper_gap("R1", 0.158)
                    start = list(sim.getObjectPosition(supply["term"], -1))
                    target = [-1.05397, 0.086735, 0.273073]
                    for step in range(1, 9):
                        ratio = step / 8
                        sim.setObjectPosition(
                            supply["term"], -1,
                            [start[i] + (target[i] - start[i]) * ratio for i in range(3)],
                        )
                        bridge.step()
                    fire("TERMINAL_PLACED", unit)
            elif arm.robot_id == "R2":
                if name == "r2_pick_descend" and not attached["pcb"]:
                    attach_handle(bridge, sim, supply["pcb"], "R2")
                    attached["pcb"] = True
                elif name == "r2_pick_to_safe_wait":
                    arm.wait_event = event("BOX_PLACED", unit)
                elif name == "r2_place_descend" and direction == 1 and attached["pcb"]:
                    bridge.detach_object(supply["pcb"])
                    attached["pcb"] = False
                    fire("PCB_PLACED", unit)
            elif arm.robot_id == "R3":
                product = product_for(unit)
                if name == "r3_module_pick_descend" and direction == 1 and not attached["module"]:
                    bridge.set_gripper_gap("R3", 0.080)
                    attach_handle(bridge, sim, supply["module"], "R3")
                    attached["module"] = True
                    arm.wait_event = event("TERMINAL_PLACED", unit)
                elif name == "r3_module_place_descend" and direction == 1 and attached["module"]:
                    bridge.set_gripper_gap("R3", 0.170)
                    bridge.detach_object(supply["module"])
                    attached["module"] = False
                    start = list(sim.getObjectPosition(supply["module"], -1))
                    target = [-1.053, 0.111, 0.267]
                    for step in range(1, 9):
                        ratio = step / 8
                        sim.setObjectPosition(
                            supply["module"], -1,
                            [start[i] + (target[i] - start[i]) * ratio for i in range(3)],
                        )
                        bridge.step()
                    fire("MODULE_PLACED", unit)
                elif name == "r3_module_to_product_pick_app" and direction == 1:
                    set_visible(sim, product, True)
                    for handle in supply.values():
                        set_visible(sim, handle, False)
                        sim.setObjectPosition(handle, -1, [3.0, 3.0, 0.5])
                elif name == "r3_product_pick_descend" and direction == 1 and not attached["product"]:
                    bridge.set_gripper_gap("R3", 0.1564)
                    attach_handle(bridge, sim, product, "R3")
                    attached["product"] = True
                elif name == "r3_product_pick_descend" and direction == -1 and inspection_occupied:
                    arm.wait_event = "INSPECTION_FREE"
                    print(
                        f"[订单 {unit_label(unit)}] 装配完成，等待检测工位释放",
                        flush=True,
                    )
                elif name == "r3_product_place_descend" and direction == 1 and attached["product"]:
                    bridge.set_gripper_gap("R3", 0.170)
                    bridge.detach_object(product)
                    attached["product"] = False
                    inspection_occupied = True
                    events.discard("INSPECTION_FREE")
                    inspection_queue.append(unit)
                    fire("PRODUCT_PLACED", unit)

        def handle_back_end(arm: Arm) -> None:
            nonlocal defect_product_position
            unit = back_index
            name = arm.segment_name()
            direction = arm.segment_dir()
            product = product_for(unit)
            if arm.robot_id == "R4":
                if name == "r4_tcp_to_press" and direction == 1:
                    arm.delay_frames = 20
                elif name == "r4_press_to_app" and direction == 1:
                    fire("R4_SCREW_DONE", unit)
                elif name == "r4_app_to_wait" and direction == 1:
                    fire("R4_AT_WAIT", unit)
            elif arm.robot_id == "R5":
                is_defect = unit < args.orders and unit + 1 == args.defect_order
                if name == "r5_pick_descend" and direction == 1 and not attached["r5_product"]:
                    bridge.set_gripper_gap("R5", 0.150)
                    attach_handle(bridge, sim, product, "R5")
                    attached["r5_product"] = True
                    arm.wait_event = event("R4_AT_WAIT", unit)
                elif (
                    name
                    == (
                        "defect_high_to_place_final"
                        if is_defect
                        else "good_app_to_place_zfixed2"
                    )
                    and direction == 1
                    and attached["r5_product"]
                ):
                    bridge.set_gripper_gap("R5", 0.158)
                    bridge.detach_object(product)
                    attached["r5_product"] = False
                    start = list(sim.getObjectPosition(product, -1))
                    if is_defect:
                        target = [-0.15, -1.12, 0.270]
                    else:
                        distance = 0.70 + 0.12 * unit
                        target = [start[0], start[1] - distance, start[2]]
                    for step in range(1, 21):
                        ratio = step / 20
                        sim.setObjectPosition(
                            product,
                            -1,
                            [
                                start[index]
                                + (target[index] - start[index]) * ratio
                                for index in range(3)
                            ],
                        )
                        bridge.step()
                    if is_defect:
                        defect_product_position = list(target)

        # Runtime collision collections for all ten robot pairs.
        collections = {}
        for robot_id, root in roots.items():
            collection = sim.createCollection(sim.handle_all)
            sim.addItemToCollection(collection, sim.handle_tree, root, 0)
            collections[robot_id] = collection
        collision_counts = {
            f"{left}-{right}": 0
            for index, left in enumerate(roots)
            for right in list(roots)[index + 1 :]
        }

        bridge.set_stepping(True)
        if not bridge.start_simulation():
            raise RuntimeError(bridge.last_error or "cannot start simulation")
        time.sleep(0.2)
        start_front(0)
        next_front = 1
        frame = 0

        while completed < args.orders + int(bool(urgent_order_id)):
            if frame % 5 == 0:
                for request in urgent_reader.poll():
                    if request["product_type"] != "B":
                        raise RuntimeError(
                            "本验证模式运行中只接受一台B型急单"
                        )
                    if urgent_order_id:
                        raise RuntimeError("本验证模式每批只接受一台B型急单")
                    urgent_order_id = request["order_id"]
                    urgent_latched = True
                    print(
                        f"[急单] 已锁存 {urgent_order_id}；停止放行下一台A，"
                        "等待当前在制品全部完成",
                        flush=True,
                    )

            for robot_id, arm in arms.items():
                if arm.done:
                    continue
                if arm.wait_event and arm.wait_event in events and arm.delay_frames == 0:
                    arm.wait_event = None
                if arm.wait_event:
                    continue
                if arm.delay_frames > 0:
                    arm.delay_frames -= 1
                    continue
                outcome = arm.step()
                if outcome == "end":
                    if (
                        robot_id == "R4"
                        and arm.segment_name() == "r4_home_to_wait"
                        and not back_active
                    ):
                        events.add("R4_INITIAL_WAIT")
                        print("[初始化] R4 已到等待点，R5 开始入位", flush=True)
                    if robot_id in {"R1", "R2", "R3"} and front_active:
                        handle_front_end(arm)
                    elif robot_id in {"R4", "R5"} and back_active:
                        handle_back_end(arm)
                    arm.next_segment()

            if front_active and all(arms[rid].done for rid in ("R1", "R2", "R3")):
                print(
                    f"[订单 {unit_label(front_index)}] 前段完成，R1/R2/R3 空闲",
                    flush=True,
                )
                front_active = False
                if next_front < args.orders and not urgent_latched:
                    start_front(next_front)
                    next_front += 1

            if (
                not back_active
                and inspection_queue
                and arms["R4"].done
                and arms["R5"].done
            ):
                start_back(inspection_queue.popleft())

            if back_active and arms["R4"].done and arms["R5"].done:
                completed += 1
                completion_order.append(unit_label(back_index))
                print(
                    f"[订单 {unit_label(back_index)}] 锁付分拣完成 "
                    f"({completed}/{args.orders + int(bool(urgent_order_id))})",
                    flush=True,
                )
                if back_index == urgent_index:
                    urgent_done = True
                    urgent_latched = False
                    print(
                        f"[急单] {urgent_order_id} 已完成，恢复剩余A型订单",
                        flush=True,
                    )
                back_active = False
                inspection_occupied = False
                events.add("INSPECTION_FREE")

            changeover_safe = (
                not front_active
                and not back_active
                and not inspection_queue
                and not inspection_occupied
                and all(arm.done for arm in arms.values())
            )
            if urgent_latched and not urgent_started and changeover_safe:
                urgent_started = True
                print(
                    f"[换型] 产线已清空并到达安全等待位，开始 {urgent_order_id}",
                    flush=True,
                )
                start_front(urgent_index)
            elif urgent_done and next_front < args.orders and changeover_safe:
                start_front(next_front)
                next_front += 1

            bridge.step()
            if frame % 20 == 0:
                robot_ids = list(roots)
                for index, left in enumerate(robot_ids):
                    for right in robot_ids[index + 1 :]:
                        hit = sim.checkCollision(
                            collections[left], collections[right]
                        )
                        if bool(hit[0] if isinstance(hit, tuple) else hit):
                            key = f"{left}-{right}"
                            collision_counts[key] += 1
                            print(
                                "[碰撞监测] "
                                f"{key} frame={frame} "
                                f"segments={arms[left].segment_name()}/"
                                f"{arms[right].segment_name()}",
                                flush=True,
                            )
            frame += 1
            if frame > 200000:
                raise RuntimeError("pipeline exceeded the deterministic frame limit")

        print(
            f"B ORIENTATION: opening={'UP' if b_opening_up else 'INVALID'}",
            flush=True,
        )
        print(
            f"QUALITY ROUTE: defect=A{args.defect_order}"
            if args.defect_order
            else "QUALITY ROUTE: all-good",
            flush=True,
        )
        if defect_product_position is not None:
            print(
                "DEFECT PRODUCT POSITION: "
                + ",".join(f"{value:.3f}" for value in defect_product_position),
                flush=True,
            )
        print(f"TEACHING TARGETS HIDDEN: {hidden_target_count}", flush=True)
        print(
            "B FIXTURE ALIGNMENT: "
            f"adjusted={b_fixture_correction_mm:+.1f}mm "
            f"clearance={b_fixture_clearance_mm:.1f}mm",
            flush=True,
        )
        print(f"PIPELINE COLLISIONS: {collision_counts}", flush=True)
        if any(collision_counts.values()):
            raise RuntimeError(
                f"pipeline collision gate failed: {collision_counts}"
            )
        print(f"PIPELINE ORDER: {','.join(completion_order)}", flush=True)
        print(
            f"PIPELINE COMPLETE: orders={args.orders + int(bool(urgent_order_id))} "
            f"urgent={urgent_order_id or 'none'}",
            flush=True,
        )
        return 0
    finally:
        try:
            if bridge.is_connected():
                bridge.stop_simulation()
                if created_products:
                    sim.removeObjects(created_products)
                for template, (pose, visibility) in template_states.items():
                    sim.setObjectPose(template, -1, pose)
                    for shape, layer in visibility.items():
                        sim.setObjectInt32Param(
                            shape,
                            sim.objintparam_visibility_layer,
                            layer,
                        )
        finally:
            bridge.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
