#!/usr/bin/env python3
"""前端协调演示: R1+R2+R3 并行装配

机制: 单时间步循环(所有臂同一 b.step 推进) + 事件触发 + 等待点保持。
流程: 三臂并行抓取 -> R2 到等待点 -> R1 放箱(箱体就位) -> R2 放PCB
      -> R1 抓端子安装(端子就位) -> R3 放模块 -> 全部回位。
"""
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim_bridge.coppelia_client import SimBridge

MAX_STEP = 5.0

OUT = ROOT / "data" / "captured_paths"
R5_GOOD_TRANSFER_PATH = "pick_to_good_app_avoid_r4wait"
R5_GOOD_TRANSFER_SMOOTHING_PASSES = 2


def _hit(res):
    if isinstance(res, tuple):
        return bool(res[0])
    return bool(res)


def chaikin_smooth(points, passes=1):
    """Round a joint-space polyline while preserving its exact endpoints.

    Corner cutting stays inside each adjacent-point corridor.  This keeps the
    taught R5/R4 avoidance side, whereas a direct arc crosses R4's wait pose.
    """
    result = [list(point) for point in points]
    for _ in range(max(0, int(passes))):
        if len(result) < 2:
            break
        rounded = [list(result[0])]
        for first, second in zip(result, result[1:]):
            rounded.append(
                [
                    0.75 * left + 0.25 * right
                    for left, right in zip(first, second)
                ]
            )
            rounded.append(
                [
                    0.25 * left + 0.75 * right
                    for left, right in zip(first, second)
                ]
            )
        rounded.append(list(result[-1]))
        result = rounded
    return result


def load(name):
    d = json.load(open(OUT / f"{name}.json"))
    points = d["trajectories"][0]["points_deg"]
    if name == R5_GOOD_TRANSFER_PATH:
        return chaikin_smooth(
            points,
            R5_GOOD_TRANSFER_SMOOTHING_PASSES,
        )
    return points


class Arm:
    def __init__(self, bridge, sim, robot_id, joints, step_deg=MAX_STEP):
        self.b = bridge
        self.sim = sim
        self.robot_id = robot_id
        self.joints = joints
        self.seq = []
        self.seg_i = -1
        self.pts = []
        self.arc = []
        self.total = 0.0
        self.arc_pos = 0.0
        self.wait_event = None
        self.done = False
        self.step_deg = step_deg
        self.delay_frames = 0

    def set_sequence(self, seq):
        self.seq = seq
        self.seg_i = -1
        self.done = False
        self.wait_event = None
        self.delay_frames = 0
        self.next_segment()

    def next_segment(self):
        self.seg_i += 1
        if self.seg_i >= len(self.seq):
            self.done = True
            return
        name, d = self.seq[self.seg_i]
        self.pts = load(name) if d == 1 else load(name)[::-1]
        self.arc = []
        self.total = 0.0
        for i in range(len(self.pts) - 1):
            g = max(abs(x - y) for x, y in zip(self.pts[i], self.pts[i + 1]))
            self.arc.append(g)
            self.total += g
        self.arc_pos = 0.0

    def sample(self, s):
        acc = 0.0
        for i, g in enumerate(self.arc):
            if acc + g >= s - 1e-12:
                t = (s - acc) / g if g > 0 else 0.0
                a, c = self.pts[i], self.pts[i + 1]
                return [a[m] + (c[m] - a[m]) * t for m in range(6)]
            acc += g
        return list(self.pts[-1])

    def step(self):
        """按恒定弧长推进。反走抬升段用 1.5 倍步进(退出要快)。返回 'ok'/'end'/None"""
        if self.done:
            return None
        if self.arc_pos >= self.total - 1e-9:
            return "end"
        step = self.step_deg
        # R5 also uses reversed paths for its loaded outbound transfer, so keep
        # its configured dense 2-degree step instead of treating every reverse
        # segment as a fast empty-tool retreat.
        if (
            self.robot_id != "R5"
            and self.seg_i >= 0
            and self.seq[self.seg_i][1] == -1
        ):
            step = self.step_deg * 1.5
        nxt = min(self.arc_pos + step, self.total)
        tgt = self.sample(nxt)
        for j, v in zip(self.joints, [math.radians(x) for x in tgt]):
            self.sim.setJointPosition(j, v)
        self.arc_pos = nxt
        if nxt >= self.total - 1e-9:
            return "end"
        return "ok"

    def segment_name(self):
        if 0 <= self.seg_i < len(self.seq):
            return self.seq[self.seg_i][0]
        return None

    def segment_dir(self):
        if 0 <= self.seg_i < len(self.seq):
            return self.seq[self.seg_i][1]
        return 1

    def at_segment_end(self):
        return self.pt_i >= len(self.pts) - 1 and self.sub == 0


def main():
    start_from_wait = "--start-from-wait" in sys.argv
    keep_running = "--keep-running" in sys.argv
    reuse_running = "--reuse-running" in sys.argv
    b = SimBridge(request_timeout=20.0, validate_contract=False)
    for _ in range(10):
        if b.connect():
            break
        time.sleep(2)
    if not b._connected:
        print("connect failed:", b.last_error)
        return 1
    sim = b._client.require("sim")
    if sim.getSimulationState() != 0 and not reuse_running:
        b.stop_simulation()
        time.sleep(0.5)

    # 物料
    box = b.get_object_handle("BOX_BLANK")
    term = b.get_object_handle("TERMINAL_BLOCK_SUPPLY")
    pcb = b.get_object_handle("PCB_SUPPLY")
    module = b.get_object_handle("CONTROL_MODULE_SUPPLY")
    product = b.get_object_handle("INSPECTION_PRODUCT")
    try:
        sim.removeObjects([b.get_object_handle("ASSEMBLY_PRODUCT")])
    except Exception:
        pass

    r1j = b.get_robot_joint_handles("R1")
    r2j = b.get_robot_joint_handles("R2")
    r3j = b.get_robot_joint_handles("R3")
    r1 = b.get_object_handle("R1")
    r2 = b.get_object_handle("R2")
    r3 = b.get_object_handle("R3")
    r4j = b.get_robot_joint_handles("R4")
    r5j = b.get_robot_joint_handles("R5")
    r4 = b.get_object_handle("R4")
    r5 = b.get_object_handle("R5")
    if start_from_wait and not reuse_running:
        d1 = json.load(open(OUT / "r1_key_poses.json"))["key_poses"]
        d2 = json.load(open(OUT / "r2_key_poses.json"))["key_poses"]
        d3 = json.load(open(OUT / "r3_key_poses.json"))["key_poses"]
        d4 = json.load(open(OUT / "r4_key_poses.json"))["key_poses"]
        for j, v in zip(r1j, [math.radians(x) for x in d1["box_pick_app"]]):
            sim.setJointPosition(j, v)
        for j, v in zip(r2j, [math.radians(x) for x in d2["safe_wait"]]):
            sim.setJointPosition(j, v)
        for j, v in zip(r3j, [math.radians(x) for x in d3["module_pick_app"]]):
            sim.setJointPosition(j, v)
        for j, v in zip(r4j, [math.radians(x) for x in d4["wait"]]):
            sim.setJointPosition(j, v)
    elif not reuse_running:
        for js in (r1j, r2j, r3j):
            for j in js:
                sim.setJointPosition(j, 0.0)
    sim.setObjectPosition(box, -1, [-1.86, 0.22, 0.156])
    sim.setObjectQuaternion(box, -1, [1, 0, 0, 0])
    sim.setObjectPosition(term, -1, [-1.82, -0.02, 0.1665])
    sim.setObjectQuaternion(term, -1, [1, 0, 0, 0])
    sim.setObjectPosition(pcb, -1, [-1.22, -0.42, 0.1584])
    sim.setObjectQuaternion(pcb, -1, [1, 0, 0, 0])
    sim.setObjectPosition(module, -1, [-0.78, -0.20, 0.1665])
    sim.setObjectQuaternion(module, -1, [1, 0, 0, 0])
    for part in (box, term, pcb, module):
        for s in sim.getObjectsInTree(part, sim.object_shape_type, 0):
            sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 1)
    def current_material_color():
        try:
            product_state = str(sim.getStringSignal("cell_product_state") or "").lower()
        except Exception:
            product_state = ""
        return {
            "product_a": [1.00, 0.50, 0.20],
            "product_b": [0.85, 0.05, 0.05],
            "product_c": [0.20, 0.55, 1.00],
        }.get(product_state)

    def apply_current_product_color(root_handle):
        color = current_material_color()
        if not color:
            return
        try:
            for s in sim.getObjectsInTree(root_handle, sim.object_shape_type, 0):
                sim.setShapeColor(s, "", sim.colorcomponent_ambient_diffuse, color)
        except Exception:
            pass

    for material_part in (box, term, pcb, module, product):
        apply_current_product_color(material_part)
    # 检测区展示产品B (PartsB) 与转运产品重叠, 隐藏+移走
    try:
        insp_b = sim.getObject("/FiveCR5A_Cell/PartsB/Inspection_ControlBox_Product_B")
        sim.setObjectPosition(insp_b, -1, [3.0, 3.0, 0.5])
        for s in sim.getObjectsInTree(insp_b, sim.object_shape_type, 0):
            sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 0)
    except Exception:
        pass

    for js in (r1j, r2j, r3j):
        for j in js:
            sim.setObjectFloatParam(j, sim.jointfloatparam_maxvel, math.radians(500.0))
    b.set_stepping(True)

    def _load_key_pose(robot: str, key: str):
        return json.load(open(OUT / f"{robot}_key_poses.json"))["key_poses"][key]

    def _path_endpoint(path_name: str):
        return load(path_name)[-1]

    def set_wait_posture() -> None:
        """Keep the line visibly ready for the next product/order."""
        wait_configs = {
            "R1": _load_key_pose("r1", "box_pick_app"),
            "R2": _load_key_pose("r2", "safe_wait"),
            "R3": _load_key_pose("r3", "module_pick_app"),
            "R4": _load_key_pose("r4", "wait"),
            "R5": _path_endpoint("good_place_to_wait_new"),
        }
        for robot_id, joints in (
            ("R1", r1j),
            ("R2", r2j),
            ("R3", r3j),
            ("R4", r4j),
            ("R5", r5j),
        ):
            for joint, value in zip(joints, wait_configs[robot_id]):
                sim.setJointPosition(joint, math.radians(value))

    # 每臂流程: (段名, 方向)  1=正向 -1=反走抬升
    R1_SEQ = []
    if not start_from_wait:
        R1_SEQ.append(("r1_initial_to_box_pick_app", 1))
    R1_SEQ += [
        ("r1_box_descend", 1),
        ("r1_box_grasp", 1),
        ("r1_box_grasp", -1),          # 抬升
        ("r1_box_lift_to_mid2", 1),    # -> 中间点2
        ("r1_mid2_to_mid1", 1),        # 中间点2 -> 中间点1
        ("r1_mid1_to_place_app", 1),   # 中间点1 -> 装配台上方
        ("r1_box_place_descend", 1),
        ("r1_box_place_descend", -1),  # 放完立刻抬升退出
        ("r1_box_to_term_transition", 1),  # -> 中间点1
        ("r1_mid1_to_mid2", 1),            # 中间点1 -> 中间点2
        ("r1_mid2_to_pick_app", 1),        # 中间点2 -> 抓取上方
        ("r1_terminal_descend", 1),
        ("r1_terminal_descend", -1),   # 抬升
        ("r1_terminal_mid_transfer", 1),
        ("r1_terminal_mid_to_place_app", 1),
        ("r1_terminal_place_descend", 1),
        ("r1_terminal_place_descend", -1),  # 抬升
        ("r1_return_home", 1),
        ("r1_initial_to_box_pick_app", 1),  # 完成后回到抓箱等待位
    ]
    R2_SEQ = []
    if start_from_wait:
        R2_SEQ.append(("r2_safe_wait_to_pick_app", 1))
    else:
        R2_SEQ.append(("r2_initial_to_pick_app", 1))
    R2_SEQ += [
        ("r2_pick_descend", 1),
        ("r2_pick_to_safe_wait", 1),   # 之后等待 BOX_PLACED
        ("r2_safe_wait_to_place_app", 1),
        ("r2_place_descend", 1),
        ("r2_place_descend", -1),      # 放完立刻抬升退出
        ("r2_place_to_safe_wait", 1),
    ]
    R3_SEQ = []
    if not start_from_wait:
        R3_SEQ.append(("r3_initial_to_module_pick_app", 1))
    R3_SEQ += [
        ("r3_module_pick_descend", 1), # 之后等待 TERMINAL_PLACED
        ("r3_module_lift_transfer", 1),
        ("r3_module_place_descend", 1),
        ("r3_module_place_descend", -1),  # 放完抬升退出
        ("r3_module_to_product_pick_app", 1),
        ("r3_product_pick_descend", 1),
        ("r3_product_pick_descend", -1),  # 抓完产品抬升
        ("r3_product_transfer", 1),
        ("r3_product_place_descend", 1),
        ("r3_product_place_descend", -1),  # 放完抬升退出
        ("r3_place_to_module_pick_app", 1),  # 回抓模块上方
    ]

    R4_SEQ = []
    if not start_from_wait:
        R4_SEQ.append(("r4_home_to_wait", 1))
    R4_SEQ += [
        ("r4_wait_to_app", 1),
        ("r4_app_to_tcp", 1),
        ("r4_tcp_to_press", 1),
        ("r4_press_to_app", 1),
        ("r4_app_to_wait", 1),
    ]
    for js in (r4j, r5j):
        for j in js:
            sim.setObjectFloatParam(j, sim.jointfloatparam_maxvel, math.radians(500.0))
    R5_SEQ = []
    if not start_from_wait:
        R5_SEQ.append(("r5_home_to_wait", 1))  # 阶段A: 先到等待点
    R5_SEQ += [
        ("r5_wait_to_pick_app", 1),      # R4完成后: 先去产品上方
        ("r5_pick_descend", 1),          # 再垂直下降抓取
        ("r5_pick_descend", -1),         # 抓取后先抬升末端
        ("r5_wait_to_pick_app", -1),     # 沿安全通道退回等待位
        ("good_place_to_wait_new", -1),  # 反走回程轨迹进入GOOD位
        ("good_place_to_wait_new", 1),
    ]
    arms = {
        "R1": Arm(b, sim, "R1", r1j),
        "R2": Arm(b, sim, "R2", r2j, step_deg=8.0),
        "R3": Arm(b, sim, "R3", r3j),
        "R4": Arm(b, sim, "R4", r4j),
        "R5": Arm(b, sim, "R5", r5j),
    }
    arms["R4"].set_sequence(R4_SEQ)
    arms["R5"].set_sequence(R5_SEQ)
    arms["R1"].set_sequence(R1_SEQ)
    arms["R2"].set_sequence(R2_SEQ)
    arms["R3"].set_sequence(R3_SEQ)

    events = set()
    attached = {"box": False, "term": False, "pcb": False, "module": False, "product": False, "r5prod": False}
    final_good_position = None

    def fire(ev):
        events.add(ev)
        print(f"  [事件] {ev}")

    def good_conveyor_path():
        """Return the current GOOD belt start/end product-center positions.

        The latest AGV/changeover docs move Good_Conveyor left by 0.5 m, but
        older cached scenes may still use the previous X coordinate.  Read the
        belt object at runtime so R5 always releases the visual product onto
        the belt that is actually loaded.
        """
        try:
            belt = sim.getObject(
                "/FiveCR5A_Cell/Conveyors/Good_Conveyor/"
                "Good_Conveyor_Belt_Black"
            )
            center = list(sim.getObjectPosition(belt, -1))
            return (
                [center[0], center[1] + 0.62, 0.270],
                [center[0], center[1] - 0.52, 0.270],
            )
        except Exception:
            return ([0.48, -1.06, 0.270], [0.48, -2.20, 0.270])

    def align_good_conveyor_to_agv_layout():
        """Apply the latest AGV guide's GOOD conveyor left-shift in-place."""
        target_conveyor_x = 0.48
        target_place_x = 0.35
        try:
            conveyor = sim.getObject("/FiveCR5A_Cell/Conveyors/Good_Conveyor")
            pos = list(sim.getObjectPosition(conveyor, -1))
            pos[0] = target_conveyor_x
            sim.setObjectPosition(conveyor, -1, pos)
        except Exception as exc:
            print(f"  [场景对齐警告] Good_Conveyor 未移动: {exc}")
        for target_name in ("R5_GOOD_PLACE_APP", "R5_GOOD_PLACE_TCP"):
            try:
                target = sim.getObject(
                    f"/FiveCR5A_Cell/Targets/R5_Targets/{target_name}"
                )
                pos = list(sim.getObjectPosition(target, -1))
                pos[0] = target_place_x
                sim.setObjectPosition(target, -1, pos)
            except Exception as exc:
                print(f"  [场景对齐警告] {target_name} 未移动: {exc}")
        print(
            "  [场景对齐] Good_Conveyor X=0.48, "
            "R5_GOOD_PLACE X=0.35"
        )

    align_good_conveyor_to_agv_layout()
    b.start_simulation()
    time.sleep(0.3)
    sim.setObjectPosition(product, -1, [-1.08, 0.12, 0.2160])
    sim.setObjectQuaternion(product, -1, [0, 0, 0, 1])  # yaw=180
    for s in sim.getObjectsInTree(product, sim.object_shape_type, 0):
        sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 0)
    frame = 0

    # 距离监测
    from sim_bridge.scene_objects import ROBOT_TIPS
    from robot_control.runtime_cartesian import find_unique_alias
    tips = {
        "R1": find_unique_alias(sim, r1, ROBOT_TIPS["R1"]),
        "R2": find_unique_alias(sim, r2, ROBOT_TIPS["R2"]),
        "R3": find_unique_alias(sim, r3, ROBOT_TIPS["R3"]),
    }
    min_dists = {"R1-R2": 9.9, "R1-R3": 9.9, "R2-R3": 9.9}
    # 碰撞集合: R1(+箱体) vs R2
    coll_r1 = sim.createCollection(sim.handle_all)
    sim.addItemToCollection(coll_r1, sim.handle_tree, r1, 0)
    sim.addItemToCollection(coll_r1, sim.handle_tree, box, 0)
    coll_r2 = sim.createCollection(sim.handle_all)
    sim.addItemToCollection(coll_r2, sim.handle_tree, r2, 0)
    coll_r3 = sim.createCollection(sim.handle_all)
    sim.addItemToCollection(coll_r3, sim.handle_tree, r3, 0)
    coll_r4 = sim.createCollection(sim.handle_all)
    sim.addItemToCollection(coll_r4, sim.handle_tree, r4, 0)
    coll_r5 = sim.createCollection(sim.handle_all)
    sim.addItemToCollection(coll_r5, sim.handle_tree, r5, 0)
    coll_hits = {"R1-R2": 0, "R1-R3": 0, "R2-R3": 0, "R1-R4": 0, "R2-R4": 0, "R3-R4": 0, "R1-R5": 0, "R2-R5": 0, "R3-R5": 0, "R4-R5": 0}

    def handle_end(arm):
        nonlocal final_good_position
        rid = arm.robot_id
        name = arm.segment_name()
        d = arm.segment_dir()
        if rid == "R1":
            if name == "r1_box_grasp" and d == 1 and not attached["box"]:
                b.set_gripper_gap("R1", 0.150)
                b.attach_object("BOX_BLANK", "R1")
                attached["box"] = True
                vis = {sim.getObjectAlias(s): sim.getObjectInt32Param(s, sim.objintparam_visibility_layer)
                       for s in sim.getObjectsInTree(box, sim.object_shape_type, 0)}
                print(f"  [R1 箱体 attach] Box可见性: {vis}")
            elif name == "r1_box_place_descend" and attached["box"]:
                b.set_gripper_gap("R1", 0.158)
                b.detach_object(box)
                apply_current_product_color(box)
                attached["box"] = False
                print("  [R1 箱体 detach]")
                fire("BOX_PLACED")
            elif name == "r1_terminal_mid_to_place_app" and d == 1:
                arm.wait_event = "PCB_PLACED"
                print("  [R1 装端子前等 R2 放完 PCB]")
            elif name == "r1_terminal_descend" and d == 1 and not attached["term"]:
                apply_current_product_color(term)
                b.set_gripper_gap("R1", 0.046)
                b.attach_object("TERMINAL_BLOCK_SUPPLY", "R1")
                attached["term"] = True
                print("  [R1 端子 attach]")
            elif name == "r1_terminal_place_descend" and d == 1 and attached["term"]:
                b.detach_object(term)
                apply_current_product_color(term)
                attached["term"] = False
                b.set_gripper_gap("R1", 0.158)
                print("  [R1 端子 detach, 夹爪打开, 端子落下安装]")
                # 物理未触发下落, 分步插值落到安装位
                start = list(sim.getObjectPosition(term, -1))
                target = [-1.05397, 0.086735, 0.273073]
                for k in range(1, 9):
                    f = k / 8
                    sim.setObjectPosition(term, -1, [start[m] + (target[m] - start[m]) * f for m in range(3)])
                    b.step()
                tp = sim.getObjectPosition(term, -1)
                print(f"  [端子到位 z={tp[2]:.4f} (安装位 0.2730)]")
                fire("TERMINAL_PLACED")
            elif name == "r1_return_home" and d == 1:
                fire("R1_CLEARED")
                print("  [R1 已回抓箱上方, 离开装配台]")
        elif rid == "R2":
            if name == "r2_pick_descend" and not attached["pcb"]:
                apply_current_product_color(pcb)
                b.attach_object("PCB_SUPPLY", "R2")
                attached["pcb"] = True
                print("  [R2 PCB attach]")
            elif name == "r2_place_descend" and attached["pcb"]:
                b.detach_object(pcb)
                apply_current_product_color(pcb)
                attached["pcb"] = False
                print("  [R2 PCB detach]")
                fire("PCB_PLACED")
            if name == "r2_pick_to_safe_wait":
                arm.wait_event = "BOX_PLACED"
                fire("R2_AT_WAIT")
                print("  [R2 到达等待点, 等 BOX_PLACED]")
        elif rid == "R3":
            if name == "r3_module_pick_descend" and not attached["module"]:
                apply_current_product_color(module)
                b.set_gripper_gap("R3", 0.080)
                b.attach_object("CONTROL_MODULE_SUPPLY", "R3")
                attached["module"] = True
                print("  [R3 模块 attach]")
                arm.wait_event = "TERMINAL_PLACED"
                print("  [R3 抓完模块, 等 R1 装完端子]")
            elif name == "r3_module_place_descend" and d == 1 and attached["module"]:
                b.set_gripper_gap("R3", 0.170)
                b.detach_object(module)
                apply_current_product_color(module)
                attached["module"] = False
                print("  [R3 模块 detach, 松开夹爪]")
                start = list(sim.getObjectPosition(module, -1))
                target = [-1.053, 0.111, 0.267]
                for k in range(1, 9):
                    f = k / 8
                    sim.setObjectPosition(module, -1, [start[m] + (target[m] - start[m]) * f for m in range(3)])
                    b.step()
                print(f"  [模块下降到位 z={sim.getObjectPosition(module, -1)[2]:.4f}]")
                fire("MODULE_PLACED")
            elif name == "r3_module_to_product_pick_app" and d == 1:
                # 组装完成: 成品(Inspection产品)可见, 组装件隐藏
                apply_current_product_color(product)
                for s in sim.getObjectsInTree(product, sim.object_shape_type, 0):
                    sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 1)
                for part in (box, pcb, module, term):
                    for s in sim.getObjectsInTree(part, sim.object_shape_type, 0):
                        sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 0)
                    sim.setObjectPosition(part, -1, [3.0, 3.0, 0.5])
                print("  [组装完成: 成品可见, 组装件隐藏]")
            elif name == "r3_product_pick_descend" and d == 1 and not attached["product"]:
                b.set_gripper_gap("R3", 0.1564)
                for s in sim.getObjectsInTree(product, sim.object_shape_type, 0):
                    try:
                        sim.setShapeMassAndInertia(s, 0.0, [0, 0, 0],
                                                   [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                                   [0, 0, 0])
                    except Exception:
                        pass
                b.attach_object("INSPECTION_PRODUCT", "R3")
                attached["product"] = True
                print("  [R3 产品 attach (非动态)]")
            elif name == "r3_product_place_descend" and d == 1 and attached["product"]:
                b.set_gripper_gap("R3", 0.170)
                b.detach_object(product)
                attached["product"] = False
                print("  [R3 产品 detach, 到检测区]")
                fire("PRODUCT_PLACED")
        elif rid == "R5":
            if name == "r5_home_to_wait" and d == 1:
                arm.wait_event = "R4_SCREW_DONE"
                print("  [R5 到等待点, 等 R4 锁付完成]")
            elif name == "r5_wait_to_pick_app" and start_from_wait and arm.wait_event is None:
                arm.wait_event = "R4_SCREW_DONE"
                print("  [R5 已在等待点, 等 R4 锁付完成]")
            if name == "r5_pick_descend" and d == 1 and not attached["r5prod"]:
                b.set_gripper_gap("R5", 0.150)
                b.attach_object("INSPECTION_PRODUCT", "R5")
                attached["r5prod"] = True
                arm.wait_event = "R4_AT_WAIT"
                print("  [R5 抓取完成, 等 R4 回等待点再搬运]")
            elif name == "good_place_to_wait_new" and d == -1 and attached["r5prod"]:
                b.set_gripper_gap("R5", 0.158)
                b.detach_object(product)
                attached["r5prod"] = False
                print("  [R5 产品 detach, 分拣到GOOD]")
                # Snap onto the actually loaded GOOD conveyor, then move
                # along its Y direction. This avoids old/new scene X-offset
                # mismatches after the AGV guide moved Good_Conveyor.
                apply_current_product_color(product)
                p0, p1 = good_conveyor_path()
                sim.setObjectPosition(product, -1, p0)
                for k in range(1, 21):
                    f = k / 20
                    sim.setObjectPosition(
                        product, -1,
                        [p0[m] + (p1[m] - p0[m]) * f for m in range(3)]
                    )
                    b.step()
                    time.sleep(0.01)
                final_good_position = p1
                print(f"  [产品沿传送带移动完成: {p0} -> {p1}]")
        elif rid == "R4":
            if name == "r4_home_to_wait":
                arm.wait_event = "PRODUCT_PLACED"
                print("  [R4 到等待点, 等产品到检测区]")
            elif name == "r4_wait_to_app" and start_from_wait and arm.wait_event is None:
                arm.wait_event = "PRODUCT_PLACED"
                print("  [R4 已在等待点, 等产品到检测区]")
            elif name == "r4_tcp_to_press" and d == 1:
                arm.delay_frames = 20
                print("  [R4 锁付按压 1s]")
            elif name == "r4_press_to_app" and d == 1:
                fire("R4_SCREW_DONE")
                print("  [R4 锁付完开始回程, R5 出发抓取]")
            elif name == "r4_app_to_wait" and d == 1:
                fire("R4_AT_WAIT")
                print("  [R4 回到等待点]")

    try:
        while not all(a.done for a in arms.values()):
            for rid, arm in arms.items():
                if arm.done:
                    continue
                if arm.wait_event and arm.wait_event in events and arm.delay_frames == 0:
                    if rid == "R3":
                        arm.delay_frames = 10
                        print("  [R3 延迟 0.5s 再装模块]")
                    arm.wait_event = None
                if arm.wait_event and arm.wait_event not in events:
                    continue
                if arm.delay_frames > 0:
                    arm.delay_frames -= 1
                    continue
                r = arm.step()
                if r == "end":
                    handle_end(arm)
                    if arm.wait_event and arm.wait_event not in events:
                        continue
                    arm.next_segment()
            b.step()
            # R1 搬箱段期间碰撞检测 R1(含箱体) vs R2
            r1a = arms["R1"]
            if r1a.segment_name() == "r1_box_lift_and_transfer":
                if _hit(sim.checkCollision(coll_r1, coll_r2)):
                    coll_hits["R1-R2"] += 1
                    print(f"  [碰撞!] R1搬箱 vs R2 (帧 {frame})")
            if frame % 20 == 0:
                if _hit(sim.checkCollision(coll_r1, coll_r2)):
                    coll_hits["R1-R2"] += 1
                if _hit(sim.checkCollision(coll_r1, coll_r3)):
                    coll_hits["R1-R3"] += 1
                if _hit(sim.checkCollision(coll_r2, coll_r3)):
                    coll_hits["R2-R3"] += 1
            frame += 1
        time.sleep(0.3)
        print(f"碰撞帧数: {coll_hits}")
        print(f"最小距离: { {k: round(v*1000) for k, v in min_dists.items()} } mm")
        print("全部完成")
    finally:
        try:
            set_wait_posture()
            for _ in range(5):
                b.step()
            if keep_running:
                print("  [连续模式] 保持仿真运行，准备衔接下一件产品")
            else:
                if final_good_position is not None:
                    apply_current_product_color(product)
                    sim.setObjectParent(product, sim.getObject("/FiveCR5A_Cell/Parts"), True)
                    sim.setObjectPosition(product, -1, final_good_position)
                    sim.setObjectQuaternion(product, -1, [0, 0, 0, 1])
                    for s in sim.getObjectsInTree(product, sim.object_shape_type, 0):
                        sim.setObjectInt32Param(s, sim.objintparam_visibility_layer, 1)
                    print(f"  [最终画面保持: 良品在传送带 {final_good_position}]")
                print("  [等待状态] 机械臂保持等待下一单")
        except Exception:
            pass
        try:
            b.disconnect()
        except Exception:
            pass
    if keep_running:
        print("完成, 保持运行等待下一轮")
    else:
        print("完成, 机械臂保持等待状态")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
