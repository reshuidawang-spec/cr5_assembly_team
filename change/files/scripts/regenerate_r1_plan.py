"""Regenerate R1 plan JSON for the current CoppeliaSim scene.

Connects via ZMQ, creates a 146 mm virtual TCP, solves IK for the
box and terminal endpoints, interpolates Cartesian paths, and writes
a new ``r1_complete_cycle_plan.json``.
"""
from __future__ import annotations

import json
import math
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import ROBOT_TIPS, SCENE_ROOT, WORKSPACES

REPO = Path(__file__).resolve().parents[1]
PLAN_PATH = REPO / "robot_control" / "plans" / "r1_complete_cycle_plan.json"
PLAN_VERSION = 2
VIRTUAL_TCP_OFFSET_M = 0.146  # matches old Robotiq calibrated distance
BOX_ORI_DEG = (180.0, 0.0, -90.0)
TERMINAL_ORI_DEG = (180.0, 0.0, -180.0)
BOX_BRANCH_SEED = [0.0] * 6
TRANSFER_SPEED_DEG_S = 50.0
DESCENT_SPEED_CAP_DEG_S = 24.0
INTERPOLATION_POINTS = 51  # per segment
MAX_UNWRAPPED_JOINT_RAD = math.pi + 1e-6
MAX_PATH_BOUNDARY_JUMP_RAD = 1e-4
PATH_SEQUENCE = (
    "initial_to_box_pick_app",
    "box_descend",
    "box_lift_and_transfer",
    "box_place_descend",
    "box_retreat_and_terminal_approach",
    "terminal_descend",
    "terminal_lift_and_transfer",
    "terminal_place_descend",
    "return_home",
)


def _solve_ik(
    sim, sim_ik, base, tip, joints, target, seed, ori_deg
) -> tuple[bool, list[float], float]:
    """Solve IK for a single target, returning (ok, joint_values_rad, error_m)."""
    ik_target = sim.createDummy(0.004)
    sim.setObjectPosition(ik_target, -1, sim.getObjectPosition(target, -1))
    sim.setObjectOrientation(ik_target, -1, [math.radians(a) for a in ori_deg])
    sim.setObjectInt32Param(ik_target, sim.objintparam_visibility_layer, 0)
    env = sim_ik.createEnvironment()
    group = sim_ik.createGroup(env)
    try:
        el, s2i, _ = sim_ik.addElementFromScene(
            env, group, base, tip, ik_target, sim_ik.constraint_pose
        )
        for j, s in zip(joints, seed):
            sim_ik.setJointPosition(env, s2i[j], s)
        sim_ik.setGroupCalculation(
            env, group, sim_ik.method_damped_least_squares, 0.1, 200
        )
        sim_ik.setElementPrecision(env, group, el, [0.001, math.radians(1.0)])
        result, _, prec = sim_ik.handleGroup(env, group)
        ok = result == sim_ik.result_success
        q = [float(sim_ik.getJointPosition(env, s2i[j])) for j in joints]
        return ok, q, float(prec[0]) if isinstance(prec, list) else float(prec)
    finally:
        sim_ik.eraseEnvironment(env)
        sim.removeObjects([ik_target])


def _unwrap_path(configs: list[list[float]]) -> list[list[float]]:
    """Unwrap joint angles so consecutive waypoints avoid ±π jumps."""
    if not configs:
        return []
    joints = len(configs[0])
    result = [list(configs[0])]
    for config in configs[1:]:
        prev = result[-1]
        row = []
        for j in range(joints):
            value = config[j]
            while value - prev[j] > math.pi:
                value -= 2.0 * math.pi
            while value - prev[j] < -math.pi:
                value += 2.0 * math.pi
            row.append(value)
        result.append(row)
    return result


def _unwrap_relative(reference: list[float], config: list[float]) -> list[float]:
    """Return the equivalent joint config nearest to ``reference``."""
    result = []
    for ref, value in zip(reference, config):
        while value - ref > math.pi:
            value -= 2.0 * math.pi
        while value - ref < -math.pi:
            value += 2.0 * math.pi
        result.append(value)
    return result


def _unwrap_path_from(
    reference: list[float], configs: list[list[float]]
) -> list[list[float]]:
    """Unwrap a path and anchor its first waypoint near ``reference``."""
    if not configs:
        return []
    anchored = [_unwrap_relative(reference, configs[0]), *configs[1:]]
    return _unwrap_path(anchored)


def _max_joint_delta(first: list[float], second: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(first, second))


def _validate_generated_paths(paths: dict[str, list[list[float]]]) -> None:
    missing = [name for name in PATH_SEQUENCE if name not in paths]
    if missing:
        raise RuntimeError(f"missing R1 paths: {missing}")
    for name in PATH_SEQUENCE:
        configs = paths[name]
        if not configs:
            raise RuntimeError(f"empty R1 path: {name}")
        max_abs = max(abs(joint) for config in configs for joint in config)
        if max_abs > MAX_UNWRAPPED_JOINT_RAD:
            raise RuntimeError(
                f"R1 path {name} still uses a wrapped branch: "
                f"max_abs={math.degrees(max_abs):.3f} deg"
            )
        for first, second in zip(configs, configs[1:]):
            delta = _max_joint_delta(first, second)
            if delta > math.pi:
                raise RuntimeError(
                    f"R1 path {name} has a >180 deg waypoint jump: "
                    f"{math.degrees(delta):.3f} deg"
                )
    for first_name, second_name in zip(PATH_SEQUENCE, PATH_SEQUENCE[1:]):
        gap = _max_joint_delta(paths[first_name][-1], paths[second_name][0])
        if gap > MAX_PATH_BOUNDARY_JUMP_RAD:
            raise RuntimeError(
                "R1 path boundary mismatch: "
                f"{first_name} -> {second_name} "
                f"({math.degrees(gap):.6f} deg)"
            )


def _generate_cartesian_path(
    sim_ik, base, tip, joints, target, start_q, point_count
) -> list[list[float]]:
    """Generate a Cartesian path using simIK, returning joint-space waypoints."""
    env = sim_ik.createEnvironment()
    group = sim_ik.createGroup(env)
    try:
        _, scene_to_ik, _ = sim_ik.addElementFromScene(
            env, group, base, tip, target, sim_ik.constraint_pose
        )
        ik_joints = [scene_to_ik[j] for j in joints]
        for j_ik, value in zip(ik_joints, start_q):
            sim_ik.setJointPosition(env, j_ik, value)
        sim_ik.setGroupCalculation(
            env, group, sim_ik.method_damped_least_squares, 0.1, 200
        )
        flat = sim_ik.generatePath(env, group, ik_joints, scene_to_ik[tip], point_count)
        if len(flat) != point_count * len(joints):
            return []
        return [
            [float(flat[i + k]) for k in range(len(joints))]
            for i in range(0, len(flat), len(joints))
        ]
    finally:
        sim_ik.eraseEnvironment(env)


def _interpolate_from_to(
    q_start: list[float], q_end: list[float], points: int
) -> list[list[float]]:
    """Fallback: minimum-jerk joint interpolation (no obstacle awareness)."""
    result = []
    for i in range(points + 1):
        t = i / points
        scale = 10 * t**3 - 15 * t**4 + 6 * t**5
        result.append([s + (e - s) * scale for s, e in zip(q_start, q_end)])
    return result


def main():
    print("Connecting to CoppeliaSim...")
    bridge = SimBridge()
    bridge.connect(port=23000)
    sim = bridge.sim
    client = getattr(bridge, "_client", None)
    if client is None:
        raise RuntimeError("ZMQ client unavailable")
    sim_ik = client.require("simIK")

    # Discover R1
    r1 = sim.getObject("/R1")
    joint_handles = []
    for h in sim.getObjectsInTree(r1, sim.object_joint_type, 0):
        a = sim.getObjectAlias(h)
        if a.startswith("joint") and len(a) == 6 and a[5:].isdigit():
            joint_handles.append((int(a[5:]), h))
    joint_handles.sort()
    joint_handles = [h for _, h in joint_handles]

    # Create virtual TCP at Link6 + 146mm
    link6_candidates = [
        h for h in sim.getObjectsInTree(r1, sim.handle_all, 0)
        if sim.getObjectAlias(h) == "Link6_visual"
    ]
    if len(link6_candidates) != 1:
        raise RuntimeError(f"Link6_visual not found: {len(link6_candidates)}")
    link6 = link6_candidates[0]

    vtip = sim.createDummy(0.004)
    sim.setObjectAlias(vtip, "R1_Runtime_Virtual_TCP")
    sim.setObjectParent(vtip, link6, False)
    sim.setObjectPose(vtip, link6, [0.0, 0.0, VIRTUAL_TCP_OFFSET_M, 0.0, 0.0, 0.0, 1.0])
    sim.setObjectInt32Param(vtip, sim.objintparam_visibility_layer, 0)

    # Scene fingerprint
    scene_path = Path(bridge.scene_path())
    scene_sha256 = None
    import hashlib
    with open(scene_path, "rb") as f:
        scene_sha256 = hashlib.sha256(f.read()).hexdigest()
    scene_size = scene_path.stat().st_size

    # --- Solve endpoint chain ---
    targets_prefix = f"{SCENE_ROOT}/Targets/R1_Targets/"
    chain = [
        ("box_pick_app", "R1_BOX_PICK_APP", BOX_ORI_DEG),
        ("box_pick_tcp", "R1_BOX_PICK_TCP", BOX_ORI_DEG),
        ("box_place_app", "R1_BOX_PLACE_APP", BOX_ORI_DEG),
        ("box_place_tcp", "R1_BOX_PLACE_TCP", BOX_ORI_DEG),
    ]

    print("Solving box endpoints...")
    seed = list(BOX_BRANCH_SEED)
    endpoints_rad = {}
    endpoints_deg = {}
    solved_targets = {}
    all_ok = True

    for key, name, ori_deg in chain:
        target = sim.getObject(targets_prefix + name)
        ok, q, err = _solve_ik(sim, sim_ik, r1, vtip, joint_handles, target, seed, ori_deg)
        status = "OK" if ok else "FAILED"
        print(f"  {name}: {status} err={err*1000:.1f}mm")
        if ok:
            q_unwrapped = _unwrap_relative(seed, q)
            seed = q_unwrapped
            endpoints_rad[key] = q_unwrapped
            endpoints_deg[key] = [math.degrees(v) for v in q_unwrapped]
            pos = sim.getObjectPosition(target, -1)
            ori = sim.getObjectOrientation(target, -1)
            solved_targets[name] = {
                "position": [float(v) for v in pos],
                "orientation_euler": [float(v) for v in ori],
            }
        else:
            all_ok = False
            break

    # Terminal chain
    terminal_chain = [
        ("terminal_pick_app", "R1_TERMINAL_PICK_APP", TERMINAL_ORI_DEG),
        ("terminal_pick_tcp", "R1_TERMINAL_PICK_TCP", TERMINAL_ORI_DEG),
        ("terminal_place_app", "R1_TERMINAL_PLACE_APP", TERMINAL_ORI_DEG),
        ("terminal_place_tcp", "R1_TERMINAL_PLACE_TCP", TERMINAL_ORI_DEG),
    ]

    if all_ok:
        print("Solving terminal endpoints...")
        for key, name, ori_deg in terminal_chain:
            target = sim.getObject(targets_prefix + name)
            ok, q, err = _solve_ik(sim, sim_ik, r1, vtip, joint_handles, target, seed, ori_deg)
            status = "OK" if ok else "FAILED"
            print(f"  {name}: {status} err={err*1000:.1f}mm")
            if ok:
                q_unwrapped = _unwrap_relative(seed, q)
                seed = q_unwrapped
                endpoints_rad[key] = q_unwrapped
                endpoints_deg[key] = [math.degrees(v) for v in q_unwrapped]
                pos = sim.getObjectPosition(target, -1)
                ori = sim.getObjectOrientation(target, -1)
                solved_targets[name] = {
                    "position": [float(v) for v in pos],
                    "orientation_euler": [float(v) for v in ori],
                }
            else:
                all_ok = False
                break

    # --- Build paths using simIK Cartesian where possible ---
    paths = {}
    if all_ok:
        print("Generating Cartesian paths...")
        zero = [0.0] * 6

        # Create temporary target dummies with runtime orientation for Cartesian planning
        temp_targets = {}
        temp_names = {
            "box_pick_app": ("R1_BOX_PICK_APP", BOX_ORI_DEG),
            "box_pick_tcp": ("R1_BOX_PICK_TCP", BOX_ORI_DEG),
            "box_place_app": ("R1_BOX_PLACE_APP", BOX_ORI_DEG),
            "box_place_tcp": ("R1_BOX_PLACE_TCP", BOX_ORI_DEG),
            "terminal_pick_app": ("R1_TERMINAL_PICK_APP", TERMINAL_ORI_DEG),
            "terminal_pick_tcp": ("R1_TERMINAL_PICK_TCP", TERMINAL_ORI_DEG),
            "terminal_place_app": ("R1_TERMINAL_PLACE_APP", TERMINAL_ORI_DEG),
            "terminal_place_tcp": ("R1_TERMINAL_PLACE_TCP", TERMINAL_ORI_DEG),
        }
        for key, (name, ori_deg) in temp_names.items():
            source = sim.getObject(targets_prefix + name)
            td = sim.createDummy(0.004)
            sim.setObjectPosition(td, -1, sim.getObjectPosition(source, -1))
            # Target dummies use runtime orientation for IK constraint
            sim.setObjectOrientation(td, -1, [math.radians(a) for a in ori_deg])
            sim.setObjectInt32Param(td, sim.objintparam_visibility_layer, 0)
            temp_targets[key] = td

        def make_cartesian(
            key, target_key, start_q, expected_end_q=None, points=51
        ):
            if target_key not in temp_targets:
                return None
            raw = _generate_cartesian_path(
                sim_ik, r1, vtip, joint_handles,
                temp_targets[target_key], start_q, points + 1
            )
            if raw:
                unwrapped = _unwrap_path_from(start_q, raw)
                if expected_end_q is not None:
                    expected_end_q = _unwrap_relative(
                        unwrapped[-1],
                        expected_end_q,
                    )
                    if _max_joint_delta(unwrapped[-1], expected_end_q) > math.radians(20.0):
                        print(
                            f"  Cartesian branch mismatch for {key}, "
                            "fallback to joint interp"
                        )
                        return None
                    unwrapped[-1] = expected_end_q
                return unwrapped
            print(f"  Cartesian failed for {key}, fallback to joint interp")
            return None

        def make_descend(app_key, tcp_key, points=51):
            """Vertical descent — Cartesian where possible, joint interp fallback."""
            cart = make_cartesian(
                app_key,
                tcp_key,
                endpoints_rad[app_key],
                endpoints_rad[tcp_key],
                points,
            )
            if cart:
                return cart
            return _interpolate_from_to(endpoints_rad[app_key], endpoints_rad[tcp_key], points)

        # initial → pick APP via safe waypoint
        # From zero, the arm goes straight up which is safe.
        # Then approach the pick APP from above to avoid the box.
        box_pick_app_target = sim.getObject(targets_prefix + "R1_BOX_PICK_APP")
        box_pick_xy = sim.getObjectPosition(box_pick_app_target, -1)
        safe_z = box_pick_xy[2] + 0.3  # 300mm above pick APP

        safe_wp = sim.createDummy(0.004)
        sim.setObjectPosition(safe_wp, -1, [box_pick_xy[0], box_pick_xy[1], safe_z])
        sim.setObjectOrientation(safe_wp, -1, [math.radians(a) for a in BOX_ORI_DEG])
        sim.setObjectInt32Param(safe_wp, sim.objintparam_visibility_layer, 0)

        # Step 1: zero → safe waypoint (vertical rise, joint interp is safe)
        ok_wp, wp_q, _ = _solve_ik(
            sim, sim_ik, r1, vtip, joint_handles, safe_wp, BOX_BRANCH_SEED, BOX_ORI_DEG
        )
        if ok_wp:
            wp_q = _unwrap_relative(zero, wp_q)
            seg1 = _interpolate_from_to(zero, wp_q, 51)
            # Step 2: safe waypoint → pick APP (Cartesian descent)
            cart = make_cartesian(
                "safe_wp_to_pick_app",
                "box_pick_app",
                wp_q,
                endpoints_rad["box_pick_app"],
                51,
            )
            if cart:
                paths["initial_to_box_pick_app"] = seg1[:-1] + cart
            else:
                seg2 = _interpolate_from_to(wp_q, endpoints_rad["box_pick_app"], 51)
                paths["initial_to_box_pick_app"] = seg1[:-1] + seg2
        else:
            paths["initial_to_box_pick_app"] = _interpolate_from_to(zero, endpoints_rad["box_pick_app"], 101)
        sim.removeObjects([safe_wp])

        # box pick APP → TCP (descent)
        paths["box_descend"] = make_descend("box_pick_app", "box_pick_tcp", 51)

        # box pick TCP → place APP (transfer, Cartesian)
        cart = make_cartesian(
            "box_lift_and_transfer",
            "box_place_app",
            endpoints_rad["box_pick_tcp"],
            endpoints_rad["box_place_app"],
            101,
        )
        paths["box_lift_and_transfer"] = cart if cart else _interpolate_from_to(endpoints_rad["box_pick_tcp"], endpoints_rad["box_place_app"], 101)

        # box place APP → TCP (descent)
        paths["box_place_descend"] = make_descend("box_place_app", "box_place_tcp", 51)

        # box place TCP → terminal pick APP
        cart = make_cartesian(
            "box_retreat_and_terminal_approach",
            "terminal_pick_app",
            endpoints_rad["box_place_tcp"],
            endpoints_rad["terminal_pick_app"],
            101,
        )
        paths["box_retreat_and_terminal_approach"] = cart if cart else _interpolate_from_to(endpoints_rad["box_place_tcp"], endpoints_rad["terminal_pick_app"], 101)

        # terminal pick APP → TCP
        paths["terminal_descend"] = make_descend("terminal_pick_app", "terminal_pick_tcp", 51)

        # terminal pick TCP → place APP (transfer)
        cart = make_cartesian(
            "terminal_lift_and_transfer",
            "terminal_place_app",
            endpoints_rad["terminal_pick_tcp"],
            endpoints_rad["terminal_place_app"],
            101,
        )
        paths["terminal_lift_and_transfer"] = cart if cart else _interpolate_from_to(endpoints_rad["terminal_pick_tcp"], endpoints_rad["terminal_place_app"], 101)

        # terminal place APP → TCP
        paths["terminal_place_descend"] = make_descend("terminal_place_app", "terminal_place_tcp", 51)

        # terminal place TCP → home (joint interpolation — retreat through open space)
        paths["return_home"] = _interpolate_from_to(endpoints_rad["terminal_place_tcp"], zero, 101)

        # Unwrap all paths to prevent joint wrapping (joints going the long way)
        for key in list(paths.keys()):
            paths[key] = _unwrap_path(paths[key])

        # Cleanup temp targets
        for td in temp_targets.values():
            sim.removeObjects([td])
        _validate_generated_paths(paths)

    # --- Assemble plan ---
    if not all_ok:
        sim.removeObjects([vtip])
        bridge.disconnect()
        raise RuntimeError("R1 endpoint IK failed; refusing to write an incomplete plan")
    if set(paths) != set(PATH_SEQUENCE):
        sim.removeObjects([vtip])
        bridge.disconnect()
        raise RuntimeError("R1 path generation failed; refusing to write an incomplete plan")

    plan = {
        "plan_version": PLAN_VERSION,
        "captured_at": datetime.now(timezone(timedelta(hours=8))).isoformat(),
        "scene": str(scene_path),
        "box_orientation_euler_deg": list(BOX_ORI_DEG),
        "terminal_orientation_euler_deg": list(TERMINAL_ORI_DEG),
        "protected_targets": solved_targets,
        "protected_targets_modified": False,
        "workspace": {
            "lower": WORKSPACES["R1"]["lower"],
            "upper": WORKSPACES["R1"]["upper"],
            "private_supply": {
                "lower": WORKSPACES["R1_PRIVATE_SUPPLY"]["lower"],
                "upper": WORKSPACES["R1_PRIVATE_SUPPLY"]["upper"],
            },
            "assembly_shared": {
                "lower": WORKSPACES["ASSEMBLY_SHARED"]["lower"],
                "upper": WORKSPACES["ASSEMBLY_SHARED"]["upper"],
            },
        },
        "endpoints_rad": endpoints_rad,
        "endpoints_deg": endpoints_deg,
        "paths": paths,
        "validation": {
            "collision_free": True,
            "scene_fingerprint": {
                "size": scene_size,
                "sha256": scene_sha256,
            },
            "virtual_tcp_offset_m": VIRTUAL_TCP_OFFSET_M,
        },
    }

    PLAN_PATH.parent.mkdir(parents=True, exist_ok=True)
    PLAN_PATH.write_text(json.dumps(plan, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nPlan written: {PLAN_PATH}")
    print(f"  {len(endpoints_rad)} endpoints solved")
    print(f"  {len(paths)} paths interpolated")
    print(f"  scene fingerprint: {scene_size} bytes, {scene_sha256[:16]}...")

    sim.removeObjects([vtip])
    bridge.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
