#!/usr/bin/env python3
"""Read-only structural and geometry audit for an open five-CR5A scene."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_bridge.coppelia_client import _remote_api_client_class
from sim_bridge.scene_objects import (
    ARM_JOINT_ALIASES,
    PARTS,
    POINTS,
    ROBOT_IDS,
    ROBOT_TIPS,
    SCENE_ROOT,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = REPO_ROOT / "scenes" / "five_cr5a_cell.ttt"
DEFAULT_BASELINE = (
    REPO_ROOT / "configs" / "five_cr5a_scene_audit_baseline.json"
)
SCRIPT_DUMMIES = ("Main_Cell_Generator", "ROS2_All_Robot_Bridge")
RUNTIME_MARKERS = ("Runtime", "Candidate", "Command_Bridge")


def fingerprint_file(path: Path) -> dict[str, Any]:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"size": path.stat().st_size, "sha256": digest.hexdigest()}


def _near(first: Iterable[float], second: Iterable[float], tolerance: float) -> bool:
    first_values = list(first)
    second_values = list(second)
    return len(first_values) == len(second_values) and max(
        abs(a - b) for a, b in zip(first_values, second_values)
    ) <= tolerance


def is_protected_target(name: str, baseline: dict[str, Any]) -> bool:
    return any(
        name.startswith(prefix)
        for prefix in baseline["protected_target_prefixes"]
    )


def compare_target_snapshots(
    actual: dict[str, dict[str, list[float]]],
    expected: dict[str, dict[str, list[float]]],
    baseline: dict[str, Any],
) -> list[dict[str, Any]]:
    tolerance = float(baseline["tolerances"]["target"])
    changes = []
    for name, before in expected.items():
        after = actual.get(name)
        if after is None:
            changes.append(
                {
                    "name": name,
                    "protected": is_protected_target(name, baseline),
                    "missing": True,
                    "before": before,
                }
            )
            continue
        if not _near(before["position"], after["position"], tolerance) or not _near(
            before["orientation"], after["orientation"], tolerance
        ):
            changes.append(
                {
                    "name": name,
                    "protected": is_protected_target(name, baseline),
                    "missing": False,
                    "before": before,
                    "after": after,
                }
            )
    return changes


def compute_r5_height_result(
    product_root_z: float,
    pick_tcp_z: float,
    place_tcp_z: float,
    desired_root_z: float,
) -> dict[str, float]:
    rigid_final = product_root_z + place_tcp_z - pick_tcp_z
    return {
        "rigid_final_product_root_z": rigid_final,
        "desired_product_root_z": desired_root_z,
        "height_error_m": rigid_final - desired_root_z,
    }


def _round_vector(values: Iterable[float]) -> list[float]:
    return [round(float(value), 9) for value in values]


def _path(sim: Any, handle: int) -> str | None:
    return sim.getObjectAlias(handle, 1) if handle != -1 else None


def _safe_get(sim: Any, path: str) -> int:
    try:
        return int(sim.getObject(path))
    except Exception:
        return -1


def _unique_alias(sim: Any, root: int, alias: str, object_type: int) -> int:
    matches = [
        handle
        for handle in sim.getObjectsInTree(root, object_type, 0)
        if sim.getObjectAlias(handle) == alias
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected one {alias} below {_path(sim, root)}, found {len(matches)}"
        )
    return matches[0]


def _collision_between_shape_sets(
    sim: Any, first: Iterable[int], second: Iterable[int]
) -> dict[str, Any]:
    first_shapes = list(first)
    second_shapes = list(second)
    if not first_shapes or not second_shapes:
        return {"collision": False, "pair": None, "incomplete": True}
    first_collection = sim.createCollection(1)
    second_collection = sim.createCollection(1)
    try:
        for handle in first_shapes:
            sim.addItemToCollection(
                first_collection, sim.handle_single, handle, 0
            )
        for handle in second_shapes:
            sim.addItemToCollection(
                second_collection, sim.handle_single, handle, 0
            )
        state, pair = sim.checkCollision(first_collection, second_collection)
        return {
            "collision": bool(state),
            "pair": [_path(sim, handle) for handle in pair] if state else None,
            "incomplete": False,
        }
    finally:
        sim.destroyCollection(first_collection)
        sim.destroyCollection(second_collection)


def _tree_shapes(sim: Any, root: int) -> list[int]:
    if root == -1:
        return []
    return list(sim.getObjectsInTree(root, sim.object_shape_type, 0))


def _target_snapshot(sim: Any) -> tuple[dict[str, Any], list[str]]:
    result = {}
    missing = []
    for name, path in POINTS.items():
        handle = _safe_get(sim, path)
        if handle == -1:
            missing.append(name)
            continue
        result[name] = {
            "position": _round_vector(sim.getObjectPosition(handle, -1)),
            "orientation": _round_vector(sim.getObjectOrientation(handle, -1)),
        }
    return result, missing


def _robot_record(sim: Any, robot_id: str, joint_tolerance_deg: float) -> dict[str, Any]:
    root = _safe_get(sim, f"/{robot_id}")
    if root == -1:
        return {"missing": True}
    all_joints = list(sim.getObjectsInTree(root, sim.object_joint_type, 0))
    by_alias = {sim.getObjectAlias(handle): handle for handle in all_joints}
    missing_joints = [alias for alias in ARM_JOINT_ALIASES if alias not in by_alias]
    arm_joints = [by_alias[alias] for alias in ARM_JOINT_ALIASES if alias in by_alias]
    joint_degrees = [
        math.degrees(float(sim.getJointPosition(handle))) for handle in arm_joints
    ]
    tip_matches = [
        handle
        for handle in sim.getObjectsInTree(root, sim.object_dummy_type, 0)
        if sim.getObjectAlias(handle) == ROBOT_TIPS[robot_id]
    ]
    tip_record = None
    if len(tip_matches) == 1:
        tip = tip_matches[0]
        parent = sim.getObjectParent(tip)
        tip_record = {
            "path": _path(sim, tip),
            "parent": _path(sim, parent),
            "pose_relative_parent": _round_vector(sim.getObjectPose(tip, parent)),
        }
    return {
        "missing": False,
        "arm_joint_aliases_missing": missing_joints,
        "all_joint_count": len(all_joints),
        "joint_deg": _round_vector(joint_degrees),
        "at_zero": len(joint_degrees) == 6
        and max(abs(value) for value in joint_degrees) <= joint_tolerance_deg,
        "tip_match_count": len(tip_matches),
        "tip": tip_record,
    }


def build_audit(
    sim: Any,
    scene_file: Path,
    baseline: dict[str, Any],
    allow_running: bool = False,
) -> dict[str, Any]:
    errors: list[str] = []
    warnings: list[str] = []
    scene_file = scene_file.resolve()
    fingerprint = fingerprint_file(scene_file)
    expected_fingerprint = baseline["scene"]
    fingerprint_changed = fingerprint != {
        "size": expected_fingerprint["size"],
        "sha256": expected_fingerprint["sha256"],
    }
    if fingerprint_changed:
        warnings.append("scene fingerprint changed; full path regression required")

    live_scene = Path(
        sim.getStringParam(sim.stringparam_scene_path_and_name)
    ).resolve()
    if live_scene != scene_file:
        errors.append(f"open scene differs from audited file: {live_scene}")
    simulation_state = int(sim.getSimulationState())
    if simulation_state != sim.simulation_stopped and not allow_running:
        errors.append("simulation must be stopped for a clean-scene acceptance audit")

    cell = _safe_get(sim, SCENE_ROOT)
    targets_root = _safe_get(sim, f"{SCENE_ROOT}/Targets")
    cell_count = (
        len(sim.getObjectsInTree(cell, sim.handle_all, 0)) if cell != -1 else 0
    )
    target_count = (
        len(sim.getObjectsInTree(targets_root, sim.object_dummy_type, 0))
        if targets_root != -1
        else 0
    )
    if cell == -1:
        errors.append(f"missing scene root {SCENE_ROOT}")
    if targets_root == -1:
        errors.append("missing target tree")
    if cell_count != int(baseline["counts"]["cell_objects"]):
        warnings.append(
            f"cell object count changed: {baseline['counts']['cell_objects']} -> {cell_count}"
        )
    if target_count != int(baseline["counts"]["target_tree_dummies"]):
        warnings.append(
            "target-tree Dummy count changed: "
            f"{baseline['counts']['target_tree_dummies']} -> {target_count}"
        )

    all_scene_objects = list(
        sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
    )
    alias_counts = {
        alias: sum(sim.getObjectAlias(handle) == alias for handle in all_scene_objects)
        for alias in SCRIPT_DUMMIES
    }
    for alias, count in alias_counts.items():
        if count != 1:
            errors.append(f"expected one {alias}, found {count}")

    joint_tolerance = float(baseline["tolerances"]["joint_deg"])
    robots = {
        robot_id: _robot_record(sim, robot_id, joint_tolerance)
        for robot_id in ROBOT_IDS
    }
    for robot_id, record in robots.items():
        if record["missing"]:
            errors.append(f"missing robot /{robot_id}")
            continue
        if record["arm_joint_aliases_missing"]:
            errors.append(
                f"{robot_id} arm joints missing: {record['arm_joint_aliases_missing']}"
            )
        if record["tip_match_count"] != 1:
            errors.append(
                f"{robot_id} expected one {ROBOT_TIPS[robot_id]}, "
                f"found {record['tip_match_count']}"
            )
        if not record["at_zero"]:
            errors.append(f"{robot_id} is not at the six-axis zero baseline")

    targets, missing_targets = _target_snapshot(sim)
    if missing_targets:
        errors.append(f"missing contract targets: {missing_targets}")
    target_changes = compare_target_snapshots(
        targets, baseline["targets"], baseline
    )
    protected_changes = [change for change in target_changes if change["protected"]]
    if protected_changes:
        errors.append(
            "protected R1/R2/R4 targets changed without an accepted baseline update"
        )
    if any(not change["protected"] for change in target_changes):
        warnings.append("R3/R5/sensor targets changed; recalibration required")

    parts_root = _safe_get(sim, f"{SCENE_ROOT}/Parts")
    part_tolerance = float(baseline["tolerances"]["part_position_m"])
    parts = {}
    for name, expected_position in baseline["initial_parts"].items():
        handle = _safe_get(sim, PARTS[name])
        if handle == -1:
            parts[name] = {"missing": True}
            errors.append(f"missing part {name}")
            continue
        position = _round_vector(sim.getObjectPosition(handle, -1))
        parent = sim.getObjectParent(handle)
        parts[name] = {
            "missing": False,
            "position": position,
            "parent": _path(sim, parent),
            "position_changed": not _near(
                position, expected_position, part_tolerance
            ),
        }
        if parent != parts_root:
            errors.append(f"{name} is not owned by /Parts")
        if parts[name]["position_changed"]:
            warnings.append(f"initial position changed for {name}")

    runtime_objects = [
        _path(sim, handle)
        for handle in all_scene_objects
        if any(marker in sim.getObjectAlias(handle) for marker in RUNTIME_MARKERS)
    ]
    if runtime_objects:
        errors.append(f"runtime objects remain in scene: {runtime_objects}")

    pcb_area = _safe_get(sim, f"{SCENE_ROOT}/Areas/PCB_Supply_Area")
    r2_base = _safe_get(sim, f"{SCENE_ROOT}/RobotBases/R2_Base")
    r2_overlap = _collision_between_shape_sets(
        sim, _tree_shapes(sim, pcb_area), _tree_shapes(sim, r2_base)
    )
    if r2_overlap["collision"]:
        errors.append("PCB_Supply_Area still overlaps R2_Base")

    assembly_product = _safe_get(sim, PARTS["ASSEMBLY_PRODUCT"])
    assembly_shapes = _tree_shapes(sim, assembly_product)
    module_shapes = [
        handle
        for handle in assembly_shapes
        if "_Control_Module_" in sim.getObjectAlias(handle)
    ]
    pcb_shapes = [
        handle
        for handle in assembly_shapes
        if "_PCB_" in sim.getObjectAlias(handle)
    ]
    r3_module_overlap = _collision_between_shape_sets(
        sim, module_shapes, pcb_shapes
    )
    if r3_module_overlap["collision"]:
        errors.append("assembly template control module still overlaps PCB geometry")

    desired_belt_z = float(baseline["r5_product_on_belt_z"])
    product_z = float(parts.get("INSPECTION_PRODUCT", {}).get("position", [0, 0, math.nan])[2])
    pick_z = float(targets.get("R5_PRODUCT_PICK_TCP", {}).get("position", [0, 0, math.nan])[2])
    r5_heights = {}
    belt_tolerance = float(baseline["tolerances"]["belt_height_error_m"])
    for branch in ("GOOD", "DEFECT"):
        place_z = float(
            targets.get(f"R5_{branch}_PLACE_TCP", {}).get(
                "position", [0, 0, math.nan]
            )[2]
        )
        record = compute_r5_height_result(
            product_z, pick_z, place_z, desired_belt_z
        )
        r5_heights[branch.lower()] = record
        if not math.isfinite(record["height_error_m"]) or abs(
            record["height_error_m"]
        ) > belt_tolerance:
            errors.append(
                f"R5 {branch.lower()} rigid placement height differs from belt "
                f"by {record['height_error_m']:.6f} m"
            )

    camera_view = _safe_get(
        sim,
        f"{SCENE_ROOT}/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area",
    )
    inspection_product = _safe_get(sim, PARTS["INSPECTION_PRODUCT"])
    camera_view_overlap = _collision_between_shape_sets(
        sim,
        _tree_shapes(sim, camera_view),
        _tree_shapes(sim, inspection_product),
    )
    if camera_view_overlap["collision"]:
        warnings.append(
            "Camera_View_Area overlaps the inspection product; make the visual volume non-collidable"
        )

    return {
        "status": "blocked" if errors else "ready_for_path_revalidation",
        "errors": errors,
        "warnings": warnings,
        "scene": {
            "audited_file": str(scene_file),
            "open_scene": str(live_scene),
            "simulation_state": simulation_state,
            "fingerprint": fingerprint,
            "baseline_fingerprint": {
                "size": expected_fingerprint["size"],
                "sha256": expected_fingerprint["sha256"],
            },
            "fingerprint_changed": fingerprint_changed,
            "cell_object_count": cell_count,
            "target_tree_dummy_count": target_count,
            "script_dummy_counts": alias_counts,
        },
        "robots": robots,
        "target_changes": target_changes,
        "parts": parts,
        "runtime_objects": runtime_objects,
        "layout_checks": {
            "r2_pcb_area_to_base": r2_overlap,
            "r3_module_to_template_pcb": r3_module_overlap,
            "r5_height": r5_heights,
            "camera_view_to_inspection_product": camera_view_overlap,
            "r5_good_path_note": (
                "target/geometry audit only; rerun full carried-product path "
                "validation after any R5 layout change"
            ),
        },
        "full_path_revalidation_required": bool(
            fingerprint_changed or target_changes
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--allow-running",
        action="store_true",
        help="report a running scene without treating its state as an error",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="print only status/errors/warnings while retaining full --output",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
    client_class = _remote_api_client_class()
    client = client_class(host=args.host, port=args.port)
    sim = client.require("sim")
    report = build_audit(
        sim,
        args.scene,
        baseline,
        allow_running=args.allow_running,
    )
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    if args.summary_only:
        summary = {
            "status": report["status"],
            "errors": report["errors"],
            "warnings": report["warnings"],
            "fingerprint_changed": report["scene"]["fingerprint_changed"],
            "target_change_count": len(report["target_changes"]),
            "runtime_objects": report["runtime_objects"],
            "full_path_revalidation_required": report[
                "full_path_revalidation_required"
            ],
        }
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        print(rendered, end="")
    return 0 if report["status"] == "ready_for_path_revalidation" else 1


if __name__ == "__main__":
    raise SystemExit(main())
