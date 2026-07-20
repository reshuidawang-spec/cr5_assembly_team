#!/usr/bin/env python3
"""Repeat the formal R4 task from a freshly reloaded scene and log evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_bridge.coppelia_client import _remote_api_client_class
from sim_bridge.scene_objects import PARTS, ROBOT_BASES


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = REPO_ROOT / "scenes" / "five_cr5a_cell.ttt"
DEFAULT_OUTPUT = REPO_ROOT / "data" / "logs" / "r4_repeat_acceptance.json"
EXPECTED_SIZE = 2875907
EXPECTED_SHA256 = "0e1c1b8ac6b0e9a7cdf1a49cc9abce85243fd5c03c5b38563d3e3cf3433af657"
HOME_TOLERANCE_DEG = 0.01
PRODUCT_POSITION = (0.15, 0.05, 0.216)
POSITION_TOLERANCE_M = 0.002


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _client(host: str, port: int) -> tuple[Any, Any]:
    client = _remote_api_client_class()(host=host, port=port)
    return client, client.require("sim")


def _robot_joints(sim: Any, robot_id: str) -> list[int]:
    robot = sim.getObject(f"/{robot_id}")
    by_alias = {
        sim.getObjectAlias(handle): handle
        for handle in sim.getObjectsInTree(robot, sim.object_joint_type, 0)
    }
    return [by_alias[f"joint{index}"] for index in range(1, 7)]


def _runtime_objects(sim: Any) -> list[str]:
    return [
        sim.getObjectAlias(handle, 1)
        for handle in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
        if sim.getObjectAlias(handle).startswith("R4_Runtime_")
    ]


def _near(first: list[float], second: list[float], tolerance: float) -> bool:
    return len(first) == len(second) and max(
        abs(a - b) for a, b in zip(first, second)
    ) <= tolerance


def _wait_stopped(sim: Any, timeout_seconds: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_seconds
    while (
        sim.getSimulationState() != sim.simulation_stopped
        and time.monotonic() < deadline
    ):
        time.sleep(0.05)
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError("simulation did not stop within 30 seconds")


def reload_clean_scene(host: str, port: int, scene: Path) -> dict[str, Any]:
    _, sim = _client(host, port)
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        _wait_stopped(sim)
    sim.loadScene(str(scene))

    open_scene = Path(
        sim.getStringParam(sim.stringparam_scene_path_and_name)
    ).resolve()
    if open_scene != scene.resolve():
        raise RuntimeError(f"unexpected open scene after reload: {open_scene}")
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError("reloaded scene is not stopped")

    joints = _robot_joints(sim, "R4")
    joint_degrees = [
        math.degrees(float(sim.getJointPosition(handle))) for handle in joints
    ]
    if max(abs(value) for value in joint_degrees) > HOME_TOLERANCE_DEG:
        raise RuntimeError(f"R4 is not zero after reload: {joint_degrees}")
    product = sim.getObject(PARTS["INSPECTION_PRODUCT"])
    parts = sim.getObject(PARTS["INSPECTION_PRODUCT"].rsplit("/", 1)[0])
    product_position = [
        float(value) for value in sim.getObjectPosition(product, -1)
    ]
    if sim.getObjectParent(product) != parts:
        raise RuntimeError("inspection product is not owned by /Parts")
    if not _near(product_position, list(PRODUCT_POSITION), POSITION_TOLERANCE_M):
        raise RuntimeError(
            f"inspection product is not at its clean position: {product_position}"
        )
    runtime = _runtime_objects(sim)
    if runtime:
        raise RuntimeError(f"R4 runtime objects remain after reload: {runtime}")
    return {
        "scene": str(open_scene),
        "simulation_state": int(sim.getSimulationState()),
        "r4_joint_deg": joint_degrees,
        "inspection_product_position": product_position,
        "runtime_objects": runtime,
    }


def _postflight_collision(sim: Any) -> dict[str, Any]:
    robot = sim.getObject("/R4")
    robot_shapes = set(
        sim.getObjectsInTree(robot, sim.object_shape_type, 0)
    )
    robot_base = sim.getObject(ROBOT_BASES["R4"])
    mover = sim.createCollection(1)
    environment = sim.createCollection(1)
    try:
        for handle in robot_shapes:
            sim.addItemToCollection(
                mover, sim.handle_single, handle, 0
            )
        for handle in sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0
        ):
            if handle in robot_shapes or handle == robot_base:
                continue
            if sim.getObjectInt32Param(
                handle, sim.objintparam_visibility_layer
            ) == 0:
                continue
            sim.addItemToCollection(
                environment, sim.handle_single, handle, 0
            )
        state, pair = sim.checkCollision(mover, environment)

        aliases = ["base_link_respondable"] + [
            f"Link{index}_respondable" for index in range(1, 7)
        ]
        by_alias = {
            sim.getObjectAlias(handle): handle for handle in robot_shapes
        }
        links = [by_alias[alias] for alias in aliases]
        self_pair = None
        for index, first in enumerate(links):
            for second in links[index + 2 :]:
                self_state, self_handles = sim.checkCollision(first, second)
                if self_state:
                    self_pair = [
                        sim.getObjectAlias(handle, 1)
                        for handle in self_handles
                    ]
                    break
            if self_pair is not None:
                break
        return {
            "environment_collision": [
                sim.getObjectAlias(handle, 1) for handle in pair
            ]
            if state
            else None,
            "self_collision": self_pair,
        }
    finally:
        sim.destroyCollection(mover)
        sim.destroyCollection(environment)


def postflight(host: str, port: int) -> dict[str, Any]:
    _, sim = _client(host, port)
    state = int(sim.getSimulationState())
    before = float(sim.getSimulationTime())
    time.sleep(0.2)
    after = float(sim.getSimulationTime())
    joints = _robot_joints(sim, "R4")
    joint_degrees = [
        math.degrees(float(sim.getJointPosition(handle))) for handle in joints
    ]
    runtime = _runtime_objects(sim)
    product = sim.getObject(PARTS["INSPECTION_PRODUCT"])
    product_shapes = sim.getObjectsInTree(product, sim.object_shape_type, 0)
    collision = _postflight_collision(sim)
    result = {
        "simulation_state": state,
        "simulation_time_before": before,
        "simulation_time_after": after,
        "simulation_time_advanced": after > before,
        "r4_joint_deg": joint_degrees,
        "max_home_error_deg": max(abs(value) for value in joint_degrees),
        "runtime_objects": runtime,
        "inspection_product_position": [
            float(value) for value in sim.getObjectPosition(product, -1)
        ],
        "inspection_product_parent": sim.getObjectAlias(
            sim.getObjectParent(product), 1
        ),
        "inspection_product_visible_shapes": sum(
            sim.getObjectInt32Param(
                handle, sim.objintparam_visibility_layer
            )
            != 0
            for handle in product_shapes
        ),
        **collision,
    }
    failures = []
    if state == sim.simulation_stopped:
        failures.append("simulation stopped unexpectedly")
    if not result["simulation_time_advanced"]:
        failures.append("simulation time did not advance after task")
    if result["max_home_error_deg"] > HOME_TOLERANCE_DEG:
        failures.append("R4 home error exceeded tolerance")
    if runtime:
        failures.append("R4 runtime objects remain")
    if result["environment_collision"] is not None:
        failures.append("R4 environment collision remains")
    if result["self_collision"] is not None:
        failures.append("R4 self collision remains")
    if not _near(
        result["inspection_product_position"],
        list(PRODUCT_POSITION),
        POSITION_TOLERANCE_M,
    ):
        failures.append("inspection product moved")
    if result["inspection_product_parent"] != "/Parts":
        failures.append("inspection product parent changed")
    if result["inspection_product_visible_shapes"] != len(product_shapes):
        failures.append("inspection product is not fully visible")
    result["failures"] = failures
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=8)
    parser.add_argument("--prior-successes", type=int, default=2)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.runs <= 0 or args.prior_successes < 0:
        raise SystemExit("runs must be positive and prior-successes non-negative")
    scene = args.scene.resolve()
    fingerprint = {"size": scene.stat().st_size, "sha256": _sha256(scene)}
    if fingerprint != {"size": EXPECTED_SIZE, "sha256": EXPECTED_SHA256}:
        raise RuntimeError(f"scene fingerprint differs: {fingerprint}")

    evidence: dict[str, Any] = {
        "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "scene": str(scene),
        "scene_fingerprint": fingerprint,
        "prior_successes": args.prior_successes,
        "requested_new_runs": args.runs,
        "runs": [],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    successful_new_runs = 0
    try:
        for index in range(1, args.runs + 1):
            overall_index = args.prior_successes + index
            print(
                f"[R4 ACCEPTANCE] preparing run {overall_index}/"
                f"{args.prior_successes + args.runs}",
                flush=True,
            )
            record: dict[str, Any] = {
                "new_run_index": index,
                "overall_run_index": overall_index,
            }
            record["preflight"] = reload_clean_scene(
                args.host, args.port, scene
            )
            started = time.time()
            process = subprocess.run(
                [
                    sys.executable,
                    "robot_control/run_r4_task.py",
                    "R4_SCREW_DONE",
                    "--host",
                    args.host,
                    "--port",
                    str(args.port),
                    "--task-id",
                    f"R4-ACCEPTANCE-{overall_index:02d}",
                ],
                cwd=REPO_ROOT,
                text=True,
                capture_output=True,
            )
            record["subprocess_wall_time_s"] = time.time() - started
            record["command_exit_code"] = process.returncode
            record["stderr"] = process.stderr
            try:
                record["task_result"] = json.loads(process.stdout)
            except json.JSONDecodeError:
                record["task_result"] = None
                record["stdout"] = process.stdout
            record["postflight"] = postflight(args.host, args.port)
            task_finished = (
                process.returncode == 0
                and record["task_result"] is not None
                and record["task_result"].get("status") == "finished"
            )
            postflight_ok = not record["postflight"]["failures"]
            record["success"] = bool(task_finished and postflight_ok)
            evidence["runs"].append(record)
            args.output.write_text(
                json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            if not record["success"]:
                print(
                    f"[R4 ACCEPTANCE] run {overall_index} failed: "
                    f"task_finished={task_finished}, "
                    f"postflight={record['postflight']['failures']}",
                    flush=True,
                )
                break
            successful_new_runs += 1
            print(
                f"[R4 ACCEPTANCE] run {overall_index} passed: "
                f"wall={record['subprocess_wall_time_s']:.2f}s, "
                f"home_error={record['postflight']['max_home_error_deg']:.6f}deg",
                flush=True,
            )
    finally:
        evidence["finished_at"] = datetime.now().astimezone().isoformat(
            timespec="seconds"
        )
        evidence["successful_new_runs"] = successful_new_runs
        evidence["total_successes"] = args.prior_successes + successful_new_runs
        evidence["acceptance_passed"] = (
            successful_new_runs == args.runs
            and evidence["total_successes"]
            == args.prior_successes + args.runs
        )
        args.output.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    print(
        f"[R4 ACCEPTANCE] total={evidence['total_successes']}/"
        f"{args.prior_successes + args.runs}, "
        f"log={args.output}",
        flush=True,
    )
    return 0 if evidence["acceptance_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
