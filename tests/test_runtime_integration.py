from __future__ import annotations

import math
import tempfile
import threading
import unittest
from pathlib import Path

from interfaces.robot_interface import IRobotExecutor
from interfaces.types import Order, RobotState, Task, TaskResult, TaskStatus
from orchestration.cell_orchestrator import CellOrchestrator
from robot_control.motion_safety import motion_gate_status
from robot_control.r3_motion import (
    _quaternion_multiply,
    _rotate_vector,
    _surface_aligned_product_grasp_quaternion,
)
from robot_control.scene_aware_executor import SceneAwareExecutor
from scheduler.scheduler import Scheduler
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.process_manager import CoppeliaProcessManager
from sim_bridge.scene_objects import POINTS


class InstantExecutor(IRobotExecutor):
    def __init__(self, quality="OK", fail_process=""):
        self.quality = quality
        self.fail_process = fail_process
        self.calls: list[str] = []
        self.robots = {
            resource: RobotState(resource)
            for resource in ("R1", "R2", "R3", "R4", "R5", "CAMERA")
        }

    def execute_task(self, task):
        robot_id = task.available_robots[0]
        self.calls.append(task.task_id)
        status = (
            TaskStatus.FAILED.value
            if task.process == self.fail_process
            else TaskStatus.FINISHED.value
        )
        return TaskResult(
            task_id=task.task_id,
            robot_id=robot_id,
            status=status,
            quality_result=self.quality if task.process == "inspect" else "",
        )

    def execute_task_async(self, task, callback):
        callback(self.execute_task(task))

    def get_robot_states(self):
        return list(self.robots.values())

    def move_to_point(self, robot_id, point_name):
        return True

    def gripper_open(self, robot_id):
        return True

    def gripper_close(self, robot_id):
        return True

    def screw_execute(self, robot_id, point_name):
        return True

    def robot_home(self, robot_id):
        return True

    def set_robot_fault(self, robot_id):
        self.robots[robot_id].status = "fault"

    def clear_robot_fault(self, robot_id):
        self.robots[robot_id].status = "idle"


class GatedExecutor(InstantExecutor):
    def __init__(self):
        super().__init__("OK")
        self.entered = threading.Event()
        self.release = threading.Event()

    def execute_task(self, task):
        self.entered.set()
        self.release.wait(2.0)
        return super().execute_task(task)


class FakeSignalBridge:
    def __init__(self, fail=False):
        self.fail = fail
        self.process_commands = []
        self.quality_results = []

    def send_process_command(self, command):
        if self.fail:
            raise RuntimeError("signal rejected")
        self.process_commands.append(command)

    def send_quality_result(self, quality):
        if self.fail:
            raise RuntimeError("quality rejected")
        self.quality_results.append(quality)


class FakeSim:
    handle_all = -1
    object_joint_type = 1
    simulation_stopped = 0
    simulation_running = 1

    def __init__(self):
        self.state = self.simulation_stopped
        self.signals = {}
        self.paths = {
            "/FiveCR5A_Cell": 1,
            "/R1": 10,
            POINTS["R1_BOX_PICK_APP"]: 30,
        }
        self.aliases = {
            1: "FiveCR5A_Cell",
            10: "R1",
            17: "R1_gripper_tip",
            30: "R1_BOX_PICK_APP",
            **{handle: f"joint{handle - 10}" for handle in range(11, 17)},
        }
        self.joints = {handle: 0.0 for handle in range(11, 17)}
        self.targets = {}

    def getSimulationState(self):
        return self.state

    def getObject(self, path):
        if path not in self.paths:
            raise RuntimeError(path)
        return self.paths[path]

    def getObjectsInTree(self, root, object_type, options):
        if root == 10 and object_type == self.object_joint_type:
            return list(range(11, 17))
        if root == 10 and object_type == self.handle_all:
            return list(range(11, 18))
        return []

    def getObjectAlias(self, handle):
        return self.aliases[handle]

    def getJointPosition(self, handle):
        return self.joints[handle]

    def setJointPosition(self, handle, value):
        self.joints[handle] = value

    def setJointTargetPosition(self, handle, value):
        self.targets[handle] = value

    def getObjectPosition(self, handle, relative):
        return [1.0, 2.0, 3.0]

    def getObjectOrientation(self, handle, relative):
        return [0.1, 0.2, 0.3]

    def getObjectQuaternion(self, handle, relative):
        return [0.0, 0.0, 0.0, 1.0]

    def setStringSignal(self, name, value):
        self.signals[name] = value

    def getStringSignal(self, name):
        return self.signals.get(name)

    def clearStringSignal(self, name):
        self.signals.pop(name, None)

    def startSimulation(self):
        self.state = self.simulation_running

    def stopSimulation(self):
        self.state = self.simulation_stopped


class FakeClient:
    def __init__(self):
        self.sim = FakeSim()
        self.stepping = False
        self.steps = 0

    def require(self, name):
        return self.sim

    def setStepping(self, enabled):
        self.stepping = bool(enabled)

    def step(self):
        self.steps += 1


class RuntimeIntegrationTests(unittest.TestCase):
    def test_r3_product_grasp_aligns_pads_without_tilting_approach(self):
        half_y = math.radians(-90.0) / 2.0
        half_z = math.radians(-13.0) / 2.0
        source = _quaternion_multiply(
            (0.0, 0.0, math.sin(half_z), math.cos(half_z)),
            (0.0, math.sin(half_y), 0.0, math.cos(half_y)),
        )
        source_approach = _rotate_vector(source, (1.0, 0.0, 0.0))
        aligned = _surface_aligned_product_grasp_quaternion(source)
        aligned_approach = _rotate_vector(aligned, (1.0, 0.0, 0.0))
        aligned_closing = _rotate_vector(aligned, (0.0, 1.0, 0.0))

        for before, after in zip(source_approach, aligned_approach):
            self.assertAlmostEqual(before, after, places=9)
        self.assertAlmostEqual(abs(aligned_approach[2]), 1.0, places=9)
        self.assertAlmostEqual(abs(aligned_closing[1]), 1.0, places=9)
        self.assertAlmostEqual(aligned_closing[2], 0.0, places=9)

    def test_orchestrator_dispatches_latest_ok_chain_once(self):
        executor = InstantExecutor("OK")
        orchestrator = CellOrchestrator(Scheduler(), executor, 0.005)
        orchestrator.start([Order("A100", "A", 1)])
        self.assertEqual(orchestrator.wait(2.0), "finished")
        processes = [task.process for task in orchestrator.tasks]
        self.assertEqual(
            processes,
            [
                "box_feed",
                "pcb_install",
                "terminal_install",
                "module_install",
                "transfer_to_inspection",
                "inspect",
                "screw",
                "sort_good",
            ],
        )
        self.assertEqual(len(executor.calls), 8)
        self.assertEqual(len(set(executor.calls)), 8)
        self.assertEqual(orchestrator.dispatched_task_ids, set(executor.calls))

    def test_orchestrator_stops_on_failed_task(self):
        executor = InstantExecutor("NG", fail_process="pcb_install")
        orchestrator = CellOrchestrator(Scheduler(), executor, 0.005)
        orchestrator.start([Order("A100", "A", 1)])
        self.assertEqual(orchestrator.wait(2.0), "failed")
        self.assertEqual(executor.calls, ["T0001", "T0002"])

    def test_orchestrator_accepts_a_live_urgent_order(self):
        executor = GatedExecutor()
        orchestrator = CellOrchestrator(Scheduler(), executor, 0.005)
        orchestrator.start([Order("A100", "A", 1)])
        self.assertTrue(executor.entered.wait(1.0))
        added = orchestrator.add_order(
            Order("B900", "B", 10),
            urgent=True,
        )
        self.assertEqual(len(added), 6)
        executor.release.set()
        self.assertEqual(orchestrator.wait(4.0), "finished")
        self.assertIn("B900", {task.order_id for task in orchestrator.tasks})
        self.assertEqual(
            len(orchestrator.dispatched_task_ids),
            len(orchestrator.tasks),
        )

    def test_scene_aware_executor_commits_only_successful_results(self):
        bridge = FakeSignalBridge()
        wrapped = SceneAwareExecutor(InstantExecutor("NG"), bridge)
        task = Task(
            "T1",
            "O1",
            "A",
            "box_feed",
            "box_supply_area",
            "R1_BOX_PLACE_TCP",
            ["R1"],
            scene_command="R1_BOX_PLACED",
        )
        result = wrapped.execute_task(task)
        self.assertEqual(result.status, TaskStatus.FINISHED.value)
        self.assertEqual(bridge.process_commands, ["R1_BOX_PLACED"])

        inspect = Task(
            "T2",
            "O1",
            "A",
            "inspect",
            "camera_area",
            "CAMERA_INSPECTION_CENTER",
            ["CAMERA"],
        )
        result = wrapped.execute_task(inspect)
        self.assertEqual(result.quality_result, "NG")
        self.assertEqual(bridge.quality_results, ["NG"])

    def test_scene_update_failure_turns_task_into_failure(self):
        wrapped = SceneAwareExecutor(
            InstantExecutor("OK"),
            FakeSignalBridge(fail=True),
        )
        task = Task(
            "T1",
            "O1",
            "A",
            "box_feed",
            "box_supply_area",
            "R1_BOX_PLACE_TCP",
            ["R1"],
            scene_command="R1_BOX_PLACED",
        )
        self.assertEqual(
            wrapped.execute_task(task).status,
            TaskStatus.FAILED.value,
        )

    def test_real_bridge_supports_signals_but_refuses_cartesian_motion(self):
        client = FakeClient()
        bridge = SimBridge(
            client_factory=lambda **kwargs: client,
            validate_contract=False,
        )
        self.assertTrue(bridge.connect("localhost", 23000))
        self.assertEqual(client.timeout, 5.0)
        bridge.send_process_command("R1_BOX_PLACED")
        bridge.send_quality_result("NG")
        self.assertEqual(
            client.sim.signals["cell_product_state"],
            "camera_defect",
        )
        self.assertEqual(
            bridge.get_robot_joint_handles("R1"),
            list(range(11, 17)),
        )
        self.assertFalse(bridge.move_robot_pose("R1", 1, 2, 3))
        self.assertIn("collision-checked", bridge.last_error)

    def test_physical_gate_is_closed_while_simulation_motion_is_enabled(self):
        status = motion_gate_status()
        self.assertFalse(status["enabled"])
        self.assertFalse(status["physical_enabled"])
        self.assertTrue(status["simulation_enabled"])
        self.assertEqual(
            set(status["validated_plans"]),
            {"R1", "R2", "R3", "R4", "R5"},
        )

    def test_coppelia_process_manager_launches_configured_scene(self):
        calls = []

        class FakeProcess:
            returncode = None

            def poll(self):
                return None

            def terminate(self):
                self.returncode = 0

        def fake_popen(command, **kwargs):
            calls.append((command, kwargs))
            return FakeProcess()

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            executable = root / "coppeliaSim.sh"
            scene = root / "cell.ttt"
            executable.write_text("#!/bin/sh\n", encoding="utf-8")
            scene.write_bytes(b"scene")
            config = root / "runtime.yaml"
            config.write_text(
                "coppeliasim:\n"
                f"  executable: {executable}\n"
                f"  scene: {scene}\n"
                "  host: 127.0.0.2\n"
                "  port: 24000\n",
                encoding="utf-8",
            )
            manager = CoppeliaProcessManager(config, fake_popen)
            manager.launch()

        self.assertEqual(calls[0][0], [str(executable), str(scene)])
        self.assertEqual(calls[0][1]["cwd"], str(executable.parent))
        self.assertTrue(calls[0][1]["start_new_session"])


if __name__ == "__main__":
    unittest.main()
