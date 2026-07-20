from __future__ import annotations

import threading
import unittest

from interfaces.robot_interface import IRobotExecutor
from interfaces.types import RobotStatus, Task, TaskStatus
from robot_control.r1_motion import (
    R1_BOX_PLACED,
    R1_COMPLETE_CYCLE,
    R1_TERMINAL_PLACED,
    load_r1_plan,
)
from robot_control.r2_motion import R2_PCB_PLACED
from robot_control.r3_motion import R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION
from robot_control.r4_motion import R4_SCREW_DONE, R4MotionController
from robot_control.r5_motion import (
    GOOD_RUNTIME_XY_OFFSET_M,
    PROTECTED_TARGETS,
    R5_SORT_DEFECT_DONE,
    R5_SORT_GOOD_DONE,
    R5MotionController,
)
from robot_control.robot_executor import RobotExecutor


class FakeBridge:
    def __init__(self):
        self.connected = False
        self.last_error = ""
        self.gripper_calls: list[tuple[str, bool]] = []

    def is_connected(self):
        return self.connected

    def connect(self):
        self.connected = True
        return True

    def set_gripper(self, robot_id, opened):
        self.gripper_calls.append((robot_id, opened))
        return robot_id == "R1"


class FakeR5TargetBridge:
    def __init__(self):
        self.positions = {
            name: list(position) for name, position in PROTECTED_TARGETS.items()
        }

    def get_target_pose(self, name):
        return {"position": self.positions[name], "orientation": [0.0] * 3}


class StubMotionController:
    actions: list[str] = []
    prepare_actions: list[str] = []
    continuous_values: list[bool] = []
    enter_ready_count = 0
    fail_with: Exception | None = None
    kwargs_log: list[dict] = []

    def __init__(self, bridge, **kwargs):
        self.bridge = bridge
        self.kwargs = kwargs
        self.kwargs_log.append(kwargs)

    def execute(self, action):
        self.actions.append(action)
        if self.fail_with is not None:
            raise self.fail_with
        return {"action": action}

    def prepare(self, action):
        self.prepare_actions.append(action)
        return {"robot_id": "R1", "path_points": {"cached": 10}}

    def set_continuous_stepping(self, enabled):
        self.continuous_values.append(bool(enabled))

    def enter_ready(self):
        type(self).enter_ready_count += 1
        return {"stepping_held": True}


class StubR2MotionController:
    actions: list[str] = []
    prepare_actions: list[str] = []
    continuous_values: list[bool] = []
    fail_with: Exception | None = None
    kwargs_log: list[dict] = []

    def __init__(self, bridge, **kwargs):
        self.bridge = bridge
        self.kwargs = kwargs
        self.kwargs_log.append(kwargs)

    def execute(self, action):
        self.actions.append(action)
        if self.fail_with is not None:
            raise self.fail_with
        return {"action": action}

    def prepare(self, action):
        self.prepare_actions.append(action)
        return {"robot_id": "R2", "path_points": {"cached": 20}}

    def set_continuous_stepping(self, enabled):
        self.continuous_values.append(bool(enabled))


class StubR4MotionController:
    actions: list[str] = []
    prepare_actions: list[str] = []
    continuous_values: list[bool] = []
    fail_with: Exception | None = None
    kwargs_log: list[dict] = []

    def __init__(self, bridge, **kwargs):
        self.bridge = bridge
        self.kwargs = kwargs
        self.kwargs_log.append(kwargs)

    def execute(self, action):
        self.actions.append(action)
        if self.fail_with is not None:
            raise self.fail_with
        return {"action": action}

    def prepare(self, action):
        self.prepare_actions.append(action)
        return {
            "robot_id": action.split("_", 1)[0],
            "path_points": {"cached": 30},
        }

    def set_continuous_stepping(self, enabled):
        self.continuous_values.append(bool(enabled))


class StubR3MotionController(StubR4MotionController):
    actions: list[str] = []
    prepare_actions: list[str] = []
    continuous_values: list[bool] = []
    fail_with: Exception | None = None
    kwargs_log: list[dict] = []


class StubR5MotionController(StubR4MotionController):
    actions: list[str] = []
    prepare_actions: list[str] = []
    continuous_values: list[bool] = []
    fail_with: Exception | None = None
    kwargs_log: list[dict] = []


def make_task(
    target_point: str,
    robots: list[str] | None = None,
    process: str = "assemble",
    task_id: str = "T-R1",
) -> Task:
    return Task(
        task_id=task_id,
        order_id="ORDER-1",
        product_type="A",
        process=process,
        target_area="assembly_area",
        target_point=target_point,
        available_robots=list(["R1"] if robots is None else robots),
    )


class RobotExecutorTests(unittest.TestCase):
    def setUp(self):
        StubMotionController.actions = []
        StubMotionController.prepare_actions = []
        StubMotionController.continuous_values = []
        StubMotionController.enter_ready_count = 0
        StubMotionController.fail_with = None
        StubMotionController.kwargs_log = []
        StubR2MotionController.actions = []
        StubR2MotionController.prepare_actions = []
        StubR2MotionController.continuous_values = []
        StubR2MotionController.fail_with = None
        StubR2MotionController.kwargs_log = []
        StubR4MotionController.actions = []
        StubR4MotionController.prepare_actions = []
        StubR4MotionController.continuous_values = []
        StubR4MotionController.fail_with = None
        StubR4MotionController.kwargs_log = []
        StubR3MotionController.actions = []
        StubR3MotionController.prepare_actions = []
        StubR3MotionController.continuous_values = []
        StubR3MotionController.fail_with = None
        StubR3MotionController.kwargs_log = []
        StubR5MotionController.actions = []
        StubR5MotionController.prepare_actions = []
        StubR5MotionController.continuous_values = []
        StubR5MotionController.fail_with = None
        StubR5MotionController.kwargs_log = []
        self.bridge = FakeBridge()
        self.executor = RobotExecutor(
            sim_bridge=self.bridge,
            motion_controller_factory=StubMotionController,
            r2_motion_controller_factory=StubR2MotionController,
            r3_motion_controller_factory=StubR3MotionController,
            r4_motion_controller_factory=StubR4MotionController,
            r5_motion_controller_factory=StubR5MotionController,
        )

    def test_implements_interface_and_reports_five_robots(self):
        self.assertIsInstance(self.executor, IRobotExecutor)
        states = self.executor.get_robot_states()
        self.assertEqual([state.robot_id for state in states], ["R1", "R2", "R3", "R4", "R5"])
        self.assertTrue(all(state.status == RobotStatus.IDLE.value for state in states))

    def test_prepare_cycle_caches_all_controllers_and_both_r5_branches(self):
        evidence = self.executor.prepare_cycle(
            quality="good", preload_both_r5=True
        )

        self.assertTrue(evidence["ready"])
        self.assertEqual(evidence["path_points_total"], 180)
        self.assertEqual(StubMotionController.prepare_actions, [R1_BOX_PLACED])
        self.assertEqual(StubR2MotionController.prepare_actions, [R2_PCB_PLACED])
        self.assertEqual(
            StubR3MotionController.prepare_actions,
            [R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION],
        )
        self.assertEqual(StubR4MotionController.prepare_actions, [R4_SCREW_DONE])
        self.assertEqual(
            StubR5MotionController.prepare_actions,
            [R5_SORT_GOOD_DONE, R5_SORT_DEFECT_DONE],
        )
        self.assertEqual(StubMotionController.continuous_values, [True])
        self.assertEqual(StubR2MotionController.continuous_values, [True])
        self.assertEqual(StubR3MotionController.continuous_values, [True])
        self.assertEqual(StubR4MotionController.continuous_values, [True])
        self.assertEqual(StubR5MotionController.continuous_values, [False])
        self.assertEqual(StubMotionController.enter_ready_count, 1)

        self.executor.execute_task(make_task(R1_BOX_PLACED))
        self.executor.execute_task(
            make_task(R3_MODULE_PLACED, robots=["R3"])
        )
        self.assertEqual(len(StubMotionController.kwargs_log), 1)
        self.assertEqual(len(StubR3MotionController.kwargs_log), 1)

    def test_prepare_cycle_rejects_unknown_quality_before_scene_changes(self):
        with self.assertRaisesRegex(ValueError, "quality"):
            self.executor.prepare_cycle("maybe")
        self.assertFalse(self.bridge.connected)

    def test_executes_exact_r1_box_command(self):
        result = self.executor.execute_task(make_task(R1_BOX_PLACED))
        self.assertEqual(result.status, TaskStatus.FINISHED.value)
        self.assertEqual(result.robot_id, "R1")
        self.assertEqual(StubMotionController.actions, [R1_BOX_PLACED])
        state = self.executor.get_robot_states()[0]
        self.assertEqual(state.position, "R1_TERMINAL_PICK_APP")
        self.assertEqual(state.completed_tasks, 1)
        self.assertIn("physical grasp not validated", result.message)

    def test_accepts_action_from_process_or_task_id(self):
        from_process = make_task(
            "R1_TERMINAL_PLACE_TCP", process=R1_TERMINAL_PLACED
        )
        from_task_id = make_task(
            "R1_TERMINAL_PLACE_TCP",
            task_id=R1_COMPLETE_CYCLE,
        )
        self.assertEqual(
            self.executor.execute_task(from_process).status,
            TaskStatus.FINISHED.value,
        )
        self.assertEqual(
            self.executor.execute_task(from_task_id).status,
            TaskStatus.FINISHED.value,
        )
        self.assertEqual(
            StubMotionController.actions,
            [R1_TERMINAL_PLACED, R1_COMPLETE_CYCLE],
        )

    def test_rejects_unknown_task_without_placeholder_success(self):
        result = self.executor.execute_task(
            make_task("R3_UNKNOWN_ACTION", robots=["R3"])
        )
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("unsupported task", result.message)
        self.assertEqual(StubMotionController.actions, [])
        self.assertEqual(StubR2MotionController.actions, [])

    def test_executes_exact_r3_actions_and_passes_both_shared_locks(self):
        module = self.executor.execute_task(
            make_task(R3_MODULE_PLACED, robots=["R3"], task_id="T-R3-MODULE")
        )
        transfer = self.executor.execute_task(
            make_task(
                R3_PRODUCT_TO_INSPECTION,
                robots=["R3"],
                task_id="T-R3-TRANSFER",
            )
        )
        self.assertEqual(module.status, TaskStatus.FINISHED.value)
        self.assertEqual(transfer.status, TaskStatus.FINISHED.value)
        self.assertEqual(
            StubR3MotionController.actions,
            [R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION],
        )
        kwargs = StubR3MotionController.kwargs_log[0]
        self.assertIs(kwargs["assembly_lock"], self.executor._assembly_lock)
        self.assertIs(kwargs["inspection_lock"], self.executor._inspection_lock)
        self.assertIn("physical grasp not validated", transfer.message)

    def test_rejects_r3_action_assigned_to_another_robot(self):
        result = self.executor.execute_task(
            make_task(R3_MODULE_PLACED, robots=["R2"])
        )
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("assigned to R3", result.message)
        self.assertEqual(StubR3MotionController.actions, [])

    def test_rejects_r1_action_assigned_to_another_robot(self):
        result = self.executor.execute_task(
            make_task(R1_BOX_PLACED, robots=["R2"])
        )
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("assigned to R1", result.message)

    def test_executes_exact_r2_pcb_command_and_updates_r2_state(self):
        result = self.executor.execute_task(
            make_task(R2_PCB_PLACED, robots=["R2"], task_id="T-R2")
        )
        self.assertEqual(result.status, TaskStatus.FINISHED.value)
        self.assertEqual(result.robot_id, "R2")
        self.assertEqual(StubMotionController.actions, [])
        self.assertEqual(StubR2MotionController.actions, [R2_PCB_PLACED])
        state = self.executor.get_robot_states()[1]
        self.assertEqual(state.status, RobotStatus.IDLE.value)
        self.assertEqual(state.position, "home")
        self.assertEqual(state.completed_tasks, 1)
        self.assertIn("visual suction", result.message)
        self.assertIn("physical grasp not validated", result.message)

    def test_accepts_r2_action_from_process_task_id_or_inference(self):
        from_process = make_task(
            "R2_PCB_PLACE_TCP",
            robots=["R2"],
            process=R2_PCB_PLACED,
            task_id="T-R2-PROCESS",
        )
        from_task_id = make_task(
            "R2_PCB_PLACE_TCP",
            robots=[],
            task_id=R2_PCB_PLACED,
        )
        self.assertEqual(
            self.executor.execute_task(from_process).status,
            TaskStatus.FINISHED.value,
        )
        inferred = self.executor.execute_task(from_task_id)
        self.assertEqual(inferred.status, TaskStatus.FINISHED.value)
        self.assertEqual(inferred.robot_id, "R2")
        self.assertEqual(
            StubR2MotionController.actions,
            [R2_PCB_PLACED, R2_PCB_PLACED],
        )

    def test_rejects_r2_action_assigned_to_another_robot(self):
        result = self.executor.execute_task(
            make_task(R2_PCB_PLACED, robots=["R1"])
        )
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("assigned to R2", result.message)
        self.assertEqual(StubR2MotionController.actions, [])

    def test_executes_exact_r4_screw_command_and_updates_r4_state(self):
        result = self.executor.execute_task(
            make_task(R4_SCREW_DONE, robots=["R4"], task_id="T-R4")
        )
        self.assertEqual(result.status, TaskStatus.FINISHED.value)
        self.assertEqual(result.robot_id, "R4")
        self.assertEqual(StubR4MotionController.actions, [R4_SCREW_DONE])
        state = self.executor.get_robot_states()[3]
        self.assertEqual(state.status, RobotStatus.IDLE.value)
        self.assertEqual(state.position, "home")
        self.assertEqual(state.completed_tasks, 1)
        self.assertIn("visual screwdriver", result.message)
        self.assertIn("physical torque not validated", result.message)

    def test_rejects_r4_action_assigned_to_another_robot(self):
        result = self.executor.execute_task(
            make_task(R4_SCREW_DONE, robots=["R3"])
        )
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("assigned to R4", result.message)
        self.assertEqual(StubR4MotionController.actions, [])

    def test_executes_both_r5_sort_branches_with_inspection_lock(self):
        good = self.executor.execute_task(
            make_task(R5_SORT_GOOD_DONE, robots=["R5"], task_id="T-R5-GOOD")
        )
        defect = self.executor.execute_task(
            make_task(
                R5_SORT_DEFECT_DONE,
                robots=["R5"],
                task_id="T-R5-DEFECT",
            )
        )
        self.assertEqual(good.status, TaskStatus.FINISHED.value)
        self.assertEqual(defect.status, TaskStatus.FINISHED.value)
        self.assertEqual(
            StubR5MotionController.actions,
            [R5_SORT_GOOD_DONE, R5_SORT_DEFECT_DONE],
        )
        kwargs = StubR5MotionController.kwargs_log[0]
        self.assertIs(kwargs["inspection_lock"], self.executor._inspection_lock)
        self.assertIn("physical grasp not validated", good.message)

    def test_rejects_r5_action_assigned_to_another_robot(self):
        result = self.executor.execute_task(
            make_task(R5_SORT_GOOD_DONE, robots=["R4"])
        )
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("assigned to R5", result.message)
        self.assertEqual(StubR5MotionController.actions, [])

    def test_r4_controller_receives_inspection_lock_and_plan_path(self):
        result = self.executor.execute_task(
            make_task(R4_SCREW_DONE, robots=["R4"])
        )
        self.assertEqual(result.status, TaskStatus.FINISHED.value)
        kwargs = StubR4MotionController.kwargs_log[0]
        self.assertIs(kwargs["inspection_lock"], self.executor._inspection_lock)
        self.assertIn("r1_plan_path", kwargs)

    def test_r1_and_r2_controllers_receive_the_same_assembly_lock(self):
        self.assertEqual(
            self.executor.execute_task(make_task(R1_BOX_PLACED)).status,
            TaskStatus.FINISHED.value,
        )
        self.assertEqual(
            self.executor.execute_task(
                make_task(R2_PCB_PLACED, robots=["R2"])
            ).status,
            TaskStatus.FINISHED.value,
        )
        r1_lock = StubMotionController.kwargs_log[0]["assembly_lock"]
        r2_lock = StubR2MotionController.kwargs_log[0]["assembly_lock"]
        self.assertIs(r1_lock, r2_lock)
        self.assertIn("plan_path", StubMotionController.kwargs_log[0])
        self.assertIn("r1_plan_path", StubR2MotionController.kwargs_log[0])

    def test_motion_failure_restores_idle_without_completion(self):
        StubMotionController.fail_with = RuntimeError("collision test failure")
        result = self.executor.execute_task(make_task(R1_BOX_PLACED))
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("collision test failure", result.message)
        state = self.executor.get_robot_states()[0]
        self.assertEqual(state.status, RobotStatus.IDLE.value)
        self.assertEqual(state.completed_tasks, 0)

    def test_r2_motion_failure_restores_idle_without_completion(self):
        StubR2MotionController.fail_with = RuntimeError(
            "R2 collision test failure"
        )
        result = self.executor.execute_task(
            make_task(R2_PCB_PLACED, robots=["R2"])
        )
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("R2 collision test failure", result.message)
        state = self.executor.get_robot_states()[1]
        self.assertEqual(state.status, RobotStatus.IDLE.value)
        self.assertIsNone(state.current_task)
        self.assertEqual(state.position, "home")
        self.assertEqual(state.completed_tasks, 0)

    def test_r4_motion_failure_restores_idle_without_completion(self):
        StubR4MotionController.fail_with = RuntimeError(
            "R4 tool collision test failure"
        )
        result = self.executor.execute_task(
            make_task(R4_SCREW_DONE, robots=["R4"])
        )
        self.assertEqual(result.status, TaskStatus.FAILED.value)
        self.assertIn("R4 tool collision test failure", result.message)
        state = self.executor.get_robot_states()[3]
        self.assertEqual(state.status, RobotStatus.IDLE.value)
        self.assertIsNone(state.current_task)
        self.assertEqual(state.position, "home")
        self.assertEqual(state.completed_tasks, 0)

    def test_faulted_robot_refuses_task_until_cleared(self):
        self.executor.set_robot_fault("R1")
        failed = self.executor.execute_task(make_task(R1_BOX_PLACED))
        self.assertEqual(failed.status, TaskStatus.FAILED.value)
        self.executor.clear_robot_fault("R1")
        finished = self.executor.execute_task(make_task(R1_BOX_PLACED))
        self.assertEqual(finished.status, TaskStatus.FINISHED.value)

    def test_async_execution_calls_back(self):
        event = threading.Event()
        results = []

        def callback(result):
            results.append(result)
            event.set()

        self.executor.execute_task_async(make_task(R1_BOX_PLACED), callback)
        self.assertTrue(event.wait(2.0))
        self.assertEqual(results[0].status, TaskStatus.FINISHED.value)

    def test_low_level_methods_are_conservative(self):
        self.assertTrue(self.executor.robot_home("R1"))
        self.assertFalse(self.executor.move_to_point("R1", "R1_BOX_PICK_APP"))
        self.assertTrue(self.executor.screw_execute("R4", "R4_SCREW_TCP"))
        self.assertFalse(self.executor.screw_execute("R3", "R4_SCREW_TCP"))
        self.assertTrue(self.executor.gripper_open("R1"))
        self.assertFalse(self.executor.gripper_close("R2"))
        self.assertEqual(StubR4MotionController.actions, [R4_SCREW_DONE])
        self.assertEqual(
            self.bridge.gripper_calls, [("R1", True), ("R2", False)]
        )

    def test_checked_in_plan_has_required_safety_metadata(self):
        plan = load_r1_plan()
        self.assertEqual(set(plan["paths"]), {
            "initial_to_box_pick_app",
            "box_descend",
            "box_lift_and_transfer",
            "box_place_descend",
            "box_retreat_and_terminal_approach",
            "terminal_descend",
            "terminal_lift_and_transfer",
            "terminal_place_descend",
            "return_home",
        })
        self.assertTrue(plan["validation"]["collision_free"])
        self.assertEqual(
            plan["validation"]["workspace_bounds_observed"]["lower"],
            [-1.986598, 0.045463, 0.158333],
        )

    def test_r4_runtime_tool_cleanup_removes_children_first(self):
        class RemovalSim:
            def __init__(self):
                self.removed = []

            def removeObjects(self, handles):
                self.removed.extend(handles)

        sim = RemovalSim()
        R4MotionController._remove_runtime_tool(
            sim, {"objects": [1, 2, 3, 4, 5]}
        )
        self.assertEqual(sim.removed, [5, 4, 3, 2, 1])


class R5MotionGeometryTests(unittest.TestCase):
    def setUp(self):
        self.bridge = FakeR5TargetBridge()
        self.controller = R5MotionController(self.bridge)

    def test_good_runtime_offset_does_not_mutate_git_targets(self):
        positions = self.controller._positions(R5_SORT_GOOD_DONE)

        self.assertEqual(positions[0], PROTECTED_TARGETS["R5_PRODUCT_PICK_APP"])
        self.assertEqual(positions[1], PROTECTED_TARGETS["R5_PRODUCT_PICK_TCP"])
        self.assertEqual(positions[2], [0.64, -1.08, 0.62])
        self.assertEqual(positions[3], [0.64, -1.08, 0.42])
        self.assertEqual(GOOD_RUNTIME_XY_OFFSET_M, (-0.010, 0.020))
        self.assertEqual(
            self.bridge.positions["R5_GOOD_PLACE_APP"], [0.65, -1.10, 0.62]
        )
        self.assertEqual(
            self.bridge.positions["R5_GOOD_PLACE_TCP"], [0.65, -1.10, 0.42]
        )

    def test_defect_targets_do_not_receive_good_runtime_offset(self):
        positions = self.controller._positions(R5_SORT_DEFECT_DONE)

        self.assertEqual(positions[2], [-0.35, -1.12, 0.62])
        self.assertEqual(positions[3], [-0.35, -1.12, 0.42])

    def test_parallel_yaw_error_treats_opposite_directions_as_parallel(self):
        self.assertAlmostEqual(
            self.controller._parallel_yaw_error_deg(-90.0, -90.0), 0.0
        )
        self.assertAlmostEqual(
            self.controller._parallel_yaw_error_deg(90.0, -90.0), 0.0
        )
        self.assertAlmostEqual(
            self.controller._parallel_yaw_error_deg(-123.0, -90.0), 33.0
        )

    def test_rigid_payload_pose_error_uses_all_seven_components(self):
        grasp = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
        release = list(grasp)
        release[5] += 2e-10

        self.assertAlmostEqual(
            self.controller._pose_max_error(grasp, release), 2e-10
        )
        with self.assertRaisesRegex(ValueError, "seven values"):
            self.controller._pose_max_error(grasp[:6], release)


if __name__ == "__main__":
    unittest.main()
