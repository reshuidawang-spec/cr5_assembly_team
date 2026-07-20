from __future__ import annotations

import unittest

from interfaces.types import RobotState, TaskResult, TaskStatus
from robot_control.five_arm_coordinator import FiveArmCoordinator
from robot_control.r1_motion import R1_BOX_PLACED, R1_TERMINAL_PLACED
from robot_control.r2_motion import R2_PCB_PLACED
from robot_control.r3_motion import R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION
from robot_control.r4_motion import R4_SCREW_DONE
from robot_control.r5_motion import R5_SORT_DEFECT_DONE, R5_SORT_GOOD_DONE


class FakeSim:
    def __init__(self):
        self.simulation_time = 0.0

    def getSimulationTime(self):
        return self.simulation_time


class FakeBridge:
    def __init__(self):
        self.sim = FakeSim()
        self.connected = True
        self.last_error = ""
        self.signals = []
        self.stepping = []

    def is_connected(self):
        return self.connected

    def connect(self):
        self.connected = True
        return True

    def scene_path(self):
        return "/tmp/five_cr5a_cell.ttt"

    def set_stepping(self, enabled):
        self.stepping.append(bool(enabled))

    def set_string_signal(self, name, value):
        self.signals.append((name, value))

    def step(self):
        self.sim.simulation_time += 0.05
        return True


class FakeExecutor:
    def __init__(self, bridge, fail_action=None):
        self.bridge = bridge
        self.fail_action = fail_action
        self.actions = []

    def execute_task(self, task):
        self.actions.append(task.target_point)
        self.bridge.sim.simulation_time += 1.0
        status = (
            TaskStatus.FAILED.value
            if task.target_point == self.fail_action
            else TaskStatus.FINISHED.value
        )
        return TaskResult(
            task_id=task.task_id,
            robot_id=task.available_robots[0],
            status=status,
            message=task.target_point,
        )

    def get_robot_states(self):
        return [RobotState(robot_id=f"R{index}") for index in range(1, 6)]


class FakeMotionMonitor:
    def __init__(self, bridge, robot_id):
        self.bridge = bridge
        self.robot_id = robot_id
        self.first_motion_simulation_time_s = None

    def start(self):
        self.first_motion_simulation_time_s = (
            self.bridge.sim.simulation_time + 0.1
        )

    def stop(self):
        return {
            "robot_id": self.robot_id,
            "motion_detected": True,
            "dispatch_to_first_motion_wall_s": 0.01,
            "first_motion_simulation_time_s": (
                self.first_motion_simulation_time_s
            ),
            "monitor_error": "",
        }


class FiveArmCoordinatorTests(unittest.TestCase):
    @staticmethod
    def _coordinator(bridge, executor):
        return FiveArmCoordinator(
            bridge=bridge,
            executor=executor,
            motion_monitor_factory=lambda robot_id: FakeMotionMonitor(
                bridge, robot_id
            ),
        )

    def test_good_cycle_uses_all_five_robots_in_process_order(self):
        bridge = FakeBridge()
        executor = FakeExecutor(bridge)
        result = self._coordinator(bridge, executor).execute_cycle(
            "good", "ORDER-GOOD"
        )
        self.assertEqual(result["status"], "finished")
        self.assertEqual(
            executor.actions,
            [
                R1_BOX_PLACED,
                R2_PCB_PLACED,
                R3_MODULE_PLACED,
                R1_TERMINAL_PLACED,
                R3_PRODUCT_TO_INSPECTION,
                R4_SCREW_DONE,
                R5_SORT_GOOD_DONE,
            ],
        )
        self.assertEqual(
            bridge.signals, [("cell_product_state", "camera_good")]
        )
        self.assertEqual(len(result["tasks"]), 7)
        self.assertTrue(
            all(
                record["handoff_delay_simulation_s"] <= 0.05
                for record in result["tasks"][1:]
            )
        )
        self.assertTrue(
            all(record["motion_timing"]["motion_detected"] for record in result["tasks"])
        )
        self.assertTrue(
            all(
                record["handoff_to_first_motion_simulation_s"] <= 0.15
                for record in result["tasks"][1:]
            )
        )

    def test_defect_cycle_selects_only_defect_sort_branch(self):
        bridge = FakeBridge()
        executor = FakeExecutor(bridge)
        result = self._coordinator(bridge, executor).execute_cycle(
            "defect"
        )
        self.assertEqual(result["status"], "finished")
        self.assertEqual(executor.actions[-1], R5_SORT_DEFECT_DONE)
        self.assertNotIn(R5_SORT_GOOD_DONE, executor.actions)
        self.assertEqual(
            bridge.signals, [("cell_product_state", "camera_defect")]
        )

    def test_failure_stops_the_sequence_without_false_completion(self):
        bridge = FakeBridge()
        executor = FakeExecutor(bridge, fail_action=R3_MODULE_PLACED)
        result = self._coordinator(bridge, executor).execute_cycle(
            "good"
        )
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["failed_action"], R3_MODULE_PLACED)
        self.assertEqual(
            executor.actions,
            [R1_BOX_PLACED, R2_PCB_PLACED, R3_MODULE_PLACED],
        )
        self.assertEqual(bridge.signals, [])


if __name__ == "__main__":
    unittest.main()
