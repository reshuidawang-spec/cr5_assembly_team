from __future__ import annotations

import unittest
import time

from interfaces.types import Order, RobotState, Task, TaskResult, TaskStatus
from robot_control.integrated_executor import IntegratedRobotExecutor
from robot_control.r1_motion import R1_BOX_PLACED, R1_TERMINAL_PLACED
from robot_control.r2_motion import R2_PCB_PLACED
from robot_control.r3_motion import R3_MODULE_PLACED, R3_PRODUCT_TO_INSPECTION
from robot_control.r4_motion import R4_SCREW_DONE
from robot_control.r5_motion import R5_SORT_DEFECT_DONE, R5_SORT_GOOD_DONE
from scheduler.order_parser import OrderParser
from scheduler.scheduler import Scheduler


class FakeSim:
    def __init__(self):
        self.simulation_time = 0.0

    def getSimulationTime(self):
        return self.simulation_time


class FakeBridge:
    def __init__(self):
        self.host = "127.0.0.1"
        self.port = 23000
        self.sim = FakeSim()
        self.connected = True
        self.last_error = ""
        self.signals = []
        self.stepping = []
        self._stepping = False

    @property
    def stepping_enabled(self):
        return self._stepping

    def is_connected(self):
        return self.connected

    def connect(self):
        self.connected = True
        return True

    def set_stepping(self, enabled):
        self._stepping = bool(enabled)
        self.stepping.append(bool(enabled))

    def set_string_signal(self, name, value):
        self.signals.append((name, value))

    def step(self):
        self.sim.simulation_time += 0.05
        return True


class FakeBaseExecutor:
    def __init__(self, bridge):
        self.bridge = bridge
        self.actions = []
        self.prepare_calls = []

    def prepare_cycle(self, quality, preload_both_r5=False):
        self.prepare_calls.append((quality, preload_both_r5))
        return {"ready": True, "path_points_total": 123}

    def execute_task(self, task):
        self.actions.append(task.target_point)
        started = self.bridge.sim.simulation_time
        self.bridge.sim.simulation_time += 1.0
        return TaskResult(
            task_id=task.task_id,
            robot_id=task.available_robots[0],
            status=TaskStatus.FINISHED.value,
            start_time=started,
            end_time=started + 1.0,
        )

    def execute_task_async(self, task, callback):
        callback(self.execute_task(task))

    def get_robot_states(self):
        return [RobotState(robot_id=f"R{index}") for index in range(1, 6)]

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
        pass

    def clear_robot_fault(self, robot_id):
        pass


class FakeMotionMonitor:
    def __init__(self, robot_id, first_motion_simulation_time_s):
        self.robot_id = robot_id
        self.first_motion_simulation_time_s = first_motion_simulation_time_s
        self.first_motion_wall_epoch_s = None

    def start(self):
        self.first_motion_wall_epoch_s = time.time() + 0.02

    def stop(self):
        return {
            "robot_id": self.robot_id,
            "motion_detected": True,
            "dispatch_to_first_motion_wall_s": 0.02,
            "first_motion_wall_epoch_s": self.first_motion_wall_epoch_s,
            "first_motion_simulation_time_s": (
                self.first_motion_simulation_time_s
            ),
            "monitor_error": "",
        }


class FakeMotionMonitorFactory:
    def __init__(self):
        self.prepared = 0
        self.closed = 0

    def prepare(self):
        self.prepared += 1

    def close(self):
        self.closed += 1

    def __call__(self, robot_id):
        return FakeMotionMonitor(robot_id, 0.1)


def make_order(quality="OK"):
    return Order(
        order_id="ORDER-A-001",
        product_type="A",
        priority=1,
        quantity=1,
        expected_quality=quality,
    )


def make_task(action, robot_id, task_id):
    return Task(
        task_id=task_id,
        order_id="ORDER-A-001",
        product_type="A",
        process="assemble",
        target_area="area",
        target_point=action,
        available_robots=[robot_id],
    )


class OrderAndSchedulerIntegrationTests(unittest.TestCase):
    def test_order_quality_roundtrip_and_parser_validation(self):
        parser = OrderParser()
        order = parser.parse_dict(make_order("NG").to_dict())
        self.assertEqual(order.expected_quality, "NG")
        self.assertEqual(Order.from_dict(order.to_dict()), order)
        with self.assertRaisesRegex(ValueError, "expected_quality"):
            parser.parse_dict({**order.to_dict(), "expected_quality": "MAYBE"})

    def test_good_order_generates_validated_seven_task_chain(self):
        scheduler = Scheduler()
        tasks = scheduler.generate_tasks([make_order("OK")])
        self.assertEqual(
            [task.target_point for task in tasks],
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
            [task.available_robots[0] for task in tasks],
            ["R1", "R2", "R3", "R1", "R3", "R4", "R5"],
        )
        self.assertTrue(
            all(
                task.predecessors == [tasks[index - 1].task_id]
                for index, task in enumerate(tasks[1:], start=1)
            )
        )

    def test_defect_order_selects_only_defect_branch(self):
        tasks = Scheduler().generate_tasks([make_order("NG")])
        self.assertEqual(tasks[-1].target_point, R5_SORT_DEFECT_DONE)
        self.assertNotIn(
            R5_SORT_GOOD_DONE, [task.target_point for task in tasks]
        )

    def test_real_scheduler_rejects_unvalidated_scene_workloads(self):
        scheduler = Scheduler()
        with self.assertRaisesRegex(ValueError, "exactly one order"):
            scheduler.generate_tasks([make_order(), make_order()])
        with self.assertRaisesRegex(ValueError, "product type A"):
            scheduler.generate_tasks(
                [
                    Order(
                        order_id="B-001",
                        product_type="B",
                        priority=1,
                    )
                ]
            )
        with self.assertRaisesRegex(ValueError, "quantity must be 1"):
            scheduler.generate_tasks(
                [
                    Order(
                        order_id="A-002",
                        product_type="A",
                        priority=1,
                        quantity=2,
                    )
                ]
            )
        with self.assertRaisesRegex(ValueError, "cannot insert"):
            scheduler.insert_urgent_order(make_order())

    def test_scheduler_dispatches_one_task_and_propagates_failure(self):
        scheduler = Scheduler()
        tasks = scheduler.generate_tasks([make_order()])
        robots = [RobotState(robot_id=f"R{index}") for index in range(1, 6)]
        scheduler.schedule(tasks, robots)
        self.assertEqual(
            [task.task_id for task in tasks if task.status == "running"],
            [tasks[0].task_id],
        )
        scheduler.on_task_complete(
            TaskResult(
                task_id=tasks[0].task_id,
                robot_id="R1",
                status=TaskStatus.FAILED.value,
            ),
            tasks,
            robots,
        )
        self.assertTrue(
            all(task.status == TaskStatus.FAILED.value for task in tasks)
        )


class IntegratedRobotExecutorTests(unittest.TestCase):
    def test_prepare_cycle_and_close_manage_persistent_monitor(self):
        bridge = FakeBridge()
        base = FakeBaseExecutor(bridge)
        monitors = FakeMotionMonitorFactory()
        executor = IntegratedRobotExecutor(
            bridge=bridge,
            executor=base,
            motion_monitor_factory=monitors,
        )

        evidence = executor.prepare_cycle("defect", preload_both_r5=True)
        executor.close()

        self.assertTrue(evidence["ready"])
        self.assertEqual(base.prepare_calls, [("defect", True)])
        self.assertEqual(monitors.prepared, 1)
        self.assertEqual(monitors.closed, 1)

    def test_camera_signal_and_real_motion_handoff_metrics(self):
        bridge = FakeBridge()
        base = FakeBaseExecutor(bridge)
        first_motion_times = iter((0.10, 1.25))
        executor = IntegratedRobotExecutor(
            bridge=bridge,
            executor=base,
            quality_resolver=lambda order_id: "NG",
            motion_monitor_factory=lambda robot_id: FakeMotionMonitor(
                robot_id, next(first_motion_times)
            ),
        )

        first = executor.execute_task(
            make_task(R1_BOX_PLACED, "R1", "TASK-01")
        )
        second = executor.execute_task(
            make_task(R4_SCREW_DONE, "R4", "TASK-02")
        )

        self.assertEqual(
            bridge.signals, [("cell_product_state", "camera_defect")]
        )
        self.assertEqual(second.quality_result, "NG")
        self.assertTrue(first.metrics["motion_timing"]["motion_detected"])
        self.assertIsNotNone(
            first.metrics["motion_timing"][
                "task_call_to_first_motion_wall_s"
            ]
        )
        self.assertAlmostEqual(
            second.metrics["handoff_to_first_motion_simulation_s"], 0.25
        )
        self.assertIn("camera_transition", second.metrics)
        self.assertEqual(bridge.stepping[-2:], [True, False])

    def test_camera_step_preserves_existing_ready_stepping(self):
        bridge = FakeBridge()
        bridge._stepping = True
        base = FakeBaseExecutor(bridge)
        executor = IntegratedRobotExecutor(
            bridge=bridge,
            executor=base,
            quality_resolver=lambda order_id: "OK",
            motion_monitor_factory=lambda robot_id: FakeMotionMonitor(
                robot_id, 0.10
            ),
        )

        result = executor.execute_task(
            make_task(R4_SCREW_DONE, "R4", "TASK-CAMERA")
        )

        self.assertEqual(result.status, TaskStatus.FINISHED.value)
        self.assertTrue(bridge.stepping_enabled)
        self.assertNotIn(False, bridge.stepping)


if __name__ == "__main__":
    unittest.main()
