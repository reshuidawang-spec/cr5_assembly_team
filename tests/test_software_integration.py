from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from app.dashboard import compare_results, compute_runtime_kpi
from interfaces.robot_interface import IRobotExecutor
from interfaces.types import Order, RobotState, Task, TaskResult, TaskStatus
from orchestration.cell_orchestrator import CellOrchestrator
from scheduler.experiment import DiscreteEventExperiment
from scheduler.order_parser import OrderParser
from scheduler.scheduler import Scheduler


class _InstantExecutor(IRobotExecutor):
    def __init__(self):
        self.robots = {
            robot_id: RobotState(robot_id)
            for robot_id in ("R1", "R2", "R3", "R4", "R5", "CAMERA")
        }

    def execute_task(self, task):
        return TaskResult(
            task_id=task.task_id,
            robot_id=task.available_robots[0],
            status=TaskStatus.FINISHED.value,
            quality_result="OK" if task.process == "inspect" else "",
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


class SoftwareIntegrationTests(unittest.TestCase):
    def test_order_parser_accepts_wrapped_file_and_rejects_duplicate(self):
        parser = OrderParser()
        payload = {
            "orders": [
                {
                    "order_id": "A-001",
                    "product_type": "a",
                    "priority": 4,
                    "quantity": 2,
                    "due_time": 80,
                }
            ]
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "orders.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            orders = parser.parse_file(str(path))
        self.assertEqual(orders[0].product_type, "A")
        self.assertEqual(orders[0].quantity, 2)
        with self.assertRaisesRegex(ValueError, "重复"):
            parser.add_order(orders[0])

    def test_order_parser_rejects_invalid_production_fields(self):
        parser = OrderParser()
        with self.assertRaises(ValueError):
            parser.parse_dict({"order_id": "", "product_type": "A"})
        with self.assertRaises(ValueError):
            parser.parse_dict({"order_id": "X", "product_type": "D"})
        with self.assertRaises(ValueError):
            parser.parse_dict({"order_id": "X", "product_type": "A", "priority": 11})
        with self.assertRaises(ValueError):
            parser.parse_dict({"order_id": "X", "product_type": "A", "quantity": 0})

    def test_runtime_kpi_uses_executor_timestamps_and_dependencies(self):
        tasks = [
            Task("T1", "O1", "A", "box_feed", "a", "p1"),
            Task("T2", "O1", "A", "pcb_install", "b", "p2", predecessors=["T1"]),
        ]
        results = {
            "T1": TaskResult("T1", "R1", "finished", start_time=10.0, end_time=12.0),
            "T2": TaskResult("T2", "R2", "finished", start_time=14.0, end_time=17.0),
        }
        kpi = compute_runtime_kpi(
            tasks, results, conflict_count=2, robot_ids=("R1", "R2", "R3")
        )
        self.assertEqual(kpi["makespan"], 7.0)
        self.assertEqual(kpi["avg_waiting_time"], 1.0)
        self.assertAlmostEqual(kpi["utilization"]["R1"], 2.0 / 7.0)
        self.assertAlmostEqual(kpi["utilization"]["R2"], 3.0 / 7.0)
        self.assertEqual(kpi["conflict_count"], 2)
        self.assertEqual(kpi["completed"], 2)

    def test_orchestrator_retains_every_task_result(self):
        orchestrator = CellOrchestrator(Scheduler(), _InstantExecutor(), 0.005)
        orchestrator.start([Order("A-LEDGER", "A", 1)])
        self.assertEqual(orchestrator.wait(2.0), "finished")
        self.assertEqual(set(orchestrator.results_by_task), orchestrator.dispatched_task_ids)
        self.assertEqual(len(orchestrator.results_by_task), len(orchestrator.tasks))

    def test_scheduling_comparison_is_serializable_for_dashboard_export(self):
        experiment = DiscreteEventExperiment()
        orders = [
            Order("A1", "A", 1, due_time=120),
            Order("C1", "C", 8, due_time=80, arrival_time=5),
        ]
        comparison = compare_results(
            experiment.run_baseline(orders), experiment.run_proposed(orders)
        )
        encoded = json.dumps(comparison)
        self.assertIn("proposed", encoded)
        self.assertIn("improvement_percent", encoded)


if __name__ == "__main__":
    unittest.main()
