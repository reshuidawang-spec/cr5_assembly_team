import unittest

from interfaces.types import Order, RobotState, Task, TaskResult, TaskStatus
from mock.mock_scheduler import MockScheduler
from scheduler.assembly_process import AssemblyProcessPlanner
from scheduler.experiment import DiscreteEventExperiment
from scheduler.scheduler import Scheduler
from scheduler.task_generator import TaskGenerator


class SchedulerV2Tests(unittest.TestCase):
    def test_order_arrival_time_round_trip(self):
        order = Order("O1", "A", 2, due_time=80, arrival_time=12)
        restored = Order.from_dict(order.to_dict())
        self.assertEqual(restored.arrival_time, 12)

    def test_quantity_is_expanded_into_independent_units(self):
        generator = TaskGenerator()
        tasks = generator.generate([Order("A100", "A", 1, quantity=2)])
        self.assertEqual(len(tasks), 14)
        self.assertEqual({task.order_id for task in tasks}, {"A100-01", "A100-02"})

    def test_generated_tasks_match_scene_endpoints_and_shared_platforms(self):
        tasks = TaskGenerator().generate([Order("A100", "A", 1)])
        by_process = {task.process: task for task in tasks}
        self.assertEqual(by_process["box_feed"].target_point, "R1_BOX_PLACE_TCP")
        self.assertEqual(by_process["pcb_install"].target_point, "R2_PCB_PLACE_TCP")
        self.assertEqual(by_process["module_install"].target_point, "R3_MODULE_PLACE_TCP")
        self.assertEqual(by_process["terminal_install"].target_point, "R1_TERMINAL_PLACE_TCP")
        self.assertEqual(by_process["transfer_to_inspection"].target_point, "R3_PRODUCT_PLACE_INSPECTION_TCP")
        self.assertEqual(by_process["inspect"].target_point, "CAMERA_INSPECTION_CENTER")
        self.assertEqual(by_process["screw"].target_point, "R4_SCREW_PRESS")
        self.assertIn("assembly_fixture", by_process["box_feed"].required_areas)
        self.assertIn("inspection_platform_area", by_process["screw"].required_areas)
        self.assertIn("inspection_platform_area", by_process["inspect"].required_areas)
        self.assertEqual(by_process["inspect"].available_robots, ["CAMERA"])
        self.assertEqual(by_process["screw"].available_robots, ["R4"])

    def test_waiting_aging_increases_task_score(self):
        generator = TaskGenerator()
        task = generator.generate([Order("A100", "A", 1, due_time=200)])[0]
        early = generator.task_score(task, current_time=0, ready_time=0)
        aged = generator.task_score(task, current_time=40, ready_time=0)
        self.assertGreater(aged, early)

    def test_bottleneck_penalty_applies_only_to_normal_orders(self):
        generator = TaskGenerator()
        normal_r3 = Task(
            "T_R3",
            "O1",
            "A",
            "module_install",
            "module_supply_area",
            "R3_MODULE_PLACE_TCP",
            ["R3"],
            priority=1,
        )
        normal_r1 = Task(
            "T_R1",
            "O1",
            "A",
            "terminal_install",
            "terminal_supply_area",
            "R1_TERMINAL_PLACE_TCP",
            ["R1"],
            priority=1,
        )
        urgent_r3 = Task(
            "T_U",
            "O2",
            "A",
            "module_install",
            "module_supply_area",
            "R3_MODULE_PLACE_TCP",
            ["R3"],
            priority=5,
        )
        weights = {
            "bottleneck_penalty_weight": 0.05,
            "bottleneck_resources": ["R3"],
            "urgent_threshold": 5,
        }
        self.assertLess(
            generator.task_score(normal_r3, weights=weights),
            generator.task_score(normal_r1, weights=weights),
        )
        self.assertEqual(
            generator.task_score(urgent_r3, weights=weights),
            generator.task_score(urgent_r3, weights={}),
        )

    def test_one_robot_receives_at_most_one_task_per_decision(self):
        scheduler = Scheduler()
        tasks = [
            Task(
                f"T{i}",
                f"O{i}",
                "A",
                "box_feed",
                "box_supply_area",
                "R1_BOX_PLACE_TCP",
                ["R1"],
            )
            for i in range(3)
        ]
        robots = [RobotState("R1", status="idle")]
        scheduler.schedule(tasks, robots)
        running = [task for task in tasks if task.status == TaskStatus.RUNNING.value]
        self.assertEqual(len(running), 1)

    def test_faulted_candidate_does_not_block_healthy_alternative(self):
        scheduler = Scheduler()
        task = Task(
            "T1",
            "O1",
            "A",
            "box_feed",
            "box_supply_area",
            "R1_BOX_PLACE_TCP",
            ["R1", "R2"],
        )
        robots = [
            RobotState("R1", status="fault"),
            RobotState("R2", status="idle"),
        ]
        scheduler.schedule([task], robots)
        self.assertEqual(task.status, TaskStatus.RUNNING.value)
        self.assertEqual(task.available_robots, ["R2", "R1"])

    def test_shared_inspection_platform_blocks_r4_r5_overlap(self):
        scheduler = Scheduler()
        screw = Task(
            "T1", "O1", "A", "screw", "inspection_screw_area", "R4_SCREW_PRESS", ["R4"],
            required_areas=["inspection_platform_area"],
        )
        sort_task = Task(
            "T2", "O2", "A", "sort_good", "good_conveyor_area", "R5_GOOD_PLACE_TCP", ["R5"],
            required_areas=["inspection_platform_area"],
        )
        robots = [RobotState("R4", status="idle"), RobotState("R5", status="idle")]
        scheduler.schedule([screw, sort_task], robots)
        running = [task for task in (screw, sort_task) if task.status == TaskStatus.RUNNING.value]
        self.assertEqual(len(running), 1)
        self.assertEqual(scheduler.conflict_count, 1)

    def test_real_scheduler_generates_only_selected_sort_branch(self):
        scheduler = Scheduler()
        tasks = scheduler.generate_tasks([Order("A100", "A", 1)])
        inspect = next(task for task in tasks if task.process == "inspect")
        screw = next(task for task in tasks if task.process == "screw")
        inspect.status = TaskStatus.RUNNING.value
        inspect_result = TaskResult(
            inspect.task_id,
            "CAMERA",
            TaskStatus.FINISHED.value,
            end_time=30,
            quality_result="OK",
        )
        scheduler.on_task_complete(inspect_result, tasks, [])
        self.assertFalse(any(task.process.startswith("sort_") for task in tasks))
        screw.status = TaskStatus.RUNNING.value
        screw_result = TaskResult(
            screw.task_id,
            "R4",
            TaskStatus.FINISHED.value,
            end_time=40,
        )
        scheduler.on_task_complete(screw_result, tasks, [])
        branches = [task.process for task in tasks if task.process.startswith("sort_")]
        self.assertEqual(branches, ["sort_good"])

    def test_mock_scheduler_generates_only_selected_sort_branch(self):
        scheduler = MockScheduler()
        tasks = scheduler.generate_tasks([Order("A100", "A", 1)])
        self.assertFalse(any(task.process.startswith("sort_") for task in tasks))
        inspect = next(task for task in tasks if task.process == "inspect")
        screw = next(task for task in tasks if task.process == "screw")
        inspect_result = TaskResult(
            inspect.task_id,
            "CAMERA",
            TaskStatus.FINISHED.value,
            quality_result="NG",
        )
        scheduler.on_task_complete(inspect_result, tasks, [])
        self.assertFalse(any(task.process.startswith("sort_") for task in tasks))
        screw_result = TaskResult(
            screw.task_id,
            "R4",
            TaskStatus.FINISHED.value,
        )
        scheduler.on_task_complete(screw_result, tasks, [])
        branches = [task.process for task in tasks if task.process.startswith("sort_")]
        self.assertEqual(branches, ["sort_defect"])

    def test_parallel_fifo_is_a_fair_parallel_baseline(self):
        experiment = DiscreteEventExperiment()
        orders = [
            Order("A1", "A", 1, due_time=120),
            Order("B1", "B", 2, due_time=160),
            Order("C1", "C", 5, due_time=90, arrival_time=10),
        ]
        serial = experiment.run_baseline(orders)
        parallel = experiment.run_parallel_fifo(orders)
        proposed = experiment.run_proposed(orders)
        self.assertLessEqual(parallel.makespan, serial.makespan)
        self.assertLessEqual(
            proposed.urgent_response_time, parallel.urgent_response_time
        )
        self.assertLessEqual(
            proposed.urgent_completion_time, parallel.urgent_completion_time
        )
        self.assertEqual(len(proposed.order_completion_times), 3)

    def test_fault_matrix_runs_all_scene_resources(self):
        experiment = DiscreteEventExperiment()
        orders = [Order("A1", "A", 5, due_time=100, arrival_time=20)]
        results = experiment.run_fault_matrix(orders)
        modes = {result.mode for result in results}
        self.assertEqual(
            modes,
            {
                "fault_r1_key_window",
                "fault_r2_key_window",
                "fault_r3_key_window",
                "fault_r4_key_window",
                "fault_r5_key_window",
                "fault_camera_key_window",
            },
        )
        self.assertTrue(all(result.makespan > 0 for result in results))

    def test_assembly_process_planner_builds_layered_sequence_and_balance(self):
        planner = AssemblyProcessPlanner()
        sequence = planner.component_sequence_rows()
        ordered_processes = [row["process"] for row in sequence]
        self.assertEqual(
            [row["node_id"] for row in sequence[:9]],
            [
                "box_shell",
                "pcb_board",
                "pcb_electronic_parts",
                "pcb_holes",
                "control_module_body",
                "control_module_label",
                "terminal_block_body",
                "terminal_slots",
                "terminal_screw_head",
            ],
        )
        self.assertIn("transfer_to_inspection", ordered_processes)
        self.assertEqual(set(ordered_processes[-2:]), {"sort_good", "sort_defect"})
        self.assertEqual(sequence[0]["topology_level"], 1)
        self.assertEqual(sequence[-1]["level"], 12)
        self.assertEqual(sequence[-1]["topology_level"], sequence[-2]["topology_level"])

        experiment = DiscreteEventExperiment()
        result = experiment.run_proposed([Order("A1", "A", 5, due_time=100)])
        steps = planner.expand_schedule_to_worksteps(result.records)
        self.assertTrue(any(step.step_label == "固定相机定位检测区域" for step in steps))
        self.assertTrue(any(step.step_label == "相机检测并输出 OK/NG" for step in steps))
        self.assertTrue(any(step.target_point == "R4_SCREW_PRESS" for step in steps))

        balance = planner.line_balance_summary(steps)
        self.assertGreater(balance["balance_rate"], 0)
        self.assertIn(balance["bottleneck_resource"], balance["station_times"])
        self.assertTrue(planner.balance_recommendations(balance))


if __name__ == "__main__":
    unittest.main()
