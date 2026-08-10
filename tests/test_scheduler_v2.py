import unittest

from interfaces.types import Order, RobotState, Task, TaskResult, TaskStatus
from mock.mock_scheduler import MockScheduler
from pathlib import Path
from robot_control.motion_control import WorkstepMotionPlanner
from scheduler.assembly_process import AssemblyProcessPlanner
from scheduler.dynamic_order_sequence import (
    DynamicOrderInput,
    QualityOverrideExperiment,
    expand_quality_overrides_with_policy,
    expand_order_inputs_as_units,
    plan_dynamic_order_sequence,
    quality_evaluation,
)
from scheduler.experiment import DiscreteEventExperiment
from scheduler.scheduler import Scheduler
from scheduler.task_generator import TaskGenerator
from scripts.run_coppelia_order_demo import product_color_for, shell_groups_for_process


class SchedulerV2Tests(unittest.TestCase):
    def test_workstep_motion_planner_follows_configured_points(self):
        planner = WorkstepMotionPlanner(Path(__file__).resolve().parents[1])
        box_mid = planner.frame_for("box_feed", 0.25, "R1_BOX_PLACE_TCP")
        screw_end = planner.frame_for("screw", 1.0, "R4_SCREW_PRESS")
        sort_end = planner.frame_for("sort_good", 1.0, "R5_GOOD_PLACE_TCP")
        self.assertIn(box_mid.point_name, {"R1_BOX_PICK_TCP", "R1_BOX_PLACE_APP"})
        self.assertEqual(screw_end.point_name, "R4_SCREW_PRESS")
        self.assertEqual(sort_end.point_name, "R5_GOOD_PLACE_TCP")

    def test_order_arrival_time_round_trip(self):
        order = Order("O1", "A", 2, due_time=80, arrival_time=12)
        restored = Order.from_dict(order.to_dict())
        self.assertEqual(restored.arrival_time, 12)

    def test_quantity_is_expanded_into_independent_units(self):
        generator = TaskGenerator()
        tasks = generator.generate([Order("A100", "A", 1, quantity=2)])
        self.assertEqual(len(tasks), 12)
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
        self.assertNotIn("screw", by_process)
        self.assertIn("assembly_fixture", by_process["box_feed"].required_areas)
        self.assertIn("inspection_platform_area", by_process["inspect"].required_areas)
        self.assertEqual(by_process["inspect"].available_robots, ["CAMERA"])
        ok_branch = TaskGenerator().build_post_inspection_task(by_process["inspect"], "OK")
        ng_branch = TaskGenerator().build_post_inspection_task(by_process["inspect"], "NG")
        self.assertEqual(ok_branch.process, "screw")
        self.assertEqual(ok_branch.available_robots, ["R4"])
        self.assertEqual(ng_branch.process, "sort_defect")
        self.assertEqual(ng_branch.available_robots, ["R5"])

    def test_waiting_aging_increases_task_score(self):
        generator = TaskGenerator()
        task = generator.generate([Order("A100", "A", 1, due_time=200)])[0]
        early = generator.task_score(task, current_time=0, ready_time=0)
        aged = generator.task_score(task, current_time=40, ready_time=0)
        self.assertGreater(aged, early)

    def test_due_lateness_risk_raises_task_score(self):
        generator = TaskGenerator()
        early_due_task, far_due_task = [
            tasks[0]
            for tasks in (
                generator.generate([Order("DUE_SOON", "A", 1, due_time=120)]),
                generator.generate([Order("DUE_LATER", "A", 1, due_time=400)]),
            )
        ]
        weights = {
            "priority_weight": 0.0,
            "due_weight": 0.30,
            "waiting_weight": 0.0,
            "critical_path_weight": 0.0,
            "lateness_risk_weight": 0.18,
            "overdue_boost": 0.35,
            "urgency_horizon": 60,
        }

        overdue_score = generator.task_score(
            early_due_task,
            current_time=100,
            ready_time=100,
            remaining_work=50,
            weights=weights,
        )
        safe_score = generator.task_score(
            far_due_task,
            current_time=100,
            ready_time=100,
            remaining_work=50,
            weights=weights,
        )
        self.assertGreater(overdue_score, safe_score)

    def test_post_inspection_tasks_get_platform_clearance_bonus(self):
        generator = TaskGenerator()
        tasks = generator.generate([Order("A100", "A", 1)])
        normal_task = next(task for task in tasks if task.process == "terminal_install")
        inspect = next(task for task in tasks if task.process == "inspect")
        screw = generator.build_post_inspection_task(inspect, "OK")
        sort_defect = generator.build_post_inspection_task(inspect, "NG")
        weights = {
            "priority_weight": 0.0,
            "due_weight": 0.0,
            "waiting_weight": 0.0,
            "critical_path_weight": 0.0,
            "post_inspection_clearance_bonus": 0.40,
            "screw_clearance_bonus": 0.35,
            "sort_clearance_bonus": 0.60,
        }

        self.assertEqual(generator.task_score(normal_task, weights=weights), 0.0)
        self.assertAlmostEqual(generator.task_score(screw, weights=weights), 0.75)
        self.assertAlmostEqual(generator.task_score(sort_defect, weights=weights), 1.00)

    def test_experiment_reports_inspection_platform_metrics(self):
        experiment = DiscreteEventExperiment()
        result = experiment.run_proposed([Order("A100", "A", 1, due_time=180)])
        summary = result.summary_dict()

        self.assertIn("inspection_platform_avg_residency_time", summary)
        self.assertIn("post_inspection_avg_clearance_wait", summary)
        self.assertGreater(summary["inspection_platform_avg_residency_time"], 0.0)
        self.assertGreaterEqual(summary["post_inspection_avg_clearance_wait"], 0.0)

    def test_dynamic_sequence_accepts_scoring_weight_overrides(self):
        experiment = QualityOverrideExperiment(scoring_weights={"priority_weight": 1.23})
        self.assertEqual(experiment.scoring_config["priority_weight"], 1.23)

    def test_product_color_mapping_uses_red_for_urgent_orders(self):
        self.assertEqual(product_color_for("A", 1), [0.62, 0.62, 0.62])
        self.assertEqual(product_color_for("B", 1), [0.15, 0.20, 0.55])
        self.assertEqual(product_color_for("C", 1), [0.20, 0.55, 1.00])
        self.assertEqual(product_color_for("A", 5), [1.00, 0.05, 0.05])

    def test_product_shell_coloring_is_scoped_by_station(self):
        self.assertEqual(shell_groups_for_process("box_feed"), ("assembly",))
        self.assertEqual(shell_groups_for_process("inspect"), ("inspection",))
        self.assertEqual(shell_groups_for_process("screw"), ("inspection",))
        self.assertEqual(shell_groups_for_process("sort_good"), ("inspection",))
        self.assertEqual(shell_groups_for_process("transfer_to_inspection"), ("assembly", "inspection"))

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

    def test_shared_inspection_platform_blocks_camera_r5_overlap(self):
        scheduler = Scheduler()
        inspect = Task(
            "T1", "O1", "A", "inspect", "camera_area", "CAMERA_INSPECTION_CENTER", ["CAMERA"],
            required_areas=["inspection_platform_area", "camera_area"],
        )
        sort_task = Task(
            "T2", "O2", "A", "sort_good", "good_conveyor_area", "R5_GOOD_PLACE_TCP", ["R5"],
            required_areas=["inspection_platform_area"],
        )
        robots = [RobotState("CAMERA", status="idle"), RobotState("R5", status="idle")]
        scheduler.schedule([inspect, sort_task], robots)
        running = [task for task in (inspect, sort_task) if task.status == TaskStatus.RUNNING.value]
        self.assertEqual(len(running), 1)
        self.assertEqual(scheduler.conflict_count, 1)

    def test_real_scheduler_branches_by_quality_after_inspection(self):
        scheduler = Scheduler()
        tasks = scheduler.generate_tasks([Order("A100", "A", 1)])
        inspect = next(task for task in tasks if task.process == "inspect")
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
        screw = next(task for task in tasks if task.process == "screw")
        self.assertEqual(screw.predecessors, [inspect.task_id])
        screw.status = TaskStatus.RUNNING.value
        scheduler.on_task_complete(
            TaskResult(
                screw.task_id,
                "R4",
                TaskStatus.FINISHED.value,
                end_time=40,
            ),
            tasks,
            [],
        )
        branches = [task.process for task in tasks if task.process.startswith("sort_")]
        self.assertEqual(branches, ["sort_good"])

    def test_mock_scheduler_sorts_directly_after_inspection(self):
        scheduler = MockScheduler()
        tasks = scheduler.generate_tasks([Order("A100", "A", 1)])
        self.assertFalse(any(task.process.startswith("sort_") for task in tasks))
        inspect = next(task for task in tasks if task.process == "inspect")
        inspect_result = TaskResult(
            inspect.task_id,
            "CAMERA",
            TaskStatus.FINISHED.value,
            quality_result="NG",
        )
        scheduler.on_task_complete(inspect_result, tasks, [])
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

    def test_dynamic_order_sequence_prioritizes_inserted_urgent_order(self):
        _, rows = plan_dynamic_order_sequence([
            DynamicOrderInput("A001", "A", priority=1, due_time=260, arrival_time=0, quality="OK"),
            DynamicOrderInput("A002", "A", priority=1, due_time=280, arrival_time=0, quality="OK"),
            DynamicOrderInput("URG_C", "C", priority=5, due_time=115, arrival_time=20, quality="OK"),
        ])
        order_rank = {row.order_id: row.rank for row in rows}
        self.assertLess(order_rank["URG_C"], order_rank["A002"])

    def test_dynamic_order_quantity_is_expanded_before_resequencing(self):
        expanded = expand_order_inputs_as_units([
            DynamicOrderInput("A001", "A", quantity=3, priority=1, due_time=260),
        ])
        self.assertEqual([item.order_id for item in expanded], ["A001-01", "A001-02", "A001-03"])
        self.assertTrue(all(item.quantity == 1 for item in expanded))

    def test_auto_quality_policy_sets_configured_defect_count(self):
        inputs = [DynamicOrderInput("A_BATCH", "A", quantity=100, priority=1, due_time=1200)]
        overrides = expand_quality_overrides_with_policy(inputs, defects_per_100=2)
        self.assertEqual(sum(1 for value in overrides.values() if value == "NG"), 2)
        self.assertEqual(sum(1 for value in overrides.values() if value == "OK"), 98)

    def test_ab_changeover_sequence_finishes_first_a_before_inserted_b(self):
        result, rows = plan_dynamic_order_sequence(
            [
                DynamicOrderInput("A_FIRST", "A", 5, 1, 260, 0, "AUTO"),
                DynamicOrderInput("B_SWITCH", "B", 2, 6, 380, 205, "AUTO"),
                DynamicOrderInput("A_REMAIN", "A", 5, 1, 650, 360, "AUTO"),
            ],
            defects_per_100=20,
        )
        by_order = {row.order_id: row for row in rows}
        self.assertLess(by_order["A_FIRST-05"].completion_time, by_order["B_SWITCH-01"].first_start)
        self.assertLess(by_order["B_SWITCH-02"].completion_time, by_order["A_REMAIN-01"].first_start)
        evaluation = quality_evaluation(result, defects_per_100=20)
        self.assertEqual(evaluation["total_products"], 12.0)
        self.assertEqual(evaluation["defect_count"], 2.0)

    def test_dynamic_order_sequence_keeps_process_route_inside_each_order(self):
        result, rows = plan_dynamic_order_sequence([
            DynamicOrderInput("OK_A", "A", priority=1, due_time=200, arrival_time=0, quality="OK"),
            DynamicOrderInput("NG_B", "B", priority=1, due_time=200, arrival_time=0, quality="NG"),
        ])
        chains = {row.order_id: row.process_chain for row in rows}
        self.assertIn("inspect → screw → sort_good", chains["OK_A"])
        self.assertIn("inspect → sort_defect", chains["NG_B"])
        ng_records = [record.process for record in result.records if record.order_id == "NG_B"]
        self.assertNotIn("screw", ng_records)

    def test_quality_branch_clears_platform_at_correct_step(self):
        experiment = DiscreteEventExperiment()
        orders = [
            Order("A001", "A", 1, due_time=260),
            Order("A002", "A", 1, due_time=280),
            Order("A003", "A", 1, due_time=300),
            Order("B001", "B", 2, due_time=340),
            Order("C001", "C", 2, due_time=380),
            Order("C002", "C", 2, due_time=410),
            Order("URGENT_C", "C", 5, due_time=115, arrival_time=20),
        ]
        proposed = experiment.run_proposed(orders)
        records_by_order = {}
        for record in proposed.records:
            records_by_order.setdefault(record.order_id, []).append(record)

        for records in records_by_order.values():
            inspect = next(record for record in records if record.process == "inspect")
            sort_task = next(
                record for record in records if record.process in ("sort_good", "sort_defect")
            )
            if sort_task.process == "sort_defect":
                self.assertEqual(sort_task.start_time, inspect.end_time)
                self.assertFalse(any(record.process == "screw" for record in records))
            else:
                screw = next(record for record in records if record.process == "screw")
                self.assertEqual(screw.start_time, inspect.end_time)
                self.assertEqual(sort_task.start_time, screw.end_time)

    def test_station_residency_prevents_overlapping_products(self):
        experiment = DiscreteEventExperiment()
        orders = [
            Order("A001", "A", 1, due_time=260),
            Order("A002", "A", 1, due_time=280),
            Order("A003", "A", 1, due_time=300),
            Order("B001", "B", 2, due_time=340),
            Order("C001", "C", 2, due_time=380),
            Order("C002", "C", 2, due_time=410),
            Order("URGENT_C", "C", 5, due_time=115, arrival_time=20),
        ]
        proposed = experiment.run_proposed(orders)
        records_by_order = {}
        for record in proposed.records:
            records_by_order.setdefault(record.order_id, []).append(record)

        assembly_windows = []
        inspection_windows = []
        for order_id, records in records_by_order.items():
            by_process = {record.process: record for record in records}
            assembly_windows.append((
                by_process["box_feed"].start_time,
                by_process["transfer_to_inspection"].end_time,
                order_id,
            ))
            sort_task = next(
                record for record in records if record.process in ("sort_good", "sort_defect")
            )
            inspection_windows.append((
                by_process["transfer_to_inspection"].start_time,
                sort_task.end_time,
                order_id,
            ))

        for windows in (assembly_windows, inspection_windows):
            ordered = sorted(windows)
            for previous, current in zip(ordered, ordered[1:]):
                self.assertLessEqual(previous[1], current[0], (previous, current))

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
        self.assertIn("screw", ordered_processes)
        self.assertLess(ordered_processes.index("inspect"), ordered_processes.index("sort_defect"))
        self.assertLess(ordered_processes.index("inspect"), ordered_processes.index("screw"))
        self.assertLess(ordered_processes.index("screw"), ordered_processes.index("sort_good"))
        self.assertEqual(sequence[0]["topology_level"], 1)
        self.assertEqual(max(row["level"] for row in sequence), 12)

        experiment = DiscreteEventExperiment()
        result = experiment.run_proposed([Order("A2", "A", 5, due_time=100)])
        steps = planner.expand_schedule_to_worksteps(result.records)
        self.assertTrue(any(step.step_label == "固定相机定位检测区域" for step in steps))
        self.assertTrue(any(step.step_label == "相机检测并输出 OK/NG" for step in steps))
        self.assertTrue(any(step.target_point == "R4_SCREW_PRESS" for step in steps))
        self.assertTrue(any(step.target_point in {"R5_GOOD_PLACE_TCP", "R5_DEFECT_PLACE_TCP"} for step in steps))

        balance = planner.line_balance_summary(steps)
        self.assertGreater(balance["balance_rate"], 0)
        self.assertIn(balance["bottleneck_resource"], balance["station_times"])
        self.assertTrue(planner.balance_recommendations(balance))


if __name__ == "__main__":
    unittest.main()
