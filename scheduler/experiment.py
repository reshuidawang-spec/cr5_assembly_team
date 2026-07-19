"""离散事件调度实验器。

用于对比固定顺序 baseline 与动态优先级 proposed，不依赖真实机械臂或 GUI。
"""

from dataclasses import dataclass, asdict
import csv
import json
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

from interfaces.types import Order, Task
from scheduler.config_loader import load_yaml
from scheduler.task_generator import TaskGenerator


@dataclass
class ScheduleRecord:
    task_id: str
    order_id: str
    product_type: str
    process: str
    robot_id: str
    target_area: str
    start_time: float
    end_time: float
    wait_time: float
    priority: int


@dataclass
class ExperimentResult:
    mode: str
    makespan: float
    average_waiting_time: float
    urgent_completion_time: float
    urgent_response_time: float
    throughput: float
    parallel_efficiency: float
    weighted_tardiness: float
    conflict_count: int
    robot_utilization: Dict[str, float]
    order_completion_times: Dict[str, float]
    robot_idle_time: Dict[str, float]
    records: List[ScheduleRecord]

    def summary_dict(self) -> dict:
        return {
            "mode": self.mode,
            "makespan": self.makespan,
            "average_waiting_time": self.average_waiting_time,
            "urgent_completion_time": self.urgent_completion_time,
            "urgent_response_time": self.urgent_response_time,
            "throughput": self.throughput,
            "parallel_efficiency": self.parallel_efficiency,
            "weighted_tardiness": self.weighted_tardiness,
            "conflict_count": self.conflict_count,
            "robot_utilization": self.robot_utilization,
            "order_completion_times": self.order_completion_times,
            "robot_idle_time": self.robot_idle_time,
        }


class DiscreteEventExperiment:
    """调度策略离散事件仿真。"""

    def __init__(self):
        root = Path(__file__).resolve().parents[1]
        self.root = root
        self.product_config_path = root / "configs" / "product_types.yaml"
        self.scheduler_config = load_yaml(root / "configs" / "scheduler.yaml").get("scheduler", {})
        self.urgent_threshold = int(self.scheduler_config.get("urgent_threshold", 5))
        self.scoring_config = self.scheduler_config.get("scoring", {})
        self.sort_trigger_process = str(self.scheduler_config.get("sort_trigger_process", "inspect"))

    def run_baseline(self, orders: List[Order]) -> ExperimentResult:
        """固定订单顺序：一个订单完整做完，再做下一个订单。"""
        generator = TaskGenerator(str(self.product_config_path))
        tasks = generator.generate(orders)
        current_time = 0.0
        ready_time: Dict[str, float] = {}
        records: List[ScheduleRecord] = []
        busy_time = self._empty_busy_time()
        quality_by_order: Dict[str, str] = {}

        for task in tasks:
            robot_id = task.available_robots[0]
            arrival = generator.task_arrival_times.get(task.task_id, 0.0)
            if not task.predecessors:
                ready_time[task.task_id] = arrival
            else:
                ready_time.setdefault(task.task_id, current_time)
            start = max(current_time, arrival)
            end = start + task.duration
            records.append(self._record(
                task, robot_id, start, end, start - ready_time[task.task_id]
            ))
            busy_time[robot_id] += task.duration
            current_time = end

            post_task = self._build_sort_task_if_ready(generator, task, quality_by_order)
            if post_task:
                robot_id = post_task.available_robots[0]
                start = current_time
                end = start + post_task.duration
                records.append(self._record(post_task, robot_id, start, end, 0.0))
                busy_time[robot_id] += post_task.duration
                current_time = end

        return self._result("baseline", records, busy_time, conflict_count=0, orders=orders)

    def run_parallel_fifo(self, orders: List[Order]) -> ExperimentResult:
        """并行 FIFO：与 proposed 使用相同资源和区域约束，只改变排序策略。"""
        return self._run_parallel(orders, mode="parallel_fifo", use_scoring=False)

    def run_proposed(self, orders: List[Order]) -> ExperimentResult:
        """动态综合评分：多机械臂并行、交期感知、等待老化和区域锁约束。"""
        return self._run_parallel(orders, mode="proposed", use_scoring=True)

    def _run_parallel(
        self,
        orders: List[Order],
        mode: str,
        use_scoring: bool,
    ) -> ExperimentResult:
        generator = TaskGenerator(str(self.product_config_path))
        tasks = generator.generate(orders)
        task_map: Dict[str, Task] = {task.task_id: task for task in tasks}
        ready_time: Dict[str, float] = {
            task.task_id: generator.task_arrival_times.get(task.task_id, 0.0)
            for task in tasks
            if not task.predecessors
        }
        finished: Set[str] = set()
        running: List[Tuple[float, Task, str]] = []
        robot_available = self._empty_busy_time()
        busy_time = self._empty_busy_time()
        area_locks: Dict[str, Set[str]] = {}
        robot_reservations: Dict[str, float] = {}
        records: List[ScheduleRecord] = []
        conflict_count = 0
        current_time = 0.0
        quality_by_order: Dict[str, str] = {}

        while len(finished) < len(task_map):
            assigned = True
            while assigned:
                assigned = False
                idle_robots = {
                    rid for rid, available_time in robot_available.items()
                    if available_time <= current_time
                    and not any(item[2] == rid for item in running)
                }
                candidates = [
                    task for task in task_map.values()
                    if task.task_id not in finished
                    and not any(item[1].task_id == task.task_id for item in running)
                    and generator.task_arrival_times.get(task.task_id, 0.0) <= current_time
                    and self._predecessors_done(task, finished)
                    and any(robot in idle_robots for robot in task.available_robots)
                ]
                if use_scoring:
                    candidates.sort(
                        key=lambda task: generator.task_sort_key(
                            task,
                            current_time=current_time,
                            ready_time=ready_time.get(task.task_id, current_time),
                            remaining_work=self._remaining_order_work(task, task_map, finished),
                            weights=self.scoring_config,
                        )
                    )
                else:
                    candidates.sort(
                        key=lambda task: generator.task_sequence.get(task.task_id, 0)
                    )

                for task in candidates:
                    robot_id = self._select_robot(
                        task,
                        idle_robots,
                        reservations=robot_reservations,
                        current_time=current_time,
                    )
                    if not robot_id:
                        continue
                    if not self._lock_area(task, area_locks):
                        conflict_count += 1
                        continue
                    start = current_time
                    end = start + task.duration
                    wait_time = start - ready_time.setdefault(task.task_id, current_time)
                    records.append(self._record(task, robot_id, start, end, wait_time))
                    busy_time[robot_id] += task.duration
                    robot_available[robot_id] = end
                    running.append((end, task, robot_id))
                    idle_robots.remove(robot_id)
                    if use_scoring and task.priority >= self.urgent_threshold:
                        self._reserve_successor_robots(
                            task, task_map, end, robot_reservations
                        )
                    assigned = True

            if not running:
                future_arrivals = [
                    generator.task_arrival_times.get(task.task_id, 0.0)
                    for task in task_map.values()
                    if task.task_id not in finished
                    and not task.predecessors
                    and generator.task_arrival_times.get(task.task_id, 0.0) > current_time
                ]
                if future_arrivals:
                    current_time = min(future_arrivals)
                    continue
                break

            next_end = min(item[0] for item in running)
            current_time = next_end
            completed_now = [item for item in running if item[0] == next_end]
            running = [item for item in running if item[0] != next_end]

            for _, task, _ in completed_now:
                finished.add(task.task_id)
                self._release_area(task, area_locks)
                post_task = self._build_sort_task_if_ready(generator, task, quality_by_order)
                if post_task:
                    task_map[post_task.task_id] = post_task
                    ready_time[post_task.task_id] = current_time
                self._mark_newly_ready(task_map, finished, ready_time, current_time)

        return self._result(mode, records, busy_time, conflict_count, orders=orders)

    def run_fault_scenario(
        self,
        orders: List[Order],
        fault_robot: str = "R4",
        fault_start: float = 82.0,
        repair_duration: float = 18.0,
        mode_name: Optional[str] = None,
        use_scoring: bool = True,
    ) -> ExperimentResult:
        """动态调度 + 机械臂故障恢复场景。

        故障期间，目标机械臂不能接收新任务；若故障发生时该机械臂正在执行任务，
        任务中断并在维修完成后重新进入候选队列。
        """
        generator = TaskGenerator(str(self.product_config_path))
        tasks = generator.generate(orders)
        task_map: Dict[str, Task] = {task.task_id: task for task in tasks}
        ready_time: Dict[str, float] = {
            task.task_id: generator.task_arrival_times.get(task.task_id, 0.0)
            for task in tasks
            if not task.predecessors
        }
        finished: Set[str] = set()
        running: List[Tuple[float, Task, str]] = []
        robot_available = self._empty_busy_time()
        busy_time = self._empty_busy_time()
        area_locks: Dict[str, Set[str]] = {}
        records: List[ScheduleRecord] = []
        conflict_count = 0
        current_time = 0.0
        quality_by_order: Dict[str, str] = {}
        fault_end = fault_start + repair_duration
        fault_triggered = False

        while len(finished) < len(task_map):
            if not fault_triggered and current_time >= fault_start:
                running = self._interrupt_faulted_robot(
                    running,
                    fault_robot,
                    fault_start,
                    records,
                    busy_time,
                    area_locks,
                    ready_time,
                )
                robot_available[fault_robot] = fault_end
                current_time = max(current_time, fault_start)
                fault_triggered = True

            assigned = True
            while assigned:
                assigned = False
                idle_robots = {
                    rid for rid, available_time in robot_available.items()
                    if available_time <= current_time
                    and not any(item[2] == rid for item in running)
                    and not (rid == fault_robot and fault_start <= current_time < fault_end)
                }
                candidates = [
                    task for task in task_map.values()
                    if task.task_id not in finished
                    and not any(item[1].task_id == task.task_id for item in running)
                    and generator.task_arrival_times.get(task.task_id, 0.0) <= current_time
                    and self._predecessors_done(task, finished)
                    and any(robot in idle_robots for robot in task.available_robots)
                ]
                if use_scoring:
                    candidates.sort(
                        key=lambda task: generator.task_sort_key(
                            task,
                            current_time=current_time,
                            ready_time=ready_time.get(task.task_id, current_time),
                            remaining_work=self._remaining_order_work(task, task_map, finished),
                            weights=self.scoring_config,
                        )
                    )
                else:
                    candidates.sort(
                        key=lambda task: generator.task_sequence.get(task.task_id, 0)
                    )

                for task in candidates:
                    robot_id = self._select_robot(task, idle_robots)
                    if not robot_id:
                        continue
                    if not self._lock_area(task, area_locks):
                        conflict_count += 1
                        continue
                    start = current_time
                    end = start + task.duration
                    wait_time = start - ready_time.setdefault(task.task_id, current_time)
                    records.append(self._record(task, robot_id, start, end, wait_time))
                    busy_time[robot_id] += task.duration
                    robot_available[robot_id] = end
                    running.append((end, task, robot_id))
                    idle_robots.remove(robot_id)
                    assigned = True

            if not running:
                future_times = [value for value in robot_available.values() if value > current_time]
                future_times.extend(
                    generator.task_arrival_times.get(task.task_id, 0.0)
                    for task in task_map.values()
                    if task.task_id not in finished
                    and not task.predecessors
                    and generator.task_arrival_times.get(task.task_id, 0.0) > current_time
                )
                if not fault_triggered and fault_start > current_time:
                    future_times.append(fault_start)
                if not future_times:
                    break
                current_time = min(future_times)
                continue

            next_end = min(item[0] for item in running)
            if not fault_triggered and fault_start < next_end:
                current_time = fault_start
                continue
            current_time = next_end
            completed_now = [item for item in running if item[0] == next_end]
            running = [item for item in running if item[0] != next_end]

            for _, task, _ in completed_now:
                finished.add(task.task_id)
                self._release_area(task, area_locks)
                post_task = self._build_sort_task_if_ready(generator, task, quality_by_order)
                if post_task:
                    task_map[post_task.task_id] = post_task
                    ready_time[post_task.task_id] = current_time
                self._mark_newly_ready(task_map, finished, ready_time, current_time)

        return self._result(
            mode_name or f"fault_{fault_robot.lower()}",
            records,
            busy_time,
            conflict_count + 1,
            orders=orders,
        )

    def run_fault_matrix(
        self,
        orders: List[Order],
        scenarios: Optional[List[Tuple[str, str, float, float]]] = None,
        use_scoring: bool = True,
    ) -> List[ExperimentResult]:
        """Run a batch of production-time fault scenarios.

        Each scenario tuple is: (mode_name, fault_robot, fault_start, repair_duration).
        The default set targets the key working window of each resource in the
        five-CR5A cell, so the result shows which resource is most sensitive.
        """
        scenarios = scenarios or [
            ("fault_r1_key_window", "R1", 56.0, 18.0),
            ("fault_r2_key_window", "R2", 31.0, 18.0),
            ("fault_r3_key_window", "R3", 45.0, 18.0),
            ("fault_r4_key_window", "R4", 82.0, 18.0),
            ("fault_r5_key_window", "R5", 93.0, 18.0),
            ("fault_camera_key_window", "CAMERA", 73.0, 18.0),
        ]
        return [
            self.run_fault_scenario(
                orders,
                fault_robot=robot,
                fault_start=fault_start,
                repair_duration=repair_duration,
                mode_name=mode,
                use_scoring=use_scoring,
            )
            for mode, robot, fault_start, repair_duration in scenarios
        ]

    def run_fault_timing_scenarios(
        self,
        orders: List[Order],
        fault_robot: str = "R4",
        repair_duration: float = 18.0,
        use_scoring: bool = True,
    ) -> List[ExperimentResult]:
        """Run early/middle/late failures for one bottleneck resource."""
        timing_points = [
            ("early", 35.0),
            ("middle", 82.0),
            ("late", 140.0),
        ]
        return [
            self.run_fault_scenario(
                orders,
                fault_robot=fault_robot,
                fault_start=fault_start,
                repair_duration=repair_duration,
                mode_name=f"fault_{fault_robot.lower()}_{label}",
                use_scoring=use_scoring,
            )
            for label, fault_start in timing_points
        ]

    def export_results(self, results: List[ExperimentResult], output_dir: Optional[str] = None) -> Path:
        out = Path(output_dir) if output_dir else self.root / "data" / "results"
        out.mkdir(parents=True, exist_ok=True)

        for result in results:
            csv_path = out / f"{result.mode}_schedule.csv"
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(asdict(result.records[0]).keys()))
                writer.writeheader()
                for record in result.records:
                    writer.writerow(asdict(record))

        summary_path = out / "metrics_summary.json"
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump([result.summary_dict() for result in results], f, ensure_ascii=False, indent=2)
        self._export_charts(results, out)
        return out

    def _result(
        self,
        mode: str,
        records: List[ScheduleRecord],
        busy_time: Dict[str, float],
        conflict_count: int,
        orders: List[Order],
    ) -> ExperimentResult:
        makespan = max((record.end_time for record in records), default=0.0)
        avg_wait = sum(record.wait_time for record in records) / len(records) if records else 0.0
        order_attributes = self._order_attributes(orders)
        urgent_done = max(
            (record.end_time for record in records if record.priority >= self.urgent_threshold),
            default=0.0,
        )
        urgent_first_start = min(
            (
                record.start_time
                - order_attributes.get(record.order_id, {}).get("arrival_time", 0.0)
                for record in records
                if record.priority >= self.urgent_threshold
            ),
            default=0.0,
        )
        order_completion = self._order_completion_times(records)
        weighted_tardiness = sum(
            order_attributes.get(order_id, {}).get("priority", 1.0)
            * max(
                completion
                - order_attributes.get(order_id, {}).get("due_time", completion),
                0.0,
            )
            for order_id, completion in order_completion.items()
            if order_attributes.get(order_id, {}).get("due_time", 0.0) > 0
        )
        total_work = sum(busy_time.values())
        utilization = {
            robot_id: (time_used / makespan if makespan else 0.0)
            for robot_id, time_used in busy_time.items()
        }
        idle_time = {
            robot_id: max(makespan - time_used, 0.0)
            for robot_id, time_used in busy_time.items()
        }
        return ExperimentResult(
            mode=mode,
            makespan=makespan,
            average_waiting_time=avg_wait,
            urgent_completion_time=urgent_done,
            urgent_response_time=urgent_first_start,
            throughput=(len(order_completion) / makespan if makespan else 0.0),
            parallel_efficiency=(total_work / (makespan * len(busy_time)) if makespan else 0.0),
            weighted_tardiness=weighted_tardiness,
            conflict_count=conflict_count,
            robot_utilization=utilization,
            order_completion_times=order_completion,
            robot_idle_time=idle_time,
            records=records,
        )

    def _record(
        self,
        task: Task,
        robot_id: str,
        start: float,
        end: float,
        wait_time: float,
    ) -> ScheduleRecord:
        return ScheduleRecord(
            task_id=task.task_id,
            order_id=task.order_id,
            product_type=task.product_type,
            process=task.process,
            robot_id=robot_id,
            target_area=task.target_area,
            start_time=start,
            end_time=end,
            wait_time=wait_time,
            priority=task.priority,
        )

    def _empty_busy_time(self) -> Dict[str, float]:
        return {"R1": 0.0, "R2": 0.0, "R3": 0.0, "R4": 0.0, "R5": 0.0, "CAMERA": 0.0}

    def _build_sort_task_if_ready(
        self,
        generator: TaskGenerator,
        task: Task,
        quality_by_order: Dict[str, str],
    ) -> Optional[Task]:
        if task.process == "inspect":
            quality_by_order[task.order_id] = self._quality_for(task)
        if task.process != self.sort_trigger_process:
            return None
        return generator.build_post_inspection_task(
            task,
            quality_by_order.get(task.order_id, ""),
        )

    def _order_completion_times(self, records: List[ScheduleRecord]) -> Dict[str, float]:
        completion: Dict[str, float] = {}
        for record in records:
            completion[record.order_id] = max(completion.get(record.order_id, 0.0), record.end_time)
        return completion

    def _order_attributes(self, orders: List[Order]) -> Dict[str, Dict[str, float]]:
        attributes: Dict[str, Dict[str, float]] = {}
        for order in orders:
            for unit_index in range(order.quantity):
                suffix = "" if order.quantity == 1 else f"-{unit_index + 1:02d}"
                attributes[f"{order.order_id}{suffix}"] = {
                    "priority": float(order.priority),
                    "due_time": float(order.due_time),
                    "arrival_time": float(order.arrival_time),
                }
        return attributes

    def _remaining_order_work(
        self,
        task: Task,
        task_map: Dict[str, Task],
        finished: Set[str],
    ) -> float:
        return sum(
            item.duration
            for item in task_map.values()
            if item.order_id == task.order_id and item.task_id not in finished
        )

    def _reserve_successor_robots(
        self,
        task: Task,
        task_map: Dict[str, Task],
        ready_at: float,
        reservations: Dict[str, float],
    ) -> Set[str]:
        reserved: Set[str] = set()
        for successor in task_map.values():
            if task.task_id not in successor.predecessors:
                continue
            for robot_id in successor.available_robots:
                reserved.add(robot_id)
                reservations[robot_id] = max(
                    reservations.get(robot_id, 0.0),
                    ready_at,
                )
        return reserved

    def _mark_newly_ready(
        self,
        task_map: Dict[str, Task],
        finished: Set[str],
        ready_time: Dict[str, float],
        current_time: float,
    ) -> None:
        for task in task_map.values():
            if task.task_id not in ready_time and self._predecessors_done(task, finished):
                ready_time[task.task_id] = current_time

    def _quality_for(self, task: Task) -> str:
        """确定性质量样本，与订单优先级无关，便于策略公平对比。"""
        checksum = sum(ord(char) for char in task.order_id)
        return "NG" if checksum % 3 == 0 else "OK"

    def _interrupt_faulted_robot(
        self,
        running: List[Tuple[float, Task, str]],
        fault_robot: str,
        fault_start: float,
        records: List[ScheduleRecord],
        busy_time: Dict[str, float],
        area_locks: Dict[str, Set[str]],
        ready_time: Dict[str, float],
    ) -> List[Tuple[float, Task, str]]:
        remaining = []
        for end_time, task, robot_id in running:
            if robot_id != fault_robot:
                remaining.append((end_time, task, robot_id))
                continue
            for index in range(len(records) - 1, -1, -1):
                record = records[index]
                if record.task_id == task.task_id and record.robot_id == robot_id:
                    busy_time[robot_id] -= max(record.end_time - record.start_time, 0.0)
                    del records[index]
                    break
            ready_time[task.task_id] = fault_start
            self._release_area(task, area_locks)
        return remaining

    def _predecessors_done(self, task: Task, finished: Set[str]) -> bool:
        return all(pred in finished for pred in task.predecessors)

    def _select_robot(
        self,
        task: Task,
        idle_robots: Set[str],
        reservations: Optional[Dict[str, float]] = None,
        current_time: float = 0.0,
    ) -> Optional[str]:
        reservations = reservations or {}
        for robot_id in task.available_robots:
            if robot_id not in idle_robots:
                continue
            reserved_until = reservations.get(robot_id, 0.0)
            if (
                task.priority < self.urgent_threshold
                and reserved_until > current_time
                and current_time + task.duration > reserved_until
            ):
                continue
            return robot_id
        return None

    def _lock_area(self, task: Task, area_locks: Dict[str, Set[str]]) -> bool:
        areas = [area for area in self._task_lock_areas(task) if self._area_needs_lock(area)]
        for area in areas:
            owners = area_locks.get(area, set())
            if task.task_id not in owners and len(owners) >= self._area_capacity(area):
                return False
        for area in areas:
            area_locks.setdefault(area, set()).add(task.task_id)
        return True

    def _release_area(self, task: Task, area_locks: Dict[str, Set[str]]) -> None:
        for area in self._task_lock_areas(task):
            owners = area_locks.get(area)
            if not owners:
                continue
            owners.discard(task.task_id)
            if not owners:
                del area_locks[area]

    def _task_lock_areas(self, task: Task) -> List[str]:
        return task.required_areas or [task.target_area]

    def _area_needs_lock(self, area: str) -> bool:
        areas = self.scheduler_config.get("areas", {})
        return bool(areas.get(area, {}).get("lock_required", False))

    def _area_capacity(self, area: str) -> int:
        areas = self.scheduler_config.get("areas", {})
        return max(int(areas.get(area, {}).get("max_robots", 1)), 1)

    def _resource_order(self, results: List[ExperimentResult]) -> List[str]:
        preferred = ["R1", "R2", "R3", "R4", "R5", "CAMERA"]
        seen = {
            robot
            for result in results
            for robot in result.robot_utilization.keys()
        }
        ordered = [robot for robot in preferred if robot in seen]
        ordered.extend(sorted(seen.difference(ordered)))
        return ordered

    def _export_charts(self, results: List[ExperimentResult], out: Path) -> None:
        self._export_svg_charts(results, out)
        try:
            import matplotlib.pyplot as plt
        except Exception:
            return

        names = [result.mode for result in results]
        makespans = [result.makespan for result in results]
        urgent_times = [result.urgent_completion_time for result in results]

        self._bar_chart(
            plt,
            names,
            makespans,
            "Makespan Compare",
            "Time (s)",
            out / "makespan_compare.png",
        )
        self._bar_chart(
            plt,
            names,
            urgent_times,
            "Urgent Order Completion",
            "Time (s)",
            out / "urgent_compare.png",
        )
        self._utilization_chart(plt, results, out / "utilization_compare.png")
        proposed = next((result for result in results if result.mode == "proposed"), results[-1])
        self._gantt_chart(plt, proposed, out / "gantt_proposed.png")

    def _export_svg_charts(self, results: List[ExperimentResult], out: Path) -> None:
        names = [result.mode for result in results]
        self._svg_bar_chart(
            names,
            [result.makespan for result in results],
            "Makespan Compare",
            "Time (s)",
            out / "makespan_compare.svg",
        )
        self._svg_bar_chart(
            names,
            [result.urgent_completion_time for result in results],
            "Urgent Order Completion",
            "Time (s)",
            out / "urgent_compare.svg",
        )
        self._svg_utilization_chart(results, out / "utilization_compare.svg")
        proposed = next((result for result in results if result.mode == "proposed"), results[-1])
        self._svg_gantt_chart(proposed, out / "gantt_proposed.svg")

    def _svg_bar_chart(
        self,
        labels: List[str],
        values: List[float],
        title: str,
        ylabel: str,
        path: Path,
    ) -> None:
        width, height = 760, 420
        margin_left, margin_bottom, margin_top = 70, 70, 55
        chart_h = height - margin_top - margin_bottom
        chart_w = width - margin_left - 40
        max_value = max(values) if values else 1.0
        colors = ["#8b949e", "#238636", "#d29922", "#f85149"]
        bar_w = chart_w / max(len(values), 1) * 0.55
        parts = [self._svg_header(width, height)]
        parts.append(f'<text x="{width/2}" y="30" text-anchor="middle" class="title">{title}</text>')
        parts.append(f'<text x="18" y="{height/2}" transform="rotate(-90 18 {height/2})" class="axis">{ylabel}</text>')
        parts.append(f'<line x1="{margin_left}" y1="{height-margin_bottom}" x2="{width-30}" y2="{height-margin_bottom}" class="axis-line"/>')
        parts.append(f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{height-margin_bottom}" class="axis-line"/>')
        for index, (label, value) in enumerate(zip(labels, values)):
            x_center = margin_left + (index + 0.5) * chart_w / len(values)
            h = (value / max_value) * chart_h if max_value else 0
            y = height - margin_bottom - h
            x = x_center - bar_w / 2
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{h:.1f}" fill="{colors[index % len(colors)]}"/>')
            parts.append(f'<text x="{x_center:.1f}" y="{y-8:.1f}" text-anchor="middle" class="label">{value:.1f}</text>')
            parts.append(f'<text x="{x_center:.1f}" y="{height-35}" text-anchor="middle" class="label">{label}</text>')
        parts.append("</svg>")
        path.write_text("\n".join(parts), encoding="utf-8")

    def _svg_utilization_chart(self, results: List[ExperimentResult], path: Path) -> None:
        width, height = 820, 430
        robots = self._resource_order(results)
        margin_left, margin_bottom, margin_top = 70, 70, 55
        chart_h = height - margin_top - margin_bottom
        chart_w = width - margin_left - 40
        colors = ["#8b949e", "#238636", "#d29922", "#f85149"]
        group_w = chart_w / len(robots)
        bar_w = group_w * 0.7 / max(len(results), 1)
        parts = [self._svg_header(width, height)]
        parts.append(f'<text x="{width/2}" y="30" text-anchor="middle" class="title">Robot Utilization Compare</text>')
        parts.append(f'<line x1="{margin_left}" y1="{height-margin_bottom}" x2="{width-30}" y2="{height-margin_bottom}" class="axis-line"/>')
        parts.append(f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{height-margin_bottom}" class="axis-line"/>')
        for r_index, robot in enumerate(robots):
            group_x = margin_left + r_index * group_w
            center = group_x + group_w / 2
            parts.append(f'<text x="{center:.1f}" y="{height-35}" text-anchor="middle" class="label">{robot}</text>')
            for index, result in enumerate(results):
                value = result.robot_utilization.get(robot, 0.0) * 100
                h = (value / 100) * chart_h
                x = group_x + group_w * 0.15 + index * bar_w
                y = height - margin_bottom - h
                parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w-2:.1f}" height="{h:.1f}" fill="{colors[index % len(colors)]}"/>')
                parts.append(f'<text x="{x + bar_w/2:.1f}" y="{y-6:.1f}" text-anchor="middle" class="tiny">{value:.0f}%</text>')
        for index, result in enumerate(results):
            x = width - 190
            y = margin_top + index * 22
            parts.append(f'<rect x="{x}" y="{y-12}" width="12" height="12" fill="{colors[index % len(colors)]}"/>')
            parts.append(f'<text x="{x+18}" y="{y-2}" class="label">{result.mode}</text>')
        parts.append("</svg>")
        path.write_text("\n".join(parts), encoding="utf-8")

    def _svg_gantt_chart(self, result: ExperimentResult, path: Path) -> None:
        robots = self._resource_order([result])
        width, height = 1100, max(520, 120 + 82 * len(robots))
        margin_left, margin_top, margin_bottom = 80, 60, 50
        chart_w = width - margin_left - 40
        row_h = 82
        max_time = max((record.end_time for record in result.records), default=1.0)
        colors = {
            "box_feed": "#58a6ff",
            "pcb_install": "#3fb950",
            "module_install": "#6f42c1",
            "terminal_install": "#dbab09",
            "transfer_to_inspection": "#0969da",
            "screw": "#d29922",
            "inspect": "#a371f7",
            "sort_good": "#238636",
            "sort_defect": "#f85149",
        }
        parts = [self._svg_header(width, height)]
        parts.append(f'<text x="{width/2}" y="30" text-anchor="middle" class="title">Proposed Dynamic Schedule Gantt Chart</text>')
        for idx, robot in enumerate(robots):
            y = margin_top + idx * row_h
            parts.append(f'<text x="40" y="{y+34}" text-anchor="middle" class="label">{robot}</text>')
            parts.append(f'<line x1="{margin_left}" y1="{y+42}" x2="{width-30}" y2="{y+42}" stroke="#d0d7de" stroke-width="1"/>')
        for record in result.records:
            y = margin_top + robots.index(record.robot_id) * row_h + 16
            x = margin_left + record.start_time / max_time * chart_w
            w = max((record.end_time - record.start_time) / max_time * chart_w, 2)
            label = f"{record.order_id}-{record.process}"
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="28" rx="3" fill="{colors.get(record.process, "#8b949e")}"/>')
            parts.append(f'<text x="{x+4:.1f}" y="{y+18:.1f}" class="tiny" fill="#ffffff">{label}</text>')
        parts.append(f'<line x1="{margin_left}" y1="{height-margin_bottom}" x2="{width-30}" y2="{height-margin_bottom}" class="axis-line"/>')
        for tick in range(0, int(max_time) + 1, max(int(max_time // 6), 1)):
            x = margin_left + tick / max_time * chart_w
            parts.append(f'<line x1="{x:.1f}" y1="{height-margin_bottom}" x2="{x:.1f}" y2="{height-margin_bottom+6}" class="axis-line"/>')
            parts.append(f'<text x="{x:.1f}" y="{height-margin_bottom+24}" text-anchor="middle" class="tiny">{tick}s</text>')
        parts.append("</svg>")
        path.write_text("\n".join(parts), encoding="utf-8")

    def _svg_header(self, width: int, height: int) -> str:
        return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<style>
    .title {{ font: 700 18px Arial, sans-serif; fill: #24292f; }}
    .label {{ font: 12px Arial, sans-serif; fill: #24292f; }}
    .tiny {{ font: 10px Arial, sans-serif; fill: #24292f; }}
    .axis {{ font: 12px Arial, sans-serif; fill: #57606a; }}
    .axis-line {{ stroke: #57606a; stroke-width: 1; }}
</style>
<rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff"/>'''

    def _bar_chart(self, plt, labels: List[str], values: List[float], title: str, ylabel: str, path: Path) -> None:
        plt.figure(figsize=(7, 4))
        bars = plt.bar(labels, values, color=["#8b949e", "#238636", "#d29922"][: len(labels)])
        plt.title(title)
        plt.ylabel(ylabel)
        for bar, value in zip(bars, values):
            plt.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.1f}", ha="center", va="bottom")
        plt.tight_layout()
        plt.savefig(path, dpi=160)
        plt.close()

    def _utilization_chart(self, plt, results: List[ExperimentResult], path: Path) -> None:
        robots = self._resource_order(results)
        x = range(len(robots))
        width = 0.8 / max(len(results), 1)
        plt.figure(figsize=(8, 4))
        for idx, result in enumerate(results):
            offset = (idx - (len(results) - 1) / 2) * width
            values = [result.robot_utilization.get(robot, 0.0) * 100 for robot in robots]
            plt.bar([item + offset for item in x], values, width=width, label=result.mode)
        plt.xticks(list(x), robots)
        plt.ylabel("Utilization (%)")
        plt.title("Robot Utilization Compare")
        plt.legend()
        plt.tight_layout()
        plt.savefig(path, dpi=160)
        plt.close()

    def _gantt_chart(self, plt, result: ExperimentResult, path: Path) -> None:
        robots = self._resource_order([result])
        colors = {
            "box_feed": "#58a6ff",
            "pcb_install": "#3fb950",
            "module_install": "#6f42c1",
            "terminal_install": "#dbab09",
            "transfer_to_inspection": "#0969da",
            "screw": "#d29922",
            "inspect": "#a371f7",
            "sort_good": "#238636",
            "sort_defect": "#f85149",
        }
        plt.figure(figsize=(10, 4.8))
        for record in result.records:
            y = robots.index(record.robot_id)
            plt.barh(
                y,
                record.end_time - record.start_time,
                left=record.start_time,
                color=colors.get(record.process, "#8b949e"),
                edgecolor="black",
                height=0.45,
            )
            label = f"{record.order_id}-{record.process}"
            plt.text(record.start_time + 0.2, y, label, va="center", fontsize=7)
        plt.yticks(range(len(robots)), robots)
        plt.xlabel("Time (s)")
        plt.title("Proposed Dynamic Schedule Gantt Chart")
        plt.tight_layout()
        plt.savefig(path, dpi=160)
        plt.close()
