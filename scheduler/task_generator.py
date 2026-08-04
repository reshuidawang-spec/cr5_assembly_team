"""配置驱动的任务生成器。"""

from pathlib import Path
from typing import Dict, List, Optional, Tuple

from interfaces.types import Order, Task, TaskStatus
from scheduler.config_loader import load_yaml


class TaskGenerator:
    """按产品工艺模板生成任务 DAG。"""

    def __init__(self, config_path: Optional[str] = None):
        root = Path(__file__).resolve().parents[1]
        self.config_path = Path(config_path) if config_path else root / "configs" / "product_types.yaml"
        self.product_config = self._load_config(self.config_path)
        self._task_counter = 0
        self.task_due_times: Dict[str, float] = {}
        self.task_arrival_times: Dict[str, float] = {}
        self.task_sequence: Dict[str, int] = {}

    def _load_config(self, path: Path) -> dict:
        return load_yaml(path)

    def next_task_id(self) -> str:
        self._task_counter += 1
        return f"T{self._task_counter:04d}"

    def generate(self, orders: List[Order]) -> List[Task]:
        tasks: List[Task] = []
        for order in orders:
            product_cfg = self.product_config.get(order.product_type)
            if not product_cfg:
                raise ValueError(f"未知产品类型: {order.product_type}")

            for unit_index in range(order.quantity):
                unit_suffix = "" if order.quantity == 1 else f"-{unit_index + 1:02d}"
                unit_order_id = f"{order.order_id}{unit_suffix}"
                tasks.extend(self._generate_unit_tasks(order, unit_order_id, product_cfg))
        return tasks

    def build_post_inspection_task(
        self,
        source_task: Task,
        quality_result: str,
    ) -> Optional[Task]:
        process_name = ""
        if source_task.process == "inspect" and quality_result in ("OK", "NG"):
            process_name = "screw"
        elif source_task.process == "screw":
            process_name = (
                "sort_good"
                if quality_result == "OK"
                else "sort_defect"
                if quality_result == "NG"
                else ""
            )
        if not process_name:
            return None

        for step in self.product_config.get("post_inspection", []):
            if step.get("process") == process_name:
                task = self._make_task(
                    order_id=source_task.order_id,
                    product_type=source_task.product_type,
                    step=step,
                    predecessor=source_task.task_id,
                    priority=source_task.priority,
                    due_time=self.task_due_times.get(source_task.task_id, 0.0),
                    arrival_time=self.task_arrival_times.get(source_task.task_id, 0.0),
                )
                return task
        return None

    def _generate_unit_tasks(self, order: Order, unit_order_id: str, product_cfg: dict) -> List[Task]:
        tasks: List[Task] = []
        previous_id: Optional[str] = None
        for step in product_cfg.get("processes", []):
            task = self._make_task(
                order_id=unit_order_id,
                product_type=order.product_type,
                step=step,
                predecessor=previous_id,
                priority=order.priority,
                due_time=order.due_time,
                arrival_time=order.arrival_time,
            )
            tasks.append(task)
            previous_id = task.task_id
        return tasks

    def _make_task(
        self,
        order_id: str,
        product_type: str,
        step: dict,
        predecessor: Optional[str],
        priority: int,
        due_time: float,
        arrival_time: float,
    ) -> Task:
        task_id = self.next_task_id()
        task = Task(
            task_id=task_id,
            order_id=order_id,
            product_type=product_type,
            process=step["process"],
            target_area=step["area"],
            target_point=step["point"],
            available_robots=list(step.get("available_robots", [])),
            duration=float(
                step.get("duration_by_product", {}).get(
                    product_type,
                    step.get("duration", 5.0),
                )
            ),
            predecessors=[predecessor] if predecessor else [],
            priority=priority,
            status=TaskStatus.PENDING.value if not predecessor else TaskStatus.WAITING.value,
            required_areas=list(step.get("required_areas", [])),
            scene_command=str(step.get("scene_done_cmd", "")),
        )
        self.task_due_times[task_id] = due_time
        self.task_arrival_times[task_id] = arrival_time
        self.task_sequence[task_id] = len(self.task_sequence)
        return task

    def task_score(
        self,
        task: Task,
        current_time: float = 0.0,
        ready_time: float = 0.0,
        remaining_work: Optional[float] = None,
        weights: Optional[Dict[str, float]] = None,
    ) -> float:
        """计算可解释的在线综合评分，分值越高越优先。"""
        cfg = weights or {}
        priority_weight = float(cfg.get("priority_weight", 0.45))
        due_weight = float(cfg.get("due_weight", 0.30))
        waiting_weight = float(cfg.get("waiting_weight", 0.15))
        critical_path_weight = float(cfg.get("critical_path_weight", 0.10))
        lateness_risk_weight = float(cfg.get("lateness_risk_weight", 0.0))
        bottleneck_penalty_weight = float(cfg.get("bottleneck_penalty_weight", 0.0))
        area_conflict_penalty_weight = float(cfg.get("area_conflict_penalty_weight", 0.0))
        post_inspection_clearance_bonus = float(cfg.get("post_inspection_clearance_bonus", 0.0))
        screw_clearance_bonus = float(cfg.get("screw_clearance_bonus", 0.0))
        sort_clearance_bonus = float(cfg.get("sort_clearance_bonus", 0.0))
        overdue_boost = float(cfg.get("overdue_boost", 0.0))
        urgent_due_boost = float(cfg.get("urgent_due_boost", 0.0))
        urgent_threshold = int(cfg.get("urgent_threshold", 5))
        bottleneck_resources = set(cfg.get("bottleneck_resources", []))
        conflict_sensitive_areas = set(cfg.get("conflict_sensitive_areas", []))
        aging_horizon = max(float(cfg.get("aging_horizon", 30.0)), 1.0)
        waiting_aging_shape = max(float(cfg.get("waiting_aging_shape", 1.0)), 0.1)
        urgency_horizon = max(float(cfg.get("urgency_horizon", 60.0)), 1.0)
        critical_horizon = max(float(cfg.get("critical_horizon", 60.0)), 1.0)

        priority = min(max(float(task.priority), 0.0) / 10.0, 1.0)
        waiting = min(max(current_time - ready_time, 0.0) / aging_horizon, 1.0)
        waiting = waiting ** (1.0 / waiting_aging_shape)
        remaining = max(float(remaining_work if remaining_work is not None else task.duration), 0.0)
        criticality = min(remaining / critical_horizon, 1.0)

        due_time = self.task_due_times.get(task.task_id, 0.0)
        urgency = 0.0
        lateness_risk = 0.0
        slack = None
        if due_time > 0:
            slack = due_time - current_time - remaining
            urgency = 1.0 if slack <= 0 else 1.0 / (1.0 + slack / urgency_horizon)
            lateness_risk = min(max(-slack, 0.0) / urgency_horizon, 2.0)

        score = (
            priority_weight * priority
            + due_weight * urgency
            + waiting_weight * waiting
            + critical_path_weight * criticality
            + lateness_risk_weight * lateness_risk
        )
        if slack is not None and slack <= 0:
            score += overdue_boost
        if task.priority >= urgent_threshold and (slack is None or slack <= urgency_horizon):
            score += urgent_due_boost
        if task.process in ("screw", "sort_good", "sort_defect"):
            score += post_inspection_clearance_bonus
        if task.process == "screw":
            score += screw_clearance_bonus
        if task.process in ("sort_good", "sort_defect"):
            score += sort_clearance_bonus
        if task.priority < urgent_threshold:
            if bottleneck_resources.intersection(task.available_robots):
                score -= bottleneck_penalty_weight
            if conflict_sensitive_areas.intersection(task.required_areas):
                score -= area_conflict_penalty_weight
        return score

    def task_sort_key(
        self,
        task: Task,
        current_time: float = 0.0,
        ready_time: float = 0.0,
        remaining_work: Optional[float] = None,
        weights: Optional[Dict[str, float]] = None,
    ) -> Tuple[float, int]:
        score = self.task_score(task, current_time, ready_time, remaining_work, weights)
        return (-score, self.task_sequence.get(task.task_id, 0))
