"""Assembly component planning and step-level time simulation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

from scheduler.config_loader import load_yaml
from scheduler.experiment import ScheduleRecord


@dataclass(frozen=True)
class AssemblyNode:
    node_id: str
    name: str
    kind: str
    process: str
    resource: str
    level: int
    dependencies: List[str]
    granularity: str = ""
    scene_object: str = ""
    description: str = ""


@dataclass(frozen=True)
class WorkStepRecord:
    order_id: str
    task_id: str
    process: str
    step_code: str
    step_label: str
    robot_id: str
    target_point: str
    start_time: float
    end_time: float
    duration: float
    level: int


class AssemblyProcessPlanner:
    """Load component metadata, sort it by hierarchy, and expand schedules."""

    def __init__(self, config_path: Optional[str] = None):
        root = Path(__file__).resolve().parents[1]
        self.config_path = Path(config_path) if config_path else root / "configs" / "assembly_components.yaml"
        self.config = load_yaml(self.config_path)
        self.nodes = self._load_nodes()
        self.process_to_level = {
            node.process: node.level
            for node in self.nodes.values()
            if node.process
        }

    def _load_nodes(self) -> Dict[str, AssemblyNode]:
        nodes: Dict[str, AssemblyNode] = {}
        for node_id, raw in self.config.get("components", {}).items():
            nodes[node_id] = AssemblyNode(
                node_id=node_id,
                name=raw.get("name", node_id),
                kind="component",
                process=raw.get("install_process", ""),
                resource=raw.get("install_robot", ""),
                level=int(raw.get("install_level", 0)),
                dependencies=list(raw.get("dependencies", [])),
                granularity=raw.get("granularity", "main_component"),
                scene_object=raw.get("scene_object", ""),
                description=raw.get("description", ""),
            )
        for node_id, raw in self.config.get("operations", {}).items():
            nodes[node_id] = AssemblyNode(
                node_id=node_id,
                name=raw.get("name", node_id),
                kind="operation",
                process=node_id,
                resource=raw.get("robot", ""),
                level=int(raw.get("level", 0)),
                dependencies=list(raw.get("dependencies", [])),
                granularity="operation",
            )
        return nodes

    def hierarchical_topological_sort(self) -> List[List[AssemblyNode]]:
        """Return dependency-safe installation levels.

        Nodes in the same returned list have no dependency relation between each
        other, so they are theoretically candidates for the same assembly layer.
        """
        indegree = {node_id: 0 for node_id in self.nodes}
        children = {node_id: [] for node_id in self.nodes}
        for node_id, node in self.nodes.items():
            for dep in node.dependencies:
                if dep not in self.nodes:
                    raise ValueError(f"Unknown dependency {dep!r} for {node_id!r}")
                indegree[node_id] += 1
                children[dep].append(node_id)

        ready = sorted(
            [node_id for node_id, degree in indegree.items() if degree == 0],
            key=self._node_order_key,
        )
        levels: List[List[AssemblyNode]] = []
        visited = 0
        while ready:
            current_ids = ready
            ready = []
            levels.append([self.nodes[node_id] for node_id in current_ids])
            visited += len(current_ids)
            for node_id in current_ids:
                for child in children[node_id]:
                    indegree[child] -= 1
                    if indegree[child] == 0:
                        ready.append(child)
            ready.sort(key=self._node_order_key)

        if visited != len(self.nodes):
            raise ValueError("Assembly component graph has a dependency cycle")
        return levels

    def component_sequence_rows(self) -> List[dict]:
        rows = []
        for level_index, level in enumerate(self.hierarchical_topological_sort(), start=1):
            for node in level:
                rows.append({
                    "topology_level": level_index,
                    **asdict(node),
                })
        return rows

    def expand_schedule_to_worksteps(
        self,
        records: Sequence[ScheduleRecord],
    ) -> List[WorkStepRecord]:
        rows: List[WorkStepRecord] = []
        worksteps = self.config.get("worksteps", {})
        for record in records:
            steps = worksteps.get(record.process)
            if not steps:
                rows.append(self._single_step(record))
                continue
            task_duration = max(record.end_time - record.start_time, 0.0)
            cursor = record.start_time
            for index, step in enumerate(steps):
                if index == len(steps) - 1:
                    end = record.end_time
                else:
                    end = cursor + task_duration * float(step.get("ratio", 0.0))
                rows.append(WorkStepRecord(
                    order_id=record.order_id,
                    task_id=record.task_id,
                    process=record.process,
                    step_code=step.get("code", f"{record.process}_{index + 1}"),
                    step_label=step.get("label", record.process),
                    robot_id=record.robot_id,
                    target_point=step.get("point", record.target_area),
                    start_time=round(cursor, 3),
                    end_time=round(end, 3),
                    duration=round(max(end - cursor, 0.0), 3),
                    level=self.process_to_level.get(record.process, 0),
                ))
                cursor = end
        return rows

    def line_balance_summary(self, rows: Iterable[WorkStepRecord]) -> dict:
        station_times: Dict[str, float] = {}
        for row in rows:
            station_times[row.robot_id] = station_times.get(row.robot_id, 0.0) + row.duration
        if not station_times:
            return {
                "station_times": {},
                "total_work_time": 0.0,
                "cycle_time": 0.0,
                "balance_rate": 0.0,
                "balance_loss_rate": 0.0,
                "bottleneck_resource": "",
            }
        total = sum(station_times.values())
        cycle = max(station_times.values())
        station_count = len(station_times)
        balance_rate = total / (cycle * station_count) if cycle else 0.0
        bottleneck = max(station_times.items(), key=lambda item: item[1])[0]
        return {
            "station_times": {k: round(v, 3) for k, v in sorted(station_times.items())},
            "total_work_time": round(total, 3),
            "cycle_time": round(cycle, 3),
            "balance_rate": round(balance_rate, 4),
            "balance_loss_rate": round(1.0 - balance_rate, 4),
            "bottleneck_resource": bottleneck,
        }

    def balance_recommendations(self, balance: dict) -> List[str]:
        bottleneck = balance.get("bottleneck_resource", "")
        station_times = balance.get("station_times", {})
        if not bottleneck or bottleneck not in station_times:
            return []

        recommendations = [
            f"当前瓶颈资源是 {bottleneck}，应优先检查它承担的工序是否可以提前准备、拆分或错峰。",
        ]
        if bottleneck == "R3":
            recommendations.append(
                "R3 同时负责控制模块安装和产品转移，普通订单可先让 R1/R2 准备箱体、PCB、端子排，减少 R3 前后的空等。"
            )
        if "inspection_platform_area" in self.config.get("balance_focus_areas", ["inspection_platform_area"]):
            recommendations.append(
                "检测平台是 CAMERA、R4、R5 的共享区域，调度时应减少检测、锁付、分拣之间的重叠等待。"
            )
        return recommendations

    def _single_step(self, record: ScheduleRecord) -> WorkStepRecord:
        return WorkStepRecord(
            order_id=record.order_id,
            task_id=record.task_id,
            process=record.process,
            step_code=record.process,
            step_label=record.process,
            robot_id=record.robot_id,
            target_point=record.target_area,
            start_time=record.start_time,
            end_time=record.end_time,
            duration=max(record.end_time - record.start_time, 0.0),
            level=self.process_to_level.get(record.process, 0),
        )

    def _node_order_key(self, node_id: str) -> tuple[int, str]:
        node = self.nodes[node_id]
        return (node.level, node.node_id)
