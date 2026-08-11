"""Runtime KPI and embedded scheduling-analysis dashboard.

This module is the integration boundary between the live CoppeliaSim cycle and
the fourth member's discrete-event scheduling experiment.  Calculations are
kept independent from Tk so they can also be exported and unit-tested.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import asdict
from pathlib import Path
from typing import Callable, Iterable, Mapping, Optional

from interfaces.types import Order, Task, TaskResult, TaskStatus
from scheduler.experiment import DiscreteEventExperiment, ExperimentResult


def compute_runtime_kpi(
    tasks: Iterable[Task],
    results: Mapping[str, TaskResult] | Iterable[TaskResult],
    conflict_count: int = 0,
    robot_ids: Optional[Iterable[str]] = None,
) -> dict:
    """Calculate actual-cycle KPI from executor timestamps.

    Waiting time is the gap between a task's actual start and the latest
    completed predecessor.  Root tasks use the first recorded task start as
    the cycle origin.
    """

    task_list = list(tasks)
    if isinstance(results, Mapping):
        result_map = dict(results)
    else:
        result_map = {result.task_id: result for result in results}
    result_list = list(result_map.values())

    if result_list:
        cycle_start = min(result.start_time for result in result_list)
        cycle_end = max(result.end_time for result in result_list)
        makespan = max(cycle_end - cycle_start, 0.0)
    else:
        cycle_start = 0.0
        makespan = 0.0

    busy_time: dict[str, float] = defaultdict(float)
    for result in result_list:
        busy_time[result.robot_id] += max(result.end_time - result.start_time, 0.0)

    known_robots = set(robot_ids or [])
    known_robots.update(robot_id for robot_id in busy_time if robot_id)
    utilization = {
        robot_id: (busy_time.get(robot_id, 0.0) / makespan if makespan else 0.0)
        for robot_id in sorted(known_robots)
    }
    average_utilization = (
        sum(utilization.values()) / len(utilization) if utilization else 0.0
    )

    waits: list[float] = []
    for task in task_list:
        result = result_map.get(task.task_id)
        if result is None:
            continue
        predecessor_ends = [
            result_map[pred].end_time
            for pred in task.predecessors
            if pred in result_map
        ]
        ready_time = max(predecessor_ends) if predecessor_ends else cycle_start
        waits.append(max(result.start_time - ready_time, 0.0))

    completed = sum(
        result.status == TaskStatus.FINISHED.value for result in result_list
    )
    failed = sum(result.status == TaskStatus.FAILED.value for result in result_list)
    return {
        "makespan": makespan,
        "utilization": utilization,
        "average_utilization": average_utilization,
        "avg_waiting_time": sum(waits) / len(waits) if waits else 0.0,
        "conflict_count": int(conflict_count),
        "completed": completed,
        "failed": failed,
        "total": len(task_list),
    }


def compute_kpi(
    tasks: Iterable[Task],
    robots: Optional[Iterable[object]] = None,
    results: Optional[Mapping[str, TaskResult] | Iterable[TaskResult]] = None,
    conflict_count: int = 0,
) -> dict:
    """Backward-compatible KPI entry point used by older integrations."""

    robot_list = list(robots or [])
    robot_ids = [getattr(robot, "robot_id", str(robot)) for robot in robot_list]
    if results is None:
        utilization = {
            robot_id: float(getattr(robot, "utilization", 0.0))
            for robot_id, robot in zip(robot_ids, robot_list)
        }
        task_list = list(tasks)
        return {
            "makespan": 0.0,
            "utilization": utilization,
            "average_utilization": (
                sum(utilization.values()) / len(utilization) if utilization else 0.0
            ),
            "avg_waiting_time": 0.0,
            "conflict_count": int(conflict_count),
            "completed": sum(
                task.status == TaskStatus.FINISHED.value for task in task_list
            ),
            "failed": sum(task.status == TaskStatus.FAILED.value for task in task_list),
            "total": len(task_list),
        }
    return compute_runtime_kpi(tasks, results, conflict_count, robot_ids)


def compare_results(
    baseline: ExperimentResult,
    proposed: ExperimentResult,
) -> dict:
    """Return serializable comparison data and improvement percentages."""

    lower_is_better = {
        "makespan": "总完成时间",
        "average_waiting_time": "平均等待",
        "weighted_tardiness": "加权延期",
        "conflict_count": "冲突次数",
    }
    improvements = {}
    for key, label in lower_is_better.items():
        old = float(getattr(baseline, key))
        new = float(getattr(proposed, key))
        improvements[key] = {
            "label": label,
            "baseline": old,
            "proposed": new,
            "improvement_percent": ((old - new) / old * 100.0) if old else 0.0,
        }
    return {
        "baseline": baseline.summary_dict(),
        "proposed": proposed.summary_dict(),
        "improvements": improvements,
    }


def generate_comparison_chart(
    baseline_data: dict,
    proposed_data: dict,
    save_path: str | None = None,
):
    """Generate a compact Baseline/Proposed chart when matplotlib is present."""

    keys = ("makespan", "average_waiting_time", "weighted_tardiness", "conflict_count")
    chart_data = {
        "metrics": list(keys),
        "baseline": [float(baseline_data.get(key, 0.0)) for key in keys],
        "proposed": [float(proposed_data.get(key, 0.0)) for key in keys],
    }
    if save_path:
        import matplotlib.pyplot as plt

        positions = range(len(keys))
        fig, axis = plt.subplots(figsize=(9, 4.5))
        axis.bar([pos - 0.2 for pos in positions], chart_data["baseline"], 0.4, label="Baseline")
        axis.bar([pos + 0.2 for pos in positions], chart_data["proposed"], 0.4, label="Proposed")
        axis.set_xticks(list(positions), keys, rotation=15)
        axis.legend()
        fig.tight_layout()
        fig.savefig(Path(save_path), dpi=160)
        plt.close(fig)
    return chart_data


class SchedulingDashboard:
    """Tk page that embeds the scheduling experiment in the main software."""

    def __init__(
        self,
        parent,
        order_provider: Callable[[], Iterable[Order]],
        log_callback: Optional[Callable[[str, str], None]] = None,
        colors: Optional[dict] = None,
    ):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.parent = parent
        self.order_provider = order_provider
        self.log_callback = log_callback
        self.colors = {
            "bg": "#0d1117",
            "panel": "#161b22",
            "border": "#30363d",
            "text": "#c9d1d9",
            "dim": "#8b949e",
            "accent": "#f0c040",
            "green": "#3fb950",
            "blue": "#58a6ff",
            "button": "#21262d",
        }
        self.colors.update(colors or {})
        self.last_baseline: Optional[ExperimentResult] = None
        self.last_proposed: Optional[ExperimentResult] = None
        self._kpi_labels: dict[str, dict[str, object]] = {}
        self._build()

    def _build(self) -> None:
        tk, ttk, c = self.tk, self.ttk, self.colors
        toolbar = tk.Frame(self.parent, bg=c["panel"], height=48)
        toolbar.pack(fill=tk.X, padx=6, pady=(5, 3))
        tk.Label(
            toolbar,
            text="多订单动态调度效能分析",
            bg=c["panel"], fg=c["accent"], font=("Microsoft YaHei", 11, "bold"),
        ).pack(side=tk.LEFT, padx=12)
        tk.Button(
            toolbar, text="分析当前订单", command=self.analyze,
            bg="#238636", fg="white", relief=tk.FLAT,
            font=("Consolas", 9, "bold"), cursor="hand2",
        ).pack(side=tk.RIGHT, padx=10, pady=8, ipadx=12, ipady=3)
        self.status_label = tk.Label(
            toolbar, text="请先在‘仿真执行’页加入订单", bg=c["panel"],
            fg=c["dim"], font=("Microsoft YaHei", 9),
        )
        self.status_label.pack(side=tk.RIGHT, padx=8)

        comparison = tk.Frame(self.parent, bg=c["border"])
        comparison.pack(fill=tk.X, padx=6, pady=3)
        comparison_inner = tk.Frame(comparison, bg=c["panel"])
        comparison_inner.pack(fill=tk.X, padx=1, pady=1)

        cards = tk.Frame(comparison_inner, bg=c["panel"])
        cards.pack(fill=tk.X, padx=8, pady=(8, 4))
        card_specs = (
            ("makespan", "总完成时间", "s"),
            ("waiting", "平均等待", "s"),
            ("tardiness", "加权延期", ""),
            ("conflicts", "调度冲突", "次"),
            ("efficiency", "并行效率", "%"),
        )
        for key, title, unit in card_specs:
            card = tk.Frame(cards, bg=c["bg"], highlightbackground=c["border"], highlightthickness=1)
            card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=3)
            tk.Label(
                card, text=title, bg=c["bg"], fg=c["dim"],
                font=("Microsoft YaHei", 9, "bold"),
            ).pack(anchor=tk.W, padx=10, pady=(7, 0))
            value = tk.Label(
                card, text="--", bg=c["bg"], fg=c["green"],
                font=("Consolas", 18, "bold"),
            )
            value.pack(anchor=tk.W, padx=10)
            baseline = tk.Label(
                card, text="基准 --", bg=c["bg"], fg=c["dim"],
                font=("Microsoft YaHei", 8),
            )
            baseline.pack(anchor=tk.W, padx=10)
            improvement = tk.Label(
                card, text="等待分析", bg=c["bg"], fg=c["blue"],
                font=("Consolas", 8, "bold"),
            )
            improvement.pack(anchor=tk.W, padx=10, pady=(0, 7))
            self._kpi_labels[key] = {
                "value": value,
                "baseline": baseline,
                "improvement": improvement,
                "unit": unit,
            }

        chart_header = tk.Frame(comparison_inner, bg=c["panel"])
        chart_header.pack(fill=tk.X, padx=12, pady=(2, 0))
        tk.Label(
            chart_header, text="方案关键指标对比",
            bg=c["panel"], fg=c["accent"],
            font=("Microsoft YaHei", 9, "bold"),
        ).pack(side=tk.LEFT)
        tk.Label(
            chart_header, text="■ Baseline   ■ Proposed",
            bg=c["panel"], fg=c["dim"], font=("Consolas", 8),
        ).pack(side=tk.RIGHT)
        self.comparison_canvas = tk.Canvas(
            comparison_inner,
            height=116,
            bg=c["panel"],
            highlightthickness=0,
        )
        self.comparison_canvas.pack(fill=tk.X, padx=10, pady=(1, 7))
        self.comparison_canvas.bind(
            "<Configure>", lambda _event: self._draw_comparison_chart()
        )

        schedule = tk.Frame(self.parent, bg=c["border"])
        schedule.pack(fill=tk.BOTH, expand=True, padx=6, pady=(3, 6))
        schedule_inner = tk.Frame(schedule, bg=c["panel"])
        schedule_inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        tk.Label(
            schedule_inner, text="推荐方案任务时间线",
            bg=c["panel"], fg=c["accent"], font=("Microsoft YaHei", 10, "bold"),
        ).pack(anchor=tk.W, padx=10, pady=(8, 3))
        columns = ("task", "order", "process", "robot", "start", "end", "wait")
        self.schedule_tree = ttk.Treeview(schedule_inner, columns=columns, show="headings")
        for key, label, width in (
            ("task", "任务", 135), ("order", "订单", 90),
            ("process", "工艺", 170), ("robot", "设备", 80),
            ("start", "开始/s", 90), ("end", "结束/s", 90), ("wait", "等待/s", 90),
        ):
            self.schedule_tree.heading(key, text=label)
            self.schedule_tree.column(key, width=width, anchor=tk.CENTER)
        scrollbar = ttk.Scrollbar(schedule_inner, command=self.schedule_tree.yview)
        self.schedule_tree.configure(yscrollcommand=scrollbar.set)
        self.schedule_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0), pady=(2, 10))
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 10), pady=(2, 10))

    def analyze(self) -> bool:
        orders = list(self.order_provider())
        if not orders:
            self.status_label.configure(text="没有可分析的订单", fg=self.colors["dim"])
            return False
        try:
            experiment = DiscreteEventExperiment()
            self.last_baseline = experiment.run_baseline(orders)
            self.last_proposed = experiment.run_proposed(orders)
            self._render()
            self.status_label.configure(
                text=f"已分析 {len(orders)} 笔订单", fg=self.colors["green"]
            )
            if self.log_callback:
                self.log_callback(f"SCHEDULING ANALYSIS COMPLETE — {len(orders)} ORDERS", "ok")
            return True
        except Exception as exc:
            self.status_label.configure(text=f"分析失败: {exc}", fg="#f85149")
            if self.log_callback:
                self.log_callback(f"SCHEDULING ANALYSIS FAILED: {exc}", "error")
            return False

    def _render(self) -> None:
        assert self.last_baseline is not None and self.last_proposed is not None
        self._render_kpi_cards()
        self._draw_comparison_chart()
        for item in self.schedule_tree.get_children():
            self.schedule_tree.delete(item)
        for record in sorted(
            self.last_proposed.records,
            key=lambda item: (item.start_time, item.end_time, item.task_id),
        ):
            self.schedule_tree.insert("", self.tk.END, values=(
                record.task_id, record.order_id, record.process, record.robot_id,
                f"{record.start_time:.1f}", f"{record.end_time:.1f}", f"{record.wait_time:.1f}",
            ))

    @staticmethod
    def _change_text(old: float, new: float, higher_is_better: bool = False) -> tuple[str, bool]:
        if abs(old) < 1e-12:
            return ("持平" if abs(new) < 1e-12 else "新增数据", abs(new) < 1e-12)
        change = ((new - old) / old * 100.0) if higher_is_better else ((old - new) / old * 100.0)
        if abs(change) < 0.05:
            return "持平", True
        return f"{'改善' if change > 0 else '下降'} {abs(change):.1f}%", change >= 0

    def _render_kpi_cards(self) -> None:
        assert self.last_baseline is not None and self.last_proposed is not None
        baseline, proposed = self.last_baseline, self.last_proposed
        values = {
            "makespan": (baseline.makespan, proposed.makespan, False),
            "waiting": (baseline.average_waiting_time, proposed.average_waiting_time, False),
            "tardiness": (baseline.weighted_tardiness, proposed.weighted_tardiness, False),
            "conflicts": (float(baseline.conflict_count), float(proposed.conflict_count), False),
            "efficiency": (
                baseline.parallel_efficiency * 100.0,
                proposed.parallel_efficiency * 100.0,
                True,
            ),
        }
        for key, (old, new, higher_is_better) in values.items():
            widgets = self._kpi_labels[key]
            unit = str(widgets["unit"])
            decimals = 0 if key == "conflicts" else 1
            value_text = f"{new:.{decimals}f}{unit}"
            baseline_text = f"基准 {old:.{decimals}f}{unit}"
            change_text, positive = self._change_text(old, new, higher_is_better)
            widgets["value"].configure(text=value_text)
            widgets["baseline"].configure(text=baseline_text)
            widgets["improvement"].configure(
                text=change_text,
                fg=self.colors["green"] if positive else "#f85149",
            )

    def _draw_comparison_chart(self) -> None:
        canvas = getattr(self, "comparison_canvas", None)
        if canvas is None:
            return
        canvas.delete("all")
        if self.last_baseline is None or self.last_proposed is None:
            canvas.create_text(
                12, 48, anchor="w", text="分析订单后显示方案对比",
                fill=self.colors["dim"], font=("Microsoft YaHei", 9),
            )
            return
        width = max(canvas.winfo_width(), 760)
        label_width = 95
        value_width = 95
        bar_left = label_width + 48
        bar_right = width - value_width
        available = max(bar_right - bar_left, 120)
        rows = (
            ("总完成时间", self.last_baseline.makespan, self.last_proposed.makespan, "s"),
            ("平均等待", self.last_baseline.average_waiting_time, self.last_proposed.average_waiting_time, "s"),
            (
                "并行效率",
                self.last_baseline.parallel_efficiency * 100.0,
                self.last_proposed.parallel_efficiency * 100.0,
                "%",
            ),
        )
        for row, (label, old, new, unit) in enumerate(rows):
            y = 13 + row * 35
            scale = max(old, new, 1e-9)
            old_width = max(2, available * old / scale)
            new_width = max(2, available * new / scale)
            canvas.create_text(
                6, y + 10, anchor="w", text=label,
                fill=self.colors["text"], font=("Microsoft YaHei", 8, "bold"),
            )
            canvas.create_rectangle(
                bar_left, y, bar_left + old_width, y + 8,
                fill="#6e7681", outline="",
            )
            canvas.create_rectangle(
                bar_left, y + 12, bar_left + new_width, y + 20,
                fill=self.colors["blue"], outline="",
            )
            canvas.create_text(
                width - 6, y + 4, anchor="e", text=f"{old:.1f}{unit}",
                fill="#8b949e", font=("Consolas", 8),
            )
            canvas.create_text(
                width - 6, y + 16, anchor="e", text=f"{new:.1f}{unit}",
                fill=self.colors["blue"], font=("Consolas", 8, "bold"),
            )

    def mark_stale(self) -> None:
        if self.last_proposed is not None:
            self.status_label.configure(text="订单已变化，请重新分析", fg=self.colors["accent"])

    def reset(self) -> None:
        self.last_baseline = None
        self.last_proposed = None
        for item in self.schedule_tree.get_children():
            self.schedule_tree.delete(item)
        for widgets in self._kpi_labels.values():
            widgets["value"].configure(text="--")
            widgets["baseline"].configure(text="基准 --")
            widgets["improvement"].configure(text="等待分析", fg=self.colors["blue"])
        self._draw_comparison_chart()
        self.status_label.configure(text="请先在‘仿真执行’页加入订单", fg=self.colors["dim"])

    def export_data(self) -> Optional[dict]:
        if self.last_baseline is None or self.last_proposed is None:
            return None
        return {
            **compare_results(self.last_baseline, self.last_proposed),
            "proposed_records": [asdict(record) for record in self.last_proposed.records],
        }


__all__ = [
    "SchedulingDashboard",
    "compare_results",
    "compute_kpi",
    "compute_runtime_kpi",
    "generate_comparison_chart",
]
