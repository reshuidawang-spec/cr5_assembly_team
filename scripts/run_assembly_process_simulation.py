#!/usr/bin/env python3
"""Export component sequence, step-level timeline, and line-balance analysis."""

from __future__ import annotations

import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scheduler.assembly_process import AssemblyProcessPlanner, WorkStepRecord  # noqa: E402
from scheduler.experiment import DiscreteEventExperiment  # noqa: E402
from scheduler.order_parser import OrderParser  # noqa: E402


PROCESS_LABELS = {
    "box_feed": "箱体上料",
    "pcb_install": "PCB安装",
    "module_install": "控制模块安装",
    "terminal_install": "端子排安装",
    "transfer_to_inspection": "转移到检测区",
    "inspect": "相机检测",
    "screw": "螺钉锁付",
    "sort_good": "合格品分拣",
    "sort_defect": "缺陷品分拣",
}


def load_demo_orders():
    parser = OrderParser()
    orders = parser.parse_file(str(ROOT / "data" / "orders" / "demo_orders.json"))
    for order in orders:
        if order.priority >= 5 and order.arrival_time <= 0:
            order.arrival_time = 20.0
    return orders


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8-sig")
        return
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def build_html(
    rows: list[WorkStepRecord],
    balance: dict,
    sequence_rows: list[dict],
    recommendations: list[str],
) -> str:
    max_time = max((row.end_time for row in rows), default=1.0)
    lanes = []
    for robot in ["R1", "R2", "R3", "CAMERA", "R4", "R5"]:
        if any(row.robot_id == robot for row in rows):
            lanes.append(robot)

    row_height = 34
    lane_height = max(row_height * 2, 86)
    left = 260
    width = 1320
    plot_width = 980
    height = 130 + lane_height * len(lanes)

    lane_y = {robot: 86 + idx * lane_height for idx, robot in enumerate(lanes)}
    colors = {
        "box_feed": "#4C78A8",
        "pcb_install": "#54A24B",
        "module_install": "#7B5FB2",
        "terminal_install": "#B89B29",
        "transfer_to_inspection": "#2F6FDB",
        "inspect": "#B279A2",
        "screw": "#F58518",
        "sort_good": "#72B7B2",
        "sort_defect": "#E45756",
    }

    bars = []
    lane_offsets = {robot: 0 for robot in lanes}
    for row in rows:
        y = lane_y[row.robot_id] + (lane_offsets[row.robot_id] % 2) * row_height
        lane_offsets[row.robot_id] += 1
        x = left + row.start_time / max_time * plot_width
        w = max((row.end_time - row.start_time) / max_time * plot_width, 3)
        label = f"{row.order_id} · {row.step_label}"
        bars.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="24" rx="4" '
            f'fill="{colors.get(row.process, "#888")}" opacity="0.88">'
            f'<title>{label}: {row.start_time:.1f}-{row.end_time:.1f}s</title></rect>'
        )
        if w >= 86:
            bars.append(
                f'<text x="{x + 5:.1f}" y="{y + 17:.1f}" class="bar-label">{row.order_id}</text>'
            )

    grid = []
    for tick in range(0, int(max_time) + 20, 20):
        x = left + tick / max_time * plot_width
        grid.append(f'<line x1="{x:.1f}" y1="70" x2="{x:.1f}" y2="{height - 45}" class="grid"/>')
        grid.append(f'<text x="{x:.1f}" y="{height - 18}" class="tick">{tick}</text>')

    lane_marks = []
    for robot in lanes:
        y = lane_y[robot] - 8
        lane_marks.append(f'<text x="28" y="{y + 34}" class="lane">{robot}</text>')
        lane_marks.append(f'<line x1="{left}" y1="{y + 44}" x2="{left + plot_width}" y2="{y + 44}" class="lane-line"/>')

    seq_items = "".join(
        f'<li><span>第 {item["topology_level"]} 层</span>{item["name"]} <em>{item["resource"]}</em></li>'
        for item in sequence_rows
    )
    station_items = "".join(
        f'<li>{robot}: {time_value:.1f}s</li>'
        for robot, time_value in balance["station_times"].items()
    )
    recommendation_items = "".join(
        f"<li>{item}</li>"
        for item in recommendations
    )

    return f"""<!doctype html>
<meta charset="utf-8">
<title>五臂工步级时间仿真</title>
<style>
  body {{ font-family: "Microsoft YaHei", Arial, sans-serif; margin: 24px; color: #20242a; }}
  h1 {{ font-size: 22px; margin: 0 0 10px; }}
  h2 {{ font-size: 16px; margin: 18px 0 8px; }}
  .layout {{ display: grid; grid-template-columns: 1fr 310px; gap: 18px; align-items: start; }}
  .panel {{ border: 1px solid #d7dde5; border-radius: 8px; padding: 14px; background: #fff; }}
  svg {{ width: 100%; height: auto; border: 1px solid #d7dde5; border-radius: 8px; background: #fff; }}
  .axis, .lane-line {{ stroke: #aab3c0; stroke-width: 1.2; }}
  .grid {{ stroke: #edf0f5; stroke-width: 1; }}
  .lane {{ font-size: 15px; font-weight: 600; fill: #30343a; }}
  .tick {{ font-size: 12px; fill: #606873; text-anchor: middle; }}
  .bar-label {{ font-size: 12px; fill: #fff; font-weight: 600; pointer-events: none; }}
  ul {{ margin: 0; padding-left: 18px; }}
  li {{ margin: 6px 0; line-height: 1.35; }}
  li span {{ display: inline-block; color: #5d6673; width: 54px; }}
  li em {{ color: #5d6673; font-style: normal; margin-left: 6px; }}
  .metric {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }}
  .metric div {{ background: #f5f7fa; border-radius: 6px; padding: 8px; }}
  .metric b {{ display: block; font-size: 18px; }}
</style>
<h1>五臂工步级时间仿真</h1>
<div class="layout">
  <div>
    <svg viewBox="0 0 {width} {height}" role="img" aria-label="工步级时间轴">
      <text x="{left}" y="36" class="lane">时间轴 / 秒</text>
      <line x1="{left}" y1="{height - 45}" x2="{left + plot_width}" y2="{height - 45}" class="axis"/>
      {''.join(grid)}
      {''.join(lane_marks)}
      {''.join(bars)}
    </svg>
  </div>
  <aside class="panel">
    <h2>安装顺序</h2>
    <ul>{seq_items}</ul>
    <h2>线平衡指标</h2>
    <div class="metric">
      <div><span>瓶颈资源</span><b>{balance["bottleneck_resource"]}</b></div>
      <div><span>平衡率</span><b>{balance["balance_rate"] * 100:.1f}%</b></div>
      <div><span>节拍</span><b>{balance["cycle_time"]:.1f}s</b></div>
      <div><span>总工时</span><b>{balance["total_work_time"]:.1f}s</b></div>
    </div>
    <h2>各资源负载</h2>
    <ul>{station_items}</ul>
    <h2>优化建议</h2>
    <ul>{recommendation_items}</ul>
  </aside>
</div>
"""


def main() -> None:
    orders = load_demo_orders()
    experiment = DiscreteEventExperiment()
    proposed = experiment.run_proposed(orders)

    planner = AssemblyProcessPlanner()
    sequence_rows = planner.component_sequence_rows()
    workstep_rows = planner.expand_schedule_to_worksteps(proposed.records)
    balance = planner.line_balance_summary(workstep_rows)
    recommendations = planner.balance_recommendations(balance)

    out_dir = ROOT / "data" / "assembly_process_v2"
    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "component_sequence.csv", sequence_rows)
    write_csv(out_dir / "step_timeline.csv", [asdict(row) for row in workstep_rows])
    (out_dir / "line_balance_summary.json").write_text(
        json.dumps(balance, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (out_dir / "line_balance_recommendations.json").write_text(
        json.dumps({"recommendations": recommendations}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    vis_dir = ROOT / "output" / "visualizations"
    vis_dir.mkdir(parents=True, exist_ok=True)
    html_path = vis_dir / "assembly_step_timeline.html"
    html_path.write_text(
        build_html(workstep_rows, balance, sequence_rows, recommendations),
        encoding="utf-8",
    )

    print(f"component_sequence={out_dir / 'component_sequence.csv'}")
    print(f"step_timeline={out_dir / 'step_timeline.csv'}")
    print(f"line_balance={out_dir / 'line_balance_summary.json'}")
    print(f"recommendations={out_dir / 'line_balance_recommendations.json'}")
    print(f"visualization={html_path}")
    print(
        "balance_rate={:.1f}% bottleneck={}".format(
            balance["balance_rate"] * 100,
            balance["bottleneck_resource"],
        )
    )


if __name__ == "__main__":
    main()
