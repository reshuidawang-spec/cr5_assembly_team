#!/usr/bin/env python3
"""Run production-time fault simulations for the five-CR5A scheduler."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scheduler.experiment import DiscreteEventExperiment, ExperimentResult  # noqa: E402
from scheduler.order_parser import OrderParser  # noqa: E402


FAULT_META = {
    "no_fault_proposed": ("无故障", "正常生产"),
    "fault_r1_key_window": ("R1", "箱体上料/端子排安装窗口故障"),
    "fault_r2_key_window": ("R2", "PCB安装窗口故障"),
    "fault_r3_key_window": ("R3", "模块安装/转检测区窗口故障"),
    "fault_r4_key_window": ("R4", "螺钉锁付窗口故障"),
    "fault_r5_key_window": ("R5", "分拣窗口故障"),
    "fault_camera_key_window": ("CAMERA", "相机检测窗口故障"),
    "fault_r4_early": ("R4", "早期故障"),
    "fault_r4_middle": ("R4", "中期关键锁付窗口故障"),
    "fault_r4_late": ("R4", "后期故障"),
}


def load_demo_orders():
    parser = OrderParser()
    orders = parser.parse_file(str(ROOT / "data" / "orders" / "demo_orders.json"))
    for order in orders:
        if order.priority >= 5 and order.arrival_time <= 0:
            order.arrival_time = 20.0
    return orders


def result_row(result: ExperimentResult) -> dict:
    fault_resource, description = FAULT_META.get(result.mode, ("未知", result.mode))
    return {
        "mode": result.mode,
        "fault_resource": fault_resource,
        "description": description,
        "makespan": round(result.makespan, 2),
        "urgent_response_time": round(result.urgent_response_time, 2),
        "urgent_completion_time": round(result.urgent_completion_time, 2),
        "average_waiting_time": round(result.average_waiting_time, 2),
        "weighted_tardiness": round(result.weighted_tardiness, 2),
        "parallel_efficiency": round(result.parallel_efficiency, 4),
        "conflict_count": result.conflict_count,
        "throughput": round(result.throughput, 4),
    }


def write_summary_csv(path: Path, results: list[ExperimentResult]) -> None:
    rows = [result_row(result) for result in results]
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def md_table(results: list[ExperimentResult]) -> str:
    rows = [
        "| 场景 | 故障资源 | 总完工时间 | 急单响应 | 急单完成 | 加权延期 | 冲突次数 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        row = result_row(result)
        rows.append(
            "| {description} | {fault_resource} | {makespan:.0f} | "
            "{urgent_response_time:.0f} | {urgent_completion_time:.0f} | "
            "{weighted_tardiness:.0f} | {conflict_count} |".format(**row)
        )
    return "\n".join(rows)


def write_report(path: Path, by_robot: list[ExperimentResult], by_time: list[ExperimentResult]) -> None:
    normal = by_robot[0]
    fault_only = by_robot[1:]
    worst_makespan = max(fault_only, key=lambda item: item.makespan)
    worst_urgent = max(fault_only, key=lambda item: item.urgent_completion_time)
    worst_tardiness = max(fault_only, key=lambda item: item.weighted_tardiness)

    lines = [
        "# 五臂生产单元故障场景模拟结果",
        "",
        "## 测试目的",
        "",
        "这组实验模拟生产过程中某个机械臂或相机突然故障、维修 18 秒后恢复的情况。"
        "目的是观察调度方案在异常情况下是否还能继续完成订单，以及哪个资源故障对急单和交期影响最大。",
        "",
        "## 一、不同资源故障对比",
        "",
        md_table(by_robot),
        "",
        "### 结论",
        "",
        f"- 正常无故障时，总完工时间为 {normal.makespan:.0f}，急单完成时间为 {normal.urgent_completion_time:.0f}。",
        f"- 对总完工时间影响最大的资源故障是 {FAULT_META[worst_makespan.mode][0]}，场景为“{FAULT_META[worst_makespan.mode][1]}”，总完工时间变为 {worst_makespan.makespan:.0f}。",
        f"- 对急单完成影响最大的资源故障是 {FAULT_META[worst_urgent.mode][0]}，急单完成时间变为 {worst_urgent.urgent_completion_time:.0f}。",
        f"- 对交期惩罚影响最大的资源故障是 {FAULT_META[worst_tardiness.mode][0]}，加权延期变为 {worst_tardiness.weighted_tardiness:.0f}。",
        "",
        "## 二、R4 不同时间故障对比",
        "",
        md_table(by_time),
        "",
        "### 结论",
        "",
        "R4 是螺钉锁付资源，位于相机检测之后、R5 分拣之前。"
        "如果 R4 在关键锁付窗口发生故障，会直接阻断后续分拣，因此对急单完成和延期惩罚的影响通常比早期空闲时故障更明显。",
        "",
        "## 可以向负责人说明的话",
        "",
        "我进一步做了故障鲁棒性实验，不只是验证正常情况下能调度，还模拟了 R1、R2、R3、R4、R5 和固定相机在生产过程中突然故障的情况。"
        "实验结果可以用来判断哪个资源是瓶颈，以及调度方案在异常情况下是否仍然能恢复生产。"
        "这能体现 4 号调度模块不仅考虑正常排产，也考虑了真实产线中机械臂故障、维修恢复和任务重排的问题。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    orders = load_demo_orders()
    experiment = DiscreteEventExperiment()

    proposed = experiment.run_proposed(orders)
    by_robot_faults = [proposed] + experiment.run_fault_matrix(orders)
    by_robot_faults[0].mode = "no_fault_proposed"

    by_time_faults = [proposed] + experiment.run_fault_timing_scenarios(orders, fault_robot="R4")
    by_time_faults[0].mode = "no_fault_proposed"

    out_root = ROOT / "data" / "fault_scenarios_v2"
    by_robot_dir = out_root / "by_robot"
    by_time_dir = out_root / "by_time"
    experiment.export_results(by_robot_faults, str(by_robot_dir))
    experiment.export_results(by_time_faults, str(by_time_dir))

    write_summary_csv(out_root / "fault_by_robot_summary.csv", by_robot_faults)
    write_summary_csv(out_root / "fault_by_time_summary.csv", by_time_faults)

    out_doc = ROOT / "output" / "documents" / "机械臂故障场景模拟结果.md"
    out_doc.parent.mkdir(parents=True, exist_ok=True)
    write_report(out_doc, by_robot_faults, by_time_faults)

    print(f"by_robot={by_robot_dir}")
    print(f"by_time={by_time_dir}")
    print(f"summary={out_root}")
    print(f"report={out_doc}")
    print("\nFault by robot:")
    print(md_table(by_robot_faults))
    print("\nR4 fault timing:")
    print(md_table(by_time_faults))


if __name__ == "__main__":
    main()
