#!/usr/bin/env python3
"""Simple-random dataset metrics repair — retroactively computes and backfills missing evaluation metrics for older dataset entries."""

import argparse
import csv
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Repair invalid MoveIt planning time values in simple random dataset result files and rebuild summaries."
    )
    parser.add_argument(
        "results",
        nargs="+",
        help="One or more *_simple_random_task_dataset_results.csv files to repair in place.",
    )
    return parser.parse_args()


def parse_float(row, key, default=0.0):
    """Parse a string to float, returning a default for empty/missing values."""
    value = row.get(key, "")
    if value in ("", None):
        return default
    return float(value)


def compute_quantile(values, q):
    """Compute quantile."""
    if not values:
        return 0.0
    ordered = sorted(values)
    clamped = min(1.0, max(0.0, q))
    pos = clamped * (len(ordered) - 1)
    lower = int(math.floor(pos))
    upper = int(math.ceil(pos))
    if lower == upper:
        return ordered[lower]
    weight = pos - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def sane_upper_bound(wall_ms, budget_ms):
    """Sane upper bound."""
    return max(wall_ms + 100.0, budget_ms + 100.0, 100.0)


def needs_repair(reported_ms, wall_ms, budget_ms):
    """Needs repair."""
    if not math.isfinite(reported_ms) or reported_ms < 0.0:
        return True
    return reported_ms > sane_upper_bound(wall_ms, budget_ms)


def repair_results_file(path: Path):
    """Repair results file."""
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames
        rows = list(reader)

    repaired = 0
    for row in rows:
        if not row.get("规划器"):
            continue
        wall_ms = parse_float(row, "墙钟时间(ms)")
        budget_ms = parse_float(row, "预算上限(ms)")
        reported_ms = parse_float(row, "MoveIt规划时间(ms)")
        if needs_repair(reported_ms, wall_ms, budget_ms):
            row["MoveIt规划时间(ms)"] = f"{wall_ms:.1f}"
            repaired += 1

    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    return repaired, rows


def summary_path_from_results(results_path: Path):
    """Summary path from results."""
    name = results_path.name.replace("_results.csv", "_summary.csv")
    return results_path.with_name(name)


def rebuild_summary(results_path: Path, rows):
    """Rebuild summary."""
    summary_path = summary_path_from_results(results_path)
    fieldnames = [
        "数据集版本",
        "实验时间戳",
        "随机种子",
        "总任务数",
        "规划器",
        "模式",
        "规划器ID",
        "成功率(%)",
        "成功样本数",
        "总样本数",
        "平均时间(ms)",
        "中位时间(ms)",
        "P25(ms)",
        "P75(ms)",
        "预算命中数",
        "预算命中率(%)",
        "快速求解数",
        "快速求解率(%)",
        "平均MoveIt规划时间(ms)",
        "平均规划调用次数",
        "详细结果文件",
    ]

    by_planner = {}
    planner_order = []
    for row in rows:
        planner = row.get("规划器", "")
        if not planner:
            continue
        if planner not in by_planner:
            by_planner[planner] = []
            planner_order.append(planner)
        by_planner[planner].append(row)

    with summary_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()

        for planner in planner_order:
            subset = by_planner[planner]
            first = subset[0]
            wall_times = [parse_float(row, "墙钟时间(ms)") for row in subset]
            moveit_times = [parse_float(row, "MoveIt规划时间(ms)") for row in subset]
            planner_calls = [parse_float(row, "规划调用次数", 1.0) for row in subset]
            success_count = sum(1 for row in subset if row.get("成功", "") == "成功")
            budget_hit_count = sum(1 for row in subset if row.get("触发预算上限", "") == "是")
            fast_count = sum(1 for row in subset if row.get("快速求解(<1s)", "") == "是")
            count = len(subset)

            writer.writerow(
                {
                    "数据集版本": first.get("数据集版本", ""),
                    "实验时间戳": first.get("实验时间戳", ""),
                    "随机种子": first.get("随机种子", ""),
                    "总任务数": first.get("总任务数", ""),
                    "规划器": planner,
                    "模式": first.get("模式", ""),
                    "规划器ID": first.get("规划器ID", ""),
                    "成功率(%)": f"{(100.0 * success_count / count) if count else 0.0:.1f}",
                    "成功样本数": success_count,
                    "总样本数": count,
                    "平均时间(ms)": f"{(sum(wall_times) / count) if count else 0.0:.1f}",
                    "中位时间(ms)": f"{compute_quantile(wall_times, 0.5):.1f}",
                    "P25(ms)": f"{compute_quantile(wall_times, 0.25):.1f}",
                    "P75(ms)": f"{compute_quantile(wall_times, 0.75):.1f}",
                    "预算命中数": budget_hit_count,
                    "预算命中率(%)": f"{(100.0 * budget_hit_count / count) if count else 0.0:.1f}",
                    "快速求解数": fast_count,
                    "快速求解率(%)": f"{(100.0 * fast_count / count) if count else 0.0:.1f}",
                    "平均MoveIt规划时间(ms)": f"{(sum(moveit_times) / count) if count else 0.0:.1f}",
                    "平均规划调用次数": f"{(sum(planner_calls) / count) if count else 0.0:.1f}",
                    "详细结果文件": str(results_path.resolve()),
                }
            )

    return summary_path


def main():
    """Main."""
    args = parse_args()
    for raw_path in args.results:
        results_path = Path(raw_path)
        repaired, rows = repair_results_file(results_path)
        summary_path = rebuild_summary(results_path, rows)
        print(f"{results_path}: repaired {repaired} rows")
        print(f"{summary_path}: rebuilt")


if __name__ == "__main__":
    main()
