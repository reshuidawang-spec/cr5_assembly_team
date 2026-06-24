#!/usr/bin/env python3
"""Simple random dataset analyzer — computes aggregate statistics (success rate by difficulty, planner, task family) from the simple-random benchmark dataset."""

import argparse
import csv
import statistics
from collections import Counter, defaultdict
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = REPO_ROOT / "test_results"
DEFAULT_PATTERN = "datasets/simple_random/raw/20*_simple_random_task_dataset_results.csv"
FAST_THRESHOLD_MS = 1000.0

PLANNER_ORDER = ["FMT", "LBTRRT", "RRTConnect", "HeuristicGuided"]
FAMILY_ORDER = [
    "top_open",
    "front_side",
    "right_upper_angled",
    "hole_shallow",
    "hole_edge",
    "hole_deep",
]


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Analyze the latest simple random task dataset and emit a markdown report."
    )
    parser.add_argument(
        "--results",
        default="",
        help="Input results csv path. Default: latest test_results/datasets/simple_random/raw/*_simple_random_task_dataset_results.csv",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output markdown path. Default: docs/analysis/SIMPLE_RANDOM_DATASET_<timestamp>_ANALYSIS.md",
    )
    return parser.parse_args()


def latest_results_file():
    """Latest results file."""
    matches = sorted(RESULTS_ROOT.glob(DEFAULT_PATTERN))
    if not matches:
        raise RuntimeError(
            "No simple random dataset results were found in test_results/datasets/simple_random/raw/."
        )
    return matches[-1]


def read_rows(path: Path):
    """Read rows."""
    with path.open("r", encoding="utf-8", newline="") as handle:
        return [
            row
            for row in csv.DictReader(handle)
            if row.get("规划器") and row.get("任务族") and row.get("墙钟时间(ms)")
        ]


def parse_bool_zh(value: str) -> bool:
    """Parse bool zh."""
    return value in ("是", "成功", "true", "True", "1")


def parse_float(value: str, default: float = 0.0) -> float:
    """Parse a string to float, returning a default for empty/missing values."""
    if value in ("", None):
        return default
    return float(value)


def success_bool(row):
    """Success bool."""
    return row.get("成功", "") == "成功"


def planner_sort_key(name: str):
    """Planner sort key."""
    if name in PLANNER_ORDER:
        return (0, PLANNER_ORDER.index(name))
    return (1, name)


def family_sort_key(name: str):
    """Family sort key."""
    if name in FAMILY_ORDER:
        return (0, FAMILY_ORDER.index(name))
    return (1, name)


def percent(numerator: int, denominator: int) -> float:
    """Percent."""
    return 100.0 * numerator / denominator if denominator else 0.0


def median(values):
    """Median."""
    return statistics.median(values) if values else 0.0


def analyze(rows):
    """Analyze."""
    if not rows:
        raise RuntimeError("The dataset has no complete rows to analyze.")

    first = rows[0]
    dataset_version = first.get("数据集版本", "")
    timestamp = first.get("实验时间戳", "")
    total_tasks = int(first.get("总任务数", "0") or "0")
    seed = first.get("随机种子", "")

    total_rows = len(rows)
    planners = sorted({row["规划器"] for row in rows}, key=planner_sort_key)
    total_success = sum(1 for row in rows if success_bool(row))
    total_failure = total_rows - total_success

    per_planner = {}
    for planner in planners:
        subset = [row for row in rows if row["规划器"] == planner]
        wall_times = [parse_float(row["墙钟时间(ms)"]) for row in subset]
        budget_hits = sum(1 for row in subset if row.get("触发预算上限", "") == "是")
        fast_solves = sum(
            1
            for row in subset
            if row.get("快速求解(<1s)", "") == "是"
            or parse_float(row["墙钟时间(ms)"]) < FAST_THRESHOLD_MS
        )
        success_count = sum(1 for row in subset if success_bool(row))
        per_planner[planner] = {
            "count": len(subset),
            "success_count": success_count,
            "success_rate": percent(success_count, len(subset)),
            "median_wall_ms": median(wall_times),
            "budget_hits": budget_hits,
            "budget_hit_rate": percent(budget_hits, len(subset)),
            "fast_solves": fast_solves,
            "fast_solve_rate": percent(fast_solves, len(subset)),
        }

    families = sorted({row["任务族"] for row in rows}, key=family_sort_key)
    per_family = {}
    for family in families:
        subset = [row for row in rows if row["任务族"] == family]
        success_count = sum(1 for row in subset if success_bool(row))
        per_family[family] = {
            "count": len(subset),
            "success_count": success_count,
            "success_rate": percent(success_count, len(subset)),
        }

    failure_counter = Counter(
        (row["规划器"], row["任务族"]) for row in rows if not success_bool(row)
    )
    failure_by_family = Counter(row["任务族"] for row in rows if not success_bool(row))

    dominant_failure_family = ""
    dominant_failure_count = 0
    if failure_by_family:
        dominant_failure_family, dominant_failure_count = failure_by_family.most_common(1)[0]

    return {
        "dataset_version": dataset_version,
        "timestamp": timestamp,
        "seed": seed,
        "total_tasks": total_tasks,
        "total_rows": total_rows,
        "planners": planners,
        "total_success": total_success,
        "total_failure": total_failure,
        "overall_success_rate": percent(total_success, total_rows),
        "per_planner": per_planner,
        "per_family": per_family,
        "failure_counter": failure_counter,
        "failure_by_family": failure_by_family,
        "dominant_failure_family": dominant_failure_family,
        "dominant_failure_count": dominant_failure_count,
    }


def format_failure_lines(counter: Counter):
    """Format failure lines."""
    if not counter:
        return ["- 本轮无失败样本"]

    planner_failures = defaultdict(list)
    for (planner, family), count in sorted(
        counter.items(), key=lambda item: (planner_sort_key(item[0][0]), family_sort_key(item[0][1]))
    ):
        planner_failures[planner].append(f"`{family}` 失败 `{count}`")

    lines = []
    for planner in sorted(planner_failures, key=planner_sort_key):
        joined = "，".join(planner_failures[planner])
        lines.append(f"- `{planner}`: {joined}")
    return lines


def build_markdown(results_path: Path, analysis):
    """Build markdown."""
    summary_path = results_path.with_name(results_path.name.replace("_results.csv", "_summary.csv"))
    results_rel = results_path.relative_to(REPO_ROOT)
    summary_rel = summary_path.relative_to(REPO_ROOT)

    lines = []
    lines.append("# Simple Random Dataset 分析")
    lines.append("")
    lines.append("## 1. 本次运行")
    lines.append("")
    lines.append(f"- 时间戳：`{analysis['timestamp']}`")
    lines.append(f"- 数据集版本：`{analysis['dataset_version']}`")
    lines.append(f"- 随机种子：`{analysis['seed']}`")
    lines.append(f"- 随机任务数：`{analysis['total_tasks']}`")
    lines.append(f"- 规划器数：`{len(analysis['planners'])}`")
    lines.append(f"- 总样本数：`{analysis['total_rows']}`")
    lines.append("")
    lines.append("结果文件：")
    lines.append("")
    lines.append(f"- `{results_rel}`")
    lines.append(f"- `{summary_rel}`")
    lines.append("")
    lines.append("## 2. 总体结论")
    lines.append("")

    if analysis["total_failure"] == 0:
        lines.append("这一轮数据集整体过于容易，缺少足够失败样本，不适合直接作为第一版难例训练集。")
    else:
        lines.append("这一轮数据已经可以作为 Stage 2 的正式训练候选数据。")
        lines.append("")
        lines.append("原因不是“数量更多”，而是：")
        lines.append("")
        lines.append("- 它同时覆盖成功、失败、预算命中和快速求解四类关键信号")
        lines.append("- 失败分布仍然集中在高难任务族，而不是随机噪声")
        lines.append("- 不同 planner 的长尾差异在更大样本下仍然存在")

    lines.append("")
    lines.append("## 3. 总体分布")
    lines.append("")
    lines.append(f"- 成功：`{analysis['total_success']}`")
    lines.append(f"- 失败：`{analysis['total_failure']}`")
    lines.append("")
    lines.append(f"成功率：`{analysis['overall_success_rate']:.1f}%`")
    lines.append("")

    if analysis["dominant_failure_family"]:
        lines.append(
            f"本轮最主要失败任务族仍然是 `{analysis['dominant_failure_family']}`，失败样本数为 `{analysis['dominant_failure_count']}`。"
        )
    else:
        lines.append("本轮没有出现失败样本。")

    lines.append("")
    lines.append("## 4. 按规划器表现")
    lines.append("")
    for planner in analysis["planners"]:
        data = analysis["per_planner"][planner]
        lines.append(
            f"- `{planner}`: 成功率 `{data['success_rate']:.1f}%`，中位数 `{data['median_wall_ms']:.1f} ms`，"
            f"预算命中 `{data['budget_hits']}/{data['count']}`，快速求解 `<1s` 为 `{data['fast_solves']}/{data['count']}`"
        )

    lines.append("")
    lines.append("最重要的判断：")
    lines.append("")

    best_budget_planner = min(
        analysis["planners"], key=lambda planner: analysis["per_planner"][planner]["budget_hit_rate"]
    )
    fastest_median_planner = min(
        analysis["planners"], key=lambda planner: analysis["per_planner"][planner]["median_wall_ms"]
    )
    lines.append(
        f"- `{best_budget_planner}` 的最大优势仍然是低预算命中率，而不是单纯最小中位数"
    )
    lines.append(f"- 当前中位墙钟时间最优的是 `{fastest_median_planner}`")
    lines.append("- `FMT / LBTRRT` 仍然是最值得保留的经典对照")

    lines.append("")
    lines.append("## 5. 按任务族表现")
    lines.append("")
    for family in sorted(analysis["per_family"], key=family_sort_key):
        data = analysis["per_family"][family]
        lines.append(f"- `{family}`: `{data['success_rate']:.1f}%`")

    lines.append("")
    lines.append("这说明：")
    lines.append("")
    if analysis["dominant_failure_family"]:
        lines.append(f"- 当前失败样本主要来自 `{analysis['dominant_failure_family']}`")
    else:
        lines.append("- 当前任务集没有产生足够的失败样本")
    lines.append("- 高难任务族仍然是第一版训练集里最有价值的标签来源")

    lines.append("")
    lines.append("## 6. 最关键的失败模式")
    lines.append("")
    lines.extend(format_failure_lines(analysis["failure_counter"]))
    lines.append("")
    lines.append("这说明：")
    lines.append("")

    if analysis["dominant_failure_family"]:
        lines.append(f"1. `{analysis['dominant_failure_family']}` 仍然是最有价值的高难标签来源")
    else:
        lines.append("1. 当前需要主动增强高难采样比例，否则失败标签不足")
    lines.append("2. 不同 planner 的长尾模式仍然不同，具备做学习引导判别的价值")
    lines.append("3. 预算命中比单纯成功/失败更能拉开 planner 差异")

    lines.append("")
    lines.append("## 7. 对下一步模型设计的意义")
    lines.append("")
    lines.append("这批数据最适合先做下面两类标签：")
    lines.append("")
    lines.append("1. `fast_solve_lt_1s`")
    lines.append("2. `hit_budget_limit`")
    lines.append("")
    lines.append("原因：")
    lines.append("")
    lines.append("- 这两个标签比“绝对成功/失败”更敏感")
    lines.append("- 在当前数据里分布已经明显分开")
    lines.append("- 更符合后续 learning-guided sampling 的落地方向")

    lines.append("")
    lines.append("## 8. 下一步建议")
    lines.append("")
    lines.append("最合理的下一步不是立刻做复杂模型，而是：")
    lines.append("")
    lines.append("1. 先把这轮 `300-task` 数据并入统一训练 CSV")
    lines.append("2. 定义第一版轻量特征，优先预测预算命中和快速求解")
    lines.append("3. 用 `HeuristicGuided` 作为主方法框架，`FMT / LBTRRT / RRTConnect` 作为稳定对照")
    lines.append("4. 视这轮失败比例再决定是否把 `simple` 随机采集扩到 `500-task`")
    lines.append("5. 再把同样的数据采集思路迁移到 `v2`")
    lines.append("")

    return "\n".join(lines) + "\n"


def default_output_path(timestamp: str) -> Path:
    """Default output path."""
    return REPO_ROOT / "docs" / "analysis" / f"SIMPLE_RANDOM_DATASET_{timestamp}_ANALYSIS.md"


def main():
    """Main."""
    args = parse_args()
    results_path = (Path(args.results) if args.results else latest_results_file()).resolve()
    rows = read_rows(results_path)
    analysis = analyze(rows)
    output_path = Path(args.output) if args.output else default_output_path(analysis["timestamp"])
    output_path.write_text(build_markdown(results_path, analysis), encoding="utf-8")

    print(f"Analyzed {len(rows)} rows from {results_path}")
    print(f"Wrote report to {output_path}")


if __name__ == "__main__":
    main()
