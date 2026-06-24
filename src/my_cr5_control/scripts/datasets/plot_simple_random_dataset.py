#!/usr/bin/env python3
"""Simple random dataset plotter — generates visualizations (histograms, scatter matrices, heatmaps) of the simple-random benchmark dataset features and outcomes."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = REPO_ROOT / "test_results"
DEFAULT_RESULTS_GLOB = "datasets/simple_random/raw/*_simple_random_task_dataset_results.csv"
DEFAULT_SUMMARY_GLOB = "datasets/simple_random/raw/*_simple_random_task_dataset_summary.csv"
DEFAULT_PLOTS_ROOT = RESULTS_ROOT / "datasets" / "simple_random" / "plots"

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
        description="Generate plots for the latest simple random dataset."
    )
    parser.add_argument(
        "--results",
        default="",
        help="Input *_simple_random_task_dataset_results.csv path. Default: latest curated file.",
    )
    parser.add_argument(
        "--summary",
        default="",
        help="Input *_simple_random_task_dataset_summary.csv path. Default: summary next to the results file.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Optional output directory. Default: test_results/datasets/simple_random/plots/<timestamp>/",
    )
    return parser.parse_args()


def latest_file(pattern: str) -> Path:
    """Latest file."""
    matches = sorted(RESULTS_ROOT.glob(pattern))
    if not matches:
        raise RuntimeError(f"No files matched pattern: {pattern}")
    return matches[-1]


def read_csv_rows(path: Path):
    """Read CSV file and return rows as a list of dicts."""
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


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


def parse_float(value: str, default: float = 0.0) -> float:
    """Parse a string to float, returning a default for empty/missing values."""
    if value in ("", None):
        return default
    return float(value)


def write_csv(path: Path, fieldnames, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def annotate_bars(ax, bars, value_format="{:.1f}"):
    """Annotate bars."""
    for bar in bars:
        height = bar.get_height()
        ax.annotate(
            value_format.format(height),
            xy=(bar.get_x() + bar.get_width() / 2, height),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=8,
            rotation=90,
        )


def save_bar_chart(output_path: Path, labels, values, title: str, ylabel: str, color: str):
    """Save bar chart."""
    fig, ax = plt.subplots(figsize=(10, 5))
    bars = ax.bar(labels, values, color=color)
    annotate_bars(ax, bars)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", alpha=0.25)
    ax.set_ylim(0, max(max(values, default=0.0) * 1.15, 1.0))
    plt.xticks(rotation=20)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def save_grouped_rate_chart(output_path: Path, planners, budget_rates, fast_rates):
    """Save grouped rate chart."""
    x_positions = list(range(len(planners)))
    width = 0.38
    fig, ax = plt.subplots(figsize=(11, 5))
    budget_bars = ax.bar(
        [x - width / 2 for x in x_positions],
        budget_rates,
        width=width,
        label="Budget Hit Rate",
        color="#E15759",
    )
    fast_bars = ax.bar(
        [x + width / 2 for x in x_positions],
        fast_rates,
        width=width,
        label="Fast Solve Rate (<1s)",
        color="#76B7B2",
    )
    annotate_bars(ax, budget_bars)
    annotate_bars(ax, fast_bars)
    ax.set_xticks(x_positions)
    ax.set_xticklabels(planners, rotation=20)
    ax.set_ylabel("Rate (%)")
    ax.set_title("Budget Hit vs Fast Solve")
    ax.set_ylim(0, 110)
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def save_failure_heatmap(output_path: Path, planners, families, failure_rates):
    """Save failure heatmap."""
    values = [
        [failure_rates[family].get(planner, 0.0) for planner in planners]
        for family in families
    ]

    fig, ax = plt.subplots(figsize=(10, 5))
    image = ax.imshow(values, cmap="YlOrRd", aspect="auto", vmin=0, vmax=100)
    ax.set_xticks(range(len(planners)))
    ax.set_xticklabels(planners, rotation=20, ha="right")
    ax.set_yticks(range(len(families)))
    ax.set_yticklabels(families)
    ax.set_title("Failure Rate by Planner and Task Family")

    for family_index, family in enumerate(families):
        for planner_index, planner in enumerate(planners):
            value = failure_rates[family].get(planner, 0.0)
            ax.text(planner_index, family_index, f"{value:.0f}%", ha="center", va="center", fontsize=8)

    fig.colorbar(image, ax=ax, label="Failure Rate (%)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main():
    """Main."""
    args = parse_args()
    results_path = Path(args.results) if args.results else latest_file(DEFAULT_RESULTS_GLOB)
    summary_path = (
        Path(args.summary)
        if args.summary
        else results_path.with_name(results_path.name.replace("_results.csv", "_summary.csv"))
    )

    detail_rows = read_csv_rows(results_path)
    summary_rows = read_csv_rows(summary_path)
    if not detail_rows or not summary_rows:
        raise RuntimeError("Results or summary csv is empty.")

    timestamp = detail_rows[0].get("实验时间戳", "unknown")
    output_dir = Path(args.output_dir) if args.output_dir else DEFAULT_PLOTS_ROOT / timestamp
    output_dir.mkdir(parents=True, exist_ok=True)

    planners = sorted({row["规划器"] for row in detail_rows if row.get("规划器")}, key=planner_sort_key)
    families = sorted({row["任务族"] for row in detail_rows if row.get("任务族")}, key=family_sort_key)

    planner_rows = {}
    for row in summary_rows:
        planner_rows[row["规划器"]] = row

    planner_overall = []
    for planner in planners:
        row = planner_rows[planner]
        planner_overall.append(
            {
                "planner": planner,
                "success_rate_pct": parse_float(row.get("成功率(%)")),
                "median_time_ms": parse_float(row.get("中位时间(ms)")),
                "budget_hit_rate_pct": parse_float(row.get("预算命中率(%)")),
                "fast_solve_rate_pct": parse_float(row.get("快速求解率(%)")),
                "avg_moveit_time_ms": parse_float(row.get("平均MoveIt规划时间(ms)")),
            }
        )

    family_success = []
    failure_rates = defaultdict(dict)
    failure_rows = []
    for family in families:
        family_rows = [row for row in detail_rows if row.get("任务族") == family]
        total_count = len(family_rows)
        success_count = sum(row.get("成功") == "成功" for row in family_rows)
        family_success.append(
            {
                "family": family,
                "success_rate_pct": 100.0 * success_count / total_count if total_count else 0.0,
                "success_count": success_count,
                "total_count": total_count,
            }
        )

        for planner in planners:
            planner_family_rows = [
                row
                for row in family_rows
                if row.get("规划器") == planner
            ]
            planner_total = len(planner_family_rows)
            planner_failures = sum(row.get("成功") != "成功" for row in planner_family_rows)
            planner_failure_rate = 100.0 * planner_failures / planner_total if planner_total else 0.0
            failure_rates[family][planner] = planner_failure_rate
            failure_rows.append(
                {
                    "family": family,
                    "planner": planner,
                    "failure_count": planner_failures,
                    "total_count": planner_total,
                    "failure_rate_pct": planner_failure_rate,
                }
            )

    write_csv(
        output_dir / "planner_overall.csv",
        [
            "planner",
            "success_rate_pct",
            "median_time_ms",
            "budget_hit_rate_pct",
            "fast_solve_rate_pct",
            "avg_moveit_time_ms",
        ],
        planner_overall,
    )
    write_csv(
        output_dir / "family_success.csv",
        ["family", "success_rate_pct", "success_count", "total_count"],
        family_success,
    )
    write_csv(
        output_dir / "planner_family_failures.csv",
        ["family", "planner", "failure_count", "total_count", "failure_rate_pct"],
        failure_rows,
    )

    save_bar_chart(
        output_dir / "overall_success_rate.png",
        [row["planner"] for row in planner_overall],
        [row["success_rate_pct"] for row in planner_overall],
        title="Planner Success Rate",
        ylabel="Success Rate (%)",
        color="#4E79A7",
    )
    save_bar_chart(
        output_dir / "overall_median_time.png",
        [row["planner"] for row in planner_overall],
        [row["median_time_ms"] for row in planner_overall],
        title="Planner Median Wall Time",
        ylabel="Median Time (ms)",
        color="#F28E2B",
    )
    save_grouped_rate_chart(
        output_dir / "budget_vs_fast_rate.png",
        [row["planner"] for row in planner_overall],
        [row["budget_hit_rate_pct"] for row in planner_overall],
        [row["fast_solve_rate_pct"] for row in planner_overall],
    )
    save_bar_chart(
        output_dir / "family_success_rate.png",
        [row["family"] for row in family_success],
        [row["success_rate_pct"] for row in family_success],
        title="Task Family Success Rate",
        ylabel="Success Rate (%)",
        color="#59A14F",
    )
    save_failure_heatmap(
        output_dir / "planner_family_failure_heatmap.png",
        planners,
        families,
        failure_rates,
    )

    print(f"results={results_path}")
    print(f"summary={summary_path}")
    print(f"output_dir={output_dir}")


if __name__ == "__main__":
    main()
