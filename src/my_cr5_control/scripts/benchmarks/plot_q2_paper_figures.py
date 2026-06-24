#!/usr/bin/env python3
"""Q2 paper figure plotter — generates publication-quality figures (bar charts, box plots, scatter plots) for the Q2 journal paper from benchmark results."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, Iterable, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
BUNDLE_ROOT = REPO_ROOT / "paper_workspace" / "formal_results" / "q2_formal_20260324"
TABLE_DIR = BUNDLE_ROOT / "table_inputs"
PLOT_INPUT_DIR = BUNDLE_ROOT / "plot_inputs"
GATE_DIR = BUNDLE_ROOT / "gate_activity"
FIGURE_DIR = BUNDLE_ROOT / "figures"

PLANNER_ORDER = [
    "HeuristicGuided",
    "LBTRRT",
    "FMT",
    "PRMstar",
    "RRTConnect",
    "BFMT",
    "RRTstar",
]

PLANNER_COLORS = {
    "HeuristicGuided": "#c2410c",
    "LBTRRT": "#2563eb",
    "FMT": "#059669",
    "PRMstar": "#7c3aed",
    "RRTConnect": "#0891b2",
    "BFMT": "#65a30d",
    "RRTstar": "#6b7280",
}

CLASS_COLORS = {
    "active_guidance": "#c2410c",
    "dormant_near_tie_long_tail": "#1d4ed8",
}

DIFFICULTY_ORDER = {
    "easy": 0,
    "medium": 1,
    "hard": 2,
    "extreme": 3,
}


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    """Read CSV file and return rows as a list of dicts."""
    with path.open("r", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def ordered_rows(rows: Iterable[Dict[str, str]], key: str, preferred_order: List[str]) -> List[Dict[str, str]]:
    """Ordered rows."""
    index = {name: i for i, name in enumerate(preferred_order)}
    return sorted(rows, key=lambda row: index.get(row[key], 10_000))


def planner_color(planner: str) -> str:
    """Planner color."""
    return PLANNER_COLORS.get(planner, "#6b7280")


def setup_style() -> None:
    """Setup style."""
    plt.rcParams.update(
        {
            "font.family": "DejaVu Serif",
            "axes.titlesize": 12,
            "axes.labelsize": 10,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "figure.titlesize": 14,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def save_figure(fig: plt.Figure, stem: str) -> None:
    """Save figure."""
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    for suffix in (".png", ".pdf"):
        fig.savefig(FIGURE_DIR / f"{stem}{suffix}", dpi=240, bbox_inches="tight")
    plt.close(fig)


def annotate_hbar(ax: plt.Axes, bars, fmt: str, x_offset: float) -> None:
    """Annotate hbar."""
    for bar in bars:
        width = bar.get_width()
        ax.text(
            width + x_offset,
            bar.get_y() + bar.get_height() / 2.0,
            fmt.format(width),
            va="center",
            ha="left",
            fontsize=8,
            color="#111827",
        )


def plot_main_overview() -> None:
    """Plot main overview."""
    simple_rows = ordered_rows(
        read_csv_rows(TABLE_DIR / "simple_main_table.csv"),
        "planner",
        PLANNER_ORDER,
    )
    v2_rows = ordered_rows(
        read_csv_rows(TABLE_DIR / "v2_main_table.csv"),
        "planner",
        PLANNER_ORDER,
    )

    fig, axes = plt.subplots(2, 2, figsize=(12.5, 9.5))
    panels = [
        ("Simple Mean Planning Time", simple_rows, "mean_ms", "Mean wall time (ms)", True),
        ("V2 Mean Planning Time", v2_rows, "mean_ms", "Mean wall time (ms)", True),
        ("Simple Budget-Hit Rate", simple_rows, "budget_hit_rate_pct", "Budget-hit rate (%)", False),
        ("V2 Budget-Hit Rate", v2_rows, "budget_hit_rate_pct", "Budget-hit rate (%)", False),
    ]

    for ax, (title, rows, value_key, xlabel, use_log) in zip(axes.flatten(), panels):
        planners = [row["planner"] for row in rows]
        values = [float(row[value_key]) for row in rows]
        positions = np.arange(len(planners))
        colors = [planner_color(planner) for planner in planners]
        bars = ax.barh(positions, values, color=colors, alpha=0.92)
        ax.set_yticks(positions, planners)
        ax.invert_yaxis()
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.grid(axis="x", linestyle="--", linewidth=0.6, alpha=0.35)
        if use_log:
            ax.set_xscale("log")
            annotate_hbar(ax, bars, "{:.0f}", 0.05 * max(values))
        else:
            ax.set_xlim(0.0, max(values) * 1.18 + 1.0)
            annotate_hbar(ax, bars, "{:.1f}", max(values) * 0.015 + 0.05)

    fig.suptitle("Main Benchmark Results")
    fig.tight_layout()
    save_figure(fig, "figure_01_main_overview")


def load_time_groups(path: Path) -> Dict[str, List[float]]:
    """Load time groups."""
    rows = read_csv_rows(path)
    groups: Dict[str, List[float]] = {}
    for row in rows:
        planner = row["规划器"]
        groups.setdefault(planner, []).append(float(row["墙钟时间(ms)"]))
    return groups


def color_boxplot(boxplot, planners: List[str]) -> None:
    """Color boxplot."""
    for patch, planner in zip(boxplot["boxes"], planners):
        patch.set_facecolor(planner_color(planner))
        patch.set_alpha(0.55)
        patch.set_edgecolor("#374151")
    for median in boxplot["medians"]:
        median.set_color("#111827")
        median.set_linewidth(1.4)
    for whisker in boxplot["whiskers"]:
        whisker.set_color("#4b5563")
    for cap in boxplot["caps"]:
        cap.set_color("#4b5563")


def plot_wall_time_boxplots() -> None:
    """Plot wall time boxplots."""
    simple_groups = load_time_groups(
        PLOT_INPUT_DIR / "simple" / "planner_comparison_simple_plot_data_metrics.csv"
    )
    v2_groups = load_time_groups(
        PLOT_INPUT_DIR / "v2" / "planner_comparison_v2_plot_data_metrics.csv"
    )

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.4), sharey=True)
    for ax, title, groups in (
        (axes[0], "Simple Benchmark", simple_groups),
        (axes[1], "V2 Benchmark", v2_groups),
    ):
        planners = [planner for planner in PLANNER_ORDER if planner in groups]
        values = [groups[planner] for planner in planners]
        bp = ax.boxplot(
            values,
            patch_artist=True,
            labels=planners,
            showfliers=True,
            widths=0.65,
        )
        color_boxplot(bp, planners)
        means = [np.mean(group) for group in values]
        ax.scatter(
            np.arange(1, len(planners) + 1),
            means,
            marker="D",
            s=22,
            c="#111827",
            label="Mean",
            zorder=4,
        )
        ax.set_title(title)
        ax.set_yscale("log")
        ax.set_ylabel("Wall time (ms)")
        ax.set_xticklabels(planners, rotation=28, ha="right")
        ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.35)

    axes[1].legend(loc="upper left")
    fig.suptitle("Wall-Time Distribution Across Planners")
    fig.tight_layout()
    save_figure(fig, "figure_02_wall_time_boxplots")


def plot_cross_benchmark_budget_hit() -> None:
    """Plot cross benchmark budget hit."""
    rows = ordered_rows(
        read_csv_rows(TABLE_DIR / "cross_benchmark_budget_hit.csv"),
        "planner",
        PLANNER_ORDER,
    )

    planners = [row["planner"] for row in rows]
    simple = np.array([float(row["simple_budget_hit_rate_pct"]) for row in rows])
    v2 = np.array([float(row["v2_budget_hit_rate_pct"]) for row in rows])
    combined = np.array([float(row["combined_budget_hit_rate_pct"]) for row in rows])

    x = np.arange(len(planners))
    width = 0.23

    fig, ax = plt.subplots(figsize=(11.8, 5.6))
    ax.bar(x - width, simple, width, label="Simple", color="#60a5fa")
    ax.bar(x, v2, width, label="V2", color="#34d399")
    ax.bar(x + width, combined, width, label="Combined", color="#f59e0b")

    for idx, planner in enumerate(planners):
        if planner == "HeuristicGuided":
            ax.axvspan(idx - 0.5, idx + 0.5, color="#fed7aa", alpha=0.28, zorder=0)

    ax.set_xticks(x, planners, rotation=28, ha="right")
    ax.set_ylabel("Budget-hit rate (%)")
    ax.set_title("Cross-Benchmark Stability by Budget-Hit Rate")
    ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.35)
    ax.legend(loc="upper left", ncol=3)

    fig.tight_layout()
    save_figure(fig, "figure_03_cross_benchmark_budget_hit")


def plot_selective_guidance_profile() -> None:
    """Plot selective guidance profile."""
    profile_rows = read_csv_rows(TABLE_DIR / "heuristicguided_scene_gate_profile.csv")
    benchmark_order = ["simple", "v2"]
    benchmark_titles = {"simple": "Simple formal rerun", "v2": "V2 formal rerun"}

    fig, axes = plt.subplots(1, 2, figsize=(14.2, 5.5), sharey=False)
    for ax, benchmark in zip(axes, benchmark_order):
        rows = [row for row in profile_rows if row["benchmark"] == benchmark]
        rows.sort(key=lambda row: DIFFICULTY_ORDER.get(row["difficulty"], 99))
        scenes = [row["scene"] for row in rows]
        attempt = np.array([float(row["attempt_rate"]) * 100.0 for row in rows])
        fallback = np.array([float(row["direct_fallback_rate"]) * 100.0 for row in rows])
        mean_ms = np.array([float(row["mean_ms"]) for row in rows])
        classes = [row["class"] for row in rows]

        x = np.arange(len(rows))
        width = 0.34
        for idx, row_class in enumerate(classes):
            ax.axvspan(
                idx - 0.5,
                idx + 0.5,
                color=CLASS_COLORS.get(row_class, "#d1d5db"),
                alpha=0.10,
                zorder=0,
            )

        ax.bar(x - width / 2.0, attempt, width, color="#c2410c", label="Guide attempt rate")
        ax.bar(x + width / 2.0, fallback, width, color="#2563eb", label="Direct fallback rate")
        ax.set_xticks(x, scenes, rotation=28, ha="right")
        ax.set_ylabel("Rate (%)")
        ax.set_ylim(0.0, 115.0)
        ax.set_title(benchmark_titles[benchmark])
        ax.grid(axis="y", linestyle="--", linewidth=0.6, alpha=0.35)

        twin = ax.twinx()
        twin.plot(x, mean_ms, color="#111827", marker="o", linewidth=1.6, label="Mean wall time")
        twin.set_ylabel("Mean wall time (ms)")
        twin.set_ylim(0.0, max(mean_ms) * 1.25)

        handles_1, labels_1 = ax.get_legend_handles_labels()
        handles_2, labels_2 = twin.get_legend_handles_labels()
        ax.legend(handles_1 + handles_2, labels_1 + labels_2, loc="upper right")

    fig.suptitle("Selective Guidance Evidence in Formal Reruns")
    fig.tight_layout()
    save_figure(fig, "figure_04_selective_guidance_profile")


def write_readme() -> None:
    """Write readme."""
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    content = """# Paper Figures

This directory stores paper-facing figures generated from the official Q2 artifact bundle.

## Figures

- `figure_01_main_overview`
  - 2x2 summary of `simple` and `v2` main results
  - data source:
    - `table_inputs/simple_main_table.csv`
    - `table_inputs/v2_main_table.csv`

- `figure_02_wall_time_boxplots`
  - planner-wise wall-time distributions for `simple` and `v2`
  - data source:
    - `plot_inputs/simple/planner_comparison_simple_plot_data_metrics.csv`
    - `plot_inputs/v2/planner_comparison_v2_plot_data_metrics.csv`

- `figure_03_cross_benchmark_budget_hit`
  - cross-benchmark budget-hit comparison
  - data source:
    - `table_inputs/cross_benchmark_budget_hit.csv`

- `figure_04_selective_guidance_profile`
  - scene-level evidence for dormant direct-only vs active guidance behavior
  - data source:
    - `table_inputs/heuristicguided_scene_gate_profile.csv`

## Regeneration

```bash
python3 scripts/benchmarks/plot_q2_paper_figures.py
```
"""
    (FIGURE_DIR / "README.md").write_text(content, encoding="utf-8")


def main() -> int:
    """Main."""
    setup_style()
    plot_main_overview()
    plot_wall_time_boxplots()
    plot_cross_benchmark_budget_hit()
    plot_selective_guidance_profile()
    write_readme()
    print(FIGURE_DIR)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
