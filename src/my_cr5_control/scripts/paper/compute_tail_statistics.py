#!/usr/bin/env python3
"""Recompute manuscript tail statistics from the frozen formal-run CSV files."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/results/benchmarks"
SEED = 20260527
RESAMPLES = 20_000
FILES = {
    ("simple", "HeuristicGuided"): RESULTS / "simple/raw/20260503_154557_849_planner_comparison_simple_results.csv",
    ("simple", "RRTConnect"): RESULTS / "simple/raw/20260503_135840_216_planner_comparison_simple_results.csv",
    ("v2/WS119", "HeuristicGuided"): RESULTS / "v2/raw/20260503_170022_118_planner_comparison_v2_results.csv",
    ("v2/WS119", "RRTConnect"): RESULTS / "v2/raw/20260503_154846_556_planner_comparison_v2_results.csv",
}


def load(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load."""
    with path.open(encoding="utf-8-sig") as source:
        rows = list(csv.DictReader(source))
    times = np.asarray([float(row["墙钟时间(ms)"]) for row in rows])
    budget_hits = np.asarray([row["触发预算上限"] == "是" for row in rows])
    return times, budget_hits


def wilson_interval(hits: int, count: int, z: float = 1.959963984540054) -> tuple[float, float]:
    """Wilson interval."""
    rate = hits / count
    denominator = 1 + z * z / count
    midpoint = (rate + z * z / (2 * count)) / denominator
    half_width = z * math.sqrt(rate * (1 - rate) / count + z * z / (4 * count * count)) / denominator
    return 100 * (midpoint - half_width), 100 * (midpoint + half_width)


def main() -> None:
    """Main."""
    rng = np.random.default_rng(SEED)
    samples: dict[tuple[str, str], np.ndarray] = {}
    print(f"Bootstrap resamples: {RESAMPLES}; random seed: {SEED}")
    print("| 测试集 | 方法 | 样本数 | 平均时间(ms) | 均值 bootstrap 95% CI(ms) | P90(ms) | 最大值(ms) | 预算耗尽率(%) [Wilson 95% CI] |")
    print("| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for key, path in FILES.items():
        times, hits = load(path)
        samples[key] = times
        draw = times[rng.integers(0, len(times), (RESAMPLES, len(times)))].mean(axis=1)
        mean_ci = np.quantile(draw, [0.025, 0.975])
        budget_ci = wilson_interval(int(hits.sum()), len(hits))
        print(
            f"| `{key[0]}` | `{key[1]}` | {len(times)} | {times.mean():.1f} | "
            f"[{mean_ci[0]:.1f}, {mean_ci[1]:.1f}] | {np.quantile(times, 0.9):.1f} | "
            f"{times.max():.1f} | {100 * hits.mean():.1f} [{budget_ci[0]:.1f}, {budget_ci[1]:.1f}] |"
        )
    for benchmark in ("simple", "v2/WS119"):
        baseline = samples[(benchmark, "RRTConnect")]
        proposed = samples[(benchmark, "HeuristicGuided")]
        draw_baseline = baseline[rng.integers(0, len(baseline), (RESAMPLES, len(baseline)))].mean(axis=1)
        draw_proposed = proposed[rng.integers(0, len(proposed), (RESAMPLES, len(proposed)))].mean(axis=1)
        reductions = 100 * (1 - draw_proposed / draw_baseline)
        point = 100 * (1 - proposed.mean() / baseline.mean())
        interval = np.quantile(reductions, [0.025, 0.975])
        print(f"{benchmark}: mean-time reduction {point:.1f}% [{interval[0]:.1f}%, {interval[1]:.1f}%]")


if __name__ == "__main__":
    main()
