#!/usr/bin/env python3
"""Analyze heuristic rescue candidates — identifies planning queries where the default planner failed but a heuristic-guided retry succeeded, quantifying rescue effectiveness."""

import argparse
import csv
import json
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_GLOB = "test_results/benchmarks/simple/raw/*_planner_comparison_simple_results.csv"


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description=(
            "Analyze HeuristicGuided benchmark raw csv and identify scene subsets worth "
            "direct-rescue / slow-direct gate tuning."
        )
    )
    parser.add_argument(
        "--baseline-results",
        default="",
        help="Baseline raw results csv. Default: latest simple raw results csv.",
    )
    parser.add_argument(
        "--compare-results",
        default="",
        help="Optional comparison raw results csv, e.g. a rescue-enabled run.",
    )
    parser.add_argument(
        "--planner",
        default="HeuristicGuided",
        help="Planner name to analyze. Default: HeuristicGuided.",
    )
    parser.add_argument(
        "--mode",
        default="heuristic_guided",
        help="Planning mode to analyze. Default: heuristic_guided.",
    )
    parser.add_argument(
        "--slow-threshold-ms",
        type=float,
        default=800.0,
        help="Slow direct threshold used to flag long-tail cases. Default: 800 ms.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Optional output directory. Default: test_results/exports/heuristic_rescue_analysis/<timestamp>/",
    )
    return parser.parse_args()


def latest_results_path():
    """Latest results path."""
    matches = sorted(REPO_ROOT.glob(DEFAULT_RESULTS_GLOB))
    if not matches:
        raise RuntimeError("No simple benchmark raw results csv was found.")
    return matches[-1]


def read_csv_rows(path):
    """Read CSV file and return rows as a list of dicts."""
    with open(path, "r", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def parse_float(value, default=0.0):
    """Parse a string to float, returning a default for empty/missing values."""
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def parse_int(value, default=0):
    """Parse int."""
    if value is None or value == "":
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def parse_bool(value):
    """Parse bool."""
    return str(value).strip() in {"1", "true", "True", "TRUE", "yes", "Yes", "是", "成功"}


def percentile(values, q):
    """Percentile."""
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * q
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[int(position)]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def filter_rows(rows, planner_name, mode_name):
    """Filter rows."""
    filtered = [
        row
        for row in rows
        if row.get("规划器", "") == planner_name and row.get("模式", "") == mode_name
    ]
    if not filtered:
        raise RuntimeError(
            f"No rows matched planner={planner_name!r}, mode={mode_name!r}."
        )
    return filtered


def difficulty_rank(difficulty):
    """Difficulty rank."""
    order = {"easy": 0, "medium": 1, "hard": 2, "extreme": 3}
    return order.get(difficulty, 99)


def rescue_priority_score(summary, slow_threshold_ms):
    """Rescue priority score."""
    tail_excess = max(summary["p75_ms"] - slow_threshold_ms, 0.0) / max(slow_threshold_ms, 1.0)
    slow_rate = summary["slow_rate"]
    directness = summary["direct_fallback_rate"]
    budget_rate = summary["budget_hit_rate"]
    return tail_excess * (0.60 * directness + 0.40 * max(slow_rate, budget_rate))


def classify_scene(summary, slow_threshold_ms):
    """Classify scene."""
    if summary["guide_attempt_rate"] > 0.0 and summary["p75_ms"] > slow_threshold_ms:
        return "guide_already_triggered_but_still_slow"
    if summary["direct_fallback_rate"] >= 0.90 and summary["slow_rate"] >= 0.25:
        return "direct_fallback_long_tail"
    if summary["direct_fallback_rate"] >= 0.90 and summary["p75_ms"] >= slow_threshold_ms:
        return "direct_fallback_tail_risk"
    return "no_clear_rescue_signal"


def summarize_by_scene(rows, slow_threshold_ms):
    """Summarize by scene."""
    scenes = {}
    for row in rows:
        scene_name = row.get("场景名称", "")
        if not scene_name:
            continue
        scenes.setdefault(scene_name, []).append(row)

    summaries = []
    for scene_name, scene_rows in scenes.items():
        wall_times = [parse_float(row.get("墙钟时间(ms)", 0.0)) for row in scene_rows]
        moveit_times = [parse_float(row.get("MoveIt规划时间(ms)", 0.0)) for row in scene_rows]
        guide_attempt_counts = [parse_int(row.get("guide尝试数", 0)) for row in scene_rows]
        guide_candidate_counts = [parse_int(row.get("guide候选数", 0)) for row in scene_rows]
        planner_calls = [parse_int(row.get("规划调用次数", 0)) for row in scene_rows]

        count = len(scene_rows)
        slow_count = sum(value >= slow_threshold_ms for value in wall_times)
        budget_hit_count = sum(parse_bool(row.get("触发预算上限", "否")) for row in scene_rows)
        direct_fallback_count = sum(parse_bool(row.get("使用direct回退", "否")) for row in scene_rows)
        guide_attempted_count = sum(value > 0 for value in guide_attempt_counts)

        summary = {
            "scene_name": scene_name,
            "difficulty": scene_rows[0].get("难度", ""),
            "sample_count": count,
            "mean_ms": sum(wall_times) / count if count else 0.0,
            "median_ms": percentile(wall_times, 0.5),
            "p75_ms": percentile(wall_times, 0.75),
            "p90_ms": percentile(wall_times, 0.90),
            "max_ms": max(wall_times) if wall_times else 0.0,
            "mean_moveit_ms": sum(moveit_times) / count if count else 0.0,
            "slow_count": slow_count,
            "slow_rate": slow_count / count if count else 0.0,
            "budget_hit_count": budget_hit_count,
            "budget_hit_rate": budget_hit_count / count if count else 0.0,
            "direct_fallback_count": direct_fallback_count,
            "direct_fallback_rate": direct_fallback_count / count if count else 0.0,
            "guide_attempt_rate": guide_attempted_count / count if count else 0.0,
            "avg_guide_attempts": sum(guide_attempt_counts) / count if count else 0.0,
            "avg_guide_candidates": sum(guide_candidate_counts) / count if count else 0.0,
            "avg_planner_calls": sum(planner_calls) / count if count else 0.0,
        }
        summary["rescue_priority_score"] = rescue_priority_score(summary, slow_threshold_ms)
        summary["scene_classification"] = classify_scene(summary, slow_threshold_ms)
        summaries.append(summary)

    summaries.sort(
        key=lambda item: (
            -item["rescue_priority_score"],
            -difficulty_rank(item["difficulty"]),
            item["scene_name"],
        )
    )
    return summaries


def compare_scene_summaries(baseline_summaries, compare_summaries):
    """Compare scene summaries."""
    compare_index = {item["scene_name"]: item for item in compare_summaries}
    compared_rows = []
    for baseline in baseline_summaries:
        other = compare_index.get(baseline["scene_name"])
        if other is None:
            continue
        delta_mean = other["mean_ms"] - baseline["mean_ms"]
        delta_p75 = other["p75_ms"] - baseline["p75_ms"]
        delta_attempt_rate = other["guide_attempt_rate"] - baseline["guide_attempt_rate"]
        delta_direct_fallback_rate = other["direct_fallback_rate"] - baseline["direct_fallback_rate"]
        if delta_attempt_rate > 0.0 and delta_mean <= -100.0:
            effect = "rescue_helped"
        elif delta_attempt_rate > 0.0 and delta_mean >= 100.0:
            effect = "rescue_hurt"
        else:
            effect = "no_clear_change"
        compared_rows.append(
            {
                "scene_name": baseline["scene_name"],
                "difficulty": baseline["difficulty"],
                "baseline_mean_ms": baseline["mean_ms"],
                "compare_mean_ms": other["mean_ms"],
                "delta_mean_ms": delta_mean,
                "baseline_p75_ms": baseline["p75_ms"],
                "compare_p75_ms": other["p75_ms"],
                "delta_p75_ms": delta_p75,
                "baseline_attempt_rate": baseline["guide_attempt_rate"],
                "compare_attempt_rate": other["guide_attempt_rate"],
                "delta_attempt_rate": delta_attempt_rate,
                "baseline_direct_fallback_rate": baseline["direct_fallback_rate"],
                "compare_direct_fallback_rate": other["direct_fallback_rate"],
                "delta_direct_fallback_rate": delta_direct_fallback_rate,
                "effect": effect,
            }
        )
    compared_rows.sort(
        key=lambda item: (
            -difficulty_rank(item["difficulty"]),
            -abs(item["delta_mean_ms"]),
            item["scene_name"],
        )
    )
    return compared_rows


def recommended_subset(scene_summaries, slow_threshold_ms):
    """Recommended subset."""
    recommended = []
    for item in scene_summaries:
        if item["direct_fallback_rate"] < 0.90:
            continue
        if item["guide_attempt_rate"] > 0.0:
            continue
        if item["p75_ms"] < slow_threshold_ms and item["slow_rate"] < 0.25:
            continue
        recommended.append(item["scene_name"])
    return recommended


def write_csv(path, fieldnames, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    """Main."""
    args = parse_args()
    baseline_path = Path(args.baseline_results) if args.baseline_results else latest_results_path()
    baseline_rows = filter_rows(read_csv_rows(baseline_path), args.planner, args.mode)
    baseline_summaries = summarize_by_scene(baseline_rows, args.slow_threshold_ms)

    compare_path = Path(args.compare_results) if args.compare_results else None
    compare_summaries = []
    compare_rows = []
    if compare_path is not None:
        compare_rows = filter_rows(read_csv_rows(compare_path), args.planner, args.mode)
        compare_summaries = summarize_by_scene(compare_rows, args.slow_threshold_ms)

    timestamp = baseline_rows[0].get("实验时间戳", "unknown")
    default_output_dir = (
        REPO_ROOT / "test_results" / "exports" / "heuristic_rescue_analysis" / timestamp
    )
    output_dir = Path(args.output_dir) if args.output_dir else default_output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    write_csv(
        output_dir / "scene_summary.csv",
        [
            "scene_name",
            "difficulty",
            "sample_count",
            "mean_ms",
            "median_ms",
            "p75_ms",
            "p90_ms",
            "max_ms",
            "mean_moveit_ms",
            "slow_count",
            "slow_rate",
            "budget_hit_count",
            "budget_hit_rate",
            "direct_fallback_count",
            "direct_fallback_rate",
            "guide_attempt_rate",
            "avg_guide_attempts",
            "avg_guide_candidates",
            "avg_planner_calls",
            "rescue_priority_score",
            "scene_classification",
        ],
        baseline_summaries,
    )

    comparison_rows = compare_scene_summaries(baseline_summaries, compare_summaries)
    if comparison_rows:
        write_csv(
            output_dir / "scene_compare.csv",
            [
                "scene_name",
                "difficulty",
                "baseline_mean_ms",
                "compare_mean_ms",
                "delta_mean_ms",
                "baseline_p75_ms",
                "compare_p75_ms",
                "delta_p75_ms",
                "baseline_attempt_rate",
                "compare_attempt_rate",
                "delta_attempt_rate",
                "baseline_direct_fallback_rate",
                "compare_direct_fallback_rate",
                "delta_direct_fallback_rate",
                "effect",
            ],
            comparison_rows,
        )

    recommended_scenes = recommended_subset(baseline_summaries, args.slow_threshold_ms)
    (output_dir / "recommended_subset.txt").write_text(
        "\n".join(recommended_scenes) + ("\n" if recommended_scenes else ""),
        encoding="utf-8",
    )
    with open(output_dir / "analysis_metadata.json", "w", encoding="utf-8") as handle:
        json.dump(
            {
                "baseline_results": str(baseline_path),
                "compare_results": str(compare_path) if compare_path else "",
                "planner": args.planner,
                "mode": args.mode,
                "slow_threshold_ms": args.slow_threshold_ms,
                "recommended_scenes": recommended_scenes,
            },
            handle,
            ensure_ascii=False,
            indent=2,
        )

    print(f"baseline_results={baseline_path}")
    if compare_path:
        print(f"compare_results={compare_path}")
    for item in baseline_summaries:
        print(
            f"scene={item['scene_name']} difficulty={item['difficulty']} "
            f"mean_ms={item['mean_ms']:.1f} p75_ms={item['p75_ms']:.1f} "
            f"slow_rate={item['slow_rate']:.2f} direct_fallback_rate={item['direct_fallback_rate']:.2f} "
            f"guide_attempt_rate={item['guide_attempt_rate']:.2f} "
            f"class={item['scene_classification']} score={item['rescue_priority_score']:.3f}"
        )
    if comparison_rows:
        for item in comparison_rows:
            print(
                f"compare scene={item['scene_name']} effect={item['effect']} "
                f"delta_mean_ms={item['delta_mean_ms']:.1f} "
                f"delta_p75_ms={item['delta_p75_ms']:.1f} "
                f"delta_attempt_rate={item['delta_attempt_rate']:.2f}"
            )
    print(f"recommended_subset={','.join(recommended_scenes) if recommended_scenes else '(none)'}")
    print(f"output_dir={output_dir}")


if __name__ == "__main__":
    main()
