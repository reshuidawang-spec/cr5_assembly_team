#!/usr/bin/env python3
"""Analyze heuristic gate activity — post-processes planner logs to measure how often each heuristic gate activated and whether it improved or degraded planning outcomes."""

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
            "Analyze HeuristicGuided raw benchmark csv and summarize whether the current "
            "gate behaves like active guidance, dormant near-tie fallback, or direct-first."
        )
    )
    parser.add_argument(
        "--results",
        default="",
        help="Raw benchmark results csv. Default: latest simple raw results csv.",
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
        "--direct-slow-threshold-ms",
        type=float,
        default=800.0,
        help="Threshold for flagging slow direct attempts. Default: 800 ms.",
    )
    parser.add_argument(
        "--near-tie-delta",
        type=float,
        default=0.005,
        help="Positive delta threshold below which top guide is treated as near-tie to direct. Default: 0.005.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Optional output directory. Default: test_results/exports/heuristic_gate_activity/<timestamp>/",
    )
    return parser.parse_args()


def latest_results_path():
    """Latest results path."""
    matches = sorted(REPO_ROOT.glob(DEFAULT_RESULTS_GLOB))
    if not matches:
        raise RuntimeError("No simple benchmark raw results csv was found.")
    return matches[-1]


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


def read_rows(path):
    """Read rows."""
    with open(path, "r", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"Results csv is empty: {path}")
    return rows


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


def require_columns(rows):
    """Require columns."""
    required_columns = [
        "direct尝试时间(ms)",
        "direct路径代价",
        "top guide heuristic_cost",
        "top guide ranking_score",
        "guide候选数",
        "guide尝试数",
        "使用direct回退",
        "墙钟时间(ms)",
        "场景名称",
    ]
    missing = [name for name in required_columns if name not in rows[0]]
    if missing:
        raise RuntimeError(
            "Results csv is missing required diagnostic columns: "
            + ", ".join(missing)
            + ". Re-run benchmark with the updated benchmark writers."
        )


def classify_scene(summary, near_tie_delta, direct_slow_threshold_ms):
    """Classify scene."""
    if summary["guide_attempt_rate"] > 0.0:
        return "active_guidance"
    if (summary["direct_fallback_rate"] >= 0.99 and
            summary["mean_delta_h"] <= near_tie_delta and
            summary["direct_slow_rate"] >= 0.25):
        return "dormant_near_tie_long_tail"
    if summary["direct_fallback_rate"] >= 0.99 and summary["mean_delta_h"] <= near_tie_delta:
        return "dormant_near_tie"
    if summary["mean_delta_h"] > near_tie_delta:
        return "guide_clearly_worse_than_direct"
    if summary["direct_slow_rate"] >= 0.25:
        return "direct_long_tail_but_gap_unclear"
    return "mixed"


def tuning_priority_score(summary):
    """Tuning priority score."""
    delta = max(summary["mean_delta_h"], 0.0005)
    return summary["direct_fallback_rate"] * summary["direct_slow_rate"] / delta


def summarize_by_scene(rows, near_tie_delta, direct_slow_threshold_ms):
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
        direct_times = [parse_float(row.get("direct尝试时间(ms)", 0.0)) for row in scene_rows]
        direct_costs = [parse_float(row.get("direct路径代价", -1.0), -1.0) for row in scene_rows]
        top_h_costs = [parse_float(row.get("top guide heuristic_cost", -1.0), -1.0) for row in scene_rows]
        top_r_scores = [parse_float(row.get("top guide ranking_score", -1.0), -1.0) for row in scene_rows]
        candidate_counts = [parse_int(row.get("guide候选数", 0)) for row in scene_rows]
        attempt_counts = [parse_int(row.get("guide尝试数", 0)) for row in scene_rows]
        direct_fallback_flags = [1 if parse_bool(row.get("使用direct回退", "否")) else 0 for row in scene_rows]

        delta_h_values = []
        delta_r_values = []
        for direct_cost, top_h_cost, top_r_score in zip(direct_costs, top_h_costs, top_r_scores):
            if direct_cost >= 0.0 and top_h_cost >= 0.0:
                delta_h_values.append(top_h_cost - direct_cost)
            if direct_cost >= 0.0 and top_r_score >= 0.0:
                delta_r_values.append(top_r_score - direct_cost)

        count = len(scene_rows)
        summary = {
            "scene_name": scene_name,
            "difficulty": scene_rows[0].get("难度", ""),
            "sample_count": count,
            "mean_wall_ms": sum(wall_times) / count if count else 0.0,
            "median_wall_ms": percentile(wall_times, 0.5),
            "p75_wall_ms": percentile(wall_times, 0.75),
            "max_wall_ms": max(wall_times) if wall_times else 0.0,
            "mean_direct_ms": sum(direct_times) / count if count else 0.0,
            "direct_slow_rate": (
                sum(value >= direct_slow_threshold_ms for value in direct_times) / count if count else 0.0
            ),
            "direct_fallback_rate": sum(direct_fallback_flags) / count if count else 0.0,
            "guide_attempt_rate": sum(value > 0 for value in attempt_counts) / count if count else 0.0,
            "avg_guide_attempts": sum(attempt_counts) / count if count else 0.0,
            "avg_guide_candidates": sum(candidate_counts) / count if count else 0.0,
            "mean_direct_cost": sum(direct_costs) / count if count else 0.0,
            "mean_top_guide_heuristic_cost": sum(top_h_costs) / count if count else 0.0,
            "mean_top_guide_ranking_score": sum(top_r_scores) / count if count else 0.0,
            "mean_delta_h": sum(delta_h_values) / len(delta_h_values) if delta_h_values else 0.0,
            "mean_delta_r": sum(delta_r_values) / len(delta_r_values) if delta_r_values else 0.0,
            "min_delta_h": min(delta_h_values) if delta_h_values else 0.0,
            "max_delta_h": max(delta_h_values) if delta_h_values else 0.0,
        }
        summary["scene_classification"] = classify_scene(
            summary, near_tie_delta, direct_slow_threshold_ms
        )
        summary["tuning_priority_score"] = tuning_priority_score(summary)
        summaries.append(summary)

    summaries.sort(
        key=lambda item: (-item["tuning_priority_score"], item["scene_name"])
    )
    return summaries


def summarize_overall(rows, scene_summaries, direct_slow_threshold_ms):
    """Summarize overall."""
    wall_times = [parse_float(row.get("墙钟时间(ms)", 0.0)) for row in rows]
    direct_times = [parse_float(row.get("direct尝试时间(ms)", 0.0)) for row in rows]
    attempt_counts = [parse_int(row.get("guide尝试数", 0)) for row in rows]
    direct_fallback_flags = [1 if parse_bool(row.get("使用direct回退", "否")) else 0 for row in rows]

    overall = {
        "sample_count": len(rows),
        "mean_wall_ms": sum(wall_times) / len(rows) if rows else 0.0,
        "median_wall_ms": percentile(wall_times, 0.5),
        "p75_wall_ms": percentile(wall_times, 0.75),
        "mean_direct_ms": sum(direct_times) / len(rows) if rows else 0.0,
        "direct_slow_rate": (
            sum(value >= direct_slow_threshold_ms for value in direct_times) / len(rows) if rows else 0.0
        ),
        "direct_fallback_rate": sum(direct_fallback_flags) / len(rows) if rows else 0.0,
        "guide_attempt_rate": sum(value > 0 for value in attempt_counts) / len(rows) if rows else 0.0,
        "avg_guide_attempts": sum(attempt_counts) / len(rows) if rows else 0.0,
        "scene_count": len(scene_summaries),
    }
    overall["active_guidance_scene_count"] = sum(
        item["scene_classification"] == "active_guidance" for item in scene_summaries
    )
    overall["dormant_near_tie_scene_count"] = sum(
        item["scene_classification"] in {"dormant_near_tie", "dormant_near_tie_long_tail"}
        for item in scene_summaries
    )
    return overall


def infer_timestamp(results_path):
    """Infer timestamp."""
    stem = Path(results_path).name
    parts = stem.split("_")
    return "_".join(parts[:3]) if len(parts) >= 3 else Path(results_path).stem


def write_scene_summary(path, summaries):
    """Write scene summary."""
    fieldnames = [
        "scene_name",
        "difficulty",
        "sample_count",
        "mean_wall_ms",
        "median_wall_ms",
        "p75_wall_ms",
        "max_wall_ms",
        "mean_direct_ms",
        "direct_slow_rate",
        "direct_fallback_rate",
        "guide_attempt_rate",
        "avg_guide_attempts",
        "avg_guide_candidates",
        "mean_direct_cost",
        "mean_top_guide_heuristic_cost",
        "mean_top_guide_ranking_score",
        "mean_delta_h",
        "mean_delta_r",
        "min_delta_h",
        "max_delta_h",
        "scene_classification",
        "tuning_priority_score",
    ]
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summaries)


def main():
    """Main."""
    args = parse_args()
    results_path = REPO_ROOT / args.results if args.results else latest_results_path()
    rows = read_rows(results_path)
    require_columns(rows)
    filtered_rows = filter_rows(rows, args.planner, args.mode)
    scene_summaries = summarize_by_scene(
        filtered_rows, args.near_tie_delta, args.direct_slow_threshold_ms
    )
    overall = summarize_overall(filtered_rows, scene_summaries, args.direct_slow_threshold_ms)

    if args.output_dir:
        output_dir = REPO_ROOT / args.output_dir
    else:
        timestamp = infer_timestamp(results_path)
        output_dir = REPO_ROOT / "test_results" / "exports" / "heuristic_gate_activity" / timestamp
    output_dir.mkdir(parents=True, exist_ok=True)

    write_scene_summary(output_dir / "scene_summary.csv", scene_summaries)
    with open(output_dir / "overall_summary.json", "w", encoding="utf-8") as handle:
        json.dump(
            {
                "results_path": str(results_path),
                "planner": args.planner,
                "mode": args.mode,
                "direct_slow_threshold_ms": args.direct_slow_threshold_ms,
                "near_tie_delta": args.near_tie_delta,
                "overall": overall,
            },
            handle,
            ensure_ascii=False,
            indent=2,
        )

    print(f"results={results_path}")
    print(
        "overall "
        f"mean_ms={overall['mean_wall_ms']:.1f} "
        f"p75_ms={overall['p75_wall_ms']:.1f} "
        f"guide_attempt_rate={overall['guide_attempt_rate']:.2f} "
        f"direct_fallback_rate={overall['direct_fallback_rate']:.2f} "
        f"direct_slow_rate={overall['direct_slow_rate']:.2f}"
    )
    for item in scene_summaries:
        print(
            f"scene={item['scene_name']} "
            f"difficulty={item['difficulty']} "
            f"class={item['scene_classification']} "
            f"mean_ms={item['mean_wall_ms']:.1f} "
            f"p75_ms={item['p75_wall_ms']:.1f} "
            f"attempt_rate={item['guide_attempt_rate']:.2f} "
            f"direct_fallback_rate={item['direct_fallback_rate']:.2f} "
            f"direct_slow_rate={item['direct_slow_rate']:.2f} "
            f"mean_delta_h={item['mean_delta_h']:.4f} "
            f"score={item['tuning_priority_score']:.2f}"
        )
    print(f"output_dir={output_dir}")


if __name__ == "__main__":
    main()
