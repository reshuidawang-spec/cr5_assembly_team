#!/usr/bin/env python3
"""Guide model ablation trace analyzer — measures the marginal contribution of each guide feature by selectively disabling them and re-evaluating planner performance."""

import argparse
from pathlib import Path

from model_pipeline_common import (
    RESULTS_ROOT,
    parse_binary_label,
    parse_float,
    read_csv_rows,
    save_json,
    session_stamp,
    write_csv,
)


DEFAULT_RESULTS_GLOB = "benchmarks/simple_guidance/raw/*_learned_guidance_simple_ablation_results.csv"


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Analyze guide ablation trace outputs and export attempted-guide subsets."
    )
    parser.add_argument(
        "--results",
        default="",
        help="Ablation results csv. Default: latest under test_results/benchmarks/simple_guidance/raw/.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Optional export directory. Default: test_results/exports/guide_ablation_trace/<timestamp>/",
    )
    return parser.parse_args()


def latest_results():
    """Latest results."""
    matches = sorted(RESULTS_ROOT.glob(DEFAULT_RESULTS_GLOB))
    if not matches:
        raise RuntimeError(
            "No guide ablation result csv was found in test_results/benchmarks/simple_guidance/raw/."
        )
    return matches[-1]


def parse_bool(value):
    """Parse bool."""
    try:
        return parse_binary_label(value) == 1
    except ValueError:
        return False


def build_mode_summary(rows, mode_name):
    """Build mode summary."""
    mode_rows = [row for row in rows if row.get("模式", "") == mode_name]
    sample_count = len(mode_rows)
    attempted_rows = [row for row in mode_rows if int(row.get("guide_candidates_attempted", "0") or 0) > 0]
    direct_rows = [row for row in mode_rows if parse_bool(row.get("used_direct_plan", "否"))]
    reused_rows = [row for row in mode_rows if parse_bool(row.get("reused_direct_baseline", "否"))]
    budget_rows = [row for row in mode_rows if parse_bool(row.get("触发预算上限", "否"))]
    success_rows = [row for row in mode_rows if row.get("成功", "") == "成功"]
    wall_times = [parse_float(row.get("墙钟时间(ms)", 0.0)) for row in mode_rows]
    mean_wall_time = sum(wall_times) / sample_count if sample_count else 0.0
    return {
        "mode": mode_name,
        "sample_count": sample_count,
        "attempted_count": len(attempted_rows),
        "attempted_rate": 0.0 if sample_count == 0 else len(attempted_rows) / sample_count,
        "direct_fallback_count": len(direct_rows),
        "direct_fallback_rate": 0.0 if sample_count == 0 else len(direct_rows) / sample_count,
        "reused_direct_count": len(reused_rows),
        "budget_hit_count": len(budget_rows),
        "success_count": len(success_rows),
        "mean_wall_time_ms": mean_wall_time,
    }


def attempted_rows_export(rows):
    """Attempted rows export."""
    exported = []
    for row in rows:
        attempted = int(row.get("guide_candidates_attempted", "0") or 0)
        if attempted <= 0:
            continue
        exported.append(
            {
                "实验时间戳": row.get("实验时间戳", ""),
                "重复序号": row.get("重复序号", ""),
                "模式": row.get("模式", ""),
                "ranking_name": row.get("ranking_name", ""),
                "场景名称": row.get("场景名称", ""),
                "难度": row.get("难度", ""),
                "成功": row.get("成功", ""),
                "墙钟时间(ms)": row.get("墙钟时间(ms)", ""),
                "触发预算上限": row.get("触发预算上限", ""),
                "reused_direct_baseline": row.get("reused_direct_baseline", ""),
                "guide_candidate_count": row.get("guide_candidate_count", ""),
                "guide_candidates_attempted": row.get("guide_candidates_attempted", ""),
                "used_direct_plan": row.get("used_direct_plan", ""),
                "selected_candidate_id": row.get("selected_candidate_id", ""),
                "selected_candidate_probability": row.get("selected_candidate_probability", ""),
                "selected_candidate_heuristic_cost": row.get("selected_candidate_heuristic_cost", ""),
                "selected_candidate_ranking_score": row.get("selected_candidate_ranking_score", ""),
                "selected_guide_x": row.get("selected_guide_x", ""),
                "selected_guide_y": row.get("selected_guide_y", ""),
                "selected_guide_z": row.get("selected_guide_z", ""),
            }
        )
    return exported


def main():
    """Main."""
    args = parse_args()
    results_path = Path(args.results) if args.results else latest_results()
    rows = read_csv_rows(results_path)
    if not rows:
        raise RuntimeError(f"No rows found in {results_path}")

    run_dir = (
        Path(args.output_dir)
        if args.output_dir
        else RESULTS_ROOT / "exports" / "guide_ablation_trace" / session_stamp()
    )
    run_dir.mkdir(parents=True, exist_ok=True)

    mode_names = []
    for row in rows:
        mode = row.get("模式", "")
        if mode and mode not in mode_names:
            mode_names.append(mode)

    summary_rows = [build_mode_summary(rows, mode_name) for mode_name in mode_names]
    attempted_export_rows = attempted_rows_export(rows)

    write_csv(
        run_dir / "trace_summary.csv",
        [
            "mode",
            "sample_count",
            "attempted_count",
            "attempted_rate",
            "direct_fallback_count",
            "direct_fallback_rate",
            "reused_direct_count",
            "budget_hit_count",
            "success_count",
            "mean_wall_time_ms",
        ],
        summary_rows,
    )
    write_csv(
        run_dir / "attempted_only.csv",
        [
            "实验时间戳",
            "重复序号",
            "模式",
            "ranking_name",
            "场景名称",
            "难度",
            "成功",
            "墙钟时间(ms)",
            "触发预算上限",
            "reused_direct_baseline",
            "guide_candidate_count",
            "guide_candidates_attempted",
            "used_direct_plan",
            "selected_candidate_id",
            "selected_candidate_probability",
            "selected_candidate_heuristic_cost",
            "selected_candidate_ranking_score",
            "selected_guide_x",
            "selected_guide_y",
            "selected_guide_z",
        ],
        attempted_export_rows,
    )
    save_json(
        run_dir / "trace_metadata.json",
        {
            "results_path": str(results_path),
            "row_count": len(rows),
            "mode_summaries": summary_rows,
            "attempted_row_count": len(attempted_export_rows),
        },
    )

    print(f"results={results_path}")
    for row in summary_rows:
        print(
            f"mode={row['mode']} "
            f"samples={row['sample_count']} "
            f"attempted={row['attempted_count']} "
            f"direct_fallback={row['direct_fallback_count']} "
            f"budget_hit={row['budget_hit_count']} "
            f"mean_ms={row['mean_wall_time_ms']:.1f}"
        )
    print(f"attempted_rows={len(attempted_export_rows)}")
    print(f"output_dir={run_dir}")


if __name__ == "__main__":
    main()
