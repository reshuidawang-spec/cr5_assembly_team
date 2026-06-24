#!/usr/bin/env python3
"""Planner selection evaluator — compares multiple planner-selection strategies (random, baseline, oracle, guide-model) on shared test sets and generates comparison reports."""

import argparse
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from model_pipeline_common import (
    DEFAULT_GROUP_COLUMN,
    MODEL_ROOT,
    classification_metrics,
    latest_model_run,
    load_json,
    parse_binary_label,
    parse_float,
    predict_scores,
    read_csv_rows,
    save_json,
    transform_rows,
    write_csv,
)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Offline planner selection experiment using trained simple random baseline models."
    )
    parser.add_argument(
        "--run-dir",
        default="",
        help="Model run directory. Default: latest under test_results/models/simple_random_baseline/",
    )
    parser.add_argument(
        "--prepared-table",
        default="",
        help="Optional prepared table override. Default: value stored in run_metadata.json",
    )
    parser.add_argument(
        "--group-column",
        default="",
        help="Optional group column override. Default: task_uid from the trained run.",
    )
    parser.add_argument(
        "--budget-target",
        default="hit_budget_limit",
        help="Model target used as the primary budget-risk score.",
    )
    parser.add_argument(
        "--fast-target",
        default="fast_solve_lt_1s",
        help="Optional fast-solve model target used as a tie-break or bonus score.",
    )
    parser.add_argument(
        "--fast-weight",
        type=float,
        default=0.15,
        help="Fast-solve bonus weight in the combined selection score.",
    )
    parser.add_argument(
        "--planners",
        nargs="*",
        default=[],
        help="Optional candidate planner subset. Default: all planners present in the test split.",
    )
    return parser.parse_args()


def resolve_prepared_table(run_dir: Path, override: str):
    """Resolve prepared table."""
    run_metadata = load_json(run_dir / "run_metadata.json")
    prepared_table = Path(override) if override else Path(run_metadata["prepared_table"])
    if not prepared_table.is_absolute():
        prepared_table = run_dir.parents[3] / prepared_table
    return prepared_table


def load_model(run_dir: Path, target: str):
    """Load model."""
    target_dir = run_dir / target
    if not target_dir.exists():
        return None
    metadata = load_json(target_dir / "model_metadata.json")
    weights_data = np.load(target_dir / metadata["weights_file"])
    return {
        "metadata": metadata,
        "weights": weights_data["weights"],
        "bias": float(weights_data["bias"][0]),
    }


def attach_predictions(rows, model_bundle, output_column: str):
    """Attach predictions."""
    features = transform_rows(rows, model_bundle["metadata"]["encoder"])
    scores = predict_scores(features, model_bundle["weights"], model_bundle["bias"])
    for row, score in zip(rows, scores):
        row[output_column] = float(score)


def planner_preference_key(row):
    """Planner preference key."""
    return (
        row.get("planner", ""),
        row.get("planning_mode", ""),
        row.get("planner_id", ""),
    )


def choose_row(policy_name: str, rows, fast_weight: float):
    """Choose row."""
    if policy_name.startswith("fixed:"):
        planner_name = policy_name.split(":", 1)[1]
        for row in rows:
            if row.get("planner") == planner_name:
                return row
        raise RuntimeError(f"Planner {planner_name} was not found for task {rows[0].get('task_uid', '')}")

    if policy_name == "predicted_budget":
        return min(
            rows,
            key=lambda row: (
                row["pred_hit_budget_limit"],
                planner_preference_key(row),
            ),
        )

    if policy_name == "predicted_budget_fast":
        return min(
            rows,
            key=lambda row: (
                row["pred_hit_budget_limit"] - fast_weight * row.get("pred_fast_solve_lt_1s", 0.0),
                row["pred_hit_budget_limit"],
                -row.get("pred_fast_solve_lt_1s", 0.0),
                planner_preference_key(row),
            ),
        )

    if policy_name == "oracle_budget_fast":
        return min(
            rows,
            key=lambda row: (
                parse_binary_label(row["hit_budget_limit"]),
                -parse_binary_label(row["fast_solve_lt_1s"]),
                -parse_binary_label(row["success"]),
                parse_float(row.get("wall_time_ms", "0")),
                planner_preference_key(row),
            ),
        )

    raise RuntimeError(f"Unsupported policy: {policy_name}")


def summarize_policy(selected_rows):
    """Summarize policy."""
    sample_count = len(selected_rows)
    success = [parse_binary_label(row["success"]) for row in selected_rows]
    budget_hits = [parse_binary_label(row["hit_budget_limit"]) for row in selected_rows]
    fast_solves = [parse_binary_label(row["fast_solve_lt_1s"]) for row in selected_rows]
    wall_times = [parse_float(row.get("wall_time_ms", "0")) for row in selected_rows]
    moveit_times = [parse_float(row.get("moveit_time_ms", "0")) for row in selected_rows]

    return {
        "sample_count": sample_count,
        "success_rate": float(np.mean(success)) if success else 0.0,
        "budget_hit_rate": float(np.mean(budget_hits)) if budget_hits else 0.0,
        "fast_solve_rate": float(np.mean(fast_solves)) if fast_solves else 0.0,
        "mean_wall_time_ms": float(np.mean(wall_times)) if wall_times else 0.0,
        "median_wall_time_ms": float(np.median(wall_times)) if wall_times else 0.0,
        "mean_moveit_time_ms": float(np.mean(moveit_times)) if moveit_times else 0.0,
        "median_moveit_time_ms": float(np.median(moveit_times)) if moveit_times else 0.0,
    }


def summarize_by_family(policy_name: str, selected_rows):
    """Summarize by family."""
    grouped = defaultdict(list)
    for row in selected_rows:
        grouped[row.get("task_family", "")].append(row)

    rows = []
    for family in sorted(grouped):
        summary = summarize_policy(grouped[family])
        rows.append({"policy": policy_name, "task_family": family, **summary})
    return rows


def planner_mix_rows(policy_name: str, selected_rows):
    """Planner mix rows."""
    counts = Counter(row.get("planner", "") for row in selected_rows)
    total = sum(counts.values())
    return [
        {
            "policy": policy_name,
            "planner": planner,
            "count": count,
            "rate": count / total if total else 0.0,
        }
        for planner, count in sorted(counts.items())
    ]


def selected_task_rows(policy_name: str, selected_rows):
    """Selected task rows."""
    rows = []
    for row in selected_rows:
        rows.append(
            {
                "policy": policy_name,
                "task_uid": row.get("task_uid", ""),
                "task_index": row.get("task_index", ""),
                "task_family": row.get("task_family", ""),
                "difficulty": row.get("difficulty", ""),
                "planner": row.get("planner", ""),
                "planning_mode": row.get("planning_mode", ""),
                "planner_id": row.get("planner_id", ""),
                "pred_hit_budget_limit": f"{row.get('pred_hit_budget_limit', 0.0):.6f}",
                "pred_fast_solve_lt_1s": f"{row.get('pred_fast_solve_lt_1s', 0.0):.6f}",
                "success": row.get("success", ""),
                "hit_budget_limit": row.get("hit_budget_limit", ""),
                "fast_solve_lt_1s": row.get("fast_solve_lt_1s", ""),
                "wall_time_ms": row.get("wall_time_ms", ""),
                "moveit_time_ms": row.get("moveit_time_ms", ""),
                "budget_headroom_ms": row.get("budget_headroom_ms", ""),
            }
        )
    return rows


def main():
    """Main."""
    args = parse_args()
    run_dir = Path(args.run_dir) if args.run_dir else latest_model_run(MODEL_ROOT)
    prepared_table_path = resolve_prepared_table(run_dir, args.prepared_table)
    rows = read_csv_rows(prepared_table_path)
    if not rows:
        raise RuntimeError(f"No rows found in {prepared_table_path}")

    budget_model = load_model(run_dir, args.budget_target)
    if budget_model is None:
        raise RuntimeError(f"Budget model target {args.budget_target} was not found in {run_dir}")
    fast_model = load_model(run_dir, args.fast_target)

    group_column = args.group_column or budget_model["metadata"].get("group_column", DEFAULT_GROUP_COLUMN)
    test_groups = set(budget_model["metadata"]["test_groups"])
    test_rows = [dict(row) for row in rows if row.get(group_column, "") in test_groups]
    if not test_rows:
        raise RuntimeError(f"No test rows found for group column {group_column}")

    if args.planners:
        allowed_planners = set(args.planners)
        test_rows = [row for row in test_rows if row.get("planner", "") in allowed_planners]
        if not test_rows:
            raise RuntimeError("No test rows remained after applying the planner filter.")

    attach_predictions(test_rows, budget_model, "pred_hit_budget_limit")
    if fast_model is not None:
        attach_predictions(test_rows, fast_model, "pred_fast_solve_lt_1s")
    else:
        for row in test_rows:
            row["pred_fast_solve_lt_1s"] = 0.0

    tasks = defaultdict(list)
    for row in test_rows:
        tasks[row[group_column]].append(row)
    planners = sorted({row.get("planner", "") for row in test_rows})
    if not planners:
        raise RuntimeError("No planners remained after applying the planner filter.")

    policies = [f"fixed:{planner}" for planner in planners]
    policies.extend(["predicted_budget", "predicted_budget_fast", "oracle_budget_fast"])

    planner_tag = "all_planners" if not args.planners else "_".join(planners)
    output_dir = run_dir / "planner_selection" / planner_tag
    output_dir.mkdir(parents=True, exist_ok=True)

    summary_rows = []
    family_rows = []
    mix_rows = []
    task_rows = []
    selected_by_policy = {}

    for policy in policies:
        chosen_rows = [choose_row(policy, task_rows_for_group, args.fast_weight) for task_rows_for_group in tasks.values()]
        selected_by_policy[policy] = chosen_rows
        summary_rows.append({"policy": policy, **summarize_policy(chosen_rows)})
        family_rows.extend(summarize_by_family(policy, chosen_rows))
        mix_rows.extend(planner_mix_rows(policy, chosen_rows))
        task_rows.extend(selected_task_rows(policy, chosen_rows))

    write_csv(
        output_dir / "policy_summary.csv",
        [
            "policy",
            "sample_count",
            "success_rate",
            "budget_hit_rate",
            "fast_solve_rate",
            "mean_wall_time_ms",
            "median_wall_time_ms",
            "mean_moveit_time_ms",
            "median_moveit_time_ms",
        ],
        summary_rows,
    )
    write_csv(
        output_dir / "policy_by_task_family.csv",
        [
            "policy",
            "task_family",
            "sample_count",
            "success_rate",
            "budget_hit_rate",
            "fast_solve_rate",
            "mean_wall_time_ms",
            "median_wall_time_ms",
            "mean_moveit_time_ms",
            "median_moveit_time_ms",
        ],
        family_rows,
    )
    write_csv(
        output_dir / "planner_mix.csv",
        ["policy", "planner", "count", "rate"],
        mix_rows,
    )
    write_csv(
        output_dir / "selected_tasks.csv",
        [
            "policy",
            "task_uid",
            "task_index",
            "task_family",
            "difficulty",
            "planner",
            "planning_mode",
            "planner_id",
            "pred_hit_budget_limit",
            "pred_fast_solve_lt_1s",
            "success",
            "hit_budget_limit",
            "fast_solve_lt_1s",
            "wall_time_ms",
            "moveit_time_ms",
            "budget_headroom_ms",
        ],
        task_rows,
    )

    summary_payload = {
        "run_dir": str(run_dir),
        "prepared_table": str(prepared_table_path),
        "group_column": group_column,
        "policy_count": len(policies),
        "task_count": len(tasks),
        "fast_weight": args.fast_weight,
        "budget_target": args.budget_target,
        "fast_target": args.fast_target if fast_model is not None else "",
        "candidate_planners": planners,
        "best_fixed_by_budget_hit_rate": min(
            (row for row in summary_rows if row["policy"].startswith("fixed:")),
            key=lambda row: row["budget_hit_rate"],
        ),
        "predicted_budget": next(row for row in summary_rows if row["policy"] == "predicted_budget"),
        "predicted_budget_fast": next(row for row in summary_rows if row["policy"] == "predicted_budget_fast"),
        "oracle_budget_fast": next(row for row in summary_rows if row["policy"] == "oracle_budget_fast"),
    }
    save_json(output_dir / "selection_summary.json", summary_payload)

    for key in ("predicted_budget", "predicted_budget_fast"):
        row = summary_payload[key]
        print(
            f"{key}: budget_hit_rate={row['budget_hit_rate']:.3f} "
            f"success_rate={row['success_rate']:.3f} "
            f"fast_solve_rate={row['fast_solve_rate']:.3f} "
            f"mean_wall_time_ms={row['mean_wall_time_ms']:.1f}"
        )
    best_fixed = summary_payload["best_fixed_by_budget_hit_rate"]
    print(
        f"best_fixed: {best_fixed['policy']} budget_hit_rate={best_fixed['budget_hit_rate']:.3f} "
        f"success_rate={best_fixed['success_rate']:.3f} mean_wall_time_ms={best_fixed['mean_wall_time_ms']:.1f}"
    )
    print(f"output_dir={output_dir}")


if __name__ == "__main__":
    main()
