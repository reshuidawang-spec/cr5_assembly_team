#!/usr/bin/env python3
"""Oracle planner model evaluator — evaluates the oracle (best-possible) planner selector against ground-truth optimal planner choices to establish an upper performance bound."""

import argparse
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from model_pipeline_common import (
    ORACLE_MODEL_ROOT,
    default_prepared_metadata_path,
    latest_model_run,
    load_json,
    multiclass_metrics,
    predict_multiclass_scores,
    read_csv_rows,
    save_json,
    transform_rows,
    write_csv,
)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Evaluate a trained oracle planner classifier and compare selection quality against fixed baselines."
    )
    parser.add_argument(
        "--run-dir",
        default="",
        help="Model run directory. Default: latest under test_results/models/simple_random_oracle_planner/",
    )
    parser.add_argument(
        "--dataset",
        default="",
        help="Optional oracle dataset override. Default: value from run_metadata.json",
    )
    parser.add_argument(
        "--metadata",
        default="",
        help="Optional dataset metadata override. Default: value from run_metadata.json",
    )
    return parser.parse_args()


def resolve_path(run_dir: Path, value: str):
    """Resolve path."""
    path = Path(value)
    if path.is_absolute():
        return path
    return run_dir.parents[3] / path


def metric_value(row, planner: str, metric_name: str):
    """Metric value."""
    return row[f"planner_metric__{planner}__{metric_name}"]


def summarize_selection(rows, planner_lookup):
    """Summarize selection."""
    selected_rows = []
    for row in rows:
        planner = planner_lookup(row)
        selected_rows.append(
            {
                "task_uid": row["task_uid"],
                "task_family": row["task_family"],
                "difficulty": row["difficulty"],
                "planner": planner,
                "success": metric_value(row, planner, "success"),
                "hit_budget_limit": metric_value(row, planner, "hit_budget_limit"),
                "fast_solve_lt_1s": metric_value(row, planner, "fast_solve_lt_1s"),
                "wall_time_ms": metric_value(row, planner, "wall_time_ms"),
                "moveit_time_ms": metric_value(row, planner, "moveit_time_ms"),
                "budget_headroom_ms": metric_value(row, planner, "budget_headroom_ms"),
            }
        )

    success = np.asarray([int(row["success"]) for row in selected_rows], dtype=float)
    budget = np.asarray([int(row["hit_budget_limit"]) for row in selected_rows], dtype=float)
    fast = np.asarray([int(row["fast_solve_lt_1s"]) for row in selected_rows], dtype=float)
    wall = np.asarray([float(row["wall_time_ms"]) for row in selected_rows], dtype=float)
    moveit = np.asarray([float(row["moveit_time_ms"]) for row in selected_rows], dtype=float)

    return {
        "sample_count": len(selected_rows),
        "success_rate": float(success.mean()) if len(success) else 0.0,
        "budget_hit_rate": float(budget.mean()) if len(budget) else 0.0,
        "fast_solve_rate": float(fast.mean()) if len(fast) else 0.0,
        "mean_wall_time_ms": float(wall.mean()) if len(wall) else 0.0,
        "median_wall_time_ms": float(np.median(wall)) if len(wall) else 0.0,
        "mean_moveit_time_ms": float(moveit.mean()) if len(moveit) else 0.0,
        "median_moveit_time_ms": float(np.median(moveit)) if len(moveit) else 0.0,
        "rows": selected_rows,
    }


def family_summary(policy_name: str, selected_rows):
    """Family summary."""
    grouped = defaultdict(list)
    for row in selected_rows:
        grouped[row["task_family"]].append(row)
    rows = []
    for family in sorted(grouped):
        subset = grouped[family]
        success = np.asarray([int(row["success"]) for row in subset], dtype=float)
        budget = np.asarray([int(row["hit_budget_limit"]) for row in subset], dtype=float)
        fast = np.asarray([int(row["fast_solve_lt_1s"]) for row in subset], dtype=float)
        wall = np.asarray([float(row["wall_time_ms"]) for row in subset], dtype=float)
        rows.append(
            {
                "policy": policy_name,
                "task_family": family,
                "sample_count": len(subset),
                "success_rate": float(success.mean()) if len(success) else 0.0,
                "budget_hit_rate": float(budget.mean()) if len(budget) else 0.0,
                "fast_solve_rate": float(fast.mean()) if len(fast) else 0.0,
                "mean_wall_time_ms": float(wall.mean()) if len(wall) else 0.0,
                "median_wall_time_ms": float(np.median(wall)) if len(wall) else 0.0,
            }
        )
    return rows


def main():
    """Main."""
    args = parse_args()
    run_dir = Path(args.run_dir) if args.run_dir else latest_model_run(ORACLE_MODEL_ROOT)
    run_metadata = load_json(run_dir / "run_metadata.json")
    model_metadata = load_json(run_dir / "model_metadata.json")

    dataset_path = resolve_path(run_dir, args.dataset or run_metadata["dataset"])
    metadata_path = resolve_path(run_dir, args.metadata or run_metadata["dataset_metadata"])
    dataset_metadata = load_json(metadata_path)
    rows = read_csv_rows(dataset_path)
    class_names = list(model_metadata["class_names"])
    class_index = {name: idx for idx, name in enumerate(class_names)}

    test_groups = set(run_metadata["test_groups"])
    test_rows = [row for row in rows if row[run_metadata["group_column"]] in test_groups]
    if not test_rows:
        raise RuntimeError(f"No test rows found in {dataset_path}")

    weights_data = np.load(run_dir / model_metadata["weights_file"])
    features = transform_rows(test_rows, model_metadata["encoder"])
    probabilities = predict_multiclass_scores(features, weights_data["weights"], weights_data["bias"])
    labels = np.asarray([class_index[row["oracle_planner"]] for row in test_rows], dtype=int)
    metrics = multiclass_metrics(labels, probabilities, class_names)
    predictions = metrics["predictions"]

    selected_task_rows = []
    for row, predicted_index, probs in zip(test_rows, predictions, probabilities):
        selected_task_rows.append(
            {
                "task_uid": row["task_uid"],
                "task_family": row["task_family"],
                "difficulty": row["difficulty"],
                "oracle_planner": row["oracle_planner"],
                "predicted_planner": class_names[predicted_index],
                **{f"score__{planner}": f"{float(probs[class_index[planner]]):.6f}" for planner in class_names},
            }
        )

    summary_rows = []
    family_rows = []
    planner_mix_rows = []
    policy_selected_rows = []

    policies = {
        "predicted_oracle_planner": lambda row: row["predicted_planner"],
        "oracle_planner": lambda row: row["oracle_planner"],
    }
    for planner in class_names:
        policies[f"fixed:{planner}"] = lambda row, planner=planner: planner

    prediction_lookup = {row["task_uid"]: row["predicted_planner"] for row in selected_task_rows}
    label_lookup = {row["task_uid"]: row["oracle_planner"] for row in selected_task_rows}

    for policy_name, selector in policies.items():
        if policy_name == "predicted_oracle_planner":
            summary = summarize_selection(test_rows, lambda row: prediction_lookup[row["task_uid"]])
        elif policy_name == "oracle_planner":
            summary = summarize_selection(test_rows, lambda row: label_lookup[row["task_uid"]])
        else:
            summary = summarize_selection(test_rows, selector)
        summary_rows.append({"policy": policy_name, **{k: v for k, v in summary.items() if k != "rows"}})
        family_rows.extend(family_summary(policy_name, summary["rows"]))
        mix = Counter(row["planner"] for row in summary["rows"])
        total = sum(mix.values())
        planner_mix_rows.extend(
            {
                "policy": policy_name,
                "planner": planner,
                "count": count,
                "rate": count / total if total else 0.0,
            }
            for planner, count in sorted(mix.items())
        )
        for row in summary["rows"]:
            policy_selected_rows.append({"policy": policy_name, **row})

    per_class_rows = model_metadata["test_metrics"]["per_class"]
    confusion_rows = []
    confusion = metrics["confusion_matrix"]
    for true_index, true_name in enumerate(class_names):
        row = {"true_class": true_name}
        for pred_index, pred_name in enumerate(class_names):
            row[f"pred_{pred_name}"] = confusion[true_index][pred_index]
        confusion_rows.append(row)

    write_csv(
        run_dir / "per_class_metrics.csv",
        ["class_name", "support", "predicted_count", "precision", "recall", "f1"],
        per_class_rows,
    )
    write_csv(
        run_dir / "confusion_matrix.csv",
        ["true_class", *[f"pred_{planner}" for planner in class_names]],
        confusion_rows,
    )
    write_csv(
        run_dir / "test_predictions.csv",
        ["task_uid", "task_family", "difficulty", "oracle_planner", "predicted_planner", *[f"score__{planner}" for planner in class_names]],
        selected_task_rows,
    )
    write_csv(
        run_dir / "selection_policy_summary.csv",
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
        run_dir / "selection_by_task_family.csv",
        [
            "policy",
            "task_family",
            "sample_count",
            "success_rate",
            "budget_hit_rate",
            "fast_solve_rate",
            "mean_wall_time_ms",
            "median_wall_time_ms",
        ],
        family_rows,
    )
    write_csv(
        run_dir / "selection_planner_mix.csv",
        ["policy", "planner", "count", "rate"],
        planner_mix_rows,
    )
    write_csv(
        run_dir / "selection_selected_rows.csv",
        [
            "policy",
            "task_uid",
            "task_family",
            "difficulty",
            "planner",
            "success",
            "hit_budget_limit",
            "fast_solve_lt_1s",
            "wall_time_ms",
            "moveit_time_ms",
            "budget_headroom_ms",
        ],
        policy_selected_rows,
    )

    summary_payload = {
        "multiclass_metrics": {
            "accuracy": metrics["accuracy"],
            "balanced_accuracy": metrics["balanced_accuracy"],
            "macro_f1": metrics["macro_f1"],
        },
        "selection": {
            row["policy"]: {
                key: value
                for key, value in row.items()
                if key != "policy"
            }
            for row in summary_rows
        },
    }
    save_json(run_dir / "evaluation_summary.json", summary_payload)

    predicted = summary_payload["selection"]["predicted_oracle_planner"]
    best_fixed = min(
        (row for row in summary_rows if row["policy"].startswith("fixed:")),
        key=lambda row: row["budget_hit_rate"],
    )
    print(
        f"accuracy={metrics['accuracy']:.3f} "
        f"macro_f1={metrics['macro_f1']:.3f} "
        f"balanced_accuracy={metrics['balanced_accuracy']:.3f}"
    )
    print(
        f"predicted_oracle_planner: budget_hit_rate={predicted['budget_hit_rate']:.3f} "
        f"success_rate={predicted['success_rate']:.3f} "
        f"mean_wall_time_ms={predicted['mean_wall_time_ms']:.1f}"
    )
    print(
        f"best_fixed: {best_fixed['policy']} budget_hit_rate={best_fixed['budget_hit_rate']:.3f} "
        f"success_rate={best_fixed['success_rate']:.3f} "
        f"mean_wall_time_ms={best_fixed['mean_wall_time_ms']:.1f}"
    )
    print(f"run_dir={run_dir}")


if __name__ == "__main__":
    main()
