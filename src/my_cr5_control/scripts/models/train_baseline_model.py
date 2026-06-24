#!/usr/bin/env python3
"""Baseline model trainer — trains a logistic-regression classifier on benchmark features to predict planning success/failure, with train/test split and metric reporting."""

import argparse
from pathlib import Path

import numpy as np

from model_pipeline_common import (
    DEFAULT_GROUP_COLUMN,
    DEFAULT_PREPARED_TABLE,
    DEFAULT_TARGETS,
    MODEL_ROOT,
    balanced_sample_weights,
    classification_metrics,
    coefficient_summary_rows,
    default_prepared_metadata_path,
    fit_feature_encoder,
    fit_logistic_regression,
    parse_binary_label,
    predict_scores,
    read_csv_rows,
    relative_to_repo,
    resolve_feature_columns,
    rows_for_groups,
    load_json,
    save_json,
    session_stamp,
    split_group_ids,
    transform_rows,
    write_csv,
)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Train numpy-based logistic regression baselines for simple random dataset labels."
    )
    parser.add_argument(
        "--prepared-table",
        default=str(DEFAULT_PREPARED_TABLE),
        help="Prepared training table produced by prepare_training_table.py",
    )
    parser.add_argument(
        "--prepared-metadata",
        default="",
        help="Optional metadata json. Default: <prepared-table>_metadata.json",
    )
    parser.add_argument(
        "--targets",
        nargs="*",
        default=list(DEFAULT_TARGETS),
        choices=["success", "hit_budget_limit", "fast_solve_lt_1s"],
        help="One or more binary targets to train.",
    )
    parser.add_argument(
        "--group-column",
        default=DEFAULT_GROUP_COLUMN,
        help="Group column used for task-level train/test split. Default: task_uid",
    )
    parser.add_argument(
        "--test-ratio",
        type=float,
        default=0.2,
        help="Fraction of task groups reserved for test evaluation.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=20260317,
        help="Random seed for split and weight initialization.",
    )
    parser.add_argument(
        "--learning-rate",
        type=float,
        default=0.1,
        help="Gradient descent learning rate.",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=2500,
        help="Training epochs for each target.",
    )
    parser.add_argument(
        "--l2",
        type=float,
        default=1e-4,
        help="L2 regularization strength.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Optional run directory. Default: test_results/models/simple_random_baseline/<timestamp>/",
    )
    parser.add_argument(
        "--planners",
        nargs="*",
        default=[],
        help="Optional planner subset used for training and evaluation.",
    )
    return parser.parse_args()


def top_features(feature_names, weights, limit=10):
    """Top features."""
    pairs = list(zip(feature_names, weights))
    positive = [
        {"feature_name": name, "weight": float(weight)}
        for name, weight in sorted(pairs, key=lambda item: item[1], reverse=True)[:limit]
    ]
    negative = [
        {"feature_name": name, "weight": float(weight)}
        for name, weight in sorted(pairs, key=lambda item: item[1])[:limit]
    ]
    return {"positive": positive, "negative": negative}


def main():
    """Main."""
    args = parse_args()
    prepared_table_path = Path(args.prepared_table)
    prepared_metadata_path = (
        Path(args.prepared_metadata)
        if args.prepared_metadata
        else default_prepared_metadata_path(prepared_table_path)
    )
    rows = read_csv_rows(prepared_table_path)
    if not rows:
        raise RuntimeError(f"No rows found in {prepared_table_path}")
    prepared_metadata = load_json(prepared_metadata_path) if prepared_metadata_path.exists() else {}
    if args.planners:
        allowed_planners = set(args.planners)
        rows = [row for row in rows if row.get("planner", "") in allowed_planners]
        if not rows:
            raise RuntimeError("No rows remained after applying the planner filter.")
    numeric_feature_columns, categorical_feature_columns = resolve_feature_columns(prepared_metadata)

    group_ids = [row[args.group_column] for row in rows]
    train_groups, test_groups = split_group_ids(group_ids, args.test_ratio, args.seed)
    train_rows = rows_for_groups(rows, args.group_column, train_groups)
    test_rows = rows_for_groups(rows, args.group_column, test_groups)

    run_dir = Path(args.output_dir) if args.output_dir else MODEL_ROOT / session_stamp()
    run_dir.mkdir(parents=True, exist_ok=True)

    run_metadata = {
        "prepared_table": relative_to_repo(prepared_table_path),
        "prepared_metadata": relative_to_repo(prepared_metadata_path),
        "targets": list(args.targets),
        "group_column": args.group_column,
        "train_group_count": len(train_groups),
        "test_group_count": len(test_groups),
        "train_row_count": len(train_rows),
        "test_row_count": len(test_rows),
        "seed": args.seed,
        "test_ratio": args.test_ratio,
        "learning_rate": args.learning_rate,
        "epochs": args.epochs,
        "l2": args.l2,
        "candidate_planners": sorted({row["planner"] for row in rows}),
    }
    save_json(run_dir / "run_metadata.json", run_metadata)

    for target in args.targets:
        train_labels = np.asarray([parse_binary_label(row[target]) for row in train_rows], dtype=float)
        test_labels = np.asarray([parse_binary_label(row[target]) for row in test_rows], dtype=float)
        if len(set(train_labels.tolist())) < 2:
            raise RuntimeError(f"Target {target} has only one class in the training split.")

        encoder = fit_feature_encoder(train_rows, numeric_feature_columns, categorical_feature_columns)
        train_features = transform_rows(train_rows, encoder)
        test_features = transform_rows(test_rows, encoder)
        sample_weights = balanced_sample_weights(train_labels)
        weights, bias = fit_logistic_regression(
            train_features,
            train_labels,
            learning_rate=args.learning_rate,
            epochs=args.epochs,
            l2_strength=args.l2,
            seed=args.seed,
            sample_weights=sample_weights,
        )

        train_scores = predict_scores(train_features, weights, bias)
        test_scores = predict_scores(test_features, weights, bias)

        target_dir = run_dir / target
        target_dir.mkdir(parents=True, exist_ok=True)
        np.savez(target_dir / "weights.npz", weights=weights, bias=np.asarray([bias], dtype=float))
        write_csv(
            target_dir / "coefficient_summary.csv",
            ["direction", "rank", "feature_name", "weight", "abs_weight"],
            coefficient_summary_rows(encoder["encoded_feature_names"], weights),
        )

        model_metadata = {
            "model_type": "numpy_logistic_regression",
            "target": target,
            "prepared_table": relative_to_repo(prepared_table_path),
            "group_column": args.group_column,
            "train_groups": sorted(train_groups),
            "test_groups": sorted(test_groups),
            "encoder": encoder,
            "weights_file": "weights.npz",
            "hyperparameters": {
                "learning_rate": args.learning_rate,
                "epochs": args.epochs,
                "l2": args.l2,
                "seed": args.seed,
            },
            "metrics": {
                "train": classification_metrics(train_labels, train_scores),
                "test": classification_metrics(test_labels, test_scores),
            },
            "top_features": top_features(encoder["encoded_feature_names"], weights),
        }
        save_json(target_dir / "model_metadata.json", model_metadata)

        test_metrics = model_metadata["metrics"]["test"]
        print(
            f"{target}: test_accuracy={test_metrics['accuracy']:.3f} "
            f"test_f1={test_metrics['f1']:.3f} test_auc={test_metrics['roc_auc']:.3f}"
        )

    print(f"run_dir={run_dir}")


if __name__ == "__main__":
    main()
