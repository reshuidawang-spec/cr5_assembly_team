#!/usr/bin/env python3
"""Baseline model evaluator — computes classification metrics (accuracy, ROC-AUC, F1) for the logistic-regression baseline planner-selection model on held-out test data."""

import argparse
from pathlib import Path

import numpy as np

from model_pipeline_common import (
    DEFAULT_GROUP_COLUMN,
    MODEL_ROOT,
    PREDICTION_METADATA_COLUMNS,
    classification_metrics,
    latest_model_run,
    load_json,
    parse_binary_label,
    predict_scores,
    read_csv_rows,
    save_json,
    slice_metrics,
    transform_rows,
    write_csv,
)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Evaluate a trained simple random baseline model run and export detailed test-set reports."
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
        "--targets",
        nargs="*",
        default=[],
        help="Optional subset of targets to evaluate. Default: all targets found in the run directory.",
    )
    parser.add_argument(
        "--group-column",
        default="",
        help="Optional group column override. Default: value stored in the model metadata.",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.5,
        help="Decision threshold used to derive binary predictions from probabilities.",
    )
    return parser.parse_args()


def predictions_rows(rows, target_column, scores, threshold):
    """Predictions rows."""
    output_rows = []
    for row, score in zip(rows, scores):
        output_row = {column: row.get(column, "") for column in PREDICTION_METADATA_COLUMNS}
        output_row.update(
            {
                "true_label": row.get(target_column, ""),
                "predicted_label": "1" if score >= threshold else "0",
                "score": f"{float(score):.6f}",
            }
        )
        output_rows.append(output_row)
    return output_rows


def main():
    """Main."""
    args = parse_args()
    run_dir = Path(args.run_dir) if args.run_dir else latest_model_run(MODEL_ROOT)
    run_metadata = load_json(run_dir / "run_metadata.json")
    prepared_table_path = Path(args.prepared_table) if args.prepared_table else Path(run_metadata["prepared_table"])
    if not prepared_table_path.is_absolute():
        prepared_table_path = run_dir.parents[3] / prepared_table_path

    rows = read_csv_rows(prepared_table_path)
    targets = list(args.targets) if args.targets else list(run_metadata["targets"])

    for target in targets:
        target_dir = run_dir / target
        model_metadata = load_json(target_dir / "model_metadata.json")
        weights_data = np.load(target_dir / model_metadata["weights_file"])
        weights = weights_data["weights"]
        bias = float(weights_data["bias"][0])

        group_column = args.group_column or model_metadata.get("group_column", DEFAULT_GROUP_COLUMN)
        test_groups = set(model_metadata["test_groups"])
        test_rows = [row for row in rows if row.get(group_column, "") in test_groups]
        if not test_rows:
            raise RuntimeError(f"No test rows found for target {target} in {prepared_table_path}")

        test_features = transform_rows(test_rows, model_metadata["encoder"])
        test_scores = predict_scores(test_features, weights, bias)
        test_labels = [parse_binary_label(row[target]) for row in test_rows]

        summary = classification_metrics(test_labels, test_scores, threshold=args.threshold)
        save_json(target_dir / "evaluation_summary.json", summary)

        write_csv(
            target_dir / "test_predictions.csv",
            [*PREDICTION_METADATA_COLUMNS, "true_label", "predicted_label", "score"],
            predictions_rows(test_rows, target, test_scores, args.threshold),
        )
        for slice_column in ("planner", "task_family", "difficulty"):
            slice_rows = slice_metrics(test_rows, test_scores, target, slice_column)
            write_csv(
                target_dir / f"evaluation_by_{slice_column}.csv",
                [
                    slice_column,
                    "sample_count",
                    "positive_count",
                    "negative_count",
                    "positive_rate",
                    "predicted_positive_rate",
                    "accuracy",
                    "precision",
                    "recall",
                    "specificity",
                    "balanced_accuracy",
                    "f1",
                    "roc_auc",
                    "log_loss",
                    "true_positive",
                    "true_negative",
                    "false_positive",
                    "false_negative",
                ],
                slice_rows,
            )

        print(
            f"{target}: accuracy={summary['accuracy']:.3f} "
            f"f1={summary['f1']:.3f} auc={summary['roc_auc']:.3f}"
        )

    print(f"run_dir={run_dir}")


if __name__ == "__main__":
    main()
