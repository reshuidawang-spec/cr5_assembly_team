#!/usr/bin/env python3
"""Oracle planner model trainer — trains a multi-class classifier to predict the best planner for each query, using oracle labels from exhaustive benchmark runs."""

import argparse
from pathlib import Path

import numpy as np

from model_pipeline_common import (
    ORACLE_MODEL_ROOT,
    balanced_class_weights,
    default_prepared_metadata_path,
    fit_feature_encoder,
    fit_softmax_regression,
    load_json,
    multiclass_metrics,
    predict_multiclass_scores,
    read_csv_rows,
    relative_to_repo,
    save_json,
    session_stamp,
    split_group_ids,
    transform_rows,
)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Train a direct oracle planner classifier on the task-level simple random dataset."
    )
    parser.add_argument(
        "--dataset",
        required=True,
        help="Oracle planner dataset produced by prepare_oracle_planner_dataset.py",
    )
    parser.add_argument(
        "--metadata",
        default="",
        help="Optional dataset metadata json. Default: <dataset>_metadata.json",
    )
    parser.add_argument(
        "--group-column",
        default="task_uid",
        help="Group column for the train/test split. Default: task_uid",
    )
    parser.add_argument(
        "--test-ratio",
        type=float,
        default=0.2,
        help="Fraction of task rows reserved for test evaluation.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=20260317,
        help="Random seed for split and initialization.",
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
        default=3000,
        help="Training epochs.",
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
        help="Optional output directory. Default: test_results/models/simple_random_oracle_planner/<timestamp>/",
    )
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    dataset_path = Path(args.dataset)
    metadata_path = Path(args.metadata) if args.metadata else default_prepared_metadata_path(dataset_path)
    dataset_metadata = load_json(metadata_path)
    rows = read_csv_rows(dataset_path)
    if not rows:
        raise RuntimeError(f"No rows found in {dataset_path}")

    class_names = list(dataset_metadata["candidate_planners"])
    class_index = {name: idx for idx, name in enumerate(class_names)}
    labels = np.asarray([class_index[row["oracle_planner"]] for row in rows], dtype=int)
    group_ids = [row[args.group_column] for row in rows]

    train_groups, test_groups = split_group_ids(group_ids, args.test_ratio, args.seed)
    train_rows = [row for row in rows if row[args.group_column] in train_groups]
    test_rows = [row for row in rows if row[args.group_column] in test_groups]
    train_labels = np.asarray([class_index[row["oracle_planner"]] for row in train_rows], dtype=int)
    test_labels = np.asarray([class_index[row["oracle_planner"]] for row in test_rows], dtype=int)

    encoder = fit_feature_encoder(
        train_rows,
        dataset_metadata["numeric_feature_columns"],
        dataset_metadata["categorical_feature_columns"],
    )
    train_features = transform_rows(train_rows, encoder)
    test_features = transform_rows(test_rows, encoder)
    class_weights = balanced_class_weights(train_labels, class_names)

    weights, bias = fit_softmax_regression(
        train_features,
        train_labels,
        num_classes=len(class_names),
        learning_rate=args.learning_rate,
        epochs=args.epochs,
        l2_strength=args.l2,
        seed=args.seed,
        class_weights=class_weights,
    )

    train_probabilities = predict_multiclass_scores(train_features, weights, bias)
    test_probabilities = predict_multiclass_scores(test_features, weights, bias)
    train_metrics = multiclass_metrics(train_labels, train_probabilities, class_names)
    test_metrics = multiclass_metrics(test_labels, test_probabilities, class_names)

    run_dir = Path(args.output_dir) if args.output_dir else ORACLE_MODEL_ROOT / session_stamp()
    run_dir.mkdir(parents=True, exist_ok=True)

    np.savez(run_dir / "weights.npz", weights=weights, bias=bias)
    save_json(
        run_dir / "run_metadata.json",
        {
            "dataset": relative_to_repo(dataset_path),
            "dataset_metadata": relative_to_repo(metadata_path),
            "group_column": args.group_column,
            "train_groups": sorted(train_groups),
            "test_groups": sorted(test_groups),
            "train_row_count": len(train_rows),
            "test_row_count": len(test_rows),
            "candidate_planners": class_names,
            "hyperparameters": {
                "learning_rate": args.learning_rate,
                "epochs": args.epochs,
                "l2": args.l2,
                "seed": args.seed,
            },
        },
    )
    save_json(
        run_dir / "model_metadata.json",
        {
            "model_type": "numpy_softmax_regression",
            "class_names": class_names,
            "encoder": encoder,
            "weights_file": "weights.npz",
            "train_metrics": train_metrics,
            "test_metrics": test_metrics,
        },
    )

    print(
        f"test_accuracy={test_metrics['accuracy']:.3f} "
        f"test_macro_f1={test_metrics['macro_f1']:.3f} "
        f"test_balanced_accuracy={test_metrics['balanced_accuracy']:.3f}"
    )
    print(f"run_dir={run_dir}")


if __name__ == "__main__":
    main()
