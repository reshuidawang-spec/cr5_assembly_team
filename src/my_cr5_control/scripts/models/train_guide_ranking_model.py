#!/usr/bin/env python3
"""Guide ranking model trainer — trains a learning-to-rank model that orders candidate heuristics by predicted effectiveness for a given planning query."""

import argparse
from pathlib import Path

import numpy as np

from guide_model_schema import (
    derived_target_names,
    get_feature_profile,
    raw_target_names,
    target_names,
    target_value,
)
from model_pipeline_common import (
    balanced_sample_weights,
    classification_metrics,
    fit_logistic_regression,
    parse_binary_label,
    read_csv_rows,
    save_json,
    session_stamp,
    split_group_ids,
    write_csv,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = REPO_ROOT / "test_results"
DEFAULT_MODEL_ROOT = RESULTS_ROOT / "models" / "guide_ranking_simple"
DEFAULT_DATASET_GLOB = "datasets/guide_ranking_simple/raw/*_guide_ranking_simple_dataset_results.csv"
RAW_TARGET_CHOICES = raw_target_names()
DERIVED_TARGET_CHOICES = derived_target_names()
TARGET_CHOICES = target_names()
FEATURE_PROFILES = {profile_name: get_feature_profile(profile_name) for profile_name in ("geometric", "rescue")}


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Train a first linear learned guide-ranking model from simple guide-candidate dataset."
    )
    parser.add_argument(
        "--dataset",
        default="",
        help="Guide candidate dataset csv. Default: latest under test_results/datasets/guide_ranking_simple/raw/",
    )
    parser.add_argument(
        "--target",
        default="candidate_preferred",
        choices=TARGET_CHOICES,
        help="Binary target used for ranking model training.",
    )
    parser.add_argument(
        "--min-rows",
        type=int,
        default=100,
        help="When --dataset is not given, select the latest dataset with at least this many rows.",
    )
    parser.add_argument(
        "--group-column",
        default="场景UID",
        help="Group column used for train/test split. Default: 场景UID",
    )
    parser.add_argument(
        "--test-ratio",
        type=float,
        default=0.25,
        help="Fraction of groups reserved for test evaluation.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=20260317,
        help="Random seed.",
    )
    parser.add_argument(
        "--learning-rate",
        type=float,
        default=0.1,
        help="Logistic regression learning rate.",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=2500,
        help="Training epochs.",
    )
    parser.add_argument(
        "--l2",
        type=float,
        default=1e-4,
        help="L2 regularization strength.",
    )
    parser.add_argument(
        "--feature-profile",
        default="geometric",
        choices=sorted(FEATURE_PROFILES.keys()),
        help="Named feature profile used when --features is not explicitly provided.",
    )
    parser.add_argument(
        "--features",
        nargs="*",
        default=None,
        help="Explicit feature columns used by the guide ranker. Overrides --feature-profile.",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="Optional run directory. Default: test_results/models/guide_ranking_simple/<timestamp>/",
    )
    return parser.parse_args()


def csv_row_count(path: Path):
    """Csv row count."""
    with path.open(newline="", encoding="utf-8") as handle:
        return max(sum(1 for _ in handle) - 1, 0)


def latest_dataset(min_rows: int):
    """Latest dataset."""
    matches = sorted(RESULTS_ROOT.glob(DEFAULT_DATASET_GLOB), reverse=True)
    if not matches:
        raise RuntimeError(
            "No guide ranking dataset was found in test_results/datasets/guide_ranking_simple/raw/."
        )
    for path in matches:
        if csv_row_count(path) >= min_rows:
            return path
    raise RuntimeError(
        f"No guide ranking dataset with at least {min_rows} rows was found under "
        "test_results/datasets/guide_ranking_simple/raw/."
    )


def parse_float(value, default=0.0):
    """Parse a string to float, returning a default for empty/missing values."""
    if value in ("", None):
        return default
    return float(value)


def parse_bool(value, default=False):
    """Parse bool."""
    try:
        return parse_binary_label(value) == 1
    except ValueError:
        return default


def planning_budget_ms(row):
    """Planning budget ms."""
    return parse_float(row.get("规划预算(s)", 0.0)) * 1000.0


def feature_value(row, feature_name):
    """Feature value."""
    if feature_name == "difficulty_score_raw":
        return parse_float(row.get("难度评分", 0.0))
    if feature_name == "direct_success_flag":
        return 1.0 if parse_bool(row.get("direct_success", "0")) else 0.0
    if feature_name == "direct_hit_budget_flag":
        return 1.0 if parse_bool(row.get("direct_hit_budget", "0")) else 0.0
    if feature_name == "direct_bad_flag":
        direct_success = parse_bool(row.get("direct_success", "0"))
        direct_hit_budget = parse_bool(row.get("direct_hit_budget", "0"))
        return 1.0 if ((not direct_success) or direct_hit_budget) else 0.0
    if feature_name == "direct_wall_time_ratio":
        budget_ms = planning_budget_ms(row)
        return 0.0 if budget_ms <= 1e-9 else parse_float(row.get("direct_wall_time_ms", 0.0)) / budget_ms
    if feature_name == "direct_moveit_time_ratio":
        budget_ms = planning_budget_ms(row)
        return 0.0 if budget_ms <= 1e-9 else parse_float(row.get("direct_moveit_time_ms", 0.0)) / budget_ms
    return parse_float(row.get(feature_name, 0.0))


def fit_numeric_normalizer(rows, feature_columns):
    """Fit numeric normalizer."""
    means = []
    stds = []
    for column in feature_columns:
        values = np.asarray([feature_value(row, column) for row in rows], dtype=float)
        mean = float(values.mean()) if len(values) else 0.0
        std = float(values.std()) if len(values) else 1.0
        if std < 1e-9:
            std = 1.0
        means.append(mean)
        stds.append(std)
    return means, stds


def transform_numeric_rows(rows, feature_columns, means, stds):
    """Transform numeric rows."""
    matrix = np.zeros((len(rows), len(feature_columns)), dtype=float)
    for feature_index, column in enumerate(feature_columns):
        matrix[:, feature_index] = [
            (feature_value(row, column) - means[feature_index]) / stds[feature_index]
            for row in rows
        ]
    return matrix


def linear_model_csv_rows(feature_columns, means, stds, weights, bias, target_name):
    """Linear model csv rows."""
    rows = [
        {
            "kind": "bias",
            "feature_name": "bias",
            "weight": f"{float(bias):.12f}",
            "mean": "",
            "std": "",
            "target_name": target_name,
        }
    ]
    for feature_name, mean, std, weight in zip(feature_columns, means, stds, weights):
        rows.append(
            {
                "kind": "feature",
                "feature_name": feature_name,
                "weight": f"{float(weight):.12f}",
                "mean": f"{float(mean):.12f}",
                "std": f"{float(std):.12f}",
                "target_name": target_name,
            }
        )
    return rows


def allocate_run_dir(output_dir_arg: str):
    """Allocate run dir."""
    if output_dir_arg:
        return Path(output_dir_arg)
    base_dir = DEFAULT_MODEL_ROOT / session_stamp()
    if not base_dir.exists():
        return base_dir
    suffix = 1
    while True:
        candidate = DEFAULT_MODEL_ROOT / f"{base_dir.name}_{suffix:02d}"
        if not candidate.exists():
            return candidate
        suffix += 1


def main():
    """Main."""
    args = parse_args()
    feature_columns = list(args.features) if args.features is not None else list(FEATURE_PROFILES[args.feature_profile])
    dataset_path = Path(args.dataset) if args.dataset else latest_dataset(args.min_rows)
    rows = read_csv_rows(dataset_path)
    if not rows:
        raise RuntimeError(f"No rows found in {dataset_path}")

    group_ids = [row[args.group_column] for row in rows]
    train_groups, test_groups = split_group_ids(group_ids, args.test_ratio, args.seed)
    train_rows = [row for row in rows if row.get(args.group_column, "") in train_groups]
    test_rows = [row for row in rows if row.get(args.group_column, "") in test_groups]

    train_labels = np.asarray([parse_binary_label(target_value(row, args.target)) for row in train_rows], dtype=float)
    test_labels = np.asarray([parse_binary_label(target_value(row, args.target)) for row in test_rows], dtype=float)
    if len(set(train_labels.tolist())) < 2:
        raise RuntimeError(f"Target {args.target} has only one class in the training split.")

    means, stds = fit_numeric_normalizer(train_rows, feature_columns)
    train_features = transform_numeric_rows(train_rows, feature_columns, means, stds)
    test_features = transform_numeric_rows(test_rows, feature_columns, means, stds)

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

    train_scores = 1.0 / (1.0 + np.exp(-(train_features @ weights + bias)))
    test_scores = 1.0 / (1.0 + np.exp(-(test_features @ weights + bias)))
    train_metrics = classification_metrics(train_labels, train_scores)
    test_metrics = classification_metrics(test_labels, test_scores)

    run_dir = allocate_run_dir(args.output_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    write_csv(
        run_dir / "linear_model.csv",
        ["kind", "feature_name", "weight", "mean", "std", "target_name"],
        linear_model_csv_rows(feature_columns, means, stds, weights, bias, args.target),
    )
    write_csv(
        run_dir / "test_predictions.csv",
        [
            args.group_column,
            "scenario_name",
            "difficulty",
            "guide_index",
            "true_label",
            "predicted_label",
            "score",
        ],
        [
            {
                args.group_column: row.get(args.group_column, ""),
                "scenario_name": row.get("场景名称", row.get("scenario_name", "")),
                "difficulty": row.get("难度", row.get("difficulty", "")),
                "guide_index": row.get("guide_index", ""),
                "true_label": target_value(row, args.target),
                "predicted_label": "1" if score >= 0.5 else "0",
                "score": f"{float(score):.6f}",
            }
            for row, score in zip(test_rows, test_scores)
        ],
    )

    metadata = {
        "dataset": str(dataset_path),
        "target": args.target,
        "feature_profile": args.feature_profile,
        "feature_columns": list(feature_columns),
        "group_column": args.group_column,
        "train_group_count": len(train_groups),
        "test_group_count": len(test_groups),
        "train_row_count": len(train_rows),
        "test_row_count": len(test_rows),
        "train_positive_count": int(train_labels.sum()),
        "test_positive_count": int(test_labels.sum()),
        "train_metrics": train_metrics,
        "test_metrics": test_metrics,
        "hyperparameters": {
            "learning_rate": args.learning_rate,
            "epochs": args.epochs,
            "l2": args.l2,
            "seed": args.seed,
            "test_ratio": args.test_ratio,
            "min_rows": args.min_rows,
        },
    }
    save_json(run_dir / "model_metadata.json", metadata)

    print(
        f"target={args.target} "
        f"test_accuracy={test_metrics['accuracy']:.3f} "
        f"test_f1={test_metrics['f1']:.3f} "
        f"test_auc={test_metrics['roc_auc']:.3f}"
    )
    print(f"run_dir={run_dir}")


if __name__ == "__main__":
    main()
