#!/usr/bin/env python3
"""Model pipeline common utilities — shared data loading, feature encoding, logistic/softmax regression, and metric computation functions used by all training/evaluation scripts."""

import csv
import json
import math
import random
from datetime import datetime
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = REPO_ROOT / "test_results"
EXPORTS_ROOT = RESULTS_ROOT / "exports"
MODEL_ROOT = RESULTS_ROOT / "models" / "simple_random_baseline"
ORACLE_MODEL_ROOT = RESULTS_ROOT / "models" / "simple_random_oracle_planner"

DEFAULT_PREPARED_TABLE = EXPORTS_ROOT / "simple_random_training_table.csv"
DEFAULT_TARGETS = ["hit_budget_limit", "fast_solve_lt_1s"]
DEFAULT_GROUP_COLUMN = "task_uid"

PLANNER_INTERACTION_BASE_FEATURES = [
    "difficulty_score",
    "estimated_clearance_m",
    "local_opening_size_m",
    "depth_ratio",
    "radial_offset_m",
    "lateral_offset_norm_m",
    "approach_distance_m",
    "clearance_to_opening_ratio",
    "clearance_margin_m",
]

NUMERIC_FEATURE_COLUMNS = [
    "tip_x",
    "tip_y",
    "tip_z",
    "normal_x",
    "normal_y",
    "normal_z",
    "flange_x",
    "flange_y",
    "flange_z",
    "box_width_m",
    "box_depth_m",
    "box_height_m",
    "hole_radius_m",
    "hole_depth_m",
    "top_depth_m",
    "depth_ratio",
    "lateral_offset_x_m",
    "lateral_offset_y_m",
    "radial_offset_m",
    "local_opening_size_m",
    "estimated_clearance_m",
    "difficulty_score",
]

CATEGORICAL_FEATURE_COLUMNS = [
    "task_family",
    "difficulty",
    "planner",
    "planning_mode",
]

LABEL_COLUMNS = [
    "success",
    "hit_budget_limit",
    "fast_solve_lt_1s",
]

AUXILIARY_EVAL_COLUMNS = [
    "wall_time_ms",
    "moveit_time_ms",
    "planning_budget_ms",
    "planner_calls",
    "budget_headroom_ms",
]

PREDICTION_METADATA_COLUMNS = [
    "source_results_file",
    "dataset_version",
    "timestamp",
    "random_seed",
    "task_uid",
    "task_index",
    "task_family",
    "difficulty",
    "planner",
    "planning_mode",
    "planner_id",
]

PLANNER_DEPENDENT_CATEGORICAL_COLUMNS = {
    "planner",
    "planning_mode",
    "planner_task_family",
    "planner_difficulty",
}


def session_stamp():
    """Return a timestamp string for naming output files."""
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def default_prepared_metadata_path(table_path: Path) -> Path:
    """Return the default metadata JSON path for a prepared training table."""
    return table_path.with_name(f"{table_path.stem}_metadata.json")


def parse_float(value, default=0.0):
    """Parse a string to float, returning a default for empty/missing values."""
    if value in ("", None):
        return default
    return float(value)


def parse_binary_label(value):
    """Convert a binary label string (1/0/true/false) to an integer."""
    if value in ("1", "是", "成功", "true", "True"):
        return 1
    if value in ("0", "否", "失败", "false", "False"):
        return 0
    raise ValueError(f"Unsupported binary label value: {value!r}")


def read_csv_rows(path: Path):
    """Read CSV file and return rows as a list of dicts."""
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, fieldnames, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def load_json(path: Path):
    """Load a JSON file and return the parsed data."""
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def save_json(path: Path, data):
    """Save data as formatted JSON, creating parent directories as needed."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)


def latest_simple_random_results():
    """Return the most recent simple-random benchmark results CSV path."""
    matches = sorted(
        RESULTS_ROOT.glob("datasets/simple_random/raw/*_simple_random_task_dataset_results.csv")
    )
    if not matches:
        raise RuntimeError(
            "No simple random dataset results were found in test_results/datasets/simple_random/raw/."
        )
    return matches[-1]


def canonical_simple_random_results():
    """Return the canonical (manifest-registered) or latest simple-random results path."""
    manifest_path = RESULTS_ROOT / "dataset_manifest.csv"
    if manifest_path.exists():
        rows = read_csv_rows(manifest_path)
        for row in rows:
            if row.get("dataset_key") in ("simple_random_300", "simple_random_latest_unclassified"):
                result_file = row.get("results_file", "")
                if result_file:
                    path = REPO_ROOT / result_file
                    if path.exists():
                        return path
    return latest_simple_random_results()


def latest_model_run(model_root: Path = MODEL_ROOT):
    """Return the most recent model training run directory."""
    runs = sorted(path for path in model_root.glob("*") if path.is_dir())
    if not runs:
        raise RuntimeError(f"No model runs were found in {model_root}")
    return runs[-1]


def safe_ratio(numerator, denominator, default=0.0, eps=1e-9):
    """Compute a ratio with guard against division by near-zero denominators."""
    denominator_value = parse_float(denominator, default=0.0)
    if abs(denominator_value) < eps:
        return default
    return parse_float(numerator, default=0.0) / denominator_value


def split_group_ids(group_ids, test_ratio: float, seed: int):
    """Split group IDs into train and test sets with a fixed random seed."""
    unique_groups = sorted(set(group_ids))
    if len(unique_groups) < 2:
        raise RuntimeError("Need at least two unique groups to create a train/test split.")

    shuffled = list(unique_groups)
    random.Random(seed).shuffle(shuffled)

    raw_test_count = int(round(len(shuffled) * test_ratio))
    test_count = min(max(1, raw_test_count), len(shuffled) - 1)
    test_groups = set(shuffled[:test_count])
    train_groups = set(shuffled[test_count:])
    return train_groups, test_groups


def rows_for_groups(rows, group_column: str, allowed_groups):
    """Filter rows to those whose group column value is in the allowed set."""
    allowed = set(allowed_groups)
    return [row for row in rows if row.get(group_column, "") in allowed]


def fit_feature_encoder(rows, numeric_feature_columns, categorical_feature_columns):
    """Compute normalization parameters and category mappings from training rows."""
    numeric_means = []
    numeric_stds = []
    for column in numeric_feature_columns:
        values = np.asarray([parse_float(row.get(column, 0.0)) for row in rows], dtype=float)
        mean = float(values.mean()) if len(values) else 0.0
        std = float(values.std()) if len(values) else 1.0
        if std < 1e-9:
            std = 1.0
        numeric_means.append(mean)
        numeric_stds.append(std)

    category_values = {}
    encoded_feature_names = list(numeric_feature_columns)
    for column in categorical_feature_columns:
        values = sorted({row.get(column, "") for row in rows})
        category_values[column] = values
        encoded_feature_names.extend(f"{column}={value}" for value in values)

    return {
        "numeric_feature_columns": list(numeric_feature_columns),
        "categorical_feature_columns": list(categorical_feature_columns),
        "numeric_means": numeric_means,
        "numeric_stds": numeric_stds,
        "category_values": category_values,
        "encoded_feature_names": encoded_feature_names,
    }


def resolve_feature_columns(prepared_metadata):
    """Return numeric and categorical feature column lists from prepared metadata."""
    numeric_columns = prepared_metadata.get("numeric_feature_columns", NUMERIC_FEATURE_COLUMNS)
    categorical_columns = prepared_metadata.get("categorical_feature_columns", CATEGORICAL_FEATURE_COLUMNS)
    return list(numeric_columns), list(categorical_columns)


def resolve_task_only_feature_columns(prepared_metadata):
    """Return feature columns excluding planner-dependent columns for task-only models."""
    numeric_columns, categorical_columns = resolve_feature_columns(prepared_metadata)
    task_numeric_columns = [
        column for column in numeric_columns if not column.startswith("planner_interaction__")
    ]
    task_categorical_columns = [
        column for column in categorical_columns if column not in PLANNER_DEPENDENT_CATEGORICAL_COLUMNS
    ]
    return task_numeric_columns, task_categorical_columns


def transform_rows(rows, encoder):
    """Apply a fitted feature encoder to produce a normalized feature matrix."""
    numeric_columns = encoder["numeric_feature_columns"]
    categorical_columns = encoder["categorical_feature_columns"]
    encoded_feature_names = encoder["encoded_feature_names"]
    numeric_means = encoder["numeric_means"]
    numeric_stds = encoder["numeric_stds"]

    matrix = np.zeros((len(rows), len(encoded_feature_names)), dtype=float)

    for col_index, column in enumerate(numeric_columns):
        mean = numeric_means[col_index]
        std = numeric_stds[col_index]
        matrix[:, col_index] = [
            (parse_float(row.get(column, 0.0)) - mean) / std for row in rows
        ]

    offset = len(numeric_columns)
    for column in categorical_columns:
        mapping = {value: offset + idx for idx, value in enumerate(encoder["category_values"][column])}
        for row_index, row in enumerate(rows):
            encoded_index = mapping.get(row.get(column, ""))
            if encoded_index is not None:
                matrix[row_index, encoded_index] = 1.0
        offset += len(mapping)

    return matrix


def balanced_sample_weights(labels):
    """Compute per-sample weights that balance positive and negative classes."""
    labels = np.asarray(labels, dtype=float)
    positive_count = float(labels.sum())
    negative_count = float(len(labels) - positive_count)
    if positive_count == 0.0 or negative_count == 0.0:
        return np.ones(len(labels), dtype=float)
    positive_weight = len(labels) / (2.0 * positive_count)
    negative_weight = len(labels) / (2.0 * negative_count)
    return np.where(labels > 0.5, positive_weight, negative_weight)


def balanced_class_weights(label_indices, class_names):
    """Compute per-class weights inversely proportional to class frequency."""
    label_indices = np.asarray(label_indices, dtype=int)
    counts = np.bincount(label_indices, minlength=len(class_names)).astype(float)
    total = float(len(label_indices))
    weights = np.ones(len(class_names), dtype=float)
    for class_index, count in enumerate(counts):
        if count > 0.0:
            weights[class_index] = total / (len(class_names) * count)
    return weights


def sigmoid(values):
    """Compute the element-wise sigmoid (logistic) function."""
    values = np.asarray(values, dtype=float)
    positive_mask = values >= 0.0
    negative_mask = ~positive_mask
    output = np.empty_like(values, dtype=float)
    output[positive_mask] = 1.0 / (1.0 + np.exp(-values[positive_mask]))
    exp_values = np.exp(values[negative_mask])
    output[negative_mask] = exp_values / (1.0 + exp_values)
    return output


def softmax(logits):
    """Compute row-wise softmax probabilities."""
    logits = np.asarray(logits, dtype=float)
    shifted = logits - np.max(logits, axis=1, keepdims=True)
    exp_values = np.exp(shifted)
    return exp_values / np.sum(exp_values, axis=1, keepdims=True)


def fit_logistic_regression(
    features,
    labels,
    learning_rate: float,
    epochs: int,
    l2_strength: float,
    seed: int,
    sample_weights,
):
    """Train a binary logistic regression model via gradient descent."""
    labels = np.asarray(labels, dtype=float)
    sample_weights = np.asarray(sample_weights, dtype=float)
    sample_weights = sample_weights / max(sample_weights.mean(), 1e-9)

    rng = np.random.default_rng(seed)
    weights = rng.normal(loc=0.0, scale=0.01, size=features.shape[1])
    bias = 0.0

    denominator = float(sample_weights.sum())
    for _ in range(epochs):
        logits = features @ weights + bias
        probabilities = sigmoid(logits)
        weighted_error = (probabilities - labels) * sample_weights
        grad_weights = (features.T @ weighted_error) / denominator + l2_strength * weights
        grad_bias = float(weighted_error.sum() / denominator)
        weights -= learning_rate * grad_weights
        bias -= learning_rate * grad_bias

    return weights, bias


def fit_softmax_regression(
    features,
    labels,
    num_classes: int,
    learning_rate: float,
    epochs: int,
    l2_strength: float,
    seed: int,
    class_weights,
):
    """Train a multi-class softmax regression model via gradient descent."""
    labels = np.asarray(labels, dtype=int)
    class_weights = np.asarray(class_weights, dtype=float)
    sample_weights = class_weights[labels]
    sample_weights = sample_weights / max(sample_weights.mean(), 1e-9)

    rng = np.random.default_rng(seed)
    weights = rng.normal(loc=0.0, scale=0.01, size=(features.shape[1], num_classes))
    bias = np.zeros(num_classes, dtype=float)

    denominator = float(sample_weights.sum())
    one_hot = np.eye(num_classes, dtype=float)[labels]

    for _ in range(epochs):
        logits = features @ weights + bias
        probabilities = softmax(logits)
        error = (probabilities - one_hot) * sample_weights[:, None]
        grad_weights = (features.T @ error) / denominator + l2_strength * weights
        grad_bias = error.sum(axis=0) / denominator
        weights -= learning_rate * grad_weights
        bias -= learning_rate * grad_bias

    return weights, bias


def predict_scores(features, weights, bias):
    """Return sigmoid probability scores for a logistic regression model."""
    return sigmoid(features @ weights + bias)


def predict_multiclass_scores(features, weights, bias):
    """Return softmax probability scores for a multi-class model."""
    return softmax(features @ weights + bias)


def safe_log_loss(labels, scores):
    """Compute binary cross-entropy (log loss) with numerical clipping."""
    labels = np.asarray(labels, dtype=float)
    clipped = np.clip(scores, 1e-9, 1.0 - 1e-9)
    return float(-np.mean(labels * np.log(clipped) + (1.0 - labels) * np.log(1.0 - clipped)))


def roc_auc_score(labels, scores):
    """Compute the ROC-AUC score from binary labels and predicted scores."""
    labels = np.asarray(labels, dtype=int)
    positive_count = int(labels.sum())
    negative_count = int(len(labels) - positive_count)
    if positive_count == 0 or negative_count == 0:
        return float("nan")

    order = np.argsort(scores)
    ranks = np.empty(len(scores), dtype=float)
    ranks[order] = np.arange(1, len(scores) + 1, dtype=float)
    sum_positive_ranks = float(ranks[labels == 1].sum())
    numerator = sum_positive_ranks - positive_count * (positive_count + 1) / 2.0
    return numerator / (positive_count * negative_count)


def classification_metrics(labels, scores, threshold: float = 0.5):
    """Compute precision, recall, F1, accuracy, and ROC-AUC for binary classification."""
    labels = np.asarray(labels, dtype=int)
    scores = np.asarray(scores, dtype=float)
    predictions = (scores >= threshold).astype(int)

    true_positive = int(np.sum((labels == 1) & (predictions == 1)))
    true_negative = int(np.sum((labels == 0) & (predictions == 0)))
    false_positive = int(np.sum((labels == 0) & (predictions == 1)))
    false_negative = int(np.sum((labels == 1) & (predictions == 0)))

    positive_count = int(labels.sum())
    negative_count = int(len(labels) - positive_count)

    precision = true_positive / (true_positive + false_positive) if (true_positive + false_positive) else 0.0
    recall = true_positive / (true_positive + false_negative) if (true_positive + false_negative) else 0.0
    specificity = true_negative / (true_negative + false_positive) if (true_negative + false_positive) else 0.0
    accuracy = (true_positive + true_negative) / len(labels) if len(labels) else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if (precision + recall) else 0.0

    return {
        "sample_count": int(len(labels)),
        "positive_count": positive_count,
        "negative_count": negative_count,
        "positive_rate": positive_count / len(labels) if len(labels) else 0.0,
        "predicted_positive_rate": float(predictions.mean()) if len(predictions) else 0.0,
        "accuracy": accuracy,
        "precision": precision,
        "recall": recall,
        "specificity": specificity,
        "balanced_accuracy": 0.5 * (recall + specificity),
        "f1": f1,
        "roc_auc": roc_auc_score(labels, scores),
        "log_loss": safe_log_loss(labels, scores),
        "true_positive": true_positive,
        "true_negative": true_negative,
        "false_positive": false_positive,
        "false_negative": false_negative,
    }


def multiclass_metrics(labels, probabilities, class_names):
    """Compute per-class and macro-averaged metrics for multi-class classification."""
    labels = np.asarray(labels, dtype=int)
    probabilities = np.asarray(probabilities, dtype=float)
    predictions = np.argmax(probabilities, axis=1)
    accuracy = float(np.mean(predictions == labels)) if len(labels) else 0.0

    per_class_rows = []
    recalls = []
    f1_values = []
    confusion = np.zeros((len(class_names), len(class_names)), dtype=int)
    for true_index, pred_index in zip(labels, predictions):
        confusion[true_index, pred_index] += 1

    for class_index, class_name in enumerate(class_names):
        true_positive = int(confusion[class_index, class_index])
        false_negative = int(confusion[class_index, :].sum() - true_positive)
        false_positive = int(confusion[:, class_index].sum() - true_positive)
        true_count = int(confusion[class_index, :].sum())
        pred_count = int(confusion[:, class_index].sum())
        precision = true_positive / pred_count if pred_count else 0.0
        recall = true_positive / true_count if true_count else 0.0
        f1 = 2.0 * precision * recall / (precision + recall) if (precision + recall) else 0.0
        recalls.append(recall)
        f1_values.append(f1)
        per_class_rows.append(
            {
                "class_name": class_name,
                "support": true_count,
                "predicted_count": pred_count,
                "precision": precision,
                "recall": recall,
                "f1": f1,
            }
        )

    return {
        "sample_count": int(len(labels)),
        "accuracy": accuracy,
        "balanced_accuracy": float(np.mean(recalls)) if recalls else 0.0,
        "macro_f1": float(np.mean(f1_values)) if f1_values else 0.0,
        "per_class": per_class_rows,
        "confusion_matrix": confusion.tolist(),
        "predictions": predictions.tolist(),
    }


def slice_metrics(rows, scores, target_column: str, slice_column: str):
    """Compute classification metrics grouped by a categorical slice column."""
    values = sorted({row.get(slice_column, "") for row in rows})
    metrics_rows = []
    for value in values:
        indices = [idx for idx, row in enumerate(rows) if row.get(slice_column, "") == value]
        labels = [parse_binary_label(rows[idx][target_column]) for idx in indices]
        subset_scores = [float(scores[idx]) for idx in indices]
        metrics = classification_metrics(labels, subset_scores)
        metrics_rows.append(
            {
                slice_column: value,
                **metrics,
            }
        )
    return metrics_rows


def coefficient_summary_rows(feature_names, weights, limit: int = 15):
    """Return the top-N positive and negative feature coefficients sorted by weight."""
    pairs = list(zip(feature_names, weights))
    positive = sorted(pairs, key=lambda item: item[1], reverse=True)[:limit]
    negative = sorted(pairs, key=lambda item: item[1])[:limit]
    rows = []
    for direction, items in (("positive", positive), ("negative", negative)):
        for rank, (feature_name, weight) in enumerate(items, start=1):
            rows.append(
                {
                    "direction": direction,
                    "rank": rank,
                    "feature_name": feature_name,
                    "weight": float(weight),
                    "abs_weight": float(abs(weight)),
                }
            )
    return rows


def relative_to_repo(path: Path):
    """Convert an absolute path to a path relative to the repository root."""
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path.resolve())
