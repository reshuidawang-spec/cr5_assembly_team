#!/usr/bin/env python3
"""Oracle planner dataset preparer — constructs a training dataset where each planning query is labelled with the best-performing planner from exhaustive benchmark results."""

import argparse
from collections import defaultdict
from pathlib import Path

from model_pipeline_common import (
    DEFAULT_PREPARED_TABLE,
    default_prepared_metadata_path,
    load_json,
    parse_binary_label,
    parse_float,
    read_csv_rows,
    resolve_task_only_feature_columns,
    save_json,
    write_csv,
)


DEFAULT_PLANNERS = ["FMT", "LBTRRT", "RRTConnect"]


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Build a task-level oracle planner dataset from the prepared simple random training table."
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
        "--planners",
        nargs="*",
        default=list(DEFAULT_PLANNERS),
        help="Candidate planner subset used to define the oracle label.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional output csv path. Default: test_results/exports/simple_random_oracle_planner_dataset_<planners>.csv",
    )
    parser.add_argument(
        "--metadata-output",
        default="",
        help="Optional metadata json path. Default: <output>_metadata.json",
    )
    return parser.parse_args()


def planner_tag(planners):
    """Planner tag."""
    return "_".join(planners)


def default_output_path(prepared_table_path: Path, planners):
    """Default output path."""
    suffix = planner_tag(planners)
    return prepared_table_path.with_name(f"simple_random_oracle_planner_dataset_{suffix}.csv")


def oracle_sort_key(row):
    """Oracle sort key."""
    return (
        parse_binary_label(row["hit_budget_limit"]),
        -parse_binary_label(row["fast_solve_lt_1s"]),
        -parse_binary_label(row["success"]),
        parse_float(row.get("wall_time_ms", "0")),
        row.get("planner", ""),
    )


def main():
    """Main."""
    args = parse_args()
    prepared_table_path = Path(args.prepared_table)
    prepared_metadata_path = (
        Path(args.prepared_metadata)
        if args.prepared_metadata
        else default_prepared_metadata_path(prepared_table_path)
    )
    prepared_metadata = load_json(prepared_metadata_path)
    task_numeric_columns, task_categorical_columns = resolve_task_only_feature_columns(prepared_metadata)

    rows = read_csv_rows(prepared_table_path)
    if not rows:
        raise RuntimeError(f"No rows found in {prepared_table_path}")

    allowed_planners = list(args.planners)
    by_task = defaultdict(list)
    for row in rows:
        if row.get("planner", "") in allowed_planners:
            by_task[row["task_uid"]].append(row)

    dataset_rows = []
    skipped_tasks = 0
    for task_uid, group in by_task.items():
        planners_in_group = {row["planner"] for row in group}
        if planners_in_group != set(allowed_planners):
            skipped_tasks += 1
            continue

        oracle_row = min(group, key=oracle_sort_key)
        base_row = group[0]
        output_row = {
            "task_uid": task_uid,
            "task_index": base_row.get("task_index", ""),
            "timestamp": base_row.get("timestamp", ""),
            "random_seed": base_row.get("random_seed", ""),
            "task_family": base_row.get("task_family", ""),
            "difficulty": base_row.get("difficulty", ""),
            "task_description": base_row.get("task_description", ""),
            "oracle_planner": oracle_row["planner"],
        }
        for column in task_numeric_columns:
            output_row[column] = base_row.get(column, "")
        for column in task_categorical_columns:
            output_row[column] = base_row.get(column, "")

        for planner in allowed_planners:
            planner_row = next(row for row in group if row["planner"] == planner)
            planner_prefix = f"planner_metric__{planner}__"
            output_row[f"{planner_prefix}success"] = planner_row.get("success", "")
            output_row[f"{planner_prefix}hit_budget_limit"] = planner_row.get("hit_budget_limit", "")
            output_row[f"{planner_prefix}fast_solve_lt_1s"] = planner_row.get("fast_solve_lt_1s", "")
            output_row[f"{planner_prefix}wall_time_ms"] = planner_row.get("wall_time_ms", "")
            output_row[f"{planner_prefix}moveit_time_ms"] = planner_row.get("moveit_time_ms", "")
            output_row[f"{planner_prefix}budget_headroom_ms"] = planner_row.get("budget_headroom_ms", "")
        dataset_rows.append(output_row)

    output_path = Path(args.output) if args.output else default_output_path(prepared_table_path, allowed_planners)
    metadata_path = (
        Path(args.metadata_output)
        if args.metadata_output
        else default_prepared_metadata_path(output_path)
    )

    planner_metric_columns = []
    for planner in allowed_planners:
        planner_prefix = f"planner_metric__{planner}__"
        planner_metric_columns.extend(
            [
                f"{planner_prefix}success",
                f"{planner_prefix}hit_budget_limit",
                f"{planner_prefix}fast_solve_lt_1s",
                f"{planner_prefix}wall_time_ms",
                f"{planner_prefix}moveit_time_ms",
                f"{planner_prefix}budget_headroom_ms",
            ]
        )

    fieldnames = [
        "task_uid",
        "task_index",
        "timestamp",
        "random_seed",
        "task_family",
        "difficulty",
        "task_description",
        *task_numeric_columns,
        *task_categorical_columns,
        "oracle_planner",
        *planner_metric_columns,
    ]
    write_csv(output_path, fieldnames, dataset_rows)

    metadata = {
        "prepared_table": str(prepared_table_path),
        "prepared_metadata": str(prepared_metadata_path),
        "row_count": len(dataset_rows),
        "skipped_tasks": skipped_tasks,
        "candidate_planners": allowed_planners,
        "numeric_feature_columns": task_numeric_columns,
        "categorical_feature_columns": task_categorical_columns,
        "label_column": "oracle_planner",
        "oracle_rule": [
            "min hit_budget_limit",
            "max fast_solve_lt_1s",
            "max success",
            "min wall_time_ms",
        ],
    }
    save_json(metadata_path, metadata)

    print(f"Wrote {len(dataset_rows)} rows to {output_path}")
    print(f"Wrote metadata to {metadata_path}")
    print(f"Skipped tasks: {skipped_tasks}")


if __name__ == "__main__":
    main()
