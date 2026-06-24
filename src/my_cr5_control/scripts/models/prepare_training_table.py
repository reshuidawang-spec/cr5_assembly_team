#!/usr/bin/env python3
"""Training table preparer — joins raw benchmark results with task metadata to produce a flat feature+label table ready for model training."""

import argparse
from pathlib import Path

from model_pipeline_common import (
    AUXILIARY_EVAL_COLUMNS,
    CATEGORICAL_FEATURE_COLUMNS,
    DEFAULT_PREPARED_TABLE,
    LABEL_COLUMNS,
    NUMERIC_FEATURE_COLUMNS,
    PLANNER_INTERACTION_BASE_FEATURES,
    canonical_simple_random_results,
    default_prepared_metadata_path,
    parse_binary_label,
    parse_float,
    read_csv_rows,
    relative_to_repo,
    safe_ratio,
    save_json,
    write_csv,
)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Normalize simple random dataset results into a leakage-safe training table."
    )
    parser.add_argument(
        "--inputs",
        nargs="*",
        default=[],
        help="One or more *_simple_random_task_dataset_results.csv files. Default: canonical 300-task file from dataset_manifest.csv.",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_PREPARED_TABLE),
        help="Output csv path. Default: test_results/exports/simple_random_training_table.csv",
    )
    parser.add_argument(
        "--metadata-output",
        default="",
        help="Optional metadata json path. Default: <output>_metadata.json",
    )
    return parser.parse_args()


def normalize_row(source_path, row):
    """Normalize row."""
    timestamp = row.get("实验时间戳", "")
    random_seed = row.get("随机种子", "")
    task_index = row.get("任务序号", "")
    task_uid = f"{timestamp}:{random_seed}:{task_index}"

    tip_x = parse_float(row.get("尖端X"))
    tip_y = parse_float(row.get("尖端Y"))
    tip_z = parse_float(row.get("尖端Z"))
    flange_x = parse_float(row.get("法兰X"))
    flange_y = parse_float(row.get("法兰Y"))
    flange_z = parse_float(row.get("法兰Z"))
    local_opening_size = parse_float(row.get("局部开口尺寸(m)"))
    estimated_clearance = parse_float(row.get("估计净空(m)"))
    top_depth = parse_float(row.get("顶部深度(m)"))
    hole_depth = parse_float(row.get("孔深"))
    radial_offset = parse_float(row.get("距孔中心径向偏移(m)"))
    lateral_offset_x = parse_float(row.get("横向偏移X(m)"))
    lateral_offset_y = parse_float(row.get("横向偏移Y(m)"))
    difficulty_score = parse_float(row.get("难度评分"))
    lateral_offset_norm = (lateral_offset_x ** 2 + lateral_offset_y ** 2) ** 0.5
    approach_dx = flange_x - tip_x
    approach_dy = flange_y - tip_y
    approach_dz = flange_z - tip_z
    approach_distance = (approach_dx ** 2 + approach_dy ** 2 + approach_dz ** 2) ** 0.5

    return {
        "source_results_file": relative_to_repo(source_path),
        "dataset_version": row.get("数据集版本", ""),
        "timestamp": timestamp,
        "random_seed": random_seed,
        "total_tasks": row.get("总任务数", ""),
        "task_index": task_index,
        "task_uid": task_uid,
        "task_family": row.get("任务族", ""),
        "difficulty": row.get("难度", ""),
        "task_description": row.get("任务描述", ""),
        "planner": row.get("规划器", ""),
        "planning_mode": row.get("模式", ""),
        "planner_id": row.get("规划器ID", ""),
        "tip_x": f"{tip_x:.6f}",
        "tip_y": f"{tip_y:.6f}",
        "tip_z": f"{tip_z:.6f}",
        "normal_x": f"{parse_float(row.get('法向X')):.6f}",
        "normal_y": f"{parse_float(row.get('法向Y')):.6f}",
        "normal_z": f"{parse_float(row.get('法向Z')):.6f}",
        "flange_x": f"{flange_x:.6f}",
        "flange_y": f"{flange_y:.6f}",
        "flange_z": f"{flange_z:.6f}",
        "box_width_m": f"{parse_float(row.get('箱体宽')):.6f}",
        "box_depth_m": f"{parse_float(row.get('箱体深')):.6f}",
        "box_height_m": f"{parse_float(row.get('箱体高')):.6f}",
        "hole_radius_m": f"{parse_float(row.get('孔半径')):.6f}",
        "hole_depth_m": f"{hole_depth:.6f}",
        "top_depth_m": f"{top_depth:.6f}",
        "depth_ratio": f"{parse_float(row.get('深度比例')):.6f}",
        "lateral_offset_x_m": f"{lateral_offset_x:.6f}",
        "lateral_offset_y_m": f"{lateral_offset_y:.6f}",
        "radial_offset_m": f"{radial_offset:.6f}",
        "local_opening_size_m": f"{local_opening_size:.6f}",
        "estimated_clearance_m": f"{estimated_clearance:.6f}",
        "difficulty_score": f"{difficulty_score:.6f}",
        "abs_lateral_offset_x_m": f"{abs(lateral_offset_x):.6f}",
        "abs_lateral_offset_y_m": f"{abs(lateral_offset_y):.6f}",
        "lateral_offset_norm_m": f"{lateral_offset_norm:.6f}",
        "approach_dx_m": f"{approach_dx:.6f}",
        "approach_dy_m": f"{approach_dy:.6f}",
        "approach_dz_m": f"{approach_dz:.6f}",
        "approach_distance_m": f"{approach_distance:.6f}",
        "clearance_to_opening_ratio": f"{safe_ratio(estimated_clearance, local_opening_size):.6f}",
        "radial_to_opening_ratio": f"{safe_ratio(radial_offset, local_opening_size):.6f}",
        "top_depth_to_hole_depth_ratio": f"{safe_ratio(top_depth, hole_depth):.6f}",
        "depth_to_opening_ratio": f"{safe_ratio(top_depth, local_opening_size):.6f}",
        "clearance_margin_m": f"{(estimated_clearance - radial_offset):.6f}",
        "planner_task_family": f"{row.get('规划器', '')}__{row.get('任务族', '')}",
        "planner_difficulty": f"{row.get('规划器', '')}__{row.get('难度', '')}",
        "wall_time_ms": f"{parse_float(row.get('墙钟时间(ms)')):.3f}",
        "moveit_time_ms": f"{parse_float(row.get('MoveIt规划时间(ms)')):.3f}",
        "planning_budget_ms": f"{parse_float(row.get('预算上限(ms)')):.3f}",
        "planner_calls": f"{parse_float(row.get('规划调用次数'), default=1.0):.3f}",
        "budget_headroom_ms": f"{parse_float(row.get('预算上限(ms)')) - parse_float(row.get('墙钟时间(ms)')):.3f}",
        "success": str(parse_binary_label(row.get("成功", "0"))),
        "hit_budget_limit": str(parse_binary_label(row.get("触发预算上限", "0"))),
        "fast_solve_lt_1s": str(parse_binary_label(row.get("快速求解(<1s)", "0"))),
    }


def main():
    """Main."""
    args = parse_args()
    input_paths = [Path(path) for path in args.inputs] if args.inputs else [canonical_simple_random_results()]

    rows = []
    source_paths = []
    for source_path in input_paths:
        source_paths.append(source_path)
        source_rows = read_csv_rows(source_path)
        rows.extend(normalize_row(source_path, row) for row in source_rows if row.get("规划器"))

    planner_values = sorted({row["planner"] for row in rows})
    planner_interaction_columns = []
    for planner in planner_values:
        for feature_name in PLANNER_INTERACTION_BASE_FEATURES:
            column_name = f"planner_interaction__{planner}__{feature_name}"
            planner_interaction_columns.append(column_name)
            for row in rows:
                value = parse_float(row.get(feature_name, "0"))
                row[column_name] = f"{value:.6f}" if row["planner"] == planner else "0.000000"

    numeric_feature_columns = [
        *NUMERIC_FEATURE_COLUMNS,
        "abs_lateral_offset_x_m",
        "abs_lateral_offset_y_m",
        "lateral_offset_norm_m",
        "approach_dx_m",
        "approach_dy_m",
        "approach_dz_m",
        "approach_distance_m",
        "clearance_to_opening_ratio",
        "radial_to_opening_ratio",
        "top_depth_to_hole_depth_ratio",
        "depth_to_opening_ratio",
        "clearance_margin_m",
        *planner_interaction_columns,
    ]
    categorical_feature_columns = [
        *CATEGORICAL_FEATURE_COLUMNS,
        "planner_task_family",
        "planner_difficulty",
    ]

    output_path = Path(args.output)
    metadata_path = (
        Path(args.metadata_output)
        if args.metadata_output
        else default_prepared_metadata_path(output_path)
    )

    fieldnames = [
        "source_results_file",
        "dataset_version",
        "timestamp",
        "random_seed",
        "total_tasks",
        "task_index",
        "task_uid",
        "task_family",
        "difficulty",
        "task_description",
        "planner",
        "planning_mode",
        "planner_id",
        *numeric_feature_columns,
        *categorical_feature_columns[4:],
        *AUXILIARY_EVAL_COLUMNS,
        *LABEL_COLUMNS,
    ]
    write_csv(output_path, fieldnames, rows)

    metadata = {
        "input_files": [relative_to_repo(path) for path in source_paths],
        "output_table": relative_to_repo(output_path),
        "row_count": len(rows),
        "task_count": len({row["task_uid"] for row in rows}),
        "numeric_feature_columns": numeric_feature_columns,
        "categorical_feature_columns": categorical_feature_columns,
        "label_columns": LABEL_COLUMNS,
        "auxiliary_eval_columns": AUXILIARY_EVAL_COLUMNS,
        "group_column": "task_uid",
        "planner_values": planner_values,
        "task_family_values": sorted({row["task_family"] for row in rows}),
        "difficulty_values": sorted({row["difficulty"] for row in rows}),
    }
    save_json(metadata_path, metadata)

    print(f"Wrote {len(rows)} rows to {output_path}")
    print(f"Wrote metadata to {metadata_path}")
    print(f"Unique tasks: {metadata['task_count']}")


if __name__ == "__main__":
    main()
