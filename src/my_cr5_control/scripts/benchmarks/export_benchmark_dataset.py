#!/usr/bin/env python3
"""Benchmark dataset exporter — normalises raw planner benchmark results into a unified CSV training dataset with feature columns and binary success/failure labels."""

import argparse
import csv
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = REPO_ROOT / "test_results"
DEFAULT_OUTPUT = RESULTS_ROOT / "exports" / "benchmark_training_dataset.csv"
MANIFEST_PATH = RESULTS_ROOT / "dataset_manifest.csv"
FAST_THRESHOLD_MS = 1000.0
BUDGET_TOLERANCE_MS = 100.0


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Normalize benchmark csv files into a single dataset for analysis or training."
    )
    parser.add_argument(
        "--inputs",
        nargs="*",
        default=[],
        help="Optional result csv paths. Default: latest timestamped simple and v2 result files.",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT),
        help="Output csv path. Default: test_results/exports/benchmark_training_dataset.csv",
    )
    return parser.parse_args()


def latest_result(pattern):
    """Latest result."""
    matches = sorted(RESULTS_ROOT.glob(pattern))
    return matches[-1] if matches else None


def manifest_inputs():
    """Manifest inputs."""
    if not MANIFEST_PATH.exists():
        return []

    with MANIFEST_PATH.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))

    desired_keys = ["simple_formal_latest", "v2_formal_latest"]
    paths = []
    for dataset_key in desired_keys:
        row = next((item for item in rows if item.get("dataset_key") == dataset_key), None)
        if row is None:
            continue
        result_file = row.get("results_file", "")
        if not result_file:
            continue
        path = REPO_ROOT / result_file
        if path.exists():
            paths.append(path)
    return paths


def default_inputs():
    """Default inputs."""
    manifest_paths = manifest_inputs()
    if manifest_paths:
        return manifest_paths

    paths = []
    for pattern in (
        "benchmarks/simple/raw/20*_planner_comparison_simple_results.csv",
        "benchmarks/v2/raw/20*_planner_comparison_v2_results.csv",
    ):
        match = latest_result(pattern)
        if match is not None:
            paths.append(match)
    if not paths:
        raise RuntimeError(
            "No timestamped benchmark result csv files were found in test_results/benchmarks/*/raw/."
        )
    return paths


def read_rows(path):
    """Read rows."""
    with path.open("r", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def infer_benchmark_family(path, row):
    """Infer benchmark family."""
    version = row.get("基准版本", "")
    name = path.name
    if "simple" in version or "simple" in name:
        return "simple"
    if "v2" in version or "v2" in name:
        return "v2"
    return "unknown"


def infer_success(row):
    """Infer success."""
    return row.get("成功", "") == "成功"


def parse_float(row, *keys, default=0.0):
    """Parse a string to float, returning a default for empty/missing values."""
    for key in keys:
        value = row.get(key, "")
        if value not in ("", None):
            return float(value)
    return default


def parse_int(row, *keys, default=0):
    """Parse int."""
    for key in keys:
        value = row.get(key, "")
        if value not in ("", None):
            return int(float(value))
    return default


def parse_yes_no(row, key, fallback=False):
    """Parse yes no."""
    value = row.get(key, "")
    if value in ("是", "true", "True", "1"):
        return True
    if value in ("否", "false", "False", "0"):
        return False
    return fallback


def normalize_row(path, row):
    """Normalize row."""
    wall_time_ms = parse_float(row, "墙钟时间(ms)", "规划时间(ms)")
    budget_ms = parse_float(row, "预算上限(ms)", default=10000.0)
    moveit_time_ms = parse_float(row, "MoveIt规划时间(ms)", "规划时间(ms)", default=wall_time_ms)
    hit_budget = parse_yes_no(
        row,
        "触发预算上限",
        fallback=(budget_ms > 0.0 and wall_time_ms >= max(0.0, budget_ms - BUDGET_TOLERANCE_MS)),
    )
    fast_solve = parse_yes_no(row, "快速求解(<1s)", fallback=wall_time_ms < FAST_THRESHOLD_MS)

    return {
        "source_file": str(path),
        "benchmark_family": infer_benchmark_family(path, row),
        "benchmark_version": row.get("基准版本", ""),
        "timestamp": row.get("实验时间戳", ""),
        "repeat_index": parse_int(row, "重复序号"),
        "planner": row.get("规划器", ""),
        "planning_mode": row.get("模式", ""),
        "planner_id": row.get("规划器ID", ""),
        "scene_name": row.get("场景名称", ""),
        "difficulty": row.get("难度", ""),
        "point_description": row.get("测点描述", ""),
        "difficulty_score": parse_float(row, "难度评分"),
        "tip_x": parse_float(row, "尖端X"),
        "tip_y": parse_float(row, "尖端Y"),
        "tip_z": parse_float(row, "尖端Z"),
        "flange_x": parse_float(row, "法兰X"),
        "flange_y": parse_float(row, "法兰Y"),
        "flange_z": parse_float(row, "法兰Z"),
        "success": "1" if infer_success(row) else "0",
        "wall_time_ms": f"{wall_time_ms:.1f}",
        "moveit_time_ms": f"{moveit_time_ms:.1f}",
        "planning_budget_ms": f"{budget_ms:.1f}",
        "planner_calls": str(parse_int(row, "规划调用次数", default=1)),
        "hit_budget_limit": "1" if hit_budget else "0",
        "fast_solve_lt_1s": "1" if fast_solve else "0",
        "budget_headroom_ms": f"{budget_ms - wall_time_ms:.1f}",
    }


def write_dataset(path, rows):
    """Write dataset."""
    fieldnames = [
        "source_file",
        "benchmark_family",
        "benchmark_version",
        "timestamp",
        "repeat_index",
        "planner",
        "planning_mode",
        "planner_id",
        "scene_name",
        "difficulty",
        "point_description",
        "difficulty_score",
        "tip_x",
        "tip_y",
        "tip_z",
        "flange_x",
        "flange_y",
        "flange_z",
        "success",
        "wall_time_ms",
        "moveit_time_ms",
        "planning_budget_ms",
        "planner_calls",
        "hit_budget_limit",
        "fast_solve_lt_1s",
        "budget_headroom_ms",
    ]

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    """Main."""
    args = parse_args()
    input_paths = [Path(path) for path in args.inputs] if args.inputs else default_inputs()

    normalized_rows = []
    for path in input_paths:
        rows = read_rows(path)
        normalized_rows.extend(normalize_row(path, row) for row in rows)

    output_path = Path(args.output)
    write_dataset(output_path, normalized_rows)

    planners = sorted({row["planner"] for row in normalized_rows})
    benchmarks = sorted({row["benchmark_family"] for row in normalized_rows})
    print(f"Wrote {len(normalized_rows)} rows to {output_path}")
    print(f"Benchmarks: {', '.join(benchmarks)}")
    print(f"Planners: {', '.join(planners)}")


if __name__ == "__main__":
    main()
