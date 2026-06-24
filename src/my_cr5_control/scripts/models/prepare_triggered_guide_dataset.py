#!/usr/bin/env python3
"""Triggered guide dataset preparer — extracts only those planning queries where the guide model was actually triggered, for fine-tuning the trigger threshold."""

import argparse
from collections import Counter
from pathlib import Path

from guide_model_schema import (
    derived_target_names,
    raw_target_names,
    target_value,
)
from model_pipeline_common import (
    RESULTS_ROOT,
    read_csv_rows,
    save_json,
    session_stamp,
    write_csv,
)


DEFAULT_TRACE_GLOB = "exports/guide_ablation_trace/*/attempted_only.csv"
DEFAULT_SOURCE_GLOB = "datasets/guide_ranking_simple/raw/*_guide_ranking_simple_dataset_results.csv"
DEFAULT_RESULTS_GLOB = "benchmarks/simple_guidance/raw/*_learned_guidance_simple_ablation_results.csv"
DEFAULT_OUTPUT_ROOT = RESULTS_ROOT / "datasets" / "guide_ranking_simple" / "filtered"
DEFAULT_MODE_FILTERS = ["learned_guided"]
RAW_TARGET_COLUMNS = raw_target_names()
DERIVED_TARGET_COLUMNS = derived_target_names()


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Prepare a filtered guide-ranking dataset from scenarios that actually triggered online guide attempts."
    )
    parser.add_argument(
        "--attempted-csvs",
        nargs="*",
        default=[],
        help="One or more attempted_only.csv files. Default: latest under test_results/exports/guide_ablation_trace/.",
    )
    parser.add_argument(
        "--results-csvs",
        nargs="*",
        default=[],
        help="One or more ablation results csv files. Attempted rows will be extracted directly from these files.",
    )
    parser.add_argument(
        "--results-glob",
        default="",
        help="Optional glob relative to test_results/. Matching ablation results are converted into attempted rows.",
    )
    parser.add_argument(
        "--source-dataset",
        default="",
        help="Source guide candidate dataset csv. Default: latest raw guide dataset with at least --min-source-rows rows.",
    )
    parser.add_argument(
        "--min-source-rows",
        type=int,
        default=100,
        help="Minimum rows required for default source dataset selection.",
    )
    parser.add_argument(
        "--mode-filters",
        nargs="*",
        default=list(DEFAULT_MODE_FILTERS),
        help="Only keep attempted rows from these modes. Default: learned_guided.",
    )
    parser.add_argument(
        "--match-level",
        choices=["scene_name", "repeat_scene"],
        default="scene_name",
        help="How attempted traces are mapped back to the source dataset.",
    )
    parser.add_argument(
        "--output-root",
        default="",
        help="Optional output directory root. Default: test_results/datasets/guide_ranking_simple/filtered/",
    )
    return parser.parse_args()


def csv_row_count(path: Path):
    """Csv row count."""
    with path.open("r", encoding="utf-8", newline="") as handle:
        return max(sum(1 for _ in handle) - 1, 0)


def latest_trace_csv():
    """Latest trace csv."""
    matches = sorted(RESULTS_ROOT.glob(DEFAULT_TRACE_GLOB))
    if not matches:
        raise RuntimeError(
            "No attempted_only.csv file was found in test_results/exports/guide_ablation_trace/."
        )
    return matches[-1]


def latest_source_dataset(min_rows: int):
    """Latest source dataset."""
    matches = sorted(RESULTS_ROOT.glob(DEFAULT_SOURCE_GLOB), reverse=True)
    if not matches:
        raise RuntimeError(
            "No raw guide ranking dataset was found in test_results/datasets/guide_ranking_simple/raw/."
        )
    for path in matches:
        if csv_row_count(path) >= min_rows:
            return path
    raise RuntimeError(
        f"No raw guide ranking dataset with at least {min_rows} rows was found under "
        "test_results/datasets/guide_ranking_simple/raw/."
    )


def attempted_rows_from_results_rows(rows):
    """Attempted rows from results rows."""
    exported = []
    for row in rows:
        attempted = int(row.get("guide_candidates_attempted", "0") or 0)
        if attempted <= 0:
            continue
        exported.append(
            {
                "实验时间戳": row.get("实验时间戳", ""),
                "重复序号": row.get("重复序号", ""),
                "模式": row.get("模式", ""),
                "ranking_name": row.get("ranking_name", ""),
                "场景名称": row.get("场景名称", ""),
                "难度": row.get("难度", ""),
                "成功": row.get("成功", ""),
                "墙钟时间(ms)": row.get("墙钟时间(ms)", ""),
                "触发预算上限": row.get("触发预算上限", ""),
                "reused_direct_baseline": row.get("reused_direct_baseline", ""),
                "guide_candidate_count": row.get("guide_candidate_count", ""),
                "guide_candidates_attempted": row.get("guide_candidates_attempted", ""),
                "used_direct_plan": row.get("used_direct_plan", ""),
                "selected_candidate_id": row.get("selected_candidate_id", ""),
                "selected_candidate_probability": row.get("selected_candidate_probability", ""),
                "selected_candidate_heuristic_cost": row.get("selected_candidate_heuristic_cost", ""),
                "selected_candidate_ranking_score": row.get("selected_candidate_ranking_score", ""),
                "selected_guide_x": row.get("selected_guide_x", ""),
                "selected_guide_y": row.get("selected_guide_y", ""),
                "selected_guide_z": row.get("selected_guide_z", ""),
            }
        )
    return exported


def positive_count(rows, target_name):
    """Positive count."""
    return sum(int(target_value(row, target_name) == "1") for row in rows)


def attempted_selector_keys(attempted_rows, match_level):
    """Attempted selector keys."""
    if match_level == "scene_name":
        return {row.get("场景名称", "") for row in attempted_rows if row.get("场景名称", "")}
    if match_level == "repeat_scene":
        return {
            (row.get("重复序号", ""), row.get("场景名称", ""))
            for row in attempted_rows
            if row.get("重复序号", "") and row.get("场景名称", "")
        }
    raise KeyError(f"Unsupported match level: {match_level}")


def row_matches(row, match_level, selector_keys):
    """Row matches."""
    if match_level == "scene_name":
        return row.get("场景名称", "") in selector_keys
    if match_level == "repeat_scene":
        return (row.get("重复序号", ""), row.get("场景名称", "")) in selector_keys
    raise KeyError(f"Unsupported match level: {match_level}")


def main():
    """Main."""
    args = parse_args()
    attempted_paths = [Path(path) for path in args.attempted_csvs]
    results_paths = [Path(path) for path in args.results_csvs]
    if args.results_glob:
        results_paths.extend(sorted(RESULTS_ROOT.glob(args.results_glob)))
    if not attempted_paths and not results_paths:
        attempted_paths = [latest_trace_csv()]
    source_dataset_path = Path(args.source_dataset) if args.source_dataset else latest_source_dataset(args.min_source_rows)

    attempted_rows = []
    for path in attempted_paths:
        rows = read_csv_rows(path)
        if not rows:
            continue
        attempted_rows.extend(rows)
    for path in results_paths:
        rows = read_csv_rows(path)
        if not rows:
            continue
        attempted_rows.extend(attempted_rows_from_results_rows(rows))

    if args.mode_filters:
        allowed_modes = set(args.mode_filters)
        attempted_rows = [row for row in attempted_rows if row.get("模式", "") in allowed_modes]

    if not attempted_rows:
        raise RuntimeError("No attempted guide rows remained after loading traces and applying mode filters.")

    selector_keys = attempted_selector_keys(attempted_rows, args.match_level)
    source_rows = read_csv_rows(source_dataset_path)
    filtered_rows = [row for row in source_rows if row_matches(row, args.match_level, selector_keys)]
    if not filtered_rows:
        raise RuntimeError("No source dataset rows matched the attempted-guide selectors.")

    stamp = session_stamp()
    output_root = Path(args.output_root) if args.output_root else DEFAULT_OUTPUT_ROOT
    output_root.mkdir(parents=True, exist_ok=True)
    output_csv = output_root / f"{stamp}_guide_ranking_simple_triggered_dataset_results.csv"
    output_meta = output_root / f"{stamp}_guide_ranking_simple_triggered_dataset_metadata.json"

    write_csv(output_csv, list(filtered_rows[0].keys()), filtered_rows)

    target_counts = {
        target_name: positive_count(filtered_rows, target_name)
        for target_name in RAW_TARGET_COLUMNS + DERIVED_TARGET_COLUMNS
    }
    scene_counts = dict(sorted(Counter(row.get("场景名称", "") for row in filtered_rows).items()))
    scene_uid_counts = dict(sorted(Counter(row.get("场景UID", "") for row in filtered_rows).items()))
    repeated_scene_keys = sorted(
        {f"{row.get('重复序号', '')}:{row.get('场景名称', '')}" for row in attempted_rows}
    )

    metadata = {
        "attempted_csvs": [str(path) for path in attempted_paths],
        "results_csvs": [str(path) for path in results_paths],
        "source_dataset": str(source_dataset_path),
        "match_level": args.match_level,
        "mode_filters": list(args.mode_filters),
        "source_row_count": len(source_rows),
        "attempted_row_count": len(attempted_rows),
        "filtered_row_count": len(filtered_rows),
        "filtered_scene_count": len(scene_counts),
        "filtered_scene_uid_count": len(scene_uid_counts),
        "scene_counts": scene_counts,
        "scene_uid_counts": scene_uid_counts,
        "selected_scene_names": sorted(scene_counts.keys()),
        "selected_repeat_scene_keys": repeated_scene_keys,
        "target_positive_counts": target_counts,
        "output_csv": str(output_csv),
    }
    save_json(output_meta, metadata)

    print(f"source_dataset={source_dataset_path}")
    print(f"attempted_csv_count={len(attempted_paths)}")
    print(f"results_csv_count={len(results_paths)}")
    print(f"attempted_row_count={len(attempted_rows)}")
    print(f"match_level={args.match_level}")
    print(f"filtered_row_count={len(filtered_rows)}")
    print(f"selected_scene_names={','.join(sorted(scene_counts.keys()))}")
    for target_name, count in target_counts.items():
        print(f"{target_name}={count}/{len(filtered_rows)}")
    print(f"output_csv={output_csv}")
    print(f"output_meta={output_meta}")


if __name__ == "__main__":
    main()
