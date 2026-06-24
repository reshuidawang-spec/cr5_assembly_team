#!/usr/bin/env python3
"""Dataset manifest builder — scans the test_results directory tree and builds a CSV manifest indexing all datasets, their metadata, and result file paths."""

import csv
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = REPO_ROOT / "test_results"
OUTPUT_PATH = RESULTS_ROOT / "dataset_manifest.csv"


def rel(path: Path) -> str:
    """Rel."""
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def maybe(path_str: str) -> str:
    """Maybe."""
    if not path_str:
        return ""
    path = REPO_ROOT / path_str
    return path_str if path.exists() else ""


def latest_or_blank(pattern: str) -> str:
    """Latest or blank."""
    matches = sorted(RESULTS_ROOT.glob(pattern))
    return rel(matches[-1]) if matches else ""


def build_rows():
    """Build rows."""
    rows = []

    rows.append(
        {
            "importance": "critical",
            "dataset_type": "formal_benchmark",
            "dataset_key": "simple_formal_latest",
            "benchmark_family": "simple",
            "timestamp": "20260311_124449_790",
            "status": "canonical",
            "results_file": maybe("test_results/benchmarks/simple/raw/20260311_124449_790_planner_comparison_simple_results.csv"),
            "summary_file": maybe("test_results/benchmarks/simple/raw/20260311_124449_790_planner_comparison_simple_summary.csv"),
            "export_file": "",
            "analysis_doc": "",
            "notes": "Current formal metric-enabled simple benchmark reference.",
        }
    )

    rows.append(
        {
            "importance": "critical",
            "dataset_type": "formal_benchmark",
            "dataset_key": "v2_formal_latest",
            "benchmark_family": "v2",
            "timestamp": "20260317_142203_372",
            "status": "canonical",
            "results_file": maybe("test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_results.csv"),
            "summary_file": maybe("test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_summary.csv"),
            "export_file": "",
            "analysis_doc": maybe("docs/analysis/V2_BENCHMARK_20260317_142203_372_ANALYSIS.md"),
            "notes": "Current formal STL-driven 2-cavity v2 benchmark reference.",
        }
    )

    rows.append(
        {
            "importance": "critical",
            "dataset_type": "training_export",
            "dataset_key": "benchmark_training_dataset",
            "benchmark_family": "simple+v2",
            "timestamp": "",
            "status": "canonical",
            "results_file": "",
            "summary_file": "",
            "export_file": maybe("test_results/exports/benchmark_training_dataset.csv"),
            "analysis_doc": maybe("docs/guides/BENCHMARK_DATASET_GUIDE.md"),
            "notes": "Unified exported benchmark dataset for downstream analysis/training.",
        }
    )

    rows.append(
        {
            "importance": "medium",
            "dataset_type": "random_dataset",
            "dataset_key": "simple_random_smoke",
            "benchmark_family": "simple_random",
            "timestamp": "20260311_135953_957",
            "status": "smoke",
            "results_file": maybe("test_results/datasets/simple_random/raw/20260311_135953_957_simple_random_task_dataset_results.csv"),
            "summary_file": maybe("test_results/datasets/simple_random/raw/20260311_135953_957_simple_random_task_dataset_summary.csv"),
            "export_file": "",
            "analysis_doc": "",
            "notes": "First smoke run; useful for schema/progress tracing but not main training source.",
        }
    )

    rows.append(
        {
            "importance": "high",
            "dataset_type": "random_dataset",
            "dataset_key": "simple_random_100",
            "benchmark_family": "simple_random",
            "timestamp": "20260311_140426_647",
            "status": "training_candidate",
            "results_file": maybe("test_results/datasets/simple_random/raw/20260311_140426_647_simple_random_task_dataset_results.csv"),
            "summary_file": maybe("test_results/datasets/simple_random/raw/20260311_140426_647_simple_random_task_dataset_summary.csv"),
            "export_file": "",
            "analysis_doc": maybe("docs/analysis/SIMPLE_RANDOM_DATASET_20260311_140426_ANALYSIS.md"),
            "notes": "First formal random dataset with mixed success/failure labels.",
        }
    )

    rows.append(
        {
            "importance": "critical",
            "dataset_type": "random_dataset",
            "dataset_key": "simple_random_300",
            "benchmark_family": "simple_random",
            "timestamp": "20260311_142950_774",
            "status": "canonical_training_source",
            "results_file": maybe("test_results/datasets/simple_random/raw/20260311_142950_774_simple_random_task_dataset_results.csv"),
            "summary_file": maybe("test_results/datasets/simple_random/raw/20260311_142950_774_simple_random_task_dataset_summary.csv"),
            "export_file": "",
            "analysis_doc": maybe("docs/analysis/SIMPLE_RANDOM_DATASET_20260311_142950_774_ANALYSIS.md"),
            "notes": "Current main Stage-2 training source; hole_deep remains dominant hard family.",
        }
    )

    rows.append(
        {
            "importance": "high",
            "dataset_type": "guide_candidate_dataset",
            "dataset_key": "guide_ranking_simple_dataset_v1",
            "benchmark_family": "simple_guidance",
            "timestamp": "20260317_160531_045",
            "status": "training_candidate",
            "results_file": maybe("test_results/datasets/guide_ranking_simple/raw/20260317_160531_045_guide_ranking_simple_dataset_results.csv"),
            "summary_file": "",
            "export_file": "",
            "analysis_doc": "",
            "notes": "First guide-candidate dataset for learned guide ranking; 144 candidate rows across simple scenes.",
        }
    )

    rows.append(
        {
            "importance": "high",
            "dataset_type": "ablation_benchmark",
            "dataset_key": "learned_guidance_simple_ablation_v1",
            "benchmark_family": "simple_guidance",
            "timestamp": "20260317_160940_152",
            "status": "negative_first_closure",
            "results_file": maybe("test_results/benchmarks/simple_guidance/raw/20260317_160940_152_learned_guidance_simple_ablation_results.csv"),
            "summary_file": maybe("test_results/benchmarks/simple_guidance/raw/20260317_160940_152_learned_guidance_simple_ablation_summary.csv"),
            "export_file": maybe("test_results/models/guide_ranking_simple/20260317_160902_candidate_viable/linear_model.csv"),
            "analysis_doc": maybe("docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_160940_152_ANALYSIS.md"),
            "notes": "First online learned guide-ranking closure; current candidate_viable linear model regresses versus heuristic guidance.",
        }
    )

    rows.append(
        {
            "importance": "high",
            "dataset_type": "guide_candidate_dataset",
            "dataset_key": "guide_ranking_simple_dataset_v2_refined",
            "benchmark_family": "simple_guidance",
            "timestamp": "20260317_171238_252",
            "status": "refined_training_candidate",
            "results_file": maybe("test_results/datasets/guide_ranking_simple/raw/20260317_171238_252_guide_ranking_simple_dataset_results.csv"),
            "summary_file": "",
            "export_file": "",
            "analysis_doc": maybe("docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_171817_260_171943_687_ANALYSIS.md"),
            "notes": "Refined guide-candidate dataset with clearance/manipulability features; 305 candidate rows.",
        }
    )

    rows.append(
        {
            "importance": "high",
            "dataset_type": "ablation_benchmark",
            "dataset_key": "learned_guidance_simple_ablation_v2_viable",
            "benchmark_family": "simple_guidance",
            "timestamp": "20260317_171817_260",
            "status": "negative_refined_retry",
            "results_file": maybe("test_results/benchmarks/simple_guidance/raw/20260317_171817_260_learned_guidance_simple_ablation_results.csv"),
            "summary_file": maybe("test_results/benchmarks/simple_guidance/raw/20260317_171817_260_learned_guidance_simple_ablation_summary.csv"),
            "export_file": maybe("test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv"),
            "analysis_doc": maybe("docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_171817_260_171943_687_ANALYSIS.md"),
            "notes": "Refined candidate_viable online retry; offline metrics improved but online learned guidance still collapses to budget hits.",
        }
    )

    rows.append(
        {
            "importance": "high",
            "dataset_type": "ablation_benchmark",
            "dataset_key": "learned_guidance_simple_ablation_v2_preferred",
            "benchmark_family": "simple_guidance",
            "timestamp": "20260317_171943_687",
            "status": "negative_refined_retry",
            "results_file": maybe("test_results/benchmarks/simple_guidance/raw/20260317_171943_687_learned_guidance_simple_ablation_results.csv"),
            "summary_file": maybe("test_results/benchmarks/simple_guidance/raw/20260317_171943_687_learned_guidance_simple_ablation_summary.csv"),
            "export_file": maybe("test_results/models/guide_ranking_simple/20260317_171600_candidate_preferred_refined/linear_model.csv"),
            "analysis_doc": maybe("docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_171817_260_171943_687_ANALYSIS.md"),
            "notes": "Refined candidate_preferred online retry; negative result repeats, pointing to ranking-policy mismatch rather than missing data.",
        }
    )

    rows.append(
        {
            "importance": "critical",
            "dataset_type": "ablation_benchmark",
            "dataset_key": "learned_guidance_simple_ablation_v3_viable_gated",
            "benchmark_family": "simple_guidance",
            "timestamp": "20260318_085048_146",
            "status": "policy_fix_budget_stable",
            "results_file": maybe("test_results/benchmarks/simple_guidance/raw/20260318_085048_146_learned_guidance_simple_ablation_results.csv"),
            "summary_file": maybe("test_results/benchmarks/simple_guidance/raw/20260318_085048_146_learned_guidance_simple_ablation_summary.csv"),
            "export_file": maybe("test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv"),
            "analysis_doc": maybe("docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260318_085048_146_085205_318_ANALYSIS.md"),
            "notes": "Top-k gating fixes the previous learned-guidance budget collapse; candidate_viable run reaches 100% success with 0 budget hits but remains slower than heuristic.",
        }
    )

    rows.append(
        {
            "importance": "high",
            "dataset_type": "ablation_benchmark",
            "dataset_key": "learned_guidance_simple_ablation_v3_preferred_gated",
            "benchmark_family": "simple_guidance",
            "timestamp": "20260318_085205_318",
            "status": "policy_fix_budget_stable",
            "results_file": maybe("test_results/benchmarks/simple_guidance/raw/20260318_085205_318_learned_guidance_simple_ablation_results.csv"),
            "summary_file": maybe("test_results/benchmarks/simple_guidance/raw/20260318_085205_318_learned_guidance_simple_ablation_summary.csv"),
            "export_file": maybe("test_results/models/guide_ranking_simple/20260317_171600_candidate_preferred_refined/linear_model.csv"),
            "analysis_doc": maybe("docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260318_085048_146_085205_318_ANALYSIS.md"),
            "notes": "The same top-k gating policy removes budget collapse for candidate_preferred too, confirming the ranking-policy fix is real but still speed-limited.",
        }
    )

    latest_simple_random_results = latest_or_blank("datasets/simple_random/raw/*_simple_random_task_dataset_results.csv")
    latest_simple_random_summary = latest_or_blank("datasets/simple_random/raw/*_simple_random_task_dataset_summary.csv")
    if latest_simple_random_results and latest_simple_random_results != "test_results/datasets/simple_random/raw/20260311_142950_774_simple_random_task_dataset_results.csv":
        rows.append(
            {
                "importance": "medium",
                "dataset_type": "random_dataset",
                "dataset_key": "simple_random_latest_unclassified",
                "benchmark_family": "simple_random",
                "timestamp": latest_simple_random_results.split("/")[-1].split("_simple_random")[0],
                "status": "needs_review",
                "results_file": latest_simple_random_results,
                "summary_file": latest_simple_random_summary,
                "export_file": "",
                "analysis_doc": "",
                "notes": "Latest random dataset exists but is not yet promoted into curated memory.",
            }
        )

    return rows


def main():
    """Main."""
    fieldnames = [
        "importance",
        "dataset_type",
        "dataset_key",
        "benchmark_family",
        "timestamp",
        "status",
        "results_file",
        "summary_file",
        "export_file",
        "analysis_doc",
        "notes",
    ]

    rows = build_rows()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT_PATH.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} rows to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
