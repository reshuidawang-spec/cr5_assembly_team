#!/usr/bin/env python3
"""Evaluate a planned compensation probing dataset."""

import argparse
import csv
import json
import math
import time
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


BRANCH_ORDER = ("z", "x_pos", "x_neg", "y_pos", "y_neg")
POSE_NAMES = ("x", "y", "z", "rx", "ry", "rz")
RESIDUAL_FIELDS = ("residual_x", "residual_y", "residual_z")


def read_csv(path):
    """Read CSV rows."""
    if not path:
        return []
    path = Path(path)
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    """Write CSV rows."""
    if not rows:
        return
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen:
                fields.append(key)
                seen.add(key)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fields})


def to_float(value, default=None):
    """Parse a finite float, returning default on failure."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def pose(row, prefix):
    """Read a pose from row fields."""
    values = [to_float(row.get(f"{prefix}_{name}")) for name in POSE_NAMES]
    return None if any(value is None for value in values) else values


def xyz(p):
    """Return xyz part."""
    return p[:3]


def sub(a, b):
    """Subtract vectors."""
    return [float(x) - float(y) for x, y in zip(a, b)]


def dot(a, b):
    """Dot product."""
    return sum(float(x) * float(y) for x, y in zip(a, b))


def norm(v):
    """Vector norm."""
    return math.sqrt(dot(v, v))


def rms(values):
    """Root mean square."""
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def last_by_plan_id(rows):
    """Return last row by plan_id and duplicate counts."""
    result = {}
    counts = Counter()
    for row in rows:
        plan_id = row.get("plan_id", "")
        if not plan_id:
            continue
        counts[plan_id] += 1
        result[plan_id] = row
    return result, counts


def planned_travel_mm(plan_row):
    """Probe travel from plan row."""
    value = to_float(plan_row.get("probe_travel_mm"))
    if value is not None:
        return value
    start = pose(plan_row, "start_flange")
    target = pose(plan_row, "target_flange")
    if start is None or target is None:
        return None
    return norm(sub(xyz(target), xyz(start)))


def progress_from_pose(plan_row, measured_pose):
    """Return progress in mm and normalized ratio along the planned probe vector."""
    if measured_pose is None:
        return None, None
    start = pose(plan_row, "start_flange")
    target = pose(plan_row, "target_flange")
    travel = planned_travel_mm(plan_row)
    if start is None or target is None or travel is None or travel <= 1e-9:
        return None, None
    direction = sub(xyz(target), xyz(start))
    length = norm(direction)
    if length <= 1e-9:
        return None, None
    unit = [value / length for value in direction]
    progress_mm = dot(sub(xyz(measured_pose), xyz(start)), unit)
    return progress_mm, progress_mm / travel


def residual_by_plan_id(rows):
    """Return residual rows by plan_id."""
    result = {}
    for row in rows:
        plan_id = row.get("plan_id", "")
        if plan_id:
            result[plan_id] = row
    return result


def classify(row, args):
    """Classify a joined quality row."""
    status = row.get("status", "NOT_RUN")
    if status != "HIT":
        return "FAIL"
    residual = to_float(row.get("residual_norm_mm"))
    if residual is not None and residual > args.max_residual_mm:
        return "FAIL"
    if row.get("outlier_status") == "outlier":
        return "FAIL"
    progress = to_float(row.get("trigger_progress_ratio"))
    if progress is None:
        return "WARN"
    if progress < args.min_trigger_progress or progress > args.max_trigger_progress:
        return "WARN"
    return "PASS"


def join_rows(args):
    """Join plan, contacts, and residuals into point-level quality rows."""
    plan_rows = read_csv(args.plan_csv)
    contact_rows, duplicate_counts = last_by_plan_id(read_csv(args.contacts_csv))
    residual_rows = residual_by_plan_id(read_csv(args.residual_csv))
    joined = []
    for index, plan_row in enumerate(plan_rows, start=1):
        plan_id = plan_row.get("plan_id", "")
        contact = contact_rows.get(plan_id, {})
        residual = residual_rows.get(plan_id, {})
        status = contact.get("status") or "NOT_RUN"
        trigger_pose = pose(contact, "trigger_flange")
        stop_pose = pose(contact, "stop_flange")
        trigger_mm, trigger_ratio = progress_from_pose(plan_row, trigger_pose)
        stop_mm, stop_ratio = progress_from_pose(plan_row, stop_pose)
        overtravel = None
        if trigger_mm is not None and stop_mm is not None:
            overtravel = stop_mm - trigger_mm
        row = {
            "plan_index": str(index),
            "plan_id": plan_id,
            "branch": plan_row.get("branch", ""),
            "dataset_group": plan_row.get("dataset_group", ""),
            "dataset_split": plan_row.get("dataset_split", ""),
            "dataset_branch_point_index": plan_row.get("dataset_branch_point_index", ""),
            "dataset_repeat_index": plan_row.get("dataset_repeat_index", ""),
            "status": status,
            "contact_duplicate_count": str(duplicate_counts.get(plan_id, 0)),
            "probe_travel_mm": fmt(planned_travel_mm(plan_row)),
            "trigger_progress_mm": fmt(trigger_mm),
            "trigger_progress_ratio": fmt(trigger_ratio),
            "stop_progress_mm": fmt(stop_mm),
            "stop_progress_ratio": fmt(stop_ratio),
            "stop_after_trigger_mm": fmt(overtravel),
            "approach_x": plan_row.get("approach_x", ""),
            "approach_y": plan_row.get("approach_y", ""),
            "approach_z": plan_row.get("approach_z", ""),
            "moveit_status": plan_row.get("moveit_status", ""),
            "min_probe_clearance_mm": plan_row.get("min_probe_clearance_mm", ""),
            "residual_norm_mm": residual.get("residual_norm_mm", ""),
            "residual_x": residual.get("residual_x", ""),
            "residual_y": residual.get("residual_y", ""),
            "residual_z": residual.get("residual_z", ""),
            "outlier_status": residual.get("outlier_status", ""),
            "p_sample_distance_to_median_mm": residual.get("p_sample_distance_to_median_mm", ""),
        }
        row["quality_status"] = classify(row, args)
        joined.append(row)
    return joined


def fmt(value):
    """Format optional float."""
    return "" if value is None else f"{float(value):.6f}"


def branch_summary(rows):
    """Build branch summary stats."""
    summary = {}
    for branch in BRANCH_ORDER:
        branch_rows = [row for row in rows if row.get("branch") == branch]
        residuals = [
            to_float(row.get("residual_norm_mm"))
            for row in branch_rows
            if row.get("outlier_status") != "outlier"
        ]
        residuals = [value for value in residuals if value is not None]
        trigger_ratios = [
            to_float(row.get("trigger_progress_ratio"))
            for row in branch_rows
            if row.get("status") == "HIT"
        ]
        trigger_ratios = [value for value in trigger_ratios if value is not None]
        status_counts = Counter(row.get("status", "NOT_RUN") for row in branch_rows)
        quality_counts = Counter(row.get("quality_status", "") for row in branch_rows)
        summary[branch] = {
            "planned": len(branch_rows),
            "hit": status_counts.get("HIT", 0),
            "miss": status_counts.get("MISS", 0),
            "not_run": status_counts.get("NOT_RUN", 0),
            "early_trigger": sum(count for status, count in status_counts.items() if status.startswith("EARLY")),
            "quality_pass": quality_counts.get("PASS", 0),
            "quality_warn": quality_counts.get("WARN", 0),
            "quality_fail": quality_counts.get("FAIL", 0),
            "residual_rms_mm": rms(residuals),
            "residual_max_mm": max(residuals) if residuals else 0.0,
            "residual_mean_mm": sum(residuals) / len(residuals) if residuals else 0.0,
            "trigger_progress_mean": sum(trigger_ratios) / len(trigger_ratios) if trigger_ratios else 0.0,
            "trigger_progress_min": min(trigger_ratios) if trigger_ratios else 0.0,
            "trigger_progress_max": max(trigger_ratios) if trigger_ratios else 0.0,
        }
    return summary


def plot_evaluation(rows, summary, args):
    """Create a multi-panel evaluation plot."""
    branches = list(BRANCH_ORDER)
    x = list(range(len(branches)))
    fig = plt.figure(figsize=(14, 12), constrained_layout=True)
    fig.suptitle(args.title)
    axes = [fig.add_subplot(3, 2, i + 1) for i in range(6)]

    ax = axes[0]
    bottoms = [0] * len(branches)
    for label, color in (("hit", "tab:green"), ("miss", "tab:orange"), ("not_run", "0.7"), ("early_trigger", "tab:red")):
        values = [summary[branch][label] for branch in branches]
        ax.bar(x, values, bottom=bottoms, label=label, color=color)
        bottoms = [bottom + value for bottom, value in zip(bottoms, values)]
    ax.set_xticks(x, branches)
    ax.set_ylabel("Point count")
    ax.set_title("Execution status")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)

    ax = axes[1]
    width = 0.36
    rms_values = [summary[branch]["residual_rms_mm"] for branch in branches]
    max_values = [summary[branch]["residual_max_mm"] for branch in branches]
    ax.bar([value - width / 2 for value in x], rms_values, width=width, label="RMS")
    ax.bar([value + width / 2 for value in x], max_values, width=width, label="Max")
    ax.axhline(args.max_residual_mm, color="tab:red", linestyle="--", linewidth=1, label="limit")
    ax.set_xticks(x, branches)
    ax.set_ylabel("Residual norm (mm)")
    ax.set_title("Residual summary")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)

    ax = axes[2]
    branch_index = {branch: idx for idx, branch in enumerate(branches)}
    for row in rows:
        residual = to_float(row.get("residual_norm_mm"))
        if residual is None:
            continue
        branch = row.get("branch")
        offset = branch_index.get(branch, 0)
        point = to_float(row.get("dataset_branch_point_index"), 0) or 0
        color = {
            "PASS": "tab:blue",
            "WARN": "tab:orange",
            "FAIL": "tab:red",
        }.get(row.get("quality_status"), "0.5")
        marker = "x" if row.get("outlier_status") == "outlier" else "o"
        ax.scatter(offset + point * 0.015, residual, color=color, marker=marker, s=24)
    ax.axhline(args.max_residual_mm, color="tab:red", linestyle="--", linewidth=1)
    ax.set_xticks(x, branches)
    ax.set_ylabel("Residual norm (mm)")
    ax.set_title("Per-point residuals")
    ax.grid(axis="y", alpha=0.3)

    ax = axes[3]
    component_colors = {"residual_x": "tab:red", "residual_y": "tab:green", "residual_z": "tab:blue"}
    for row in rows:
        branch = row.get("branch")
        offset = branch_index.get(branch, 0)
        point = to_float(row.get("dataset_branch_point_index"), 0) or 0
        for component, color in component_colors.items():
            value = to_float(row.get(component))
            if value is not None:
                ax.scatter(offset + point * 0.015, value, color=color, s=12, alpha=0.75)
    ax.axhline(0.0, color="0.2", linewidth=1)
    ax.set_xticks(x, branches)
    ax.set_ylabel("Residual component (mm)")
    ax.set_title("Residual components")
    ax.grid(axis="y", alpha=0.3)

    ax = axes[4]
    for row in rows:
        progress = to_float(row.get("trigger_progress_ratio"))
        if progress is None:
            continue
        branch = row.get("branch")
        offset = branch_index.get(branch, 0)
        point = to_float(row.get("dataset_branch_point_index"), 0) or 0
        color = "tab:blue" if row.get("quality_status") == "PASS" else "tab:orange"
        ax.scatter(offset + point * 0.015, progress, color=color, s=24)
    ax.axhline(args.min_trigger_progress, color="tab:red", linestyle="--", linewidth=1)
    ax.axhline(args.max_trigger_progress, color="tab:red", linestyle="--", linewidth=1)
    ax.set_xticks(x, branches)
    ax.set_ylabel("Trigger progress ratio")
    ax.set_title("Trigger location in probe segment")
    ax.grid(axis="y", alpha=0.3)

    ax = axes[5]
    colors = {
        "z": "tab:blue",
        "x_pos": "tab:orange",
        "x_neg": "tab:green",
        "y_pos": "tab:red",
        "y_neg": "tab:purple",
    }
    for row in rows:
        nx = to_float(row.get("approach_x"))
        ny = to_float(row.get("approach_y"))
        nz = to_float(row.get("approach_z"))
        if nx is None or ny is None:
            continue
        branch = row.get("branch")
        marker = "s" if row.get("dataset_group") == "ctrl" else "o"
        alpha = 0.5 if row.get("status") == "NOT_RUN" else 0.9
        ax.scatter(nx, ny, s=24, marker=marker, color=colors.get(branch, "0.5"), alpha=alpha)
        if nz is not None and abs(nz) > 0.92:
            ax.annotate(branch, (nx, ny), fontsize=6, alpha=0.7)
    ax.set_xlabel("approach_x")
    ax.set_ylabel("approach_y")
    ax.set_title("Approach-normal coverage")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="box")

    output = Path(args.plot_output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=args.dpi)
    print(f"saved evaluation plot: {output}")


def parse_args():
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan-csv", required=True)
    parser.add_argument("--contacts-csv", required=True)
    parser.add_argument("--residual-csv", required=True)
    parser.add_argument("--quality-csv", required=True)
    parser.add_argument("--summary-json", required=True)
    parser.add_argument("--plot-output", required=True)
    parser.add_argument("--title", default="Five-branch compensation dataset evaluation")
    parser.add_argument("--max-residual-mm", type=float, default=0.20)
    parser.add_argument("--min-trigger-progress", type=float, default=0.05)
    parser.add_argument("--max-trigger-progress", type=float, default=0.95)
    parser.add_argument("--dpi", type=int, default=160)
    return parser.parse_args()


def main():
    """Evaluate and plot a compensation dataset."""
    args = parse_args()
    rows = join_rows(args)
    if not rows:
        raise SystemExit("no plan rows to evaluate")
    summary = {
        "created_at_unix": time.time(),
        "plan_csv": str(args.plan_csv),
        "contacts_csv": str(args.contacts_csv),
        "residual_csv": str(args.residual_csv),
        "thresholds": {
            "max_residual_mm": args.max_residual_mm,
            "min_trigger_progress": args.min_trigger_progress,
            "max_trigger_progress": args.max_trigger_progress,
        },
        "total": {
            "planned": len(rows),
            "hit": sum(1 for row in rows if row.get("status") == "HIT"),
            "pass": sum(1 for row in rows if row.get("quality_status") == "PASS"),
            "warn": sum(1 for row in rows if row.get("quality_status") == "WARN"),
            "fail": sum(1 for row in rows if row.get("quality_status") == "FAIL"),
        },
        "branches": branch_summary(rows),
    }
    write_csv(args.quality_csv, rows)
    output = Path(args.summary_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2) + "\n")
    plot_evaluation(rows, summary["branches"], args)
    print(f"saved quality CSV: {args.quality_csv}")
    print(f"saved summary JSON: {args.summary_json}")


if __name__ == "__main__":
    main()
