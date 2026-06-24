#!/usr/bin/env python3
"""Plot five-branch probe calibration residuals."""

import argparse
import csv
import math
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


BRANCH_ORDER = ("z", "x_pos", "x_neg", "y_pos", "y_neg")


def read_csv(path):
    """Read csv."""
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def residual_rows(path):
    """Residual rows."""
    rows = []
    for row in read_csv(path):
        branch = row.get("branch", "")
        value = row.get("residual_norm_mm", "")
        if not branch or value in ("", None):
            continue
        try:
            residual = float(value)
        except ValueError:
            continue
        if math.isfinite(residual):
            rows.append({**row, "_residual_norm_mm": residual})
    return rows


def contact_status_counts(path):
    """Contact status counts."""
    if path is None:
        return Counter()
    rows = read_csv(path)
    return Counter((row.get("branch", ""), row.get("status", "")) for row in rows)


def rms(values):
    """Rms."""
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def branch_stats(rows):
    """Branch stats."""
    grouped = defaultdict(list)
    for row in rows:
        if row.get("outlier_status", "inlier") == "outlier":
            continue
        grouped[row["branch"]].append(row["_residual_norm_mm"])
    result = {}
    for branch in BRANCH_ORDER:
        values = grouped.get(branch, [])
        result[branch] = {
            "count": len(values),
            "rms": rms(values),
            "max": max(values) if values else 0.0,
            "mean": sum(values) / len(values) if values else 0.0,
        }
    return result


def plot(args):
    """Plot residual values per branch and generate the output figure."""
    rows = residual_rows(args.residual_csv)
    if not rows:
        raise ValueError(f"{args.residual_csv}: no residual_norm_mm rows")
    stats = branch_stats(rows)
    contact_counts = contact_status_counts(args.contacts_csv)

    branches = list(BRANCH_ORDER)
    x = list(range(len(branches)))
    rms_values = [stats[branch]["rms"] for branch in branches]
    max_values = [stats[branch]["max"] for branch in branches]

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), constrained_layout=True)
    fig.suptitle(args.title)

    ax = axes[0]
    width = 0.36
    ax.bar([value - width / 2 for value in x], rms_values, width=width, label="RMS")
    ax.bar([value + width / 2 for value in x], max_values, width=width, label="Max")
    ax.set_xticks(x, branches)
    ax.set_ylabel("Residual norm (mm)")
    ax.grid(axis="y", alpha=0.3)
    ax.legend()
    for index, branch in enumerate(branches):
        count = stats[branch]["count"]
        hit = contact_counts.get((branch, "HIT"), 0)
        miss = contact_counts.get((branch, "MISS"), 0)
        label = f"n={count}"
        if args.contacts_csv:
            label += f"\nHIT={hit} MISS={miss}"
        ax.text(index, max(max_values[index], rms_values[index]) + 0.01, label, ha="center", va="bottom", fontsize=9)
    top = max(max(max_values), max(rms_values), 0.01)
    ax.set_ylim(0.0, top * 1.22 + 0.03)

    ax = axes[1]
    branch_offsets = {branch: index for index, branch in enumerate(branches)}
    for branch in branches:
        branch_rows = [row for row in rows if row["branch"] == branch]
        for point_index, row in enumerate(branch_rows, start=1):
            residual = row["_residual_norm_mm"]
            status = row.get("outlier_status", "inlier")
            color = "tab:red" if status == "outlier" else "tab:blue"
            marker = "x" if status == "outlier" else "o"
            ax.scatter(branch_offsets[branch] + (point_index - 1) * 0.035, residual, color=color, marker=marker, s=36)
    ax.set_xticks(x, branches)
    ax.set_ylabel("Per-point residual norm (mm)")
    ax.set_xlabel("Branch")
    ax.grid(axis="y", alpha=0.3)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=args.dpi)
    print(f"saved residual plot: {output}")
    """Parse args."""


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--residual-csv", required=True)
    parser.add_argument("--contacts-csv")
    parser.add_argument("--output", required=True)
    parser.add_argument("--title", default="Five-branch calibration residuals")
    parser.add_argument("--dpi", type=int, default=160)
    return parser.parse_args()


def main():
    """Main."""
    plot(parse_args())


if __name__ == "__main__":
    main()
