#!/usr/bin/env python3
"""
Create a simple 2D animation for test_rrtstar_connect_reproduction output.

Default behavior:
1. Look for path_points.csv under the current test_results directory.
2. Fall back to the archived reproduction result if needed.
3. Draw both search trees when tree_nodes.csv is available.
4. Save a GIF next to the selected CSV file.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
from matplotlib.collections import LineCollection


Point = Tuple[float, float]
TreeNode = Dict[str, float | int]

# Keep this in sync with buildDefaultProblem() in
# src/tools/test_rrtstar_connect_reproduction.cpp.
DEFAULT_OBSTACLES: Sequence[Tuple[float, float, float, float]] = (
    (0.24, 0.18, 0.40, 0.74),
    (0.50, 0.00, 0.62, 0.46),
    (0.50, 0.58, 0.62, 1.00),
    (0.74, 0.30, 0.82, 0.78),
)


def parse_args() -> argparse.Namespace:
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Animate the 2D RRT*-Connect reproduction path output."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="Path to path_points.csv. Default: auto-detect current result then archived result.",
    )
    parser.add_argument(
        "--summary",
        type=Path,
        default=None,
        help="Optional summary.txt path. Default: infer from input directory.",
    )
    parser.add_argument(
        "--tree",
        type=Path,
        default=None,
        help="Optional tree_nodes.csv path. Default: infer from input directory.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output animation path. Default: <input_dir>/rrtstar_connect_2d.gif",
    )
    parser.add_argument("--fps", type=int, default=10, help="Animation FPS.")
    parser.add_argument(
        "--pause-frames",
        type=int,
        default=8,
        help="Extra hold frames after drawing the final smoothed path.",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=140,
        help="Output DPI for the saved animation.",
    )
    parser.add_argument(
        "--show-summary",
        action="store_true",
        help="Overlay summary.txt metrics when available.",
    )
    return parser.parse_args()


def detect_default_input(repo_root: Path) -> Path:
    """Detect default input."""
    candidates = (
        repo_root / "test_results" / "operations" / "rrtstar_connect_reproduction" / "path_points.csv",
        repo_root
        / "project_archive"
        / "test_results"
        / "operations"
        / "rrtstar_connect_reproduction"
        / "path_points.csv",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "No path_points.csv found in test_results/operations/rrtstar_connect_reproduction "
        "or project_archive/test_results/operations/rrtstar_connect_reproduction."
    )


def load_paths(csv_path: Path) -> Dict[str, List[Point]]:
    """Load paths."""
    paths: Dict[str, List[Point]] = {"raw": [], "smoothed": []}
    with csv_path.open("r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            path_type = row["path_type"].strip()
            if path_type not in paths:
                continue
            paths[path_type].append((float(row["x"]), float(row["y"])))
    if not paths["raw"] and not paths["smoothed"]:
        raise ValueError(f"No usable path rows found in {csv_path}")
    return paths


def load_tree_nodes(tree_path: Path | None) -> Dict[str, List[TreeNode]]:
    """Load tree nodes."""
    trees: Dict[str, List[TreeNode]] = {"start": [], "goal": []}
    if tree_path is None or not tree_path.is_file():
        return trees

    with tree_path.open("r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            tree_type = row["tree_type"].strip()
            if tree_type not in trees:
                continue
            trees[tree_type].append(
                {
                    "node_index": int(row["node_index"]),
                    "parent_index": int(row["parent_index"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "cost": float(row["cost"]),
                }
            )

    for nodes in trees.values():
        nodes.sort(key=lambda node: int(node["node_index"]))
    return trees


def load_summary(summary_path: Path | None) -> Dict[str, str]:
    """Load summary."""
    if summary_path is None or not summary_path.is_file():
        return {}
    summary: Dict[str, str] = {}
    with summary_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            summary[key.strip()] = value.strip()
    return summary


def all_points(paths: Dict[str, List[Point]], trees: Dict[str, List[TreeNode]]) -> Iterable[Point]:
    """All points."""
    for points in paths.values():
        yield from points
    for nodes in trees.values():
        for node in nodes:
            yield float(node["x"]), float(node["y"])


def axis_limits(
    paths: Dict[str, List[Point]],
    obstacles: Sequence[Tuple[float, float, float, float]],
    trees: Dict[str, List[TreeNode]],
) -> Tuple[float, float, float, float]:
    """Axis limits."""
    xs = []
    ys = []
    for x, y in all_points(paths, trees):
        xs.append(x)
        ys.append(y)
    for x_min, y_min, x_max, y_max in obstacles:
        xs.extend((x_min, x_max))
        ys.extend((y_min, y_max))
    pad = 0.06
    x_min = max(0.0, min(xs) - pad)
    x_max = min(1.0, max(xs) + pad)
    y_min = max(0.0, min(ys) - pad)
    y_max = min(1.0, max(ys) + pad)
    return x_min, x_max, y_min, y_max


def build_tree_segments(nodes: List[TreeNode]) -> List[Tuple[Point, Point]]:
    """Build tree segments."""
    node_map = {int(node["node_index"]): node for node in nodes}
    segments: List[Tuple[Point, Point]] = []
    for node in nodes:
        parent_index = int(node["parent_index"])
        if parent_index < 0 or parent_index not in node_map:
            continue
        parent = node_map[parent_index]
        segments.append(
            (
                (float(parent["x"]), float(parent["y"])),
                (float(node["x"]), float(node["y"])),
            )
        )
    return segments


def visible_node_offsets(nodes: List[TreeNode], visible_count: int) -> np.ndarray:
    """Visible node offsets."""
    if visible_count <= 0:
        return np.empty((0, 2))
    return np.asarray(
        [(float(node["x"]), float(node["y"])) for node in nodes[:visible_count]],
        dtype=float,
    )


def visible_segment_count(total_segments: int, frame_idx: int, total_frames: int) -> int:
    """Visible segment count."""
    if total_segments <= 0 or total_frames <= 0:
        return 0
    return min(total_segments, math.ceil(total_segments * frame_idx / total_frames))


def build_animation(
    fig: plt.Figure,
    ax: plt.Axes,
    paths: Dict[str, List[Point]],
    trees: Dict[str, List[TreeNode]],
    summary: Dict[str, str],
    obstacles: Sequence[Tuple[float, float, float, float]],
    pause_frames: int,
    show_summary: bool,
) -> FuncAnimation:
    """Build animation."""
    raw_points = paths["raw"]
    smoothed_points = paths["smoothed"]
    start_tree_nodes = trees["start"]
    goal_tree_nodes = trees["goal"]
    start_tree_segments = build_tree_segments(start_tree_nodes)
    goal_tree_segments = build_tree_segments(goal_tree_nodes)

    for x_min, y_min, x_max, y_max in obstacles:
        ax.add_patch(
            patches.Rectangle(
                (x_min, y_min),
                x_max - x_min,
                y_max - y_min,
                facecolor="#9ca3af",
                edgecolor="#374151",
                linewidth=1.2,
                alpha=0.75,
            )
        )

    if raw_points:
        start = raw_points[0]
        goal = raw_points[-1]
    else:
        start = smoothed_points[0]
        goal = smoothed_points[-1]

    ax.scatter([start[0]], [start[1]], s=60, c="#16a34a", label="Start", zorder=5)
    ax.scatter([goal[0]], [goal[1]], s=60, c="#dc2626", label="Goal", zorder=5)

    start_tree_collection = LineCollection(
        [],
        colors="#10b981",
        linewidths=1.0,
        alpha=0.35,
        label="Start tree",
        zorder=1,
    )
    goal_tree_collection = LineCollection(
        [],
        colors="#f59e0b",
        linewidths=1.0,
        alpha=0.35,
        label="Goal tree",
        zorder=1,
    )
    ax.add_collection(start_tree_collection)
    ax.add_collection(goal_tree_collection)
    start_tree_scatter = ax.scatter([], [], s=10, c="#059669", alpha=0.35, zorder=2)
    goal_tree_scatter = ax.scatter([], [], s=10, c="#d97706", alpha=0.35, zorder=2)

    raw_line, = ax.plot([], [], "--", color="#ef4444", linewidth=2.2, alpha=0.9, label="Raw path")
    smoothed_line, = ax.plot([], [], "-", color="#2563eb", linewidth=2.8, alpha=0.95, label="Smoothed path")
    current_point, = ax.plot([], [], "o", color="#111827", markersize=5)

    metric_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#d1d5db"},
    )

    tree_frames = max(len(start_tree_segments), len(goal_tree_segments))
    raw_frames = len(raw_points)
    smooth_frames = len(smoothed_points)
    total_frames = max(1, tree_frames + raw_frames + smooth_frames + pause_frames)

    def update(frame_idx: int):
        """Update."""
        if tree_frames > 0 and frame_idx < tree_frames:
            tree_progress = frame_idx + 1
            start_segment_count = visible_segment_count(
                len(start_tree_segments), tree_progress, tree_frames
            )
            goal_segment_count = visible_segment_count(
                len(goal_tree_segments), tree_progress, tree_frames
            )
            raw_count = 0
            smooth_count = 0
            stage = "Tree growth"
        elif frame_idx < tree_frames + raw_frames:
            start_segment_count = len(start_tree_segments)
            goal_segment_count = len(goal_tree_segments)
            raw_count = min(frame_idx - tree_frames + 1, raw_frames)
            smooth_count = 0
            stage = "Raw path growth"
        else:
            start_segment_count = len(start_tree_segments)
            goal_segment_count = len(goal_tree_segments)
            raw_count = raw_frames
            smooth_count = min(frame_idx - tree_frames - raw_frames + 1, smooth_frames)
            stage = "Bezier smoothing"

        start_tree_collection.set_segments(start_tree_segments[:start_segment_count])
        goal_tree_collection.set_segments(goal_tree_segments[:goal_segment_count])
        start_visible_nodes = min(
            len(start_tree_nodes),
            start_segment_count + (1 if start_tree_nodes else 0),
        )
        goal_visible_nodes = min(
            len(goal_tree_nodes),
            goal_segment_count + (1 if goal_tree_nodes else 0),
        )
        start_tree_scatter.set_offsets(visible_node_offsets(start_tree_nodes, start_visible_nodes))
        goal_tree_scatter.set_offsets(visible_node_offsets(goal_tree_nodes, goal_visible_nodes))

        raw_x = [p[0] for p in raw_points[:raw_count]]
        raw_y = [p[1] for p in raw_points[:raw_count]]
        smoothed_x = [p[0] for p in smoothed_points[:smooth_count]]
        smoothed_y = [p[1] for p in smoothed_points[:smooth_count]]

        raw_line.set_data(raw_x, raw_y)
        smoothed_line.set_data(smoothed_x, smoothed_y)

        if smooth_count > 0:
            current = smoothed_points[smooth_count - 1]
        elif raw_count > 0:
            current = raw_points[raw_count - 1]
        else:
            current = start
        current_point.set_data([current[0]], [current[1]])

        title = f"RRT*-Connect 2D Reproduction | {stage} | frame {frame_idx + 1}/{total_frames}"
        ax.set_title(title)

        if show_summary and summary:
            lines = []
            for key in (
                "iterations",
                "start_tree_nodes",
                "goal_tree_nodes",
                "raw_path_length",
                "smoothed_path_length",
                "sampling_success_ratio",
            ):
                if key in summary:
                    lines.append(f"{key}: {summary[key]}")
            metric_text.set_text("\n".join(lines))
        else:
            metric_text.set_text("")

        return (
            start_tree_collection,
            goal_tree_collection,
            start_tree_scatter,
            goal_tree_scatter,
            raw_line,
            smoothed_line,
            current_point,
            metric_text,
        )

    return FuncAnimation(fig, update, frames=total_frames, interval=100, blit=False, repeat=True)


def save_animation(animation: FuncAnimation, output_path: Path, fps: int, dpi: int) -> None:
    """Save animation."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    suffix = output_path.suffix.lower()
    if suffix == ".gif":
        writer = PillowWriter(fps=fps)
    else:
        writer = FFMpegWriter(fps=fps)
    animation.save(output_path, writer=writer, dpi=dpi)


def main() -> int:
    """Main."""
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[2]

    input_path = args.input if args.input is not None else detect_default_input(repo_root)
    input_path = input_path.resolve()
    summary_path = args.summary if args.summary is not None else input_path.with_name("summary.txt")
    tree_path = args.tree if args.tree is not None else input_path.with_name("tree_nodes.csv")
    output_path = (
        args.output.resolve()
        if args.output is not None
        else input_path.with_name("rrtstar_connect_2d.gif").resolve()
    )

    paths = load_paths(input_path)
    trees = load_tree_nodes(tree_path)
    summary = load_summary(summary_path)

    fig, ax = plt.subplots(figsize=(7.2, 7.2))
    x_min, x_max, y_min, y_max = axis_limits(paths, DEFAULT_OBSTACLES, trees)
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", linewidth=0.6, alpha=0.4)
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    animation = build_animation(
        fig=fig,
        ax=ax,
        paths=paths,
        trees=trees,
        summary=summary,
        obstacles=DEFAULT_OBSTACLES,
        pause_frames=args.pause_frames,
        show_summary=args.show_summary,
    )
    ax.legend(loc="lower right")
    save_animation(animation, output_path, fps=args.fps, dpi=args.dpi)
    plt.close(fig)

    print(f"input={input_path}")
    print(f"summary={summary_path if summary_path.is_file() else 'N/A'}")
    print(f"tree={tree_path if tree_path.is_file() else 'N/A'}")
    print(f"output={output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
