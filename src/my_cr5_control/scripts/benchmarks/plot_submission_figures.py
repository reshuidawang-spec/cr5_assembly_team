#!/usr/bin/env python3
"""Generate manuscript figures from frozen formal results and trajectory exports."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import struct

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, FancyArrowPatch, FancyBboxPatch, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
MAIN_ROOT = ROOT / "paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832"
ABL_ROOT = ROOT / "paper_workspace/formal_results/q2_ablation_formal_20260511_104104"
DIRECT_ROOT = ROOT / "paper_workspace/formal_results/q2_ablation_formal_direct_only_completion_20260511_111548"
TRAJ_ROOT = ROOT / "paper_workspace/qualitative_results/v2_ws119_trajectory_20260513_153328"
MESH_PATH = ROOT / "src/meshes/WS119.STL"
OUT = ROOT / "paper_workspace/manuscript/latex/figures"

PLANNERS = ["HeuristicGuided", "RRTConnect", "RRTstar", "LBTRRT", "PRMstar"]
MODES = ["full", "direct_only", "fixed_guide", "no_anchors", "no_adaptive_difficulty", "always_guide"]
MODE_LABELS = ["Full", "Direct\nonly", "Fixed\nguide", "No\nanchors", "No\nadaptive", "Always\nguide"]
COLORS = {
    "HeuristicGuided": "#b45309",
    "RRTConnect": "#2563eb",
    "RRTstar": "#64748b",
    "LBTRRT": "#0f766e",
    "PRMstar": "#7c3aed",
    "simple": "#2563eb",
    "v2": "#b45309",
}


def rows(path: Path) -> list[dict[str, str]]:
    """Rows."""
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def scalar(row: dict[str, str], key: str) -> float:
    """Scalar."""
    return float(row[key])


def save(fig: plt.Figure, stem: str) -> None:
    """Save."""
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(OUT / f"{stem}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)


def setup_style() -> None:
    """Setup style."""
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def rounded_box(ax: plt.Axes, xy: tuple[float, float], width: float, height: float,
                text: str, facecolor: str, edgecolor: str = "#475569") -> None:
    """Rounded box."""
    ax.add_patch(
        FancyBboxPatch(
            xy, width, height, boxstyle="round,pad=0.025,rounding_size=0.05",
            linewidth=1.0, edgecolor=edgecolor, facecolor=facecolor
        )
    )
    ax.text(xy[0] + width / 2, xy[1] + height / 2, text, ha="center", va="center",
            fontsize=8.4, color="#111827")


def arrow(ax: plt.Axes, start: tuple[float, float], end: tuple[float, float]) -> None:
    """Arrow."""
    ax.add_patch(
        FancyArrowPatch(start, end, arrowstyle="-|>", mutation_scale=12,
                        linewidth=1.0, color="#475569")
    )


def plot_method_pipeline() -> None:
    """Plot method pipeline."""
    fig, ax = plt.subplots(figsize=(14.0, 3.6))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 4)
    ax.axis("off")
    rounded_box(ax, (0.15, 1.48), 1.65, 1.0, "Start pose,\ntarget pose,\nenvironment", "#e2e8f0")
    rounded_box(ax, (2.22, 1.48), 1.45, 1.0, "Geometry\ncondition d", "#dbeafe", "#2563eb")
    rounded_box(ax, (4.10, 2.48), 2.05, 0.85, "Ellipsoid samples", "#fef3c7", "#b45309")
    rounded_box(ax, (4.10, 1.35), 2.05, 0.85, "Structure anchors", "#fef3c7", "#b45309")
    rounded_box(ax, (4.10, 0.22), 2.05, 0.85, "Local refinement", "#fef3c7", "#b45309")
    rounded_box(ax, (6.62, 1.48), 1.80, 1.0, "Collision / IK /\nquality ranking", "#dcfce7", "#15803d")
    rounded_box(ax, (8.91, 1.48), 1.62, 1.0, "Selective gate\n(d >= 0.70?)", "#dbeafe", "#2563eb")
    rounded_box(ax, (11.05, 2.37), 1.53, 0.83, "Guide-first\nprecheck", "#ffedd5", "#c2410c")
    rounded_box(ax, (11.05, 0.78), 1.53, 0.83, "Direct plan", "#e2e8f0")
    rounded_box(ax, (12.93, 1.48), 0.90, 1.0, "MoveIt /\nOMPL", "#dcfce7", "#15803d")
    arrow(ax, (1.80, 1.98), (2.22, 1.98))
    for y in (2.90, 1.77, 0.64):
        arrow(ax, (3.67, 1.98), (4.10, y))
        arrow(ax, (6.15, y), (6.62, 1.98))
    arrow(ax, (8.42, 1.98), (8.91, 1.98))
    arrow(ax, (10.53, 2.15), (11.05, 2.78))
    arrow(ax, (10.53, 1.80), (11.05, 1.19))
    arrow(ax, (12.58, 2.78), (12.93, 2.14))
    arrow(ax, (12.58, 1.19), (12.93, 1.82))
    ax.text(10.66, 2.55, "yes", fontsize=8, color="#475569")
    ax.text(10.60, 1.18, "no / fallback", fontsize=8, color="#475569")
    ax.text(5.14, 3.62, "Guidance layer", ha="center", fontsize=10, fontweight="bold", color="#92400e")
    ax.text(13.38, 2.72, "Backend", ha="center", fontsize=10, fontweight="bold", color="#166534")
    save(fig, "figure_01_method_pipeline")


def plot_guide_geometry() -> None:
    """Plot guide geometry."""
    rng = np.random.default_rng(119)
    fig, ax = plt.subplots(figsize=(6.2, 4.5))
    ax.set_aspect("equal")
    ax.set_xlim(-0.4, 10.6)
    ax.set_ylim(-0.3, 6.2)
    ax.axis("off")
    ax.add_patch(Rectangle((7.2, -0.2), 3.2, 2.35, facecolor="#cbd5e1", edgecolor="#64748b"))
    ax.add_patch(Rectangle((7.2, 3.65), 3.2, 2.35, facecolor="#cbd5e1", edgecolor="#64748b"))
    ax.text(8.95, 5.0, "workpiece", ha="center", color="#334155")
    ax.text(8.50, 2.85, "restricted\nopening", ha="center", va="center", color="#334155")
    ell = Ellipse((4.9, 3.0), 7.0, 3.2, angle=0, fill=True, facecolor="#fef3c7",
                  alpha=0.5, edgecolor="#b45309", linewidth=1.2, linestyle="--")
    ax.add_patch(ell)
    candidate_x = rng.uniform(2.0, 7.2, 22)
    candidate_y = 3.0 + rng.normal(0.0, 0.65, 22)
    inside = ((candidate_x - 4.9) / 3.5) ** 2 + ((candidate_y - 3.0) / 1.6) ** 2 <= 1
    ax.scatter(candidate_x[inside], candidate_y[inside], s=18, color="#d97706", alpha=0.70,
               label="ellipsoid samples")
    anchors = np.array([[6.0, 3.0], [6.7, 3.0], [7.15, 3.0]])
    ax.scatter(anchors[:, 0], anchors[:, 1], s=58, marker="D", color="#dc2626",
               label="structure anchors", zorder=4)
    ax.scatter([1.35], [3.0], s=68, color="#2563eb", zorder=5)
    ax.scatter([9.0], [3.0], s=78, marker="*", color="#16a34a", zorder=5)
    ax.text(1.10, 3.40, "start", color="#1d4ed8")
    ax.text(8.62, 3.62, "target", color="#15803d")
    ax.plot([1.4, 9.0], [3.0, 3.0], color="#94a3b8", linestyle=":", linewidth=1.0)
    for left, right in zip([[1.42, 3.0], *anchors[:-1]], [*anchors, [8.86, 3.0]]):
        arrow(ax, tuple(left), tuple(right))
    ax.annotate("approach direction", xy=(8.0, 2.65), xytext=(4.5, 1.05),
                arrowprops={"arrowstyle": "->", "color": "#475569"}, color="#334155")
    ax.legend(loc="upper left", frameon=False, fontsize=8)
    save(fig, "figure_02_guide_geometry")


def load_ws119_mesh(step: int = 54) -> np.ndarray:
    """Load ws119 mesh."""
    with MESH_PATH.open("rb") as handle:
        handle.read(80)
        triangle_count = struct.unpack("<I", handle.read(4))[0]
        record = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        triangles = np.fromfile(handle, dtype=record, count=triangle_count)["vertices"][::step].astype(float)
    raw_min = np.array([3.394216299057007, 0.029385089874267578, 0.0050067901611328125])
    raw_max = np.array([752.5775756835938, 878.5293579101562, 1028.692138671875])
    scale = 0.00025
    offset = np.array([0.45, 0.0, 0.15]) - np.array([
        0.5 * (raw_min[0] + raw_max[0]) * scale,
        0.5 * (raw_min[1] + raw_max[1]) * scale,
        raw_min[2] * scale,
    ])
    return triangles * scale + offset


def equal_3d(ax: plt.Axes, points: np.ndarray, padding: float = 0.08) -> None:
    """Equal 3d."""
    mins = points.reshape(-1, 3).min(axis=0)
    maxs = points.reshape(-1, 3).max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = 0.5 * (maxs - mins).max() + padding
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def simple_target_points() -> list[tuple[str, np.ndarray, float]]:
    """Simple target points."""
    return [
        ("Open top", np.array([0.45, 0.00, 0.40]), 0.30),
        ("Side", np.array([0.41, 0.11, 0.26]), 0.50),
        ("Angled", np.array([0.56, 0.00, 0.30]), 0.60),
        ("Shallow", np.array([0.45, 0.00, 0.31]), 0.70),
        ("Edge offset", np.array([0.42, 0.03, 0.28]), 0.80),
        ("Deep", np.array([0.45, 0.00, 0.25]), 0.90),
    ]


def unique_v2_targets() -> list[tuple[str, np.ndarray, float]]:
    """Unique v2 targets."""
    raw = load_raw_groups_with_rows("v2")
    scores = {"Easy_HoleCenter": 0.30, "Medium_HoleEdge": 0.55,
              "Hard_DeepInterior": 0.82, "Extreme_NarrowPassage": 0.97}
    labels = {"Easy_HoleCenter": "Center", "Medium_HoleEdge": "Edge",
              "Hard_DeepInterior": "Deep", "Extreme_NarrowPassage": "Narrow"}
    targets: list[tuple[str, np.ndarray, float]] = []
    seen: set[str] = set()
    for row in raw:
        scene = row["场景名称"]
        if row["规划器"] != "HeuristicGuided" or scene in seen:
            continue
        seen.add(scene)
        position = np.array([scalar(row, "尖端X"), scalar(row, "尖端Y"), scalar(row, "尖端Z")])
        targets.append((labels[scene], position, scores[scene]))
    return targets


def color_for_difficulty(score: float) -> str:
    """Color for difficulty."""
    return plt.cm.YlOrRd(0.28 + 0.66 * score)


def draw_simple_geometry(ax: plt.Axes) -> None:
    """Draw simple geometry."""
    x = np.array([0.35, 0.55])
    y = np.array([-0.10, 0.10])
    top = 0.35
    bottom = 0.15
    xx, yy = np.meshgrid(np.linspace(x[0], x[1], 24), np.linspace(y[0], y[1], 24))
    zz = np.full_like(xx, top)
    mask = (xx - 0.45) ** 2 + yy ** 2 < 0.04 ** 2
    zz[mask] = np.nan
    ax.plot_surface(xx, yy, zz, color="#b8c6d9", alpha=0.52, linewidth=0)
    for side_x in x:
        yy2, zz2 = np.meshgrid(y, np.array([bottom, top]))
        ax.plot_surface(np.full_like(yy2, side_x), yy2, zz2, color="#cbd5e1", alpha=0.32, linewidth=0)
    theta = np.linspace(0, 2 * np.pi, 50)
    for z in (top, top - 0.12):
        ax.plot(0.45 + 0.04 * np.cos(theta), 0.04 * np.sin(theta), z,
                color="#475569", linewidth=0.8)
    for label, point, score in simple_target_points():
        ax.scatter(*point, s=40, color=color_for_difficulty(score), edgecolor="#7c2d12", depthshade=False)
        ax.text(*(point + np.array([0.004, 0.002, 0.005])), label, fontsize=6.4)


def load_raw_groups_with_rows(benchmark: str) -> list[dict[str, str]]:
    """Load raw groups with rows."""
    path = MAIN_ROOT / f"results/benchmarks/{benchmark}/aggregates/planner_comparison_{benchmark}_plot_data_metrics.csv"
    return rows(path)


def add_mesh(ax: plt.Axes, mesh: np.ndarray, alpha: float = 0.28) -> None:
    """Add mesh."""
    ax.add_collection3d(Poly3DCollection(mesh, facecolor="#94a3b8", edgecolor="none", alpha=alpha))
    vertices = mesh.reshape(-1, 3)
    ax.scatter(vertices[:, 0], vertices[:, 1], vertices[:, 2], color="#64748b",
               s=0.25, alpha=min(0.20, alpha), depthshade=False)


def plot_benchmark_scenes() -> None:
    """Plot benchmark scenes."""
    mesh = load_ws119_mesh(step=18)
    fig = plt.figure(figsize=(12.0, 5.4))
    left = fig.add_subplot(121, projection="3d")
    right = fig.add_subplot(122, projection="3d")
    draw_simple_geometry(left)
    simple_points = np.array([point for _, point, _ in simple_target_points()])
    equal_3d(left, simple_points, padding=0.06)
    left.view_init(elev=26, azim=-54)
    left.set_title("(a) Controlled simple benchmark")
    add_mesh(right, mesh, alpha=0.34)
    targets = unique_v2_targets()
    for label, point, score in targets:
        right.scatter(*point, s=44, color=color_for_difficulty(score), edgecolor="#7c2d12", depthshade=False)
        right.text(*(point + np.array([0.004, 0.002, 0.004])), label, fontsize=7)
    equal_3d(right, mesh, padding=0.01)
    right.view_init(elev=25, azim=-58)
    right.set_title("(b) Fixed STL v2/WS119 benchmark")
    for ax in (left, right):
        ax.set_xlabel("x (m)", labelpad=-2)
        ax.set_ylabel("y (m)", labelpad=-2)
        ax.set_zlabel("z (m)", labelpad=-2)
        ax.tick_params(labelsize=7, pad=0)
    fig.text(0.50, 0.02, "Marker color indicates increasing geometry-conditioned difficulty",
             ha="center", fontsize=9, color="#475569")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    save(fig, "figure_03_benchmark_scenes")


def load_main_summary(benchmark: str) -> dict[str, dict[str, str]]:
    """Load main summary."""
    path = MAIN_ROOT / f"results/benchmarks/{benchmark}/aggregates/planner_comparison_{benchmark}_plot_summary_metrics.csv"
    return {row["规划器"]: row for row in rows(path)}


def plot_main_comparison() -> None:
    """Plot main comparison."""
    simple = load_main_summary("simple")
    v2 = load_main_summary("v2")
    x = np.arange(len(PLANNERS))
    width = 0.36
    fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.6))
    for source, offset, label, color in ((simple, -width / 2, "simple", "#2563eb"),
                                          (v2, width / 2, "v2/WS119", "#b45309")):
        means = [scalar(source[p], "平均时间(ms)") for p in PLANNERS]
        hits = [scalar(source[p], "预算命中率(%)") for p in PLANNERS]
        axes[0].bar(x + offset, means, width, label=label, color=color, alpha=0.86)
        axes[1].bar(x + offset, hits, width, label=label, color=color, alpha=0.86)
    axes[0].set_yscale("log")
    axes[0].set_ylabel("Mean planning time (ms, log scale)")
    axes[1].set_ylabel("Budget-hit rate (%)")
    axes[0].set_title("(a) Mean planning time")
    axes[1].set_title("(b) Budget-hit rate")
    for ax in axes:
        ax.set_xticks(x, ["Ours", "RRT-\nConnect", "RRT*", "LBTRRT", "PRM*"])
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend(loc="upper left")
    fig.tight_layout()
    save(fig, "figure_04_main_comparison")


def load_raw_groups(benchmark: str) -> dict[str, list[float]]:
    """Load raw groups."""
    grouped = {planner: [] for planner in PLANNERS}
    for row in load_raw_groups_with_rows(benchmark):
        planner = row["规划器"]
        if planner in grouped:
            grouped[planner].append(scalar(row, "墙钟时间(ms)"))
    return grouped


def plot_wall_time_distribution() -> None:
    """Plot wall time distribution."""
    fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.8), sharey=True)
    labels = ["Ours", "RRT-\nConnect", "RRT*", "LBTRRT", "PRM*"]
    for ax, benchmark, title in zip(axes, ["simple", "v2"], ["(a) simple", "(b) v2/WS119"]):
        grouped = load_raw_groups(benchmark)
        data = [grouped[p] for p in PLANNERS]
        bp = ax.boxplot(data, patch_artist=True, widths=0.62, showfliers=False)
        for box, planner in zip(bp["boxes"], PLANNERS):
            box.set(facecolor=COLORS[planner], alpha=0.63, edgecolor="#334155")
        for median in bp["medians"]:
            median.set(color="#111827", linewidth=1.3)
        means = [np.mean(grouped[p]) for p in PLANNERS]
        ax.scatter(range(1, len(PLANNERS) + 1), means, color="#111827", marker="D", s=18, zorder=3)
        ax.set_xticks(range(1, len(labels) + 1), labels)
        ax.set_yscale("log")
        ax.set_title(title)
        ax.grid(axis="y", linestyle="--", alpha=0.35)
    axes[0].set_ylabel("Wall-clock planning time (ms, log scale)")
    fig.tight_layout()
    save(fig, "figure_05_wall_time_distribution")


def load_ablation() -> dict[str, dict[str, dict[str, str]]]:
    """Load ablation."""
    result = {"simple": {}, "v2": {}}
    for root in (ABL_ROOT, DIRECT_ROOT):
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
        for run in manifest["runs"]:
            if "summary_file" not in run:
                continue
            mode = run["ablation_mode"]
            if root == DIRECT_ROOT or mode not in result[run["benchmark"]]:
                result[run["benchmark"]][mode] = rows(ROOT / run["summary_file"])[0]
    return result


def plot_ablation() -> None:
    """Plot ablation."""
    data = load_ablation()
    fig, axes = plt.subplots(1, 2, figsize=(13.0, 4.8), sharey=True)
    x = np.arange(len(MODES))
    for ax, benchmark, title in zip(axes, ["simple", "v2"], ["(a) simple", "(b) v2/WS119"]):
        median = [scalar(data[benchmark][m], "中位时间(ms)") for m in MODES]
        p75 = [scalar(data[benchmark][m], "P75(ms)") for m in MODES]
        ax.plot(x, median, marker="o", color="#2563eb", linewidth=1.8, label="Median")
        ax.plot(x, p75, marker="s", color="#b45309", linewidth=1.8, label="P75")
        ax.set_yscale("log")
        ax.set_xticks(x, MODE_LABELS)
        ax.set_title(title)
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend(loc="upper left")
    axes[0].set_ylabel("Planning time (ms, log scale)")
    fig.tight_layout()
    save(fig, "figure_06_ablation")


def trajectory_xyz(filename: str) -> np.ndarray:
    """Trajectory xyz."""
    return np.array([[scalar(row, "x"), scalar(row, "y"), scalar(row, "z")] for row in rows(TRAJ_ROOT / filename)])


def plot_qualitative_trajectory() -> None:
    """Plot qualitative trajectory."""
    mesh = load_ws119_mesh(step=25)
    direct = trajectory_xyz("direct_ee_trajectory.csv")
    guided = trajectory_xyz("heuristic_guided_ee_trajectory.csv")
    candidates = rows(TRAJ_ROOT / "guide_candidates.csv")
    candidate_points = np.array([[scalar(row, "x"), scalar(row, "y"), scalar(row, "z")] for row in candidates])
    top = candidate_points[0]
    fig = plt.figure(figsize=(12.6, 5.4))
    direct_ax = fig.add_subplot(121, projection="3d")
    guided_ax = fig.add_subplot(122, projection="3d")
    common_extent = np.vstack([direct, guided, candidate_points, mesh.reshape(-1, 3)])
    for ax in (direct_ax, guided_ax):
        add_mesh(ax, mesh, alpha=0.24)
        ax.scatter(*direct[0], color="#334155", marker="o", s=35, label="Start")
        ax.scatter(*guided[-1], color="#16a34a", marker="*", s=85, label="Target")
        ax.view_init(elev=27, azim=-52)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        equal_3d(ax, common_extent, padding=0.02)
    direct_ax.plot(direct[:, 0], direct[:, 1], direct[:, 2], color="#dc2626", linewidth=2.3,
                   label="Direct trajectory")
    direct_ax.set_title("(a) Direct RRTConnect: 11.34 s, budget hit")
    guided_ax.scatter(candidate_points[:, 0], candidate_points[:, 1], candidate_points[:, 2],
                      color="#d97706", s=10, alpha=0.38, label="Guide candidates")
    guided_ax.plot(guided[:, 0], guided[:, 1], guided[:, 2], color="#0891b2", linewidth=2.5,
                   label="HeuristicGuided trajectory")
    guided_ax.scatter(*top, color="#1d4ed8", marker="D", s=50, label="Selected guide")
    guided_ax.set_title("(b) HeuristicGuided: 3.38 s, no budget hit")
    direct_ax.legend(loc="upper left", fontsize=7.5)
    guided_ax.legend(loc="upper left", fontsize=7.5)
    fig.suptitle("Extreme_NarrowPassage: complete trajectory comparison on WS119", fontsize=11)
    fig.tight_layout()
    save(fig, "figure_07_qualitative_trajectory")


def main() -> int:
    """Main."""
    setup_style()
    plot_method_pipeline()
    plot_guide_geometry()
    plot_benchmark_scenes()
    plot_main_comparison()
    plot_wall_time_distribution()
    plot_ablation()
    plot_qualitative_trajectory()
    print(OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
