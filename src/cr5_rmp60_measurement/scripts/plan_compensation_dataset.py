#!/usr/bin/env python3
"""Plan a five-branch compensation dataset on the calibration sphere.

This is an offline dataset planner. It uses the accepted standard-sphere
centre, registered branch offsets, existing successful anchor poses, and the
local probe collision model to generate a broad set of sphere-normal probing
points. The output CSV is compatible with auto_calibrate_five_branch_sphere.py
--plan-input.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

from calibration_registry import (
    DEFAULT_REGISTRY,
    load_registry,
    project_path,
    validate_registry,
)
from model_based_sphere_point_planner import (
    candidate_normals,
    filter_normal,
    load_branch_offset,
    load_reference_sphere,
    plan_candidate,
    sort_rows,
)
from auto_calibrate_five_branch_sphere import (
    DEFAULT_BRANCHES,
    POSE_NAMES,
    load_branch_seed_offsets,
    local_collision_primitives,
)
from cross_probe_model import DEFAULT_GEOMETRY, load_geometry


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_ANCHOR_CSV = (
    PROJECT_DIR
    / "data/2026.6.23/full_auto_five_branch_stable_20260623_1025/contacts.csv"
)
DEFAULT_OUTPUT = PROJECT_DIR / "data/2026.6.24/compensation_dataset_plan.csv"
DEFAULT_ALL_OUTPUT = PROJECT_DIR / "data/2026.6.24/compensation_dataset_candidates.csv"
DEFAULT_SUMMARY_OUTPUT = PROJECT_DIR / "data/2026.6.24/compensation_dataset_summary.json"
DEFAULT_SAFE_POSE = [-411.9550, 123.5480, 349.4650, -177.8070, -1.4130, -36.3840]


def read_csv(path):
    """Read csv."""
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, rows):
    """Write a list of dicts to a CSV file with given fieldnames."""
    if not rows:
        raise ValueError(f"no rows to write to {path}")
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = []
    seen = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with output.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def row_vector(row, prefix, names):
    """Row vector."""
    return [float(row[f"{prefix}_{name}"]) for name in names]


def row_pose(row, prefix):
    """Row pose."""
    return row_vector(row, prefix, POSE_NAMES)


def row_approach(row):
    """Row approach."""
    vector = row_vector(row, "approach", ("x", "y", "z"))
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-12:
        """Parse branch pose."""
        raise ValueError("approach vector cannot be zero")
    return [value / length for value in vector]


def parse_branch_pose(items):
    """Parse branch pose."""
    result = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError("--branch-anchor-pose entries must be BRANCH=X,Y,Z,RX,RY,RZ")
        branch, values = item.split("=", 1)
        pose = [float(value.strip()) for value in values.split(",") if value.strip()]
        if not branch.strip() or len(pose) != 6:
            """Parse branch normal."""
            raise ValueError("--branch-anchor-pose entries must be BRANCH=X,Y,Z,RX,RY,RZ")
        result[branch.strip()] = pose
    return result


def parse_branch_normal(items):
    """Parse branch normal."""
    result = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError("--branch-anchor-normal entries must be BRANCH=NX,NY,NZ")
        branch, values = item.split("=", 1)
        normal = [float(value.strip()) for value in values.split(",") if value.strip()]
        if not branch.strip() or len(normal) != 3:
            """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
            raise ValueError("--branch-anchor-normal entries must be BRANCH=NX,NY,NZ")
        result[branch.strip()] = normalize(normal)
    return result


def normalize(vector):
    """Normal angle deg."""
    length = math.sqrt(sum(float(value) * float(value) for value in vector))
    if length <= 1e-12:
        raise ValueError("cannot normalize zero vector")
    return [float(value) / length for value in vector]


def normal_angle_deg(a, b):
    """Anchor from csv."""
    dot = max(-1.0, min(1.0, sum(float(x) * float(y) for x, y in zip(a, b))))
    return math.degrees(math.acos(dot))


def anchor_from_csv(path, branches):
    """Anchor from csv."""
    anchors = {}
    normals = {}
    if not path or not Path(path).exists():
        return anchors, normals
    for row in read_csv(path):
        branch = row.get("branch")
        if branch not in branches or branch in anchors:
            continue
        if row.get("status") not in ("", None, "HIT"):
            continue
        for prefix in ("trigger_flange", "start_flange", "transition_flange", "contact_flange"):
            """Registry branch inputs."""
            if all(row.get(f"{prefix}_{name}") not in (None, "") for name in POSE_NAMES):
                anchors[branch] = row_pose(row, prefix)
                break
        if all(row.get(f"approach_{axis}") not in (None, "") for axis in ("x", "y", "z")):
            normals[branch] = row_approach(row)
    return anchors, normals


def registry_branch_inputs(registry, branches):
    """Registry branch inputs."""
    styli = registry.get("styli", {})
    fits = {}
    stylus_ids = {}
    failures = []
    for branch in branches:
        matches = [
            (stylus_id, item)
            for stylus_id, item in styli.items()
            if item.get("branch") == branch and item.get("allowed_for_auto", False)
        ]
        if len(matches) != 1:
            """Pairwise angles."""
            failures.append(f"{branch}: expected exactly one auto-ready stylus, got {len(matches)}")
            continue
        stylus_id, item = matches[0]
        fits[branch] = project_path(item["canonical_fit_json"])
        stylus_ids[branch] = stylus_id
    if failures:
        raise ValueError("registry branch input failure: " + "; ".join(failures))
    return fits, stylus_ids


def pairwise_angles(normals):
    """Coverage metrics."""
    if len(normals) < 2:
        return []
    return [
        normal_angle_deg(left, right)
        for index, left in enumerate(normals)
        for right in normals[index + 1 :]
    ]


def coverage_metrics(rows):
    """Coverage metrics."""
    normals = [row_approach(row) for row in rows]
    angles = pairwise_angles(normals)
    nearest = []
    for index, normal in enumerate(normals):
        """Candidate score."""
        others = normals[:index] + normals[index + 1 :]
        if others:
            nearest.append(min(normal_angle_deg(normal, other) for other in others))
    return {
        "normal_span_deg": max(angles) if angles else 0.0,
        "min_pairwise_normal_deg": min(angles) if angles else 0.0,
        "mean_nearest_normal_deg": sum(nearest) / len(nearest) if nearest else 0.0,
    }


def candidate_score(row, selected_normals, args):
    """Candidate score."""
    normal = row_approach(row)
    if not selected_normals:
        separation = 0.0
    else:
        separation = min(normal_angle_deg(normal, previous) for previous in selected_normals)
    """Select coverage rows."""
    clearance = float(row.get("min_probe_clearance_mm") or 0.0)
    safe_distance = float(row.get("candidate_safe_transition_distance_mm") or 0.0)
    anchor_delta = float(row.get("candidate_anchor_orientation_delta_deg") or 0.0)
    local_score = float(row.get("candidate_score") or 0.0)
    return (
        args.coverage_weight * separation
        + args.clearance_weight * clearance
        + args.local_score_weight * local_score
        - args.transition_distance_weight * safe_distance
        - args.orientation_delta_weight * anchor_delta
    )


def select_coverage_rows(rows, count, args):
    """Select coverage rows."""
    selected = []
    selected_normals = []
    remaining = list(sort_rows(rows))
    min_separation = args.min_normal_separation_deg
    while remaining and len(selected) < count:
        eligible = []
        for row in remaining:
            normal = row_approach(row)
            if selected_normals:
                nearest = min(normal_angle_deg(normal, previous) for previous in selected_normals)
                if nearest < min_separation:
                    continue
            eligible.append(row)
        if not eligible and min_separation > args.min_fallback_normal_separation_deg:
            min_separation = max(
                args.min_fallback_normal_separation_deg,
                min_separation * 0.75,
            )
            continue
        if not eligible:
            break
        best = max(
            eligible,
            key=lambda row: (
                candidate_score(row, selected_normals, args),
                float(row.get("candidate_score") or 0.0),
                float(row.get("min_probe_clearance_mm") or 0.0),
            ),
        )
        selected.append(best)
        selected_normals.append(row_approach(best))
        remaining.remove(best)
    if len(selected) < count:
        """Split for point."""
        raise ValueError(
            f"only selected {len(selected)}/{count} rows; "
            "try increasing --cone-angle-deg, --candidate-count, or lowering --min-normal-separation-deg"
        )
    return selected


def select_control_rows(rows, count, args, anchor_normal):
    """Select repeatable control rows near the branch anchor normal."""
    if count <= 0:
        return []
    if anchor_normal is None:
        raise ValueError("--control-points-per-branch requires anchor normals")
    scored = []
    for row in rows:
        normal = row_approach(row)
        angle = normal_angle_deg(normal, anchor_normal)
        if angle > args.control_max_anchor_angle_deg:
            continue
        scored.append(
            (
                angle,
                -float(row.get("candidate_score") or 0.0),
                -float(row.get("min_probe_clearance_mm") or 0.0),
                row,
            )
        )
    if len(scored) < count:
        scored = [
            (
                normal_angle_deg(row_approach(row), anchor_normal),
                -float(row.get("candidate_score") or 0.0),
                -float(row.get("min_probe_clearance_mm") or 0.0),
                row,
            )
            for row in rows
        ]
    selected = [item[-1] for item in sorted(scored)[:count]]
    if len(selected) < count:
        raise ValueError(
            f"only selected {len(selected)}/{count} control rows; "
            "try increasing --candidate-pool-per-branch"
        )
    return selected


def split_for_point(index, points_per_branch, validation_fraction):
    """Add dataset fields."""
    validation_count = int(round(points_per_branch * validation_fraction))
    if validation_count <= 0:
        return "train"
    period = max(1, points_per_branch // validation_count)
    return "validation" if (index - 1) % period == period - 1 else "train"


def add_dataset_fields(
    row,
    args,
    branch,
    stylus_id,
    point_index,
    repeat_index,
    split,
    source_plan_id,
    group="comp",
    sample_in_branch=None,
):
    """Planner args for branch."""
    result = dict(row)
    result["plan_id"] = f"{branch}_{group}_p{point_index:02d}_r{repeat_index:02d}"
    if sample_in_branch is None:
        sample_in_branch = (point_index - 1) * args.repeats + repeat_index
    result["sample_in_branch"] = str(sample_in_branch)
    result["physical_ball_id"] = stylus_id
    result["dataset_id"] = args.dataset_id
    result["dataset_mode"] = "compensation"
    result["dataset_group"] = group
    result["dataset_branch_point_index"] = str(point_index)
    result["dataset_repeat_index"] = str(repeat_index)
    result["dataset_split"] = split
    result["dataset_source_plan_id"] = source_plan_id
    result["dataset_points_per_branch"] = str(args.points_per_branch)
    result["dataset_repeats"] = str(args.repeats)
    result["dataset_control_points_per_branch"] = str(args.control_points_per_branch)
    result["dataset_control_repeats"] = str(args.control_repeats)
    result["dataset_cone_angle_deg"] = f"{args.cone_angle_deg:.4f}"
    return result


def planner_args_for_branch(args, branch, branch_fit, stylus_id, anchor_pose, anchor_normal, collision_fits):
    """Planner args for branch."""
    return SimpleNamespace(
        reference_fit_json=str(args.reference_fit_json),
        geometry=str(args.geometry),
        branch=branch,
        branch_fit_json=str(branch_fit),
        collision_branch_fit=[f"{name}={path}" for name, path in sorted(collision_fits.items())],
        branch_offset_mm=None,
        orientation_mode=args.orientation_mode,
        physical_ball_id=stylus_id,
        output=str(args.output),
        all_output=str(args.all_output) if args.all_output else None,
        plan_count=args.candidate_pool_per_branch,
        normal_mode=args.normal_mode,
        anchor_pose=list(anchor_pose) if anchor_pose else None,
        anchor_normal=list(anchor_normal) if anchor_normal else None,
        cone_angle_deg=args.cone_angle_deg,
        cone_rings=args.cone_rings,
        cone_samples_per_ring=args.cone_samples_per_ring,
        candidate_count=args.candidate_count,
        roll_angles_deg=list(args.roll_angles_deg),
        safe_pose=list(args.safe_pose) if args.safe_pose else None,
        euler_sequence=args.euler_sequence,
        sphere_radius_mm=args.sphere_radius_mm,
        probe_radius_mm=args.probe_radius_mm,
        standoff_mm=args.standoff_mm,
        transition_standoff_mm=args.transition_standoff_mm,
        overtravel_mm=args.overtravel_mm,
        max_safe_transition_orientation_delta_deg=args.max_safe_transition_orientation_delta_deg,
        min_approach_z=args.min_approach_z,
        max_approach_z=args.max_approach_z,
        min_probe_clearance_mm=args.min_probe_clearance_mm,
        disable_table_plane_check=args.disable_table_plane_check,
        table_plane_z_mm=args.table_plane_z_mm,
        min_table_clearance_mm=args.min_table_clearance_mm,
        rod_collision_radius_mm=args.rod_collision_radius_mm,
        target_stem_exclusion_mm=args.target_stem_exclusion_mm,
        collision_segment_samples=args.collision_segment_samples,
        allow_low_clearance=args.allow_low_clearance,
        allow_collision=args.allow_collision,
        _anchor_approach=None,
        _offset_source="",
    )


def generate_branch_candidates(args, branch, branch_fit, stylus_id, anchor_pose, anchor_normal, collision_fits):
    """Generate branch candidates."""
    local = planner_args_for_branch(
        args,
        branch,
        branch_fit,
        stylus_id,
        anchor_pose,
        anchor_normal,
        collision_fits,
    )
    geometry = load_geometry(local.geometry)
    reference_fit, sphere_center = load_reference_sphere(local.reference_fit_json)
    offset, offset_source = load_branch_offset(local, geometry)
    collision_offsets = load_branch_seed_offsets(local.collision_branch_fit)
    collision_offsets[branch] = list(offset)
    local._offset_source = offset_source
    contact_distance = float(
        reference_fit.get(
            "contact_center_distance_mm",
            args.sphere_radius_mm + args.probe_radius_mm,
        )
    )
    primitives, dims, active_origin = local_collision_primitives(
        geometry,
        branch,
        offset,
        local,
        collision_offsets=collision_offsets,
    )
    normals = [normal for normal in candidate_normals(local, geometry) if filter_normal(local, normal)]
    rows = []
    index = 1
    for normal in normals:
        for roll in local.roll_angles_deg:
            rows.append(
                plan_candidate(
                    local,
                    geometry,
                    dims,
                    primitives,
                    active_origin,
                    sphere_center,
                    contact_distance,
                    offset,
                    normal,
                    roll,
                    index,
                )
            )
            index += 1
    if not rows:
        """Plan dataset."""
        raise ValueError(f"{branch}: no candidates generated")
    rows = sort_rows(rows)
    if not args.allow_collision:
        rows = [row for row in rows if row["probe_collision_status"] != "COLLISION"]
    if not args.allow_low_clearance:
        rows = [row for row in rows if row["probe_collision_status"] == "OK"]
    if not rows:
        raise ValueError(f"{branch}: no candidates left after local collision filtering")
    return rows[: args.candidate_pool_per_branch]


def plan_dataset(args):
    """Plan dataset."""
    registry = validate_registry(load_registry(args.calibration_registry))
    active_sphere = registry["standard_spheres"][registry["active_standard_sphere_id"]]
    if args.reference_fit_json is None:
        args.reference_fit_json = project_path(active_sphere["canonical_fit_json"])
    fits, stylus_ids = registry_branch_inputs(registry, args.branches)
    explicit_anchors = parse_branch_pose(args.branch_anchor_pose)
    explicit_normals = parse_branch_normal(args.branch_anchor_normal)
    csv_anchors, csv_normals = anchor_from_csv(args.anchor_csv, set(args.branches))

    anchors = {branch: csv_anchors.get(branch) for branch in args.branches}
    anchors.update(explicit_anchors)
    anchor_normals = {branch: csv_normals.get(branch) for branch in args.branches}
    anchor_normals.update(explicit_normals)
    missing = [branch for branch in args.branches if not anchors.get(branch)]
    if missing:
        raise ValueError(
            "missing anchor pose for branches: "
            + ", ".join(missing)
            + "; pass --anchor-csv or --branch-anchor-pose"
        )
    if args.normal_mode == "anchor-cone":
        missing_normals = [branch for branch in args.branches if not anchor_normals.get(branch)]
        if missing_normals:
            raise ValueError(
                "missing anchor normal for branches: "
                + ", ".join(missing_normals)
                + "; pass --anchor-csv with approach columns or --branch-anchor-normal"
            )

    selected_rows = []
    all_rows = []
    summary = {
        "schema_version": 1,
        "created_at_unix": time.time(),
        "dataset_id": args.dataset_id,
        "mode": "compensation",
        "reference_fit_json": str(args.reference_fit_json),
        "calibration_registry": str(args.calibration_registry),
        "anchor_csv": str(args.anchor_csv) if args.anchor_csv else None,
        "points_per_branch": args.points_per_branch,
        "repeats": args.repeats,
        "control_points_per_branch": args.control_points_per_branch,
        "control_repeats": args.control_repeats,
        "control_max_anchor_angle_deg": args.control_max_anchor_angle_deg,
        "validation_fraction": args.validation_fraction,
        "branches": {},
    }
    for branch in args.branches:
        print(f"\nplanning compensation dataset branch: {branch}", flush=True)
        candidates = generate_branch_candidates(
            args,
            branch,
            fits[branch],
            stylus_ids[branch],
            anchors[branch],
            anchor_normals.get(branch),
            fits,
        )
        selected = select_coverage_rows(candidates, args.points_per_branch, args)
        control = select_control_rows(
            candidates,
            args.control_points_per_branch,
            args,
            anchor_normals.get(branch),
        )
        metrics = coverage_metrics(selected)
        for row in candidates:
            candidate = dict(row)
            candidate["dataset_id"] = args.dataset_id
            candidate["dataset_mode"] = "compensation_candidate"
            candidate["candidate_selected"] = "0"
            candidate["candidate_control_selected"] = "0"
            all_rows.append(candidate)
        selected_plan_ids = {row["plan_id"] for row in selected}
        control_plan_ids = {row["plan_id"] for row in control}
        for row in all_rows:
            if row.get("branch") == branch and row.get("plan_id") in selected_plan_ids:
                row["candidate_selected"] = "1"
            if row.get("branch") == branch and row.get("plan_id") in control_plan_ids:
                row["candidate_control_selected"] = "1"
        for point_index, row in enumerate(selected, start=1):
            split = split_for_point(point_index, args.points_per_branch, args.validation_fraction)
            for repeat_index in range(1, args.repeats + 1):
                selected_rows.append(
                    add_dataset_fields(
                        row,
                        args,
                        branch,
                        stylus_ids[branch],
                        point_index,
                        repeat_index,
                        split,
                        row["plan_id"],
                        group="comp",
                    )
                )
        sample_base = args.points_per_branch * args.repeats
        for point_index, row in enumerate(control, start=1):
            for repeat_index in range(1, args.control_repeats + 1):
                selected_rows.append(
                    add_dataset_fields(
                        row,
                        args,
                        branch,
                        stylus_ids[branch],
                        point_index,
                        repeat_index,
                        "control",
                        row["plan_id"],
                        group="ctrl",
                        sample_in_branch=sample_base
                        + (point_index - 1) * args.control_repeats
                        + repeat_index,
                    )
                )
        summary["branches"][branch] = {
            "stylus_id": stylus_ids[branch],
            "branch_fit_json": str(fits[branch]),
            "anchor_pose": anchors[branch],
            "anchor_normal": anchor_normals.get(branch),
            "candidate_rows": len(candidates),
            "selected_points": len(selected),
            "selected_rows_with_repeats": len(selected) * args.repeats,
            "control_points": len(control),
            "control_rows_with_repeats": len(control) * args.control_repeats,
            **metrics,
        }
        print(
            f"  selected={len(selected)} span={metrics['normal_span_deg']:.3f}deg "
            f"min_pair={metrics['min_pairwise_normal_deg']:.3f}deg "
            f"mean_nearest={metrics['mean_nearest_normal_deg']:.3f}deg "
            f"control={len(control)}",
            flush=True,
        )
    write_csv(args.output, selected_rows)
    if args.all_output:
        write_csv(args.all_output, all_rows)
    args.summary_output.parent.mkdir(parents=True, exist_ok=True)
    args.summary_output.write_text(json.dumps(summary, indent=2) + "\n")
    print(f"\nsaved compensation plan: {args.output}", flush=True)
    """Parse args."""
    print(f"saved candidate pool: {args.all_output}", flush=True)
    print(f"saved summary: {args.summary_output}", flush=True)
    print(f"selected executable rows: {len(selected_rows)}", flush=True)
    return summary


def parse_roll_angles(value):
    """Parse roll angles."""
    result = []
    for item in value.split(","):
        item = item.strip()
        if item:
            result.append(float(item))
    if not result:
        raise argparse.ArgumentTypeError("roll angle list cannot be empty")
    return result


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration-registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--reference-fit-json", type=Path)
    parser.add_argument("--geometry", type=Path, default=DEFAULT_GEOMETRY)
    parser.add_argument("--branches", nargs="+", default=list(DEFAULT_BRANCHES))
    parser.add_argument("--dataset-id", default="five_branch_compensation_dataset")
    parser.add_argument("--anchor-csv", type=Path, default=DEFAULT_ANCHOR_CSV)
    parser.add_argument("--branch-anchor-pose", nargs="*", default=[], metavar="BRANCH=X,Y,Z,RX,RY,RZ")
    parser.add_argument("--branch-anchor-normal", nargs="*", default=[], metavar="BRANCH=NX,NY,NZ")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--all-output", type=Path, default=DEFAULT_ALL_OUTPUT)
    parser.add_argument("--summary-output", type=Path, default=DEFAULT_SUMMARY_OUTPUT)
    parser.add_argument("--points-per-branch", type=int, default=25)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument(
        "--control-points-per-branch",
        type=int,
        default=0,
        help="extra near-anchor quality-control points per branch",
    )
    parser.add_argument("--control-repeats", type=int, default=1)
    parser.add_argument(
        "--control-max-anchor-angle-deg",
        type=float,
        default=6.0,
        help="prefer control points within this angle of the branch anchor normal",
    )
    parser.add_argument("--validation-fraction", type=float, default=0.20)
    parser.add_argument("--candidate-pool-per-branch", type=int, default=160)
    parser.add_argument("--normal-mode", choices=("anchor-cone", "fibonacci"), default="anchor-cone")
    parser.add_argument("--orientation-mode", choices=("anchor-fixed", "align-branch"), default="anchor-fixed")
    parser.add_argument("--cone-angle-deg", type=float, default=12.0)
    parser.add_argument("--cone-rings", type=int, default=5)
    parser.add_argument("--cone-samples-per-ring", type=int, default=12)
    parser.add_argument("--candidate-count", type=int, default=720)
    parser.add_argument("--roll-angles-deg", type=parse_roll_angles, default=parse_roll_angles("0"))
    parser.add_argument(
        "--safe-pose",
        nargs=6,
        type=float,
        default=DEFAULT_SAFE_POSE,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
    )
    parser.add_argument("--euler-sequence", default="xyz")
    parser.add_argument("--sphere-radius-mm", type=float, default=10.0)
    parser.add_argument("--probe-radius-mm", type=float, default=1.0)
    parser.add_argument("--standoff-mm", type=float, default=0.8)
    parser.add_argument("--transition-standoff-mm", type=float, default=3.0)
    parser.add_argument("--overtravel-mm", type=float, default=0.2)
    parser.add_argument("--max-safe-transition-orientation-delta-deg", type=float, default=90.0)
    """Validate args."""
    parser.add_argument("--min-approach-z", type=float, default=-1.0)
    parser.add_argument("--max-approach-z", type=float, default=0.35)
    parser.add_argument("--min-probe-clearance-mm", type=float, default=0.5)
    parser.add_argument("--disable-table-plane-check", action="store_true")
    parser.add_argument("--table-plane-z-mm", type=float, default=0.0)
    parser.add_argument("--min-table-clearance-mm", type=float, default=5.0)
    parser.add_argument("--rod-collision-radius-mm", type=float)
    parser.add_argument("--target-stem-exclusion-mm", type=float, default=3.0)
    parser.add_argument("--collision-segment-samples", type=int, default=9)
    parser.add_argument("--allow-low-clearance", action="store_true")
    parser.add_argument("--allow-collision", action="store_true")
    parser.add_argument("--min-normal-separation-deg", type=float, default=1.5)
    parser.add_argument("--min-fallback-normal-separation-deg", type=float, default=0.4)
    parser.add_argument("--coverage-weight", type=float, default=8.0)
    parser.add_argument("--clearance-weight", type=float, default=0.5)
    parser.add_argument("--local-score-weight", type=float, default=0.1)
    parser.add_argument("--transition-distance-weight", type=float, default=0.01)
    parser.add_argument("--orientation-delta-weight", type=float, default=0.05)
    return parser.parse_args()


def validate_args(args):
    """Main."""
    if args.points_per_branch <= 0:
        raise ValueError("--points-per-branch must be positive")
    if args.repeats <= 0:
        raise ValueError("--repeats must be positive")
    if args.control_points_per_branch < 0:
        raise ValueError("--control-points-per-branch cannot be negative")
    if args.control_repeats <= 0:
        raise ValueError("--control-repeats must be positive")
    if args.control_max_anchor_angle_deg <= 0:
        raise ValueError("--control-max-anchor-angle-deg must be positive")
    if not (0.0 <= args.validation_fraction < 1.0):
        raise ValueError("--validation-fraction must be in [0, 1)")
    minimum_candidates = max(args.points_per_branch, args.control_points_per_branch)
    if args.candidate_pool_per_branch < minimum_candidates:
        raise ValueError("--candidate-pool-per-branch must cover requested branch points")
    if args.cone_rings <= 0 or args.cone_samples_per_ring <= 0:
        raise ValueError("--cone-rings and --cone-samples-per-ring must be positive")
    if args.standoff_mm <= 0 or args.transition_standoff_mm <= args.standoff_mm:
        raise ValueError("--transition-standoff-mm must be greater than --standoff-mm")
    if args.overtravel_mm <= 0:
        raise ValueError("--overtravel-mm must be positive")
    if args.min_normal_separation_deg < 0 or args.min_fallback_normal_separation_deg < 0:
        raise ValueError("normal separation limits cannot be negative")
    if args.min_fallback_normal_separation_deg > args.min_normal_separation_deg:
        raise ValueError("--min-fallback-normal-separation-deg cannot exceed --min-normal-separation-deg")
    if args.orientation_mode == "anchor-fixed" and any(abs(value) > 1e-9 for value in args.roll_angles_deg):
        raise ValueError("--orientation-mode anchor-fixed requires --roll-angles-deg 0")


def main():
    """Main."""
    args = parse_args()
    try:
        validate_args(args)
        plan_dataset(args)
        return 0
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    raise SystemExit(main())
