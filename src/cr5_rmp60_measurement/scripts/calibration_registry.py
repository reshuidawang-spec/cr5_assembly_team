#!/usr/bin/env python3
"""Validate canonical sphere/stylus identities before calibration motion."""
import argparse
import json
import math
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = PROJECT_DIR / "config/calibration_registry.json"


def project_path(value):
    """Project path."""
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = PROJECT_DIR / path
    return path.resolve()


def relative_project_path(value):
    """Relative project path."""
    path = project_path(value)
    try:
        return str(path.relative_to(PROJECT_DIR))
    except ValueError:
        return str(path)


def load_registry(path=DEFAULT_REGISTRY):
    """Load and validate the calibration registry JSON file."""
    registry_path = project_path(path)
    data = json.loads(registry_path.read_text())
    if data.get("schema_version") != 1:
        raise ValueError(f"{registry_path}: unsupported calibration registry schema")
    active_id = data.get("active_standard_sphere_id")
    sphere = data.get("standard_spheres", {}).get(active_id)
    if not active_id or not sphere or sphere.get("status") != "accepted":
        raise ValueError(f"{registry_path}: active accepted standard sphere is missing")
    return data


def parse_mapping(items, option_name):
    """Parse mapping."""
    result = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError(f"{option_name} entries must be BRANCH=VALUE")
        branch, value = (part.strip() for part in item.split("=", 1))
        if not branch or not value:
            raise ValueError(f"{option_name} entries must be BRANCH=VALUE")
        if branch in result:
            raise ValueError(f"{option_name}: duplicate branch {branch}")
        result[branch] = value
    return result


def _vector(data, key, source):
    value = data.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{source}: missing 3-value {key}")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{source}: non-finite {key}")
    return result


def _assert_close(actual, expected, label, tolerance=1e-6):
    delta = math.sqrt(sum((a - b) ** 2 for a, b in zip(actual, expected)))
    if delta > tolerance:
        raise ValueError(f"{label} differs from registry by {delta:.6f} mm")


def _fit_offset(fit_data, stylus, source):
    selector = stylus.get("fit_result_selector")
    if selector:
        branch = selector.get("branch")
        result = next(
            (item for item in fit_data.get("branches", []) if item.get("branch") == branch),
            None,
        )
        if result is None:
            raise ValueError(f"{source}: no fit result for selector branch {branch}")
        data = {"local_ball_offset_mm": result.get("estimated_offset_mm")}
    else:
        data = fit_data
    return _vector(data, "local_ball_offset_mm", source)


def validate_registry(registry):
    """Cross-validate all sphere centres and stylus offsets in the registry."""
    active_id = registry["active_standard_sphere_id"]
    sphere = registry["standard_spheres"][active_id]
    sphere_fit_path = project_path(sphere["canonical_fit_json"])
    if not sphere_fit_path.is_file():
        raise ValueError(f"registered sphere fit does not exist: {sphere_fit_path}")
    sphere_fit = json.loads(sphere_fit_path.read_text())
    _assert_close(
        _vector(sphere_fit, "sphere_center_mm", sphere_fit_path),
        _vector(sphere, "sphere_center_mm", "registry sphere"),
        "canonical sphere centre",
    )

    for stylus_id, stylus in registry.get("styli", {}).items():
        if stylus.get("sphere_id") != active_id:
            raise ValueError(f"{stylus_id}: sphere_id is not the active standard sphere")
        fit_path = project_path(stylus["canonical_fit_json"])
        if not fit_path.is_file():
            raise ValueError(f"{stylus_id}: registered fit does not exist: {fit_path}")
        fit_data = json.loads(fit_path.read_text())
        fit_reference = fit_data.get("reference_fit_json")
        if fit_reference is not None and project_path(fit_reference) != sphere_fit_path:
            raise ValueError(
                f"{stylus_id}: fit references a non-canonical sphere fit: {fit_reference}"
            )
        _assert_close(
            _fit_offset(fit_data, stylus, fit_path),
            _vector(stylus, "local_ball_offset_mm", stylus_id),
            f"{stylus_id} offset",
        )
        fit_center = fit_data.get("sphere_center_mm")
        if fit_center is not None:
            _assert_close(
                [float(value) for value in fit_center],
                _vector(sphere, "sphere_center_mm", "registry sphere"),
                f"{stylus_id} sphere centre",
            )
    return registry


def resolve_branch_styli(
    registry,
    branches,
    fits,
    explicit_mapping=None,
    reference_fit_json=None,
    require_auto_ready=False,
):
    """Map measurement branches to their registered physical stylus identities."""
    active_id = registry["active_standard_sphere_id"]
    sphere = registry["standard_spheres"][active_id]
    if reference_fit_json is not None:
        expected = project_path(sphere["canonical_fit_json"])
        actual = project_path(reference_fit_json)
        if actual != expected:
            raise ValueError(
                "reference sphere fit is not canonical: "
                f"{relative_project_path(actual)} != {relative_project_path(expected)}"
            )

    explicit_mapping = explicit_mapping or {}
    unknown = sorted(set(explicit_mapping) - set(branches))
    if unknown:
        raise ValueError("stylus mapping contains inactive branches: " + ", ".join(unknown))

    deprecated = {
        project_path(item["path"]): item
        for item in registry.get("deprecated_artifacts", [])
        if item.get("path")
    }
    styli = registry.get("styli", {})
    resolved = {}
    failures = []
    for branch in branches:
        fit = fits.get(branch)
        fit_path = project_path(fit) if fit is not None else None
        if fit_path in deprecated:
            item = deprecated[fit_path]
            failures.append(f"{branch}: deprecated fit ({item.get('status')}): {relative_project_path(fit_path)}")
            continue

        stylus_id = explicit_mapping.get(branch)
        if stylus_id is None and fit_path is not None:
            matches = [
                candidate_id
                for candidate_id, candidate in styli.items()
                if candidate.get("branch") == branch
                and project_path(candidate.get("canonical_fit_json")) == fit_path
            ]
            if len(matches) == 1:
                stylus_id = matches[0]
        if stylus_id is None:
            failures.append(f"{branch}: no unambiguous physical stylus identity for selected fit")
            continue
        stylus = styli.get(stylus_id)
        if stylus is None:
            failures.append(f"{branch}: unknown physical stylus id {stylus_id}")
            continue
        if stylus.get("branch") != branch:
            failures.append(
                f"{branch}: {stylus_id} is registered as branch {stylus.get('branch')}"
            )
            continue
        if stylus.get("sphere_id") != active_id:
            failures.append(f"{branch}: {stylus_id} belongs to another sphere setup")
            continue
        canonical_fit = project_path(stylus["canonical_fit_json"])
        if fit_path != canonical_fit:
            failures.append(
                f"{branch}: selected fit {relative_project_path(fit_path)} does not match "
                f"{stylus_id} canonical fit {relative_project_path(canonical_fit)}"
            )
            continue
        if require_auto_ready and not stylus.get("allowed_for_auto", False):
            failures.append(
                f"{branch}: {stylus_id} status={stylus.get('status')} is not auto-ready"
            )
            continue
        resolved[branch] = stylus_id
    if failures:
        raise ValueError("calibration identity gate failed: " + "; ".join(failures))
    return resolved


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    parser.add_argument("--reference-fit-json")
    parser.add_argument("--branches", nargs="+", default=[])
    parser.add_argument("--branch-fit", nargs="*", default=[], metavar="BRANCH=JSON")
    parser.add_argument("--branch-stylus", nargs="*", default=[], metavar="BRANCH=STYLUS_ID")
    parser.add_argument("--require-auto-ready", action="store_true")
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    registry = validate_registry(load_registry(args.registry))
    fit_values = parse_mapping(args.branch_fit, "--branch-fit")
    stylus_values = parse_mapping(args.branch_stylus, "--branch-stylus")
    branches = args.branches or sorted(fit_values)
    if branches:
        resolved = resolve_branch_styli(
            registry,
            branches,
            {branch: Path(path) for branch, path in fit_values.items()},
            stylus_values,
            args.reference_fit_json,
            args.require_auto_ready,
        )
        for branch in branches:
            print(f"{branch}: {resolved[branch]}")
    sphere_id = registry["active_standard_sphere_id"]
    sphere = registry["standard_spheres"][sphere_id]
    print(f"registry OK: sphere={sphere_id} center={sphere['sphere_center_mm']}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
