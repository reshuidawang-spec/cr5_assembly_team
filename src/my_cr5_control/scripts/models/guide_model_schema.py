#!/usr/bin/env python3
"""Guide model schema definitions — data classes and type definitions for guide model features, labels, and prediction outputs shared across training and evaluation."""

import csv
from functools import lru_cache
from pathlib import Path

from model_pipeline_common import parse_binary_label


REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_ROOT = REPO_ROOT / "config" / "guide_model_schema"
FEATURE_PROFILE_PATH = SCHEMA_ROOT / "feature_profiles.csv"
TARGET_SPEC_PATH = SCHEMA_ROOT / "targets.csv"


def _read_csv_rows(path: Path):
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


@lru_cache(maxsize=1)
def feature_profiles():
    """Feature profiles."""
    profiles = {}
    for row in _read_csv_rows(FEATURE_PROFILE_PATH):
        profile_name = row["profile_name"]
        order_index = int(row["order_index"])
        feature_name = row["feature_name"]
        profiles.setdefault(profile_name, []).append((order_index, feature_name))
    return {
        profile_name: [feature_name for _, feature_name in sorted(items)]
        for profile_name, items in profiles.items()
    }


@lru_cache(maxsize=1)
def target_specs():
    """Target specs."""
    specs = {}
    for row in _read_csv_rows(TARGET_SPEC_PATH):
        specs[row["target_name"]] = {
            "target_kind": row["target_kind"],
            "online_supported": row.get("online_supported", "0") == "1",
            "description": row.get("description", ""),
        }
    return specs


def feature_profile_names():
    """Feature profile names."""
    return sorted(feature_profiles().keys())


def get_feature_profile(profile_name: str):
    """Get feature profile."""
    profiles = feature_profiles()
    if profile_name not in profiles:
        raise KeyError(f"Unknown feature profile: {profile_name}")
    return list(profiles[profile_name])


def known_feature_names():
    """Known feature names."""
    names = set()
    for features in feature_profiles().values():
        names.update(features)
    return sorted(names)


def target_names():
    """Target names."""
    return sorted(target_specs().keys())


def raw_target_names():
    """Raw target names."""
    return sorted(
        target_name
        for target_name, spec in target_specs().items()
        if spec["target_kind"] == "raw"
    )


def derived_target_names():
    """Derived target names."""
    return sorted(
        target_name
        for target_name, spec in target_specs().items()
        if spec["target_kind"] == "derived"
    )


def _parse_bool(value, default=False):
    try:
        return parse_binary_label(value) == 1
    except ValueError:
        return default


def derived_target_value(row, target_name: str):
    """Derived target value."""
    direct_success = _parse_bool(row.get("direct_success", "0"))
    direct_hit_budget = _parse_bool(row.get("direct_hit_budget", "0"))
    guided_success = _parse_bool(row.get("guided_success", "0"))
    guided_hit_budget = _parse_bool(row.get("guided_hit_budget", "0"))

    if target_name == "candidate_direct_rescue":
        return "1" if ((not direct_success) or direct_hit_budget) and guided_success and not guided_hit_budget else "0"
    if target_name == "candidate_budget_rescue":
        return "1" if direct_hit_budget and guided_success and not guided_hit_budget else "0"
    raise KeyError(f"Unsupported derived target: {target_name}")


def target_value(row, target_name: str):
    """Target value."""
    specs = target_specs()
    if target_name not in specs:
        raise KeyError(f"Unknown target: {target_name}")
    if specs[target_name]["target_kind"] == "raw":
        return row[target_name]
    return derived_target_value(row, target_name)
