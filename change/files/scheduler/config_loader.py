"""Configuration loading helpers for scheduler modules."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load a YAML mapping, returning an empty dict for empty files."""
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML config must contain a mapping: {path}")
    return data
