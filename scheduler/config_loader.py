"""Shared helpers for loading project configuration files."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load a UTF-8 YAML file, returning an empty mapping for empty files."""
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}
