"""Safety gate for scene-specific robot motion plans."""

from __future__ import annotations

from pathlib import Path

from scheduler.config_loader import load_yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VALIDATION = ROOT / "configs" / "motion_validation.yaml"
DEFAULT_SCENE_CONTRACT = ROOT / "configs" / "scene_contract.yaml"


def motion_gate_status(
    validation_path: Path | str = DEFAULT_VALIDATION,
    scene_contract_path: Path | str = DEFAULT_SCENE_CONTRACT,
) -> dict:
    validation = load_yaml(Path(validation_path))
    contract = load_yaml(Path(scene_contract_path))
    expected_hash = str(contract["scene"]["sha256"])
    validation_hash = str(validation.get("scene_sha256", ""))
    physical_enabled = bool(validation.get("motion_enabled", False))
    simulation_enabled = bool(validation.get("simulation_motion_enabled", False))
    plans = dict(validation.get("validated_plans", {}))
    reasons = []
    if validation_hash != expected_hash:
        reasons.append("motion validation scene hash does not match")
    if not physical_enabled:
        reasons.append(str(validation.get("reason", "motion is disabled")))
    if set(plans) != {"R1", "R2", "R3", "R4", "R5"}:
        reasons.append("validated plans are incomplete for R1-R5")
    return {
        "enabled": not reasons,
        "physical_enabled": physical_enabled,
        "simulation_enabled": simulation_enabled,
        "scene_sha256": expected_hash,
        "validation_scene_sha256": validation_hash,
        "validated_plans": plans,
        "reasons": reasons,
    }


def require_motion_enabled(
    validation_path: Path | str = DEFAULT_VALIDATION,
    scene_contract_path: Path | str = DEFAULT_SCENE_CONTRACT,
) -> dict:
    status = motion_gate_status(validation_path, scene_contract_path)
    if not status["enabled"]:
        raise RuntimeError(
            "real motion safety gate is closed: "
            + "; ".join(status["reasons"])
        )
    return status
