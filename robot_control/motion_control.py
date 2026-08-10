"""Repository-point motion helpers for the CoppeliaSim scheduler demo.

The project does not yet run a full IK planner.  This module turns the
process-level schedule into a work-step trajectory using the point table in
``configs/points.yaml`` and the work-step definitions in
``configs/assembly_components.yaml``.

It intentionally follows the repository's existing APP/TCP point scheme.  The
demo script uses these frames for station/product markers and sends the scene's
native tool/state commands; it does not invent inverse-kinematics trajectories.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

from scheduler.config_loader import load_yaml


@dataclass(frozen=True)
class WorkStep:
    code: str
    label: str
    point: str
    ratio: float


@dataclass(frozen=True)
class MotionFrame:
    process: str
    step_code: str
    step_label: str
    point_name: str
    position: list[float]
    local_ratio: float


class WorkstepMotionPlanner:
    """Generate approximate work-step motion frames from repository configs."""

    def __init__(self, repo_root: Path):
        self.repo_root = Path(repo_root)
        self.points = load_yaml(self.repo_root / "configs" / "points.yaml")
        component_config = load_yaml(self.repo_root / "configs" / "assembly_components.yaml")
        self.worksteps = self._load_worksteps(component_config.get("worksteps", {}))

    def _load_worksteps(self, raw: Mapping[str, list[dict]]) -> dict[str, list[WorkStep]]:
        result: dict[str, list[WorkStep]] = {}
        for process, rows in raw.items():
            steps: list[WorkStep] = []
            for row in rows or []:
                point = str(row.get("point", ""))
                if not point:
                    continue
                steps.append(
                    WorkStep(
                        code=str(row.get("code", point)),
                        label=str(row.get("label", row.get("code", point))),
                        point=point,
                        ratio=max(float(row.get("ratio", 1.0)), 0.0),
                    )
                )
            if steps:
                result[str(process)] = self._normalize(steps)
        return result

    def _normalize(self, steps: list[WorkStep]) -> list[WorkStep]:
        total = sum(step.ratio for step in steps)
        if total <= 0:
            even = 1.0 / max(len(steps), 1)
            return [WorkStep(step.code, step.label, step.point, even) for step in steps]
        return [WorkStep(step.code, step.label, step.point, step.ratio / total) for step in steps]

    def frame_for(self, process: str, ratio: float, fallback_point: str = "") -> MotionFrame:
        """Return the active work-step and interpolated point for a process."""

        steps = self.worksteps.get(process)
        clamped = max(0.0, min(float(ratio), 1.0))
        if not steps:
            point_name = fallback_point
            position = self.point_position(point_name)
            return MotionFrame(process, process, process, point_name, position, clamped)

        previous_point = steps[0].point
        consumed = 0.0
        for step in steps:
            next_consumed = consumed + step.ratio
            if clamped <= next_consumed or step is steps[-1]:
                local_ratio = 0.0 if step.ratio <= 0 else (clamped - consumed) / step.ratio
                start = self.point_position(previous_point)
                end = self.point_position(step.point)
                position = self.interpolate(start, end, local_ratio)
                return MotionFrame(
                    process=process,
                    step_code=step.code,
                    step_label=step.label,
                    point_name=step.point,
                    position=position,
                    local_ratio=max(0.0, min(local_ratio, 1.0)),
                )
            previous_point = step.point
            consumed = next_consumed

        last = steps[-1]
        return MotionFrame(process, last.code, last.label, last.point, self.point_position(last.point), 1.0)

    def point_position(self, point_name: str) -> list[float]:
        return list(self.points.get(point_name, {}).get("position", [0.0, 0.0, 0.6]))

    @staticmethod
    def interpolate(start: list[float], end: list[float], ratio: float) -> list[float]:
        r = max(0.0, min(float(ratio), 1.0))
        # Smoothstep makes the marker/arm ease in and out instead of jumping.
        s = r * r * (3.0 - 2.0 * r)
        return [start[index] + (end[index] - start[index]) * s for index in range(3)]

