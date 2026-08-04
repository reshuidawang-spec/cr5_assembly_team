"""Compatibility access to the application's single runtime state owner.

Runtime state is deliberately owned by :class:`CellOrchestrator`; keeping a
second task/robot/lock store in ``scheduler`` would make the GUI and executor
diverge.  The alias keeps older imports working while preserving one source of
truth.
"""

from orchestration.cell_orchestrator import CellOrchestrator

StateManager = CellOrchestrator

__all__ = ["StateManager"]
