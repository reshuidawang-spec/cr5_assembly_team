#!/usr/bin/env python3
"""Non-interactive smoke test for the GUI-to-CoppeliaSim order loop."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import tkinter as tk


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from app.main_app import Cr5AssemblyApp  # noqa: E402
from interfaces.types import TaskStatus  # noqa: E402


def main() -> int:
    root = tk.Tk()
    root.withdraw()
    app = Cr5AssemblyApp(root, scene_linked=True)
    try:
        app.order_id_var.set("GUI-SMOKE-001")
        app.product_type_var.set("A")
        app.quantity_var.set(1)
        app.priority_var.set(5)
        app.due_time_var.set(120)
        # Reproduce the normal user path: fill the form and press START.
        # START must auto-submit the form when the queue is empty.
        app._start_execution()

        deadline = time.monotonic() + 360.0
        while time.monotonic() < deadline:
            root.update()
            if (
                app.tasks
                and not app.running
                and all(
                    task.status == TaskStatus.FINISHED.value
                    for task in app.tasks
                )
            ):
                print(
                    "GUI COPPELIA LOOP OK: "
                    f"orders={len(app.orders)} tasks={len(app.tasks)}"
                )
                return 0
            time.sleep(0.05)
        raise RuntimeError(
            f"GUI integration timed out: status={app.status_bar.cget('text')}"
        )
    finally:
        if app.orchestrator is not None:
            app.orchestrator.stop()
        if app.sim_bridge.is_connected():
            app.sim_bridge.stop_simulation()
            app.sim_bridge.disconnect()
        app.coppelia_manager.terminate_owned_process()
        root.destroy()


if __name__ == "__main__":
    raise SystemExit(main())
