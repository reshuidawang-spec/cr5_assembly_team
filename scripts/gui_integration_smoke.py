#!/usr/bin/env python3
"""Non-interactive smoke test for the GUI-to-CoppeliaSim order loop."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import tkinter as tk
from tkinter import messagebox


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from app.main_app import Cr5AssemblyApp  # noqa: E402
from interfaces.types import TaskStatus  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--orders", type=int, default=1)
    parser.add_argument("--defect-order", type=int, default=0)
    parser.add_argument(
        "--urgent-b",
        action="store_true",
        help="insert one B order after the coordinated A pipeline starts",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.orders < 1 or args.orders > 20:
        raise ValueError("--orders must be between 1 and 20")
    if args.defect_order < 0 or args.defect_order > args.orders:
        raise ValueError("--defect-order must be 0 or a submitted order index")
    print("GUI SMOKE: creating Tk root", flush=True)
    root = tk.Tk()
    print("GUI SMOKE: Tk root created", flush=True)

    def fail_on_dialog(title: str, message: str, **_kwargs):
        raise RuntimeError(f"{title}: {message}")

    # A hidden modal dialog would deadlock this non-interactive smoke test.
    messagebox.showerror = fail_on_dialog
    messagebox.showwarning = fail_on_dialog
    messagebox.showinfo = fail_on_dialog
    print("GUI SMOKE: creating application", flush=True)
    app = Cr5AssemblyApp(root, scene_linked=True)
    root.update_idletasks()
    root.withdraw()
    try:
        print("GUI SMOKE: app initialized", flush=True)
        app.ng_unit_var.set(args.defect_order)
        for index in range(args.orders):
            app.order_id_var.set(f"GUI-SMOKE-{index + 1:03d}")
            app.product_type_var.set("A")
            app.quantity_var.set(1)
            app.priority_var.set(5)
            app.due_time_var.set(120)
            if args.orders > 1:
                if not app._submit_selected_order():
                    raise RuntimeError(
                        f"cannot submit smoke order {index + 1}"
                    )
        # Reproduce the normal user path: enter one or more quantity-1 orders
        # and press START.  With one order START also verifies auto-submit.
        app._start_execution()
        print(
            f"GUI SMOKE: {args.orders} quantity-1 order(s) submitted",
            flush=True,
        )

        deadline = time.monotonic() + 360.0
        previous_status = ""
        urgent_inserted = False
        while time.monotonic() < deadline:
            root.update()
            current_status = str(app.status_bar.cget("text"))
            if current_status != previous_status:
                print(f"GUI SMOKE STATUS: {current_status}", flush=True)
                previous_status = current_status
            if current_status == "EXECUTION FAILED":
                executor = app.base_robot_executor
                pipeline_result = getattr(
                    executor, "_coordinated_batch_result", {}
                )
                pipeline_error = getattr(
                    executor, "_coordinated_batch_error", ""
                )
                raise RuntimeError(
                    "GUI coordinated execution failed: "
                    f"{pipeline_error}; result={pipeline_result}"
                )
            executor = app.base_robot_executor
            if (
                args.urgent_b
                and not urgent_inserted
                and getattr(executor, "_coordinated_batch_status", "")
                == "running"
            ):
                app.product_type_var.set("B")
                app.order_id_var.set("GUI-URG-B-001")
                app.quantity_var.set(1)
                app._insert_urgent_order()
                urgent_inserted = True
                print("GUI SMOKE: B urgent order inserted", flush=True)
            if (
                app.tasks
                and not app.running
                and all(
                    task.status == TaskStatus.FINISHED.value
                    for task in app.tasks
                )
            ):
                pipeline_result = getattr(
                    app.base_robot_executor,
                    "_coordinated_batch_result",
                    {},
                )
                if pipeline_result.get("message"):
                    print(
                        "GUI SMOKE PIPELINE RESULT:\n"
                        + str(pipeline_result["message"]),
                        flush=True,
                    )
                print(
                    "GUI COPPELIA LOOP OK: "
                    f"orders={len(app.orders)} tasks={len(app.tasks)}"
                )
                expected_orders = args.orders + int(args.urgent_b)
                if len(app.orders) != expected_orders:
                    raise RuntimeError("GUI order count differs from input")
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
