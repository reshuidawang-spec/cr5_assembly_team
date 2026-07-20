#!/usr/bin/env python3
"""CR5 multi-arm scheduling system entry point."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path


def run_gui(
    use_mock: bool = True,
    host: str = "127.0.0.1",
    port: int = 23000,
    speed_deg_s: float = 50.0,
    hold_seconds: float = 0.8,
) -> int:
    """Start the GUI with either Mock or validated real adapters."""
    from app.main_app import Cr5AssemblyApp
    import tkinter as tk

    root = tk.Tk()
    app = Cr5AssemblyApp(root)

    if not use_mock:
        from robot_control.integrated_executor import IntegratedRobotExecutor
        from scheduler.order_parser import OrderParser
        from scheduler.scheduler import Scheduler
        from sim_bridge.coppelia_client import SimBridge

        bridge = SimBridge(host, port)
        scheduler = Scheduler()
        executor = IntegratedRobotExecutor(
            bridge=bridge,
            quality_resolver=scheduler.quality_for_order,
            speed_deg_s=speed_deg_s,
            hold_seconds=hold_seconds,
        )
        app.set_modules(
            order_parser=OrderParser(),
            scheduler=scheduler,
            robot_executor=executor,
            sim_bridge=bridge,
            mode="REAL",
        )

    app.run()
    return 0


def run_mock_headless() -> int:
    """Run the original fast scheduling demonstration without CoppeliaSim."""
    print("=" * 60)
    print("CR5 多机械臂调度系统 - Headless Mock 模式")
    print("=" * 60)

    from interfaces.types import TaskStatus
    from mock.mock_order_parser import MockOrderParser
    from mock.mock_robot_executor import MockRobotExecutor
    from mock.mock_scheduler import MockScheduler

    parser = MockOrderParser()
    demo_path = os.path.join(
        os.path.dirname(__file__), "data", "orders", "demo_orders.json"
    )
    orders = parser.parse_file(demo_path)
    print(f"\n加载 {len(orders)} 个订单:")
    for order in orders:
        print(
            f"  {order.order_id}: {order.product_type}型, "
            f"优先级={order.priority}"
        )

    scheduler = MockScheduler()
    tasks = scheduler.generate_tasks(orders)
    print(f"\n生成 {len(tasks)} 个任务:")
    for task in tasks:
        dependencies = (
            f" (前置: {', '.join(task.predecessors)})"
            if task.predecessors
            else ""
        )
        print(
            f"  {task.task_id}: {task.process} @ {task.target_point} "
            f"[{task.available_robots[0]}]{dependencies}"
        )

    executor = MockRobotExecutor()
    print("\n模拟执行:")
    for task in tasks:
        result = executor.execute_task(task)
        tasks = scheduler.on_task_complete(
            result, tasks, executor.get_robot_states()
        )
        quality = (
            f" 质量={result.quality_result}" if result.quality_result else ""
        )
        print(
            f"  {result.task_id}: {result.status} [{result.robot_id}] "
            f"({result.start_time:.0f}-{result.end_time:.0f}s){quality}"
        )

    finished = [
        task for task in tasks if task.status == TaskStatus.FINISHED.value
    ]
    print(f"\n统计: 完成 {len(finished)}/{len(tasks)} 个任务")
    print("=" * 60)
    return 0


def run_real_headless(
    quality: str,
    output: Path | None,
    host: str,
    port: int,
    speed_deg_s: float,
    hold_seconds: float,
    order_id: str,
) -> int:
    """Execute one validated A-type unit through the real seven-task chain."""
    from interfaces.types import TaskStatus
    from robot_control.integrated_executor import IntegratedRobotExecutor
    from scheduler.order_parser import OrderParser
    from scheduler.scheduler import Scheduler
    from sim_bridge.coppelia_client import SimBridge

    quality = quality.strip().lower()
    if quality not in {"good", "defect"}:
        raise ValueError("quality must be 'good' or 'defect'")
    quality_code = "OK" if quality == "good" else "NG"
    bridge = SimBridge(host, port)
    scheduler = Scheduler()
    executor = IntegratedRobotExecutor(
        bridge=bridge,
        quality_resolver=scheduler.quality_for_order,
        speed_deg_s=speed_deg_s,
        hold_seconds=hold_seconds,
    )
    order_parser = OrderParser()
    order = order_parser.parse_dict(
        {
            "order_id": order_id,
            "product_type": "A",
            "priority": 1,
            "quantity": 1,
            "expected_quality": quality_code,
        }
    )

    total_started = time.time()
    trigger_started: float | None = None
    evidence = {
        "status": "preparing",
        "mode": "REAL",
        "order": order.to_dict(),
        "quality": quality_code,
        "host": host,
        "port": port,
        "speed_deg_s": speed_deg_s,
        "hold_seconds": hold_seconds,
        "preparation_started_at_epoch_s": total_started,
        "ready_prepare_wall_s": None,
        "ready_evidence": None,
        "trigger_started_at_epoch_s": None,
        "scene": None,
        "tasks": [],
    }
    tasks = []
    try:
        tasks = scheduler.generate_tasks([order])
        print("=" * 72, flush=True)
        print(
            "CR5 五机械臂真实调度 - "
            f"{quality_code}, {speed_deg_s:g} deg/s, APP {hold_seconds:g} s",
            flush=True,
        )
        print("=" * 72, flush=True)

        print("PREPARING resident READY state...", flush=True)
        ready_started = time.monotonic()
        ready_evidence = executor.prepare_cycle(quality=quality)
        ready_wall = time.monotonic() - ready_started
        evidence["ready_prepare_wall_s"] = ready_wall
        evidence["ready_evidence"] = ready_evidence
        print(
            f"READY in {ready_wall:.3f}s, cached path points="
            f"{ready_evidence.get('path_points_total', 0)}",
            flush=True,
        )
        trigger_started = time.time()
        evidence["status"] = "running"
        evidence["trigger_started_at_epoch_s"] = trigger_started

        while not all(
            task.status
            in {TaskStatus.FINISHED.value, TaskStatus.FAILED.value}
            for task in tasks
        ):
            tasks = scheduler.schedule(tasks, executor.get_robot_states())
            running = [
                task
                for task in tasks
                if task.status == TaskStatus.RUNNING.value
            ]
            if len(running) != 1:
                raise RuntimeError(
                    "real scheduler did not produce exactly one runnable task"
                )
            task = running[0]
            dispatched = time.time()
            print(
                f"[{len(evidence['tasks']) + 1}/7] "
                f"{task.available_robots[0]} {task.target_point}",
                flush=True,
            )
            result = executor.execute_task(task)
            scheduler.on_task_complete(
                result, tasks, executor.get_robot_states()
            )
            task.duration = max(0.0, result.end_time - result.start_time)
            evidence["tasks"].append(
                {
                    "task": task.to_dict(),
                    "dispatch_wall_epoch_s": dispatched,
                    "result": result.to_dict(),
                }
            )
            timing = result.metrics.get("motion_timing", {})
            print(
                f"    {result.status}, wall="
                f"{result.end_time - result.start_time:.3f}s, "
                f"dispatch->motion="
                f"{timing.get('task_call_to_first_motion_wall_s')}",
                flush=True,
            )

        failed = [
            task for task in tasks if task.status == TaskStatus.FAILED.value
        ]
        evidence["status"] = "failed" if failed else "finished"
        if failed:
            evidence["failed_tasks"] = [task.task_id for task in failed]
        if bridge.is_connected():
            evidence["scene"] = bridge.scene_path()
    except Exception as exc:
        evidence["status"] = "failed"
        evidence["error"] = str(exc)
    finally:
        evidence["finished_at_epoch_s"] = time.time()
        evidence["total_wall_duration_s"] = (
            evidence["finished_at_epoch_s"] - total_started
        )
        evidence["wall_duration_s"] = (
            None
            if trigger_started is None
            else evidence["finished_at_epoch_s"] - trigger_started
        )
        evidence["robot_states"] = [
            state.to_dict() for state in executor.get_robot_states()
        ]
        executor.close()

    rendered = json.dumps(evidence, ensure_ascii=False, indent=2) + "\n"
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
        print(f"结构化日志: {output}")
    print(rendered, end="")
    return 0 if evidence["status"] == "finished" else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CR5 多机械臂柔性产线调度系统"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--mock", action="store_true", help="Mock 模式（默认）")
    mode.add_argument("--real", action="store_true", help="真实 CoppeliaSim 模式")
    parser.add_argument("--headless", action="store_true", help="无界面模式")
    parser.add_argument(
        "--quality", choices=("good", "defect"), default="good"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--speed-deg-s", type=float, default=50.0)
    parser.add_argument("--hold-seconds", type=float, default=0.8)
    parser.add_argument("--order-id", default="SCHEDULED-A-001")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.headless:
        if args.real:
            return run_real_headless(
                quality=args.quality,
                output=args.output,
                host=args.host,
                port=args.port,
                speed_deg_s=args.speed_deg_s,
                hold_seconds=args.hold_seconds,
                order_id=args.order_id,
            )
        return run_mock_headless()
    return run_gui(
        use_mock=not args.real,
        host=args.host,
        port=args.port,
        speed_deg_s=args.speed_deg_s,
        hold_seconds=args.hold_seconds,
    )


if __name__ == "__main__":
    raise SystemExit(main())
