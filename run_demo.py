#!/usr/bin/env python3
"""CR5 多机械臂柔性产线调度系统 —— 一键启动入口

用法:
    python3 run_demo.py              # 启动 GUI（CoppeliaSim 五臂运动模式）
    python3 run_demo.py --mock       # 启动离线 GUI（Mock 模式）
    python3 run_demo.py --headless   # 无界面正式调度 + Mock 执行
    python3 run_demo.py --scene-check   # 只读验证真实场景
    python3 run_demo.py --scene-replay  # Mock 动作 + 真实场景状态回放
    python3 run_demo.py --real       # 明确拒绝：本工程不连接真实机械臂
"""

import sys
import os
import argparse


def run_gui(
    scene_linked: bool = True,
    real_motion: bool = False,
    host: str = "127.0.0.1",
    port: int = 23000,
):
    """启动 GUI 主界面"""
    if real_motion:
        raise RuntimeError(
            "真实机械臂模式已禁用；请直接运行 python3 run_demo.py，"
            "它只控制 CoppeliaSim 内的 R1-R5 模型"
        )

    from app.main_app import Cr5AssemblyApp
    import tkinter as tk

    root = tk.Tk()
    app = Cr5AssemblyApp(
        root,
        scene_linked=scene_linked,
        host=host,
        port=port,
    )

    app.run()


def run_headless():
    """无界面模式：运行调度算法并输出日志"""
    print("=" * 60)
    print("CR5 多机械臂调度系统 — Headless 模式")
    print("=" * 60)

    from mock.mock_robot_executor import MockRobotExecutor
    from orchestration.cell_orchestrator import CellOrchestrator
    from scheduler.order_parser import OrderParser
    from scheduler.scheduler import Scheduler

    # 加载订单
    parser = OrderParser()
    demo_path = os.path.join(os.path.dirname(__file__), "data", "orders", "demo_orders.json")
    orders = parser.parse_file(demo_path)
    print(f"\n加载 {len(orders)} 个订单:")
    for o in orders:
        print(f"  {o.order_id}: {o.product_type}型, 优先级={o.priority}")

    # 生成任务
    scheduler = Scheduler()
    executor = MockRobotExecutor()
    orchestrator = CellOrchestrator(scheduler, executor)

    def report(event):
        if event.kind == "task_completed" and event.result is not None:
            result = event.result
            quality = (
                f" 质量={result.quality_result}"
                if result.quality_result
                else ""
            )
            print(
                f"  {result.task_id}: {result.status} "
                f"[{result.robot_id}]{quality}"
            )

    orchestrator.add_event_callback(report)
    tasks = orchestrator.start(orders)
    print(f"\n生成 {len(tasks)} 个初始任务，开始动态调度:")
    status = orchestrator.wait()

    # 统计
    finished = [
        task
        for task in orchestrator.tasks
        if task.status == "finished"
    ]
    print(
        f"\n统计: 状态={status}, "
        f"完成 {len(finished)}/{len(orchestrator.tasks)} 个任务"
    )
    print("=" * 60)


def run_scene_check(host: str, port: int) -> None:
    """Connect and validate the real scene without starting or moving it."""
    from robot_control.motion_safety import motion_gate_status
    from sim_bridge.coppelia_client import SimBridge

    bridge = SimBridge(host=host, port=port)
    try:
        if not bridge.connect(host, port):
            raise RuntimeError(bridge.last_error or "scene connection failed")
        report = bridge.contract_report
        print("SCENE CONTRACT OK")
        print(f"  path: {report['scene_path']}")
        print(f"  sha256: {report['sha256']}")
        print(f"  targets: {report['target_count']}")
        print(f"  robots: {', '.join(report['robots'])}")
        gate = motion_gate_status()
        print(
            "  CoppeliaSim motion: "
            + ("ENABLED" if gate["simulation_enabled"] else "DISABLED")
        )
        print("  physical robot motion: DISABLED")
        for reason in gate["reasons"]:
            print(f"    - {reason}")
    finally:
        bridge.disconnect()


def run_scene_replay(host: str, port: int) -> None:
    """Replay scheduler results into scene signals without robot motion."""
    from mock.mock_robot_executor import MockRobotExecutor
    from orchestration.cell_orchestrator import CellOrchestrator
    from robot_control.scene_aware_executor import SceneAwareExecutor
    from scheduler.order_parser import OrderParser
    from scheduler.scheduler import Scheduler
    from sim_bridge.coppelia_client import SimBridge

    bridge = SimBridge(host=host, port=port)
    try:
        if not bridge.connect(host, port):
            raise RuntimeError(bridge.last_error or "scene connection failed")
        if not bridge.start_simulation():
            raise RuntimeError(bridge.last_error or "simulation start failed")
        bridge.send_process_command("RESET_CELL")
        parser = OrderParser()
        demo_path = os.path.join(
            os.path.dirname(__file__),
            "data",
            "orders",
            "demo_orders.json",
        )
        orders = parser.parse_file(demo_path)
        executor = SceneAwareExecutor(MockRobotExecutor(), bridge)
        orchestrator = CellOrchestrator(Scheduler(), executor)

        def report(event):
            if (
                event.kind == "task_completed"
                and event.result is not None
                and event.result.status == "failed"
            ):
                print(
                    f"  FAILED {event.task_id}: "
                    f"{event.result.message}"
                )

        orchestrator.add_event_callback(report)
        print(
            "SCENE REPLAY: robot durations are mocked; "
            "no robot trajectory is executed."
        )
        orchestrator.start(orders)
        status = orchestrator.wait()
        print(
            f"scene replay {status}: "
            f"{len(orchestrator.dispatched_task_ids)} tasks dispatched"
        )
        if status != "finished":
            raise RuntimeError(f"scene replay ended with status {status}")
    finally:
        if bridge.is_connected():
            bridge.stop_simulation()
        bridge.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CR5 多机械臂柔性产线调度系统")
    parser.add_argument(
        "--mock",
        action="store_true",
        help="离线 GUI Mock 模式（默认 GUI 为真实场景联动安全模式）",
    )
    parser.add_argument(
        "--real",
        action="store_true",
        help="物理机械臂模式（本工程明确禁用）",
    )
    parser.add_argument("--headless", action="store_true", help="无界面模式")
    parser.add_argument(
        "--scene-check",
        action="store_true",
        help="只读检查真实CoppeliaSim场景契约",
    )
    parser.add_argument(
        "--scene-replay",
        action="store_true",
        help="使用Mock动作驱动真实场景状态，不执行机器人轨迹",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()

    try:
        if args.scene_check:
            run_scene_check(args.host, args.port)
        elif args.scene_replay:
            run_scene_replay(args.host, args.port)
        elif args.headless:
            run_headless()
        else:
            run_gui(
                scene_linked=not args.mock,
                real_motion=args.real,
                host=args.host,
                port=args.port,
            )
    except RuntimeError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(2)
