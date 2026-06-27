#!/usr/bin/env python3
"""CR5 多机械臂柔性产线调度系统 —— 一键启动入口

用法:
    python3 run_demo.py              # 启动 GUI（Mock 模式）
    python3 run_demo.py --mock       # 同上
    python3 run_demo.py --real       # 使用真实模块（需团队成员先实现）
    python3 run_demo.py --headless   # 无界面模式（仅跑调度算法）
"""

import sys
import os
import argparse


def run_gui(use_mock: bool = True):
    """启动 GUI 主界面"""
    from app.main_app import Cr5AssemblyApp
    import tkinter as tk

    root = tk.Tk()
    app = Cr5AssemblyApp(root)

    if not use_mock:
        # TODO: 团队成员替换为真实模块
        # from scheduler.order_parser import OrderParser
        # from scheduler.scheduler import Scheduler
        # from robot_control.robot_executor import RobotExecutor
        # from sim_bridge.coppelia_client import SimBridge
        # app.set_modules(
        #     order_parser=OrderParser(),
        #     scheduler=Scheduler(),
        #     robot_executor=RobotExecutor(),
        #     sim_bridge=SimBridge(),
        # )
        print("[WARN] 真实模块尚未实现，回退到 Mock 模式")

    app.run()


def run_headless():
    """无界面模式：运行调度算法并输出日志"""
    print("=" * 60)
    print("CR5 多机械臂调度系统 — Headless 模式")
    print("=" * 60)

    from mock.mock_order_parser import MockOrderParser
    from mock.mock_scheduler import MockScheduler
    from mock.mock_robot_executor import MockRobotExecutor
    from interfaces.types import TaskStatus

    # 加载订单
    parser = MockOrderParser()
    demo_path = os.path.join(os.path.dirname(__file__), "data", "orders", "demo_orders.json")
    orders = parser.parse_file(demo_path)
    print(f"\n加载 {len(orders)} 个订单:")
    for o in orders:
        print(f"  {o.order_id}: {o.product_type}型, 优先级={o.priority}")

    # 生成任务
    scheduler = MockScheduler()
    tasks = scheduler.generate_tasks(orders)
    print(f"\n生成 {len(tasks)} 个任务:")
    for t in tasks:
        deps = f" (前置: {', '.join(t.predecessors)})" if t.predecessors else ""
        print(f"  {t.task_id}: {t.process} @ {t.target_point} [{t.available_robots[0]}]{deps}")

    # 模拟执行
    executor = MockRobotExecutor()
    print("\n模拟执行:")
    for task in tasks:
        result = executor.execute_task(task)
        tasks = scheduler.on_task_complete(result, tasks, executor.get_robot_states())
        print(f"  {result.task_id}: {result.status} [{result.robot_id}] "
              f"({result.start_time:.0f}-{result.end_time:.0f}s)"
              + (f" 质量={result.quality_result}" if result.quality_result else ""))

    # 统计
    finished = [t for t in tasks if t.status == TaskStatus.FINISHED.value]
    print(f"\n统计: 完成 {len(finished)}/{len(tasks)} 个任务")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CR5 多机械臂柔性产线调度系统")
    parser.add_argument("--mock", action="store_true", default=True, help="Mock 模式（默认）")
    parser.add_argument("--real", action="store_true", help="真实模块模式")
    parser.add_argument("--headless", action="store_true", help="无界面模式")
    args = parser.parse_args()

    if args.headless:
        run_headless()
    else:
        use_mock = not args.real
        run_gui(use_mock=use_mock)
