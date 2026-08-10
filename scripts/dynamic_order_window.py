#!/usr/bin/env python3
"""Small dynamic order input window for the CR5 scheduler.

Use this when you want to test order-level resequencing without changing the
fixed process route inside each A/B/C product.
"""

from __future__ import annotations

import os
import sys
import json
import subprocess
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scheduler.dynamic_order_sequence import (  # noqa: E402
    DynamicOrderInput,
    plan_dynamic_order_sequence,
    quality_evaluation,
)


PROCESS_LABELS = {
    "box_feed": "箱体上料",
    "pcb_install": "PCB安装",
    "module_install": "模块安装",
    "terminal_install": "端子排安装",
    "transfer_to_inspection": "转移检测",
    "inspect": "相机检测",
    "screw": "R4锁付",
    "sort_good": "R5良品分拣",
    "sort_defect": "R5缺陷品分拣",
}


class DynamicOrderWindow:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("CR5 动态订单输入与重排序窗口")
        self.root.geometry("1280x780")
        self.root.minsize(1120, 680)
        self.orders: list[DynamicOrderInput] = []
        self.next_index = 1
        self.demo_process: subprocess.Popen | None = None
        self.demo_started_at: float | None = None
        self.demo_speed = 2.0
        self.orders_json_path = ROOT / "output" / "dynamic_order_window_orders.json"
        self.order_status_items: dict[str, tuple[str, float, float]] = {}
        self.sequence_status_items: list[tuple[str, float, float]] = []
        self.timeline_status_items: list[tuple[str, float, float]] = []

        self._build_style()
        self._build_layout()
        self._load_demo_orders()
        self._tick_demo_time()

    def _build_style(self) -> None:
        self.bg = "#0d1117"
        self.panel = "#161b22"
        self.line = "#30363d"
        self.text = "#c9d1d9"
        self.dim = "#8b949e"
        self.accent = "#f0c040"
        self.green = "#3fb950"
        self.red = "#f85149"
        self.blue = "#58a6ff"
        self.root.configure(bg=self.bg)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", background="#0d1117", foreground=self.text, fieldbackground="#0d1117", rowheight=24)
        style.configure("Treeview.Heading", background=self.panel, foreground=self.accent, font=("Consolas", 9, "bold"))
        style.map("Treeview", background=[("selected", "#1f3541")])
        style.configure("TCombobox", fieldbackground="#0d1117", background=self.panel, foreground=self.text)

    def _build_layout(self) -> None:
        header = tk.Frame(self.root, bg=self.bg, height=54)
        header.pack(fill=tk.X)
        header.pack_propagate(False)
        tk.Label(
            header,
            text="动态订单输入与调度重排序  |  不改变单件产品内部工序，只改变订单执行优先顺序",
            bg=self.bg,
            fg=self.accent,
            font=("Microsoft YaHei", 15, "bold"),
        ).pack(side=tk.LEFT, padx=16, pady=12)

        body = tk.Frame(self.root, bg=self.bg)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        left_outer, left_holder = self._panel(body, "订单输入")
        left_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        left_outer.configure(width=310)
        left_outer.pack_propagate(False)
        left = self._scrollable_panel(left_holder)
        self._build_input_panel(left)

        center = tk.Frame(body, bg=self.bg)
        center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        top_outer, top = self._panel(center, "已输入订单 / 每次输入后自动重新排序")
        top_outer.pack(fill=tk.X, pady=(0, 8))
        self._build_order_tree(top)

        middle_outer, middle = self._panel(center, "推荐订单顺序（由现有动态评分调度算法计算）")
        middle_outer.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        self._build_sequence_tree(middle)

        bottom_outer, bottom = self._panel(center, "工序时间轴（用于检查单件内部流程未被改变）")
        bottom_outer.pack(fill=tk.BOTH, expand=True)
        self._build_timeline_tree(bottom)

    def _panel(self, parent: tk.Widget, title: str) -> tuple[tk.Frame, tk.Frame]:
        outer = tk.Frame(parent, bg=self.line)
        inner = tk.Frame(outer, bg=self.panel)
        inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        tk.Label(inner, text=f"◈ {title}", bg=self.panel, fg=self.accent, font=("Microsoft YaHei", 10, "bold")).pack(anchor=tk.W, padx=10, pady=(8, 4))
        tk.Frame(inner, bg=self.line, height=1).pack(fill=tk.X, padx=10, pady=(0, 8))
        return outer, inner

    def _scrollable_panel(self, parent: tk.Frame) -> tk.Frame:
        wrapper = tk.Frame(parent, bg=self.panel)
        wrapper.pack(fill=tk.BOTH, expand=True)
        canvas = tk.Canvas(wrapper, bg=self.panel, highlightthickness=0, bd=0)
        scrollbar = ttk.Scrollbar(wrapper, orient=tk.VERTICAL, command=canvas.yview)
        content = tk.Frame(canvas, bg=self.panel)
        window_id = canvas.create_window((0, 0), window=content, anchor=tk.NW)
        canvas.configure(yscrollcommand=scrollbar.set)

        def update_scrollregion(_event: tk.Event | None = None) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))

        def update_content_width(event: tk.Event) -> None:
            canvas.itemconfigure(window_id, width=event.width)

        def on_mousewheel(event: tk.Event) -> None:
            delta = getattr(event, "delta", 0)
            if delta:
                canvas.yview_scroll(int(-1 * (delta / 120)), "units")
            elif getattr(event, "num", None) == 4:
                canvas.yview_scroll(-3, "units")
            elif getattr(event, "num", None) == 5:
                canvas.yview_scroll(3, "units")

        content.bind("<Configure>", update_scrollregion)
        canvas.bind("<Configure>", update_content_width)
        canvas.bind("<Enter>", lambda _event: canvas.bind_all("<MouseWheel>", on_mousewheel))
        canvas.bind("<Leave>", lambda _event: canvas.unbind_all("<MouseWheel>"))
        canvas.bind("<Button-4>", on_mousewheel)
        canvas.bind("<Button-5>", on_mousewheel)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        return content

    def _label(self, parent: tk.Widget, text: str) -> None:
        tk.Label(parent, text=text, bg=self.panel, fg=self.dim, font=("Microsoft YaHei", 9)).pack(anchor=tk.W, padx=12, pady=(6, 2))

    def _build_input_panel(self, parent: tk.Frame) -> None:
        self.product_var = tk.StringVar(value="A")
        self.quantity_var = tk.IntVar(value=1)
        self.priority_var = tk.IntVar(value=1)
        self.arrival_var = tk.DoubleVar(value=0.0)
        self.due_var = tk.DoubleVar(value=260.0)
        self.quality_var = tk.StringVar(value="AUTO")
        self.now_var = tk.DoubleVar(value=20.0)
        self.defects_per_100_var = tk.DoubleVar(value=2.0)
        self.changeover_seconds_var = tk.DoubleVar(value=3.0)
        self.priority_weight_var = tk.DoubleVar(value=0.45)
        self.due_weight_var = tk.DoubleVar(value=0.30)
        self.lateness_weight_var = tk.DoubleVar(value=0.18)
        self.waiting_weight_var = tk.DoubleVar(value=0.15)
        self.clearance_weight_var = tk.DoubleVar(value=0.40)
        self.screw_clearance_weight_var = tk.DoubleVar(value=0.35)
        self.urgent_boost_var = tk.DoubleVar(value=0.20)

        self._label(parent, "产品类型")
        ttk.Combobox(parent, textvariable=self.product_var, values=("A", "B", "C"), state="readonly").pack(fill=tk.X, padx=12)

        self._label(parent, "数量")
        tk.Spinbox(parent, from_=1, to=20, textvariable=self.quantity_var, bg="#0d1117", fg=self.text, insertbackground=self.text).pack(fill=tk.X, padx=12)

        self._label(parent, "优先级 priority（>=5 视为急单）")
        tk.Spinbox(parent, from_=1, to=10, textvariable=self.priority_var, bg="#0d1117", fg=self.text, insertbackground=self.text).pack(fill=tk.X, padx=12)

        self._label(parent, "到达时间 arrival_time")
        tk.Entry(parent, textvariable=self.arrival_var, bg="#0d1117", fg=self.text, insertbackground=self.text).pack(fill=tk.X, padx=12)

        self._label(parent, "交期 due_time")
        tk.Entry(parent, textvariable=self.due_var, bg="#0d1117", fg=self.text, insertbackground=self.text).pack(fill=tk.X, padx=12)

        self._label(parent, "预计检测结果")
        ttk.Combobox(parent, textvariable=self.quality_var, values=("AUTO", "OK", "NG"), state="readonly").pack(fill=tk.X, padx=12)

        self._label(parent, "AUTO次品数 / 100台")
        tk.Spinbox(parent, from_=0, to=100, increment=1, textvariable=self.defects_per_100_var, bg="#0d1117", fg=self.text, insertbackground=self.text).pack(fill=tk.X, padx=12)

        self._label(parent, "物料换型等待秒数")
        tk.Spinbox(parent, from_=0, to=20, increment=1, textvariable=self.changeover_seconds_var, bg="#0d1117", fg=self.text, insertbackground=self.text).pack(fill=tk.X, padx=12)

        self._label(parent, "当前仿真时间（用于急单插入）")
        tk.Entry(parent, textvariable=self.now_var, bg="#0d1117", fg=self.text, insertbackground=self.text).pack(fill=tk.X, padx=12)

        btns = tk.Frame(parent, bg=self.panel)
        btns.pack(fill=tk.X, padx=12, pady=12)
        tk.Button(btns, text="添加普通订单", command=self._add_order, bg="#238636", fg="white", relief=tk.FLAT).pack(fill=tk.X, pady=3, ipady=5)
        tk.Button(btns, text="插入B换型订单", command=self._insert_b_changeover_order, bg="#2ea043", fg="white", relief=tk.FLAT).pack(fill=tk.X, pady=3, ipady=5)
        tk.Button(btns, text="插入急单", command=self._add_urgent_order, bg=self.red, fg="white", relief=tk.FLAT).pack(fill=tk.X, pady=3, ipady=5)
        tk.Button(btns, text="开始仿真演示", command=self._start_coppelia_demo, bg=self.blue, fg="white", relief=tk.FLAT).pack(fill=tk.X, pady=3, ipady=5)
        tk.Button(btns, text="载入A-B-A换型流程", command=self._load_ab_changeover_orders, bg="#8957e5", fg="white", relief=tk.FLAT).pack(fill=tk.X, pady=3, ipady=5)
        tk.Button(btns, text="载入先前演示订单", command=self._load_demo_orders, bg="#21262d", fg=self.text, relief=tk.FLAT).pack(fill=tk.X, pady=3, ipady=5)
        tk.Button(btns, text="清空", command=self._clear_orders, bg="#21262d", fg=self.text, relief=tk.FLAT).pack(fill=tk.X, pady=3, ipady=5)

        weights = tk.LabelFrame(parent, text="算法权重调节", bg=self.panel, fg=self.accent, font=("Microsoft YaHei", 9, "bold"))
        weights.pack(fill=tk.X, padx=12, pady=(0, 10))
        self._weight_entry(weights, "优先级", self.priority_weight_var)
        self._weight_entry(weights, "交期紧急", self.due_weight_var)
        self._weight_entry(weights, "延期风险", self.lateness_weight_var)
        self._weight_entry(weights, "等待老化", self.waiting_weight_var)
        self._weight_entry(weights, "检测后清台", self.clearance_weight_var)
        self._weight_entry(weights, "R4锁付清台", self.screw_clearance_weight_var)
        self._weight_entry(weights, "急单临期", self.urgent_boost_var)
        tk.Button(
            weights,
            text="应用权重并重排",
            command=self._apply_scoring_weights,
            bg="#8957e5",
            fg="white",
            relief=tk.FLAT,
        ).pack(fill=tk.X, padx=8, pady=(6, 8), ipady=4)

        hint = (
            "评分原则：优先级、交期紧急程度、等待时间、剩余关键路径、瓶颈资源与检测平台冲突。\n"
            "说明：窗口只重排订单/任务启动顺序；A/B/C 内部工序链保持固定。"
        )
        tk.Label(parent, text=hint, bg=self.panel, fg=self.dim, justify=tk.LEFT, wraplength=270, font=("Microsoft YaHei", 8)).pack(fill=tk.X, padx=12, pady=(4, 10))

        self.summary_var = tk.StringVar(value="暂无订单")
        tk.Label(parent, textvariable=self.summary_var, bg="#0d1117", fg=self.blue, justify=tk.LEFT, wraplength=270, font=("Consolas", 9)).pack(fill=tk.X, padx=12, pady=(0, 8))

    def _build_order_tree(self, parent: tk.Frame) -> None:
        columns = ("order_id", "type", "qty", "pri", "arrival", "due", "quality")
        self.order_tree = self._tree(parent, columns, height=5)
        labels = {
            "order_id": ("订单号", 120),
            "type": ("类型", 55),
            "qty": ("数量", 55),
            "pri": ("优先级", 65),
            "arrival": ("到达", 75),
            "due": ("交期", 75),
            "quality": ("检测", 75),
        }
        self._setup_tree_columns(self.order_tree, labels)

    def _build_sequence_tree(self, parent: tk.Frame) -> None:
        columns = ("rank", "order_id", "type", "pri", "arrival", "due", "quality", "first", "inspect", "done", "branch")
        self.sequence_tree = self._tree(parent, columns, height=8)
        labels = {
            "rank": ("序号", 48),
            "order_id": ("订单/单件", 115),
            "type": ("类型", 48),
            "pri": ("优先级", 58),
            "arrival": ("到达", 62),
            "due": ("交期", 62),
            "quality": ("检测", 55),
            "first": ("开始", 62),
            "inspect": ("检测完", 70),
            "done": ("完成", 62),
            "branch": ("分支", 185),
        }
        self._setup_tree_columns(self.sequence_tree, labels)

    def _build_timeline_tree(self, parent: tk.Frame) -> None:
        columns = ("start", "end", "robot", "order_id", "type", "process")
        self.timeline_tree = self._tree(parent, columns, height=10)
        labels = {
            "start": ("开始", 65),
            "end": ("结束", 65),
            "robot": ("资源", 70),
            "order_id": ("订单/单件", 115),
            "type": ("类型", 50),
            "process": ("工序", 180),
        }
        self._setup_tree_columns(self.timeline_tree, labels)

    def _tree(self, parent: tk.Frame, columns: tuple[str, ...], height: int) -> ttk.Treeview:
        wrapper = tk.Frame(parent, bg=self.panel)
        wrapper.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        tree = ttk.Treeview(wrapper, columns=columns, show="headings", height=height)
        tree.tag_configure("pending", foreground=self.dim)
        tree.tag_configure("active", foreground=self.accent)
        tree.tag_configure("finished", foreground=self.green)
        vsb = ttk.Scrollbar(wrapper, orient=tk.VERTICAL, command=tree.yview)
        tree.configure(yscrollcommand=vsb.set)
        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        return tree

    def _setup_tree_columns(self, tree: ttk.Treeview, labels: dict[str, tuple[str, int]]) -> None:
        for column, (label, width) in labels.items():
            tree.heading(column, text=label)
            tree.column(column, width=width, anchor=tk.CENTER)

    def _weight_entry(self, parent: tk.Widget, label: str, variable: tk.DoubleVar) -> None:
        row = tk.Frame(parent, bg=self.panel)
        row.pack(fill=tk.X, padx=8, pady=2)
        tk.Label(row, text=label, bg=self.panel, fg=self.dim, width=10, anchor=tk.W, font=("Microsoft YaHei", 8)).pack(side=tk.LEFT)
        tk.Entry(row, textvariable=variable, bg="#0d1117", fg=self.text, insertbackground=self.text, width=8).pack(side=tk.RIGHT)

    def _scoring_weights(self) -> dict[str, float]:
        return {
            "priority_weight": max(float(self.priority_weight_var.get()), 0.0),
            "due_weight": max(float(self.due_weight_var.get()), 0.0),
            "lateness_risk_weight": max(float(self.lateness_weight_var.get()), 0.0),
            "waiting_weight": max(float(self.waiting_weight_var.get()), 0.0),
            "post_inspection_clearance_bonus": max(float(self.clearance_weight_var.get()), 0.0),
            "screw_clearance_bonus": max(float(self.screw_clearance_weight_var.get()), 0.0),
            "urgent_due_boost": max(float(self.urgent_boost_var.get()), 0.0),
        }

    def _apply_scoring_weights(self) -> None:
        self._refresh()
        self._sync_orders_to_running_demo()

    def _add_order(self) -> None:
        try:
            arrival_time = (
                float(self.now_var.get())
                if self._demo_is_running()
                else float(self.arrival_var.get())
            )
            order_id = f"{self.product_var.get()}{self.next_index:03d}"
            self.next_index += 1
            self.orders.append(
                DynamicOrderInput(
                    order_id=order_id,
                    product_type=self.product_var.get(),
                    quantity=max(int(self.quantity_var.get()), 1),
                    priority=max(int(self.priority_var.get()), 1),
                    arrival_time=arrival_time,
                    due_time=float(self.due_var.get()),
                    quality=self.quality_var.get(),  # type: ignore[arg-type]
                )
            )
            self._refresh()
            self._sync_orders_to_running_demo()
        except Exception as exc:
            messagebox.showerror("输入错误", str(exc))

    def _add_urgent_order(self) -> None:
        try:
            now = float(self.now_var.get())
            order_id = f"URG_{self.product_var.get()}{self.next_index:03d}"
            self.next_index += 1
            self.orders.append(
                DynamicOrderInput(
                    order_id=order_id,
                    product_type=self.product_var.get(),
                    quantity=max(int(self.quantity_var.get()), 1),
                    priority=max(int(self.priority_var.get()), 5),
                    arrival_time=now,
                    due_time=now + 95.0,
                    quality=self.quality_var.get(),  # type: ignore[arg-type]
                )
            )
            self._refresh()
            self._sync_orders_to_running_demo()
        except Exception as exc:
            messagebox.showerror("急单输入错误", str(exc))

    def _insert_b_changeover_order(self) -> None:
        try:
            now = float(self.now_var.get())
            order_id = f"B_SWITCH_{self.next_index:03d}"
            self.next_index += 1
            self.orders.append(
                DynamicOrderInput(
                    order_id=order_id,
                    product_type="B",
                    quantity=max(int(self.quantity_var.get()), 1),
                    priority=max(int(self.priority_var.get()), 5),
                    arrival_time=now,
                    due_time=now + 120.0,
                    quality=self.quality_var.get(),  # type: ignore[arg-type]
                )
            )
            self._refresh()
            self._sync_orders_to_running_demo()
        except Exception as exc:
            messagebox.showerror("B换型订单输入错误", str(exc))

    def _load_ab_changeover_orders(self) -> None:
        self.orders = [
            DynamicOrderInput("A_FIRST", "A", 5, 1, 260, 0, "AUTO"),
            DynamicOrderInput("B_SWITCH", "B", 2, 6, 380, 205, "AUTO"),
            DynamicOrderInput("A_REMAIN", "A", 5, 1, 650, 360, "AUTO"),
        ]
        self.product_var.set("B")
        self.quantity_var.set(2)
        self.priority_var.set(6)
        self.due_var.set(380.0)
        self.now_var.set(205.0)
        self.next_index = 1
        self._refresh()
        self._sync_orders_to_running_demo()

    def _load_demo_orders(self) -> None:
        self.orders = [
            DynamicOrderInput("A001", "A", 1, 1, 260, 0, "AUTO"),
            DynamicOrderInput("A002", "A", 1, 1, 280, 0, "AUTO"),
            DynamicOrderInput("A003", "A", 1, 1, 300, 0, "AUTO"),
            DynamicOrderInput("B001", "B", 1, 2, 340, 0, "AUTO"),
            DynamicOrderInput("C001", "C", 1, 2, 380, 0, "AUTO"),
            DynamicOrderInput("C002", "C", 1, 2, 410, 0, "AUTO"),
            DynamicOrderInput("URGENT_C", "C", 1, 5, 115, 20, "AUTO"),
        ]
        self.next_index = 1
        self._refresh()
        self._sync_orders_to_running_demo()

    def _clear_orders(self) -> None:
        self.orders.clear()
        self._refresh()
        self._sync_orders_to_running_demo()

    def _start_coppelia_demo(self) -> None:
        if not self.orders:
            messagebox.showwarning("没有订单", "请先输入至少一个订单。")
            return
        if self._demo_is_running():
            self._sync_orders_to_running_demo()
            messagebox.showinfo("仿真已在运行", "当前订单已同步到正在运行的仿真。")
            return
        output_dir = ROOT / "output"
        output_dir.mkdir(exist_ok=True)
        log_path = output_dir / "dynamic_order_window_coppelia.log"
        err_path = output_dir / "dynamic_order_window_coppelia.err"
        self._write_orders_json()
        if not self._ensure_coppelia_started():
            return

        with log_path.open("w", encoding="utf-8") as out, err_path.open("w", encoding="utf-8") as err:
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(ROOT / "scripts" / "run_coppelia_order_demo.py"),
                    "--orders-json",
                    str(self.orders_json_path),
                    "--watch-orders",
                    "--speed",
                    str(self.demo_speed),
                ],
                cwd=str(ROOT),
                stdout=out,
                stderr=err,
            )
        self.demo_process = process
        self.demo_started_at = time.monotonic()
        self.summary_var.set(
            f"{self.summary_var.get()}\n\n已启动 CoppeliaSim 演示\nPID: {process.pid}\n"
            "流程：小车送料 → 物料到位 → 五臂协同装配 → 检测/锁付/分拣"
        )
        messagebox.showinfo(
            "仿真已启动",
            "已按当前窗口订单启动 CoppeliaSim 演示。\n\n"
            "演示流程：\n"
            "1. 对应型号小车先移动到供料位；\n"
            "2. 对应型号物料出现在 R1 上料位；\n"
            "3. 五臂开始协同装配、检测、锁付和分拣。\n\n"
            "颜色：小车是什么颜色，实际零件就是什么颜色；急单仅在订单标记中显示为红色。\n"
            f"进程号：{process.pid}",
        )

    def _write_orders_json(self) -> None:
        self.orders_json_path.parent.mkdir(exist_ok=True)
        payload = {
            "updated_at": time.time(),
            "scoring": self._scoring_weights(),
            "quality_policy": {
                "defects_per_100": max(float(self.defects_per_100_var.get()), 0.0),
            },
            "material_switch": {
                "changeover_seconds": max(float(self.changeover_seconds_var.get()), 0.0),
                "enabled_types": ["A", "B"],
            },
            "orders": [
                {
                    "order_id": order.order_id,
                    "product_type": order.product_type,
                    "quantity": order.quantity,
                    "priority": order.priority,
                    "arrival_time": order.arrival_time,
                    "due_time": order.due_time,
                    "quality": order.quality,
                }
                for order in self.orders
            ],
        }
        self.orders_json_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _candidate_coppelia_executables(self) -> list[Path]:
        candidates: list[Path] = []
        if os.environ.get("COPPELIASIM_EXE"):
            candidates.append(Path(os.environ["COPPELIASIM_EXE"]))
        if os.environ.get("COPPELIASIM_ROOT"):
            root = Path(os.environ["COPPELIASIM_ROOT"])
            candidates.extend([root / "coppeliaSim.sh", root / "coppeliaSim"])
        candidates.extend(
            [
                Path("/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/coppeliaSim.sh"),
                Path("/opt/CoppeliaSim/coppeliaSim.sh"),
                Path.home() / "CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04" / "coppeliaSim.sh",
                Path.home() / "CoppeliaSim" / "coppeliaSim.sh",
            ]
        )
        return candidates

    def _find_coppelia_executable(self) -> Path | None:
        for candidate in self._candidate_coppelia_executables():
            if candidate.exists():
                return candidate
        return None

    def _find_scene_path(self) -> Path | None:
        if os.environ.get("CR5_SCENE_PATH"):
            scene = Path(os.environ["CR5_SCENE_PATH"])
            if scene.exists():
                return scene
        for scene in [ROOT / "scenes" / "compact_cell1ttt.ttt", ROOT / "scenes" / "compact_cell.ttt"]:
            if scene.exists():
                return scene
        return None

    def _ensure_coppelia_started(self) -> bool:
        exe = self._find_coppelia_executable()
        scene = self._find_scene_path()
        if exe is None:
            messagebox.showwarning(
                "未找到 CoppeliaSim",
                "未找到 CoppeliaSim 可执行文件。\n"
                "Ubuntu 默认设置示例：\n"
                "  export COPPELIASIM_ROOT=/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04\n"
                "或直接设置：\n"
                "  export COPPELIASIM_EXE=/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/coppeliaSim.sh",
            )
            return False
        args = [str(exe)]
        if scene is not None:
            args.append(str(scene))
        subprocess.Popen(
            args,
            cwd=str(exe.parent),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(6)
        return True

    def _sync_orders_to_running_demo(self) -> None:
        if not self._demo_is_running():
            return
        self._write_orders_json()
        self.summary_var.set(f"{self.summary_var.get()}\n\n订单已同步到运行中的仿真")

    def _demo_is_running(self) -> bool:
        return self.demo_process is not None and self.demo_process.poll() is None

    def _tick_demo_time(self) -> None:
        if self._demo_is_running() and self.demo_started_at is not None:
            sim_time = (time.monotonic() - self.demo_started_at) * self.demo_speed
            self.now_var.set(round(sim_time, 1))
        self._update_status_tags()
        self.root.after(1000, self._tick_demo_time)

    def _status_tag(self, start: float, end: float) -> str:
        try:
            now = float(self.now_var.get())
        except Exception:
            now = 0.0
        if now < start:
            return "pending"
        if now < end:
            return "active"
        return "finished"

    def _base_order_id(self, order_id: str) -> str:
        base, sep, suffix = order_id.rpartition("-")
        if sep and suffix.isdigit():
            return base
        return order_id

    def _update_status_tags(self) -> None:
        for item_id, start, end in self.order_status_items.values():
            if self.order_tree.exists(item_id):
                self.order_tree.item(item_id, tags=(self._status_tag(start, end),))
        for item_id, start, end in self.sequence_status_items:
            if self.sequence_tree.exists(item_id):
                self.sequence_tree.item(item_id, tags=(self._status_tag(start, end),))
        for item_id, start, end in self.timeline_status_items:
            if self.timeline_tree.exists(item_id):
                self.timeline_tree.item(item_id, tags=(self._status_tag(start, end),))

    def _refresh(self) -> None:
        self._fill_order_tree()
        self.sequence_status_items.clear()
        self.timeline_status_items.clear()
        for tree in (self.sequence_tree, self.timeline_tree):
            for item in tree.get_children():
                tree.delete(item)
        if not self.orders:
            self.summary_var.set("暂无订单")
            return

        try:
            result, rows = plan_dynamic_order_sequence(
                self.orders,
                self._scoring_weights(),
                defects_per_100=max(float(self.defects_per_100_var.get()), 0.0),
            )
        except Exception as exc:
            messagebox.showerror("调度失败", str(exc))
            return

        for row in rows:
            item_id = self.sequence_tree.insert(
                "",
                tk.END,
                values=(
                    row.rank,
                    row.order_id,
                    row.product_type,
                    row.priority,
                    f"{row.arrival_time:.0f}",
                    f"{row.due_time:.0f}",
                    row.quality_result,
                    f"{row.first_start:.1f}",
                    f"{row.inspect_end:.1f}",
                    f"{row.completion_time:.1f}",
                    row.branch,
                ),
            )
            self.sequence_status_items.append((item_id, row.first_start, row.completion_time))

        for record in sorted(result.records, key=lambda item: (item.start_time, item.end_time)):
            item_id = self.timeline_tree.insert(
                "",
                tk.END,
                values=(
                    f"{record.start_time:.1f}",
                    f"{record.end_time:.1f}",
                    record.robot_id,
                    record.order_id,
                    record.product_type,
                    PROCESS_LABELS.get(record.process, record.process),
                ),
            )
            self.timeline_status_items.append((item_id, record.start_time, record.end_time))

        aggregate_status: dict[str, list[float]] = {}
        for row in rows:
            base_order_id = self._base_order_id(row.order_id)
            if base_order_id not in aggregate_status:
                aggregate_status[base_order_id] = [row.first_start, row.completion_time]
            else:
                aggregate_status[base_order_id][0] = min(aggregate_status[base_order_id][0], row.first_start)
                aggregate_status[base_order_id][1] = max(aggregate_status[base_order_id][1], row.completion_time)
        for order_id, times in aggregate_status.items():
            item = self.order_status_items.get(order_id)
            if item:
                self.order_status_items[order_id] = (item[0], times[0], times[1])
        self._update_status_tags()

        urgent = result.summary_dict().get("urgent_completion_time", 0.0)
        self.summary_var.set(
            f"订单数: {len(self.orders)}\n"
            f"展开单件: {len(rows)}\n"
            f"Makespan: {result.makespan:.1f}s\n"
            f"冲突数: {result.conflict_count}\n"
            f"急单完成: {urgent:.1f}s"
        )

        summary = result.summary_dict()
        evaluation = quality_evaluation(
            result,
            defects_per_100=max(float(self.defects_per_100_var.get()), 0.0),
        )
        self.summary_var.set(
            self.summary_var.get()
            + f"\n加权延期: {summary.get('weighted_tardiness', 0.0):.1f}"
            + f"\n检测平台平均驻留: {summary.get('inspection_platform_avg_residency_time', 0.0):.1f}s"
            + f"\n检测后平均等待清台: {summary.get('post_inspection_avg_clearance_wait', 0.0):.1f}s"
            + f"\n良品/次品: {evaluation['good_count']:.0f}/{evaluation['defect_count']:.0f}"
            + f"\n成功率: {evaluation['success_rate']:.1f}%"
        )

    def _fill_order_tree(self) -> None:
        self.order_status_items.clear()
        for item in self.order_tree.get_children():
            self.order_tree.delete(item)
        for order in self.orders:
            item_id = self.order_tree.insert(
                "",
                tk.END,
                values=(
                    order.order_id,
                    order.product_type,
                    order.quantity,
                    order.priority,
                    f"{order.arrival_time:.0f}",
                    f"{order.due_time:.0f}",
                    order.quality,
                ),
            )
            self.order_status_items[order.order_id] = (item_id, order.arrival_time, float("inf"))


def main() -> None:
    root = tk.Tk()
    DynamicOrderWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
