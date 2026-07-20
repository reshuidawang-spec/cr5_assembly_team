"""CR5 多机械臂柔性产线调度系统 —— 主界面（工业 HMI 风格）

5号同学负责维护此文件。通过注入不同的接口实现来切换 Mock / 真实模式。
"""

import sys
import os
import json
import threading
import time
import queue
from datetime import datetime
from typing import List, Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from interfaces.types import (
    Order, Task, TaskResult, TaskStatus, RobotState, RobotStatus,
    QualityResult, ProcessType, SystemSnapshot,
)
from mock.mock_order_parser import MockOrderParser
from mock.mock_scheduler import MockScheduler
from mock.mock_robot_executor import MockRobotExecutor
from mock.mock_sim_bridge import MockSimBridge

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


# ============================================================
# 工业 HMI 配色方案 (Industrial SCADA Color Scheme)
# ============================================================
C_BG = "#0d1117"              # 主背景 — 工控屏黑
C_PANEL = "#161b22"           # 面板背景 — 深灰
C_PANEL_BORDER = "#30363d"    # 面板边框 — 金属灰
C_HEADER = "#0d1117"          # 顶部栏
C_HEADER_LINE = "#f0c040"     # 顶部装饰线 — 工业琥珀
C_TEXT = "#c9d1d9"            # 正文
C_TEXT_DIM = "#8b949e"        # 次要文字
C_ACCENT = "#f0c040"          # 强调色 — 琥珀（工业经典）
C_BLUE = "#58a6ff"            # 数据蓝
C_GREEN = "#3fb950"           # 运行绿
C_RED = "#f85149"             # 报警红
C_AMBER = "#d29922"           # 警告琥珀
C_ORANGE = "#f0883e"          # 橙色
C_BUTTON = "#21262d"          # 按钮
C_BUTTON_HOVER = "#30363d"    # 按钮悬停
C_INPUT_BG = "#0d1117"        # 输入框背景
C_TREE_BG = "#0d1117"         # 表格背景
C_TREE_SEL = "#1f3541"        # 选中行

STATUS_COLORS = {
    "pending": C_TEXT_DIM, "running": C_AMBER,
    "finished": C_GREEN, "failed": C_RED,
    "waiting": C_ORANGE, "idle": C_GREEN,
    "busy": C_AMBER, "fault": C_RED,
}

PROCESS_LABELS = {
    "feed": "上料", "assemble": "装配", "screw": "锁付",
    "inspect": "检测", "sort_good": "良品分拣",
    "sort_defect": "不良品分拣", "unload": "下料", "rework": "返修拆解",
}

FONT_MONO = "Consolas"
FONT_UI = "Microsoft YaHei"
ASSETS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")


# ============================================================
# 辅助组件
# ============================================================
def _make_panel(parent, title="", **kw):
    """创建带标题和边框的工业面板"""
    outer = tk.Frame(parent, bg=C_PANEL_BORDER, **kw)
    inner = tk.Frame(outer, bg=C_PANEL)
    inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
    if title:
        hdr = tk.Frame(inner, bg=C_PANEL, height=28)
        hdr.pack(fill=tk.X, padx=10, pady=(6, 0))
        hdr.pack_propagate(False)
        tk.Label(
            hdr, text=f"◈  {title}", font=(FONT_UI, 10, "bold"),
            fg=C_ACCENT, bg=C_PANEL,
        ).pack(side=tk.LEFT)
        tk.Frame(inner, bg=C_PANEL_BORDER, height=1).pack(fill=tk.X, padx=10, pady=(2, 0))
    return inner


# ============================================================
# 主应用类
# ============================================================
class Cr5AssemblyApp:
    """多机械臂柔性产线调度系统 — 工业 HMI 界面"""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("CR5 多机械臂柔性产线调度系统 — 江苏科技大学")
        self.root.geometry("1320x860")
        self.root.configure(bg=C_BG)
        self.root.minsize(1100, 720)

        # 加载校徽
        self._logo_img = None
        logo_path = os.path.join(ASSETS_DIR, "school_brand.png")
        if os.path.exists(logo_path):
            try:
                if HAS_PIL:
                    pil_img = Image.open(logo_path)
                    h = 40
                    w = int(pil_img.width * h / pil_img.height)
                    pil_img = pil_img.resize((w, h), Image.LANCZOS)
                    self._logo_img = ImageTk.PhotoImage(pil_img)
                else:
                    self._logo_img = tk.PhotoImage(file=logo_path)
            except Exception:
                pass

        # ---- 模块实例 ----
        self.order_parser = MockOrderParser()
        self.scheduler = MockScheduler()
        self.robot_executor = MockRobotExecutor()
        self.sim_bridge = MockSimBridge()
        self.runtime_mode = "MOCK"

        # ---- 运行时状态 ----
        self.orders: List[Order] = []
        self.tasks: List[Task] = []
        self.running: bool = False
        self.paused: bool = False
        self._stop_event = threading.Event()
        self._ui_queue = queue.Queue()
        self._dispatched_task_ids = set()
        self._real_cycle_consumed = False
        self._real_preparing = False
        self._real_ready = False
        self._real_ready_evidence = None
        self._real_ready_error = ""
        self.product_buttons = {}

        self.scheduler.set_state_change_callback(self._on_state_change)

        # ---- 构建界面 ----
        self._build_header()
        self._build_body()
        self._build_footer()
        self._setup_tags()

        # ---- 定时刷新 ----
        self._process_ui_queue()
        self._refresh_robot_panel()

        self._log("SYSTEM INIT OK — Mock Mode")
        self._log("READY. Submit order or load demo_orders.json")

    # ============================================================
    # 模块替换
    # ============================================================
    def set_modules(
        self,
        order_parser=None,
        scheduler=None,
        robot_executor=None,
        sim_bridge=None,
        mode="REAL",
    ):
        if order_parser is not None:
            self.order_parser = order_parser
        if scheduler is not None:
            self.scheduler = scheduler
            self.scheduler.set_state_change_callback(self._on_state_change)
        if robot_executor is not None:
            self.robot_executor = robot_executor
        if sim_bridge is not None:
            self.sim_bridge = sim_bridge
        self.runtime_mode = str(mode).upper()
        self._set_mode_badge()
        if self.runtime_mode == "REAL":
            for product_type, button in self.product_buttons.items():
                button.configure(
                    state=tk.NORMAL if product_type == "A" else tk.DISABLED
                )
            self.demo_orders_btn.configure(state=tk.DISABLED)
            self._real_preparing = True
            self._real_ready = False
            self._real_ready_error = ""
            self.start_btn.configure(state=tk.DISABLED, text="PREPARING")
            self.status_bar.configure(text="PREPARING...", fg=C_AMBER)
            self._log("REAL READY PREPARATION STARTED")
            threading.Thread(
                target=self._prepare_real_cycle, daemon=True
            ).start()
        self._log(f"MODULE SWITCHED TO {self.runtime_mode} IMPLEMENTATION")

    def _prepare_real_cycle(self):
        try:
            evidence = self.robot_executor.prepare_cycle(
                quality="good", preload_both_r5=True
            )
            self._ui_queue.put(("real_ready", evidence))
        except Exception as exc:
            self._ui_queue.put(("real_ready_failed", str(exc)))

    def _set_mode_badge(self, running=False):
        text = self.runtime_mode + (" RUN" if running else "")
        background = C_GREEN if running else (
            C_BLUE if self.runtime_mode == "REAL" else C_AMBER
        )
        foreground = "white" if self.runtime_mode == "REAL" else C_HEADER
        self._mode_label.configure(text=text, bg=background, fg=foreground)

    # ============================================================
    # 顶部：校徽 + 校名 + 系统标题 + 模式
    # ============================================================
    def _build_header(self):
        header = tk.Frame(self.root, bg=C_HEADER, height=56)
        header.pack(fill=tk.X, side=tk.TOP)
        header.pack_propagate(False)

        # 左侧：校徽 + 校名
        left = tk.Frame(header, bg=C_HEADER)
        left.pack(side=tk.LEFT, padx=(12, 0))

        if self._logo_img:
            tk.Label(left, image=self._logo_img, bg=C_HEADER).pack(
                side=tk.LEFT, pady=(8, 0),
            )
        else:
            # 无图片时绘制校徽占位
            badge = tk.Canvas(left, width=42, height=42, bg=C_HEADER, highlightthickness=0)
            badge.pack(side=tk.LEFT, pady=(7, 0))
            # 简化船形图标
            badge.create_polygon(21, 4, 6, 34, 12, 38, 21, 24, 30, 38, 36, 34,
                                 fill=C_ACCENT, outline=C_ACCENT)
            badge.create_rectangle(17, 28, 25, 38, fill=C_HEADER, outline=C_ACCENT, width=2)

        tk.Label(
            left, text="江苏科技大学", font=(FONT_UI, 13, "bold"),
            fg="#ffffff", bg=C_HEADER,
        ).pack(side=tk.LEFT, padx=(8, 16))

        # 竖线分隔
        tk.Frame(header, bg=C_HEADER_LINE, width=2).pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=10)

        # 系统标题
        tk.Label(
            header, text="多机械臂柔性产线调度控制系统",
            font=(FONT_UI, 14, "bold"), fg=C_ACCENT, bg=C_HEADER,
        ).pack(side=tk.LEFT, padx=8)

        # 副标题
        tk.Label(
            header, text="工序自适 · 群臂协同",
            font=(FONT_UI, 9), fg=C_TEXT_DIM, bg=C_HEADER,
        ).pack(side=tk.LEFT, padx=(6, 0))

        # 右侧：模式 + 时钟
        right = tk.Frame(header, bg=C_HEADER)
        right.pack(side=tk.RIGHT, padx=16)

        self._mode_label = tk.Label(
            right, text="MOCK", font=(FONT_MONO, 10, "bold"),
            fg=C_HEADER, bg=C_AMBER, padx=10, pady=1,
        )
        self._mode_label.pack(side=tk.RIGHT, padx=(12, 0))

        tk.Label(
            right, text="STATUS: ONLINE", font=(FONT_MONO, 9),
            fg=C_GREEN, bg=C_HEADER,
        ).pack(side=tk.RIGHT)

        # 底部装饰线
        tk.Frame(self.root, bg=C_HEADER_LINE, height=2).pack(fill=tk.X, side=tk.TOP)

    # ============================================================
    # 主体三栏
    # ============================================================
    def _build_body(self):
        body = tk.Frame(self.root, bg=C_BG)
        body.pack(fill=tk.BOTH, expand=True, padx=6, pady=(4, 2))

        # 左栏 — 订单输入
        self._build_order_panel(body)

        # 中栏 — 任务队列 + 机械臂状态
        center = tk.Frame(body, bg=C_BG)
        center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=3)
        self._build_task_panel(center)
        self._build_robot_panel(center)

        # 右栏 — 日志 + 指标
        self._build_log_panel(body)

    # ---- 左栏：订单输入 ----
    def _build_order_panel(self, parent):
        outer = tk.Frame(parent, bg=C_PANEL_BORDER, width=260)
        outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 2))
        outer.pack_propagate(False)

        inner = tk.Frame(outer, bg=C_PANEL)
        inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)

        # 面板标题
        hdr = tk.Frame(inner, bg=C_PANEL, height=28)
        hdr.pack(fill=tk.X, padx=10, pady=(6, 0))
        hdr.pack_propagate(False)
        tk.Label(
            hdr, text="◈  订单输入  ORDER INPUT",
            font=(FONT_MONO, 10, "bold"), fg=C_ACCENT, bg=C_PANEL,
        ).pack(side=tk.LEFT)
        tk.Frame(inner, bg=C_PANEL_BORDER, height=1).pack(fill=tk.X, padx=10, pady=(4, 6))

        # 产品按钮
        for ptype, name, spec, color in [
            ("A", "A 型配电柜", "3ELM / 6SCR", "#3fb950"),
            ("B", "B 型配电柜", "5ELM / 10SCR", "#58a6ff"),
            ("C", "C 型配电柜", "6ELM / 12SCR", "#d29922"),
        ]:
            frm = tk.Frame(inner, bg=C_PANEL)
            frm.pack(fill=tk.X, padx=10, pady=2)

            btn = tk.Button(
                frm, text=f" {name}   {spec} ",
                font=(FONT_MONO, 9, "bold"), bg=C_BUTTON, fg=color,
                activebackground=C_BUTTON_HOVER, activeforeground=color,
                relief=tk.FLAT, cursor="hand2", anchor=tk.W,
                command=lambda t=ptype: self._add_order(t),
            )
            btn.pack(fill=tk.X, ipady=8)
            self.product_buttons[ptype] = btn

        quality_frame = tk.Frame(inner, bg=C_PANEL)
        quality_frame.pack(fill=tk.X, padx=10, pady=(8, 2))
        tk.Label(
            quality_frame,
            text="QUALITY:",
            font=(FONT_MONO, 8),
            fg=C_TEXT_DIM,
            bg=C_PANEL,
        ).pack(side=tk.LEFT)
        self.quality_var = tk.StringVar(value="OK")
        for value, label, color in (
            ("OK", "GOOD", C_GREEN),
            ("NG", "DEFECT", C_RED),
        ):
            tk.Radiobutton(
                quality_frame,
                text=label,
                variable=self.quality_var,
                value=value,
                indicatoron=False,
                font=(FONT_MONO, 8, "bold"),
                bg=C_BUTTON,
                fg=color,
                activebackground=C_BUTTON_HOVER,
                activeforeground=color,
                selectcolor=C_TREE_SEL,
                relief=tk.FLAT,
                padx=7,
                pady=2,
            ).pack(side=tk.RIGHT, padx=(3, 0))

        # 分隔
        tk.Frame(inner, bg=C_PANEL_BORDER, height=1).pack(fill=tk.X, padx=10, pady=8)

        # 优先级
        pf = tk.Frame(inner, bg=C_PANEL)
        pf.pack(fill=tk.X, padx=10)
        tk.Label(pf, text="PRIORITY (1-10):", font=(FONT_MONO, 8), fg=C_TEXT_DIM, bg=C_PANEL).pack(side=tk.LEFT)
        self.priority_var = tk.IntVar(value=1)
        s = tk.Spinbox(
            pf, from_=1, to=10, textvariable=self.priority_var, width=4,
            bg=C_INPUT_BG, fg=C_TEXT, font=(FONT_MONO, 10),
            buttonbackground=C_BUTTON, relief=tk.FLAT,
            insertbackground=C_TEXT,
        )
        s.pack(side=tk.RIGHT)

        # 操作按钮
        bf = tk.Frame(inner, bg=C_PANEL)
        bf.pack(fill=tk.X, padx=10, pady=(8, 4))

        tk.Button(
            bf, text="▶  提交订单  SUBMIT", font=(FONT_MONO, 9, "bold"),
            bg="#238636", fg="white", activebackground="#2ea043",
            relief=tk.FLAT, cursor="hand2",
            command=self._submit_selected_order,
        ).pack(fill=tk.X, ipady=6, pady=2)

        tk.Button(
            bf, text="⚡ 急单插入  URGENT", font=(FONT_MONO, 9, "bold"),
            bg=C_RED, fg="white", activebackground="#e01040",
            relief=tk.FLAT, cursor="hand2",
            command=self._insert_urgent_order,
        ).pack(fill=tk.X, ipady=6, pady=2)

        self.demo_orders_btn = tk.Button(
            bf, text="📂 加载 Demo", font=(FONT_MONO, 8),
            bg=C_BUTTON, fg=C_TEXT, activebackground=C_BUTTON_HOVER,
            relief=tk.FLAT, cursor="hand2",
            command=self._load_demo_orders,
        )
        self.demo_orders_btn.pack(fill=tk.X, ipady=4, pady=1)

        tk.Button(
            bf, text="📂 从文件加载...", font=(FONT_MONO, 8),
            bg=C_BUTTON, fg=C_TEXT, activebackground=C_BUTTON_HOVER,
            relief=tk.FLAT, cursor="hand2",
            command=self._load_orders_from_file,
        ).pack(fill=tk.X, ipady=4)

        # 已提交订单
        tk.Frame(inner, bg=C_PANEL_BORDER, height=1).pack(fill=tk.X, padx=10, pady=(10, 4))
        tk.Label(
            inner, text="◈  已提交订单  QUEUE",
            font=(FONT_MONO, 9, "bold"), fg=C_ACCENT, bg=C_PANEL,
        ).pack(anchor=tk.W, padx=10, pady=(0, 4))

        self.order_listbox = tk.Listbox(
            inner, bg=C_INPUT_BG, fg=C_TEXT, font=(FONT_MONO, 9),
            selectbackground=C_TREE_SEL, relief=tk.FLAT, height=8,
            highlightthickness=0,
        )
        self.order_listbox.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

    # ---- 中栏上：任务队列 ----
    def _build_task_panel(self, parent):
        inner = _make_panel(parent, title="任务队列  TASK QUEUE")
        inner.pack(fill=tk.BOTH, expand=True, pady=(0, 2))

        columns = ("task_id", "order_id", "process", "robot", "point", "status")
        self.task_tree = ttk.Treeview(inner, columns=columns, show="headings", height=8, selectmode="browse")

        for col, label, w in [
            ("task_id", "TASK ID", 78), ("order_id", "ORDER", 72),
            ("process", "PROCESS", 88), ("robot", "ROBOT", 60),
            ("point", "TARGET", 125), ("status", "STATUS", 78),
        ]:
            self.task_tree.heading(col, text=label)
            self.task_tree.column(col, width=w, anchor=tk.CENTER)

        vsb = ttk.Scrollbar(inner, orient=tk.VERTICAL, command=self.task_tree.yview)
        self.task_tree.configure(yscrollcommand=vsb.set)
        self.task_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0), pady=(4, 8))
        vsb.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 10), pady=(4, 8))

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", background=C_TREE_BG, foreground=C_TEXT,
                        fieldbackground=C_TREE_BG, borderwidth=0, rowheight=24)
        style.configure("Treeview.Heading", background=C_HEADER, foreground=C_ACCENT,
                        font=(FONT_MONO, 8, "bold"), borderwidth=0)
        style.map("Treeview", background=[("selected", C_TREE_SEL)])

    # ---- 中栏下：机械臂状态 ----
    def _build_robot_panel(self, parent):
        inner = _make_panel(parent, title="机械臂状态  ROBOT STATUS")
        inner.pack(fill=tk.X)

        cards = tk.Frame(inner, bg=C_PANEL)
        cards.pack(fill=tk.X, padx=10, pady=(4, 10))

        self.robot_widgets = {}
        robots_def = [
            ("R1", "BOX/TERMINAL", "箱体/端子"),
            ("R2", "PCB", "PCB 装配"),
            ("R3", "MODULE/TRANSFER", "模块/转移"),
            ("R4", "SCREW", "视觉锁付"),
            ("R5", "SORT", "良品/不良品分拣"),
        ]
        for rid, rtype, rname in robots_def:
            card = tk.Frame(cards, bg=C_INPUT_BG, relief=tk.FLAT, bd=1, highlightbackground=C_PANEL_BORDER, highlightthickness=1)
            card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=2)

            # 顶部标签
            top = tk.Frame(card, bg=C_BUTTON)
            top.pack(fill=tk.X)
            tk.Label(top, text=rid, font=(FONT_MONO, 13, "bold"), fg=C_ACCENT, bg=C_BUTTON).pack(pady=(4, 0))
            tk.Label(top, text=rname, font=(FONT_UI, 7), fg=C_TEXT_DIM, bg=C_BUTTON).pack(pady=(0, 3))

            # 状态指示灯 + 标签
            body = tk.Frame(card, bg=C_INPUT_BG)
            body.pack(fill=tk.X, pady=(6, 2))

            cv = tk.Canvas(body, width=14, height=14, bg=C_INPUT_BG, highlightthickness=0)
            cv.pack()
            dot = cv.create_oval(1, 1, 13, 13, fill=C_GREEN, outline="")

            sl = tk.Label(body, text="IDLE", font=(FONT_MONO, 9, "bold"), fg=C_GREEN, bg=C_INPUT_BG)
            sl.pack(pady=(1, 0))

            tl = tk.Label(body, text="-", font=(FONT_UI, 7), fg=C_TEXT_DIM, bg=C_INPUT_BG, wraplength=100)
            tl.pack(pady=(2, 6))

            self.robot_widgets[rid] = {
                "card": card, "canvas": cv, "indicator": dot,
                "status_label": sl, "task_label": tl,
            }

    # ---- 右栏：日志 + 指标 ----
    def _build_log_panel(self, parent):
        outer = tk.Frame(parent, bg=C_PANEL_BORDER, width=340)
        outer.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(2, 0))
        outer.pack_propagate(False)

        inner = tk.Frame(outer, bg=C_PANEL)
        inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)

        # 标题
        hdr = tk.Frame(inner, bg=C_PANEL, height=28)
        hdr.pack(fill=tk.X, padx=10, pady=(6, 0))
        hdr.pack_propagate(False)
        tk.Label(
            hdr, text="◈  运行日志  EVENT LOG",
            font=(FONT_MONO, 10, "bold"), fg=C_ACCENT, bg=C_PANEL,
        ).pack(side=tk.LEFT)
        tk.Frame(inner, bg=C_PANEL_BORDER, height=1).pack(fill=tk.X, padx=10, pady=(4, 4))

        self.log_text = tk.Text(
            inner, bg=C_INPUT_BG, fg=C_TEXT, font=(FONT_MONO, 8),
            relief=tk.FLAT, wrap=tk.WORD, state=tk.DISABLED,
            highlightthickness=0, padx=4, pady=4,
        )
        ls = ttk.Scrollbar(inner, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=ls.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0))
        ls.pack(side=tk.RIGHT, fill=tk.Y, padx=(0, 10))

        # 指标区
        tk.Frame(inner, bg=C_PANEL_BORDER, height=1).pack(fill=tk.X, padx=10, pady=6)

        tk.Label(
            inner, text="◈  效能指标  KPI", font=(FONT_MONO, 9, "bold"),
            fg=C_ACCENT, bg=C_PANEL,
        ).pack(anchor=tk.W, padx=10, pady=(0, 4))

        mg = tk.Frame(inner, bg=C_PANEL)
        mg.pack(fill=tk.X, padx=10, pady=(0, 10))

        self.metrics_labels = {}
        items = [
            ("makespan", "MAKESPAN"), ("utilization", "UTIL %"),
            ("conflicts", "CONFLICTS"), ("completed", "COMPLETED"),
        ]
        for i, (key, label) in enumerate(items):
            row, col = i // 2, i % 2
            tk.Label(mg, text=label, font=(FONT_MONO, 7), fg=C_TEXT_DIM, bg=C_PANEL).grid(
                row=row, column=col * 2, sticky=tk.W, padx=(0, 2), pady=3)
            val = tk.Label(mg, text="--", font=(FONT_MONO, 12, "bold"), fg=C_BLUE, bg=C_PANEL)
            val.grid(row=row, column=col * 2 + 1, sticky=tk.W, padx=(0, 16), pady=3)
            self.metrics_labels[key] = val

    # ---- 底部控制栏 ----
    def _build_footer(self):
        bar = tk.Frame(self.root, bg=C_HEADER, height=42)
        bar.pack(fill=tk.X, side=tk.BOTTOM)
        bar.pack_propagate(False)

        # 上部装饰线
        tk.Frame(self.root, bg=C_HEADER_LINE, height=2).pack(fill=tk.X, side=tk.BOTTOM)

        bs = {"font": (FONT_MONO, 9, "bold"), "relief": tk.FLAT, "cursor": "hand2"}
        bp = {"side": tk.LEFT, "padx": 3, "pady": 5, "ipadx": 14, "ipady": 3}

        self.start_btn = tk.Button(
            bar, text="▶  START", bg="#238636", fg="white",
            activebackground="#2ea043", command=self._start_execution, **bs,
        )
        self.start_btn.pack(**{**bp, "padx": (10, 3)})

        self.pause_btn = tk.Button(
            bar, text="⏸  PAUSE", bg=C_AMBER, fg="black",
            activebackground="#c48f1a", command=self._toggle_pause, **bs,
        )
        self.pause_btn.pack(**bp)

        self.reset_btn = tk.Button(
            bar, text="↺  RESET", bg=C_BUTTON, fg=C_TEXT,
            activebackground=C_BUTTON_HOVER, command=self._reset, **bs,
        )
        self.reset_btn.pack(**bp)

        tk.Button(
            bar, text="⚡ FAULT(R3)", bg=C_ORANGE, fg="black",
            activebackground="#e07830", command=lambda: self._simulate_fault("R3"), **bs,
        ).pack(**bp)

        tk.Button(
            bar, text="📊 EXPORT", bg=C_BUTTON, fg=C_TEXT,
            activebackground=C_BUTTON_HOVER, command=self._export_data, **bs,
        ).pack(**bp)

        # 状态栏
        self.status_bar = tk.Label(
            bar, text="READY", font=(FONT_MONO, 8), fg=C_GREEN, bg=C_HEADER,
        )
        self.status_bar.pack(side=tk.RIGHT, padx=16)

        # 时间戳
        self._clock_label = tk.Label(
            bar, text="", font=(FONT_MONO, 8), fg=C_TEXT_DIM, bg=C_HEADER,
        )
        self._clock_label.pack(side=tk.RIGHT, padx=8)
        self._update_clock()

    def _update_clock(self):
        self._clock_label.configure(text=datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        self.root.after(1000, self._update_clock)

    # ============================================================
    # 日志标签
    # ============================================================
    def _setup_tags(self):
        for name, color in [
            ("ok", C_GREEN), ("error", C_RED),
            ("warn", C_AMBER), ("info", C_BLUE),
        ]:
            self.log_text.tag_config(name, foreground=color)

    # ============================================================
    # 日志
    # ============================================================
    def _log(self, msg: str, level: str = "info"):
        ts = datetime.now().strftime("%H:%M:%S")
        self._ui_queue.put(("log", (f"[{ts}] {msg}\n", level)))

    def _log_direct(self, line: str, level: str):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line, level)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # ============================================================
    # 订单操作
    # ============================================================
    def _add_order(self, product_type: str):
        if self.runtime_mode == "REAL" and self.orders:
            messagebox.showwarning(
                "REAL MODE", "Current scene supports exactly one order."
            )
            return
        if self.runtime_mode == "REAL" and product_type != "A":
            messagebox.showwarning(
                "REAL MODE", "Current scene is calibrated only for type A."
            )
            return
        priority = self.priority_var.get()
        order = Order(
            order_id=f"{product_type}{len(self.orders)+1:03d}",
            product_type=product_type, priority=priority, quantity=1,
            expected_quality=self.quality_var.get(),
        )
        self.orders.append(order)
        self.order_parser.add_order(order)
        self.order_listbox.insert(
            tk.END,
            f"  {order.order_id}  |  TYPE-{product_type}  |  "
            f"{order.expected_quality}  |  PRI={priority}",
        )
        self._log(
            f"NEW ORDER: {order.order_id} TYPE={product_type} "
            f"QUALITY={order.expected_quality} PRI={priority}"
        )

    def _submit_selected_order(self):
        if not self.orders:
            messagebox.showwarning("WARNING", "No orders in queue")
            return
        self._start_execution()

    def _insert_urgent_order(self):
        if self.runtime_mode == "REAL":
            messagebox.showwarning(
                "REAL MODE",
                "Urgent insertion is unavailable in the single-product scene.",
            )
            return
        order = Order(
            order_id=f"URG-{len(self.orders)+1:02d}",
            product_type="A",
            priority=10,
            quantity=1,
            expected_quality=self.quality_var.get(),
        )
        self.orders.append(order)
        self.order_parser.add_order(order)
        self.order_listbox.insert(tk.END, f"  {order.order_id}  |  !!URGENT!!  |  PRI=10")
        self._log(f"!!! URGENT ORDER INSERTED: {order.order_id}", "warn")
        self.status_bar.configure(text="URGENT ORDER PENDING", fg=C_RED)

    def _load_demo_orders(self):
        if self.runtime_mode == "REAL":
            messagebox.showwarning(
                "REAL MODE", "Mock demo orders are disabled in REAL mode."
            )
            return
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "data", "orders", "demo_orders.json")
        self._load_orders(path)

    def _load_orders_from_file(self):
        path = filedialog.askopenfilename(title="Load Orders", filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if path:
            self._load_orders(path)

    def _load_orders(self, path: str):
        try:
            new = self.order_parser.parse_file(path)
            if self.runtime_mode == "REAL" and (
                len(self.orders) != 0
                or len(new) != 1
                or new[0].product_type.upper() != "A"
                or new[0].quantity != 1
            ):
                self.order_parser.clear()
                for existing in self.orders:
                    self.order_parser.add_order(existing)
                raise ValueError(
                    "REAL mode requires one A-type order with quantity 1"
                )
            self.orders.extend(new)
            for o in new:
                self.order_listbox.insert(
                    tk.END,
                    f"  {o.order_id}  |  TYPE-{o.product_type}  |  "
                    f"{o.expected_quality}  |  PRI={o.priority}",
                )
            self._log(f"LOADED {len(new)} ORDERS FROM {os.path.basename(path)}")
        except Exception as e:
            self._log(f"LOAD FAILED: {e}", "error")
            messagebox.showerror("ERROR", str(e))

    # ============================================================
    # 执行引擎
    # ============================================================
    def _start_execution(self):
        if self.running:
            return
        if not self.orders:
            self._log("NO ORDERS TO EXECUTE", "warn")
            return
        if self.runtime_mode == "REAL" and self._real_cycle_consumed:
            messagebox.showwarning(
                "REAL MODE",
                "This scene cycle was already used. Reload the clean scene "
                "before starting another REAL run.",
            )
            return
        if self.runtime_mode == "REAL" and not self._real_ready:
            detail = self._real_ready_error or "READY preparation is still running"
            self._log(f"REAL START BLOCKED: {detail}", "warn")
            return

        try:
            new_tasks = self.scheduler.generate_tasks(self.orders)
        except Exception as exc:
            self._log(f"TASK GENERATION FAILED: {exc}", "error")
            self.status_bar.configure(text="ORDER REJECTED", fg=C_RED)
            messagebox.showerror("TASK GENERATION FAILED", str(exc))
            return

        self.running = True
        self.paused = False
        self._stop_event.clear()
        self._dispatched_task_ids.clear()
        if self.runtime_mode == "REAL":
            self._real_cycle_consumed = True
        self.start_btn.configure(state=tk.DISABLED, text="●  RUNNING")
        self._set_mode_badge(running=True)
        self.status_bar.configure(text="EXECUTING...", fg=C_GREEN)

        self._log("=" * 50)
        self._log(f"EXECUTION STARTED — {len(self.orders)} ORDERS")

        self.tasks.extend(new_tasks)
        self._refresh_task_tree()
        self._log(f"GENERATED {len(new_tasks)} TASKS")

        threading.Thread(target=self._execution_loop, daemon=True).start()

    def _execution_loop(self):
        try:
            self._execution_loop_impl()
        except Exception as exc:
            for task in self.tasks:
                if task.status != TaskStatus.FINISHED.value:
                    task.status = TaskStatus.FAILED.value
            self._ui_queue.put(
                ("log", (f"EXECUTION LOOP FAILED: {exc}\n", "error"))
            )
            self._ui_queue.put(("refresh_tasks", None))
            self._ui_queue.put(("done", None))

    def _execution_loop_impl(self):
        while not self._stop_event.is_set():
            if self.paused:
                time.sleep(0.1)
                continue

            robots = self.robot_executor.get_robot_states()
            pending = [t for t in self.tasks if t.status in (
                TaskStatus.PENDING.value, TaskStatus.WAITING.value,
            )]

            if not pending:
                running_tasks = [t for t in self.tasks if t.status == TaskStatus.RUNNING.value]
                if not running_tasks:
                    self._ui_queue.put(("done", None))
                    break
                time.sleep(0.1)
                continue

            self.tasks = self.scheduler.schedule(self.tasks, robots)
            self._ui_queue.put(("refresh_tasks", None))

            for task in self.tasks:
                if task.status == TaskStatus.RUNNING.value:
                    if task.task_id in self._dispatched_task_ids:
                        continue
                    idle_robots = [r for r in robots if r.status == "idle"]
                    valid = [r for r in task.available_robots if r in [ir.robot_id for ir in idle_robots]]
                    if valid:
                        rid = valid[0]
                        self._dispatched_task_ids.add(task.task_id)
                        self.robot_executor.execute_task_async(
                            task, lambda r, t=task: self._on_task_done(t, r),
                        )
                        pn = PROCESS_LABELS.get(task.process, task.process)
                        self._ui_queue.put(("log", (f"  {task.task_id} → {rid}  {pn}  [{task.target_point}]\n", "info")))
                        self._ui_queue.put(("refresh_tasks", None))
                        self._ui_queue.put(("refresh_robots", None))
            time.sleep(0.3)

    def _on_task_done(self, task: Task, result: TaskResult):
        self.tasks = self.scheduler.on_task_complete(result, self.tasks, self.robot_executor.get_robot_states())
        qi = f"  QLTY={result.quality_result}" if result.quality_result else ""
        dur = max(0.0, result.end_time - result.start_time)
        task.duration = dur
        outcome = "DONE" if result.status == TaskStatus.FINISHED.value else "FAILED"
        level = "ok" if result.status == TaskStatus.FINISHED.value else "error"
        self._log(
            f"  {outcome} {result.task_id} [{result.robot_id}] {dur:.1f}s{qi}",
            level,
        )
        motion = result.metrics.get("motion_timing", {})
        first_motion = motion.get("task_call_to_first_motion_wall_s")
        handoff = result.metrics.get("handoff_to_first_motion_simulation_s")
        if first_motion is not None or handoff is not None:
            wall_text = "n/a" if first_motion is None else f"{first_motion:.3f}s"
            sim_text = "n/a" if handoff is None else f"{handoff:.3f}s"
            self._log(
                f"    FIRST MOTION wall={wall_text} sim-handoff={sim_text}"
            )
        if motion.get("monitor_error"):
            self._log(
                f"    MOTION MONITOR: {motion['monitor_error']}", "warn"
            )
        self._ui_queue.put(("refresh_tasks", None))
        self._ui_queue.put(("refresh_robots", None))
        self._ui_queue.put(("update_metrics", None))

    def _toggle_pause(self):
        self.paused = not self.paused
        if self.paused:
            self._log("⏸  PAUSED", "warn")
            self.pause_btn.configure(text="▶  RESUME")
            self.status_bar.configure(text="PAUSED", fg=C_AMBER)
        else:
            self._log("▶  RESUMED")
            self.pause_btn.configure(text="⏸  PAUSE")
            self.status_bar.configure(text="EXECUTING...", fg=C_GREEN)

    def _reset(self):
        if self.runtime_mode == "REAL" and self.running:
            messagebox.showwarning(
                "REAL MODE", "Cannot reset the GUI while a physical task is running."
            )
            return
        consumed_real_cycle = (
            self.runtime_mode == "REAL" and self._real_cycle_consumed
        )
        clean_scene_confirmed = False
        if consumed_real_cycle:
            clean_scene_confirmed = messagebox.askyesno(
                "REAL MODE",
                "RESET only clears GUI state. Has the clean CoppeliaSim scene "
                "already been reloaded?",
            )
        self._stop_event.set()
        self.running = False
        self.paused = False
        self.orders.clear()
        self.tasks.clear()
        self.order_parser.clear()
        self._dispatched_task_ids.clear()
        if clean_scene_confirmed:
            self._real_cycle_consumed = False
        self.order_listbox.delete(0, tk.END)
        for item in self.task_tree.get_children():
            self.task_tree.delete(item)
        start_state = (
            tk.NORMAL
            if self.runtime_mode != "REAL" or self._real_ready
            else tk.DISABLED
        )
        self.start_btn.configure(state=start_state, text="▶  START")
        self.pause_btn.configure(text="⏸  PAUSE")
        self._set_mode_badge()
        self.status_bar.configure(
            text=(
                "RELOAD CLEAN SCENE"
                if consumed_real_cycle and not clean_scene_confirmed
                else "READY"
            ),
            fg=(
                C_AMBER
                if consumed_real_cycle and not clean_scene_confirmed
                else C_GREEN
            ),
        )
        for k in self.metrics_labels:
            self.metrics_labels[k].configure(text="--")
        self._log("=" * 50)
        self._log("SYSTEM RESET")
        if consumed_real_cycle and not clean_scene_confirmed:
            self._log(
                "REAL UI RESET ONLY - reload the clean scene before next run",
                "warn",
            )

    def _simulate_fault(self, robot_id: str):
        self.robot_executor.set_robot_fault(robot_id)
        self.tasks = self.scheduler.handle_robot_fault(robot_id, self.tasks)
        if self.runtime_mode == "REAL":
            detail = "DEPENDENT TASKS HALTED"
        else:
            detail = "TASKS REASSIGNED"
        self._log(f"!!! FAULT INJECTED: {robot_id} — {detail}", "error")
        self._refresh_task_tree()
        messagebox.showwarning("FAULT INJECTION", f"{robot_id} FAULT\n{detail}")

    # ============================================================
    # UI 刷新
    # ============================================================
    def _on_state_change(self):
        self._ui_queue.put(("refresh_tasks", None))
        self._ui_queue.put(("refresh_robots", None))

    def _process_ui_queue(self):
        try:
            while True:
                action, data = self._ui_queue.get_nowait()
                if action == "log":
                    self._log_direct(*data)
                elif action == "refresh_tasks":
                    self._refresh_task_tree()
                elif action == "refresh_robots":
                    self._refresh_robot_panel()
                elif action == "update_metrics":
                    self._update_metrics()
                elif action == "done":
                    self._on_all_done()
                elif action == "real_ready":
                    self._real_preparing = False
                    self._real_ready = True
                    self._real_ready_evidence = data
                    self.start_btn.configure(state=tk.NORMAL, text="▶  START")
                    self.status_bar.configure(text="READY", fg=C_GREEN)
                    self._log(
                        "REAL READY - cached path points="
                        f"{data.get('path_points_total', 0)}",
                        "ok",
                    )
                elif action == "real_ready_failed":
                    self._real_preparing = False
                    self._real_ready = False
                    self._real_ready_error = str(data)
                    self.start_btn.configure(
                        state=tk.DISABLED, text="READY FAILED"
                    )
                    self.status_bar.configure(text="READY FAILED", fg=C_RED)
                    self._log(f"REAL READY FAILED: {data}", "error")
        except queue.Empty:
            pass
        self.root.after(200, self._process_ui_queue)

    def _refresh_task_tree(self):
        for item in self.task_tree.get_children():
            self.task_tree.delete(item)
        for task in self.tasks:
            robot = task.available_robots[0] if task.available_robots else "-"
            self.task_tree.insert("", tk.END, values=(
                task.task_id, task.order_id,
                PROCESS_LABELS.get(task.process, task.process),
                robot, task.target_point, task.status,
            ))

    def _refresh_robot_panel(self):
        try:
            robots = self.robot_executor.get_robot_states()
        except Exception:
            return
        for r in robots:
            w = self.robot_widgets.get(r.robot_id)
            if not w:
                continue
            color = STATUS_COLORS.get(r.status, C_TEXT_DIM)
            w["canvas"].itemconfig(w["indicator"], fill=color)
            w["status_label"].configure(text=r.status.upper(), fg=color)
            txt = r.current_task or "-"
            if r.status == "fault":
                txt, color = "!!! FAULT !!!", C_RED
            w["task_label"].configure(text=txt, fg=color)

    def _update_metrics(self):
        finished = [t for t in self.tasks if t.status == TaskStatus.FINISHED.value]
        total = len(self.tasks)
        self.metrics_labels["completed"].configure(text=f"{len(finished)}/{total}")
        if finished:
            dur = sum(max(t.duration, 0) for t in self.tasks if t.status == TaskStatus.FINISHED.value)
            self.metrics_labels["makespan"].configure(text=f"{dur:.0f}s")
        robots = self.robot_executor.get_robot_states()
        busy = sum(1 for r in robots if r.status == "busy")
        if robots:
            self.metrics_labels["utilization"].configure(text=f"{busy / len(robots) * 100:.0f}%")

    def _on_all_done(self):
        self.running = False
        self._stop_event.set()
        self.start_btn.configure(
            state=(tk.DISABLED if self.runtime_mode == "REAL" else tk.NORMAL),
            text="▶  START",
        )
        self._set_mode_badge()
        finished = [t for t in self.tasks if t.status == TaskStatus.FINISHED.value]
        failed = [t for t in self.tasks if t.status == TaskStatus.FAILED.value]
        self.status_bar.configure(
            text=("TASK CHAIN FAILED" if failed else "ALL TASKS COMPLETE"),
            fg=(C_RED if failed else C_GREEN),
        )
        self._log(
            f"EXECUTION COMPLETE — DONE: {len(finished)} | FAILED: {len(failed)}",
            "error" if failed else "ok",
        )
        self._update_metrics()

    # ============================================================
    # 导出
    # ============================================================
    def _export_data(self):
        path = filedialog.asksaveasfilename(
            title="Export Data", defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("JSON", "*.json")],
        )
        if not path:
            return
        try:
            if path.endswith(".csv"):
                import csv
                with open(path, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(["task_id", "order_id", "robot_id", "process", "status", "duration"])
                    for t in self.tasks:
                        w.writerow([t.task_id, t.order_id,
                                    t.available_robots[0] if t.available_robots else "",
                                    t.process, t.status, t.duration])
            else:
                with open(path, "w", encoding="utf-8") as f:
                    json.dump([t.to_dict() for t in self.tasks], f, ensure_ascii=False, indent=2)
            self._log(f"DATA EXPORTED: {path}", "ok")
            messagebox.showinfo("EXPORT OK", f"Saved to:\n{path}")
        except Exception as e:
            self._log(f"EXPORT FAILED: {e}", "error")

    def run(self):
        self.root.mainloop()


def main():
    root = tk.Tk()
    app = Cr5AssemblyApp(root)
    app.run()


if __name__ == "__main__":
    main()
