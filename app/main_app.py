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

from interfaces.types import Order, Task, TaskStatus
from mock.mock_robot_executor import MockRobotExecutor
from mock.mock_sim_bridge import MockSimBridge
from orchestration.cell_orchestrator import CellOrchestrator, OrchestratorEvent
from robot_control.scene_aware_executor import SceneAwareExecutor
from scheduler.order_parser import OrderParser
from scheduler.scheduler import Scheduler
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.process_manager import CoppeliaProcessManager

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
    "box_feed": "箱体上料", "pcb_install": "PCB安装",
    "module_install": "模块安装", "terminal_install": "端子安装",
    "transfer_to_inspection": "转移检测", "screw": "锁付",
    "inspect": "检测", "sort_good": "良品分拣",
    "sort_defect": "不良品分拣",
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

    def __init__(
        self,
        root: tk.Tk,
        scene_linked: bool = True,
        host: str = "127.0.0.1",
        port: int = 23000,
    ):
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
        self.scene_linked = bool(scene_linked)
        self.host = host
        self.port = int(port)
        self.order_parser = OrderParser()
        self.scheduler = Scheduler()
        self.base_robot_executor = MockRobotExecutor()
        self.robot_executor = self.base_robot_executor
        self.sim_bridge = (
            SimBridge(host=self.host, port=self.port)
            if self.scene_linked
            else MockSimBridge()
        )
        self.coppelia_manager = CoppeliaProcessManager()
        self.coppelia_manager.host = self.host
        self.coppelia_manager.port = self.port
        self.orchestrator: Optional[CellOrchestrator] = None

        # ---- 运行时状态 ----
        self.orders: List[Order] = []
        self.tasks: List[Task] = []
        self.running: bool = False
        self.paused: bool = False
        self._ui_queue = queue.Queue()
        self._pending_start = False
        self._connection_in_progress = False
        self._scene_preparation_in_progress = False
        self._scene_ready = False
        self._order_counter = 0

        # ---- 构建界面 ----
        self._build_header()
        self._build_body()
        self._build_footer()
        self._setup_tags()

        # ---- 定时刷新 ----
        self._process_ui_queue()
        self._refresh_robot_panel()
        self.root.protocol("WM_DELETE_WINDOW", self._on_window_close)

        mode = "COPPELIASIM MOTION MODE" if self.scene_linked else "OFFLINE MOCK MODE"
        self._log(f"SYSTEM INIT OK — {mode}")
        self._log("READY. Add orders, connect CoppeliaSim, then press START")

    # ============================================================
    # 模块替换
    # ============================================================
    def set_modules(self, order_parser=None, scheduler=None, robot_executor=None, sim_bridge=None):
        if order_parser is not None:
            self.order_parser = order_parser
        if scheduler is not None:
            self.scheduler = scheduler
        if robot_executor is not None:
            self.base_robot_executor = robot_executor
            self.robot_executor = robot_executor
        if sim_bridge is not None:
            self.sim_bridge = sim_bridge
        self._log("MODULE SWITCHED TO REAL IMPLEMENTATION")

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
            right,
            text="COP SIM" if self.scene_linked else "MOCK",
            font=(FONT_MONO, 10, "bold"),
            fg=C_HEADER, bg=C_AMBER, padx=10, pady=1,
        )
        self._mode_label.pack(side=tk.RIGHT, padx=(12, 0))

        self._connection_label = tk.Label(
            right,
            text="COP: OFFLINE" if self.scene_linked else "COP: MOCK",
            font=(FONT_MONO, 9),
            fg=C_RED if self.scene_linked else C_AMBER,
            bg=C_HEADER,
        )
        self._connection_label.pack(side=tk.RIGHT)

        # 底部装饰线
        tk.Frame(self.root, bg=C_HEADER_LINE, height=2).pack(fill=tk.X, side=tk.TOP)

    # ============================================================
    # 主体三栏
    # ============================================================
    def _build_body(self):
        style = ttk.Style()
        style.configure(
            "Industrial.TNotebook",
            background=C_BG,
            borderwidth=0,
        )
        style.configure(
            "Industrial.TNotebook.Tab",
            background=C_BUTTON,
            foreground=C_TEXT,
            padding=(18, 7),
            font=(FONT_UI, 9, "bold"),
        )
        style.map(
            "Industrial.TNotebook.Tab",
            background=[("selected", C_PANEL)],
            foreground=[("selected", C_ACCENT)],
        )
        self.workspace_tabs = ttk.Notebook(
            self.root,
            style="Industrial.TNotebook",
        )
        self.workspace_tabs.pack(fill=tk.BOTH, expand=True, padx=6, pady=(4, 2))

        execution_tab = tk.Frame(self.workspace_tabs, bg=C_BG)
        analysis_tab = tk.Frame(self.workspace_tabs, bg=C_BG)
        self.workspace_tabs.add(execution_tab, text="仿真执行  SIMULATION")
        self.workspace_tabs.add(analysis_tab, text="调度分析  SCHEDULING")

        body = tk.Frame(execution_tab, bg=C_BG)
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

        from app.dashboard import SchedulingDashboard

        self.scheduling_dashboard = SchedulingDashboard(
            analysis_tab,
            order_provider=lambda: list(self.orders),
            log_callback=self._log,
            colors={
                "bg": C_BG,
                "panel": C_PANEL,
                "border": C_PANEL_BORDER,
                "text": C_TEXT,
                "dim": C_TEXT_DIM,
                "accent": C_ACCENT,
                "green": C_GREEN,
                "blue": C_BLUE,
                "button": C_BUTTON,
            },
        )

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

        # 产品类型快捷选择
        self.product_type_var = tk.StringVar(value="A")
        for ptype, name, spec, color in [
            ("A", "A 型电控箱", "标准节拍", "#3fb950"),
            ("B", "B 型电控箱", "增强节拍", "#58a6ff"),
            ("C", "C 型电控箱", "复杂节拍", "#d29922"),
        ]:
            frm = tk.Frame(inner, bg=C_PANEL)
            frm.pack(fill=tk.X, padx=10, pady=2)

            btn = tk.Button(
                frm, text=f" {name}   {spec} ",
                font=(FONT_MONO, 9, "bold"), bg=C_BUTTON, fg=color,
                activebackground=C_BUTTON_HOVER, activeforeground=color,
                relief=tk.FLAT, cursor="hand2", anchor=tk.W,
                command=lambda t=ptype: self._select_product_type(t),
            )
            btn.pack(fill=tk.X, ipady=8)

        # 分隔
        tk.Frame(inner, bg=C_PANEL_BORDER, height=1).pack(fill=tk.X, padx=10, pady=8)

        def form_row(label):
            row = tk.Frame(inner, bg=C_PANEL)
            row.pack(fill=tk.X, padx=10, pady=2)
            tk.Label(
                row, text=label, width=13, anchor=tk.W,
                font=(FONT_MONO, 8), fg=C_TEXT_DIM, bg=C_PANEL,
            ).pack(side=tk.LEFT)
            return row

        order_row = form_row("ORDER ID:")
        self.order_id_var = tk.StringVar(value="")
        tk.Entry(
            order_row, textvariable=self.order_id_var,
            bg=C_INPUT_BG, fg=C_TEXT, insertbackground=C_TEXT,
            font=(FONT_MONO, 9), relief=tk.FLAT,
        ).pack(side=tk.RIGHT, fill=tk.X, expand=True)

        type_row = form_row("PRODUCT TYPE:")
        self.product_type_combo = ttk.Combobox(
            type_row,
            textvariable=self.product_type_var,
            values=("A", "B", "C"),
            state="readonly",
            width=8,
            font=(FONT_MONO, 9),
        )
        self.product_type_combo.pack(side=tk.RIGHT)

        quantity_row = form_row("QUANTITY:")
        self.quantity_var = tk.IntVar(value=1)
        tk.Spinbox(
            quantity_row, from_=1, to=99,
            textvariable=self.quantity_var, width=5,
            bg=C_INPUT_BG, fg=C_TEXT, font=(FONT_MONO, 9),
            buttonbackground=C_BUTTON, relief=tk.FLAT,
            insertbackground=C_TEXT,
        ).pack(side=tk.RIGHT)

        priority_row = form_row("PRIORITY 1-10:")
        self.priority_var = tk.IntVar(value=1)
        tk.Spinbox(
            priority_row, from_=1, to=10,
            textvariable=self.priority_var, width=5,
            bg=C_INPUT_BG, fg=C_TEXT, font=(FONT_MONO, 9),
            buttonbackground=C_BUTTON, relief=tk.FLAT,
            insertbackground=C_TEXT,
        ).pack(side=tk.RIGHT)

        due_row = form_row("DUE TIME (s):")
        self.due_time_var = tk.DoubleVar(value=0)
        tk.Spinbox(
            due_row, from_=0, to=86400, increment=30,
            textvariable=self.due_time_var, width=7,
            bg=C_INPUT_BG, fg=C_TEXT, font=(FONT_MONO, 9),
            buttonbackground=C_BUTTON, relief=tk.FLAT,
            insertbackground=C_TEXT,
        ).pack(side=tk.RIGHT)

        tk.Label(
            inner,
            text="填好后可直接 START；ADD 用于累计多笔订单",
            font=(FONT_UI, 8), fg=C_BLUE, bg=C_PANEL,
            wraplength=230, justify=tk.LEFT,
        ).pack(fill=tk.X, padx=10, pady=(5, 0))

        # 操作按钮
        bf = tk.Frame(inner, bg=C_PANEL)
        bf.pack(fill=tk.X, padx=10, pady=(8, 4))

        tk.Button(
            bf, text="＋ 加入订单队列  ADD", font=(FONT_MONO, 9, "bold"),
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

        tk.Button(
            bf, text="📂 加载 Demo", font=(FONT_MONO, 8),
            bg=C_BUTTON, fg=C_TEXT, activebackground=C_BUTTON_HOVER,
            relief=tk.FLAT, cursor="hand2",
            command=self._load_demo_orders,
        ).pack(fill=tk.X, ipady=4, pady=1)

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
            ("R1", "BOX/TERMINAL", "箱体与端子"),
            ("R2", "PCB", "PCB吸装"),
            ("R3", "MODULE/MOVE", "模块与转移"),
            ("CAMERA", "INSPECT", "视觉检测"),
            ("R4", "SCREW", "螺钉锁付"),
            ("R5", "SORT", "质量分拣"),
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

            tl = tk.Label(body, text="-", font=(FONT_UI, 7), fg=C_TEXT_DIM, bg=C_INPUT_BG, wraplength=80)
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
            ("makespan", "MAKESPAN"), ("utilization", "AVG UTIL %"),
            ("waiting", "AVG WAIT"), ("conflicts", "CONFLICTS"),
            ("completed", "COMPLETED"), ("failed", "FAILED"),
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

        self.cop_btn = tk.Button(
            bar,
            text="◉  启动/连接 COP" if self.scene_linked else "◉  OFFLINE MOCK",
            bg=C_BLUE if self.scene_linked else C_BUTTON,
            fg="black" if self.scene_linked else C_TEXT_DIM,
            activebackground="#79b8ff",
            command=self._launch_or_connect_coppelia,
            state=tk.NORMAL if self.scene_linked else tk.DISABLED,
            **bs,
        )
        self.cop_btn.pack(**{**bp, "padx": (10, 3)})

        self.start_btn = tk.Button(
            bar, text="▶  START", bg="#238636", fg="white",
            activebackground="#2ea043", command=self._start_execution, **bs,
        )
        self.start_btn.pack(**bp)

        self.pause_btn = tk.Button(
            bar, text="⏸  PAUSE", bg=C_AMBER, fg="black",
            activebackground="#c48f1a", command=self._toggle_pause, **bs,
        )
        self.pause_btn.pack(**bp)

        self.stop_btn = tk.Button(
            bar, text="■  STOP", bg=C_RED, fg="white",
            activebackground="#da3633", command=self._stop_execution, **bs,
        )
        self.stop_btn.pack(**bp)

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
    def _select_product_type(self, product_type: str):
        self.product_type_var.set(product_type)
        self._log(f"PRODUCT TYPE SELECTED: {product_type}")

    def _next_order_id(self, product_type: str) -> str:
        existing = {order.order_id for order in self.orders}
        while True:
            self._order_counter += 1
            candidate = f"{product_type}{self._order_counter:03d}"
            if candidate not in existing:
                return candidate

    def _register_order(
        self,
        order: Order,
        urgent: bool = False,
        parser_already_added: bool = False,
    ) -> None:
        order = self.order_parser.parse_dict(order.to_dict())
        if any(item.order_id == order.order_id for item in self.orders):
            raise ValueError(f"订单编号重复: {order.order_id}")
        if self.running and self.scene_linked:
            raise RuntimeError(
                "当前 CoppeliaSim 周期只有一套实体工件，运行中不能插入新订单；"
                "请在本轮完成后 RESET，或在调度分析页进行多订单分析。"
            )

        if not parser_already_added:
            self.order_parser.add_order(order)
        self.orders.append(order)
        if self.running and self.orchestrator is not None:
            self.orchestrator.add_order(order, urgent=urgent)
            self.tasks = list(self.orchestrator.tasks)
            self._log(
                f"LIVE ORDER ADDED: {order.order_id}"
                + (" [URGENT]" if urgent else ""),
                "warn" if urgent else "info",
            )
        else:
            self._log(
                f"ORDER QUEUED: {order.order_id} TYPE={order.product_type} "
                f"QTY={order.quantity} PRI={order.priority}"
            )
        self._refresh_order_list()
        self._refresh_task_tree()
        self.scheduling_dashboard.mark_stale()

    def _submit_selected_order(self) -> bool:
        try:
            product_type = self.product_type_var.get().strip().upper()
            order_id = self.order_id_var.get().strip()
            if not order_id:
                order_id = self._next_order_id(product_type)
            order = Order(
                order_id=order_id,
                product_type=product_type,
                priority=int(self.priority_var.get()),
                quantity=int(self.quantity_var.get()),
                due_time=float(self.due_time_var.get()),
            )
            self._register_order(order)
            self.order_id_var.set("")
            return True
        except Exception as exc:
            messagebox.showerror("订单输入错误", str(exc))
            return False

    def _insert_urgent_order(self):
        try:
            product_type = self.product_type_var.get().strip().upper()
            order = Order(
                order_id=self.order_id_var.get().strip()
                or f"URG-{self._next_order_id(product_type)}",
                product_type=product_type,
                priority=10,
                quantity=max(int(self.quantity_var.get()), 1),
                due_time=float(self.due_time_var.get()),
            )
            self._register_order(order, urgent=True)
            self.order_id_var.set("")
            self.status_bar.configure(text="URGENT ORDER ACCEPTED", fg=C_RED)
        except Exception as exc:
            messagebox.showerror("急单输入错误", str(exc))

    def _load_demo_orders(self):
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "data", "orders", "demo_orders.json")
        self._load_orders(path)

    def _load_orders_from_file(self):
        path = filedialog.askopenfilename(title="Load Orders", filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if path:
            self._load_orders(path)

    def _load_orders(self, path: str):
        try:
            if self.running and self.scene_linked:
                raise RuntimeError(
                    "CoppeliaSim 运行中不能加载新订单；请完成后 RESET，"
                    "或先停止仿真再加载。"
                )
            new = self.order_parser.parse_file(path)
            for o in new:
                self._register_order(o, parser_already_added=True)
            self._log(f"LOADED {len(new)} ORDERS FROM {os.path.basename(path)}")
        except Exception as e:
            self._log(f"LOAD FAILED: {e}", "error")
            messagebox.showerror("ERROR", str(e))

    # ============================================================
    # CoppeliaSim 启动与连接
    # ============================================================
    def _launch_or_connect_coppelia(self):
        if not self.scene_linked:
            return
        if self._scene_ready:
            self._on_coppelia_connected(self.sim_bridge.contract_report)
            return
        if self._connection_in_progress:
            return

        self._connection_in_progress = True
        self.cop_btn.configure(state=tk.DISABLED, text="◌  CONNECTING...")
        self._connection_label.configure(text="COP: CONNECTING", fg=C_AMBER)
        self.status_bar.configure(text="STARTING COPPELIASIM...", fg=C_AMBER)
        self._log("CONNECTING TO COPPELIASIM...")
        threading.Thread(
            target=self._connect_coppelia_worker,
            name="coppeliasim-connector",
            daemon=True,
        ).start()

    def _connect_coppelia_worker(self):
        endpoint_present = self.coppelia_manager.endpoint_reachable(
            self.host, self.port
        )
        if endpoint_present:
            if self.sim_bridge.connect(self.host, self.port):
                self._ui_queue.put(
                    ("coppelia_connected", self.sim_bridge.contract_report)
                )
                return
            self._ui_queue.put(
                (
                    "coppelia_failed",
                    "检测到 CoppeliaSim 已运行，但当前场景契约不匹配："
                    + self.sim_bridge.last_error,
                )
            )
            return

        try:
            process = self.coppelia_manager.launch()
        except Exception as exc:
            self._ui_queue.put(("coppelia_failed", str(exc)))
            return

        self._ui_queue.put(("log", ("CoppeliaSim process launched\n", "info")))
        deadline = time.monotonic() + self.coppelia_manager.startup_timeout
        last_error = "ZMQ endpoint is not ready"
        while time.monotonic() < deadline:
            if not self.coppelia_manager.is_owned_process_running():
                self._ui_queue.put(
                    (
                        "coppelia_failed",
                        f"CoppeliaSim exited during startup (code={process.returncode})",
                    )
                )
                return
            time.sleep(0.5)
            if not self.coppelia_manager.endpoint_reachable(
                self.host, self.port
            ):
                continue
            if self.sim_bridge.connect(self.host, self.port):
                self._ui_queue.put(
                    ("coppelia_connected", self.sim_bridge.contract_report)
                )
                return
            last_error = self.sim_bridge.last_error

        self._ui_queue.put(
            (
                "coppelia_failed",
                "等待 CoppeliaSim 场景就绪超时：" + last_error,
            )
        )

    def _on_coppelia_connected(self, report: dict):
        self._connection_in_progress = False
        self._scene_ready = True
        self.cop_btn.configure(state=tk.NORMAL, text="✓  COP CONNECTED")
        self._connection_label.configure(text="COP: CONNECTED", fg=C_GREEN)
        self.status_bar.configure(text="COPPELIASIM READY", fg=C_GREEN)
        self._log(
            "COPPELIASIM CONTRACT OK — "
            f"{report.get('target_count', '?')} TARGETS / "
            f"{len(report.get('robots', {}))} ROBOTS",
            "ok",
        )
        if self._pending_start:
            self._prepare_scene_for_execution()

    def _prepare_scene_for_execution(self):
        if self._scene_preparation_in_progress:
            return
        self._scene_preparation_in_progress = True
        self.start_btn.configure(state=tk.DISABLED, text="◌  PREPARING MOTION")
        self.status_bar.configure(text="VALIDATING R1-R5 PATHS...", fg=C_AMBER)
        self._log("VALIDATING R1-R5 PLANS AND ENTERING SIMULATION READY...")

        def prepare():
            try:
                if not self.sim_bridge.is_connected():
                    raise RuntimeError("CoppeliaSim connection was lost")
                sim = self.sim_bridge.sim
                if sim.getSimulationState() != sim.simulation_stopped:
                    if not self.sim_bridge.stop_simulation():
                        raise RuntimeError(
                            self.sim_bridge.last_error or "无法复位运行中的仿真"
                        )
                from robot_control.simulation_executor import SimulationCellExecutor

                executor = SimulationCellExecutor(self.sim_bridge)
                evidence = executor.prepare_cycle()
                self._ui_queue.put(("scene_prepared", (executor, evidence)))
            except Exception as exc:
                self._ui_queue.put(("coppelia_failed", str(exc)))

        threading.Thread(
            target=prepare,
            name="scene-preparation",
            daemon=True,
        ).start()

    def _on_scene_prepared(self, payload):
        executor, evidence = payload
        self._scene_preparation_in_progress = False
        self.base_robot_executor = executor
        self.robot_executor = executor
        self._log(
            "SIMULATION MOTION READY — "
            f"{evidence.get('path_points_total', '?')} VALIDATED PATH POINTS",
            "ok",
        )
        self._begin_execution()

    def _connection_failed(self, message: str):
        self._connection_in_progress = False
        self._scene_preparation_in_progress = False
        self._scene_ready = False
        self._pending_start = False
        self.cop_btn.configure(state=tk.NORMAL, text="◉  RETRY COP")
        self.start_btn.configure(state=tk.NORMAL, text="▶  START")
        self._connection_label.configure(text="COP: ERROR", fg=C_RED)
        self.status_bar.configure(text="COPPELIASIM ERROR", fg=C_RED)
        self._log(f"COPPELIASIM ERROR: {message}", "error")
        messagebox.showerror("CoppeliaSim 连接失败", message)

    def _stop_scene_async(self):
        if not self.scene_linked or not self._scene_ready:
            return
        executor = self.robot_executor
        bridge = self.sim_bridge

        def stop_scene():
            try:
                if not bridge.is_connected():
                    self._scene_ready = False
                    return
                if isinstance(executor, SceneAwareExecutor):
                    executor.stop_simulation()
                else:
                    bridge.stop_simulation()
            except Exception as exc:
                self._ui_queue.put(
                    ("log", (f"Scene stop warning: {exc}\n", "warn"))
                )

        threading.Thread(target=stop_scene, daemon=True).start()

    # ============================================================
    # 执行引擎
    # ============================================================
    def _start_execution(self):
        if self.running:
            return
        if not self.orders:
            self.status_bar.configure(text="ADDING CURRENT ORDER...", fg=C_AMBER)
            if not self._submit_selected_order():
                self.status_bar.configure(text="ORDER INPUT ERROR", fg=C_RED)
                return
            self._log("START AUTO-SUBMITTED THE CURRENT ORDER", "ok")
        if self.tasks:
            messagebox.showwarning("需要复位", "当前批次已有任务记录，请先点击 RESET。")
            return
        if self.scene_linked:
            if len(self.orders) != 1 or sum(order.quantity for order in self.orders) != 1:
                messagebox.showwarning(
                    "仿真批次限制",
                    "当前运动场景包含一套实体工件，请每次执行 1 个订单、数量 1；"
                    "完成后点击 RESET 再执行下一单。",
                )
                self.status_bar.configure(text="ONE UNIT PER SIM CYCLE", fg=C_AMBER)
                return
            self._pending_start = True
            self.start_btn.configure(state=tk.DISABLED, text="◌  CONNECTING COP")
            self.status_bar.configure(text="CONNECTING COPPELIASIM...", fg=C_AMBER)
            self._log("START REQUESTED — waiting for CoppeliaSim connection")
            if self._scene_ready:
                self._prepare_scene_for_execution()
            else:
                self._launch_or_connect_coppelia()
            return
        self._begin_execution()

    def _begin_execution(self):
        if self.running:
            return
        self.scheduler = Scheduler()
        # SimulationCellExecutor moves the real scene workpieces itself.  A
        # SceneAwareExecutor wrapper would also reveal template products in the
        # legacy embedded stage script, creating duplicate geometry.
        self.robot_executor = self.base_robot_executor
        self.orchestrator = CellOrchestrator(
            self.scheduler,
            self.robot_executor,
        )
        owner = self.orchestrator
        owner.add_event_callback(
            lambda event: self._ui_queue.put(
                ("orchestrator_event", (owner, event))
            )
        )
        self.running = True
        self.paused = False
        self._pending_start = False
        self.start_btn.configure(state=tk.DISABLED, text="●  RUNNING")
        self._mode_label.configure(
            text="COP MOTION" if self.scene_linked else "MOCK RUN",
            bg=C_GREEN,
        )
        self.status_bar.configure(text="EXECUTING...", fg=C_GREEN)

        self._log("=" * 50)
        self._log(f"EXECUTION STARTED — {len(self.orders)} ORDERS")
        new_tasks = owner.start(list(self.orders))
        self.tasks = list(owner.tasks)
        self._refresh_task_tree()
        self._refresh_order_list()
        self._log(f"GENERATED {len(new_tasks)} TASKS")

    def _toggle_pause(self):
        if not self.running or self.orchestrator is None:
            return
        self.paused = not self.paused
        if self.paused:
            self.orchestrator.pause()
            self._log("⏸  PAUSED", "warn")
            self.pause_btn.configure(text="▶  RESUME")
            self.status_bar.configure(text="PAUSED", fg=C_AMBER)
        else:
            self.orchestrator.resume()
            self._log("▶  RESUMED")
            self.pause_btn.configure(text="⏸  PAUSE")
            self.status_bar.configure(text="EXECUTING...", fg=C_GREEN)

    def _stop_execution(self):
        self._pending_start = False
        if self.orchestrator is not None:
            self.orchestrator.stop()
        self.running = False
        self.paused = False
        self.start_btn.configure(state=tk.DISABLED, text="RESET REQUIRED")
        self.pause_btn.configure(text="⏸  PAUSE")
        self.status_bar.configure(text="STOPPING...", fg=C_RED)
        self._log("STOP REQUESTED", "warn")
        if self.scene_linked and self.coppelia_manager.is_owned_process_running():
            threading.Thread(
                target=self.coppelia_manager.terminate_owned_process,
                name="coppeliasim-stop",
                daemon=True,
            ).start()
            self._scene_ready = False
        else:
            self._stop_scene_async()

    def _reset(self):
        previous = self.orchestrator
        if previous is not None:
            previous.stop()
        self.orchestrator = None
        if self.scene_linked and self.coppelia_manager.is_owned_process_running():
            try:
                self.sim_bridge.disconnect()
            except Exception:
                pass
            self.coppelia_manager.terminate_owned_process()
            self.sim_bridge = SimBridge(host=self.host, port=self.port)
            self._scene_ready = False
            self._connection_label.configure(text="COP: OFFLINE", fg=C_RED)
            self.cop_btn.configure(state=tk.NORMAL, text="◉  启动/连接 COP")
        else:
            self._stop_scene_async()
        self.running = False
        self.paused = False
        self.orders.clear()
        self.tasks.clear()
        self.order_parser.clear()
        self.scheduler = Scheduler()
        self.base_robot_executor = MockRobotExecutor()
        self.robot_executor = self.base_robot_executor
        self.order_listbox.delete(0, tk.END)
        for item in self.task_tree.get_children():
            self.task_tree.delete(item)
        self.start_btn.configure(state=tk.NORMAL, text="▶  START")
        self.pause_btn.configure(text="⏸  PAUSE")
        self._mode_label.configure(
            text="COP SIM" if self.scene_linked else "MOCK",
            bg=C_AMBER,
        )
        self.status_bar.configure(text="READY", fg=C_GREEN)
        for k in self.metrics_labels:
            self.metrics_labels[k].configure(text="--")
        self.scheduling_dashboard.reset()
        self._log("=" * 50)
        self._log("SYSTEM RESET")

    def _simulate_fault(self, robot_id: str):
        self.base_robot_executor.set_robot_fault(robot_id)
        self.tasks = self.scheduler.handle_robot_fault(robot_id, self.tasks)
        self._log(f"!!! FAULT INJECTED: {robot_id} — TASKS REASSIGNED", "error")
        self._refresh_task_tree()
        messagebox.showwarning("FAULT INJECTION", f"{robot_id} FAULT\nTasks reassigned to available robots.")

    # ============================================================
    # UI 刷新
    # ============================================================
    def _process_ui_queue(self):
        try:
            while True:
                action, data = self._ui_queue.get_nowait()
                if action == "log":
                    self._log_direct(*data)
                elif action == "update_metrics":
                    self._update_metrics()
                elif action == "orchestrator_event":
                    self._handle_orchestrator_event(*data)
                elif action == "coppelia_connected":
                    self._on_coppelia_connected(data)
                elif action == "scene_prepared":
                    self._on_scene_prepared(data)
                elif action == "coppelia_failed":
                    self._connection_failed(data)
        except queue.Empty:
            pass
        self.root.after(200, self._process_ui_queue)

    def _handle_orchestrator_event(
        self,
        owner: CellOrchestrator,
        event: OrchestratorEvent,
    ):
        if owner is not self.orchestrator:
            return
        self.tasks = list(owner.tasks)
        if event.kind == "task_dispatched":
            task = next(
                (item for item in self.tasks if item.task_id == event.task_id),
                None,
            )
            if task is not None:
                robot = task.available_robots[0] if task.available_robots else "-"
                process = PROCESS_LABELS.get(task.process, task.process)
                self._log(
                    f"DISPATCH {task.task_id} → {robot}  {process} "
                    f"[{task.target_point}]"
                )
        elif event.kind == "task_completed" and event.result is not None:
            result = event.result
            quality = (
                f" QLTY={result.quality_result}"
                if result.quality_result
                else ""
            )
            level = "ok" if result.status == TaskStatus.FINISHED.value else "error"
            self._log(
                f"DONE {result.task_id} [{result.robot_id}] "
                f"STATUS={result.status}{quality}",
                level,
            )
        elif event.kind == "order_added":
            self._log(event.message, "warn")
        elif event.kind in {"finished", "failed", "stopped"}:
            self._on_all_done(event.kind, event.message)

        self._refresh_task_tree()
        self._refresh_order_list()
        self._refresh_robot_panel()
        self._update_metrics()

    def _refresh_order_list(self):
        self.order_listbox.delete(0, tk.END)
        for order in self.orders:
            prefix = order.order_id + "-"
            related = [
                task
                for task in self.tasks
                if task.order_id == order.order_id
                or task.order_id.startswith(prefix)
            ]
            total = len(related)
            completed = sum(
                task.status == TaskStatus.FINISHED.value
                for task in related
            )
            if any(task.status == TaskStatus.FAILED.value for task in related):
                status, color = "失败", C_RED
            elif related and completed == total:
                status, color = "完成", C_GREEN
            elif any(task.status == TaskStatus.RUNNING.value for task in related):
                status, color = "执行中", C_AMBER
            elif related and completed:
                status, color = "处理中", C_BLUE
            elif related:
                status, color = "等待", C_ORANGE
            else:
                status, color = "已排队", C_TEXT_DIM
            progress = f"{completed}/{total}" if total else "0/-"
            self.order_listbox.insert(
                tk.END,
                f" {order.order_id} | {order.product_type}×{order.quantity} "
                f"| P{order.priority} | {status} {progress}",
            )
            self.order_listbox.itemconfig(tk.END, fg=color)

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
        from app.dashboard import compute_runtime_kpi

        try:
            robots = self.robot_executor.get_robot_states()
        except Exception:
            robots = []
        results = (
            self.orchestrator.results_by_task
            if self.orchestrator is not None
            else {}
        )
        kpi = compute_runtime_kpi(
            self.tasks,
            results,
            conflict_count=getattr(self.scheduler, "conflict_count", 0),
            robot_ids=[robot.robot_id for robot in robots],
        )
        self.metrics_labels["makespan"].configure(text=f"{kpi['makespan']:.1f}s")
        self.metrics_labels["utilization"].configure(
            text=f"{kpi['average_utilization'] * 100:.1f}%"
        )
        self.metrics_labels["waiting"].configure(
            text=f"{kpi['avg_waiting_time']:.1f}s"
        )
        self.metrics_labels["conflicts"].configure(text=str(kpi["conflict_count"]))
        self.metrics_labels["completed"].configure(
            text=f"{kpi['completed']}/{kpi['total']}"
        )
        self.metrics_labels["failed"].configure(text=str(kpi["failed"]))

    def _on_all_done(self, status="finished", message=""):
        self.running = False
        self.paused = False
        self.start_btn.configure(state=tk.DISABLED, text="RESET REQUIRED")
        self.pause_btn.configure(text="⏸  PAUSE")
        if status == "finished":
            status_text, color = "ALL ORDERS COMPLETE", C_GREEN
        elif status == "failed":
            status_text, color = "EXECUTION FAILED", C_RED
        else:
            status_text, color = "EXECUTION STOPPED", C_AMBER
        self._mode_label.configure(
            text="COP SIM" if self.scene_linked else "MOCK",
            bg=C_AMBER,
        )
        self.status_bar.configure(text=status_text, fg=color)
        finished = [t for t in self.tasks if t.status == TaskStatus.FINISHED.value]
        failed = [t for t in self.tasks if t.status == TaskStatus.FAILED.value]
        self._log(
            f"{status_text} — DONE: {len(finished)} | FAILED: {len(failed)}"
            + (f" | {message}" if message else ""),
            "ok" if status == "finished" else "warn",
        )
        self._refresh_order_list()
        self._update_metrics()
        if not (
            self.scene_linked
            and self.coppelia_manager.is_owned_process_running()
        ):
            self._stop_scene_async()
        else:
            self._log(
                "FINAL COPPELIASIM STATE HELD — press RESET for a clean cycle",
                "info",
            )

    # ============================================================
    # 导出
    # ============================================================
    def _export_data(self):
        path = filedialog.asksaveasfilename(
            title="Export Data", defaultextension=".json",
            filetypes=[("JSON", "*.json"), ("CSV", "*.csv")],
        )
        if not path:
            return
        try:
            results = (
                self.orchestrator.results_by_task
                if self.orchestrator is not None
                else {}
            )
            from app.dashboard import compute_runtime_kpi

            try:
                robot_ids = [
                    robot.robot_id
                    for robot in self.robot_executor.get_robot_states()
                ]
            except Exception:
                robot_ids = []
            runtime_kpi = compute_runtime_kpi(
                self.tasks,
                results,
                conflict_count=getattr(self.scheduler, "conflict_count", 0),
                robot_ids=robot_ids,
            )
            if path.endswith(".csv"):
                import csv
                with open(path, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow([
                        "task_id", "order_id", "robot_id", "process", "status",
                        "planned_duration", "start_time", "end_time",
                        "actual_duration", "quality_result", "message",
                    ])
                    for t in self.tasks:
                        result = results.get(t.task_id)
                        w.writerow([
                            t.task_id, t.order_id,
                            result.robot_id if result else (
                                t.available_robots[0] if t.available_robots else ""
                            ),
                            t.process, result.status if result else t.status, t.duration,
                            result.start_time if result else "",
                            result.end_time if result else "",
                            max(result.end_time - result.start_time, 0.0) if result else "",
                            result.quality_result if result else "",
                            result.message if result else "",
                        ])
            else:
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(
                        {
                            "orders": [order.to_dict() for order in self.orders],
                            "tasks": [task.to_dict() for task in self.tasks],
                            "results": {
                                task_id: result.to_dict()
                                for task_id, result in results.items()
                            },
                            "runtime_kpi": runtime_kpi,
                            "scheduling_analysis": self.scheduling_dashboard.export_data(),
                        },
                        f,
                        ensure_ascii=False,
                        indent=2,
                    )
            self._log(f"DATA EXPORTED: {path}", "ok")
            messagebox.showinfo("EXPORT OK", f"Saved to:\n{path}")
        except Exception as e:
            self._log(f"EXPORT FAILED: {e}", "error")

    def run(self):
        self.root.mainloop()

    def _on_window_close(self):
        self._pending_start = False
        if self.orchestrator is not None:
            self.orchestrator.stop()
        try:
            self.sim_bridge.disconnect()
        except Exception:
            pass
        self.coppelia_manager.terminate_owned_process()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = Cr5AssemblyApp(root)
    app.run()


if __name__ == "__main__":
    main()
