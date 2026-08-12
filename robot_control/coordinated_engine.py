"""五臂协同装配引擎 (CoppeliaSim)

封装完整的五臂协调流程 (帧同步 + 事件触发 + 等待点保持):
  阶段A: R4/R5 到等待点, R1/R2/R3 并行抓取
  装配:  R1放箱 -> R2放PCB -> R1装端子 -> R3放模块
  后端:  R3转运成品 -> R4锁付 -> R5分拣 -> 全部回位

供 RobotExecutor / 团队调度系统调用:
    engine = CoordinatedEngine()
    engine.run_cycle(quality="good")
    engine.get_status()

底层执行复用 scripts/coordinated_front.py (可独立运行演示)。
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim_bridge.coppelia_client import SimBridge

COORD_SCRIPT = ROOT / "scripts" / "coordinated_front.py"
PIPELINE_SCRIPT = ROOT / "scripts" / "coordinated_pipeline.py"

# 协调流程阶段 (供 app 展示/轮询)
PHASES = [
    "phase_a_parallel_grasp",   # R4/R5 到等待点 + R1/R2/R3 并行抓取
    "box_placed",               # R1 放箱
    "pcb_placed",               # R2 放 PCB
    "terminal_installed",       # R1 装端子
    "module_installed",         # R3 放模块
    "product_transferred",      # R3 转运成品到检测区
    "screw_done",               # R4 锁付完成
    "sorted_done",              # R5 分拣完成
]


class CoordinatedEngine:
    """五臂协同装配引擎."""

    def __init__(self, bridge: Optional[SimBridge] = None, script: Path = COORD_SCRIPT):
        self.bridge = bridge
        self.script = Path(script)
        self._last_result: dict[str, Any] = {}
        self._running = False
        self._urgent_lock = threading.RLock()
        self._urgent_file: Optional[Path] = None
        self._pending_urgent: list[dict[str, str]] = []

    def enqueue_urgent_b(self, order_id: str) -> None:
        """Send one B-type urgent unit to a running pipeline."""
        request = {"order_id": str(order_id), "product_type": "B"}
        with self._urgent_lock:
            if self._urgent_file is None:
                self._pending_urgent.append(request)
                return
            with self._urgent_file.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(request, ensure_ascii=False) + "\n")

    def _activate_urgent_file(self, path: Path) -> None:
        with self._urgent_lock:
            self._urgent_file = path
            path.touch(exist_ok=True)
            pending, self._pending_urgent = self._pending_urgent, []
            if pending:
                with path.open("a", encoding="utf-8") as stream:
                    for request in pending:
                        stream.write(
                            json.dumps(request, ensure_ascii=False) + "\n"
                        )

    @staticmethod
    def _persist_execution_log(
        command: list[str],
        returncode: int | str,
        stdout: str = "",
        stderr: str = "",
    ) -> Path:
        """Persist complete child-process output for post-failure diagnosis."""
        log_dir = ROOT / "log" / "runtime"
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / (
            "coordinated_"
            + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            + ".log"
        )
        path.write_text(
            "COMMAND: " + " ".join(command) + "\n"
            + f"RETURN CODE: {returncode}\n"
            + "\nSTDOUT\n"
            + (stdout or "")
            + "\nSTDERR\n"
            + (stderr or ""),
            encoding="utf-8",
        )
        return path

    # ------------------------------------------------------------------
    # 主入口: 运行一轮完整协调
    # ------------------------------------------------------------------
    def run_cycle(
        self,
        quality: str = "good",
        start_from_wait: bool = False,
        timeout_s: int = 600,
        order_count: int = 1,
        defect_order_index: int = 0,
        keep_running: bool = False,
        reuse_running: bool = False,
    ) -> dict:
        """运行一轮完整五臂装配协调.

        quality: "good" 走合格品分拣路线; "defect" 走缺陷品路线(待实现).
        start_from_wait: True 时各臂从上一轮等待点出发 (多产品连续生产).
        返回 {"status": "ok|failed", "phase": ..., "message": ...}
        """
        self._running = True
        try:
            count = int(order_count)
            if count < 1:
                raise ValueError("order_count must be positive")
            defect_index = int(defect_order_index)
            if defect_index < 0 or defect_index > count:
                raise ValueError("defect_order_index is outside the batch")
            selected_script = self.script if count == 1 else PIPELINE_SCRIPT
            cmd = [sys.executable, str(selected_script)]
            if count > 1:
                cmd.extend(["--orders", str(count)])
                if defect_index:
                    cmd.extend(["--defect-order", str(defect_index)])
            else:
                if start_from_wait:
                    cmd.append("--start-from-wait")
                if keep_running:
                    cmd.append("--keep-running")
                if reuse_running:
                    cmd.append("--reuse-running")
            with tempfile.TemporaryDirectory(prefix="cr5-urgent-", dir="/tmp") as temp:
                urgent_file = Path(temp) / "urgent-orders.jsonl"
                if count > 1:
                    cmd.extend(["--urgent-file", str(urgent_file)])
                self._activate_urgent_file(urgent_file)
                env = dict(os.environ)
                env.setdefault("CR5_SKIP_SCENE_FINGERPRINT", "1")
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=timeout_s,
                    env=env,
                )
            ok = result.returncode == 0
            log_path = self._persist_execution_log(
                cmd,
                result.returncode,
                result.stdout or "",
                result.stderr or "",
            )
            stdout_tail = result.stdout[-2000:] if result.stdout else ""
            stderr_tail = result.stderr[-2000:] if result.stderr else ""
            detail = stdout_tail if ok else (stderr_tail or stdout_tail)
            self._last_result = {
                "status": "ok" if ok else "failed",
                "phase": PHASES[-1] if ok else "error",
                "returncode": result.returncode,
                "message": detail + f"\nfull log: {log_path}",
                "stdout": stdout_tail,
                "stderr": stderr_tail,
                "log_path": str(log_path),
                "order_count": count,
                "defect_order_index": defect_index,
            }
            return self._last_result
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout or ""
            stderr = exc.stderr or ""
            if isinstance(stdout, bytes):
                stdout = stdout.decode(errors="replace")
            if isinstance(stderr, bytes):
                stderr = stderr.decode(errors="replace")
            log_path = self._persist_execution_log(
                cmd,
                "timeout",
                stdout,
                stderr,
            )
            self._last_result = {
                "status": "timeout", "phase": "error",
                "message": (
                    f"协调流程超过 {timeout_s}s；full log: {log_path}"
                ),
                "log_path": str(log_path),
            }
            return self._last_result
        finally:
            with self._urgent_lock:
                self._urgent_file = None
                self._pending_urgent.clear()
            self._running = False

    def run_cycle_async(self, quality: str = "good", callback=None) -> None:
        """异步运行一轮协调 (不阻塞调用方)."""
        import threading

        def _run():
            result = self.run_cycle(quality=quality)
            if callback:
                callback(result)

        threading.Thread(target=_run, daemon=True).start()

    # ------------------------------------------------------------------
    # 状态查询
    # ------------------------------------------------------------------
    def get_status(self) -> dict:
        """返回当前协调/场景状态 (各臂位置, 是否运行中)."""
        if self._running:
            return {"status": "running", "phase": "coordinating"}
        if self._last_result:
            return {
                "status": self._last_result.get("status", "idle"),
                "last_phase": self._last_result.get("phase"),
            }
        return {"status": "idle"}

    def get_robot_states(self) -> dict:
        """查询五臂当前关节角 (度)."""
        bridge = self.bridge or SimBridge(request_timeout=20.0)
        own = self.bridge is None
        if own:
            for _ in range(5):
                if bridge.connect():
                    break
                time.sleep(2)
        if not bridge._connected:
            return {"error": bridge.last_error}
        sim = bridge._client.require("sim")
        states = {}
        for rid in ("R1", "R2", "R3", "R4", "R5"):
            try:
                joints = bridge.get_robot_joint_handles(rid)
                states[rid] = [
                    round(math.degrees(sim.getJointPosition(j)), 2) for j in joints
                ]
            except Exception as exc:
                states[rid] = {"error": str(exc)}
        if own:
            bridge.disconnect()
        return states

    # ------------------------------------------------------------------
    # 辅助: 各臂关键点位 (供 app 展示/调试)
    # ------------------------------------------------------------------
    def load_key_poses(self) -> dict:
        """加载五臂关键姿态定义 (data/captured_paths/r*_key_poses.json)."""
        out = {}
        for robot in ("r1", "r2", "r3", "r4", "r5"):
            f = ROOT / "data" / "captured_paths" / f"{robot}_key_poses.json"
            if f.exists():
                out[robot.upper()] = json.load(open(f))
        return out
