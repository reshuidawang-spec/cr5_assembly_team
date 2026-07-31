"""Observe the first real joint movement through a separate ZMQ connection."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Optional

from sim_bridge.coppelia_client import SimBridge


class JointMotionMonitor:
    def __init__(
        self,
        host: str,
        port: int,
        robot_id: str,
        threshold_deg: float = 0.02,
        poll_interval_s: float = 0.02,
        bridge_factory: Callable[..., SimBridge] = SimBridge,
        disconnect_on_stop: bool = True,
    ):
        self.host = host
        self.port = int(port)
        self.robot_id = robot_id
        self.threshold_deg = float(threshold_deg)
        self.poll_interval_s = float(poll_interval_s)
        self.bridge_factory = bridge_factory
        self.disconnect_on_stop = bool(disconnect_on_stop)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._armed = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._dispatch_monotonic_s: Optional[float] = None
        self._result: dict[str, Any] = {
            "robot_id": robot_id,
            "motion_detected": False,
            "threshold_deg": self.threshold_deg,
            "dispatch_wall_epoch_s": 0.0,
            "dispatch_simulation_time_s": None,
            "first_motion_wall_epoch_s": None,
            "first_motion_simulation_time_s": None,
            "dispatch_to_first_motion_wall_s": None,
            "max_joint_delta_deg_at_detection": None,
            "monitor_error": "",
        }

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("joint motion monitor already started")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=2.0):
            self._result["monitor_error"] = (
                "joint motion monitor baseline was not ready within 2.0 s"
            )
            self._stop.set()
            return
        if self._result["monitor_error"]:
            return
        self._result["dispatch_wall_epoch_s"] = time.time()
        self._dispatch_monotonic_s = time.monotonic()
        self._armed.set()

    def _run(self) -> None:
        bridge = self.bridge_factory(host=self.host, port=self.port)
        try:
            if not bridge.is_connected() and not bridge.connect():
                raise RuntimeError(bridge.last_error or "monitor connection failed")
            baseline = bridge.get_robot_joint_positions(self.robot_id)
            self._result["dispatch_simulation_time_s"] = float(
                bridge.sim.getSimulationTime()
            )
            self._ready.set()
            while not self._armed.is_set() and not self._stop.is_set():
                self._stop.wait(0.001)
            while not self._stop.is_set():
                current = bridge.get_robot_joint_positions(self.robot_id)
                if len(baseline) != 6 or len(current) != 6:
                    raise RuntimeError(
                        f"expected six joints for {self.robot_id}, got "
                        f"{len(baseline)} baseline and {len(current)} current"
                    )
                delta_deg = max(
                    abs(math.degrees(after - before))
                    for before, after in zip(baseline, current)
                )
                if delta_deg >= self.threshold_deg:
                    first_wall = time.time()
                    first_monotonic = time.monotonic()
                    self._result.update(
                        {
                            "motion_detected": True,
                            "first_motion_wall_epoch_s": first_wall,
                            "first_motion_simulation_time_s": float(
                                bridge.sim.getSimulationTime()
                            ),
                            "dispatch_to_first_motion_wall_s": (
                                None
                                if self._dispatch_monotonic_s is None
                                else first_monotonic
                                - self._dispatch_monotonic_s
                            ),
                            "max_joint_delta_deg_at_detection": delta_deg,
                        }
                    )
                    return
                self._stop.wait(self.poll_interval_s)
        except Exception as exc:
            self._result["monitor_error"] = str(exc)
            self._ready.set()
        finally:
            if self.disconnect_on_stop:
                try:
                    bridge.disconnect()
                except Exception:
                    pass

    def stop(self) -> dict[str, Any]:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive() and not self._result["monitor_error"]:
                self._result["monitor_error"] = (
                    "joint motion monitor did not stop within 2.0 s"
                )
        return dict(self._result)


class _PersistentMonitorLease:
    def __init__(self, monitor: JointMotionMonitor, lock: threading.Lock):
        self.monitor = monitor
        self.lock = lock
        self.acquired = False

    def start(self) -> None:
        self.lock.acquire()
        self.acquired = True
        try:
            self.monitor.start()
        except Exception:
            self.acquired = False
            self.lock.release()
            raise

    def stop(self) -> dict[str, Any]:
        try:
            return self.monitor.stop()
        finally:
            if self.acquired:
                self.acquired = False
                self.lock.release()


class PersistentJointMotionMonitorFactory:
    """Reuse one observation connection across a sequential real cycle."""

    def __init__(
        self,
        host: str,
        port: int,
        threshold_deg: float = 0.02,
        poll_interval_s: float = 0.02,
        bridge_factory: Callable[..., SimBridge] = SimBridge,
    ):
        self.host = host
        self.port = int(port)
        self.threshold_deg = float(threshold_deg)
        self.poll_interval_s = float(poll_interval_s)
        self.bridge = bridge_factory(host=host, port=port)
        self._lock = threading.Lock()

    def prepare(self) -> None:
        with self._lock:
            if not self.bridge.is_connected() and not self.bridge.connect():
                raise RuntimeError(
                    self.bridge.last_error or "monitor READY connection failed"
                )

    def close(self) -> None:
        with self._lock:
            self.bridge.disconnect()

    def __call__(self, robot_id: str) -> _PersistentMonitorLease:
        monitor = JointMotionMonitor(
            self.host,
            self.port,
            robot_id,
            threshold_deg=self.threshold_deg,
            poll_interval_s=self.poll_interval_s,
            bridge_factory=lambda **kwargs: self.bridge,
            disconnect_on_stop=False,
        )
        return _PersistentMonitorLease(monitor, self._lock)


__all__ = ["JointMotionMonitor", "PersistentJointMotionMonitorFactory"]
