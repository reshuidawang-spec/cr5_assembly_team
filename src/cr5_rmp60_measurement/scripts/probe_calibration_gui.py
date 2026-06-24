#!/usr/bin/env python3
"""Qt5 GUI for CR5 + RMP60 probe pose capture and guarded jog recording."""
import argparse
import csv
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy
from PyQt5.QtCore import QObject, Qt, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from jog_and_record_contacts import (
    AXIS_INDEX,
    format_pose,
    issue_movl,
    jog_target,
    make_contact_row,
    current_pose,
    pose_reached,
    retract_opposite,
    vector_jog_target,
    wait_jog,
)
from probe_touch import PROBE_SPIN_TIMEOUT_SEC, ProbeTouch
from semi_auto_normal_probe import compute_plan, load_fit


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_DIR / "data/gui_probe_contacts.csv"
DEFAULT_NORMAL_FIT = PROJECT_DIR / "data/2026.6.10/yneg_distance_only_coarse_fit_20260610.json"
POSE_NAMES = ("x", "y", "z", "rx", "ry", "rz")


def pose_field_map(prefix, pose):
    """Pose field map."""
    return {
        f"{prefix}_{name}": "" if pose is None else f"{float(pose[index]):.4f}"
        for index, name in enumerate(POSE_NAMES)
    }


def vector_field_map(prefix, values, count, digits=6):
    """Vector field map."""
    return {
        f"{prefix}_{index + 1}": (
            "" if values is None or index >= len(values) else f"{float(values[index]):.{digits}f}"
        )
        for index in range(count)
    }


def write_stable_row(path, row):
    """Write stable row."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    if exists:
        with path.open(newline="") as f:
            header = next(csv.reader(f), None)
        if header != list(row.keys()):
            backup = path.with_name(f"{path.stem}_{int(time.time())}{path.suffix}")
            path.replace(backup)
            exists = False
    with path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def existing_max_sample_index(path):
    """Existing max sample index."""
    path = Path(path)
    if not path.exists():
        return 0
    max_index = 0
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            value = row.get("sample_index") or row.get("gui_sample_index") or ""
            try:
                max_index = max(max_index, int(float(value)))
            except (TypeError, ValueError):
                continue
    return max_index


def make_capture_row(settings, sample_index, snapshot, command="capture_pose"):
    """Make capture row."""
    pose = snapshot.get("pose")
    row = {
        "timestamp": f"{time.time():.3f}",
        "sample_index": str(sample_index),
        "session_id": settings["session_id"],
        "workpiece_id": settings["workpiece_id"],
        "artifact_id": settings["artifact_id"],
        "artifact_type": settings["artifact_type"],
        "physical_ball_id": settings["physical_ball_id"],
        "branch": settings["branch"],
        "operator_note": settings["operator_note"],
        "command": command,
        "linear_step_mm": f"{settings['linear_step_mm']:.4f}",
        "angular_step_deg": f"{settings['angular_step_deg']:.4f}",
        "speed": str(settings["speed"]),
        "auto_retract_mm": f"{settings['auto_retract_mm']:.4f}",
        "trigger_feed_sequence": str(snapshot.get("sequence", "")),
        "trigger_feed_wall_time": (
            "" if snapshot.get("wall_time") is None else f"{float(snapshot['wall_time']):.6f}"
        ),
        "trigger_digital_input_bits": (
            "" if snapshot.get("digital_input_bits") is None else str(snapshot["digital_input_bits"])
        ),
        "trigger_di1": "" if snapshot.get("di1") is None else str(int(bool(snapshot["di1"]))),
    }
    row.update(pose_field_map("start_flange", pose))
    row.update(pose_field_map("target_flange", pose))
    row.update(pose_field_map("trigger_flange", pose))
    row.update(pose_field_map("stop_flange", pose))
    row.update(vector_field_map("trigger_joint", snapshot.get("joints"), 6))
    return row


class RosProbeWorker(QObject):
    connected = pyqtSignal(bool)
    status = pyqtSignal(str, str)
    poseUpdated = pyqtSignal(list, bool, object, float)
    sampleRecorded = pyqtSignal(dict)
    busyChanged = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.node = None
        self.sample_index = 0
        self.busy = False
        self.last_start_pose = None
        self.last_target_pose = None
        self.last_command = ""

    def set_busy(self, value):
        """Set busy."""
        if self.busy != value:
            self.busy = value
            self.busyChanged.emit(value)

    def ensure_node(self):
        """Ensure node."""
        if self.node is None:
            raise RuntimeError("尚未连接 ROS2 Dobot 驱动")
        return self.node

    @staticmethod
    def fresh_feed_di1(node, max_age_sec=0.2):
        """Fresh feed di1."""
        if node.last_feed_time is None:
            raise RuntimeError("尚未收到 FeedInfo，无法判断 DI1")
        age = time.monotonic() - node.last_feed_time
        if age > max_age_sec:
            raise RuntimeError(f"FeedInfo 已过期 {age:.3f}s，拒绝判断 DI1")
        return bool(node.di1)

    @pyqtSlot()
    def connect_robot(self):
        """Connect robot."""
        if self.busy:
            return
        self.set_busy(True)
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            if self.node is None:
                self.node = ProbeTouch()
            self.node.wait_services(10.0)
            self.node.wait_fresh_feed()
            self.status.emit("SUCCESS", "已连接 ROS2 服务并收到 FeedInfo")
            self.connected.emit(True)
            self.refresh()
        except Exception as exc:
            self.status.emit("ERROR", str(exc))
            self.connected.emit(False)
        finally:
            self.set_busy(False)

    @pyqtSlot()
    def shutdown(self):
        """Shutdown."""
        try:
            if self.node is not None:
                self.node.destroy_node()
                self.node = None
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            self.status.emit("WARN", f"关闭 ROS 节点时出错: {exc}")

    @pyqtSlot()
    def refresh(self):
        """Refresh."""
        if self.busy or self.node is None:
            return
        try:
            node = self.ensure_node()
            rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
            snapshot = node.feed_snapshot()
            pose = snapshot.get("pose") or current_pose(node, max_age_sec=0.5)
            di1 = bool(snapshot.get("di1"))
            feed_age = 999.0
            if node.last_feed_time is not None:
                feed_age = time.monotonic() - node.last_feed_time
            self.poseUpdated.emit(list(pose), di1, snapshot.get("joints"), feed_age)
        except Exception as exc:
            self.status.emit("WARN", f"刷新失败: {exc}")

    @pyqtSlot()
    def stop(self):
        """Stop."""
        if self.node is None:
            self.status.emit("WARN", "尚未连接，无法 Stop")
            return
        self.set_busy(True)
        try:
            self.node.stop_fast_then_confirm()
            self.status.emit("WARN", "已发送 Stop")
        except Exception as exc:
            self.status.emit("ERROR", f"Stop 失败: {exc}")
        finally:
            self.set_busy(False)

    @pyqtSlot(dict)
    def retract_last(self, settings):
        """Retract last."""
        if self.busy:
            return
        self.set_busy(True)
        try:
            node = self.ensure_node()
            if self.last_start_pose is None or self.last_target_pose is None:
                self.status.emit("WARN", "没有可用的上一次点动路径，无法自动回退")
                return
            node.stop_fast_then_confirm()
            retract_pose = retract_opposite(
                node,
                self.last_start_pose,
                self.last_target_pose,
                settings["auto_retract_mm"],
                settings["timeout_sec"],
            )
            if retract_pose is None:
                self.status.emit("WARN", "上一次点动没有平移方向，无法按路径回退")
                return
            self.status.emit(
                "SUCCESS",
                f"已按上次路径反向回退 {settings['auto_retract_mm']:.3f}mm: {self.last_command}",
            )
            self.refresh()
        except Exception as exc:
            self.status.emit("ERROR", f"按上次路径回退失败: {exc}")
        finally:
            self.set_busy(False)

    @pyqtSlot(dict)
    def capture_pose(self, settings):
        """Capture pose."""
        if self.busy:
            return
        self.set_busy(True)
        try:
            node = self.ensure_node()
            node.wait_fresh_feed()
            rclpy.spin_once(node, timeout_sec=PROBE_SPIN_TIMEOUT_SEC)
            snapshot = node.feed_snapshot()
            if snapshot.get("pose") is None:
                snapshot["pose"] = current_pose(node, max_age_sec=0.5)
            if snapshot.get("di1") is None:
                snapshot["di1"] = self.fresh_feed_di1(node, max_age_sec=0.5)
            self.sample_index += 1
            row = make_capture_row(settings, self.sample_index, snapshot)
            write_stable_row(settings["output"], row)
            self.sampleRecorded.emit(row)
            self.status.emit("SUCCESS", f"已记录当前位姿: sample {self.sample_index}")
        except Exception as exc:
            self.status.emit("ERROR", f"记录当前位姿失败: {exc}")
        finally:
            self.set_busy(False)

    @pyqtSlot(str, float, dict)
    def jog_axis(self, axis, sign, settings):
        """Jog axis."""
        self._execute_jog(settings, f"{axis}{'+' if sign > 0 else '-'}", lambda pose: jog_target(pose, axis, sign, settings))

    @pyqtSlot(float, float, float, float, dict)
    def jog_vector(self, dx, dy, dz, distance_mm, settings):
        """Jog vector."""
        command = f"vec {dx:g} {dy:g} {dz:g} {distance_mm:g}"
        self._execute_jog(settings, command, lambda pose: vector_jog_target(pose, [dx, dy, dz], distance_mm))

    def _normal_plan_from_settings(self, node, settings):
        search_mm = float(settings["normal_search_mm"])
        if search_mm <= 0.0 or search_mm > 1.0:
            raise RuntimeError("法向搜索距离必须在 0..1.0mm 内")
        fit, sphere_center, local_offset = load_fit(settings["normal_fit_json"])
        fit_branch = fit.get("branch")
        if fit_branch and fit_branch != settings["branch"]:
            raise RuntimeError(f"拟合文件分支 {fit_branch!r} 与当前分支 {settings['branch']!r} 不一致")
        node.wait_fresh_feed()
        pose = current_pose(node, max_age_sec=0.5)
        plan = compute_plan(
            pose,
            sphere_center,
            local_offset,
            settings["normal_euler_sequence"],
            search_mm,
        )
        plan["initial_fit"] = fit
        return plan

    @pyqtSlot(dict)
    def dry_run_normal_probe(self, settings):
        """Dry run normal probe."""
        if self.busy:
            return
        self.set_busy(True)
        try:
            node = self.ensure_node()
            plan = self._normal_plan_from_settings(node, settings)
            approach = plan["approach"]
            target = plan["target_pose"]
            self.status.emit(
                "INFO",
                "法向 dry-run: "
                f"approach=[{approach[0]:.6f}, {approach[1]:.6f}, {approach[2]:.6f}], "
                f"距离={plan['center_distance_mm']:.4f}mm, "
                f"target={format_pose(target)}",
            )
        except Exception as exc:
            self.status.emit("ERROR", f"法向 dry-run 失败: {exc}")
        finally:
            self.set_busy(False)

    @pyqtSlot(dict)
    def jog_normal_vector(self, settings):
        """Jog normal vector."""
        if self.busy:
            return
        try:
            if not settings["normal_ack_safe"]:
                self.status.emit("WARN", "请先勾选已确认路径安全")
                return
            node = self.ensure_node()
            plan = self._normal_plan_from_settings(node, settings)
            approach = plan["approach"]
            distance_mm = float(settings["normal_search_mm"])
            command = f"normal_jog {approach[0]:.6f} {approach[1]:.6f} {approach[2]:.6f} {distance_mm:.4f}"
            self._execute_jog(
                settings,
                command,
                lambda pose, approach=approach, distance_mm=distance_mm: vector_jog_target(
                    pose,
                    approach,
                    distance_mm,
                ),
            )
        except Exception as exc:
            self.status.emit("ERROR", f"法向复合点动失败: {exc}")

    @pyqtSlot(dict)
    def execute_normal_probe(self, settings):
        """Execute normal probe."""
        if self.busy:
            return
        self.set_busy(True)
        try:
            if not settings["normal_ack_safe"]:
                self.status.emit("WARN", "请先勾选已确认法向短探测路径安全")
                return
            node = self.ensure_node()
            plan = self._normal_plan_from_settings(node, settings)
            if self.fresh_feed_di1(node):
                self.status.emit("WARN", "DI1 已经触发，请先退开测针")
                return
            start_pose = plan["flange_pose"]
            target_pose = plan["target_pose"]
            self.last_start_pose = list(start_pose)
            self.last_target_pose = list(target_pose)
            approach = plan["approach"]
            command = (
                f"normal {approach[0]:.6f} {approach[1]:.6f} {approach[2]:.6f} "
                f"{settings['normal_search_mm']:.4f}"
            )
            self.last_command = command
            node.set_speed(settings["speed"])
            future = issue_movl(node, target_pose)
            trigger_snapshot, reached_target = wait_jog(
                node,
                target_pose,
                settings["timeout_sec"],
                settings["position_tolerance_mm"],
                settings["orientation_tolerance_deg"],
            )
            if future.done():
                node.check_ready_future(node.movl_cli, future)
            if trigger_snapshot is None and reached_target:
                self.status.emit("WARN", f"{command}: 到达目标，无 DI1 触发")
                retract_opposite(node, start_pose, target_pose, settings["auto_retract_mm"], settings["timeout_sec"])
                self.refresh()
                return
            stop_pose = current_pose(node, max_age_sec=0.5)
            self.sample_index += 1
            settings["sample_index"] = self.sample_index
            args = SimpleNamespace(
                session_id=settings["session_id"],
                workpiece_id=settings["workpiece_id"],
                artifact_id=settings["artifact_id"],
                artifact_type=settings["artifact_type"],
            )
            row = make_contact_row(args, settings, command, start_pose, target_pose, trigger_snapshot, stop_pose)
            write_stable_row(settings["output"], row)
            retract_opposite(node, start_pose, target_pose, settings["auto_retract_mm"], settings["timeout_sec"])
            self.sampleRecorded.emit(row)
            self.status.emit("SUCCESS", f"{command}: DI1 触发，已记录 sample {self.sample_index}")
            self.refresh()
        except Exception as exc:
            self.status.emit("ERROR", f"法向短探测执行失败: {exc}")
        finally:
            self.set_busy(False)

    def _execute_jog(self, settings, command, target_from_start):
        if self.busy:
            return
        self.set_busy(True)
        try:
            node = self.ensure_node()
            node.wait_fresh_feed()
            if self.fresh_feed_di1(node):
                self.status.emit("WARN", "DI1 已经触发，请先退开测针；如确认安全，可点“按上次路径回退”")
                return
            start_pose = current_pose(node)
            target_pose = target_from_start(start_pose)
            self.last_start_pose = list(start_pose)
            self.last_target_pose = list(target_pose)
            self.last_command = command
            node.set_speed(settings["speed"])
            future = issue_movl(node, target_pose)
            trigger_snapshot, reached_target = wait_jog(
                node,
                target_pose,
                settings["timeout_sec"],
                settings["position_tolerance_mm"],
                settings["orientation_tolerance_deg"],
            )
            if future.done():
                node.check_ready_future(node.movl_cli, future)
            if trigger_snapshot is None and reached_target:
                self.status.emit("INFO", f"{command}: 到达目标，无 DI1 触发")
                self.refresh()
                return
            stop_pose = current_pose(node, max_age_sec=0.5)
            self.sample_index += 1
            settings["sample_index"] = self.sample_index
            args = SimpleNamespace(
                session_id=settings["session_id"],
                workpiece_id=settings["workpiece_id"],
                artifact_id=settings["artifact_id"],
                artifact_type=settings["artifact_type"],
            )
            row = make_contact_row(args, settings, command, start_pose, target_pose, trigger_snapshot, stop_pose)
            write_stable_row(settings["output"], row)
            retract_pose = retract_opposite(
                node,
                start_pose,
                target_pose,
                settings["auto_retract_mm"],
                settings["timeout_sec"],
            )
            if retract_pose is not None:
                row["retract_flange_x"] = f"{retract_pose[0]:.4f}"
                row["retract_flange_y"] = f"{retract_pose[1]:.4f}"
                row["retract_flange_z"] = f"{retract_pose[2]:.4f}"
            self.sampleRecorded.emit(row)
            self.status.emit("SUCCESS", f"{command}: DI1 触发，已记录 sample {self.sample_index}")
            self.refresh()
        except Exception as exc:
            self.status.emit("ERROR", f"{command} 执行失败: {exc}")
        finally:
            self.set_busy(False)


class ProbeCalibrationWindow(QMainWindow):
    connectRequested = pyqtSignal()
    refreshRequested = pyqtSignal()
    stopRequested = pyqtSignal()
    retractLastRequested = pyqtSignal(dict)
    captureRequested = pyqtSignal(dict)
    jogAxisRequested = pyqtSignal(str, float, dict)
    jogVectorRequested = pyqtSignal(float, float, float, float, dict)
    normalDryRunRequested = pyqtSignal(dict)
    normalJogRequested = pyqtSignal(dict)
    normalProbeRequested = pyqtSignal(dict)
    shutdownRequested = pyqtSignal()

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.worker_thread = QThread(self)
        self.worker = RosProbeWorker()
        self.worker.sample_index = existing_max_sample_index(self.args.output)
        self.sample_index_output = str(self.args.output)
        self.worker.moveToThread(self.worker_thread)
        self.shutdownRequested.connect(self.worker.shutdown)
        self.connectRequested.connect(self.worker.connect_robot)
        self.refreshRequested.connect(self.worker.refresh)
        self.stopRequested.connect(self.worker.stop)
        self.retractLastRequested.connect(self.worker.retract_last)
        self.captureRequested.connect(self.worker.capture_pose)
        self.jogAxisRequested.connect(self.worker.jog_axis)
        self.jogVectorRequested.connect(self.worker.jog_vector)
        self.normalDryRunRequested.connect(self.worker.dry_run_normal_probe)
        self.normalJogRequested.connect(self.worker.jog_normal_vector)
        self.normalProbeRequested.connect(self.worker.execute_normal_probe)
        self.worker.connected.connect(self.on_connected)
        self.worker.status.connect(self.append_log)
        self.worker.poseUpdated.connect(self.on_pose_updated)
        self.worker.sampleRecorded.connect(self.on_sample_recorded)
        self.worker.busyChanged.connect(self.on_busy_changed)
        self.worker_thread.start()

        self.pose_labels = {}
        self.joint_labels = []
        self.motion_buttons = []
        self.normal_buttons = []
        self.connected = False
        self.setup_ui()
        self.apply_style()

        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(800)
        self.refresh_timer.timeout.connect(self.refreshRequested.emit)
        self.refresh_timer.start()

    def closeEvent(self, event):
        """CloseEvent."""
        self.refresh_timer.stop()
        self.shutdownRequested.emit()
        self.worker_thread.quit()
        self.worker_thread.wait(2000)
        super().closeEvent(event)

    def setup_ui(self):
        """Setup ui."""
        self.setWindowTitle("CR5 RMP60 标定采集控制台")
        self.resize(1280, 760)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setFrameShape(QScrollArea.NoFrame)
        self.setCentralWidget(scroll)

        root = QWidget()
        root.setMinimumSize(1020, 680)
        scroll.setWidget(root)
        main = QVBoxLayout(root)
        main.setContentsMargins(16, 14, 16, 14)
        main.setSpacing(12)

        header = QHBoxLayout()
        title = QLabel("CR5 RMP60 标定采集控制台")
        title.setObjectName("Title")
        subtitle = QLabel("位姿记录 / 低速点动 / DI1 触发采集")
        subtitle.setObjectName("Subtitle")
        title_box = QVBoxLayout()
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch(1)
        self.connection_badge = QLabel("未连接")
        self.connection_badge.setObjectName("BadgeOff")
        self.di_badge = QLabel("DI1 -")
        self.di_badge.setObjectName("BadgeNeutral")
        self.feed_badge = QLabel("Feed -")
        self.feed_badge.setObjectName("BadgeNeutral")
        header.addWidget(self.feed_badge)
        header.addWidget(self.di_badge)
        header.addWidget(self.connection_badge)
        main.addLayout(header)

        content = QHBoxLayout()
        content.setSpacing(12)
        main.addLayout(content, 1)

        left = QVBoxLayout()
        left.addWidget(self.create_connection_group())
        left.addWidget(self.create_pose_group())
        left.addWidget(self.create_metadata_group())
        content.addLayout(left, 2)

        center = QVBoxLayout()
        center.addWidget(self.create_jog_group())
        center.addWidget(self.create_vector_group())
        center.addWidget(self.create_normal_probe_group())
        center.addWidget(self.create_safety_group())
        content.addLayout(center, 2)

        right = QVBoxLayout()
        right.addWidget(self.create_record_table_group(), 3)
        right.addWidget(self.create_log_group(), 2)
        content.addLayout(right, 3)

    def create_connection_group(self):
        """Create connection group."""
        group = QGroupBox("连接与采集")
        layout = QGridLayout(group)
        self.connect_btn = QPushButton("连接 ROS2 驱动")
        self.refresh_btn = QPushButton("刷新位姿")
        self.capture_btn = QPushButton("记录当前位姿")
        self.retract_last_btn = QPushButton("按上次路径回退")
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setObjectName("StopButton")
        self.connect_btn.clicked.connect(self.connectRequested.emit)
        self.refresh_btn.clicked.connect(self.refreshRequested.emit)
        self.capture_btn.clicked.connect(lambda: self.captureRequested.emit(self.settings()))
        self.retract_last_btn.clicked.connect(lambda: self.retractLastRequested.emit(self.settings()))
        self.stop_btn.clicked.connect(self.stopRequested.emit)
        layout.addWidget(self.connect_btn, 0, 0)
        layout.addWidget(self.refresh_btn, 0, 1)
        layout.addWidget(self.capture_btn, 1, 0)
        layout.addWidget(self.retract_last_btn, 1, 1)
        layout.addWidget(self.stop_btn, 2, 0, 1, 2)
        return group

    def create_pose_group(self):
        """Create pose group."""
        group = QGroupBox("实时法兰位姿")
        layout = QGridLayout(group)
        for index, name in enumerate(POSE_NAMES):
            label = QLabel(name.upper())
            value = QLabel("--")
            value.setObjectName("PoseValue")
            self.pose_labels[name] = value
            layout.addWidget(label, index // 3 * 2, index % 3)
            layout.addWidget(value, index // 3 * 2 + 1, index % 3)
        joint_title = QLabel("关节角")
        joint_title.setObjectName("SectionLabel")
        layout.addWidget(joint_title, 4, 0, 1, 3)
        for index in range(6):
            value = QLabel(f"J{index + 1}: --")
            value.setObjectName("JointValue")
            self.joint_labels.append(value)
            layout.addWidget(value, 5 + index // 3, index % 3)
        return group

    def create_metadata_group(self):
        """Create metadata group."""
        group = QGroupBox("数据元信息")
        layout = QFormLayout(group)
        self.output_edit = QLineEdit(str(self.args.output))
        output_row = QHBoxLayout()
        output_row.addWidget(self.output_edit, 1)
        browse = QPushButton("选择/追加")
        browse.clicked.connect(self.choose_output)
        output_row.addWidget(browse)
        self.session_edit = QLineEdit(self.args.session_id)
        self.workpiece_edit = QLineEdit(self.args.workpiece_id)
        self.artifact_edit = QLineEdit(self.args.artifact_id)
        self.ball_edit = QLineEdit(self.args.physical_ball_id)
        self.branch_combo = QComboBox()
        self.branch_combo.addItems(["y_neg", "y_pos", "x_neg", "x_pos", "z_down", "unknown"])
        self.branch_combo.setCurrentText(self.args.branch)
        self.note_edit = QLineEdit(self.args.operator_note)
        layout.addRow("输出 CSV", output_row)
        layout.addRow("Session", self.session_edit)
        layout.addRow("工件 ID", self.workpiece_edit)
        layout.addRow("标准件 ID", self.artifact_edit)
        layout.addRow("测球编号", self.ball_edit)
        layout.addRow("分支标签", self.branch_combo)
        layout.addRow("备注", self.note_edit)
        return group

    def create_jog_group(self):
        """Create jog group."""
        group = QGroupBox("坐标点动")
        layout = QGridLayout(group)
        self.linear_step_spin = self.double_spin(0.01, 10.0, self.args.linear_step_mm, 3, 0.1)
        self.angular_step_spin = self.double_spin(0.1, 20.0, self.args.angular_step_deg, 2, 0.5)
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 5)
        self.speed_spin.setValue(self.args.speed)
        self.retract_spin = self.double_spin(0.0, 10.0, self.args.auto_retract_mm, 3, 0.1)
        layout.addWidget(QLabel("平移步长 mm"), 0, 0)
        layout.addWidget(self.linear_step_spin, 0, 1)
        layout.addWidget(QLabel("转动步长 deg"), 0, 2)
        layout.addWidget(self.angular_step_spin, 0, 3)
        layout.addWidget(QLabel("速度倍率"), 1, 0)
        layout.addWidget(self.speed_spin, 1, 1)
        layout.addWidget(QLabel("触发回退 mm"), 1, 2)
        layout.addWidget(self.retract_spin, 1, 3)

        buttons = [
            ("X-", "x", -1.0, 2, 0), ("X+", "x", 1.0, 2, 1),
            ("Y-", "y", -1.0, 2, 2), ("Y+", "y", 1.0, 2, 3),
            ("Z-", "z", -1.0, 3, 0), ("Z+", "z", 1.0, 3, 1),
            ("RX-", "rx", -1.0, 3, 2), ("RX+", "rx", 1.0, 3, 3),
            ("RY-", "ry", -1.0, 4, 0), ("RY+", "ry", 1.0, 4, 1),
            ("RZ-", "rz", -1.0, 4, 2), ("RZ+", "rz", 1.0, 4, 3),
        ]
        for text, axis, sign, row, col in buttons:
            btn = QPushButton(text)
            btn.clicked.connect(lambda _, axis=axis, sign=sign: self.jogAxisRequested.emit(axis, sign, self.settings()))
            self.motion_buttons.append(btn)
            layout.addWidget(btn, row, col)
        return group

    def create_vector_group(self):
        """Create vector group."""
        group = QGroupBox("复合方向点动")
        layout = QGridLayout(group)
        self.vec_x_spin = self.double_spin(-10.0, 10.0, -1.0, 3, 0.1)
        self.vec_y_spin = self.double_spin(-10.0, 10.0, 1.0, 3, 0.1)
        self.vec_z_spin = self.double_spin(-10.0, 10.0, 0.0, 3, 0.1)
        self.vec_dist_spin = self.double_spin(0.01, 20.0, 0.2, 3, 0.1)
        self.vec_btn = QPushButton("执行复合点动")
        self.vec_btn.clicked.connect(self.emit_vector_jog)
        layout.addWidget(QLabel("dx"), 0, 0)
        layout.addWidget(self.vec_x_spin, 0, 1)
        layout.addWidget(QLabel("dy"), 0, 2)
        layout.addWidget(self.vec_y_spin, 0, 3)
        layout.addWidget(QLabel("dz"), 1, 0)
        layout.addWidget(self.vec_z_spin, 1, 1)
        layout.addWidget(QLabel("距离 mm"), 1, 2)
        layout.addWidget(self.vec_dist_spin, 1, 3)
        layout.addWidget(self.vec_btn, 2, 0, 1, 4)
        hint = QLabel("当前常用：dx=-1, dy=1, dz=0，用于 X-/Y+ 复合接近。")
        hint.setObjectName("Hint")
        layout.addWidget(hint, 3, 0, 1, 4)
        return group

    def create_normal_probe_group(self):
        """Create normal probe group."""
        group = QGroupBox("半自动法向")
        layout = QGridLayout(group)
        self.normal_fit_edit = QLineEdit(self.args.normal_fit_json)
        browse = QPushButton("选择")
        browse.clicked.connect(self.choose_normal_fit)
        layout.addWidget(QLabel("拟合 JSON"), 0, 0)
        layout.addWidget(self.normal_fit_edit, 0, 1, 1, 2)
        layout.addWidget(browse, 0, 3)

        self.normal_search_spin = self.double_spin(0.01, 1.0, self.args.normal_search_mm, 3, 0.05)
        self.normal_ack_check = QCheckBox("已确认路径安全")
        layout.addWidget(QLabel("搜索 mm"), 1, 0)
        layout.addWidget(self.normal_search_spin, 1, 1)
        layout.addWidget(self.normal_ack_check, 1, 2, 1, 2)

        self.normal_dry_btn = QPushButton("计算法向")
        self.normal_jog_btn = QPushButton("沿法向点动")
        self.normal_execute_btn = QPushButton("执行短探测")
        self.normal_dry_btn.clicked.connect(lambda: self.normalDryRunRequested.emit(self.settings()))
        self.normal_jog_btn.clicked.connect(lambda: self.normalJogRequested.emit(self.settings()))
        self.normal_execute_btn.clicked.connect(lambda: self.normalProbeRequested.emit(self.settings()))
        self.normal_buttons.extend([self.normal_dry_btn, self.normal_jog_btn, self.normal_execute_btn])
        layout.addWidget(self.normal_dry_btn, 2, 0)
        layout.addWidget(self.normal_jog_btn, 2, 1)
        layout.addWidget(self.normal_execute_btn, 2, 2, 1, 2)
        return group

    def create_safety_group(self):
        """Create safety group."""
        group = QGroupBox("执行约束")
        layout = QFormLayout(group)
        self.timeout_spin = self.double_spin(1.0, 30.0, self.args.timeout_sec, 1, 1.0)
        self.pos_tol_spin = self.double_spin(0.01, 1.0, self.args.position_tolerance_mm, 3, 0.01)
        self.rot_tol_spin = self.double_spin(0.01, 2.0, self.args.orientation_tolerance_deg, 3, 0.01)
        layout.addRow("单步超时 s", self.timeout_spin)
        layout.addRow("到位位置容差 mm", self.pos_tol_spin)
        layout.addRow("到位姿态容差 deg", self.rot_tol_spin)
        warning = QLabel("运动前必须确认现场路径安全；DI1 已触发时拒绝点动。")
        warning.setWordWrap(True)
        warning.setObjectName("WarningText")
        layout.addRow(warning)
        return group

    def create_record_table_group(self):
        """Create record table group."""
        group = QGroupBox("采集记录")
        layout = QVBoxLayout(group)
        self.record_table = QTableWidget(0, 7)
        self.record_table.setHorizontalHeaderLabels(["#", "命令", "X", "Y", "Z", "DI1", "备注"])
        self.record_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.record_table.verticalHeader().setVisible(False)
        layout.addWidget(self.record_table)
        return group

    def create_log_group(self):
        """Create log group."""
        group = QGroupBox("状态日志")
        layout = QVBoxLayout(group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        return group

    def settings(self):
        """Settings."""
        output = self.output_edit.text().strip() or str(DEFAULT_OUTPUT)
        if output != self.sample_index_output:
            self.worker.sample_index = existing_max_sample_index(output)
            self.sample_index_output = output
        return {
            "output": output,
            "session_id": self.session_edit.text().strip(),
            "workpiece_id": self.workpiece_edit.text().strip(),
            "artifact_id": self.artifact_edit.text().strip(),
            "artifact_type": "sphere",
            "physical_ball_id": self.ball_edit.text().strip(),
            "branch": self.branch_combo.currentText(),
            "operator_note": self.note_edit.text().strip(),
            "linear_step_mm": self.linear_step_spin.value(),
            "angular_step_deg": self.angular_step_spin.value(),
            "speed": self.speed_spin.value(),
            "auto_retract_mm": self.retract_spin.value(),
            "timeout_sec": self.timeout_spin.value(),
            "position_tolerance_mm": self.pos_tol_spin.value(),
            "orientation_tolerance_deg": self.rot_tol_spin.value(),
            "normal_fit_json": self.normal_fit_edit.text().strip(),
            "normal_search_mm": self.normal_search_spin.value(),
            "normal_ack_safe": self.normal_ack_check.isChecked(),
            "normal_euler_sequence": self.args.normal_euler_sequence,
            "sample_index": 0,
        }

    def emit_vector_jog(self):
        """Emit vector jog."""
        dx = self.vec_x_spin.value()
        dy = self.vec_y_spin.value()
        dz = self.vec_z_spin.value()
        if math.sqrt(dx * dx + dy * dy + dz * dz) <= 1e-9:
            QMessageBox.warning(self, "方向错误", "复合方向不能为零向量。")
            return
        self.jogVectorRequested.emit(dx, dy, dz, self.vec_dist_spin.value(), self.settings())

    def choose_output(self):
        """Choose output."""
        dialog = QFileDialog(self, "选择要追加的输出 CSV")
        dialog.setNameFilter("CSV (*.csv)")
        dialog.setAcceptMode(QFileDialog.AcceptOpen)
        dialog.setFileMode(QFileDialog.AnyFile)
        current = self.output_edit.text().strip()
        if current:
            dialog.selectFile(current)
        if dialog.exec_() != QFileDialog.Accepted:
            return
        selected = dialog.selectedFiles()
        if not selected:
            return
        path = selected[0]
        self.output_edit.setText(path)
        self.worker.sample_index = existing_max_sample_index(path)
        self.sample_index_output = path
        self.append_log("INFO", f"输出 CSV 已选择为追加模式；下一条 sample 从 {self.worker.sample_index + 1} 开始")

    def choose_normal_fit(self):
        """Choose normal fit."""
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择粗拟合 JSON",
            self.normal_fit_edit.text(),
            "JSON (*.json)",
        )
        if path:
            self.normal_fit_edit.setText(path)

    def on_connected(self, connected):
        """On connected."""
        self.connected = connected
        self.connection_badge.setText("已连接" if connected else "未连接")
        self.connection_badge.setObjectName("BadgeOn" if connected else "BadgeOff")
        self.connection_badge.style().unpolish(self.connection_badge)
        self.connection_badge.style().polish(self.connection_badge)

    def on_pose_updated(self, pose, di1, joints, feed_age):
        """On pose updated."""
        for index, name in enumerate(POSE_NAMES):
            self.pose_labels[name].setText(f"{pose[index]:.4f}")
        if joints:
            for index, value in enumerate(joints[:6]):
                self.joint_labels[index].setText(f"J{index + 1}: {float(value):.4f}")
        self.di_badge.setText(f"DI1 {int(di1)}")
        self.di_badge.setObjectName("BadgeWarn" if di1 else "BadgeOn")
        self.feed_badge.setText(f"Feed {feed_age:.2f}s")
        self.feed_badge.setObjectName("BadgeWarn" if feed_age > 0.2 else "BadgeOn")
        for label in (self.di_badge, self.feed_badge):
            label.style().unpolish(label)
            label.style().polish(label)

    def on_sample_recorded(self, row):
        """On sample recorded."""
        r = self.record_table.rowCount()
        self.record_table.insertRow(r)
        values = [
            row.get("sample_index", ""),
            row.get("command", ""),
            row.get("trigger_flange_x", ""),
            row.get("trigger_flange_y", ""),
            row.get("trigger_flange_z", ""),
            row.get("trigger_di1", ""),
            row.get("operator_note", ""),
        ]
        for col, value in enumerate(values):
            item = QTableWidgetItem(str(value))
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.record_table.setItem(r, col, item)
        self.record_table.scrollToBottom()

    def on_busy_changed(self, busy):
        """On busy changed."""
        self.connect_btn.setEnabled(not busy)
        self.refresh_btn.setEnabled(not busy)
        self.capture_btn.setEnabled(not busy)
        self.retract_last_btn.setEnabled(not busy)
        self.vec_btn.setEnabled(not busy)
        for btn in self.motion_buttons:
            btn.setEnabled(not busy)
        for btn in self.normal_buttons:
            btn.setEnabled(not busy)

    def append_log(self, level, message):
        """Append log."""
        color = {
            "SUCCESS": "#20C997",
            "WARN": "#F59F00",
            "ERROR": "#FF6B6B",
            "INFO": "#74C0FC",
        }.get(level, "#DEE2E6")
        ts = time.strftime("%H:%M:%S")
        self.log_text.append(f'<span style="color:#868E96">[{ts}]</span> '
                             f'<span style="color:{color};font-weight:600">{level}</span> '
                             f'<span style="color:#DEE2E6">{message}</span>')

    @staticmethod
    def double_spin(minimum, maximum, value, decimals, step):
        """Double spin."""
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setKeyboardTracking(False)
        return spin

    def apply_style(self):
        """Apply style."""
        app = QApplication.instance()
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor("#151A1F"))
        palette.setColor(QPalette.WindowText, QColor("#E9ECEF"))
        palette.setColor(QPalette.Base, QColor("#0F1317"))
        palette.setColor(QPalette.AlternateBase, QColor("#20262D"))
        palette.setColor(QPalette.Text, QColor("#F1F3F5"))
        palette.setColor(QPalette.Button, QColor("#2B333B"))
        palette.setColor(QPalette.ButtonText, QColor("#F8F9FA"))
        palette.setColor(QPalette.Highlight, QColor("#1971C2"))
        palette.setColor(QPalette.HighlightedText, QColor("#FFFFFF"))
        app.setPalette(palette)
        app.setFont(QFont("Noto Sans CJK SC", 10))
        self.setStyleSheet("""
            QMainWindow { background: #151A1F; }
            QGroupBox {
                border: 1px solid #343A40;
                border-radius: 6px;
                margin-top: 12px;
                padding: 10px;
                background: #1B2026;
                color: #F1F3F5;
                font-weight: 600;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QLabel { color: #DDE2E6; }
            QLabel#Title { font-size: 24px; font-weight: 700; color: #F8F9FA; }
            QLabel#Subtitle { color: #ADB5BD; }
            QLabel#PoseValue {
                color: #69DB7C;
                background: #0F1317;
                border: 1px solid #2B333B;
                border-radius: 4px;
                padding: 8px;
                font-family: "DejaVu Sans Mono";
                font-size: 16px;
                font-weight: 700;
            }
            QLabel#JointValue { color: #CED4DA; font-family: "DejaVu Sans Mono"; }
            QLabel#SectionLabel { color: #74C0FC; font-weight: 700; margin-top: 8px; }
            QLabel#Hint { color: #ADB5BD; }
            QLabel#WarningText { color: #FFD43B; }
            QLabel#BadgeOn, QLabel#BadgeOff, QLabel#BadgeWarn, QLabel#BadgeNeutral {
                border-radius: 4px;
                padding: 7px 12px;
                font-weight: 700;
                min-width: 72px;
                qproperty-alignment: AlignCenter;
            }
            QLabel#BadgeOn { background: #087F5B; color: #E6FCF5; }
            QLabel#BadgeOff { background: #862E2E; color: #FFF5F5; }
            QLabel#BadgeWarn { background: #E67700; color: #FFF9DB; }
            QLabel#BadgeNeutral { background: #343A40; color: #DEE2E6; }
            QPushButton {
                background: #2B333B;
                border: 1px solid #495057;
                border-radius: 5px;
                padding: 9px 12px;
                color: #F8F9FA;
                font-weight: 600;
            }
            QPushButton:hover { background: #36424C; border-color: #74C0FC; }
            QPushButton:pressed { background: #1971C2; }
            QPushButton:disabled { background: #212529; color: #6C757D; border-color: #343A40; }
            QPushButton#StopButton { background: #C92A2A; border-color: #FF8787; color: white; }
            QPushButton#StopButton:hover { background: #E03131; }
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
                background: #0F1317;
                color: #F1F3F5;
                border: 1px solid #495057;
                border-radius: 4px;
                padding: 6px;
            }
            QTextEdit, QTableWidget {
                background: #0F1317;
                color: #DEE2E6;
                border: 1px solid #343A40;
                border-radius: 4px;
            }
            QHeaderView::section {
                background: #2B333B;
                color: #F8F9FA;
                border: 0;
                padding: 7px;
                font-weight: 700;
            }
        """)


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--session-id", default="session_gui_probe_calibration")
    parser.add_argument("--workpiece-id", default="calibration_sphere_20mm")
    parser.add_argument("--artifact-id", default="standard_sphere_20mm")
    parser.add_argument("--physical-ball-id", default="1")
    parser.add_argument("--branch", default="y_neg")
    parser.add_argument("--operator-note", default="")
    parser.add_argument("--linear-step-mm", type=float, default=0.2)
    parser.add_argument("--angular-step-deg", type=float, default=1.0)
    parser.add_argument("--auto-retract-mm", type=float, default=1.0)
    parser.add_argument("--speed", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=5.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=0.08)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=0.08)
    parser.add_argument("--normal-fit-json", default=str(DEFAULT_NORMAL_FIT))
    parser.add_argument("--normal-search-mm", type=float, default=0.2)
    parser.add_argument("--normal-euler-sequence", default="xyz")
    return parser.parse_args()


def main():
    """Main."""
    args = parse_args()
    app = QApplication(sys.argv)
    window = ProbeCalibrationWindow(args)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
