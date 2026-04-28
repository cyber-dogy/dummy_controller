#!/usr/bin/env python3
"""
示教功能对话框 v3 - 重新设计
流程: 连接 → 使能 → 进入示教(降低增益) → 拖动+记录 → 退出示教(恢复增益) → 回放
"""

import os
from datetime import datetime

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QProgressBar, QTextEdit, QFileDialog, QMessageBox,
    QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QSlider, QFrame
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
import numpy as np

from core.gripper import normalize_gripper_value


# ── 回放线程 ────────────────────────────────────────────────────────────────

class PlaybackThread(QThread):
    progress_signal = pyqtSignal(int)
    finished_signal = pyqtSignal()
    log_signal = pyqtSignal(str)

    def __init__(self, robot, trajectory, speed_ratio):
        super().__init__()
        self.robot = robot
        self.trajectory = trajectory
        self.speed_ratio = speed_ratio
        self.stopped = False

    def run(self):
        import time
        traj = self.trajectory
        n = len(traj)
        self.log_signal.emit(f"开始回放 {n} 个点，速度 {self.speed_ratio:.1f}x")

        last_gripper = None
        for idx, point in enumerate(traj):
            if self.stopped:
                break
            target_angles = point.playback_angles() if hasattr(point, "playback_angles") else point.angles
            ok, msg = self.robot.move_to(target_angles, check_limits=False)
            if not ok:
                self.log_signal.emit(f"第 {idx} 点发送失败: {msg}")

            if hasattr(self.robot, "set_gripper_state"):
                target_gripper = normalize_gripper_value(
                    point.playback_gripper() if hasattr(point, "playback_gripper") else getattr(point, "gripper", 0.0)
                )
                if idx == 0 or abs(target_gripper - last_gripper) >= 1e-6:
                    self.robot.set_gripper_state(target_gripper)
                    last_gripper = target_gripper
                elif target_gripper < 0.5 and hasattr(self.robot, "refresh_gripper_hold"):
                    self.robot.refresh_gripper_hold()

            if idx < n - 1:
                dt = (traj[idx + 1].timestamp - point.timestamp) / self.speed_ratio
                time.sleep(max(0.02, dt))

            self.progress_signal.emit(int((idx + 1) / n * 100))

        self.finished_signal.emit()

    def stop(self):
        self.stopped = True


# ── 主对话框 ─────────────────────────────────────────────────────────────────

class TeachDialog(QDialog):

    # 后台线程 → 主线程更新点数（线程安全）
    record_count_signal = pyqtSignal(int)

    def __init__(self, robot, parent=None):
        super().__init__(parent)
        self.robot = robot
        self.teach_mode = None
        self.recording = False
        self.in_teach_mode = False
        self.current_trajectory = []
        self.playback_thread = None

        self.setWindowTitle("示教功能")
        self.setMinimumSize(620, 680)
        self._build_ui()

        # 定时刷新状态栏（500ms，主线程安全）
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(500)

        # signal → label（线程安全更新点数）
        self.record_count_signal.connect(
            lambda n: self.lbl_record_status.setText(f"记录中…  {n} 点")
        )

        try:
            from core.teach_mode import TeachMode
            self.teach_mode = TeachMode(robot)
            self.teach_mode.on_record_callback = self._on_record_point
        except Exception as e:
            self._log(f"示教模块初始化失败: {e}")

    # ── UI 构建 ──────────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(8)

        # ── 1. 状态栏 ────────────────────────────────────────────────────────
        status_frame = QFrame()
        status_frame.setFrameShape(QFrame.Shape.StyledPanel)
        status_frame.setStyleSheet("background:#f0f4f8; border-radius:4px;")
        sl = QHBoxLayout(status_frame)
        sl.setContentsMargins(10, 6, 10, 6)

        self.lbl_conn = QLabel("● 未连接")
        self.lbl_conn.setStyleSheet("color:red; font-weight:bold;")
        self.lbl_enable = QLabel("○ 未使能")
        self.lbl_enable.setStyleSheet("color:gray; font-weight:bold;")

        self.btn_enable = QPushButton("使能电机")
        self.btn_enable.setFixedWidth(80)
        self.btn_enable.clicked.connect(self._toggle_enable)

        btn_estop = QPushButton("急停")
        btn_estop.setFixedWidth(60)
        btn_estop.setStyleSheet("background:#f44336; color:white; font-weight:bold;")
        btn_estop.clicked.connect(self._emergency_stop)

        sl.addWidget(self.lbl_conn)
        sl.addSpacing(16)
        sl.addWidget(self.lbl_enable)
        sl.addStretch()
        sl.addWidget(self.btn_enable)
        sl.addWidget(btn_estop)
        root.addWidget(status_frame)

        # ── 2. 示教模式 ───────────────────────────────────────────────────────
        teach_group = QGroupBox("示教模式")
        tl = QVBoxLayout(teach_group)

        info = QLabel(
            "进入示教模式后发送 !DISABLE，电机停止位置控制，手感与断电相同（只剩减速器机械摩擦）。\n"
            "退出时先读当前位置，再上电并立即发送保持指令，不会突然跳位。\n"
            "注意：减速比较大（50-100:1），拖动需要一定力度，臂会因重力略有下沉。"
        )
        info.setStyleSheet("color:#555; font-size:10px;")
        info.setWordWrap(True)
        tl.addWidget(info)

        teach_btn_row = QHBoxLayout()
        self.btn_enter_teach = QPushButton("进入示教模式")
        self.btn_enter_teach.setStyleSheet(
            "background:#2196F3; color:white; padding:8px; font-size:13px;")
        self.btn_enter_teach.clicked.connect(self._enter_teach)

        self.btn_exit_teach = QPushButton("退出示教模式")
        self.btn_exit_teach.setStyleSheet(
            "background:#FF9800; color:white; padding:8px; font-size:13px;")
        self.btn_exit_teach.clicked.connect(self._exit_teach)
        self.btn_exit_teach.setEnabled(False)

        teach_btn_row.addWidget(self.btn_enter_teach)
        teach_btn_row.addWidget(self.btn_exit_teach)
        tl.addLayout(teach_btn_row)
        root.addWidget(teach_group)

        # ── 3. 轨迹记录 ───────────────────────────────────────────────────────
        rec_group = QGroupBox("轨迹记录")
        rl = QVBoxLayout(rec_group)

        # 参数行
        param_row = QHBoxLayout()
        param_row.addWidget(QLabel("采样间隔:"))
        self.spin_interval = QDoubleSpinBox()
        self.spin_interval.setRange(0.05, 0.5)
        self.spin_interval.setValue(0.1)
        self.spin_interval.setSingleStep(0.05)
        self.spin_interval.setSuffix(" s")
        self.spin_interval.setFixedWidth(80)
        param_row.addWidget(self.spin_interval)

        param_row.addSpacing(16)
        self.chk_smooth = QCheckBox("自动平滑")
        self.chk_smooth.setChecked(True)
        param_row.addWidget(self.chk_smooth)

        param_row.addWidget(QLabel("强度:"))
        self.sld_smooth = QSlider(Qt.Orientation.Horizontal)
        self.sld_smooth.setRange(1, 10)
        self.sld_smooth.setValue(5)
        self.sld_smooth.setFixedWidth(100)
        self.lbl_smooth_val = QLabel("5")
        self.sld_smooth.valueChanged.connect(
            lambda v: self.lbl_smooth_val.setText(str(v))
        )
        param_row.addWidget(self.sld_smooth)
        param_row.addWidget(self.lbl_smooth_val)
        param_row.addStretch()
        rl.addLayout(param_row)

        # 按钮行
        rec_btn_row = QHBoxLayout()
        self.btn_start_rec = QPushButton("开始记录")
        self.btn_start_rec.setStyleSheet(
            "background:#4CAF50; color:white; padding:7px; font-size:12px;")
        self.btn_start_rec.clicked.connect(self._start_recording)

        self.btn_stop_rec = QPushButton("停止记录")
        self.btn_stop_rec.setStyleSheet(
            "background:#f44336; color:white; padding:7px; font-size:12px;")
        self.btn_stop_rec.clicked.connect(self._stop_recording)
        self.btn_stop_rec.setEnabled(False)

        rec_btn_row.addWidget(self.btn_start_rec)
        rec_btn_row.addWidget(self.btn_stop_rec)
        rl.addLayout(rec_btn_row)

        self.lbl_record_status = QLabel("就绪")
        self.lbl_record_status.setStyleSheet("color:#666;")
        rl.addWidget(self.lbl_record_status)

        root.addWidget(rec_group)

        # ── 4. 回放 & 保存 ────────────────────────────────────────────────────
        pb_group = QGroupBox("轨迹回放 & 保存")
        pl = QVBoxLayout(pb_group)

        # 速度行
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("速度:"))
        self.sld_speed = QSlider(Qt.Orientation.Horizontal)
        self.sld_speed.setRange(25, 300)
        self.sld_speed.setValue(100)
        self.sld_speed.setFixedWidth(120)
        self.lbl_speed = QLabel("1.0x")
        self.sld_speed.valueChanged.connect(
            lambda v: self.lbl_speed.setText(f"{v/100:.1f}x")
        )
        speed_row.addWidget(self.sld_speed)
        speed_row.addWidget(self.lbl_speed)
        speed_row.addStretch()
        pl.addLayout(speed_row)

        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        pl.addWidget(self.progress_bar)

        # 按钮行 1：回放控制
        pb_btn_row = QHBoxLayout()
        self.btn_play = QPushButton("回放轨迹")
        self.btn_play.setStyleSheet(
            "background:#9C27B0; color:white; padding:7px; font-size:12px;")
        self.btn_play.clicked.connect(self._playback)

        self.btn_stop_play = QPushButton("停止回放")
        self.btn_stop_play.setStyleSheet(
            "background:#f44336; color:white; padding:7px; font-size:12px;")
        self.btn_stop_play.clicked.connect(self._stop_playback)
        self.btn_stop_play.setEnabled(False)

        pb_btn_row.addWidget(self.btn_play)
        pb_btn_row.addWidget(self.btn_stop_play)
        pl.addLayout(pb_btn_row)

        # 按钮行 2：保存/加载
        save_row = QHBoxLayout()
        btn_save_json = QPushButton("保存 JSON")
        btn_save_json.clicked.connect(lambda: self._save('json'))
        btn_save_csv = QPushButton("保存 CSV")
        btn_save_csv.clicked.connect(lambda: self._save('csv'))
        btn_load = QPushButton("加载轨迹")
        btn_load.clicked.connect(self._load)

        for b in (btn_save_json, btn_save_csv, btn_load):
            b.setStyleSheet("padding:5px;")
            save_row.addWidget(b)
        save_row.addStretch()
        pl.addLayout(save_row)

        self.lbl_traj_info = QLabel("无轨迹")
        self.lbl_traj_info.setStyleSheet("color:#666; font-size:10px;")
        pl.addWidget(self.lbl_traj_info)

        root.addWidget(pb_group)

        # ── 5. 日志 ───────────────────────────────────────────────────────────
        log_group = QGroupBox("日志")
        ll = QVBoxLayout(log_group)
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumHeight(110)
        ll.addWidget(self.txt_log)
        root.addWidget(log_group)

    # ── 状态刷新 ─────────────────────────────────────────────────────────────

    def _refresh_status(self):
        if self.robot.connected:
            self.lbl_conn.setText("● 已连接")
            self.lbl_conn.setStyleSheet("color:green; font-weight:bold;")
        else:
            self.lbl_conn.setText("● 未连接")
            self.lbl_conn.setStyleSheet("color:red; font-weight:bold;")

        if self.robot.enabled:
            self.lbl_enable.setText("● 已使能")
            self.lbl_enable.setStyleSheet("color:green; font-weight:bold;")
            self.btn_enable.setText("禁用电机")
        else:
            self.lbl_enable.setText("○ 未使能")
            self.lbl_enable.setStyleSheet("color:gray; font-weight:bold;")
            self.btn_enable.setText("使能电机")

    # ── 使能 / 急停 ──────────────────────────────────────────────────────────

    def _toggle_enable(self):
        if not self.robot.connected:
            QMessageBox.warning(self, "警告", "请先连接机器人！")
            return
        if self.robot.enabled:
            self.robot.disable()
            self._log("电机已禁用")
        else:
            if self.robot.enable():
                self._log("电机已使能")
            else:
                self._log("使能失败")

    def _emergency_stop(self):
        self.robot.emergency_stop()
        self._log("急停！")

    # ── 示教模式 ─────────────────────────────────────────────────────────────

    def _enter_teach(self):
        if not self.robot.connected:
            QMessageBox.warning(self, "警告", "请先连接机器人！")
            return
        if not self.robot.enabled:
            self._log("电机未使能，正在自动使能…")
            if not self.robot.enable():
                QMessageBox.critical(self, "错误", "使能失败，请检查连接！")
                return

        self._log("正在进入示教模式（降低 PID 增益）…")
        if self.robot.enter_teach_mode():
            self.in_teach_mode = True
            self.btn_enter_teach.setEnabled(False)
            self.btn_exit_teach.setEnabled(True)
            self._log("已进入示教模式 — 现在可以轻松拖动机械臂")
        else:
            QMessageBox.critical(self, "错误", "进入示教模式失败！")

    def _exit_teach(self):
        if self.recording:
            self._stop_recording()
        self._log("正在退出示教模式（恢复增益并保位）…")
        if self.robot.exit_teach_mode():
            self.in_teach_mode = False
            self.btn_enter_teach.setEnabled(True)
            self.btn_exit_teach.setEnabled(False)
            self._log("已退出示教模式，恢复位置控制")
        else:
            QMessageBox.critical(self, "错误", "退出示教模式失败！")

    # ── 记录 ─────────────────────────────────────────────────────────────────

    def _start_recording(self):
        if not self.in_teach_mode:
            QMessageBox.warning(self, "警告", "请先进入示教模式！")
            return
        if not self.teach_mode:
            return

        self.teach_mode.sample_interval = self.spin_interval.value()
        self.teach_mode.start_recording()
        self.recording = True
        self.robot.set_runtime_state("recording", True)

        self.btn_start_rec.setEnabled(False)
        self.btn_stop_rec.setEnabled(True)
        self.lbl_record_status.setText("记录中…  0 点")
        self._log(f"开始记录（采样间隔 {self.spin_interval.value():.2f}s）")

    def _stop_recording(self):
        if not self.teach_mode:
            return
        self.recording = False
        self.robot.set_runtime_state("recording", False)
        raw = self.teach_mode.stop_recording()
        self._log(f"停止记录，原始点数: {len(raw)}")

        if self.chk_smooth.isChecked() and len(raw) >= 4:
            strength = self.sld_smooth.value()
            self.current_trajectory = self.teach_mode.smooth_trajectory_adaptive(
                raw, strength=strength
            )
            self._log(f"自适应平滑完成（强度 {strength}）: "
                      f"{len(raw)} → {len(self.current_trajectory)} 点")
        else:
            self.current_trajectory = raw

        self.btn_start_rec.setEnabled(True)
        self.btn_stop_rec.setEnabled(False)
        self._update_traj_info()

    def _on_record_point(self, point):
        """后台线程回调 → 通过 signal 安全转主线程"""
        self.record_count_signal.emit(len(self.teach_mode.trajectory))

    # ── 回放 ─────────────────────────────────────────────────────────────────

    def _playback(self):
        if not self.current_trajectory:
            QMessageBox.warning(self, "警告", "没有轨迹数据！请先记录或加载轨迹。")
            return
        if not self.robot.connected:
            QMessageBox.warning(self, "警告", "请先连接机器人！")
            return
        if not self.robot.enabled:
            reply = QMessageBox.question(
                self, "电机未使能",
                "电机当前未使能，是否立即使能后开始回放？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply == QMessageBox.StandardButton.No:
                return
            if not self.robot.enable():
                QMessageBox.critical(self, "错误", "使能失败！")
                return

        speed = self.sld_speed.value() / 100.0
        self.progress_bar.setValue(0)
        self.btn_play.setEnabled(False)
        self.btn_stop_play.setEnabled(True)

        self.playback_thread = PlaybackThread(
            self.robot, self.current_trajectory, speed
        )
        self.playback_thread.progress_signal.connect(self.progress_bar.setValue)
        self.playback_thread.finished_signal.connect(self._on_playback_done)
        self.playback_thread.log_signal.connect(self._log)
        self.robot.set_runtime_state("playback", True)
        self.playback_thread.start()

    def _stop_playback(self):
        if self.playback_thread:
            self.playback_thread.stop()
            self.playback_thread.wait()
        self._on_playback_done()
        self._log("回放已停止")

    def _on_playback_done(self):
        self.robot.set_runtime_state("playback", False)
        self.btn_play.setEnabled(True)
        self.btn_stop_play.setEnabled(False)
        self._log("回放完成")

    # ── 保存 / 加载 ──────────────────────────────────────────────────────────

    def _save(self, fmt: str):
        if not self.current_trajectory:
            QMessageBox.warning(self, "警告", "没有轨迹数据可保存！")
            return
        ext = '.json' if fmt == 'json' else '.csv'
        filt = f"{'JSON' if fmt=='json' else 'CSV'} files (*{ext})"
        default = os.path.expanduser(
            f"~/Desktop/轨迹_{datetime.now().strftime('%m%d_%H%M')}{ext}"
        )
        path, _ = QFileDialog.getSaveFileName(self, "保存轨迹", default, filt)
        if not path:
            return
        try:
            self.teach_mode.save_trajectory(
                self.current_trajectory, path, format=fmt,
                metadata={'created': datetime.now().isoformat()}
            )
            self._log(f"已保存: {path}")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败:\n{e}")

    def _load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "加载轨迹", os.path.expanduser("~/Desktop"),
            "Trajectory files (*.json *.csv)"
        )
        if not path:
            return
        try:
            traj = self.teach_mode.load_trajectory(path)
            if traj:
                self.current_trajectory = traj
                self._update_traj_info()
                self._log(f"加载完成: {len(traj)} 点")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载失败:\n{e}")

    # ── 工具 ─────────────────────────────────────────────────────────────────

    def _update_traj_info(self):
        traj = self.current_trajectory
        if not traj:
            self.lbl_traj_info.setText("无轨迹")
            return
        dur = traj[-1].timestamp if traj else 0
        self.lbl_traj_info.setText(
            f"{len(traj)} 点 | 时长 {dur:.1f}s | "
            f"点间隔 {dur/(len(traj)-1)*1000:.0f}ms"
            if len(traj) > 1 else f"{len(traj)} 点"
        )
        self.lbl_record_status.setText(f"已记录 {len(traj)} 点")

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt_log.append(f"[{ts}] {msg}")

    def closeEvent(self, event):
        self.status_timer.stop()
        if self.recording:
            self._stop_recording()
        if self.playback_thread and self.playback_thread.isRunning():
            self.playback_thread.stop()
            self.playback_thread.wait()
        if self.in_teach_mode:
            self.robot.exit_teach_mode()
        event.accept()
