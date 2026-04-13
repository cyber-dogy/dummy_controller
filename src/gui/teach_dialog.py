#!/usr/bin/env python3
"""
示教功能对话框
集成电流环示教、轨迹记录、平滑、保存
"""

import json
import os
from datetime import datetime
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QProgressBar, QTextEdit, QFileDialog, QMessageBox,
    QSpinBox, QDoubleSpinBox, QComboBox, QCheckBox, QSlider
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QFont
import numpy as np


class PlaybackThread(QThread):
    """轨迹回放后台线程"""
    progress_signal = pyqtSignal(int)
    finished_signal = pyqtSignal()
    log_signal = pyqtSignal(str)
    
    def __init__(self, robot, trajectory, speed_ratio):
        super().__init__()
        self.robot = robot
        self.trajectory = trajectory
        self.speed_ratio = speed_ratio
        self.stopped = False
        self.violations_count = 0
    
    def run(self):
        import time
        from utils.config import JointLimitChecker
        
        # CAN总线保护参数
        MIN_TX_INTERVAL = 0.03  # 最小发送间隔 30ms（约33Hz）
        
        # 先验证并限制所有轨迹点
        valid_trajectory = []
        for point in self.trajectory:
            clamped = JointLimitChecker.clamp_angles(point.angles)
            if not JointLimitChecker.is_valid(point.angles):
                self.violations_count += 1
            valid_trajectory.append(type(point)(timestamp=point.timestamp, angles=clamped))
        
        if self.violations_count > 0:
            self.log_signal.emit(f"[警告] 轨迹中有 {self.violations_count} 个点超出限位，已自动限制")
        
        # 检查轨迹频率，如果过高则稀疏化
        if len(valid_trajectory) > 2:
            duration = valid_trajectory[-1].timestamp - valid_trajectory[0].timestamp
            if duration > 0:
                rate = len(valid_trajectory) / duration
                if rate > 25:  # 如果频率超过25Hz，需要稀疏化
                    self.log_signal.emit(f"[CAN保护] 轨迹频率 {rate:.1f}Hz 过高，进行稀疏化")
                    step = max(1, int(rate / 20))  # 降到约20Hz
                    valid_trajectory = valid_trajectory[::step]
                    self.log_signal.emit(f"[CAN保护] 稀疏化后: {len(valid_trajectory)}点")
        
        # 获取当前位置，添加平滑过渡
        current_pos = self.robot.get_position()
        if current_pos:
            current_pos = JointLimitChecker.clamp_angles(current_pos)
            first_point = valid_trajectory[0]
            max_diff = max(abs(a - b) for a, b in zip(first_point.angles, current_pos))
            
            if max_diff > 10:  # 如果差距大于10度，插入过渡点
                self.log_signal.emit(f"[信息] 当前位置与轨迹起点差距 {max_diff:.1f}°，插入平滑过渡")
                transition = JointLimitChecker.interpolate_to_limit(
                    current_pos, first_point.angles, steps=5
                )
                for t_angles in transition[:-1]:
                    if self.stopped:
                        return
                    self.robot.move_to(t_angles, check_limits=False)
                    time.sleep(0.05)
        
        # 执行轨迹 - 带CAN总线保护
        last_tx_time = 0
        tx_count = 0
        skip_count = 0
        error_count = 0
        
        self.log_signal.emit(f"[CAN回放] 开始回放 {len(valid_trajectory)} 点")
        
        for idx, point in enumerate(valid_trajectory):
            if self.stopped:
                break
            
            current_time = time.time()
            
            # CAN总线限流保护
            if current_time - last_tx_time < MIN_TX_INTERVAL:
                skip_count += 1
                continue
            
            # 发送运动指令
            success, msg = self.robot.move_to(point.angles, check_limits=False)
            
            if success:
                tx_count += 1
                last_tx_time = current_time
            else:
                error_count += 1
                if error_count <= 3:  # 只显示前3个错误
                    self.log_signal.emit(f"[警告] 第 {idx} 点发送失败: {msg}")
            
            # 计算等待时间
            if idx < len(valid_trajectory) - 1:
                next_time = valid_trajectory[idx + 1].timestamp
                point_time = point.timestamp
                wait_time = (next_time - point_time) / self.speed_ratio
                # 确保最小等待时间
                time.sleep(max(MIN_TX_INTERVAL, wait_time))
            
            progress = int((idx + 1) / len(valid_trajectory) * 100)
            self.progress_signal.emit(progress)
        
        # 回放统计
        if skip_count > 0:
            self.log_signal.emit(f"[CAN回放] 完成: 发送 {tx_count} 点, 限流跳过 {skip_count} 点")
        else:
            self.log_signal.emit(f"[CAN回放] 完成: 发送 {tx_count} 点")
        
        if error_count > 0:
            self.log_signal.emit(f"[CAN回放] 警告: {error_count} 次发送失败")
        
        self.finished_signal.emit()
    
    def stop(self):
        self.stopped = True


class TeachDialog(QDialog):
    """示教功能对话框"""
    
    def __init__(self, robot, parent=None):
        super().__init__(parent)
        self.robot = robot
        self.teach_mode = None
        self.recording = False
        self.current_trajectory = []
        self.playback_thread = None
        
        self.setWindowTitle("🎓 示教功能 - 拖动示教 & 轨迹记录")
        self.setMinimumSize(700, 600)
        self.setup_ui()
        
        # 更新定时器
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_live_data)
        
        # 初始化示教模块
        try:
            from core.teach_mode import TeachMode
            self.teach_mode = TeachMode(robot)
            self.teach_mode.on_record_callback = self.on_record_point
        except Exception as e:
            self.log(f"⚠️ 示教模块初始化失败: {e}")
    
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # ========== 示教控制区 ==========
        teach_group = QGroupBox("🎮 示教控制")
        teach_layout = QVBoxLayout(teach_group)
        
        # 电流限制设置（电流环模式）
        current_group = QGroupBox("⚡ 电流环设置")
        current_group_layout = QVBoxLayout(current_group)
        
        # 说明文字
        current_info = QLabel(
            "电流环模式：电机输出恒定电流，产生可预测的阻力\n"
            "• 小电流(0.2-0.4A)：轻松拖动，适合快速示教\n"
            "• 中电流(0.5-0.8A)：适度阻力，适合精确示教\n"
            "• 大电流(1.0A+)：较重阻力，接近正常工作状态"
        )
        current_info.setStyleSheet("color: #666; font-size: 10px;")
        current_group_layout.addWidget(current_info)
        
        # 电流值设置
        current_layout = QHBoxLayout()
        current_layout.addWidget(QLabel("电流值:"))
        
        self.current_slider = QSlider(Qt.Orientation.Horizontal)
        self.current_slider.setRange(1, 20)  # 0.1A ~ 2.0A
        self.current_slider.setValue(5)  # 默认 0.5A
        self.current_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        current_layout.addWidget(self.current_slider)
        
        self.current_spin = QDoubleSpinBox()
        self.current_spin.setRange(0.1, 2.0)
        self.current_spin.setValue(0.5)
        self.current_spin.setSingleStep(0.1)
        self.current_spin.setSuffix(" A")
        current_layout.addWidget(self.current_spin)
        
        # 联动滑动条和数字框
        self.current_slider.valueChanged.connect(
            lambda v: self.current_spin.setValue(v / 10.0)
        )
        self.current_spin.valueChanged.connect(
            lambda v: self.current_slider.setValue(int(v * 10))
        )
        
        current_group_layout.addLayout(current_layout)
        teach_layout.addWidget(current_group)
        
        # 示教按钮
        btn_layout = QHBoxLayout()
        self.btn_enter_teach = QPushButton("🔓 进入示教模式")
        self.btn_enter_teach.setStyleSheet("background: #2196F3; color: white; padding: 10px;")
        self.btn_enter_teach.clicked.connect(self.enter_teach_mode)
        
        self.btn_exit_teach = QPushButton("🔒 退出示教模式")
        self.btn_exit_teach.setStyleSheet("background: #FF9800; color: white; padding: 10px;")
        self.btn_exit_teach.clicked.connect(self.exit_teach_mode)
        self.btn_exit_teach.setEnabled(False)
        
        btn_layout.addWidget(self.btn_enter_teach)
        btn_layout.addWidget(self.btn_exit_teach)
        teach_layout.addLayout(btn_layout)
        
        # 关节限位信息
        limits_group = QGroupBox("⚠️ 关节限位信息")
        limits_layout = QVBoxLayout(limits_group)
        
        from utils.config import JOINT_NAMES, JOINT_LIMITS
        limits_text = ""
        for i, (name, (min_val, max_val)) in enumerate(zip(JOINT_NAMES, JOINT_LIMITS)):
            limits_text += f"{name}: [{min_val}°, {max_val}°]"
            if i < 5:
                limits_text += " | "
            if i == 2:  # 每3个换行
                limits_text += "\n"
        
        limits_label = QLabel(limits_text)
        limits_label.setStyleSheet("color: #666; font-size: 9px; font-family: monospace;")
        limits_layout.addWidget(limits_label)
        
        # 特别警告J3的限制
        j3_warning = QLabel("⚠️ 特别注意: J3 肘部最小 35° (固件硬编码限制)")
        j3_warning.setStyleSheet("color: #FF5722; font-size: 10px; font-weight: bold;")
        limits_layout.addWidget(j3_warning)
        
        teach_layout.addWidget(limits_group)
        
        layout.addWidget(teach_group)
        
        # ========== 轨迹记录区 ==========
        record_group = QGroupBox("📝 轨迹记录")
        record_layout = QVBoxLayout(record_group)
        
        # 采样间隔
        sample_layout = QHBoxLayout()
        sample_layout.addWidget(QLabel("采样间隔:"))
        self.sample_spin = QDoubleSpinBox()
        self.sample_spin.setRange(0.02, 0.5)
        self.sample_spin.setValue(0.05)
        self.sample_spin.setSingleStep(0.01)
        self.sample_spin.setSuffix(" s")
        sample_layout.addWidget(self.sample_spin)
        
        sample_layout.addWidget(QLabel("平滑窗口:"))
        self.smooth_spin = QSpinBox()
        self.smooth_spin.setRange(3, 21)
        self.smooth_spin.setValue(5)
        self.smooth_spin.setSingleStep(2)
        sample_layout.addWidget(self.smooth_spin)
        
        self.chk_auto_smooth = QCheckBox("自动平滑")
        self.chk_auto_smooth.setChecked(True)
        sample_layout.addWidget(self.chk_auto_smooth)
        
        sample_layout.addStretch()
        record_layout.addLayout(sample_layout)
        
        # 记录按钮
        record_btn_layout = QHBoxLayout()
        self.btn_start_record = QPushButton("⏺️ 开始记录")
        self.btn_start_record.setStyleSheet("background: #4CAF50; color: white; padding: 10px;")
        self.btn_start_record.clicked.connect(self.start_recording)
        
        self.btn_stop_record = QPushButton("⏹️ 停止记录")
        self.btn_stop_record.setStyleSheet("background: #f44336; color: white; padding: 10px;")
        self.btn_stop_record.clicked.connect(self.stop_recording)
        self.btn_stop_record.setEnabled(False)
        
        record_btn_layout.addWidget(self.btn_start_record)
        record_btn_layout.addWidget(self.btn_stop_record)
        record_layout.addLayout(record_btn_layout)
        
        # 记录状态
        self.lbl_record_status = QLabel("状态: 未记录 | 点数: 0")
        record_layout.addWidget(self.lbl_record_status)
        
        layout.addWidget(record_group)
        
        # ========== 回放控制区 ==========
        playback_group = QGroupBox("▶️ 轨迹回放")
        playback_layout = QVBoxLayout(playback_group)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("回放速度:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(25, 400)
        self.speed_slider.setValue(100)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        speed_layout.addWidget(self.speed_slider)
        self.lbl_speed = QLabel("1.0x")
        self.speed_slider.valueChanged.connect(lambda v: self.lbl_speed.setText(f"{v/100:.1f}x"))
        speed_layout.addWidget(self.lbl_speed)
        playback_layout.addLayout(speed_layout)
        
        # 回放进度
        self.progress_bar = QProgressBar()
        self.progress_bar.setTextVisible(True)
        playback_layout.addWidget(self.progress_bar)
        
        # 回放按钮
        playback_btn_layout = QHBoxLayout()
        self.btn_playback = QPushButton("▶️ 回放轨迹")
        self.btn_playback.setStyleSheet("background: #9C27B0; color: white; padding: 10px;")
        self.btn_playback.clicked.connect(self.playback_trajectory)
        
        self.btn_stop_playback = QPushButton("⏹️ 停止回放")
        self.btn_stop_playback.setStyleSheet("background: #f44336; color: white; padding: 10px;")
        self.btn_stop_playback.clicked.connect(self.stop_playback)
        self.btn_stop_playback.setEnabled(False)
        
        playback_btn_layout.addWidget(self.btn_playback)
        playback_btn_layout.addWidget(self.btn_stop_playback)
        playback_layout.addLayout(playback_btn_layout)
        
        layout.addWidget(playback_group)
        
        # ========== 保存与导出 ==========
        save_group = QGroupBox("💾 保存与导出")
        save_layout = QVBoxLayout(save_group)
        
        # 文件名设置
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("轨迹名称:"))
        self.txt_traj_name = QTextEdit()
        self.txt_traj_name.setMaximumHeight(30)
        self.txt_traj_name.setPlaceholderText(f"轨迹_{datetime.now().strftime('%m%d_%H%M')}")
        name_layout.addWidget(self.txt_traj_name)
        save_layout.addLayout(name_layout)
        
        # 格式选择
        format_layout = QHBoxLayout()
        format_layout.addWidget(QLabel("保存格式:"))
        self.cmb_format = QComboBox()
        self.cmb_format.addItems(["JSON (完整数据)", "CSV (ML训练格式)"])
        format_layout.addWidget(self.cmb_format)
        format_layout.addStretch()
        save_layout.addLayout(format_layout)
        
        # 保存按钮
        save_btn_layout = QHBoxLayout()
        self.btn_save = QPushButton("💾 保存轨迹")
        self.btn_save.setStyleSheet("background: #009688; color: white; padding: 8px;")
        self.btn_save.clicked.connect(self.save_trajectory)
        
        self.btn_load = QPushButton("📂 加载轨迹")
        self.btn_load.setStyleSheet("background: #607D8B; color: white; padding: 8px;")
        self.btn_load.clicked.connect(self.load_trajectory)
        
        save_btn_layout.addWidget(self.btn_save)
        save_btn_layout.addWidget(self.btn_load)
        save_layout.addLayout(save_btn_layout)
        
        layout.addWidget(save_group)
        
        # ========== 日志区 ==========
        log_group = QGroupBox("📋 操作日志")
        log_layout = QVBoxLayout(log_group)
        
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumHeight(120)
        log_layout.addWidget(self.txt_log)
        
        layout.addWidget(log_group)
    
    def enter_teach_mode(self):
        """进入示教模式（电流环模式）"""
        if not self.robot.connected:
            QMessageBox.warning(self, "警告", "请先连接机器人！")
            return
        
        if not self.robot.enabled:
            QMessageBox.warning(self, "警告", "请先使能机器人！")
            return
        
        current = self.current_spin.value()
        
        self.log(f"🔄 正在进入示教模式（电流限制: {current}A）...")
        
        # 使用新的电流环示教模式
        if self.robot.enter_teach_mode(current):
            self.btn_enter_teach.setEnabled(False)
            self.btn_exit_teach.setEnabled(True)
            
            self.log(f"🔓 已进入示教模式（电流: {current}A）")
            self.log("💡 现在你可以手动拖动机械臂了")
            self.log("   提示：如果感觉太重，可以减小电流值")
            
            # 启动实时更新
            self.update_timer.start(100)
        else:
            QMessageBox.critical(self, "错误", "进入示教模式失败！")
    
    def exit_teach_mode(self):
        """
        退出示教模式，恢复位置控制
        如果当前位置超出限位，会先平滑回到边界
        """
        if self.recording:
            self.stop_recording()
        
        self.log("🔄 正在退出示教模式...")
        
        # 读取当前位置
        current_pos = self.robot.get_position()
        if not current_pos:
            QMessageBox.warning(self, "警告", "无法读取当前位置，直接退出示教模式")
            self._finish_exit_teach_mode()
            return
        
        # 检查是否超出限位
        from utils.config import JointLimitChecker, JOINT_NAMES
        is_valid = JointLimitChecker.is_valid(current_pos)
        
        if not is_valid:
            # 获取违规信息
            violations = JointLimitChecker.get_violations(current_pos)
            self.log("⚠️ 当前位置超出关节限位:")
            for joint_idx, angle, min_val, max_val in violations:
                self.log(f"   {JOINT_NAMES[joint_idx]}: {angle:.1f}° 超出 [{min_val}, {max_val}]")
            
            # 询问用户是否平滑回到合法位置
            reply = QMessageBox.question(
                self, "位置超限",
                f"当前位置有 {len(violations)} 个关节超出限位。\n"
                f"是否平滑移动回合法位置后再退出？\n\n"
                f"选择"是"：平滑过渡回边界点（推荐）\n"
                f"选择"否"：直接退出（可能有抖动）",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No | QMessageBox.StandardButton.Cancel
            )
            
            if reply == QMessageBox.StandardButton.Cancel:
                self.log("❌ 取消退出示教模式")
                return
            
            if reply == QMessageBox.StandardButton.Yes:
                # 先退出电流环模式，但保持电机使能
                self.log("🔄 正在平滑回到合法位置...")
                
                # 计算合法的目标位置（最近的边界点）
                target_pos = JointLimitChecker.clamp_angles(current_pos)
                
                # 如果偏差较大，插入过渡点
                max_diff = max(abs(c - t) for c, t in zip(current_pos, target_pos))
                
                if max_diff > 5:  # 如果偏差大于5度，平滑过渡
                    self.log(f"   当前位置与边界点偏差 {max_diff:.1f}°，执行平滑过渡")
                    
                    # 生成过渡轨迹
                    transition = JointLimitChecker.interpolate_to_limit(
                        current_pos, target_pos, steps=10
                    )
                    
                    # 需要先使能电机才能运动
                    self.robot.enable()
                    
                    # 执行过渡运动（慢速）
                    for i, angles in enumerate(transition):
                        self.robot.move_to(angles, speed=10)  # 低速运动
                        import time
                        time.sleep(0.1)  # 等待运动完成
                        
                        if i % 3 == 0:  # 每3个点更新一次进度
                            self.log(f"   过渡进度: {i+1}/{len(transition)}")
                else:
                    # 偏差小，直接运动到边界点
                    self.robot.enable()
                    self.robot.move_to(target_pos, speed=10)
                    import time
                    time.sleep(0.5)
                
                self.log(f"✅ 已回到合法位置: {target_pos}")
        
        # 完成退出
        self._finish_exit_teach_mode()
    
    def _finish_exit_teach_mode(self):
        """完成退出示教模式的后续操作"""
        if self.robot.exit_teach_mode():
            self.btn_enter_teach.setEnabled(True)
            self.btn_exit_teach.setEnabled(False)
            self.update_timer.stop()
            self.log("🔒 已退出示教模式，恢复位置控制")
        else:
            QMessageBox.critical(self, "错误", "退出示教模式失败！")
    
    def start_recording(self):
        """开始记录"""
        if not self.btn_exit_teach.isEnabled():
            QMessageBox.warning(self, "警告", "请先进入示教模式！")
            return
        
        self.current_trajectory = []
        self.recording = True
        
        # 设置采样间隔
        self.teach_mode.sample_interval = self.sample_spin.value()
        self.teach_mode.start_recording()
        
        self.btn_start_record.setEnabled(False)
        self.btn_stop_record.setEnabled(True)
        
        self.log(f"⏺️ 开始记录轨迹（采样间隔: {self.sample_spin.value()}s）")
    
    def stop_recording(self):
        """停止记录"""
        self.recording = False
        
        raw_trajectory = self.teach_mode.stop_recording()
        self.log(f"🛑 停止记录，原始点数: {len(raw_trajectory)}")
        
        # 自动平滑（包含边界限制）
        if self.chk_auto_smooth.isChecked() and len(raw_trajectory) > 5:
            window = self.smooth_spin.value()
            self.current_trajectory = self.teach_mode.smooth_trajectory(raw_trajectory, window)
            self.log(f"✅ 平滑处理完成（窗口: {window}）")
        else:
            self.current_trajectory = raw_trajectory
        
        # 统一采样间隔（包含边界限制）
        self.current_trajectory = self.teach_mode.interpolate_trajectory(
            self.current_trajectory, 
            self.sample_spin.value()
        )
        
        # 最终边界验证
        from utils.config import JointLimitChecker
        violations = []
        for idx, point in enumerate(self.current_trajectory):
            if not JointLimitChecker.is_valid(point.angles):
                violations.append(idx)
        
        if violations:
            self.log(f"⚠️ 警告: 轨迹中有 {len(violations)} 个点超出限位")
            self.log("   已自动限制到合法范围")
        
        self.log(f"📊 最终轨迹: {len(self.current_trajectory)} 点")
        self.log(f"   J1范围: {min(p.angles[0] for p in self.current_trajectory):.1f}° ~ {max(p.angles[0] for p in self.current_trajectory):.1f}°")
        self.log(f"   J2范围: {min(p.angles[1] for p in self.current_trajectory):.1f}° ~ {max(p.angles[1] for p in self.current_trajectory):.1f}°")
        self.log(f"   J3范围: {min(p.angles[2] for p in self.current_trajectory):.1f}° ~ {max(p.angles[2] for p in self.current_trajectory):.1f}°")
        
        self.btn_start_record.setEnabled(True)
        self.btn_stop_record.setEnabled(False)
        self.lbl_record_status.setText(f"状态: 记录完成 | 点数: {len(self.current_trajectory)}")
    
    def on_record_point(self, point):
        """记录点回调（后台线程）"""
        count = len(self.teach_mode.trajectory)
        self.lbl_record_status.setText(f"状态: 记录中 | 点数: {count}")
    
    def update_live_data(self):
        """实时更新当前位置，并检查是否接近边界"""
        if not self.robot.connected:
            return
        
        # 读取当前位置
        angles = self.robot.get_position()
        if not angles:
            return
        
        # 检查是否接近边界（示教模式下）
        if self.btn_exit_teach.isEnabled():  # 在示教模式中
            from utils.config import JointLimitChecker, JOINT_LIMITS, JOINT_NAMES
            
            warnings = []
            for i, angle in enumerate(angles):
                min_val, max_val = JOINT_LIMITS[i]
                # 如果距离边界小于 5 度，给出警告
                if angle < min_val + 5:
                    warnings.append(f"{JOINT_NAMES[i]}接近下限({angle:.1f}°)")
                elif angle > max_val - 5:
                    warnings.append(f"{JOINT_NAMES[i]}接近上限({angle:.1f}°)")
            
            # 如果有警告，每5秒显示一次
            if warnings:
                current_time = self._get_time_ms()
                if not hasattr(self, '_last_warning_time') or \
                   current_time - self._last_warning_time > 5000:
                    self.log(f"⚠️ 边界警告: {', '.join(warnings[:2])}")
                    if len(warnings) > 2:
                        self.log(f"   还有 {len(warnings)-2} 个关节接近限位")
                    self._last_warning_time = current_time
    
    def _get_time_ms(self):
        """获取当前时间（毫秒）"""
        import time
        return int(time.time() * 1000)
    
    def playback_trajectory(self):
        """回放轨迹"""
        if not self.current_trajectory:
            QMessageBox.warning(self, "警告", "请先记录或加载轨迹！")
            return
        
        if not self.robot.connected or not self.robot.enabled:
            QMessageBox.warning(self, "警告", "请先连接并使能机器人！")
            return
        
        # 先验证轨迹
        from utils.config import JointLimitChecker
        violations = []
        for idx, point in enumerate(self.current_trajectory):
            if not JointLimitChecker.is_valid(point.angles):
                violations.append(idx)
        
        if violations:
            reply = QMessageBox.question(
                self, "轨迹验证",
                f"轨迹中有 {len(violations)} 个点超出关节限位，将自动限制到合法范围。\n"
                f"是否继续回放？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply == QMessageBox.StandardButton.No:
                return
        
        speed = self.speed_slider.value() / 100.0
        self.log(f"▶️ 开始回放轨迹（速度: {speed}x，点数: {len(self.current_trajectory)}）")
        
        self.progress_bar.setValue(0)
        self.btn_playback.setEnabled(False)
        self.btn_stop_playback.setEnabled(True)
        
        self.playback_thread = PlaybackThread(self.robot, self.current_trajectory, speed)
        self.playback_thread.progress_signal.connect(self.progress_bar.setValue)
        self.playback_thread.finished_signal.connect(self.on_playback_finished)
        self.playback_thread.log_signal.connect(self.log)
        self.playback_thread.start()
    
    def stop_playback(self):
        """停止回放"""
        if self.playback_thread:
            self.playback_thread.stop()
            self.playback_thread.wait()
        
        self.on_playback_finished()
        self.log("⏹️ 回放已停止")
    
    def on_playback_finished(self):
        """回放完成回调"""
        self.btn_playback.setEnabled(True)
        self.btn_stop_playback.setEnabled(False)
        self.log("✅ 轨迹回放完成")
    
    def save_trajectory(self):
        """保存轨迹"""
        if not self.current_trajectory:
            QMessageBox.warning(self, "警告", "没有轨迹数据可保存！")
            return
        
        # 获取轨迹名称
        traj_name = self.txt_traj_name.toPlainText().strip()
        if not traj_name:
            traj_name = f"轨迹_{datetime.now().strftime('%m%d_%H%M')}"
        
        # 选择格式
        fmt_idx = self.cmb_format.currentIndex()
        if fmt_idx == 0:  # JSON
            file_filter = "JSON files (*.json)"
            default_ext = ".json"
            save_format = 'json'
        else:  # CSV
            file_filter = "CSV files (*.csv)"
            default_ext = ".csv"
            save_format = 'csv'
        
        # 选择保存路径
        default_path = os.path.expanduser(f"~/Desktop/{traj_name}{default_ext}")
        filepath, _ = QFileDialog.getSaveFileName(
            self, "保存轨迹", default_path, file_filter
        )
        
        if not filepath:
            return
        
        try:
            metadata = {
                'name': traj_name,
                'description': f'Dummy V2示教轨迹 - {datetime.now().strftime("%Y-%m-%d %H:%M")}',
                'joint_limits': 'J3 min=35° (firmware limit)',
                'sample_interval': self.sample_spin.value(),
                'smoothed': self.chk_auto_smooth.isChecked()
            }
            
            self.teach_mode.save_trajectory(
                self.current_trajectory,
                filepath,
                format=save_format,
                metadata=metadata
            )
            
            self.log(f"💾 轨迹已保存: {filepath}")
            QMessageBox.information(self, "成功", f"轨迹已保存到:\n{filepath}")
        
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败:\n{e}")
    
    def load_trajectory(self):
        """加载轨迹"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "加载轨迹", os.path.expanduser("~/Desktop"),
            "Trajectory files (*.json *.csv)"
        )
        
        if not filepath:
            return
        
        try:
            if filepath.endswith('.json'):
                self.current_trajectory = self.teach_mode.load_trajectory(filepath)
            else:
                # 从CSV加载
                self.current_trajectory = self.load_csv_trajectory(filepath)
            
            if self.current_trajectory:
                self.log(f"📂 轨迹加载完成: {len(self.current_trajectory)} 点")
                self.lbl_record_status.setText(f"状态: 已加载 | 点数: {len(self.current_trajectory)}")
                QMessageBox.information(self, "成功", f"轨迹加载完成:\n{len(self.current_trajectory)} 个轨迹点")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载失败:\n{e}")
    
    def load_csv_trajectory(self, filepath):
        """从CSV加载轨迹"""
        import csv
        from core.teach_mode import TrajectoryPoint
        
        trajectory = []
        with open(filepath, 'r', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#') or row[0] == 'timestamp':
                    continue
                trajectory.append(TrajectoryPoint(
                    timestamp=float(row[0]),
                    angles=[float(x) for x in row[1:7]]
                ))
        return trajectory
    
    def log(self, msg):
        """添加日志"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.txt_log.append(f"[{timestamp}] {msg}")
    
    def closeEvent(self, event):
        """关闭时清理"""
        if self.recording:
            self.stop_recording()
        if self.playback_thread and self.playback_thread.isRunning():
            self.playback_thread.stop()
            self.playback_thread.wait()
        
        # 如果还在示教模式，退出
        if self.btn_exit_teach.isEnabled():
            self.exit_teach_mode()
        
        event.accept()
