"""PyQt6 主窗口"""

import sys
import glob
import time
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QComboBox, QSpinBox,
    QTextEdit, QGroupBox, QMessageBox, QFrame
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QFont, QColor

from core.robot import DummyRobot
from utils.config import JOINT_NAMES, JOINT_LIMITS, POSES


class UpdateThread(QThread):
    """后台更新线程"""
    position_updated = pyqtSignal(list)
    
    def __init__(self, robot: DummyRobot):
        super().__init__()
        self.robot = robot
        self.running = True
        
    def run(self):
        while self.running:
            if self.robot.connected:
                pos = self.robot.get_position()
                if pos:
                    self.position_updated.emit(pos)
            time.sleep(0.5)
    
    def stop(self):
        self.running = False


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dummy V2 机械臂控制器 (PyQt6)")
        self.setMinimumSize(1000, 700)
        
        # 设置字体
        self.setFont(QFont("Noto Sans CJK SC", 10))
        
        self.robot = DummyRobot()
        self.update_thread = None
        
        self._create_ui()
        self._auto_detect_port()
        
        # 启动更新定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_status)
        self.timer.start(500)
    
    def _create_ui(self):
        """创建界面"""
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setSpacing(15)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # === 左侧：关节控制 ===
        left_panel = QFrame()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(10)
        
        # 串口连接
        conn_group = QGroupBox("串口连接")
        conn_layout = QHBoxLayout(conn_group)
        
        conn_layout.addWidget(QLabel("端口:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        conn_layout.addWidget(self.port_combo)
        
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(self.refresh_btn)
        
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        conn_layout.addWidget(self.connect_btn)
        
        self.status_label = QLabel("● 未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        conn_layout.addWidget(self.status_label)
        
        left_layout.addWidget(conn_group)
        
        # 关节控制
        joints_group = QGroupBox("关节控制")
        joints_layout = QVBoxLayout(joints_group)
        
        self.joint_sliders = []
        self.joint_current_labels = []
        self.joint_target_labels = []
        
        for i in range(6):
            joint_frame = QFrame()
            joint_layout = QHBoxLayout(joint_frame)
            joint_layout.setContentsMargins(5, 2, 5, 2)
            
            # 关节名称
            name_label = QLabel(JOINT_NAMES[i])
            name_label.setMinimumWidth(80)
            name_label.setStyleSheet("font-weight: bold;")
            joint_layout.addWidget(name_label)
            
            # 当前角度
            current_label = QLabel("当前: --°")
            current_label.setMinimumWidth(70)
            current_label.setStyleSheet("color: #666;")
            joint_layout.addWidget(current_label)
            self.joint_current_labels.append(current_label)
            
            # 滑块
            min_val, max_val = JOINT_LIMITS[i]
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setValue([0, -75, 180, 0, 0, 0][i])
            slider.setMinimumWidth(200)
            joint_layout.addWidget(slider)
            self.joint_sliders.append(slider)
            
            # 目标值
            target_label = QLabel(f"{[0, -75, 180, 0, 0, 0][i]}°")
            target_label.setMinimumWidth(50)
            target_label.setStyleSheet("color: #2196F3; font-weight: bold;")
            joint_layout.addWidget(target_label)
            self.joint_target_labels.append(target_label)
            
            # 归零按钮
            zero_btn = QPushButton("0")
            zero_btn.setMaximumWidth(30)
            zero_btn.clicked.connect(lambda checked, idx=i: self._set_joint_zero(idx))
            joint_layout.addWidget(zero_btn)
            
            # 滑块值变化
            slider.valueChanged.connect(lambda val, idx=i: self._on_slider_change(idx, val))
            
            joints_layout.addWidget(joint_frame)
        
        left_layout.addWidget(joints_group)
        
        # 执行按钮
        self.move_btn = QPushButton("▶ 执行运动")
        self.move_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-size: 14px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """)
        self.move_btn.clicked.connect(self._execute_move)
        left_layout.addWidget(self.move_btn)
        
        layout.addWidget(left_panel, stretch=2)
        
        # === 右侧：控制和日志 ===
        right_panel = QFrame()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(10)
        
        # 电机控制
        motor_group = QGroupBox("电机控制")
        motor_layout = QHBoxLayout(motor_group)
        
        self.enable_btn = QPushButton("🔴 使能电机")
        self.enable_btn.clicked.connect(self._toggle_enable)
        self.enable_btn.setStyleSheet("padding: 8px;")
        motor_layout.addWidget(self.enable_btn)
        
        stop_btn = QPushButton("⏹ 急停")
        stop_btn.setStyleSheet("background-color: #F44336; color: white; padding: 8px;")
        stop_btn.clicked.connect(self._emergency_stop)
        motor_layout.addWidget(stop_btn)
        
        disable_btn = QPushButton("⚪ 禁用")
        disable_btn.clicked.connect(self._disable)
        motor_layout.addWidget(disable_btn)
        
        right_layout.addWidget(motor_group)
        
        # 示教功能
        teach_group = QGroupBox("🎓 示教功能")
        teach_layout = QVBoxLayout(teach_group)
        
        teach_btn = QPushButton("🎓 打开示教面板")
        teach_btn.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                font-size: 13px;
                padding: 12px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
        """)
        teach_btn.clicked.connect(self._open_teach_dialog)
        teach_layout.addWidget(teach_btn)
        
        # 示教功能说明
        teach_info = QLabel("• 拖动示教\n• 轨迹记录\n• 平滑回放\n• 保存训练数据")
        teach_info.setStyleSheet("color: #666; font-size: 10px;")
        teach_layout.addWidget(teach_info)
        
        right_layout.addWidget(teach_group)
        
        # 预设位置
        poses_group = QGroupBox("预设位置")
        poses_layout = QVBoxLayout(poses_group)
        
        for name, angles in POSES.items():
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, a=angles, n=name: self._load_pose(a, n))
            btn.setStyleSheet("padding: 8px; text-align: left;")
            poses_layout.addWidget(btn)
        
        right_layout.addWidget(poses_group)
        
        # 速度设置
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("运动速度:"))
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 100)
        self.speed_spin.setValue(30)
        speed_layout.addWidget(self.speed_spin)
        speed_layout.addStretch()
        right_layout.addLayout(speed_layout)
        
        # 日志
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(200)
        log_layout.addWidget(self.log_text)
        
        right_layout.addWidget(log_group, stretch=1)
        
        layout.addWidget(right_panel, stretch=1)
        
        # 初始日志
        self._log("Dummy V2 控制器启动 (PyQt6)")
        self._log("请连接串口并使能电机")
    
    def _log(self, message: str):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
    
    def _auto_detect_port(self):
        """自动检测串口"""
        ports = glob.glob('/dev/ttyACM*')
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if ports:
            self._log(f"检测到串口: {ports}")
    
    def _refresh_ports(self):
        """刷新串口列表"""
        self._auto_detect_port()
    
    def _toggle_connection(self):
        """切换连接"""
        if self.robot.connected:
            self._disconnect()
        else:
            self._connect()
    
    def _connect(self):
        """连接"""
        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "错误", "请选择串口")
            return
        
        if self.robot.connect(port):
            self.connect_btn.setText("断开")
            self.connect_btn.setStyleSheet("background-color: #F44336; color: white;")
            self.status_label.setText("● 已连接")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self._log(f"已连接: {port}")
            
            # 启动更新线程
            self.update_thread = UpdateThread(self.robot)
            self.update_thread.position_updated.connect(self._update_position_display)
            self.update_thread.start()
        else:
            QMessageBox.critical(self, "错误", "连接失败")
    
    def _disconnect(self):
        """断开"""
        if self.update_thread:
            self.update_thread.stop()
            self.update_thread.wait()
        
        self.robot.disconnect()
        self.connect_btn.setText("连接")
        self.connect_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        self.status_label.setText("● 未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self._log("已断开")
    
    def _toggle_enable(self):
        """切换使能"""
        if not self.robot.connected:
            return
        
        if self.robot.enabled:
            self.robot.disable()
            self.enable_btn.setText("🔴 使能电机")
            self._log("电机已禁用")
        else:
            if self.robot.enable():
                self.enable_btn.setText("🟢 禁用电机")
                self._log("电机已使能")
    
    def _disable(self):
        """禁用"""
        self.robot.disable()
        self.enable_btn.setText("🔴 使能电机")
        self._log("电机已禁用")
    
    def _emergency_stop(self):
        """急停"""
        self.robot.emergency_stop()
        self._log("🛑 急停！")
    
    def _on_slider_change(self, joint_idx: int, value: int):
        """滑块变化"""
        self.joint_target_labels[joint_idx].setText(f"{value}°")
    
    def _set_joint_zero(self, joint_idx: int):
        """设置关节为零"""
        self.joint_sliders[joint_idx].setValue(0)
    
    def _load_pose(self, angles: list, name: str):
        """加载预设位置"""
        for i, angle in enumerate(angles):
            self.joint_sliders[i].setValue(angle)
        self._log(f"已加载 {name}")
        
        reply = QMessageBox.question(
            self, "确认运动",
            f"是否立即运动到 {name}?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        if reply == QMessageBox.StandardButton.Yes:
            self._execute_move()
    
    def _execute_move(self):
        """执行运动"""
        if not self.robot.connected:
            QMessageBox.warning(self, "错误", "请先连接串口")
            return
        
        if not self.robot.enabled:
            reply = QMessageBox.question(
                self, "确认",
                "电机未使能，是否自动使能?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply == QMessageBox.StandardButton.Yes:
                self.robot.enable()
                self.enable_btn.setText("🟢 禁用电机")
            else:
                return
        
        # 获取目标角度
        target = [slider.value() for slider in self.joint_sliders]
        speed = self.speed_spin.value()
        
        if self.robot.move_to(target, speed):
            self._log(f"运动到: {target}, 速度: {speed}")
    
    def _update_position_display(self, angles: list):
        """更新位置显示"""
        for i, angle in enumerate(angles):
            self.joint_current_labels[i].setText(f"当前: {angle:.1f}°")
    
    def _update_status(self):
        """更新状态"""
        if self.robot.connected:
            if self.robot.enabled:
                self.status_label.setText("● 已连接 | 已使能")
                self.status_label.setStyleSheet("color: green; font-weight: bold;")
            else:
                self.status_label.setText("● 已连接 | 未使能")
                self.status_label.setStyleSheet("color: orange; font-weight: bold;")
    
    def _open_teach_dialog(self):
        """打开示教对话框"""
        from gui.teach_dialog import TeachDialog
        
        if not self.robot.connected:
            QMessageBox.warning(self, "警告", "请先连接机器人！")
            return
        
        dialog = TeachDialog(self.robot, self)
        dialog.exec()
    
    def closeEvent(self, event):
        """关闭事件"""
        if self.update_thread:
            self.update_thread.stop()
            self.update_thread.wait()
        self.robot.disconnect()
        event.accept()
