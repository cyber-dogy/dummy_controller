"""
逆解测试对话框
用于 MuJoCo 数值逆解测试
"""

import sys
import os

# 添加 mujoco_sim 到路径
MUJOCO_SIM_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "mujoco_sim")
if MUJOCO_SIM_PATH not in sys.path:
    sys.path.insert(0, MUJOCO_SIM_PATH)

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QDoubleSpinBox, QTextEdit, QMessageBox, QGroupBox
)
from PyQt6.QtCore import Qt


class IKDialog(QDialog):
    """逆解测试对话框"""
    
    def __init__(self, robot=None, parent=None):
        super().__init__(parent)
        self.robot = robot
        self.setWindowTitle("🔧 逆解测试 - MuJoCo IK")
        self.setMinimumSize(500, 400)
        
        # 初始化 MuJoCo 可视化器用于计算
        self.viz = None
        try:
            from src.robot_visualizer import DummyRobotVisualizer
            self.viz = DummyRobotVisualizer()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"MuJoCo 初始化失败:\n{e}")
        
        self._create_ui()
    
    def _create_ui(self):
        layout = QVBoxLayout(self)
        
        # 说明
        info = QLabel(
            "输入目标位置（单位：毫米），MuJoCo 将计算数值逆解。\n"
            "当前使用 L-Pose 作为初始猜测。"
        )
        info.setStyleSheet("color: #666; font-size: 11px;")
        layout.addWidget(info)
        
        # 目标位置输入
        pos_group = QGroupBox("目标位置 (mm)")
        pos_layout = QHBoxLayout(pos_group)
        
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setRange(-500, 500)
        self.x_spin.setValue(200)
        self.x_spin.setDecimals(1)
        pos_layout.addWidget(QLabel("X:"))
        pos_layout.addWidget(self.x_spin)
        
        self.y_spin = QDoubleSpinBox()
        self.y_spin.setRange(-500, 500)
        self.y_spin.setValue(0)
        self.y_spin.setDecimals(1)
        pos_layout.addWidget(QLabel("Y:"))
        pos_layout.addWidget(self.y_spin)
        
        self.z_spin = QDoubleSpinBox()
        self.z_spin.setRange(-100, 600)
        self.z_spin.setValue(300)
        self.z_spin.setDecimals(1)
        pos_layout.addWidget(QLabel("Z:"))
        pos_layout.addWidget(self.z_spin)
        
        layout.addWidget(pos_group)
        
        # 按钮
        btn_layout = QHBoxLayout()
        
        self.calc_btn = QPushButton("🧮 计算逆解")
        self.calc_btn.setStyleSheet("background: #2196F3; color: white; padding: 10px;")
        self.calc_btn.clicked.connect(self.calculate_ik)
        btn_layout.addWidget(self.calc_btn)
        
        self.move_btn = QPushButton("▶️ 运动到该位置")
        self.move_btn.setStyleSheet("background: #4CAF50; color: white; padding: 10px;")
        self.move_btn.clicked.connect(self.move_to_ik_result)
        self.move_btn.setEnabled(False)
        btn_layout.addWidget(self.move_btn)
        
        layout.addLayout(btn_layout)
        
        # 预设目标
        preset_group = QGroupBox("快速目标")
        preset_layout = QHBoxLayout(preset_group)
        
        presets = [
            ("正前方 (200,0,300)", 200, 0, 300),
            ("左侧 (150,150,250)", 150, 150, 250),
            ("右侧 (150,-150,250)", 150, -150, 250),
            ("高处 (100,0,400)", 100, 0, 400),
        ]
        
        for name, x, y, z in presets:
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, xx=x, yy=y, zz=z: self._set_preset(xx, yy, zz))
            preset_layout.addWidget(btn)
        
        layout.addWidget(preset_group)
        
        # 结果显示
        result_group = QGroupBox("计算结果")
        result_layout = QVBoxLayout(result_group)
        
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setPlaceholderText("点击'计算逆解'查看结果...")
        result_layout.addWidget(self.result_text)
        
        layout.addWidget(result_group)
        
        # 当前解
        self.current_solution = None
    
    def _set_preset(self, x, y, z):
        """设置预设值"""
        self.x_spin.setValue(x)
        self.y_spin.setValue(y)
        self.z_spin.setValue(z)
    
    def calculate_ik(self):
        """计算逆解"""
        if self.viz is None:
            QMessageBox.critical(self, "错误", "MuJoCo 未初始化")
            return
        
        # 获取目标位置（米）
        x = self.x_spin.value() / 1000.0
        y = self.y_spin.value() / 1000.0
        z = self.z_spin.value() / 1000.0
        
        self.result_text.append(f"\n目标位置: X={x*1000:.1f} Y={y*1000:.1f} Z={z*1000:.1f} mm")
        self.result_text.append("正在计算逆解...")
        
        # 使用 L-Pose 作为初始猜测
        initial = [0, 0, 90, 0, 0, 0]
        
        try:
            solution = self.viz.inverse_kinematics(
                target_pos=[x, y, z],
                initial_guess=initial
            )
            
            if solution is not None:
                self.current_solution = solution
                self.move_btn.setEnabled(True)
                
                # 验证正解
                self.viz.update_state(solution)
                fk = self.viz.get_end_effector_xyz_rpy()
                
                result = (
                    f"✅ 逆解成功!\n"
                    f"关节角度:\n"
                    f"  J1={solution[0]:.2f}°\n"
                    f"  J2={solution[1]:.2f}°\n"
                    f"  J3={solution[2]:.2f}°\n"
                    f"  J4={solution[3]:.2f}°\n"
                    f"  J5={solution[4]:.2f}°\n"
                    f"  J6={solution[5]:.2f}°\n"
                    f"验证正解: X={fk[0]*1000:.1f} Y={fk[1]*1000:.1f} Z={fk[2]*1000:.1f} mm\n"
                    f"误差: {((fk[0]-x)**2 + (fk[1]-y)**2 + (fk[2]-z)**2)**0.5 * 1000:.2f} mm"
                )
                self.result_text.append(result)
            else:
                self.current_solution = None
                self.move_btn.setEnabled(False)
                self.result_text.append("❌ 逆解失败，无解或超出范围")
                
        except Exception as e:
            self.result_text.append(f"❌ 计算出错: {e}")
            self.move_btn.setEnabled(False)
    
    def move_to_ik_result(self):
        """运动到逆解结果"""
        if self.current_solution is None:
            QMessageBox.warning(self, "警告", "请先计算逆解")
            return
        
        if self.robot is None or not self.robot.connected:
            QMessageBox.warning(self, "警告", "机器人未连接，无法运动")
            return
        
        if not self.robot.enabled:
            reply = QMessageBox.question(
                self, "确认",
                "电机未使能，是否自动使能?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply == QMessageBox.StandardButton.Yes:
                self.robot.enable()
            else:
                return
        
        # 限制到合法范围
        from utils.config import JointLimitChecker
        safe_angles = JointLimitChecker.clamp_angles(self.current_solution)
        
        success, msg = self.robot.move_to(safe_angles, speed=10)
        if success:
            self.result_text.append(f"\n▶️ 已发送运动指令: {safe_angles}")
        else:
            QMessageBox.warning(self, "错误", msg)
    
    def closeEvent(self, event):
        if self.viz:
            self.viz.stop()
        event.accept()


if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    dlg = IKDialog()
    dlg.show()
    sys.exit(app.exec())
