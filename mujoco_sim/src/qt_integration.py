#!/usr/bin/env python3
"""
MuJoCo 与 PyQt6 集成模块
提供实时 3D 可视化组件
"""

import numpy as np
import mujoco
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from typing import Optional, List


class MuJoCoWidget(QWidget):
    """
    PyQt6 MuJoCo 3D 可视化组件
    
    功能：
    - 实时显示机械臂 3D 模型
    - 显示关节角度和末端位置
    - 支持多种视角
    """
    
    # 信号：末端位置更新
    end_effector_updated = pyqtSignal(list)  # [x, y, z, roll, pitch, yaw]
    
    def __init__(self, parent=None, width: int = 640, height: int = 480):
        super().__init__(parent)
        
        self.render_width = width
        self.render_height = height
        
        # MuJoCo 对象
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.renderer: Optional[mujoco.Renderer] = None
        
        # 场景和相机
        self.scene = None
        self.camera = None
        self.options = None
        self.context = None
        
        # 当前状态
        self.joint_angles = [0, -75, 180, 0, 0, 0]  # REST位置
        
        # 初始化
        self._init_ui()
        self._init_mujoco()
        
        # 更新定时器
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_frame)
        self.update_timer.start(33)  # 30 FPS
    
    def _init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 图像显示标签
        self.image_label = QLabel()
        self.image_label.setFixedSize(self.render_width, self.render_height)
        self.image_label.setStyleSheet("background-color: black;")
        layout.addWidget(self.image_label)
        
        # 信息显示
        info_layout = QHBoxLayout()
        
        self.pos_label = QLabel("末端位置: --")
        self.pos_label.setStyleSheet("font-family: monospace; font-size: 10px;")
        info_layout.addWidget(self.pos_label)
        
        self.angle_label = QLabel("关节角度: --")
        self.angle_label.setStyleSheet("font-family: monospace; font-size: 10px;")
        info_layout.addWidget(self.angle_label)
        
        layout.addLayout(info_layout)
    
    def _init_mujoco(self):
        """初始化 MuJoCo"""
        try:
            import os
            model_path = os.path.join(
                os.path.dirname(__file__), 
                "..", "models", "dummy_robot.xml"
            )
            model_path = os.path.abspath(model_path)
            
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            
            # 创建渲染器
            self.renderer = mujoco.Renderer(
                self.model, 
                self.render_height, 
                self.render_width
            )
            
            # 创建场景
            self.scene = mujoco.MjvScene(self.model, maxgeom=1000)
            self.camera = mujoco.MjvCamera()
            self.options = mujoco.MjvOption()
            
            # 设置相机
            self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.camera.trackbodyid = 0
            self.camera.distance = 1.2
            self.camera.azimuth = 135
            self.camera.elevation = -30
            
            print("[MuJoCo Widget] 初始化成功")
            
        except Exception as e:
            print(f"[MuJoCo Widget] 初始化失败: {e}")
            self.image_label.setText(f"MuJoCo 初始化失败\n{e}")
    
    def _update_frame(self):
        """更新一帧"""
        if self.renderer is None or self.data is None:
            return
        
        try:
            # 应用关节角度
            for i, angle in enumerate(np.deg2rad(self.joint_angles)):
                if i < self.model.nq:
                    self.data.qpos[i] = angle
            
            # 前向运动学
            mujoco.mj_forward(self.model, self.data)
            
            # 更新场景
            mujoco.mjv_updateScene(
                self.model, self.data, self.options, 
                None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, 
                self.scene
            )
            
            # 渲染
            self.renderer.update_scene(self.data, self.camera)
            rgb = self.renderer.render()
            
            # 转换为 QPixmap
            # RGB -> BGR (OpenCV 格式) -> RGB (Qt 格式)
            rgb = np.flip(rgb, axis=2)  # BGR -> RGB
            h, w, c = rgb.shape
            
            # 转换为 QImage
            bytes_per_line = c * w
            q_image = QImage(
                rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888
            )
            pixmap = QPixmap.fromImage(q_image)
            
            self.image_label.setPixmap(pixmap)
            
            # 更新信息显示
            self._update_info()
            
        except Exception as e:
            print(f"[MuJoCo Widget] 渲染错误: {e}")
    
    def _update_info(self):
        """更新信息显示"""
        # 获取末端位置
        try:
            site_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
            )
            if site_id >= 0:
                pos = self.data.site_xpos[site_id]
                self.pos_label.setText(
                    f"末端: X={pos[0]*1000:.1f} Y={pos[1]*1000:.1f} Z={pos[2]*1000:.1f} mm"
                )
                
                # 发射信号
                rot = self.data.site_xmat[site_id].reshape(3, 3)
                roll = np.arctan2(rot[2, 1], rot[2, 2])
                pitch = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2 + rot[2, 2]**2))
                yaw = np.arctan2(rot[1, 0], rot[0, 0])
                
                self.end_effector_updated.emit([
                    pos[0]*1000, pos[1]*1000, pos[2]*1000,
                    np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)
                ])
        except:
            pass
        
        # 关节角度
        angles_str = " ".join([f"J{i+1}={a:.1f}°" for i, a in enumerate(self.joint_angles)])
        self.angle_label.setText(angles_str)
    
    def update_joint_angles(self, angles: List[float]):
        """
        更新关节角度
        
        Args:
            angles: 6个关节角度（度）
        """
        if len(angles) >= 6:
            self.joint_angles = angles[:6]
    
    def set_camera_view(self, view_name: str):
        """
        设置相机视角
        
        Args:
            view_name: "front", "side", "top", "iso"
        """
        if self.camera is None:
            return
        
        views = {
            "front": (90, -10),    # 正面
            "side": (0, -10),      # 侧面
            "top": (90, -90),      # 俯视
            "iso": (135, -30),     # 等轴测（默认）
            "back": (-90, -10),    # 背面
        }
        
        if view_name in views:
            azimuth, elevation = views[view_name]
            self.camera.azimuth = azimuth
            self.camera.elevation = elevation
    
    def closeEvent(self, event):
        """关闭事件"""
        self.update_timer.stop()
        if self.renderer:
            self.renderer.close()
        event.accept()


class MujocoDialog(QWidget):
    """
    独立的 MuJoCo 可视化对话框
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.setWindowTitle("MuJoCo 3D 可视化")
        self.setMinimumSize(800, 600)
        
        layout = QVBoxLayout(self)
        
        # MuJoCo 组件
        self.mujoco_widget = MuJoCoWidget(self, width=800, height=480)
        layout.addWidget(self.mujoco_widget)
        
        # 视角选择按钮
        from PyQt6.QtWidgets import QPushButton, QHBoxLayout
        btn_layout = QHBoxLayout()
        
        views = [
            ("正面", "front"),
            ("侧面", "side"),
            ("俯视", "top"),
            ("等轴测", "iso"),
            ("背面", "back"),
        ]
        
        for name, view_id in views:
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, v=view_id: self.mujoco_widget.set_camera_view(v))
            btn_layout.addWidget(btn)
        
        layout.addLayout(btn_layout)
    
    def update_angles(self, angles: List[float]):
        """更新关节角度"""
        self.mujoco_widget.update_joint_angles(angles)


# 测试
if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    
    dialog = MujocoDialog()
    dialog.show()
    
    # 模拟运动
    import time
    from PyQt6.QtCore import QTimer
    
    angles = [0, -75, 180, 0, 0, 0]
    direction = 1
    
    def animate():
        global angles, direction
        
        angles[1] += direction * 0.5
        angles[2] -= direction * 0.5
        
        if angles[1] >= 0 or angles[1] <= -75:
            direction *= -1
        
        dialog.update_angles(angles)
    
    timer = QTimer()
    timer.timeout.connect(animate)
    timer.start(50)
    
    sys.exit(app.exec())
