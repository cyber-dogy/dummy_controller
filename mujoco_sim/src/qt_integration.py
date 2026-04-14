#!/usr/bin/env python3
"""
MuJoCo 与 PyQt6 集成模块
提供实时 3D 可视化组件（基于 URDF dummy2 转换的 MJCF）
"""

import numpy as np
import mujoco
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from typing import Optional, List


# ── 固件角度 → URDF 关节角度映射 ──────────────────────────────────────────────
#
# 固件使用角度制，URDF 使用弧度制，且轴方向约定不同：
#
#   joint1 axis="0 0 -1"  → urdf_j1 = -deg2rad(fw_j1)
#   joint2 axis="1 0 0"   → urdf_j2 = -deg2rad(fw_j2)  [符号相反]
#   joint3 axis="-1 0 0"  → urdf_j3 =  deg2rad(fw_j3 - 90)
#                            固件 J3=90° = URDF 零位（L 形臂）
#                            固件 J3=0°  = URDF -90°（臂竖直向上）
#                            固件 J3=180°= URDF +90°（臂折叠）
#   joint4 axis="0 -1 0"  → urdf_j4 = -deg2rad(fw_j4)
#   joint5 axis="-1 0 0"  → urdf_j5 = -deg2rad(fw_j5)
#   joint6 axis="0 -1 0"  → urdf_j6 = -deg2rad(fw_j6)
#
# 若接入真实机器人后视觉与物理不符，在此调整符号或偏移量即可。
# ────────────────────────────────────────────────────────────────────────────

def firmware_to_urdf(fw_angles_deg: List[float]) -> List[float]:
    """
    将固件角度（度）转换为 URDF MJCF 关节角度（弧度）。

    Args:
        fw_angles_deg: [J1, J2, J3, J4, J5, J6] in degrees (firmware convention)

    Returns:
        [q1..q6] in radians (URDF / MJCF convention)
    """
    j1, j2, j3, j4, j5, j6 = fw_angles_deg[:6]
    r = np.deg2rad
    return [
        -r(j1),        # joint1 axis=-Z：符号取反
        -r(j2),        # joint2 axis=+X：符号取反（固件 J2 与 URDF 方向相反）
         r(j3 - 90),   # joint3 axis=-X：偏移 90°（URDF 零位 = 固件 L 形 90°）
        -r(j4),        # joint4 axis=-Y：符号取反
        -r(j5),        # joint5 axis=-X：符号取反
        -r(j6),        # joint6 axis=-Y：符号取反
    ]


# URDF joint1-6 关节名
_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# 初始显示姿态（固件角度）：REST 折叠态 J2=-75, J3=180
_REST_FW_DEG = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]


class MuJoCoWidget(QWidget):
    """
    PyQt6 MuJoCo 3D 可视化组件

    功能：
    - 实时显示机械臂 3D 模型（基于 dummy2 URDF 网格）
    - 显示关节角度和末端位置
    - 支持多种视角
    """

    # 信号：末端位置更新 [x, y, z, roll, pitch, yaw] (mm / deg)
    end_effector_updated = pyqtSignal(list)

    def __init__(self, parent=None, width: int = 640, height: int = 480):
        super().__init__(parent)

        self.render_width = width
        self.render_height = height

        # MuJoCo 对象
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.renderer: Optional[mujoco.Renderer] = None

        self.scene = None
        self.camera = None

        # 当前固件角度（度）
        self.fw_angles_deg: List[float] = list(_REST_FW_DEG)

        self._init_ui()
        self._init_mujoco()

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_frame)
        self.update_timer.start(33)  # ~30 FPS

    # ── UI ──────────────────────────────────────────────────────────────────

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.image_label = QLabel()
        self.image_label.setFixedSize(self.render_width, self.render_height)
        self.image_label.setStyleSheet("background-color: #1a1a1a;")
        layout.addWidget(self.image_label)

        info_layout = QHBoxLayout()

        self.pos_label = QLabel("末端位置: --")
        self.pos_label.setStyleSheet("font-family: monospace; font-size: 10px;")
        info_layout.addWidget(self.pos_label)

        self.angle_label = QLabel("关节角度: --")
        self.angle_label.setStyleSheet("font-family: monospace; font-size: 10px;")
        info_layout.addWidget(self.angle_label)

        layout.addLayout(info_layout)

    # ── MuJoCo 初始化 ────────────────────────────────────────────────────────

    def _init_mujoco(self):
        try:
            import os
            model_path = os.path.abspath(os.path.join(
                os.path.dirname(__file__),
                "..", "models", "dummy_robot.xml"
            ))

            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)

            self.renderer = mujoco.Renderer(
                self.model,
                self.render_height,
                self.render_width
            )

            self.scene = mujoco.MjvScene(self.model, maxgeom=1000)
            self.camera = mujoco.MjvCamera()

            # 自由相机：从侧前方俯视，Y 为臂伸展方向
            self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.camera.lookat[0] =  0.0
            self.camera.lookat[1] =  0.15  # 偏向 +Y（臂伸出方向）
            self.camera.lookat[2] =  0.25
            self.camera.distance  =  1.1
            self.camera.azimuth   = -45    # 从左前方看
            self.camera.elevation = -20

            # 设置初始姿态
            self._apply_angles()
            mujoco.mj_forward(self.model, self.data)

            print("[MuJoCo Widget] 初始化成功 —— 使用 dummy2 URDF 网格")

        except Exception as e:
            import traceback
            print(f"[MuJoCo Widget] 初始化失败: {e}")
            traceback.print_exc()
            self.image_label.setText(f"MuJoCo 初始化失败\n{e}")

    # ── 渲染循环 ─────────────────────────────────────────────────────────────

    def _apply_angles(self):
        """将固件角度转换后写入 MuJoCo qpos"""
        if self.model is None or self.data is None:
            return
        urdf_rad = firmware_to_urdf(self.fw_angles_deg)
        for name, rad in zip(_JOINT_NAMES, urdf_rad):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                adr = self.model.jnt_qposadr[jid]
                # 限幅到关节范围
                lo = self.model.jnt_range[jid, 0]
                hi = self.model.jnt_range[jid, 1]
                self.data.qpos[adr] = float(np.clip(rad, lo, hi))

    def _update_frame(self):
        if self.renderer is None or self.data is None:
            return
        try:
            self._apply_angles()
            mujoco.mj_forward(self.model, self.data)

            self.renderer.update_scene(self.data, self.camera)
            rgb = self.renderer.render()

            h, w, c = rgb.shape
            q_image = QImage(
                rgb.tobytes(), w, h, c * w, QImage.Format.Format_RGB888
            )
            self.image_label.setPixmap(QPixmap.fromImage(q_image))
            self._update_info()

        except Exception as e:
            import traceback
            print(f"[MuJoCo Widget] 渲染错误: {e}")
            traceback.print_exc()

    def _update_info(self):
        # 末端位置
        try:
            sid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
            )
            if sid >= 0:
                pos = self.data.site_xpos[sid]
                self.pos_label.setText(
                    f"末端: X={pos[0]*1000:.1f} Y={pos[1]*1000:.1f} Z={pos[2]*1000:.1f} mm"
                )
                rot = self.data.site_xmat[sid].reshape(3, 3)
                roll  = np.arctan2(rot[2, 1], rot[2, 2])
                pitch = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2 + rot[2, 2]**2))
                yaw   = np.arctan2(rot[1, 0], rot[0, 0])
                self.end_effector_updated.emit([
                    pos[0]*1000, pos[1]*1000, pos[2]*1000,
                    np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)
                ])
        except Exception:
            pass

        # 关节角度
        angles_str = " ".join(
            [f"J{i+1}={a:.1f}°" for i, a in enumerate(self.fw_angles_deg)]
        )
        self.angle_label.setText(angles_str)

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def update_joint_angles(self, angles: List[float]):
        """
        更新关节角度（固件度数）

        Args:
            angles: [J1..J6] in degrees (firmware convention)
        """
        if len(angles) >= 6:
            self.fw_angles_deg = list(angles[:6])

    def set_camera_view(self, view_name: str):
        """切换相机视角"""
        if self.camera is None:
            return
        views = {
            "front": ( 90, -15),
            "side":  (  0, -15),
            "top":   ( 90, -88),
            "iso":   ( 45, -25),
            "back":  (-90, -15),
        }
        if view_name in views:
            az, el = views[view_name]
            self.camera.azimuth   = az
            self.camera.elevation = el

    def closeEvent(self, event):
        self.update_timer.stop()
        if self.renderer:
            self.renderer.close()
        event.accept()


# ════════════════════════════════════════════════════════════════════════════
class MujocoDialog(QWidget):
    """独立 MuJoCo 可视化对话框（嵌入主窗口或单独弹出）"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("MuJoCo 3D 可视化")
        self.setMinimumSize(820, 620)

        layout = QVBoxLayout(self)

        self.mujoco_widget = MuJoCoWidget(self, width=800, height=480)
        layout.addWidget(self.mujoco_widget)

        from PyQt6.QtWidgets import QPushButton, QHBoxLayout
        btn_layout = QHBoxLayout()
        for name, vid in [("正面","front"),("侧面","side"),
                          ("俯视","top"),("等轴测","iso"),("背面","back")]:
            btn = QPushButton(name)
            btn.clicked.connect(
                lambda checked, v=vid: self.mujoco_widget.set_camera_view(v)
            )
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)

    def update_angles(self, angles: List[float]):
        self.mujoco_widget.update_joint_angles(angles)


# ════════════════════════════════════════════════════════════════════════════
# 独立运行入口：可选连接真实机械臂，无连接时播放演示动画
# ════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import sys
    import glob
    import serial
    import threading
    import time
    from PyQt6.QtWidgets import (
        QApplication, QDialog, QVBoxLayout, QHBoxLayout,
        QLabel, QComboBox, QPushButton, QMessageBox
    )
    from PyQt6.QtCore import QTimer

    _ser = None
    _serial_lock = threading.Lock()

    def _auto_detect_port():
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        return ports

    def _read_loop(dialog_ref):
        global _ser
        while True:
            time.sleep(0.1)
            if _ser is None or not _ser.is_open:
                continue
            try:
                with _serial_lock:
                    _ser.reset_input_buffer()
                    _ser.write(b'#GETJPOS\n')
                    time.sleep(0.15)
                    resp = _ser.read(_ser.in_waiting).decode(errors='ignore')
                parts = resp.strip().split()
                if len(parts) >= 7 and parts[0] == 'ok':
                    angles = [float(parts[i + 1]) for i in range(6)]
                    dialog_ref.update_angles(angles)
            except Exception:
                pass

    app = QApplication(sys.argv)

    root = QDialog()
    root.setWindowTitle("MuJoCo 仿真 — 独立运行")
    root.setMinimumSize(860, 680)
    root_layout = QVBoxLayout(root)

    # ── 串口连接栏 ──
    conn_bar = QHBoxLayout()
    conn_bar.addWidget(QLabel("串口:"))

    port_combo = QComboBox()
    port_combo.setMinimumWidth(160)
    port_combo.addItems(_auto_detect_port())
    conn_bar.addWidget(port_combo)

    def refresh_ports():
        port_combo.clear()
        port_combo.addItems(_auto_detect_port())

    conn_bar.addWidget(QPushButton("刷新", clicked=refresh_ports))

    connect_btn = QPushButton("连接")
    connect_btn.setStyleSheet(
        "background-color:#4CAF50; color:white; padding:4px 12px;"
    )
    conn_status = QLabel("● 未连接")
    conn_status.setStyleSheet("color:red; font-weight:bold;")

    def toggle_connect():
        global _ser
        if _ser and _ser.is_open:
            with _serial_lock:
                _ser.close()
            _ser = None
            connect_btn.setText("连接")
            connect_btn.setStyleSheet(
                "background-color:#4CAF50; color:white; padding:4px 12px;"
            )
            conn_status.setText("● 未连接")
            conn_status.setStyleSheet("color:red; font-weight:bold;")
        else:
            port = port_combo.currentText()
            if not port:
                QMessageBox.warning(root, "错误", "未检测到串口")
                return
            try:
                _ser = serial.Serial(port, 115200, timeout=2)
                time.sleep(0.2)
                connect_btn.setText("断开")
                connect_btn.setStyleSheet(
                    "background-color:#F44336; color:white; padding:4px 12px;"
                )
                conn_status.setText(f"● 已连接 {port}")
                conn_status.setStyleSheet("color:green; font-weight:bold;")
            except Exception as e:
                QMessageBox.critical(root, "错误", f"连接失败: {e}")

    connect_btn.clicked.connect(toggle_connect)
    conn_bar.addWidget(connect_btn)
    conn_bar.addWidget(conn_status)
    conn_bar.addStretch()
    root_layout.addLayout(conn_bar)

    # ── MuJoCo 视图 ──
    dialog = MujocoDialog(root)
    root_layout.addWidget(dialog)

    # 串口读取后台线程
    threading.Thread(target=_read_loop, args=(dialog,), daemon=True).start()

    # 演示动画（无串口连接时）— 在 REST 和 L 形姿态之间平滑过渡
    _demo_angles = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
    _demo_t = 0.0

    def _demo_animate():
        global _demo_angles, _demo_t
        if _ser and _ser.is_open:
            return
        _demo_t += 0.025
        s = (np.sin(_demo_t) + 1.0) * 0.5   # 0 ~ 1
        # J1 缓慢左右旋转
        _demo_angles[0] = 60.0 * np.sin(_demo_t * 0.4)
        # J2: REST(-75°) ~ 0° 过渡
        _demo_angles[1] = -75.0 * (1.0 - s)
        # J3: 180°(折叠) ~ 90°(L形) 过渡
        _demo_angles[2] = 90.0 + 90.0 * (1.0 - s)
        dialog.update_angles(_demo_angles)

    demo_timer = QTimer()
    demo_timer.timeout.connect(_demo_animate)
    demo_timer.start(50)   # 20 Hz

    root.show()
    sys.exit(app.exec())
