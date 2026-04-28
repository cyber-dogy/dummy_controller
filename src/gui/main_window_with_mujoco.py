"""
Dummy V2 机械臂控制器 — 主窗口（集成 MuJoCo 仿真 + 正逆解 + 仿真规划 + 示教）

布局：
  左栏  (330 px) — 串口连接、关节/夹爪控制、预设、电机操作
  中栏  (700 px) — MuJoCo 3D 仿真、末端状态信息
  右栏  (390 px) — QTabWidget：正逆解 | 真机示教 | 仿真规划

影子机制：
  - 主视图始终跟随真机（或空闲时跟随左侧关节滑块）
  - 仿真规划滑块驱动蓝色影子，两者同屏显示，互不干扰

仿真规划执行：
  - 以时间戳为节奏（类示教回放），逐帧发送，防止 CAN 总线丢步

路径独立：
  全部使用 __file__ 相对路径，dummy-controller 目录可随意移动
"""

import sys
import glob
import time
import os
from typing import List, Optional, Dict, Any
import numpy as np

# ── 路径设置（__file__ 相对，与工作目录无关）────────────────────────────────
_HERE        = os.path.dirname(os.path.abspath(__file__))   # src/gui
_SRC_DIR     = os.path.dirname(_HERE)                        # src
_PROJECT_DIR = os.path.dirname(_SRC_DIR)                     # dummy-controller
_MUJOCO_SIM  = os.path.join(_PROJECT_DIR, "mujoco_sim")

for _p in (_SRC_DIR, _MUJOCO_SIM):
    if _p not in sys.path:
        sys.path.insert(0, _p)
# ─────────────────────────────────────────────────────────────────────────────

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QSlider, QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit, QGroupBox,
    QFrame, QTabWidget, QSplitter, QMessageBox, QProgressBar, QSizePolicy,
    QCheckBox, QFileDialog, QGridLayout,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt6.QtGui import QFont

from core.robot import DummyRobot
from core.gripper import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    gripper_state_label,
    normalize_gripper_value,
)
from utils.config import (
    HOME_POSE,
    J6_FIRMWARE_REDUCTION,
    J6_OUTPUT_REDUCTION_DEFAULT,
    JOINT_NAMES,
    JOINT_LIMITS,
    POSES,
    RESET_POSE,
    JointLimitChecker,
)
from utils.light_config import LIGHT_RING_MODES, LIGHT_RING_STATE_LABELS
from utils.tcp_config import load_tcp_settings, normalize_tcp_settings, save_tcp_settings
from utils.world_config import (
    load_world_settings,
    normalize_world_settings,
    save_world_settings,
)

# ── MuJoCo（可选）────────────────────────────────────────────────────────────
try:
    from src.qt_integration import MuJoCoWidget
    MUJOCO_AVAILABLE = True
except ImportError as e:
    print(f"[警告] MuJoCo 不可用: {e}")
    MUJOCO_AVAILABLE = False

# ── 示教模块（可选）──────────────────────────────────────────────────────────
try:
    from core.teach_mode import TeachMode, TrajectoryPoint
    from gui.teach_dialog import PlaybackThread
    TEACH_AVAILABLE = True
except ImportError as e:
    print(f"[警告] 示教模块导入失败: {e}")
    TEACH_AVAILABLE = False

    # 最小回退：确保 TrajectoryPoint 始终可用（仿真规划录制用）
    class TrajectoryPoint:  # type: ignore[no-redef]
        def __init__(self, timestamp: float, angles: List[float], **kwargs):
            self.timestamp = float(timestamp)
            self.angles    = list(angles)
            self.source    = kwargs.get("source", "fallback")
            self.playback_speed = kwargs.get("playback_speed")
            self.gripper = normalize_gripper_value(kwargs.get("gripper", 0.0))
            self.target_gripper = (
                normalize_gripper_value(kwargs.get("target_gripper"))
                if kwargs.get("target_gripper") is not None
                else None
            )

        def playback_angles(self) -> List[float]:
            return list(self.angles)

        def playback_gripper(self) -> float:
            return normalize_gripper_value(
                self.target_gripper if self.target_gripper is not None else self.gripper
            )

        def playback_speed_value(self, default_speed: int = 30) -> int:
            speed = self.playback_speed if self.playback_speed is not None else default_speed
            return max(1, int(speed))


# ══════════════════════════════════════════════════════════════════════════════
# 后台线程
# ══════════════════════════════════════════════════════════════════════════════

class UpdateThread(QThread):
    """低频轮询真机位置，避免运动中挤占主控串口/CAN 调度。"""
    position_updated = pyqtSignal(list)
    POLL_INTERVAL_S = 0.8

    def __init__(self, robot: DummyRobot):
        super().__init__()
        self.robot   = robot
        self.running = True

    def run(self):
        while self.running:
            if self.robot.connected:
                pos = self.robot.get_position()
                if pos:
                    self.position_updated.emit(pos)
            time.sleep(self.POLL_INTERVAL_S)

    def stop(self):
        self.running = False


class SimPlaybackThread(QThread):
    """
    按录制时间戳回放仿真轨迹到真机。

    遵循示教回放节奏：每两点之间按录制时间间隔等待（速度倍率可调），
    避免 CAN 总线因指令堆积而丢步。
    """
    progress  = pyqtSignal(int, int)    # (current_idx, total)
    log       = pyqtSignal(str)
    finished  = pyqtSignal(bool, str)   # (success, message)

    def __init__(self, robot: DummyRobot,
                 trajectory: List[TrajectoryPoint],
                 speed_ratio: float = 1.0,
                 joint_speed: int = 30,
                 min_interval_s: float = 0.10):
        super().__init__()
        self.robot       = robot
        self.trajectory  = trajectory
        self.speed_ratio = max(0.1, speed_ratio)
        self.joint_speed = max(1, int(joint_speed))
        self.min_interval_s = max(0.08, float(min_interval_s))
        self._stop       = False

    def request_stop(self):
        self._stop = True

    def run(self):
        traj = self.trajectory
        n    = len(traj)
        if n == 0:
            self.finished.emit(False, "空轨迹")
            return

        self.log.emit(
            f"开始发送轨迹（{n} 点，×{self.speed_ratio:.1f} 速，关节速度 {self.joint_speed}°/s，最小间隔 {self.min_interval_s:.2f}s）"
        )

        last_gripper: Optional[float] = None
        for idx, pt in enumerate(traj):
            if self._stop:
                self.finished.emit(False, "已中止")
                return

            target_angles = pt.playback_angles() if hasattr(pt, "playback_angles") else pt.angles
            point_speed = (
                pt.playback_speed_value(self.joint_speed)
                if hasattr(pt, "playback_speed_value")
                else self.joint_speed
            )
            ok, msg = self.robot.move_to(
                target_angles,
                speed=point_speed,
                check_limits=False,
            )
            if not ok:
                self.finished.emit(False, f"第 {idx+1} 点发送失败: {msg}")
                return

            if hasattr(self.robot, "set_gripper_state"):
                target_gripper = normalize_gripper_value(
                    pt.playback_gripper() if hasattr(pt, "playback_gripper") else getattr(pt, "gripper", 0.0)
                )
                if last_gripper is None or abs(last_gripper - target_gripper) >= 1e-6:
                    self.robot.set_gripper_state(target_gripper)
                    last_gripper = target_gripper
                elif target_gripper < 0.5 and hasattr(self.robot, "refresh_gripper_hold"):
                    self.robot.refresh_gripper_hold()

            self.progress.emit(idx + 1, n)

            if idx < n - 1:
                dt = (traj[idx + 1].timestamp - pt.timestamp) / self.speed_ratio
                time.sleep(max(self.min_interval_s, dt))

        self.finished.emit(True, f"完成，共 {n} 点")


class IKWorkerThread(QThread):
    """后台执行 MuJoCo IK，避免阻塞主线程 UI。"""
    solved = pyqtSignal(object, object, object)   # result_angles, target_xyz, ee_xyz
    failed = pyqtSignal(str)

    def __init__(self, mujoco_widget: MuJoCoWidget,
                 target_xyz: List[float], initial_angles: List[float]):
        super().__init__()
        self.mujoco_widget = mujoco_widget
        self.target_xyz = list(target_xyz)
        self.initial_angles = list(initial_angles)

    def run(self):
        try:
            result = self.mujoco_widget.compute_ik(
                self.target_xyz,
                initial_fw_deg=self.initial_angles,
            )
            ee = self.mujoco_widget.get_last_ik_ee()
            self.solved.emit(result, self.target_xyz, ee)
        except Exception as e:
            self.failed.emit(str(e))


class GripperActionThread(QThread):
    """Run a bounded gripper action without blocking the Qt event loop."""

    action_finished = pyqtSignal(bool, str, float)  # success, message, target_value

    def __init__(self, robot: DummyRobot, target_value: Optional[float] = None, disable: bool = False):
        super().__init__()
        self.robot = robot
        self.target_value = target_value
        self.disable = disable

    def run(self):
        try:
            if self.disable:
                ok, msg = self.robot.disable_gripper()
                target = self.robot.get_target_gripper()
            else:
                target = normalize_gripper_value(self.target_value)
                ok, msg = self.robot.set_gripper_state(target)
            self.action_finished.emit(ok, msg, float(target))
        except Exception as exc:
            target = (
                normalize_gripper_value(self.target_value)
                if self.target_value is not None
                else self.robot.get_target_gripper()
            )
            self.action_finished.emit(False, str(exc), float(target))


# ══════════════════════════════════════════════════════════════════════════════
# 主窗口
# ══════════════════════════════════════════════════════════════════════════════

class MainWindowWithMuJoCo(QMainWindow):
    """Dummy V2 主控界面"""

    STABILITY_DEMO_CANDIDATES = [
        [0.0, 240.0, 320.0],
        [0.0, 220.0, 300.0],
        [0.0, 260.0, 300.0],
        [60.0, 240.0, 300.0],
        [-60.0, 240.0, 300.0],
        [80.0, 220.0, 280.0],
        [-80.0, 220.0, 280.0],
    ]
    STABILITY_DEMO_POINTS_PER_LOOP = 36
    STABILITY_DEMO_LOOPS = 2
    STABILITY_DEMO_DT = 0.46
    STABILITY_DEMO_SETTLE_TIME = 1.0
    STABILITY_DEMO_JOINT_SPEED = 22
    STABILITY_DEMO_EXEC_RATIO = 1.0
    STABILITY_DEMO_MIN_INTERVAL = 0.15
    STABILITY_DEMO_J1_AMPLITUDE = 16.0
    STABILITY_DEMO_J4_AMPLITUDE = 56.0
    STABILITY_DEMO_J5_AMPLITUDE = 34.0
    STABILITY_DEMO_J6_AMPLITUDE = 60.0
    STABILITY_DEMO_JOINT_WEIGHTS = [0.35, 0.45, 0.45, 2.15, 1.70, 0.16]
    DIRECT_PREVIEW_DIFF_DEG = 1.5

    # ── 初始化 ────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dummy V2 机械臂控制器")
        self.setMinimumSize(1420, 860)
        self.setFont(QFont("Noto Sans CJK SC", 10))

        self.robot         = DummyRobot()
        self.update_thread: Optional[UpdateThread] = None
        self._tcp_settings = load_tcp_settings()
        self._world_settings = load_world_settings()
        self._mujoco_guard_enabled = bool(self._world_settings.get("mujoco_guard_enabled", False))
        self._j6_firmware_reduction = float(
            self._world_settings.get("j6_firmware_reduction", J6_FIRMWARE_REDUCTION)
        )
        self._j6_output_reduction = float(
            self._world_settings.get("j6_output_reduction", J6_OUTPUT_REDUCTION_DEFAULT)
        )

        # MuJoCo 组件
        self.mujoco_widget: Optional[MuJoCoWidget] = None

        # 仿真规划状态
        self.sim_recording     = False
        self.sim_trajectory:   List[TrajectoryPoint] = []
        self.sim_rec_start     = 0.0
        self.sim_rec_timer     = QTimer(self)
        self.sim_rec_timer.timeout.connect(self._sim_rec_tick)
        self.sim_preview_timer = QTimer(self)
        self.sim_preview_timer.timeout.connect(self._sim_preview_tick)
        self.sim_preview_idx   = 0
        self.sim_preview_active = False
        self.sim_exec_thread:  Optional[SimPlaybackThread] = None
        self._sim_traj_io = TeachMode(self.robot) if TEACH_AVAILABLE else None

        # 示教状态
        self.teach_mode_obj  = None
        self.teach_in_mode   = False
        self.teach_recording = False
        self.teach_trajectory: List[TrajectoryPoint] = []
        self.teach_playback_thread = None
        self.gui_live_follow_enabled = False
        self.gui_live_follow_dirty = False
        self.gui_live_recording = False
        self.gui_live_trajectory: List[TrajectoryPoint] = []
        self.gui_live_recorder = TeachMode(self.robot) if TEACH_AVAILABLE else None
        self._live_last_sent_angles: Optional[List[float]] = None
        self._live_last_sent_gripper: Optional[float] = None
        self._suppress_live_follow = False
        self._gripper_target_value = GRIPPER_CLOSED
        self._sim_gripper_value = GRIPPER_CLOSED
        self._gripper_action_thread: Optional[GripperActionThread] = None

        # IK 最近结果
        self.ik_result_angles: Optional[List[float]] = None
        self.ik_thread: Optional[IKWorkerThread] = None
        self.stability_demo_center_xyz: Optional[List[float]] = None
        self.stability_demo_anchor_angles: Optional[List[float]] = None

        self._create_ui()
        self._refresh_sim_safety_status()
        self._refresh_direct_target_preview()
        self._populate_tcp_offset_ui()
        self._populate_world_settings_ui()
        self._populate_light_settings_ui()
        self._auto_detect_port()

        # 定时器
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(500)

        if MUJOCO_AVAILABLE:
            self.mujoco_sync_timer = QTimer(self)
            self.mujoco_sync_timer.timeout.connect(self._sync_mujoco)
            self.mujoco_sync_timer.start(50)   # 20 Hz 同步

        self.teach_count_timer = QTimer(self)
        self.teach_count_timer.timeout.connect(self._teach_update_count)
        self.teach_count_timer.start(300)

        self.live_follow_timer = QTimer(self)
        self.live_follow_timer.timeout.connect(self._live_follow_tick)
        self.live_follow_timer.start(100)

    # ── 界面构建 ──────────────────────────────────────────────────────────────

    def _create_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        rl = QHBoxLayout(root)
        rl.setContentsMargins(8, 8, 8, 8)
        rl.setSpacing(6)

        h_split = QSplitter(Qt.Orientation.Horizontal)
        h_split.addWidget(self._build_left())
        h_split.addWidget(self._build_middle())
        h_split.addWidget(self._build_right())
        h_split.setSizes([330, 700, 390])
        rl.addWidget(h_split)

        self._log("Dummy V2 控制器启动")
        if MUJOCO_AVAILABLE:
            self._log("✅ MuJoCo 已就绪（影子渲染 + 逆解）")
        else:
            self._log("⚠️ MuJoCo 未安装，仿真不可用")
        if not TEACH_AVAILABLE:
            self._log("⚠️ 示教模块不可用")

    # ══════════════════════════════════════════════════════════════════════════
    # 左列：串口 + 关节/夹爪控制 + 预设
    # ══════════════════════════════════════════════════════════════════════════

    def _build_left(self) -> QFrame:
        frame = QFrame()
        vl    = QVBoxLayout(frame)
        vl.setSpacing(5)

        # 串口连接
        conn = QGroupBox("串口连接")
        cl   = QHBoxLayout(conn)
        cl.addWidget(QLabel("端口:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(110)
        cl.addWidget(self.port_combo)
        btn_r = QPushButton("刷新")
        btn_r.clicked.connect(self._refresh_ports)
        cl.addWidget(btn_r)
        self.connect_btn = QPushButton("连接")
        self.connect_btn.setStyleSheet("background:#4CAF50;color:white;")
        self.connect_btn.clicked.connect(self._toggle_connection)
        cl.addWidget(self.connect_btn)
        self.status_label = QLabel("● 未连接")
        self.status_label.setStyleSheet("color:red;font-weight:bold;")
        cl.addWidget(self.status_label)
        vl.addWidget(conn)

        # 关节控制滑块
        jbox = QGroupBox("关节控制（目标 / 实际）")
        jl   = QVBoxLayout(jbox)
        self.joint_sliders:    List[QSlider] = []
        self.joint_cur_labels: List[QLabel]  = []
        self.joint_tgt_labels: List[QLabel]  = []
        REST = list(RESET_POSE)
        for i in range(6):
            row = QFrame()
            rl2 = QHBoxLayout(row)
            rl2.setContentsMargins(2, 1, 2, 1)
            n_lbl = QLabel(JOINT_NAMES[i])
            n_lbl.setMinimumWidth(88)
            n_lbl.setStyleSheet("font-weight:bold;font-size:9px;")
            rl2.addWidget(n_lbl)
            cur = QLabel("--°")
            cur.setMinimumWidth(64 if i == 5 else 42)
            cur.setStyleSheet("color:#888;font-size:9px;")
            rl2.addWidget(cur)
            self.joint_cur_labels.append(cur)
            lo, hi = JOINT_LIMITS[i]
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(lo, hi)
            sl.setValue(REST[i])
            sl.setMinimumWidth(140)
            rl2.addWidget(sl)
            self.joint_sliders.append(sl)
            tgt = QLabel(self._format_joint_angle_label(i, REST[i]))
            tgt.setMinimumWidth(64 if i == 5 else 38)
            tgt.setStyleSheet("color:#2196F3;font-weight:bold;font-size:9px;")
            rl2.addWidget(tgt)
            self.joint_tgt_labels.append(tgt)
            zero = QPushButton("0")
            zero.setMaximumWidth(24)
            zero.clicked.connect(lambda _, idx=i: self._set_joint_zero(idx))
            rl2.addWidget(zero)
            sl.valueChanged.connect(lambda v, idx=i: self._on_slider_change(idx, v))
            jl.addWidget(row)
        jl.addWidget(self._build_gripper_control_panel())
        vl.addWidget(jbox)

        # 预设位置
        poses_box = QGroupBox("预设位置")
        pl = QVBoxLayout(poses_box)
        pl.setSpacing(3)
        for name, angles in POSES.items():
            b = QPushButton(name)
            b.setStyleSheet("padding:5px;text-align:left;")
            b.clicked.connect(lambda _, a=angles, n=name: self._load_pose(a, n))
            pl.addWidget(b)
        vl.addWidget(poses_box)

        # 电机控制 & 速度
        ctrl_row = QHBoxLayout()
        self.enable_btn = QPushButton("🔴 使能")
        self.enable_btn.clicked.connect(self._toggle_enable)
        ctrl_row.addWidget(self.enable_btn)
        stop_btn = QPushButton("⏹ 急停")
        stop_btn.setStyleSheet("background:#F44336;color:white;")
        stop_btn.clicked.connect(self._emergency_stop)
        ctrl_row.addWidget(stop_btn)
        ctrl_row.addWidget(QLabel("速度:"))
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 100)
        self.speed_spin.setValue(30)
        self.speed_spin.setMaximumWidth(58)
        ctrl_row.addWidget(self.speed_spin)
        vl.addLayout(ctrl_row)

        # 执行运动
        self.move_btn = QPushButton("▶  执行运动")
        self.move_btn.setStyleSheet("""
            QPushButton{background:#2196F3;color:white;font-size:13px;
                        padding:8px;border-radius:4px;}
            QPushButton:hover{background:#1976D2;}
        """)
        self.move_btn.clicked.connect(self._execute_move)
        vl.addWidget(self.move_btn)
        vl.addStretch()

        return frame

    def _j6_ratio(self) -> float:
        return max(1.0, float(getattr(self, "_j6_output_reduction", J6_OUTPUT_REDUCTION_DEFAULT)))

    def _j6_fw_ratio(self) -> float:
        return max(1.0, float(getattr(self, "_j6_firmware_reduction", J6_FIRMWARE_REDUCTION)))

    def _j6_output_scale(self) -> float:
        return max(1e-6, self._j6_fw_ratio() / self._j6_ratio())

    def _format_joint_angle_label(self, idx: int, angle: float) -> str:
        if idx == 5:
            scale = self._j6_output_scale()
            if abs(scale - 1.0) > 1e-4:
                return f"F{angle:.0f}°/O{angle * scale:.1f}°"
        return f"{angle:.0f}°"

    def _mujoco_visual_angles(self, angles: List[float]) -> List[float]:
        visual = [float(a) for a in angles[:6]]
        if len(visual) >= 6:
            visual[5] = visual[5] * self._j6_output_scale()
        return visual

    def _refresh_joint_angle_labels(self):
        if hasattr(self, "joint_sliders") and hasattr(self, "joint_tgt_labels"):
            for i, slider in enumerate(self.joint_sliders[:6]):
                self.joint_tgt_labels[i].setText(self._format_joint_angle_label(i, slider.value()))
        if hasattr(self, "joint_cur_labels") and self.robot.current_angles:
            for i, a in enumerate(self.robot.current_angles[:6]):
                self.joint_cur_labels[i].setText(self._format_joint_angle_label(i, a))
        if hasattr(self, "sim_sliders") and hasattr(self, "sim_val_labels"):
            for i, slider in enumerate(self.sim_sliders[:6]):
                self.sim_val_labels[i].setText(self._format_joint_angle_label(i, slider.value()))

    def _on_j6_output_reduction_changed(self, value: float):
        self._j6_output_reduction = max(1.0, float(value))
        self._world_settings["j6_output_reduction"] = self._j6_output_reduction
        for spin_name in ("j6_reduction_spin", "safety_j6_reduction_spin"):
            spin = getattr(self, spin_name, None)
            if spin is not None and abs(spin.value() - self._j6_output_reduction) > 1e-6:
                old = spin.blockSignals(True)
                spin.setValue(self._j6_output_reduction)
                spin.blockSignals(old)
        self._refresh_joint_angle_labels()
        self._refresh_safety_boundary_status()
        self._refresh_direct_target_preview()

    def _on_j6_firmware_reduction_changed(self, value: float):
        self._j6_firmware_reduction = max(1.0, float(value))
        self._world_settings["j6_firmware_reduction"] = self._j6_firmware_reduction
        for spin_name in ("j6_firmware_reduction_spin", "safety_j6_firmware_reduction_spin"):
            spin = getattr(self, spin_name, None)
            if spin is not None and abs(spin.value() - self._j6_firmware_reduction) > 1e-6:
                old = spin.blockSignals(True)
                spin.setValue(self._j6_firmware_reduction)
                spin.blockSignals(old)
        self._refresh_joint_angle_labels()
        self._refresh_safety_boundary_status()
        self._refresh_direct_target_preview()

    def _apply_j6_motor_tuning(self):
        if not self.robot.connected:
            QMessageBox.warning(self, "J6参数", "机械臂串口未连接，未发送任何指令。")
            return
        current = self.j6_current_limit_spin.value()
        speed = self.j6_speed_limit_spin.value()
        acc = self.j6_acc_limit_spin.value()
        ok_current = self.robot.set_joint_current_limit(6, current)
        time.sleep(0.04)
        ok_speed = self.robot.set_joint_speed_limit(6, speed)
        time.sleep(0.04)
        ok_acc = self.robot.set_joint_acceleration(6, acc)
        if ok_current and ok_speed and ok_acc:
            self._log(f"J6参数已发送：电流 {current:.2f}A，速度 {speed:.0f}，加速度 {acc:.0f}")
        else:
            QMessageBox.warning(self, "J6参数", "J6参数发送失败，请检查串口连接和主控状态。")

    def _build_gripper_control_panel(self) -> QFrame:
        panel = QFrame()
        panel.setFrameShape(QFrame.Shape.StyledPanel)
        panel.setStyleSheet("QFrame{border:1px solid #ddd;border-radius:4px;background:#fafafa;}")
        gl = QVBoxLayout(panel)
        gl.setContentsMargins(6, 5, 6, 5)
        gl.setSpacing(4)

        title_row = QHBoxLayout()
        title = QLabel("J7 线轨夹爪")
        title.setStyleSheet("font-weight:bold;font-size:10px;color:#333;border:0;background:transparent;")
        title_row.addWidget(title)
        title_row.addStretch()
        gl.addLayout(title_row)

        j6_ratio_row = QHBoxLayout()
        j6_ratio_row.addWidget(QLabel("J6固件比"))
        self.j6_firmware_reduction_spin = QDoubleSpinBox()
        self.j6_firmware_reduction_spin.setRange(1.0, 200.0)
        self.j6_firmware_reduction_spin.setDecimals(2)
        self.j6_firmware_reduction_spin.setSingleStep(1.0)
        self.j6_firmware_reduction_spin.setValue(self._j6_fw_ratio())
        self.j6_firmware_reduction_spin.setMaximumWidth(74)
        self.j6_firmware_reduction_spin.valueChanged.connect(self._on_j6_firmware_reduction_changed)
        j6_ratio_row.addWidget(self.j6_firmware_reduction_spin)
        j6_ratio_row.addWidget(QLabel("实际比"))
        self.j6_reduction_spin = QDoubleSpinBox()
        self.j6_reduction_spin.setRange(1.0, 200.0)
        self.j6_reduction_spin.setDecimals(2)
        self.j6_reduction_spin.setSingleStep(1.0)
        self.j6_reduction_spin.setValue(self._j6_ratio())
        self.j6_reduction_spin.setMaximumWidth(74)
        self.j6_reduction_spin.valueChanged.connect(self._on_j6_output_reduction_changed)
        j6_ratio_row.addWidget(self.j6_reduction_spin)
        j6_ratio_row.addStretch()
        gl.addLayout(j6_ratio_row)

        j6_tune_row = QHBoxLayout()
        j6_tune_row.addWidget(QLabel("J6电流"))
        self.j6_current_limit_spin = QDoubleSpinBox()
        self.j6_current_limit_spin.setRange(0.10, 2.00)
        self.j6_current_limit_spin.setDecimals(2)
        self.j6_current_limit_spin.setSingleStep(0.05)
        self.j6_current_limit_spin.setValue(1.20)
        self.j6_current_limit_spin.setSuffix(" A")
        self.j6_current_limit_spin.setMaximumWidth(78)
        j6_tune_row.addWidget(self.j6_current_limit_spin)
        j6_tune_row.addWidget(QLabel("速"))
        self.j6_speed_limit_spin = QDoubleSpinBox()
        self.j6_speed_limit_spin.setRange(1.0, 100.0)
        self.j6_speed_limit_spin.setDecimals(0)
        self.j6_speed_limit_spin.setValue(24.0)
        self.j6_speed_limit_spin.setMaximumWidth(58)
        j6_tune_row.addWidget(self.j6_speed_limit_spin)
        j6_tune_row.addWidget(QLabel("加"))
        self.j6_acc_limit_spin = QDoubleSpinBox()
        self.j6_acc_limit_spin.setRange(1.0, 100.0)
        self.j6_acc_limit_spin.setDecimals(0)
        self.j6_acc_limit_spin.setValue(18.0)
        self.j6_acc_limit_spin.setMaximumWidth(58)
        j6_tune_row.addWidget(self.j6_acc_limit_spin)
        self.j6_apply_tuning_btn = QPushButton("应用J6")
        self.j6_apply_tuning_btn.clicked.connect(self._apply_j6_motor_tuning)
        j6_tune_row.addWidget(self.j6_apply_tuning_btn)
        j6_tune_row.addStretch()
        gl.addLayout(j6_tune_row)

        mode_row = QHBoxLayout()
        self.gripper_placeholder_chk = QCheckBox("占位模式（不下发硬件）")
        self.gripper_placeholder_chk.setChecked(self.robot.is_gripper_placeholder_mode())
        self.gripper_placeholder_chk.stateChanged.connect(self._toggle_gripper_placeholder_mode)
        mode_row.addWidget(self.gripper_placeholder_chk)
        mode_row.addStretch()
        gl.addLayout(mode_row)

        self.gripper_mode_lbl = QLabel("")
        self.gripper_mode_lbl.setStyleSheet("color:#555;font-size:10px;border:0;background:transparent;")
        gl.addWidget(self.gripper_mode_lbl)

        state_row = QHBoxLayout()
        state_row.addWidget(QLabel("目标:"))
        self.gripper_state_lbl = QLabel("夹紧")
        self.gripper_state_lbl.setMinimumWidth(52)
        self.gripper_state_lbl.setStyleSheet(
            "color:#1565C0;font-weight:bold;font-size:11px;border:0;background:transparent;"
        )
        state_row.addWidget(self.gripper_state_lbl)
        self.gripper_hold_lbl = QLabel("保持：--")
        self.gripper_hold_lbl.setStyleSheet("color:#555;font-size:10px;border:0;background:transparent;")
        state_row.addWidget(self.gripper_hold_lbl)
        state_row.addStretch()
        gl.addLayout(state_row)

        quick_row = QHBoxLayout()
        self.gripper_open_btn = QPushButton("打开")
        self.gripper_open_btn.clicked.connect(lambda: self._command_gripper(GRIPPER_OPEN))
        quick_row.addWidget(self.gripper_open_btn)
        self.gripper_close_btn = QPushButton("夹紧")
        self.gripper_close_btn.setStyleSheet("background:#607D8B;color:white;padding:5px;")
        self.gripper_close_btn.clicked.connect(lambda: self._command_gripper(GRIPPER_CLOSED))
        quick_row.addWidget(self.gripper_close_btn)
        self.gripper_disable_btn = QPushButton("断使能")
        self.gripper_disable_btn.clicked.connect(self._disable_gripper_from_ui)
        quick_row.addWidget(self.gripper_disable_btn)
        quick_row.addStretch()
        gl.addLayout(quick_row)

        grip_cfg = QGridLayout()
        self.gripper_open_current_spin = QDoubleSpinBox()
        self.gripper_close_current_spin = QDoubleSpinBox()
        self.gripper_hold_current_spin = QDoubleSpinBox()
        self.gripper_open_pulse_spin = QDoubleSpinBox()
        self.gripper_close_pulse_spin = QDoubleSpinBox()
        self.gripper_hold_timeout_spin = QDoubleSpinBox()
        for spin, value, low, high, suffix in (
            (self.gripper_open_current_spin, 0.70, 0.05, 1.20, " A"),
            (self.gripper_close_current_spin, 0.70, 0.05, 1.20, " A"),
            (self.gripper_hold_current_spin, 0.25, 0.00, 0.80, " A"),
            (self.gripper_open_pulse_spin, 1.00, 0.05, 2.50, " s"),
            (self.gripper_close_pulse_spin, 0.50, 0.05, 2.50, " s"),
            (self.gripper_hold_timeout_spin, 60.0, 1.0, 300.0, " s"),
        ):
            spin.setRange(low, high)
            spin.setDecimals(2 if suffix.strip() == "A" else 1)
            spin.setSingleStep(0.05 if suffix.strip() == "A" else 0.1)
            spin.setValue(value)
            spin.setSuffix(suffix)
            spin.setMaximumWidth(82)
            spin.valueChanged.connect(self._apply_gripper_settings_from_ui)

        self.gripper_hold_chk = QCheckBox("夹紧后保持")
        self.gripper_hold_chk.setChecked(True)
        self.gripper_hold_chk.stateChanged.connect(self._apply_gripper_settings_from_ui)

        grip_cfg.addWidget(QLabel("打开电流"), 0, 0)
        grip_cfg.addWidget(self.gripper_open_current_spin, 0, 1)
        grip_cfg.addWidget(QLabel("打开时长"), 0, 2)
        grip_cfg.addWidget(self.gripper_open_pulse_spin, 0, 3)
        grip_cfg.addWidget(QLabel("夹紧电流"), 1, 0)
        grip_cfg.addWidget(self.gripper_close_current_spin, 1, 1)
        grip_cfg.addWidget(QLabel("夹紧时长"), 1, 2)
        grip_cfg.addWidget(self.gripper_close_pulse_spin, 1, 3)
        grip_cfg.addWidget(QLabel("保持电流"), 2, 0)
        grip_cfg.addWidget(self.gripper_hold_current_spin, 2, 1)
        grip_cfg.addWidget(self.gripper_hold_chk, 2, 2)
        grip_cfg.addWidget(self.gripper_hold_timeout_spin, 2, 3)
        gl.addLayout(grip_cfg)

        self._apply_gripper_settings_from_ui()
        self._refresh_gripper_mode_label()
        self._refresh_gripper_ui_state()
        return panel

    def _build_direct_preview_panel(self) -> QGroupBox:
        preview_box = QGroupBox("目标影子预览")
        preview_box.setMaximumHeight(140)
        pvl = QVBoxLayout(preview_box)
        pvl.setContentsMargins(8, 5, 8, 6)
        pvl.setSpacing(4)

        preview_hint = QLabel(
            "暖色影子表示左侧滑块目标姿态；主视图继续显示实际机械臂。"
        )
        preview_hint.setWordWrap(True)
        preview_hint.setStyleSheet("color:#666;font-size:10px;")
        pvl.addWidget(preview_hint)

        preview_row = QHBoxLayout()
        self.direct_preview_chk = QCheckBox("显示直接控制目标影子")
        self.direct_preview_chk.setChecked(True)
        self.direct_preview_chk.stateChanged.connect(self._toggle_direct_target_preview)
        preview_row.addWidget(self.direct_preview_chk)
        preview_row.addStretch()
        pvl.addLayout(preview_row)

        self.direct_preview_status_lbl = QLabel("目标影子：待评估")
        self.direct_preview_status_lbl.setWordWrap(True)
        self.direct_preview_status_lbl.setStyleSheet(
            "font-size:10px;background:#fff4eb;padding:6px;border-radius:4px;color:#8a5a21;"
        )
        pvl.addWidget(self.direct_preview_status_lbl)
        return preview_box

    def _build_log_panel(self) -> QGroupBox:
        log_box = QGroupBox("日志")
        log_box.setMaximumHeight(140)
        ll = QVBoxLayout(log_box)
        ll.setContentsMargins(8, 5, 8, 6)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMinimumHeight(78)
        self.log_text.setStyleSheet("font-size:10px;")
        ll.addWidget(self.log_text)
        return log_box

    # ══════════════════════════════════════════════════════════════════════════
    # 中列：3D 仿真 + 末端状态
    # ══════════════════════════════════════════════════════════════════════════

    def _build_middle(self) -> QFrame:
        frame = QFrame()
        vl    = QVBoxLayout(frame)
        vl.setSpacing(4)

        sim_box = QGroupBox("🎮  MuJoCo 3D 仿真")
        sl = QVBoxLayout(sim_box)
        sl.setSpacing(3)

        if MUJOCO_AVAILABLE:
            self.mujoco_widget = MuJoCoWidget(self, width=680, height=450)
            sl.addWidget(self.mujoco_widget)

            overlay_legend = QLabel(
                "主视图 = 真机实际姿态 | 暖色影子 = 左侧直接控制目标 | 蓝色影子 = 仿真规划 / IK 预览"
            )
            overlay_legend.setWordWrap(True)
            overlay_legend.setStyleSheet("color:#666;font-size:10px;")
            sl.addWidget(overlay_legend)

            # 视角按钮
            vrow = QHBoxLayout()
            vrow.addWidget(QLabel("视角:"))
            for label, vid in [("正","front"),("侧","side"),
                                ("俯","top"),("轴","iso"),("背","back")]:
                b = QPushButton(label)
                b.setMaximumWidth(38)
                b.setStyleSheet("padding:3px;")
                b.clicked.connect(lambda _, v=vid: self._set_view(v))
                vrow.addWidget(b)
            vrow.addStretch()
            sl.addLayout(vrow)

            self.mujoco_widget.end_effector_updated.connect(self._update_ee_label)
        else:
            sl.addWidget(QLabel("MuJoCo 未安装，无法显示 3D 仿真"))

        vl.addWidget(sim_box)

        aux_row = QHBoxLayout()
        aux_row.setSpacing(6)
        aux_row.addWidget(self._build_direct_preview_panel(), stretch=2)
        aux_row.addWidget(self._build_log_panel(), stretch=3)
        vl.addLayout(aux_row)

        # 末端状态
        ee_box = QGroupBox("末端状态（正解）")
        el = QVBoxLayout(ee_box)
        el.setContentsMargins(8, 4, 8, 4)
        self.ee_label     = QLabel("末端位置: 等待…")
        self.fk_xyz_label = QLabel("位置: --")
        self.fk_rpy_label = QLabel("姿态: --")
        for lbl in (self.ee_label, self.fk_xyz_label, self.fk_rpy_label):
            lbl.setStyleSheet("font-family:monospace;font-size:11px;")
            el.addWidget(lbl)
        vl.addWidget(ee_box)

        vl.addStretch()
        return frame

    # ══════════════════════════════════════════════════════════════════════════
    # 右列：选项卡
    # ══════════════════════════════════════════════════════════════════════════

    def _build_right(self) -> QTabWidget:
        tabs = QTabWidget()
        tabs.addTab(self._build_tab_fk_ik(),   "📐 正逆解")
        tabs.addTab(self._build_tab_teach(),    "🎓 示教")
        tabs.addTab(self._build_tab_sim_traj(), "🎮 仿真规划")
        tabs.addTab(self._build_tab_safety_boundary(), "🧭 安全边界")
        tabs.addTab(self._build_tab_light_ring(), "💡 灯环")
        return tabs

    # ── 选项卡 0：正逆解 ──────────────────────────────────────────────────────

    def _build_tab_fk_ik(self) -> QWidget:
        w  = QWidget()
        vl = QVBoxLayout(w)
        vl.setSpacing(6)

        tcp_box = QGroupBox("TCP 偏移（末端点）")
        tl = QVBoxLayout(tcp_box)

        tcp_hint = QLabel(
            "当前软件把 TCP 定义在 J6 前向延伸的位置。默认是 75 mm。\n"
            "如果你现在没装夹爪、实际 J6 没那么长，可以把这个值调短；它会同时影响正解、IK、演示轨迹和导出。"
        )
        tcp_hint.setWordWrap(True)
        tcp_hint.setStyleSheet("color:#666;font-size:10px;")
        tl.addWidget(tcp_hint)

        tcp_row = QHBoxLayout()
        tcp_row.addWidget(QLabel("偏移:"))
        self.tcp_offset_spin = QDoubleSpinBox()
        self.tcp_offset_spin.setRange(0.0, 150.0)
        self.tcp_offset_spin.setDecimals(1)
        self.tcp_offset_spin.setSingleStep(1.0)
        self.tcp_offset_spin.setSuffix(" mm")
        self.tcp_offset_spin.setValue(float(self._tcp_settings["tcp_offset_mm"]))
        tcp_row.addWidget(self.tcp_offset_spin)
        for preset in (0.0, 25.0, 50.0, 75.0):
            btn = QPushButton(f"{int(preset)}")
            btn.setMaximumWidth(38)
            btn.clicked.connect(
                lambda _, v=preset: self.tcp_offset_spin.setValue(v)
            )
            tcp_row.addWidget(btn)
        tcp_apply_btn = QPushButton("保存并应用")
        tcp_apply_btn.clicked.connect(self._save_tcp_offset_from_ui)
        tcp_row.addWidget(tcp_apply_btn)
        tcp_row.addStretch()
        tl.addLayout(tcp_row)

        self.tcp_offset_status_lbl = QLabel("当前 TCP 偏移：--")
        self.tcp_offset_status_lbl.setStyleSheet("color:#555;font-size:10px;")
        tl.addWidget(self.tcp_offset_status_lbl)
        vl.addWidget(tcp_box)

        world_box = QGroupBox("世界坐标标定（旧 MuJoCo 参数）")
        wl = QVBoxLayout(world_box)

        world_hint = QLabel(
            "用于把 MuJoCo 世界坐标对齐到你的真实工作台。\n"
            "推荐顺序：先调 TCP，再标定工作面 Z，再设置世界原点/Yaw；这些参数默认不再拦截真机运动。"
        )
        world_hint.setWordWrap(True)
        world_hint.setStyleSheet("color:#666;font-size:10px;")
        wl.addWidget(world_hint)

        origin_row = QHBoxLayout()
        self.world_origin_x_spin = QDoubleSpinBox()
        self.world_origin_y_spin = QDoubleSpinBox()
        self.world_origin_z_spin = QDoubleSpinBox()
        self.world_yaw_spin = QDoubleSpinBox()
        for label_text, spin, low, high, suffix in (
            ("X0:", self.world_origin_x_spin, -1000.0, 1000.0, " mm"),
            ("Y0:", self.world_origin_y_spin, -1000.0, 1000.0, " mm"),
            ("Z0:", self.world_origin_z_spin, -1000.0, 1000.0, " mm"),
            ("Yaw:", self.world_yaw_spin, -180.0, 180.0, " °"),
        ):
            spin.setRange(low, high)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setSuffix(suffix)
            origin_row.addWidget(QLabel(label_text))
            origin_row.addWidget(spin)
        wl.addLayout(origin_row)

        surface_row = QHBoxLayout()
        self.work_surface_z_spin = QDoubleSpinBox()
        self.min_tcp_clearance_spin = QDoubleSpinBox()
        self.min_arm_clearance_spin = QDoubleSpinBox()
        self.base_guard_radius_spin = QDoubleSpinBox()
        self.base_guard_height_spin = QDoubleSpinBox()
        for label_text, spin, low, high in (
            ("工作面Z:", self.work_surface_z_spin, -500.0, 1000.0),
            ("TCP离台:", self.min_tcp_clearance_spin, 0.0, 300.0),
            ("连杆离台:", self.min_arm_clearance_spin, 0.0, 300.0),
            ("基座半径:", self.base_guard_radius_spin, 0.0, 500.0),
            ("基座高度:", self.base_guard_height_spin, 0.0, 500.0),
        ):
            spin.setRange(low, high)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setSuffix(" mm")
            surface_row.addWidget(QLabel(label_text))
            surface_row.addWidget(spin)
        wl.addLayout(surface_row)

        workspace_box = QGroupBox("旧工作空间盒子（可选）")
        wsl = QVBoxLayout(workspace_box)
        self.workspace_enabled_chk = QCheckBox("作为旧 MuJoCo 检查的一部分")
        wsl.addWidget(self.workspace_enabled_chk)

        ws_grid = QGridLayout()
        self.ws_min_x_spin = QDoubleSpinBox()
        self.ws_max_x_spin = QDoubleSpinBox()
        self.ws_min_y_spin = QDoubleSpinBox()
        self.ws_max_y_spin = QDoubleSpinBox()
        self.ws_min_z_spin = QDoubleSpinBox()
        self.ws_max_z_spin = QDoubleSpinBox()
        for spin in (
            self.ws_min_x_spin,
            self.ws_max_x_spin,
            self.ws_min_y_spin,
            self.ws_max_y_spin,
            self.ws_min_z_spin,
            self.ws_max_z_spin,
        ):
            spin.setRange(-1200.0, 1200.0)
            spin.setDecimals(1)
            spin.setSingleStep(5.0)
            spin.setSuffix(" mm")
        ws_grid.addWidget(QLabel("Xmin"), 0, 0)
        ws_grid.addWidget(self.ws_min_x_spin, 0, 1)
        ws_grid.addWidget(QLabel("Xmax"), 0, 2)
        ws_grid.addWidget(self.ws_max_x_spin, 0, 3)
        ws_grid.addWidget(QLabel("Ymin"), 1, 0)
        ws_grid.addWidget(self.ws_min_y_spin, 1, 1)
        ws_grid.addWidget(QLabel("Ymax"), 1, 2)
        ws_grid.addWidget(self.ws_max_y_spin, 1, 3)
        ws_grid.addWidget(QLabel("Zmin"), 2, 0)
        ws_grid.addWidget(self.ws_min_z_spin, 2, 1)
        ws_grid.addWidget(QLabel("Zmax"), 2, 2)
        ws_grid.addWidget(self.ws_max_z_spin, 2, 3)
        wsl.addLayout(ws_grid)

        ws_btn_row = QHBoxLayout()
        ws_min_btn = QPushButton("用当前末端写入下角")
        ws_min_btn.clicked.connect(lambda: self._capture_current_world_point_to_workspace("min"))
        ws_btn_row.addWidget(ws_min_btn)
        ws_max_btn = QPushButton("用当前末端写入上角")
        ws_max_btn.clicked.connect(lambda: self._capture_current_world_point_to_workspace("max"))
        ws_btn_row.addWidget(ws_max_btn)
        ws_btn_row.addStretch()
        wsl.addLayout(ws_btn_row)
        wl.addWidget(workspace_box)

        world_action_row = QHBoxLayout()
        world_apply_btn = QPushButton("保存并应用标定")
        world_apply_btn.clicked.connect(self._save_world_settings_from_ui)
        world_action_row.addWidget(world_apply_btn)
        world_action_row.addStretch()
        wl.addLayout(world_action_row)

        self.world_status_lbl = QLabel("世界标定：--")
        self.world_status_lbl.setWordWrap(True)
        self.world_status_lbl.setStyleSheet("color:#555;font-size:10px;")
        wl.addWidget(self.world_status_lbl)
        vl.addWidget(world_box)

        # FK（只读，由末端状态信号更新）
        fk_box = QGroupBox("正解 — 当前末端位置（实时）")
        fl = QVBoxLayout(fk_box)
        self.fk_tab_xyz = QLabel("位置: --")
        self.fk_tab_rpy = QLabel("姿态: --")
        for lbl in (self.fk_tab_xyz, self.fk_tab_rpy):
            lbl.setStyleSheet("font-family:monospace;font-size:11px;")
            fl.addWidget(lbl)
        vl.addWidget(fk_box)

        # IK 输入
        ik_box = QGroupBox("逆解 — 输入目标末端位置 (mm)")
        ikl = QVBoxLayout(ik_box)

        hint = QLabel(
            "可达范围参考：X ±380 mm  Y −20~400 mm  Z 100~640 mm\n"
            "（基于关节限位正解推算，超出范围将提示不可达）"
        )
        hint.setStyleSheet("color:#888;font-size:10px;")
        hint.setWordWrap(True)
        ikl.addWidget(hint)

        # XYZ 输入
        xyz_row = QHBoxLayout()
        self.ik_x = QDoubleSpinBox()
        self.ik_x.setRange(-450, 450)
        self.ik_x.setValue(0)
        self.ik_x.setDecimals(1)
        self.ik_x.setSuffix(" mm")
        self.ik_y = QDoubleSpinBox()
        self.ik_y.setRange(-450, 450)
        self.ik_y.setValue(350)
        self.ik_y.setDecimals(1)
        self.ik_y.setSuffix(" mm")
        self.ik_z = QDoubleSpinBox()
        self.ik_z.setRange(80, 680)
        self.ik_z.setValue(350)
        self.ik_z.setDecimals(1)
        self.ik_z.setSuffix(" mm")
        for label_text, sp in [("X:", self.ik_x), ("Y:", self.ik_y), ("Z:", self.ik_z)]:
            xyz_row.addWidget(QLabel(label_text))
            xyz_row.addWidget(sp)
        ikl.addLayout(xyz_row)

        # 快速预设
        pre_row = QHBoxLayout()
        presets = [
            ("正前", 0, 350, 350),
            ("左侧", -280, 200, 300),
            ("右侧", 280, 200, 300),
            ("高位", 0, 60, 600),
            ("折叠", 13, 186, 147),
        ]
        for name, x, y, z in presets:
            b = QPushButton(name)
            b.setMaximumWidth(52)
            b.setStyleSheet("padding:3px;")
            b.clicked.connect(
                lambda _, xx=x, yy=y, zz=z: self._ik_set_preset(xx, yy, zz)
            )
            pre_row.addWidget(b)
        pre_row.addStretch()
        ikl.addLayout(pre_row)

        # 计算按钮
        self.ik_compute_btn = QPushButton("🧮  计算逆解")
        self.ik_compute_btn.setStyleSheet(
            "background:#2196F3;color:white;padding:7px;border-radius:3px;"
        )
        self.ik_compute_btn.clicked.connect(self._ik_compute)
        ikl.addWidget(self.ik_compute_btn)

        # 结果标签
        self.ik_result_label = QLabel("结果：待计算")
        self.ik_result_label.setWordWrap(True)
        self.ik_result_label.setStyleSheet(
            "font-family:monospace;font-size:10px;"
            "background:#f5f5f5;padding:6px;border-radius:3px;"
        )
        self.ik_result_label.setMinimumHeight(120)
        ikl.addWidget(self.ik_result_label)

        # 操作按钮
        act_row = QHBoxLayout()
        self.ik_preview_btn = QPushButton("👁 仿真预览")
        self.ik_preview_btn.setEnabled(False)
        self.ik_preview_btn.clicked.connect(self._ik_preview)
        act_row.addWidget(self.ik_preview_btn)

        self.ik_apply_btn = QPushButton("📌 应用到控制")
        self.ik_apply_btn.setEnabled(False)
        self.ik_apply_btn.clicked.connect(self._ik_apply)
        act_row.addWidget(self.ik_apply_btn)

        self.ik_exec_btn = QPushButton("▶ 发送到真机")
        self.ik_exec_btn.setEnabled(False)
        self.ik_exec_btn.setStyleSheet("background:#4CAF50;color:white;padding:5px;")
        self.ik_exec_btn.clicked.connect(self._ik_execute)
        act_row.addWidget(self.ik_exec_btn)
        ikl.addLayout(act_row)

        vl.addWidget(ik_box)

        demo_box = QGroupBox("定位精度演示 — 鸡头稳定")
        dl = QVBoxLayout(demo_box)

        demo_hint = QLabel(
            "自动挑选一个安全可达点位，然后生成一段“末端位置基本不动、手臂姿态在变化”的演示轨迹。\n"
            "当前版本会让小臂/腕部动作更明显，并用更长时长与更低发送频率来兼顾真机跟随和 CAN 安全。"
        )
        demo_hint.setWordWrap(True)
        demo_hint.setStyleSheet("color:#666;font-size:10px;")
        dl.addWidget(demo_hint)

        self.stability_demo_label = QLabel("状态：待生成")
        self.stability_demo_label.setWordWrap(True)
        self.stability_demo_label.setMinimumHeight(92)
        self.stability_demo_label.setStyleSheet(
            "font-family:monospace;font-size:10px;"
            "background:#f5f5f5;padding:6px;border-radius:3px;"
        )
        dl.addWidget(self.stability_demo_label)

        demo_row = QHBoxLayout()
        self.stability_demo_generate_btn = QPushButton("🧠 生成演示")
        self.stability_demo_generate_btn.setStyleSheet(
            "background:#607D8B;color:white;padding:6px;border-radius:3px;"
        )
        self.stability_demo_generate_btn.clicked.connect(self._generate_stability_demo)
        demo_row.addWidget(self.stability_demo_generate_btn)

        self.stability_demo_preview_btn = QPushButton("👁 预览")
        self.stability_demo_preview_btn.setEnabled(False)
        self.stability_demo_preview_btn.clicked.connect(self._preview_stability_demo)
        demo_row.addWidget(self.stability_demo_preview_btn)

        self.stability_demo_exec_btn = QPushButton("▶ 发送到真机")
        self.stability_demo_exec_btn.setEnabled(False)
        self.stability_demo_exec_btn.setStyleSheet(
            "background:#4CAF50;color:white;padding:6px;border-radius:3px;"
        )
        self.stability_demo_exec_btn.clicked.connect(self._execute_stability_demo)
        demo_row.addWidget(self.stability_demo_exec_btn)
        dl.addLayout(demo_row)

        vl.addWidget(demo_box)
        vl.addStretch()
        return w

    # ── 选项卡 1：示教（真机） ────────────────────────────────────────────────

    def _build_tab_teach(self) -> QWidget:
        w  = QWidget()
        vl = QVBoxLayout(w)
        vl.setSpacing(8)

        if not TEACH_AVAILABLE:
            vl.addWidget(QLabel("示教模块不可用\n（需 core/teach_mode.py 和 gui/teach_dialog.py）"))
            return w

        # 示教模式
        mode_box = QGroupBox("示教模式（真机）")
        ml = QVBoxLayout(mode_box)

        mode_info = QLabel(
            "进入示教模式后，机械臂电机禁用，可轻松拖动关节。\n"
            "退出时自动读取当前位置并立即保持（防止跳位）。"
        )
        mode_info.setStyleSheet("color:#666;font-size:10px;")
        mode_info.setWordWrap(True)
        ml.addWidget(mode_info)

        self.teach_mode_btn = QPushButton("🖐  进入示教模式")
        self.teach_mode_btn.setStyleSheet(
            "QPushButton{background:#9C27B0;color:white;"
            "padding:10px;border-radius:4px;}"
        )
        self.teach_mode_btn.clicked.connect(self._teach_toggle_mode)
        ml.addWidget(self.teach_mode_btn)

        self.teach_status_lbl = QLabel("状态：空闲")
        self.teach_status_lbl.setStyleSheet("color:#888;")
        ml.addWidget(self.teach_status_lbl)
        vl.addWidget(mode_box)

        # 轨迹录制
        rec_box = QGroupBox("轨迹录制（拖动臂时录制真机位置）")
        rl = QVBoxLayout(rec_box)

        self.teach_rec_btn = QPushButton("🔴  开始录制")
        self.teach_rec_btn.clicked.connect(self._teach_toggle_record)
        self.teach_rec_btn.setEnabled(False)
        rl.addWidget(self.teach_rec_btn)

        self.teach_count_lbl = QLabel("已录制：0 点")
        self.teach_count_lbl.setStyleSheet("color:#555;font-size:10px;")
        rl.addWidget(self.teach_count_lbl)
        vl.addWidget(rec_box)

        # GUI 实时跟随录制
        gui_box = QGroupBox("GUI 实时跟随录制（左侧控件）")
        gl = QVBoxLayout(gui_box)

        gui_hint = QLabel(
            "开启后，拖动左侧关节控件会以 10 Hz 左右持续发送到真机。\n"
            "录制保存时会同时写出 observation.state / action，格式为 x,y,z,rot6d,gripper(0/1)。"
        )
        gui_hint.setWordWrap(True)
        gui_hint.setStyleSheet("color:#666;font-size:10px;")
        gl.addWidget(gui_hint)

        row_live = QHBoxLayout()
        self.gui_live_follow_chk = QCheckBox("启用实时跟随")
        self.gui_live_follow_chk.stateChanged.connect(self._toggle_live_follow)
        row_live.addWidget(self.gui_live_follow_chk)
        row_live.addWidget(QLabel("采样:"))
        self.gui_live_interval_spin = QDoubleSpinBox()
        self.gui_live_interval_spin.setRange(0.05, 0.5)
        self.gui_live_interval_spin.setSingleStep(0.05)
        self.gui_live_interval_spin.setValue(0.1)
        self.gui_live_interval_spin.setSuffix(" s")
        self.gui_live_interval_spin.setMaximumWidth(84)
        row_live.addWidget(self.gui_live_interval_spin)
        row_live.addStretch()
        gl.addLayout(row_live)

        row_rec = QHBoxLayout()
        self.gui_live_rec_btn = QPushButton("🔴 录制 GUI 轨迹")
        self.gui_live_rec_btn.clicked.connect(self._gui_live_toggle_record)
        row_rec.addWidget(self.gui_live_rec_btn)
        self.gui_live_save_btn = QPushButton("💾 导出")
        self.gui_live_save_btn.clicked.connect(self._gui_live_save)
        self.gui_live_save_btn.setEnabled(False)
        row_rec.addWidget(self.gui_live_save_btn)
        row_rec.addStretch()
        gl.addLayout(row_rec)

        self.gui_live_status_lbl = QLabel("实时跟随：关闭")
        self.gui_live_status_lbl.setStyleSheet("color:#555;font-size:10px;")
        gl.addWidget(self.gui_live_status_lbl)
        self.gui_live_count_lbl = QLabel("GUI 轨迹：0 点")
        self.gui_live_count_lbl.setStyleSheet("color:#555;font-size:10px;")
        gl.addWidget(self.gui_live_count_lbl)
        vl.addWidget(gui_box)

        # 轨迹回放
        play_box = QGroupBox("轨迹回放到真机")
        pl = QVBoxLayout(play_box)

        pr = QHBoxLayout()
        self.teach_play_btn = QPushButton("▶  回放")
        self.teach_play_btn.clicked.connect(self._teach_playback)
        self.teach_play_btn.setEnabled(False)
        pr.addWidget(self.teach_play_btn)
        pr.addWidget(QLabel("速度×:"))
        self.teach_speed_spin = QSpinBox()
        self.teach_speed_spin.setRange(1, 5)
        self.teach_speed_spin.setValue(1)
        pr.addWidget(self.teach_speed_spin)
        pl.addLayout(pr)

        self.teach_play_progress = QProgressBar()
        self.teach_play_progress.setVisible(False)
        pl.addWidget(self.teach_play_progress)
        vl.addWidget(play_box)

        # 完整示教面板（保存/加载/平滑）
        adv = QPushButton("🔧  打开完整示教面板（保存 / 加载 / 平滑）")
        adv.clicked.connect(self._open_teach_dialog)
        vl.addWidget(adv)

        vl.addStretch()
        return w

    # ── 选项卡 2：仿真轨迹规划 ────────────────────────────────────────────────

    def _build_tab_sim_traj(self) -> QWidget:
        w  = QWidget()
        vl = QVBoxLayout(w)
        vl.setSpacing(5)

        hint = QLabel(
            "① 拖动下方滑块 → 蓝色影子实时跟随（主视图始终为真机）\n"
            "② 「开始录制」后拖动各关节，系统按 10 Hz 录制连续轨迹\n"
            "③ 先看下方安全检查，再决定直接发送当前规划姿态或整条规划轨迹\n"
            "④ 轨迹发送按时间节奏重现，防止 CAN 总线丢步"
        )
        hint.setStyleSheet(
            "color:#555;font-size:10px;background:#f0f4ff;"
            "padding:5px;border-radius:3px;"
        )
        vl.addWidget(hint)

        # 同步按钮
        sync_row = QHBoxLayout()
        sync_btn = QPushButton("↩  从真机同步当前位置到规划滑块")
        sync_btn.clicked.connect(self._sim_sync_from_robot)
        sync_row.addWidget(sync_btn)
        sync_row.addStretch()

        # 影子开关
        self.ghost_cb = QCheckBox("显示规划影子")
        self.ghost_cb.setChecked(True)
        self.ghost_cb.stateChanged.connect(self._ghost_cb_changed)
        sync_row.addWidget(self.ghost_cb)
        vl.addLayout(sync_row)

        # 仿真姿态滑块（控制影子）
        sliders_box = QGroupBox("仿真姿态控制（影子跟随）")
        sbl = QVBoxLayout(sliders_box)
        sbl.setSpacing(2)
        self.sim_sliders:    List[QSlider] = []
        self.sim_val_labels: List[QLabel]  = []
        REST = list(RESET_POSE)
        for i in range(6):
            row = QFrame()
            rl2 = QHBoxLayout(row)
            rl2.setContentsMargins(2, 0, 2, 0)
            lbl = QLabel(f"J{i+1}")
            lbl.setMinimumWidth(22)
            lbl.setStyleSheet("font-weight:bold;font-size:9px;")
            rl2.addWidget(lbl)
            lo, hi = JOINT_LIMITS[i]
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(lo, hi)
            sl.setValue(REST[i])
            rl2.addWidget(sl)
            self.sim_sliders.append(sl)
            val = QLabel(self._format_joint_angle_label(i, REST[i]))
            val.setMinimumWidth(64 if i == 5 else 38)
            val.setStyleSheet("color:#1565C0;font-size:9px;")
            rl2.addWidget(val)
            self.sim_val_labels.append(val)
            sl.valueChanged.connect(
                lambda v, idx=i: self._sim_slider_changed(idx, v)
            )
            sbl.addWidget(row)
        vl.addWidget(sliders_box)

        sim_grip_box = QGroupBox("仿真夹爪状态")
        sgl = QHBoxLayout(sim_grip_box)
        sgl.addWidget(QLabel("规划:"))
        self.sim_gripper_state_lbl = QLabel("夹紧")
        self.sim_gripper_state_lbl.setMinimumWidth(46)
        self.sim_gripper_state_lbl.setStyleSheet("color:#607D8B;font-weight:bold;font-size:10px;")
        sgl.addWidget(self.sim_gripper_state_lbl)
        sim_open_btn = QPushButton("打开")
        sim_open_btn.clicked.connect(lambda: self._set_sim_gripper(GRIPPER_OPEN))
        sgl.addWidget(sim_open_btn)
        sim_close_btn = QPushButton("夹紧")
        sim_close_btn.clicked.connect(lambda: self._set_sim_gripper(GRIPPER_CLOSED))
        sgl.addWidget(sim_close_btn)
        sgl.addStretch()
        vl.addWidget(sim_grip_box)

        # 录制控制
        rec_box = QGroupBox("仿真轨迹录制（10 Hz）")
        rbl = QVBoxLayout(rec_box)
        rec_ctrl = QHBoxLayout()
        self.sim_rec_btn = QPushButton("🔴  开始录制")
        self.sim_rec_btn.setStyleSheet("padding:6px;")
        self.sim_rec_btn.clicked.connect(self._sim_toggle_record)
        rec_ctrl.addWidget(self.sim_rec_btn)
        save_btn = QPushButton("💾  保存")
        save_btn.clicked.connect(self._sim_save_traj)
        rec_ctrl.addWidget(save_btn)
        load_btn = QPushButton("📂  加载")
        load_btn.clicked.connect(self._sim_load_traj)
        rec_ctrl.addWidget(load_btn)
        clr_btn = QPushButton("🗑  清空")
        clr_btn.clicked.connect(self._sim_clear_traj)
        rec_ctrl.addWidget(clr_btn)
        rbl.addLayout(rec_ctrl)
        self.sim_traj_info = QLabel("轨迹：空")
        self.sim_traj_info.setStyleSheet("color:#888;font-size:10px;")
        rbl.addWidget(self.sim_traj_info)
        vl.addWidget(rec_box)

        # 预览 & 执行
        pe_box = QGroupBox("预览 & 发送到真机")
        pel = QVBoxLayout(pe_box)

        self.sim_safety_lbl = QLabel("当前规划安全：待评估")
        self.sim_safety_lbl.setWordWrap(True)
        self.sim_safety_lbl.setStyleSheet(
            "font-size:10px;background:#eef3f8;padding:6px;border-radius:4px;color:#425466;"
        )
        pel.addWidget(self.sim_safety_lbl)

        pe_row1 = QHBoxLayout()
        self.sim_prev_btn = QPushButton("🔍  预览轨迹")
        self.sim_prev_btn.clicked.connect(self._sim_toggle_preview)
        self.sim_prev_btn.setEnabled(False)
        pe_row1.addWidget(self.sim_prev_btn)
        pe_row1.addWidget(QLabel("速度×:"))
        self.sim_speed_spin = QDoubleSpinBox()
        self.sim_speed_spin.setRange(0.1, 5.0)
        self.sim_speed_spin.setValue(1.0)
        self.sim_speed_spin.setSingleStep(0.1)
        self.sim_speed_spin.setDecimals(1)
        self.sim_speed_spin.setMaximumWidth(68)
        pe_row1.addWidget(self.sim_speed_spin)
        pe_row1.addStretch()
        pel.addLayout(pe_row1)

        self.sim_progress = QProgressBar()
        self.sim_progress.setVisible(False)
        pel.addWidget(self.sim_progress)

        self.sim_pose_exec_btn = QPushButton("▶  发送当前规划姿态到真机")
        self.sim_pose_exec_btn.setStyleSheet("""
            QPushButton{background:#607D8B;color:white;
                        font-size:12px;padding:8px;border-radius:4px;}
            QPushButton:hover{background:#546E7A;}
        """)
        self.sim_pose_exec_btn.clicked.connect(self._sim_execute_current_pose)
        pel.addWidget(self.sim_pose_exec_btn)

        self.sim_exec_btn = QPushButton("▶  发送规划轨迹到真机执行")
        self.sim_exec_btn.setStyleSheet("""
            QPushButton{background:#4CAF50;color:white;
                        font-size:13px;padding:9px;border-radius:4px;}
            QPushButton:hover{background:#388E3C;}
            QPushButton:disabled{background:#aaa;}
        """)
        self.sim_exec_btn.clicked.connect(self._sim_execute)
        self.sim_exec_btn.setEnabled(False)
        pel.addWidget(self.sim_exec_btn)

        self.sim_stop_btn = QPushButton("⏹  中止执行")
        self.sim_stop_btn.setStyleSheet(
            "background:#F44336;color:white;padding:8px;border-radius:4px;"
        )
        self.sim_stop_btn.setVisible(False)
        self.sim_stop_btn.clicked.connect(self._sim_stop_execution)
        pel.addWidget(self.sim_stop_btn)

        vl.addWidget(pe_box)
        vl.addStretch()
        return w

    def _build_tab_safety_boundary(self) -> QWidget:
        w = QWidget()
        vl = QVBoxLayout(w)
        vl.setSpacing(6)

        tip = QLabel(
            "这里设置的是你自己的软边界，不是固件硬限位。先把机械臂移动到允许空间的边缘，"
            "再用下方按钮记录当前关节角或末端世界坐标；保存后只有勾选启用的边界会拦截动作。"
        )
        tip.setWordWrap(True)
        tip.setStyleSheet(
            "color:#555;font-size:10px;background:#eef3f8;padding:6px;border-radius:4px;"
        )
        vl.addWidget(tip)

        mode_box = QGroupBox("运行开关")
        ml = QVBoxLayout(mode_box)
        self.mujoco_guard_chk = QCheckBox("启用旧 MuJoCo 碰台/基座/工作盒检查")
        self.mujoco_guard_chk.setChecked(self._mujoco_guard_enabled)
        self.mujoco_guard_chk.stateChanged.connect(self._toggle_mujoco_guard)
        ml.addWidget(self.mujoco_guard_chk)
        self.user_boundary_enabled_chk = QCheckBox("启用用户软边界拦截")
        self.user_boundary_enabled_chk.setChecked(bool(self._world_settings.get("user_boundary_enabled", False)))
        self.user_boundary_enabled_chk.stateChanged.connect(lambda _: self._refresh_safety_boundary_status())
        ml.addWidget(self.user_boundary_enabled_chk)

        ratio_row = QHBoxLayout()
        ratio_row.addWidget(QLabel("J6固件比:"))
        self.safety_j6_firmware_reduction_spin = QDoubleSpinBox()
        self.safety_j6_firmware_reduction_spin.setRange(1.0, 200.0)
        self.safety_j6_firmware_reduction_spin.setDecimals(2)
        self.safety_j6_firmware_reduction_spin.setSingleStep(1.0)
        self.safety_j6_firmware_reduction_spin.setValue(self._j6_fw_ratio())
        self.safety_j6_firmware_reduction_spin.setMaximumWidth(82)
        self.safety_j6_firmware_reduction_spin.valueChanged.connect(self._on_j6_firmware_reduction_changed)
        ratio_row.addWidget(self.safety_j6_firmware_reduction_spin)
        ratio_row.addWidget(QLabel("实际比:"))
        self.safety_j6_reduction_spin = QDoubleSpinBox()
        self.safety_j6_reduction_spin.setRange(1.0, 200.0)
        self.safety_j6_reduction_spin.setDecimals(2)
        self.safety_j6_reduction_spin.setSingleStep(1.0)
        self.safety_j6_reduction_spin.setValue(self._j6_ratio())
        self.safety_j6_reduction_spin.setMaximumWidth(90)
        self.safety_j6_reduction_spin.valueChanged.connect(self._on_j6_output_reduction_changed)
        ratio_row.addWidget(self.safety_j6_reduction_spin)
        ratio_row.addWidget(QLabel("输出角 = 固件角 × 固件比 / 实际比"))
        ratio_row.addStretch()
        ml.addLayout(ratio_row)
        vl.addWidget(mode_box)

        joint_box = QGroupBox("关节软边界（固件角度）")
        jl = QVBoxLayout(joint_box)
        self.joint_boundary_enabled_chk = QCheckBox("启用关节角边界")
        self.joint_boundary_enabled_chk.setChecked(bool(self._world_settings.get("joint_boundary_enabled", False)))
        self.joint_boundary_enabled_chk.stateChanged.connect(lambda _: self._refresh_safety_boundary_status())
        jl.addWidget(self.joint_boundary_enabled_chk)

        grid = QGridLayout()
        grid.addWidget(QLabel("关节"), 0, 0)
        grid.addWidget(QLabel("下界"), 0, 1)
        grid.addWidget(QLabel("上界"), 0, 2)
        self.joint_boundary_min_spins: List[QDoubleSpinBox] = []
        self.joint_boundary_max_spins: List[QDoubleSpinBox] = []
        for i, name in enumerate(JOINT_NAMES):
            lo, hi = JOINT_LIMITS[i]
            grid.addWidget(QLabel(name), i + 1, 0)
            min_spin = QDoubleSpinBox()
            max_spin = QDoubleSpinBox()
            for spin in (min_spin, max_spin):
                spin.setRange(float(lo), float(hi))
                spin.setDecimals(1)
                spin.setSingleStep(1.0)
                spin.setSuffix(" °")
            grid.addWidget(min_spin, i + 1, 1)
            grid.addWidget(max_spin, i + 1, 2)
            self.joint_boundary_min_spins.append(min_spin)
            self.joint_boundary_max_spins.append(max_spin)
        jl.addLayout(grid)

        joint_btn_row = QHBoxLayout()
        joint_min_btn = QPushButton("当前关节设为下界")
        joint_min_btn.clicked.connect(lambda: self._capture_current_joints_to_boundary("min"))
        joint_btn_row.addWidget(joint_min_btn)
        joint_max_btn = QPushButton("当前关节设为上界")
        joint_max_btn.clicked.connect(lambda: self._capture_current_joints_to_boundary("max"))
        joint_btn_row.addWidget(joint_max_btn)
        joint_expand_btn = QPushButton("扩展包含当前位置")
        joint_expand_btn.clicked.connect(lambda: self._capture_current_joints_to_boundary("expand"))
        joint_btn_row.addWidget(joint_expand_btn)
        joint_btn_row.addStretch()
        jl.addLayout(joint_btn_row)
        vl.addWidget(joint_box)

        world_box = QGroupBox("末端世界坐标软边界")
        wl = QVBoxLayout(world_box)
        self.world_boundary_enabled_chk = QCheckBox("启用末端世界坐标盒")
        self.world_boundary_enabled_chk.setChecked(bool(self._world_settings.get("world_boundary_enabled", False)))
        self.world_boundary_enabled_chk.stateChanged.connect(lambda _: self._refresh_safety_boundary_status())
        wl.addWidget(self.world_boundary_enabled_chk)

        ws_grid = QGridLayout()
        ws_grid.addWidget(QLabel("轴"), 0, 0)
        ws_grid.addWidget(QLabel("下界"), 0, 1)
        ws_grid.addWidget(QLabel("上界"), 0, 2)
        self.boundary_world_min_spins: Dict[str, QDoubleSpinBox] = {}
        self.boundary_world_max_spins: Dict[str, QDoubleSpinBox] = {}
        for row, axis in enumerate(("x_mm", "y_mm", "z_mm"), start=1):
            axis_label = {"x_mm": "X", "y_mm": "Y", "z_mm": "Z"}[axis]
            ws_grid.addWidget(QLabel(axis_label), row, 0)
            min_spin = QDoubleSpinBox()
            max_spin = QDoubleSpinBox()
            for spin in (min_spin, max_spin):
                spin.setRange(-1500.0, 1500.0)
                spin.setDecimals(1)
                spin.setSingleStep(5.0)
                spin.setSuffix(" mm")
            ws_grid.addWidget(min_spin, row, 1)
            ws_grid.addWidget(max_spin, row, 2)
            self.boundary_world_min_spins[axis] = min_spin
            self.boundary_world_max_spins[axis] = max_spin
        wl.addLayout(ws_grid)

        world_btn_row = QHBoxLayout()
        world_min_btn = QPushButton("当前末端设为下角")
        world_min_btn.clicked.connect(lambda: self._capture_current_world_to_boundary("min"))
        world_btn_row.addWidget(world_min_btn)
        world_max_btn = QPushButton("当前末端设为上角")
        world_max_btn.clicked.connect(lambda: self._capture_current_world_to_boundary("max"))
        world_btn_row.addWidget(world_max_btn)
        world_expand_btn = QPushButton("扩展包含当前末端")
        world_expand_btn.clicked.connect(lambda: self._capture_current_world_to_boundary("expand"))
        world_btn_row.addWidget(world_expand_btn)
        world_btn_row.addStretch()
        wl.addLayout(world_btn_row)
        vl.addWidget(world_box)

        action_row = QHBoxLayout()
        save_btn = QPushButton("保存并应用")
        save_btn.clicked.connect(self._save_safety_boundary_from_ui)
        action_row.addWidget(save_btn)
        reset_btn = QPushButton("恢复固件限位")
        reset_btn.clicked.connect(self._reset_safety_boundary_to_firmware_limits)
        action_row.addWidget(reset_btn)
        action_row.addStretch()
        vl.addLayout(action_row)

        self.safety_boundary_status_lbl = QLabel("用户边界：--")
        self.safety_boundary_status_lbl.setWordWrap(True)
        self.safety_boundary_status_lbl.setStyleSheet(
            "color:#555;font-size:10px;background:#f7f7f7;padding:6px;border-radius:4px;"
        )
        vl.addWidget(self.safety_boundary_status_lbl)
        vl.addStretch()

        self._populate_safety_boundary_ui()
        return w

    def _build_tab_light_ring(self) -> QWidget:
        w = QWidget()
        vl = QVBoxLayout(w)
        vl.setSpacing(8)

        tip = QLabel(
            "灯环通过 REF 原生 USB 接口控制，不走当前 ASCII 串口。\n"
            "当前控制面板使用莫兰迪风格配色：烟粉红 / 鼠尾草绿 / 雾霾蓝。\n"
            "亮度档位为 0~45；若当前板子还是旧 REF 固件，亮度和真正全灭会在重刷后完全生效。"
        )
        tip.setWordWrap(True)
        tip.setStyleSheet(
            "color:#555;font-size:10px;background:#eef1ee;"
            "padding:6px;border-radius:4px;"
        )
        vl.addWidget(tip)

        status_box = QGroupBox("灯环状态")
        sl = QVBoxLayout(status_box)
        self.light_link_status_lbl = QLabel("接口：未探测")
        self.light_auto_state_lbl = QLabel("自动状态：--")
        self.light_last_mode_lbl = QLabel("当前灯效：--")
        self.light_brightness_lbl = QLabel("当前亮度：--")
        self.light_error_lbl = QLabel("最近错误：--")
        self.light_error_lbl.setWordWrap(True)
        for lbl in (
            self.light_link_status_lbl,
            self.light_auto_state_lbl,
            self.light_last_mode_lbl,
            self.light_brightness_lbl,
            self.light_error_lbl,
        ):
            lbl.setStyleSheet("font-size:11px;")
            sl.addWidget(lbl)
        vl.addWidget(status_box)

        cfg_box = QGroupBox("状态映射")
        gl = QGridLayout(cfg_box)
        gl.addWidget(QLabel("状态"), 0, 0)
        gl.addWidget(QLabel("灯效 / 颜色"), 0, 1)

        self.light_enabled_chk = QCheckBox("启用灯环自动联动")
        gl.addWidget(self.light_enabled_chk, 1, 0, 1, 2)

        gl.addWidget(QLabel("亮度"), 2, 0)
        brightness_row = QHBoxLayout()
        self.light_brightness_slider = QSlider(Qt.Orientation.Horizontal)
        self.light_brightness_slider.setRange(0, 45)
        self.light_brightness_slider.setValue(24)
        self.light_brightness_slider.valueChanged.connect(self._update_light_brightness_label)
        brightness_row.addWidget(self.light_brightness_slider)
        self.light_brightness_value_lbl = QLabel("24 / 45")
        self.light_brightness_value_lbl.setMinimumWidth(52)
        brightness_row.addWidget(self.light_brightness_value_lbl)
        gl.addLayout(brightness_row, 2, 1)

        self.light_state_combos = {}
        row = 3
        for state_key, state_label in LIGHT_RING_STATE_LABELS.items():
            gl.addWidget(QLabel(state_label), row, 0)
            combo = QComboBox()
            for mode_key, mode_cfg in LIGHT_RING_MODES.items():
                combo.addItem(mode_cfg["label"], mode_key)
            self.light_state_combos[state_key] = combo
            gl.addWidget(combo, row, 1)
            row += 1
        vl.addWidget(cfg_box)

        preview_box = QGroupBox("预览与保存")
        pl = QVBoxLayout(preview_box)

        preview_row = QHBoxLayout()
        preview_row.addWidget(QLabel("预览状态:"))
        self.light_preview_state_combo = QComboBox()
        for state_key, state_label in LIGHT_RING_STATE_LABELS.items():
            self.light_preview_state_combo.addItem(state_label, state_key)
        preview_row.addWidget(self.light_preview_state_combo)
        preview_btn = QPushButton("预览")
        preview_btn.setStyleSheet("background:#8ea3b0;color:white;padding:5px;")
        preview_btn.clicked.connect(self._preview_light_state)
        preview_row.addWidget(preview_btn)
        restore_btn = QPushButton("恢复自动")
        restore_btn.setStyleSheet("background:#a6a39a;color:white;padding:5px;")
        restore_btn.clicked.connect(self._restore_light_auto_state)
        preview_row.addWidget(restore_btn)
        pl.addLayout(preview_row)

        palette_lbl = QLabel(
            "固定色预览：烟粉红 #927370  |  鼠尾草绿 #85937B  |  雾霾蓝 #748599"
        )
        palette_lbl.setStyleSheet("color:#666;font-size:10px;")
        palette_lbl.setWordWrap(True)
        pl.addWidget(palette_lbl)

        save_btn = QPushButton("保存并应用")
        save_btn.setStyleSheet("background:#85937b;color:white;padding:7px;")
        save_btn.clicked.connect(self._save_light_settings)
        pl.addWidget(save_btn)

        vl.addWidget(preview_box)
        vl.addStretch()
        return w

    # ══════════════════════════════════════════════════════════════════════════
    # 串口 & 连接
    # ══════════════════════════════════════════════════════════════════════════

    def _auto_detect_port(self):
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if ports:
            self._log(f"检测到串口: {ports}")

    def _refresh_ports(self):
        self._auto_detect_port()

    def _toggle_connection(self):
        if self.robot.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "错误", "请选择串口")
            return
        if self.robot.connect(port):
            self.connect_btn.setText("断开")
            self.connect_btn.setStyleSheet("background:#F44336;color:white;")
            self.status_label.setText(f"● 已连接 {port}")
            self.status_label.setStyleSheet("color:green;font-weight:bold;")
            self._log(f"已连接: {port}")
            self.update_thread = UpdateThread(self.robot)
            self.update_thread.position_updated.connect(self._update_position_display)
            self.update_thread.start()
            self._refresh_direct_target_preview()
        else:
            QMessageBox.critical(self, "错误", "连接失败")

    def _disconnect(self):
        if self.update_thread:
            self.update_thread.stop()
            self.update_thread.wait()
        self.robot.disconnect()
        self.connect_btn.setText("连接")
        self.connect_btn.setStyleSheet("background:#4CAF50;color:white;")
        self.status_label.setText("● 未连接")
        self.status_label.setStyleSheet("color:red;font-weight:bold;")
        self._log("已断开")
        if self.mujoco_widget:
            self.mujoco_widget.clear_target_ghost()
        self._refresh_direct_target_preview()
        self._refresh_light_panel()

    # ══════════════════════════════════════════════════════════════════════════
    # 电机 & 运动
    # ══════════════════════════════════════════════════════════════════════════

    def _toggle_enable(self):
        if not self.robot.connected:
            return
        if self.robot.enabled:
            self.robot.disable()
            self.enable_btn.setText("🔴 使能")
            self._log("电机已禁用")
        else:
            if self.robot.enable():
                self.enable_btn.setText("🟢 禁用")
                self._log("电机已使能")

    def _emergency_stop(self):
        self.robot.emergency_stop()
        self._log("🛑 急停")

    def _load_pose(self, angles: list, name: str):
        self._suppress_live_follow = True
        try:
            for i, a in enumerate(angles):
                self.joint_sliders[i].setValue(int(a))
        finally:
            self._suppress_live_follow = False
        self.gui_live_follow_dirty = False
        if self.mujoco_widget:
            self.mujoco_widget.update_joint_angles(
                self._mujoco_visual_angles(angles),
                self._gripper_target_value,
            )
            self.mujoco_widget.snap_to_angles()
        self._refresh_direct_target_preview()
        self._log(f"已加载预设: {name}")
        if QMessageBox.question(
            self, "确认", f"立即运动到 {name}？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        ) == QMessageBox.StandardButton.Yes:
            self._execute_move()

    def _on_slider_change(self, idx: int, val: int):
        self.joint_tgt_labels[idx].setText(self._format_joint_angle_label(idx, val))
        # 未连接时，左侧滑块直接驱动主视图
        if self.mujoco_widget and not self.robot.connected:
            angles = [s.value() for s in self.joint_sliders]
            self.mujoco_widget.update_joint_angles(
                self._mujoco_visual_angles(angles),
                self._gripper_target_value,
            )
        self._refresh_direct_target_preview()
        if not self._suppress_live_follow:
            self.gui_live_follow_dirty = True

    def _set_joint_zero(self, idx: int):
        self.joint_sliders[idx].setValue(0)

    def _ensure_robot_ready(self) -> bool:
        """确保真机已连接并使能。"""
        if not self.robot.connected:
            QMessageBox.warning(self, "错误", "请先连接串口")
            return False
        if not self.robot.enabled:
            if QMessageBox.question(
                self, "确认", "电机未使能，是否自动使能？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            ) == QMessageBox.StandardButton.Yes:
                if not self.robot.enable():
                    QMessageBox.warning(self, "错误", "电机使能失败")
                    return False
                self.enable_btn.setText("🟢 禁用")
            else:
                return False
        return True

    def _analyze_pose_safety(self, angles: List[float]) -> Dict[str, Any]:
        if not self.mujoco_widget:
            return {
                "available": False,
                "safe": True,
                "issues": [],
                "summary": "MuJoCo 不可用，跳过安全检查",
            }
        report = self.mujoco_widget.analyze_pose_safety(self._mujoco_visual_angles(angles))
        if not self._mujoco_guard_enabled:
            report = dict(report)
            report["safe"] = True
            report["issues"] = []
            report["summary"] = "旧 MuJoCo 安全检查已关闭"
            report["guard_disabled"] = True
        return report

    def _format_pose_safety_message(self, report: Dict[str, Any], context: str) -> str:
        if not report.get("available", True):
            return f"{context}：{report.get('summary', 'MuJoCo 不可用，跳过安全检查')}"

        lines = [f"{context} 未通过安全检查："]
        for issue in report.get("issues", [])[:4]:
            lines.append(f"• {issue}")

        tcp_mm = report.get("tcp_mm")
        if tcp_mm and len(tcp_mm) >= 3:
            lines.append("")
            lines.append(
                f"参考：TCP = ({tcp_mm[0]:.1f}, {tcp_mm[1]:.1f}, {tcp_mm[2]:.1f}) mm"
            )

        floor_min = report.get("floor_min_z_mm")
        if isinstance(floor_min, (int, float)):
            lines.append(f"最低关键连杆离工作面 = {floor_min:.1f} mm")

        return "\n".join(lines)

    def _check_pose_safety(
        self,
        angles: List[float],
        context: str,
        show_dialog: bool = True,
    ) -> bool:
        report = self._analyze_pose_safety(angles)
        if report.get("safe", True):
            return True

        message = self._format_pose_safety_message(report, context)
        self._log(f"⚠️ {context}未通过安全检查：{report.get('summary', '存在碰撞风险')}")
        if show_dialog:
            QMessageBox.warning(self, "安全检查未通过", message)
        return False

    def _analyze_trajectory_safety(self, trajectory: List[TrajectoryPoint]) -> Dict[str, Any]:
        if not trajectory or not self.mujoco_widget:
            return {
                "available": bool(self.mujoco_widget),
                "safe": True,
                "issues": [],
                "summary": "没有轨迹或 MuJoCo 不可用，跳过轨迹安全检查",
            }
        if not self._mujoco_guard_enabled:
            return {
                "available": True,
                "safe": True,
                "issues": [],
                "summary": "旧 MuJoCo 安全检查已关闭",
                "guard_disabled": True,
            }

        issues = []
        worst_tcp_z = float("inf")
        worst_arm_z = float("inf")

        for idx, point in enumerate(trajectory):
            angles = point.playback_angles() if hasattr(point, "playback_angles") else point.angles
            report = self._analyze_pose_safety(angles)
            tcp_mm = report.get("tcp_mm") or [0.0, 0.0, float("inf")]
            worst_tcp_z = min(worst_tcp_z, float(tcp_mm[2]))
            floor_min = report.get("floor_min_z_mm")
            if isinstance(floor_min, (int, float)):
                worst_arm_z = min(worst_arm_z, float(floor_min))
            if not report.get("safe", True):
                issues.append({
                    "index": idx,
                    "timestamp": float(getattr(point, "timestamp", idx)),
                    "issues": list(report.get("issues", [])),
                })

        summary = "安全" if not issues else f"共 {len(issues)} 个轨迹点存在碰撞风险"
        return {
            "available": True,
            "safe": len(issues) == 0,
            "issues": issues,
            "summary": summary,
            "worst_tcp_z_mm": worst_tcp_z,
            "worst_arm_z_mm": worst_arm_z,
        }

    def _format_trajectory_safety_message(self, report: Dict[str, Any]) -> str:
        lines = [f"规划轨迹未通过安全检查：{report.get('summary', '存在风险')}"]
        for item in report.get("issues", [])[:5]:
            lines.append(
                f"• 第 {item['index'] + 1} 点 / t={item['timestamp']:.1f}s："
                + "；".join(item.get("issues", [])[:3])
            )

        worst_tcp = report.get("worst_tcp_z_mm")
        worst_arm = report.get("worst_arm_z_mm")
        if isinstance(worst_tcp, (int, float)) or isinstance(worst_arm, (int, float)):
            lines.append("")
        if isinstance(worst_tcp, (int, float)):
            lines.append(f"最低 TCP 世界坐标 Z = {worst_tcp:.1f} mm")
        if isinstance(worst_arm, (int, float)) and worst_arm < float("inf"):
            lines.append(f"最低关键连杆离工作面 = {worst_arm:.1f} mm")
        return "\n".join(lines)

    def _send_angles_to_robot(self, angles: List[float], speed: int, context: str):
        if not self._ensure_robot_ready():
            return False, "未就绪"
        if not self._check_pose_safety(angles, context=context, show_dialog=True):
            return False, "未通过安全检查"
        if not self._check_user_boundary(angles, context=context, show_dialog=True):
            return False, "超出用户安全边界"
        return self.robot.move_to(angles, speed)

    def _set_direct_preview_status(self, text: str, tone: str = "neutral"):
        if not hasattr(self, "direct_preview_status_lbl"):
            return
        palette = {
            "neutral": ("#fff4eb", "#8a5a21"),
            "safe": ("#eef6ea", "#2f6f3d"),
            "warn": ("#fff1e6", "#a05a00"),
            "muted": ("#eef3f8", "#425466"),
        }
        bg, fg = palette.get(tone, palette["neutral"])
        self.direct_preview_status_lbl.setText(text)
        self.direct_preview_status_lbl.setStyleSheet(
            f"font-size:10px;background:{bg};padding:6px;border-radius:4px;color:{fg};"
        )

    def _refresh_direct_target_preview(self):
        if not hasattr(self, "direct_preview_chk"):
            return

        target = [s.value() for s in self.joint_sliders] if hasattr(self, "joint_sliders") else []
        if len(target) != 6:
            self._set_direct_preview_status("目标影子：待评估", tone="neutral")
            return

        report = self._analyze_pose_safety(target)
        tcp_mm = report.get("tcp_mm") or [0.0, 0.0, 0.0]
        safe_suffix = (
            f"TCP Z={tcp_mm[2]:.1f} mm"
            if report.get("available", True)
            else "MuJoCo 不可用"
        )

        if not self.mujoco_widget:
            self._set_direct_preview_status(f"目标影子：{safe_suffix}", tone="muted")
            return

        if not self.direct_preview_chk.isChecked():
            self.mujoco_widget.clear_target_ghost()
            tone = "warn" if not report.get("safe", True) else "muted"
            text = (
                "目标影子：已关闭"
                if report.get("safe", True)
                else "目标影子：已关闭，但当前目标姿态存在风险"
            )
            if not report.get("safe", True) and report.get("issues"):
                text += f" | {report['issues'][0]}"
            self._set_direct_preview_status(text, tone=tone)
            return

        if not self.robot.connected:
            self.mujoco_widget.clear_target_ghost()
            text = f"目标影子：未连接时主视图已直接跟随目标 | {safe_suffix}"
            if not report.get("safe", True) and report.get("issues"):
                text = f"目标姿态存在风险 | {report['issues'][0]}"
                self._set_direct_preview_status(text, tone="warn")
            else:
                self._set_direct_preview_status(text, tone="muted")
            return

        current = list(self.robot.current_angles[:6])
        max_diff = max(abs(a - b) for a, b in zip(target, current))
        if max_diff <= self.DIRECT_PREVIEW_DIFF_DEG:
            self.mujoco_widget.clear_target_ghost()
            if report.get("safe", True):
                self._set_direct_preview_status(
                    f"目标影子：当前姿态已接近目标 | {safe_suffix}",
                    tone="safe",
                )
            else:
                self._set_direct_preview_status(
                    f"目标姿态存在风险 | {report['issues'][0]}",
                    tone="warn",
                )
            return

        self.mujoco_widget.set_target_ghost(
            self._mujoco_visual_angles(target),
            self._gripper_target_value,
        )
        if report.get("safe", True):
            self._set_direct_preview_status(
                f"目标影子已显示 | 与当前最大差值 {max_diff:.1f}° | {safe_suffix}",
                tone="safe",
            )
        else:
            self._set_direct_preview_status(
                f"目标影子已显示，但当前目标姿态存在风险 | {report['issues'][0]}",
                tone="warn",
            )

    def _toggle_direct_target_preview(self, state: int):
        if self.mujoco_widget and state == 0:
            self.mujoco_widget.clear_target_ghost()
        self._refresh_direct_target_preview()

    def _refresh_sim_safety_status(self):
        if not hasattr(self, "sim_safety_lbl"):
            return

        angles = [s.value() for s in getattr(self, "sim_sliders", [])] if hasattr(self, "sim_sliders") else []
        if len(angles) != 6:
            self.sim_safety_lbl.setText("当前规划安全：待评估")
            return

        report = self._analyze_pose_safety(angles)
        if report.get("guard_disabled"):
            tcp_mm = report.get("tcp_mm") or [0.0, 0.0, 0.0]
            self.sim_safety_lbl.setText(
                f"当前规划安全：旧 MuJoCo 检查已关闭 | TCP Z={tcp_mm[2]:.1f} mm"
            )
            self.sim_safety_lbl.setStyleSheet(
                "font-size:10px;background:#eef3f8;padding:6px;border-radius:4px;color:#425466;"
            )
            return
        if not report.get("available", True):
            self.sim_safety_lbl.setText("当前规划安全：MuJoCo 不可用，无法评估")
            self.sim_safety_lbl.setStyleSheet(
                "font-size:10px;background:#eef3f8;padding:6px;border-radius:4px;color:#425466;"
            )
            return

        tcp_mm = report.get("tcp_mm") or [0.0, 0.0, 0.0]
        floor_min = report.get("floor_min_z_mm", 0.0)
        if report.get("safe", True):
            self.sim_safety_lbl.setText(
                f"当前规划安全：通过 | TCP Z={tcp_mm[2]:.1f} mm | 最低关键连杆离工作面={floor_min:.1f} mm"
            )
            self.sim_safety_lbl.setStyleSheet(
                "font-size:10px;background:#eaf4ea;padding:6px;border-radius:4px;color:#1f6f43;"
            )
        else:
            self.sim_safety_lbl.setText(
                "当前规划安全：存在风险 | "
                + "；".join(report.get("issues", [])[:2])
            )
            self.sim_safety_lbl.setStyleSheet(
                "font-size:10px;background:#fff4e8;padding:6px;border-radius:4px;color:#9a5a00;"
            )

    def _execute_move(self):
        target = [s.value() for s in self.joint_sliders]
        ok, msg = self._send_angles_to_robot(
            target,
            self.speed_spin.value(),
            context="左侧关节控制目标姿态",
        )
        if ok:
            self._log(f"运动 → {target}")
        else:
            if msg not in ("未通过安全检查", "超出用户安全边界"):
                QMessageBox.warning(self, "错误", msg)

    def _apply_gripper_settings_from_ui(self, *_):
        if not hasattr(self, "gripper_open_current_spin"):
            return
        self.robot.configure_gripper(
            open_current_a=self.gripper_open_current_spin.value(),
            close_current_a=self.gripper_close_current_spin.value(),
            hold_current_a=self.gripper_hold_current_spin.value(),
            open_pulse_s=self.gripper_open_pulse_spin.value(),
            close_pulse_s=self.gripper_close_pulse_spin.value(),
            hold_after_close=self.gripper_hold_chk.isChecked(),
            max_hold_s=self.gripper_hold_timeout_spin.value(),
        )

    def _refresh_gripper_ui_state(self):
        if not hasattr(self, "gripper_state_lbl"):
            return

        value = normalize_gripper_value(self._gripper_target_value)
        self.gripper_state_lbl.setText(gripper_state_label(value))
        if value >= 0.5:
            self.gripper_state_lbl.setStyleSheet("color:#1565C0;font-weight:bold;font-size:11px;")
        else:
            self.gripper_state_lbl.setStyleSheet("color:#607D8B;font-weight:bold;font-size:11px;")

        snapshot = self.robot.get_gripper_status_snapshot()
        remaining = snapshot.get("hold_remaining_s")
        if snapshot.get("holding") and isinstance(remaining, (int, float)):
            hold_text = f"保持：{remaining:.0f}s"
        elif snapshot.get("placeholder_mode"):
            hold_text = "保持：占位"
        else:
            hold_text = "保持：未保持"
        self.gripper_hold_lbl.setText(hold_text)

        if self.mujoco_widget:
            self.mujoco_widget.update_gripper(value)

    def _toggle_gripper_placeholder_mode(self, state: int):
        ok, msg = self.robot.set_gripper_placeholder_mode(bool(state))
        self._refresh_gripper_mode_label()
        self._log(msg if ok else f"⚠️ {msg}")

    def _refresh_gripper_mode_label(self):
        if not hasattr(self, "gripper_mode_lbl"):
            return
        if self.robot.is_gripper_placeholder_mode():
            self.gripper_mode_lbl.setText("当前：占位记录模式，导出 gripper=0/1，不下发硬件")
        else:
            self.gripper_mode_lbl.setText("当前：真实硬件模式，使用 !HAND_* ASCII 指令控制线轨夹爪")

    def _set_gripper_buttons_busy(self, busy: bool):
        for btn_name in ("gripper_open_btn", "gripper_close_btn", "gripper_disable_btn"):
            btn = getattr(self, btn_name, None)
            if btn is not None:
                btn.setEnabled(not busy)

    def _command_gripper(self, target_value: float):
        if self._gripper_action_thread and self._gripper_action_thread.isRunning():
            self._log("夹爪动作仍在执行，已忽略本次重复指令")
            return
        if not self.robot.is_gripper_placeholder_mode() and not self.robot.connected:
            QMessageBox.warning(self, "错误", "真实硬件模式需要先连接机械臂串口")
            return

        self._apply_gripper_settings_from_ui()
        self._gripper_target_value = normalize_gripper_value(target_value)
        self.robot.target_gripper = self._gripper_target_value
        self._refresh_gripper_ui_state()
        self._set_gripper_buttons_busy(True)

        self._gripper_action_thread = GripperActionThread(
            self.robot,
            target_value=self._gripper_target_value,
        )
        self._gripper_action_thread.action_finished.connect(self._on_gripper_action_finished)
        self._gripper_action_thread.start()

    def _disable_gripper_from_ui(self):
        if self._gripper_action_thread and self._gripper_action_thread.isRunning():
            self._log("夹爪动作仍在执行，等待动作结束后再断使能")
            return
        if self.robot.is_gripper_placeholder_mode():
            self._log("夹爪处于占位模式，无硬件使能需要断开")
            return
        if not self.robot.connected:
            QMessageBox.warning(self, "错误", "请先连接机械臂串口")
            return

        self._set_gripper_buttons_busy(True)
        self._gripper_action_thread = GripperActionThread(self.robot, disable=True)
        self._gripper_action_thread.action_finished.connect(self._on_gripper_action_finished)
        self._gripper_action_thread.start()

    def _on_gripper_action_finished(self, ok: bool, msg: str, target_value: float):
        self._set_gripper_buttons_busy(False)
        self._gripper_action_thread = None
        if ok:
            self._gripper_target_value = normalize_gripper_value(target_value)
            self._live_last_sent_gripper = self._gripper_target_value
            self._log(msg)
        else:
            self._log(f"⚠️ {msg}")
            QMessageBox.warning(self, "夹爪控制失败", msg)
        self._refresh_gripper_ui_state()

    def _toggle_live_follow(self, state: int):
        self.gui_live_follow_enabled = bool(state)
        self.gui_live_follow_dirty = self.gui_live_follow_enabled
        self._live_last_sent_angles = None
        self._live_last_sent_gripper = None

        if self.gui_live_follow_enabled:
            self._log("GUI 实时跟随已开启，拖动左侧关节控件即可驱动真机")
        else:
            self._log("GUI 实时跟随已关闭")

        self._update_live_follow_status_label()

    def _update_live_follow_status_label(self):
        if not hasattr(self, "gui_live_status_lbl"):
            return

        if not self.gui_live_follow_enabled:
            status = "实时跟随：关闭"
            color = "#555"
        elif not self.robot.connected:
            status = "实时跟随：等待串口连接"
            color = "#C77700"
        elif not self.robot.enabled:
            status = "实时跟随：等待电机使能"
            color = "#C77700"
        else:
            status = "实时跟随：已启用"
            color = "#2E7D32"

        if self.gui_live_recording:
            status += " | 录制中"

        self.gui_live_status_lbl.setText(status)
        self.gui_live_status_lbl.setStyleSheet(f"color:{color};font-size:10px;")

    def _live_follow_tick(self):
        if not self.gui_live_follow_enabled or not self.gui_live_follow_dirty:
            return
        if self._suppress_live_follow:
            return
        if not self.robot.connected or not self.robot.enabled:
            self._update_live_follow_status_label()
            return

        target_angles = [s.value() for s in self.joint_sliders]
        target_gripper = normalize_gripper_value(self._gripper_target_value)

        same_angles = self._live_last_sent_angles == target_angles
        same_gripper = (
            self._live_last_sent_gripper is not None
            and abs(self._live_last_sent_gripper - target_gripper) < 1e-6
        )
        if same_angles and same_gripper:
            self.gui_live_follow_dirty = False
            return

        if not self._check_user_boundary(target_angles, "GUI 实时跟随", show_dialog=False):
            self.gui_live_follow_dirty = False
            return

        ok, msg = self.robot.move_to(target_angles, self.speed_spin.value())
        if not ok:
            self._log(f"⚠️ 实时跟随发送失败: {msg}")
            self.gui_live_follow_dirty = False
            return

        grip_changed = (
            self._live_last_sent_gripper is None
            or abs(self._live_last_sent_gripper - target_gripper) >= 1e-6
        )
        if grip_changed:
            ok_g, msg_g = self.robot.set_gripper_state(target_gripper)
            if not ok_g:
                self._log(f"⚠️ 夹爪发送失败: {msg_g}")

        self._live_last_sent_angles = list(target_angles)
        self._live_last_sent_gripper = target_gripper
        self.gui_live_follow_dirty = False
        self._update_live_follow_status_label()

    def _current_gui_action_payload(self):
        return {
            "angles": [s.value() for s in self.joint_sliders],
            "gripper": normalize_gripper_value(self._gripper_target_value),
        }

    def _gui_live_toggle_record(self):
        if not self.gui_live_recording:
            self._gui_live_start_record()
        else:
            self._gui_live_stop_record()

    def _gui_live_start_record(self):
        if not TEACH_AVAILABLE or not self.gui_live_recorder:
            QMessageBox.warning(self, "错误", "示教/轨迹模块不可用")
            return
        if not self.robot.connected:
            QMessageBox.warning(self, "错误", "请先连接机械臂")
            return
        if self.teach_in_mode:
            QMessageBox.warning(self, "错误", "请先退出拖动示教模式，再进行 GUI 实时录制")
            return
        if not self.robot.enabled:
            if QMessageBox.question(
                self, "确认", "电机未使能，是否自动使能后开始 GUI 录制？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            ) == QMessageBox.StandardButton.No:
                return
            if not self.robot.enable():
                QMessageBox.warning(self, "错误", "电机使能失败")
                return
            self.enable_btn.setText("🟢 禁用")

        if not self.gui_live_follow_enabled:
            self.gui_live_follow_chk.setChecked(True)

        self.gui_live_recorder.sample_interval = self.gui_live_interval_spin.value()
        self.gui_live_recorder.start_recording(
            action_provider=self._current_gui_action_payload,
            source="gui_live",
        )
        self.gui_live_recording = True
        self.robot.set_runtime_state("recording", True)
        self.gui_live_rec_btn.setText("⏹ 停止 GUI 录制")
        self.gui_live_rec_btn.setStyleSheet("background:#F44336;color:white;padding:6px;")
        self.gui_live_save_btn.setEnabled(False)
        self.gui_live_trajectory = []
        self.gui_live_count_lbl.setText("GUI 轨迹：录制中 0 点")
        self._update_live_follow_status_label()
        self._log("🔴 开始录制 GUI 实时跟随轨迹")

    def _gui_live_stop_record(self):
        if not self.gui_live_recorder:
            return
        self.gui_live_trajectory = self.gui_live_recorder.stop_recording()
        self.gui_live_recording = False
        self.robot.set_runtime_state("recording", False)
        self.gui_live_rec_btn.setText("🔴 录制 GUI 轨迹")
        self.gui_live_rec_btn.setStyleSheet("")
        self.gui_live_save_btn.setEnabled(len(self.gui_live_trajectory) > 0)
        n = len(self.gui_live_trajectory)
        dur = self.gui_live_trajectory[-1].timestamp if n > 0 else 0.0
        self.gui_live_count_lbl.setText(f"GUI 轨迹：{n} 点 | 时长 {dur:.1f}s")
        self._update_live_follow_status_label()
        self._log(f"⏹ 停止 GUI 录制，共 {n} 点，{dur:.1f}s")

    def _gui_live_save(self):
        if not self.gui_live_trajectory or not self.gui_live_recorder:
            QMessageBox.information(self, "提示", "当前没有可导出的 GUI 轨迹")
            return

        path, selected = QFileDialog.getSaveFileName(
            self,
            "导出 GUI 实时轨迹",
            os.path.join(_PROJECT_DIR, "gui_live_trajectory.json"),
            "JSON Files (*.json);;CSV Files (*.csv)",
        )
        if not path:
            return

        fmt = 'csv' if selected.startswith("CSV") or path.lower().endswith(".csv") else 'json'
        if fmt == 'json' and not path.lower().endswith(".json"):
            path += ".json"
        if fmt == 'csv' and not path.lower().endswith(".csv"):
            path += ".csv"

        metadata = {
            "source": "gui_live",
            "record_hz": round(1.0 / max(self.gui_live_interval_spin.value(), 1e-6), 2),
            "control_mode": "joint_slider_live_follow",
            "state_action_layout": "ee6d_absolute",
            "gripper_encoding": "0.0=closed/clamped, 1.0=open",
        }
        self.gui_live_recorder.save_trajectory(
            self.gui_live_trajectory,
            path,
            format=fmt,
            metadata=metadata,
        )
        self._log(f"💾 GUI 实时轨迹已导出: {os.path.basename(path)}")

    # ══════════════════════════════════════════════════════════════════════════
    # MuJoCo 同步
    # ══════════════════════════════════════════════════════════════════════════

    def _sync_mujoco(self):
        """
        50ms 定时器：
        - 已连接真机 → 主视图跟随真机位置
        - 未连接       → 主视图跟随左侧关节滑块（自由预览模式）
        蓝色影子由仿真规划驱动，暖色影子由左侧直接控制目标驱动。
        """
        if not self.mujoco_widget:
            return
        if self.robot.connected:
            self.mujoco_widget.update_joint_angles(
                self._mujoco_visual_angles(self.robot.current_angles),
                self.robot.get_gripper(),
            )
        self._refresh_direct_target_preview()
        # 未连接时保持上次滑块设定值（_on_slider_change 已处理）

    def _update_position_display(self, angles: list):
        """真机位置更新回调：刷新实际角度标签"""
        for i, a in enumerate(angles[:6]):
            self.joint_cur_labels[i].setText(self._format_joint_angle_label(i, float(a)))
        self._refresh_direct_target_preview()

    def _update_status(self):
        if self.robot.connected:
            txt   = "● 已连接 | " + ("已使能" if self.robot.enabled else "未使能")
            color = "green" if self.robot.enabled else "orange"
            self.status_label.setText(txt)
            self.status_label.setStyleSheet(f"color:{color};font-weight:bold;")
        else:
            self.status_label.setText("● 未连接")
            self.status_label.setStyleSheet("color:red;font-weight:bold;")
        self._update_live_follow_status_label()
        self._refresh_light_panel()
        self._refresh_gripper_ui_state()

    def _update_ee_label(self, pose: list):
        x, y, z, ro, pi, ya = pose
        txt_xyz = f"位置: X={x:.1f}  Y={y:.1f}  Z={z:.1f} mm"
        txt_rpy = f"姿态: R={ro:.1f}°  P={pi:.1f}°  Y={ya:.1f}°"
        self.ee_label.setText(f"末端: X={x:.1f}  Y={y:.1f}  Z={z:.1f} mm | "
                              f"R={ro:.1f}° P={pi:.1f}° Y={ya:.1f}°")
        self.fk_xyz_label.setText(txt_xyz)
        self.fk_rpy_label.setText(txt_rpy)
        self.fk_tab_xyz.setText(txt_xyz)
        self.fk_tab_rpy.setText(txt_rpy)

    def _set_view(self, view: str):
        if self.mujoco_widget:
            self.mujoco_widget.set_camera_view(view)

    def _refresh_tcp_offset_status(self):
        if not hasattr(self, "tcp_offset_status_lbl"):
            return
        current = float(self._tcp_settings["tcp_offset_mm"])
        self.tcp_offset_status_lbl.setText(
            f"当前 TCP 偏移：{current:.1f} mm（沿 J6 前向）"
        )

    def _refresh_world_settings_status(self):
        if not hasattr(self, "world_status_lbl"):
            return
        ws = self._world_settings
        box = "开启" if ws["workspace_box_enabled"] else "关闭"
        self.world_status_lbl.setText(
            "世界标定："
            f"原点=({ws['origin_x_mm']:.1f}, {ws['origin_y_mm']:.1f}, {ws['origin_z_mm']:.1f}) mm | "
            f"Yaw={ws['yaw_deg']:.1f}° | 工作面Z={ws['work_surface_z_mm']:.1f} mm | "
            f"空间盒={box}"
        )

    def _collect_world_settings_from_ui(self) -> Dict[str, object]:
        settings = dict(self._world_settings)
        settings.update({
            "origin_x_mm": self.world_origin_x_spin.value(),
            "origin_y_mm": self.world_origin_y_spin.value(),
            "origin_z_mm": self.world_origin_z_spin.value(),
            "yaw_deg": self.world_yaw_spin.value(),
            "work_surface_z_mm": self.work_surface_z_spin.value(),
            "min_tcp_clearance_mm": self.min_tcp_clearance_spin.value(),
            "min_arm_clearance_mm": self.min_arm_clearance_spin.value(),
            "base_guard_radius_mm": self.base_guard_radius_spin.value(),
            "base_guard_height_mm": self.base_guard_height_spin.value(),
            "workspace_box_enabled": self.workspace_enabled_chk.isChecked(),
            "workspace_min": {
                "x_mm": self.ws_min_x_spin.value(),
                "y_mm": self.ws_min_y_spin.value(),
                "z_mm": self.ws_min_z_spin.value(),
            },
            "workspace_max": {
                "x_mm": self.ws_max_x_spin.value(),
                "y_mm": self.ws_max_y_spin.value(),
                "z_mm": self.ws_max_z_spin.value(),
            },
        })
        return normalize_world_settings(settings)

    def _apply_world_settings_to_runtime(self, settings: Dict[str, object], persist: bool = False) -> str:
        self._world_settings = normalize_world_settings(settings)
        self._mujoco_guard_enabled = bool(self._world_settings.get("mujoco_guard_enabled", False))
        self._j6_firmware_reduction = float(
            self._world_settings.get("j6_firmware_reduction", J6_FIRMWARE_REDUCTION)
        )
        self._j6_output_reduction = float(
            self._world_settings.get("j6_output_reduction", J6_OUTPUT_REDUCTION_DEFAULT)
        )

        if self.mujoco_widget:
            self.mujoco_widget.set_world_settings(self._world_settings)

        for teach_mode in (
            self._sim_traj_io,
            self.gui_live_recorder,
            getattr(self, "_tm", None),
        ):
            if teach_mode:
                teach_mode.set_world_settings(self._world_settings)

        if persist:
            path = save_world_settings(self._world_settings)
            msg = f"世界标定已保存到 {path}"
        else:
            msg = "世界标定已应用"

        self._sync_settings_widgets_from_runtime()
        self._refresh_world_settings_status()
        self._refresh_safety_boundary_status()
        self._refresh_joint_angle_labels()
        self._refresh_sim_safety_status()
        self._refresh_direct_target_preview()
        return msg

    def _sync_settings_widgets_from_runtime(self):
        for spin_name in ("j6_firmware_reduction_spin", "safety_j6_firmware_reduction_spin"):
            spin = getattr(self, spin_name, None)
            if spin is None:
                continue
            old = spin.blockSignals(True)
            spin.setValue(self._j6_fw_ratio())
            spin.blockSignals(old)

        for spin_name in ("j6_reduction_spin", "safety_j6_reduction_spin"):
            spin = getattr(self, spin_name, None)
            if spin is None:
                continue
            old = spin.blockSignals(True)
            spin.setValue(self._j6_ratio())
            spin.blockSignals(old)

        if hasattr(self, "mujoco_guard_chk"):
            old = self.mujoco_guard_chk.blockSignals(True)
            self.mujoco_guard_chk.setChecked(self._mujoco_guard_enabled)
            self.mujoco_guard_chk.blockSignals(old)

    def _populate_world_settings_ui(self):
        if not hasattr(self, "world_origin_x_spin"):
            return
        ws = self._world_settings
        self.world_origin_x_spin.setValue(float(ws["origin_x_mm"]))
        self.world_origin_y_spin.setValue(float(ws["origin_y_mm"]))
        self.world_origin_z_spin.setValue(float(ws["origin_z_mm"]))
        self.world_yaw_spin.setValue(float(ws["yaw_deg"]))
        self.work_surface_z_spin.setValue(float(ws["work_surface_z_mm"]))
        self.min_tcp_clearance_spin.setValue(float(ws["min_tcp_clearance_mm"]))
        self.min_arm_clearance_spin.setValue(float(ws["min_arm_clearance_mm"]))
        self.base_guard_radius_spin.setValue(float(ws["base_guard_radius_mm"]))
        self.base_guard_height_spin.setValue(float(ws["base_guard_height_mm"]))
        self.workspace_enabled_chk.setChecked(bool(ws["workspace_box_enabled"]))
        self.ws_min_x_spin.setValue(float(ws["workspace_min"]["x_mm"]))
        self.ws_min_y_spin.setValue(float(ws["workspace_min"]["y_mm"]))
        self.ws_min_z_spin.setValue(float(ws["workspace_min"]["z_mm"]))
        self.ws_max_x_spin.setValue(float(ws["workspace_max"]["x_mm"]))
        self.ws_max_y_spin.setValue(float(ws["workspace_max"]["y_mm"]))
        self.ws_max_z_spin.setValue(float(ws["workspace_max"]["z_mm"]))
        self._apply_world_settings_to_runtime(ws, persist=False)

    def _save_world_settings_from_ui(self):
        msg = self._apply_world_settings_to_runtime(
            self._collect_world_settings_from_ui(),
            persist=True,
        )
        self._log(msg)

    def _populate_safety_boundary_ui(self):
        if not hasattr(self, "joint_boundary_min_spins"):
            return
        ws = self._world_settings
        if hasattr(self, "mujoco_guard_chk"):
            self.mujoco_guard_chk.setChecked(bool(ws.get("mujoco_guard_enabled", False)))
        self.user_boundary_enabled_chk.setChecked(bool(ws.get("user_boundary_enabled", False)))
        self.joint_boundary_enabled_chk.setChecked(bool(ws.get("joint_boundary_enabled", False)))
        self.world_boundary_enabled_chk.setChecked(bool(ws.get("world_boundary_enabled", False)))

        joint_min = list(ws.get("joint_boundary_min", []))
        joint_max = list(ws.get("joint_boundary_max", []))
        for i, (min_spin, max_spin) in enumerate(
            zip(self.joint_boundary_min_spins, self.joint_boundary_max_spins)
        ):
            lo, hi = JOINT_LIMITS[i]
            min_spin.setValue(float(joint_min[i] if i < len(joint_min) else lo))
            max_spin.setValue(float(joint_max[i] if i < len(joint_max) else hi))

        world_min = ws.get("world_boundary_min", {})
        world_max = ws.get("world_boundary_max", {})
        for axis in ("x_mm", "y_mm", "z_mm"):
            self.boundary_world_min_spins[axis].setValue(float(world_min.get(axis, 0.0)))
            self.boundary_world_max_spins[axis].setValue(float(world_max.get(axis, 0.0)))

        self._sync_settings_widgets_from_runtime()
        self._refresh_safety_boundary_status()

    def _collect_safety_boundary_from_ui(self) -> Dict[str, object]:
        settings = dict(self._world_settings)
        settings.update({
            "mujoco_guard_enabled": self.mujoco_guard_chk.isChecked(),
            "user_boundary_enabled": self.user_boundary_enabled_chk.isChecked(),
            "joint_boundary_enabled": self.joint_boundary_enabled_chk.isChecked(),
            "joint_boundary_min": [spin.value() for spin in self.joint_boundary_min_spins],
            "joint_boundary_max": [spin.value() for spin in self.joint_boundary_max_spins],
            "world_boundary_enabled": self.world_boundary_enabled_chk.isChecked(),
            "world_boundary_min": {
                axis: self.boundary_world_min_spins[axis].value()
                for axis in ("x_mm", "y_mm", "z_mm")
            },
            "world_boundary_max": {
                axis: self.boundary_world_max_spins[axis].value()
                for axis in ("x_mm", "y_mm", "z_mm")
            },
            "j6_firmware_reduction": self._j6_fw_ratio(),
            "j6_output_reduction": self._j6_ratio(),
        })
        return normalize_world_settings(settings)

    def _save_safety_boundary_from_ui(self):
        msg = self._apply_world_settings_to_runtime(
            self._collect_safety_boundary_from_ui(),
            persist=True,
        )
        self._log(msg)

    def _reset_safety_boundary_to_firmware_limits(self):
        for i, (min_spin, max_spin) in enumerate(
            zip(self.joint_boundary_min_spins, self.joint_boundary_max_spins)
        ):
            lo, hi = JOINT_LIMITS[i]
            min_spin.setValue(float(lo))
            max_spin.setValue(float(hi))
        self.joint_boundary_enabled_chk.setChecked(False)
        self.world_boundary_enabled_chk.setChecked(False)
        self.user_boundary_enabled_chk.setChecked(False)
        self._refresh_safety_boundary_status()
        self._log("用户软边界已恢复为固件限位，尚未保存")

    def _current_joint_angles_for_boundary(self) -> List[float]:
        if self.robot.connected and any(self.robot.current_angles):
            return [float(a) for a in self.robot.current_angles[:6]]
        if hasattr(self, "joint_sliders"):
            return [float(s.value()) for s in self.joint_sliders[:6]]
        return list(RESET_POSE)

    def _capture_current_joints_to_boundary(self, mode: str):
        angles = self._current_joint_angles_for_boundary()
        for i, angle in enumerate(angles[:6]):
            if mode == "min":
                self.joint_boundary_min_spins[i].setValue(angle)
            elif mode == "max":
                self.joint_boundary_max_spins[i].setValue(angle)
            else:
                self.joint_boundary_min_spins[i].setValue(
                    min(self.joint_boundary_min_spins[i].value(), angle)
                )
                self.joint_boundary_max_spins[i].setValue(
                    max(self.joint_boundary_max_spins[i].value(), angle)
                )
        self.user_boundary_enabled_chk.setChecked(True)
        self.joint_boundary_enabled_chk.setChecked(True)
        self._refresh_safety_boundary_status()
        self._log(f"已记录当前关节到用户边界: {[round(a, 1) for a in angles]}")

    def _capture_current_world_to_boundary(self, mode: str):
        point = self._current_world_point()
        if not point:
            QMessageBox.warning(self, "提示", "当前没有可读取的末端世界坐标")
            return
        for axis, value in zip(("x_mm", "y_mm", "z_mm"), point[:3]):
            if mode == "min":
                self.boundary_world_min_spins[axis].setValue(float(value))
            elif mode == "max":
                self.boundary_world_max_spins[axis].setValue(float(value))
            else:
                self.boundary_world_min_spins[axis].setValue(
                    min(self.boundary_world_min_spins[axis].value(), float(value))
                )
                self.boundary_world_max_spins[axis].setValue(
                    max(self.boundary_world_max_spins[axis].value(), float(value))
                )
        self.user_boundary_enabled_chk.setChecked(True)
        self.world_boundary_enabled_chk.setChecked(True)
        self._refresh_safety_boundary_status()
        self._log(
            f"已记录当前末端到用户世界边界: X={point[0]:.1f}, Y={point[1]:.1f}, Z={point[2]:.1f} mm"
        )

    def _toggle_mujoco_guard(self, state: int):
        self._mujoco_guard_enabled = bool(state)
        self._world_settings["mujoco_guard_enabled"] = self._mujoco_guard_enabled
        self._refresh_safety_boundary_status()
        self._refresh_sim_safety_status()
        self._refresh_direct_target_preview()

    def _refresh_safety_boundary_status(self):
        if not hasattr(self, "safety_boundary_status_lbl"):
            return
        ws = self._world_settings
        user_enabled = (
            self.user_boundary_enabled_chk.isChecked()
            if hasattr(self, "user_boundary_enabled_chk")
            else bool(ws.get("user_boundary_enabled", False))
        )
        joint_enabled = (
            self.joint_boundary_enabled_chk.isChecked()
            if hasattr(self, "joint_boundary_enabled_chk")
            else bool(ws.get("joint_boundary_enabled", False))
        )
        world_enabled = (
            self.world_boundary_enabled_chk.isChecked()
            if hasattr(self, "world_boundary_enabled_chk")
            else bool(ws.get("world_boundary_enabled", False))
        )
        user = "开启" if user_enabled else "关闭"
        joint = "开启" if joint_enabled else "关闭"
        world = "开启" if world_enabled else "关闭"
        mujoco = "开启" if self._mujoco_guard_enabled else "关闭"
        self.safety_boundary_status_lbl.setText(
            f"用户边界={user} | 关节边界={joint} | 世界盒={world} | "
            f"旧MuJoCo检查={mujoco} | J6输出=固件×{self._j6_fw_ratio():.2f}/{self._j6_ratio():.2f}"
        )

    def _tcp_world_for_angles(self, angles: List[float]) -> Optional[List[float]]:
        if not self.mujoco_widget:
            return None
        try:
            report = self.mujoco_widget.analyze_pose_safety(self._mujoco_visual_angles(angles))
        except Exception as exc:
            self._log(f"⚠️ 计算末端世界坐标失败: {exc}")
            return None
        tcp = report.get("tcp_mm") if isinstance(report, dict) else None
        if tcp and len(tcp) >= 3:
            return [float(tcp[0]), float(tcp[1]), float(tcp[2])]
        return None

    def _user_boundary_issues(self, angles: List[float]) -> List[str]:
        ws = self._world_settings
        if not bool(ws.get("user_boundary_enabled", False)):
            return []

        issues: List[str] = []
        if bool(ws.get("joint_boundary_enabled", False)):
            joint_min = list(ws.get("joint_boundary_min", []))
            joint_max = list(ws.get("joint_boundary_max", []))
            for i, value in enumerate(angles[:6]):
                lo = float(joint_min[i]) if i < len(joint_min) else float(JOINT_LIMITS[i][0])
                hi = float(joint_max[i]) if i < len(joint_max) else float(JOINT_LIMITS[i][1])
                if value < lo or value > hi:
                    issues.append(f"J{i + 1}={value:.1f}° 超出用户关节边界 [{lo:.1f}, {hi:.1f}]")

        if bool(ws.get("world_boundary_enabled", False)):
            tcp = self._tcp_world_for_angles(angles)
            if tcp is None:
                issues.append("无法计算末端世界坐标，不能确认用户世界边界")
            else:
                world_min = ws.get("world_boundary_min", {})
                world_max = ws.get("world_boundary_max", {})
                for axis_idx, axis in enumerate(("x_mm", "y_mm", "z_mm")):
                    lo = float(world_min.get(axis, -float("inf")))
                    hi = float(world_max.get(axis, float("inf")))
                    value = float(tcp[axis_idx])
                    axis_name = axis[0].upper()
                    if value < lo or value > hi:
                        issues.append(
                            f"TCP {axis_name}={value:.1f} mm 超出用户世界边界 [{lo:.1f}, {hi:.1f}]"
                        )

        return issues

    def _check_user_boundary(
        self,
        angles: List[float],
        context: str,
        show_dialog: bool = True,
    ) -> bool:
        issues = self._user_boundary_issues(angles)
        if not issues:
            return True

        message = f"{context} 超出用户安全边界：\n" + "\n".join(f"• {item}" for item in issues[:6])
        self._log(f"⚠️ {context} 超出用户安全边界：{issues[0]}")
        if show_dialog:
            QMessageBox.warning(self, "用户安全边界拦截", message)
        return False

    def _current_world_point(self) -> Optional[List[float]]:
        if self.mujoco_widget:
            if self.robot.connected and any(self.robot.current_angles):
                self.mujoco_widget.update_joint_angles(
                    self._mujoco_visual_angles(self.robot.current_angles),
                    self.robot.get_gripper(),
                )
            elif hasattr(self, "joint_sliders"):
                self.mujoco_widget.update_joint_angles(
                    self._mujoco_visual_angles([s.value() for s in self.joint_sliders]),
                    self._gripper_target_value,
                )
            point = self.mujoco_widget.get_ee_position()
            if point:
                return point
        return None

    def _capture_current_world_point_to_workspace(self, bound: str):
        point = self._current_world_point()
        if not point:
            QMessageBox.warning(self, "提示", "当前没有可读取的末端世界坐标")
            return
        if bound == "min":
            self.ws_min_x_spin.setValue(point[0])
            self.ws_min_y_spin.setValue(point[1])
            self.ws_min_z_spin.setValue(point[2])
            self._log(
                f"已将当前末端写入工作空间下角: X={point[0]:.1f}, Y={point[1]:.1f}, Z={point[2]:.1f} mm"
            )
        else:
            self.ws_max_x_spin.setValue(point[0])
            self.ws_max_y_spin.setValue(point[1])
            self.ws_max_z_spin.setValue(point[2])
            self._log(
                f"已将当前末端写入工作空间上角: X={point[0]:.1f}, Y={point[1]:.1f}, Z={point[2]:.1f} mm"
            )

    def _apply_tcp_offset_to_runtime(self, offset_mm: float, persist: bool = False):
        self._tcp_settings = normalize_tcp_settings({"tcp_offset_mm": offset_mm})
        tcp_offset_mm = float(self._tcp_settings["tcp_offset_mm"])

        if self.mujoco_widget:
            self.mujoco_widget.set_tcp_offset_mm(tcp_offset_mm)

        for teach_mode in (
            self._sim_traj_io,
            self.gui_live_recorder,
            getattr(self, "_tm", None),
        ):
            if teach_mode:
                teach_mode.set_tcp_offset_mm(tcp_offset_mm)

        if persist:
            path = save_tcp_settings(self._tcp_settings)
            msg = f"TCP 偏移已保存到 {path}，当前为 {tcp_offset_mm:.1f} mm"
        else:
            msg = f"TCP 偏移已应用为 {tcp_offset_mm:.1f} mm"

        if hasattr(self, "tcp_offset_spin"):
            self.tcp_offset_spin.setValue(tcp_offset_mm)
        self._refresh_tcp_offset_status()

        if self.mujoco_widget:
            if self.robot.connected and any(self.robot.current_angles):
                self.mujoco_widget.update_joint_angles(
                    self._mujoco_visual_angles(self.robot.current_angles),
                    self.robot.get_gripper(),
                )
            elif hasattr(self, "joint_sliders"):
                self.mujoco_widget.update_joint_angles(
                    self._mujoco_visual_angles([s.value() for s in self.joint_sliders]),
                    self._gripper_target_value,
                )

        return msg

    def _populate_tcp_offset_ui(self):
        if hasattr(self, "tcp_offset_spin"):
            self.tcp_offset_spin.setValue(float(self._tcp_settings["tcp_offset_mm"]))
        self._apply_tcp_offset_to_runtime(self._tcp_settings["tcp_offset_mm"], persist=False)

    def _save_tcp_offset_from_ui(self):
        msg = self._apply_tcp_offset_to_runtime(self.tcp_offset_spin.value(), persist=True)
        self._log(msg)

    # ══════════════════════════════════════════════════════════════════════════
    # FK/IK 选项卡
    # ══════════════════════════════════════════════════════════════════════════

    def _ik_set_preset(self, x: float, y: float, z: float):
        self.ik_x.setValue(x)
        self.ik_y.setValue(y)
        self.ik_z.setValue(z)

    def _ik_compute(self):
        if not MUJOCO_AVAILABLE or not self.mujoco_widget:
            self.ik_result_label.setText("❌ MuJoCo 不可用，无法计算逆解")
            return

        target = [self.ik_x.value(), self.ik_y.value(), self.ik_z.value()]
        self.ik_compute_btn.setEnabled(False)
        self.ik_compute_btn.setText("计算中…")
        self.ik_result_label.setText(
            f"⏳ 正在计算逆解…\n目标：X={target[0]:.1f}  Y={target[1]:.1f}  Z={target[2]:.1f} mm"
        )
        for btn in (self.ik_preview_btn, self.ik_apply_btn, self.ik_exec_btn):
            btn.setEnabled(False)

        # 用当前真机角度（或左侧滑块）作为初始猜测，更快收敛
        if self.robot.connected and any(self.robot.current_angles):
            initial = list(self.robot.current_angles)
        else:
            initial = [s.value() for s in self.joint_sliders]

        self.ik_thread = IKWorkerThread(self.mujoco_widget, target, initial)
        self.ik_thread.solved.connect(self._ik_compute_done)
        self.ik_thread.failed.connect(self._ik_compute_failed)
        self.ik_thread.finished.connect(self._ik_thread_finished)
        self.ik_thread.start()

    def _ik_compute_done(self, result, target, ee):
        self.ik_compute_btn.setEnabled(True)
        self.ik_compute_btn.setText("🧮  计算逆解")

        if result is None:
            self.ik_result_label.setText(
                f"❌ 不可达\n"
                f"目标：X={target[0]:.1f}  Y={target[1]:.1f}  Z={target[2]:.1f} mm\n\n"
                "该点超出机构工作空间或关节限位，请调整目标坐标。"
            )
            self.ik_result_angles = None
            return

        self.ik_result_angles = list(result)
        angle_lines = "\n".join(
            [f"  J{i+1} = {a:.1f}°" for i, a in enumerate(self.ik_result_angles)]
        )
        ee = ee or target
        err = ((ee[0]-target[0])**2 + (ee[1]-target[1])**2 + (ee[2]-target[2])**2)**0.5
        self.ik_result_label.setText(
            f"✅ 逆解成功\n"
            f"目标：X={target[0]:.1f}  Y={target[1]:.1f}  Z={target[2]:.1f} mm\n"
            f"关节角：\n{angle_lines}\n"
            f"验证末端：X={ee[0]:.1f}  Y={ee[1]:.1f}  Z={ee[2]:.1f} mm\n"
            f"位置误差：{err:.2f} mm"
        )
        for btn in (self.ik_preview_btn, self.ik_apply_btn, self.ik_exec_btn):
            btn.setEnabled(True)
        if self.mujoco_widget:
            self.mujoco_widget.set_ghost(self._mujoco_visual_angles(self.ik_result_angles))
        self._log(f"IK 解：{[f'{a:.1f}' for a in self.ik_result_angles]}")

    def _ik_compute_failed(self, message: str):
        self.ik_compute_btn.setEnabled(True)
        self.ik_compute_btn.setText("🧮  计算逆解")
        self.ik_result_angles = None
        self.ik_result_label.setText(f"❌ 逆解计算失败\n{message}")
        self._log(f"IK 计算失败: {message}")

    def _ik_thread_finished(self):
        self.ik_thread = None

    def _ik_preview(self):
        if self.ik_result_angles and self.mujoco_widget:
            self.mujoco_widget.set_ghost(self._mujoco_visual_angles(self.ik_result_angles))
            self._log("IK 结果已在仿真影子中预览")

    def _ik_apply(self):
        if not self.ik_result_angles:
            return
        self._suppress_live_follow = True
        try:
            for i, a in enumerate(self.ik_result_angles[:6]):
                self.joint_sliders[i].setValue(int(a))
        finally:
            self._suppress_live_follow = False
        self.gui_live_follow_dirty = False
        self._log("IK 结果已应用到关节控制滑块")

    def _ik_execute(self):
        if not self.ik_result_angles:
            return
        ok, msg = self._send_angles_to_robot(
            self.ik_result_angles,
            self.speed_spin.value(),
            context="IK 规划结果",
        )
        if ok:
            self._log(f"✅ IK 运动已发送: {[f'{a:.1f}' for a in self.ik_result_angles]}")
        else:
            if msg not in ("未通过安全检查", "超出用户安全边界"):
                QMessageBox.warning(self, "错误", msg)

    def _get_ik_seed_angles(self) -> List[float]:
        if self.robot.connected and any(self.robot.current_angles):
            return list(self.robot.current_angles[:6])
        return [s.value() for s in self.joint_sliders]

    def _select_stability_demo_anchor(self, initial_angles: List[float]):
        best = None
        best_score = float("-inf")

        for point in self.STABILITY_DEMO_CANDIDATES:
            solution = self.mujoco_widget.compute_ik(
                point,
                initial_fw_deg=initial_angles,
                tolerance_mm=6.0,
            )
            if solution is None:
                continue

            min_margin = min(
                min(angle - lo, hi - angle)
                for angle, (lo, hi) in zip(solution, JOINT_LIMITS)
            )
            score = min_margin - 0.03 * abs(solution[0]) - 0.02 * abs(solution[3])
            if score > best_score:
                best_score = score
                best = (list(point), list(solution))

        return best

    def _build_stability_demo_trajectory(
        self, center_xyz: List[float], anchor_angles: List[float]
    ):
        if not self.mujoco_widget:
            return None, "MuJoCo 不可用"

        prev_angles = list(anchor_angles)
        joint_weights = list(self.STABILITY_DEMO_JOINT_WEIGHTS)
        max_err_mm = 0.0
        sum_err_mm = 0.0

        trajectory = [
            TrajectoryPoint(timestamp=0.0, angles=list(anchor_angles), source="stability_demo")
        ]
        current_t = self.STABILITY_DEMO_SETTLE_TIME
        total_steps = self.STABILITY_DEMO_POINTS_PER_LOOP * self.STABILITY_DEMO_LOOPS

        for step in range(total_steps):
            phase = 2.0 * np.pi * (step % self.STABILITY_DEMO_POINTS_PER_LOOP) / self.STABILITY_DEMO_POINTS_PER_LOOP
            preferred = list(anchor_angles)
            preferred[0] += self.STABILITY_DEMO_J1_AMPLITUDE * np.sin(phase)
            preferred[3] += self.STABILITY_DEMO_J4_AMPLITUDE * np.sin(phase)
            preferred[4] += self.STABILITY_DEMO_J5_AMPLITUDE * np.cos(phase)
            preferred[5] += self.STABILITY_DEMO_J6_AMPLITUDE * np.cos(phase)

            solution = self.mujoco_widget.compute_posture_biased_ik(
                center_xyz,
                preferred,
                initial_fw_deg=prev_angles,
                tolerance_mm=2.0,
                position_weight=28.0,
                joint_weights=joint_weights,
            )
            if solution is None:
                return None, f"第 {step + 1} 个演示点逆解失败"

            ee_xyz = self.mujoco_widget.get_last_ik_ee() or center_xyz
            err_mm = float(np.linalg.norm(np.array(ee_xyz) - np.array(center_xyz)))
            max_err_mm = max(max_err_mm, err_mm)
            sum_err_mm += err_mm

            trajectory.append(
                TrajectoryPoint(
                    timestamp=current_t,
                    angles=list(solution),
                    playback_speed=self.STABILITY_DEMO_JOINT_SPEED,
                    source="stability_demo",
                )
            )
            current_t += self.STABILITY_DEMO_DT
            prev_angles = list(solution)

        trajectory.append(
            TrajectoryPoint(
                timestamp=current_t,
                angles=list(anchor_angles),
                playback_speed=self.STABILITY_DEMO_JOINT_SPEED,
                source="stability_demo",
            )
        )
        current_t += self.STABILITY_DEMO_SETTLE_TIME
        trajectory.append(
            TrajectoryPoint(
                timestamp=current_t,
                angles=list(anchor_angles),
                playback_speed=self.STABILITY_DEMO_JOINT_SPEED,
                source="stability_demo",
            )
        )

        stats = {
            "max_err_mm": max_err_mm,
            "mean_err_mm": sum_err_mm / max(1, total_steps),
            "duration_s": current_t,
            "points": len(trajectory),
        }
        return trajectory, stats

    def _load_generated_demo_into_sim(self, trajectory: List[TrajectoryPoint]):
        self.sim_trajectory = trajectory
        self._sim_refresh_traj_state()

        first_angles = self.sim_trajectory[0].angles
        first_gripper = self.sim_trajectory[0].playback_gripper()
        for i, a in enumerate(first_angles[:6]):
            self.sim_sliders[i].setValue(int(round(a)))
        self._set_sim_gripper(first_gripper)

        self.ghost_cb.setChecked(True)
        if self.mujoco_widget:
            self.mujoco_widget.set_ghost(self._mujoco_visual_angles(first_angles), first_gripper)

    def _current_sim_is_stability_demo(self) -> bool:
        return bool(
            self.sim_trajectory
            and getattr(self.sim_trajectory[0], "source", "") == "stability_demo"
        )

    def _generate_stability_demo(self) -> bool:
        if not MUJOCO_AVAILABLE or not self.mujoco_widget:
            QMessageBox.warning(self, "错误", "MuJoCo 不可用，无法生成定位精度演示")
            return False

        if self.sim_preview_active:
            self._sim_stop_preview()

        seed_angles = self._get_ik_seed_angles()
        selected = self._select_stability_demo_anchor(seed_angles)
        if selected is None:
            self.stability_demo_label.setText(
                "❌ 未找到合适的可达中心点。\n"
                "可以先让机械臂回到 L-Pose 或 REST，再重新生成。"
            )
            self.stability_demo_preview_btn.setEnabled(False)
            self.stability_demo_exec_btn.setEnabled(False)
            self._log("⚠️ 定位精度演示生成失败：没有找到安全可达点")
            return False

        center_xyz, anchor_angles = selected
        trajectory, stats_or_error = self._build_stability_demo_trajectory(
            center_xyz, anchor_angles
        )
        if trajectory is None:
            self.stability_demo_label.setText(
                f"❌ 演示轨迹生成失败\n原因：{stats_or_error}"
            )
            self.stability_demo_preview_btn.setEnabled(False)
            self.stability_demo_exec_btn.setEnabled(False)
            self._log(f"⚠️ 定位精度演示生成失败：{stats_or_error}")
            return False

        stats = stats_or_error
        self.stability_demo_center_xyz = list(center_xyz)
        self.stability_demo_anchor_angles = list(anchor_angles)
        self._load_generated_demo_into_sim(trajectory)
        self.stability_demo_preview_btn.setEnabled(True)
        self.stability_demo_exec_btn.setEnabled(True)

        self.stability_demo_label.setText(
            "✅ 已生成鸡头稳定演示\n"
            f"中心点: X={center_xyz[0]:.1f}  Y={center_xyz[1]:.1f}  Z={center_xyz[2]:.1f} mm\n"
            f"轨迹: {stats['points']} 点  |  时长 {stats['duration_s']:.1f} s\n"
            f"MuJoCo 预计最大漂移: {stats['max_err_mm']:.2f} mm\n"
            f"MuJoCo 预计平均漂移: {stats['mean_err_mm']:.2f} mm\n"
            "幅度重点: 小臂/腕部增强版\n"
            f"演示发送将使用: ×{self.STABILITY_DEMO_EXEC_RATIO:.1f} / {self.STABILITY_DEMO_JOINT_SPEED}°/s"
        )
        self._log(
            "🧠 已生成定位精度演示："
            f"中心点 {[round(v, 1) for v in center_xyz]}，"
            f"{stats['points']} 点，预计最大漂移 {stats['max_err_mm']:.2f} mm"
        )
        return True

    def _preview_stability_demo(self):
        if not self._current_sim_is_stability_demo():
            if not self._generate_stability_demo():
                return

        self.ghost_cb.setChecked(True)
        if self.sim_preview_active:
            self._sim_stop_preview()
        self._sim_start_preview()
        self._log("👁 已开始预览定位精度演示")

    def _execute_stability_demo(self):
        if not self._current_sim_is_stability_demo():
            if not self._generate_stability_demo():
                return

        self._log(
            "▶ 准备发送定位精度演示到真机"
            f"（强制使用 ×{self.STABILITY_DEMO_EXEC_RATIO:.1f} 与 {self.STABILITY_DEMO_JOINT_SPEED}°/s，减小 CAN 与跟踪压力）"
        )
        self._sim_execute(
            speed_ratio_override=self.STABILITY_DEMO_EXEC_RATIO,
            joint_speed_override=self.STABILITY_DEMO_JOINT_SPEED,
            min_interval_override=self.STABILITY_DEMO_MIN_INTERVAL,
        )

    # ══════════════════════════════════════════════════════════════════════════
    # 仿真规划
    # ══════════════════════════════════════════════════════════════════════════

    def _ghost_cb_changed(self, state: int):
        if not self.mujoco_widget:
            return
        if state == 0:
            self.mujoco_widget.clear_ghost()
        else:
            angles = [s.value() for s in self.sim_sliders]
            self.mujoco_widget.set_ghost(self._mujoco_visual_angles(angles), self._sim_gripper_value)

    def _sim_slider_changed(self, idx: int, val: int):
        self.sim_val_labels[idx].setText(self._format_joint_angle_label(idx, val))
        if self.mujoco_widget and self.ghost_cb.isChecked():
            angles = [s.value() for s in self.sim_sliders]
            self.mujoco_widget.set_ghost(self._mujoco_visual_angles(angles), self._sim_gripper_value)
        self._refresh_sim_safety_status()

    def _set_sim_gripper(self, value: float):
        self._sim_gripper_value = normalize_gripper_value(value)
        if hasattr(self, "sim_gripper_state_lbl"):
            self.sim_gripper_state_lbl.setText(gripper_state_label(self._sim_gripper_value))
            color = "#1565C0" if self._sim_gripper_value >= 0.5 else "#607D8B"
            self.sim_gripper_state_lbl.setStyleSheet(
                f"color:{color};font-weight:bold;font-size:10px;"
            )
        if self.mujoco_widget and self.ghost_cb.isChecked():
            self.mujoco_widget.set_ghost(
                self._mujoco_visual_angles([s.value() for s in self.sim_sliders]),
                self._sim_gripper_value,
            )

    def _sim_sync_from_robot(self):
        """将真机当前位置同步到仿真规划滑块"""
        if self.robot.connected and any(self.robot.current_angles):
            angles = self.robot.current_angles
            gripper = self.robot.get_gripper()
        else:
            angles = [s.value() for s in self.joint_sliders]
            gripper = self._gripper_target_value
        for i, a in enumerate(angles[:6]):
            self.sim_sliders[i].setValue(int(a))
        self._set_sim_gripper(gripper)
        self._refresh_sim_safety_status()
        self._log("仿真规划滑块已同步为当前位置")

    def _sim_toggle_record(self):
        if not self.sim_recording:
            self._sim_start_record()
        else:
            self._sim_stop_record()

    def _sim_start_record(self):
        self.sim_trajectory = []
        self.sim_rec_start  = time.time()
        self.sim_recording  = True
        self.robot.set_runtime_state("recording", True)
        self.sim_rec_btn.setText("⏹  停止录制")
        self.sim_rec_btn.setStyleSheet(
            "background:#F44336;color:white;padding:6px;"
        )
        self.sim_traj_info.setText("录制中… 0 点")
        # 自动开启影子
        self.ghost_cb.setChecked(True)
        self.sim_rec_timer.start(100)   # 10 Hz
        self._log("🔴 开始仿真轨迹录制")

    def _sim_stop_record(self):
        self.sim_rec_timer.stop()
        self.sim_recording = False
        self.robot.set_runtime_state("recording", False)
        self.sim_rec_btn.setText("🔴  开始录制")
        self.sim_rec_btn.setStyleSheet("padding:6px;")
        n = len(self.sim_trajectory)
        dur = self.sim_trajectory[-1].timestamp if n > 0 else 0.0
        self._sim_refresh_traj_state()
        self._log(f"⏹ 录制结束：{n} 点，{dur:.1f}s")

    def _sim_rec_tick(self):
        """10 Hz 定时采样仿真规划滑块当前值"""
        if not self.sim_recording:
            return
        angles = [s.value() for s in self.sim_sliders]
        t = time.time() - self.sim_rec_start
        self.sim_trajectory.append(
            TrajectoryPoint(
                timestamp=t,
                angles=angles,
                gripper=self._sim_gripper_value,
                target_gripper=self._sim_gripper_value,
                source="sim_planner",
            )
        )
        self.sim_traj_info.setText(f"录制中… {len(self.sim_trajectory)} 点")

    def _sim_clear_traj(self):
        if self.sim_recording:
            self._sim_stop_record()
        self.sim_trajectory = []
        self._sim_refresh_traj_state()
        if self.mujoco_widget:
            self.mujoco_widget.clear_ghost()
        self._refresh_sim_safety_status()
        self._log("已清空仿真轨迹")

    def _sim_refresh_traj_state(self):
        n = len(self.sim_trajectory)
        if n == 0:
            self.sim_traj_info.setText("轨迹：空")
        else:
            dur = self.sim_trajectory[-1].timestamp
            self.sim_traj_info.setText(f"{n} 点 | 时长 {dur:.1f}s")
        has_traj = n > 1
        self.sim_prev_btn.setEnabled(has_traj)
        self.sim_exec_btn.setEnabled(has_traj)

    def _sim_save_traj(self):
        if not self.sim_trajectory:
            QMessageBox.information(self, "提示", "当前没有可保存的仿真轨迹")
            return
        if not self._sim_traj_io:
            QMessageBox.warning(self, "错误", "示教轨迹存取模块不可用")
            return

        path, selected = QFileDialog.getSaveFileName(
            self,
            "保存仿真轨迹",
            os.path.join(_PROJECT_DIR, "sim_trajectory.json"),
            "JSON Files (*.json);;CSV Files (*.csv)",
        )
        if not path:
            return

        fmt = 'csv' if selected.startswith("CSV") or path.lower().endswith(".csv") else 'json'
        if fmt == 'json' and not path.lower().endswith(".json"):
            path += ".json"
        if fmt == 'csv' and not path.lower().endswith(".csv"):
            path += ".csv"

        metadata = {
            "source": "sim_planner",
            "record_hz": 10,
            "generated_by": "MainWindowWithMuJoCo",
            "gripper_encoding": "0.0=closed/clamped, 1.0=open",
        }
        self._sim_traj_io.save_trajectory(
            self.sim_trajectory,
            path,
            format=fmt,
            metadata=metadata,
        )
        self._log(f"💾 仿真轨迹已保存: {os.path.basename(path)}")

    def _sim_load_traj(self):
        if not self._sim_traj_io:
            QMessageBox.warning(self, "错误", "示教轨迹存取模块不可用")
            return
        path, _ = QFileDialog.getOpenFileName(
            self,
            "加载仿真轨迹",
            _PROJECT_DIR,
            "Trajectory Files (*.json *.csv)",
        )
        if not path:
            return

        traj = self._sim_traj_io.load_trajectory(path, validate_limits=True)
        if not traj:
            QMessageBox.warning(self, "错误", "轨迹加载失败或内容为空")
            return

        self.sim_trajectory = traj
        self._sim_refresh_traj_state()
        first_angles = self.sim_trajectory[0].angles
        first_gripper = self.sim_trajectory[0].playback_gripper()
        for i, a in enumerate(first_angles[:6]):
            self.sim_sliders[i].setValue(int(round(a)))
        self._set_sim_gripper(first_gripper)
        if self.mujoco_widget and self.ghost_cb.isChecked():
            self.mujoco_widget.set_ghost(self._mujoco_visual_angles(first_angles), first_gripper)
        self._refresh_sim_safety_status()
        self._log(f"📂 已加载仿真轨迹: {os.path.basename(path)} ({len(traj)} 点)")

    def _sim_toggle_preview(self):
        if self.sim_preview_active:
            self._sim_stop_preview()
        else:
            self._sim_start_preview()

    def _sim_start_preview(self):
        if not self.sim_trajectory:
            return
        self.sim_preview_active = True
        self.sim_preview_idx    = 0
        self.sim_prev_btn.setText("⏹  停止预览")
        self.sim_preview_timer.start(100)
        self._sim_preview_tick()

    def _sim_stop_preview(self):
        self.sim_preview_timer.stop()
        self.sim_preview_active = False
        self.sim_prev_btn.setText("🔍  预览轨迹")
        self._set_sim_gripper(self._sim_gripper_value)

    def _sim_preview_tick(self):
        if not self.sim_trajectory or not self.sim_preview_active:
            self._sim_stop_preview()
            return
        pt = self.sim_trajectory[self.sim_preview_idx]
        if self.mujoco_widget:
            self.mujoco_widget.set_ghost(
                self._mujoco_visual_angles(pt.angles),
                pt.playback_gripper(),
            )
        # 同步更新仿真规划滑块显示（不触发 valueChanged → 不更新影子，避免循环）
        for i, a in enumerate(pt.angles[:6]):
            self.sim_val_labels[i].setText(self._format_joint_angle_label(i, float(a)))
        if hasattr(self, "sim_gripper_state_lbl"):
            self.sim_gripper_state_lbl.setText(gripper_state_label(pt.playback_gripper()))
        self.sim_preview_idx = (self.sim_preview_idx + 1) % len(self.sim_trajectory)

    def _sim_execute_current_pose(self):
        angles = [s.value() for s in self.sim_sliders]
        ok, msg = self._send_angles_to_robot(
            angles,
            self.speed_spin.value(),
            context="仿真规划当前姿态",
        )
        if ok:
            ok_g, msg_g = self.robot.set_gripper_state(self._sim_gripper_value)
            if not ok_g:
                self._log(f"⚠️ 规划夹爪状态发送失败: {msg_g}")
            self._log(f"✅ 当前规划姿态已发送: {[f'{a:.1f}' for a in angles]}")
        else:
            if msg not in ("未通过安全检查", "超出用户安全边界"):
                QMessageBox.warning(self, "错误", msg)

    def _sim_execute(self,
                     speed_ratio_override: Optional[float] = None,
                     joint_speed_override: Optional[int] = None,
                     min_interval_override: Optional[float] = None):
        if not self.sim_trajectory:
            return
        if not self._ensure_robot_ready():
            return

        # 限位检查
        violations: List[str] = []
        for i, pt in enumerate(self.sim_trajectory):
            for ji, val, lo, hi in JointLimitChecker.get_violations(pt.angles):
                violations.append(
                    f"t={pt.timestamp:.1f}s J{ji+1}: {val:.1f}°超出[{lo},{hi}]"
                )

        if violations:
            reply = QMessageBox.question(
                self, "存在限位违规",
                f"共 {len(violations)} 处超限（前 5 条）：\n"
                + "\n".join(violations[:5])
                + "\n\n自动截断后执行？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            )
            if reply == QMessageBox.StandardButton.No:
                return
            for pt in self.sim_trajectory:
                pt.angles = JointLimitChecker.clamp_angles(pt.angles)

        boundary_violations: List[str] = []
        for i, pt in enumerate(self.sim_trajectory):
            angles = pt.playback_angles() if hasattr(pt, "playback_angles") else pt.angles
            issues = self._user_boundary_issues(angles)
            if issues:
                boundary_violations.append(
                    f"第 {i + 1} 点 / t={pt.timestamp:.1f}s：{issues[0]}"
                )

        if boundary_violations:
            QMessageBox.warning(
                self,
                "用户安全边界拦截",
                f"规划轨迹有 {len(boundary_violations)} 个点超出用户边界（前 6 条）：\n"
                + "\n".join(boundary_violations[:6]),
            )
            self._log(f"⚠️ 规划轨迹超出用户安全边界：{boundary_violations[0]}")
            return

        safety_report = self._analyze_trajectory_safety(self.sim_trajectory)
        if not safety_report.get("safe", True):
            msg = self._format_trajectory_safety_message(safety_report)
            self._log(f"⚠️ 规划轨迹未通过安全检查：{safety_report.get('summary', '存在碰撞风险')}")
            QMessageBox.warning(self, "规划轨迹不安全", msg)
            return

        n   = len(self.sim_trajectory)
        dur = self.sim_trajectory[-1].timestamp
        spd = (
            float(speed_ratio_override)
            if speed_ratio_override is not None
            else self.sim_speed_spin.value()
        )
        joint_speed = (
            int(joint_speed_override)
            if joint_speed_override is not None
            else int(self.speed_spin.value())
        )
        min_interval_s = (
            float(min_interval_override)
            if min_interval_override is not None
            else 0.10
        )
        if QMessageBox.question(
            self, "确认执行",
            f"将发送 {n} 个轨迹点（录制时长 {dur:.1f}s，×{spd:.1f} 速，关节速度 {joint_speed}°/s）。\n"
            "执行过程中仿真显示真机实际运动，影子同步回放。确认？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        ) == QMessageBox.StandardButton.No:
            return

        self._sim_stop_preview()

        self.sim_progress.setVisible(True)
        self.sim_progress.setRange(0, n)
        self.sim_progress.setValue(0)
        self.sim_exec_btn.setVisible(False)
        self.sim_stop_btn.setVisible(True)

        self.sim_exec_thread = SimPlaybackThread(
            self.robot,
            self.sim_trajectory,
            spd,
            joint_speed=joint_speed,
            min_interval_s=min_interval_s,
        )
        self.sim_exec_thread.progress.connect(
            lambda cur, tot: self.sim_progress.setValue(cur)
        )
        self.sim_exec_thread.log.connect(self._log)
        self.sim_exec_thread.finished.connect(self._sim_exec_finished)
        self.robot.set_runtime_state("playback", True)
        self.sim_exec_thread.start()
        self._log(
            f"▶ 开始向真机发送仿真轨迹（×{spd:.1f} 速，关节速度 {joint_speed}°/s，最小间隔 {min_interval_s:.2f}s）"
        )

    def _sim_stop_execution(self):
        if self.sim_exec_thread and self.sim_exec_thread.isRunning():
            self.sim_exec_thread.request_stop()
            self.robot.emergency_stop()
            self._log("⏹ 轨迹执行已中止")

    def _sim_exec_finished(self, success: bool, msg: str):
        self.robot.set_runtime_state("playback", False)
        self.sim_progress.setVisible(False)
        self.sim_exec_btn.setVisible(True)
        self.sim_stop_btn.setVisible(False)
        self._log(f"{'✅' if success else '❌'} {msg}")

    # ══════════════════════════════════════════════════════════════════════════
    # 示教（真机）
    # ══════════════════════════════════════════════════════════════════════════

    def _teach_toggle_mode(self):
        if not self.robot.connected:
            QMessageBox.warning(self, "错误", "请先连接机械臂")
            return
        if not self.teach_in_mode:
            if not hasattr(self, '_tm') or self._tm is None:
                self._tm = TeachMode(self.robot)
            if self._tm.enter_teach_mode():
                self.teach_in_mode = True
                self.teach_mode_btn.setText("🖐  退出示教模式")
                self.teach_mode_btn.setStyleSheet(
                    "QPushButton{background:#F44336;color:white;"
                    "padding:10px;border-radius:4px;}"
                )
                self.teach_rec_btn.setEnabled(True)
                self.teach_status_lbl.setText("状态：示教模式（可拖动关节）")
                self._log("✅ 进入示教模式")
            else:
                QMessageBox.warning(self, "错误", "进入示教模式失败")
        else:
            if self.teach_recording:
                self._teach_stop_record()
            self._tm.exit_teach_mode()
            self.teach_in_mode = False
            self.teach_mode_btn.setText("🖐  进入示教模式")
            self.teach_mode_btn.setStyleSheet(
                "QPushButton{background:#9C27B0;color:white;"
                "padding:10px;border-radius:4px;}"
            )
            self.teach_rec_btn.setEnabled(False)
            self.teach_rec_btn.setText("🔴  开始录制")
            self.teach_status_lbl.setText("状态：空闲")
            self._log("退出示教模式")

    def _teach_toggle_record(self):
        if not self.teach_recording:
            self._teach_start_record()
        else:
            self._teach_stop_record()

    def _teach_start_record(self):
        if not self.teach_in_mode:
            return
        self._tm.start_recording()
        self.teach_recording = True
        self.robot.set_runtime_state("recording", True)
        self.teach_rec_btn.setText("⏹  停止录制")
        self.teach_rec_btn.setStyleSheet(
            "background:#F44336;color:white;padding:6px;"
        )
        self.teach_status_lbl.setText("状态：录制中…")
        self.teach_play_btn.setEnabled(False)
        self._log("🔴 开始录制示教轨迹")

    def _teach_stop_record(self):
        self.teach_trajectory = self._tm.stop_recording()
        self.teach_recording  = False
        self.robot.set_runtime_state("recording", False)
        n = len(self.teach_trajectory)
        self.teach_rec_btn.setText("🔴  开始录制")
        self.teach_rec_btn.setStyleSheet("")
        self.teach_status_lbl.setText(f"状态：已录制 {n} 点")
        self.teach_play_btn.setEnabled(n > 0)
        self._log(f"⏹ 停止录制，共 {n} 点")

    def _teach_update_count(self):
        if self.teach_recording and hasattr(self, '_tm') and self._tm:
            n = len(self._tm.trajectory)
            self.teach_count_lbl.setText(f"已录制：{n} 点")
        if self.gui_live_recording and self.gui_live_recorder:
            n = len(self.gui_live_recorder.trajectory)
            self.gui_live_count_lbl.setText(f"GUI 轨迹：录制中 {n} 点")

    def _teach_playback(self):
        if not self.teach_trajectory:
            return
        if not self.robot.connected or not self.robot.enabled:
            QMessageBox.warning(self, "错误", "请先连接并使能机械臂")
            return

        self.teach_play_progress.setVisible(True)
        self.teach_play_progress.setRange(0, 100)
        self.teach_play_progress.setValue(0)

        speed_r = float(self.teach_speed_spin.value())
        self.teach_playback_thread = PlaybackThread(
            self.robot, self.teach_trajectory, speed_r
        )
        # PlaybackThread.progress_signal = pyqtSignal(int)  — 0~100
        self.teach_playback_thread.progress_signal.connect(
            self.teach_play_progress.setValue
        )
        # PlaybackThread.finished_signal = pyqtSignal()  — no args
        self.teach_playback_thread.finished_signal.connect(
            self._teach_playback_done
        )
        self.robot.set_runtime_state("playback", True)
        self.teach_playback_thread.start()
        self._log(f"▶ 开始示教回放（×{speed_r:.0f} 速）")

    def _teach_playback_done(self):
        self.robot.set_runtime_state("playback", False)
        self.teach_play_progress.setVisible(False)
        self._log("✅ 示教回放完成")

    def _collect_light_settings_from_ui(self) -> dict:
        settings = {
            "enabled": self.light_enabled_chk.isChecked(),
            "brightness": self.light_brightness_slider.value(),
            "state_modes": {},
        }
        for state_key, combo in self.light_state_combos.items():
            settings["state_modes"][state_key] = combo.currentData()
        return settings

    def _populate_light_settings_ui(self):
        settings = self.robot.get_light_settings()
        if hasattr(self, "light_enabled_chk"):
            self.light_enabled_chk.setChecked(settings["enabled"])
        if hasattr(self, "light_brightness_slider"):
            self.light_brightness_slider.setValue(int(settings.get("brightness", 24)))
            self._update_light_brightness_label()
        if hasattr(self, "light_state_combos"):
            for state_key, combo in self.light_state_combos.items():
                mode_key = settings["state_modes"].get(state_key, "off")
                idx = combo.findData(mode_key)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
        self._refresh_light_panel()

    def _save_light_settings(self):
        settings = self._collect_light_settings_from_ui()
        ok, msg = self.robot.update_light_settings(settings, persist=True)
        self._log(msg if ok else f"⚠️ {msg}")
        self._refresh_light_panel()

    def _preview_light_state(self):
        settings = self._collect_light_settings_from_ui()
        state_key = self.light_preview_state_combo.currentData()
        ok, msg = self.robot.preview_light_state(state_key, settings=settings)
        self._log(msg if ok else f"⚠️ {msg}")
        self._refresh_light_panel()

    def _restore_light_auto_state(self):
        ok, msg = self.robot.restore_light_state()
        self._log(msg if ok else f"⚠️ {msg}")
        self._refresh_light_panel()

    def _refresh_light_panel(self):
        if not hasattr(self, "light_link_status_lbl"):
            return

        snapshot = self.robot.get_light_status_snapshot()
        linked = "已连接 REF native USB" if snapshot["connected"] else "未连接"
        auto_state = snapshot.get("active_state_label") or "--"
        last_mode = snapshot.get("last_mode_label") or "--"
        if not snapshot["enabled"]:
            auto_state += "（自动联动已关闭）"

        self.light_link_status_lbl.setText(f"接口：{linked}")
        self.light_auto_state_lbl.setText(f"自动状态：{auto_state}")
        self.light_last_mode_lbl.setText(f"当前灯效：{last_mode}")
        brightness_text = f"当前亮度：{snapshot.get('brightness', '--')} / 45"
        if snapshot.get("brightness_supported") is False:
            brightness_text += "（当前固件未支持，重刷后生效）"
        self.light_brightness_lbl.setText(brightness_text)

        err = snapshot.get("last_error") or "--"
        self.light_error_lbl.setText(f"最近错误：{err}")

    def _update_light_brightness_label(self):
        if not hasattr(self, "light_brightness_value_lbl"):
            return
        self.light_brightness_value_lbl.setText(
            f"{self.light_brightness_slider.value()} / 45"
        )

    def _open_teach_dialog(self):
        from gui.teach_dialog import TeachDialog
        if not self.robot.connected:
            QMessageBox.warning(self, "警告", "请先连接机械臂")
            return
        TeachDialog(self.robot, self).exec()

    # ══════════════════════════════════════════════════════════════════════════
    # 通用
    # ══════════════════════════════════════════════════════════════════════════

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")

    def closeEvent(self, event):
        if self.update_thread:
            self.update_thread.stop()
            self.update_thread.wait()
        if self.ik_thread and self.ik_thread.isRunning():
            self.ik_thread.wait()
        if self.sim_exec_thread and self.sim_exec_thread.isRunning():
            self.sim_exec_thread.request_stop()
        if self.teach_playback_thread and self.teach_playback_thread.isRunning():
            self.teach_playback_thread.stop()
            self.teach_playback_thread.wait()
        if self._gripper_action_thread and self._gripper_action_thread.isRunning():
            self._gripper_action_thread.wait(3000)
        if self.gui_live_recording:
            self._gui_live_stop_record()
        self.robot.disconnect()
        event.accept()
