#!/usr/bin/env python3
"""
MuJoCo 与 PyQt6 集成模块
提供实时 3D 可视化组件（基于 URDF dummy2 转换的 MJCF）

新增功能：
  - 影子渲染（ghost）：蓝色半透明影像，用于仿真轨迹规划预览
  - 数值逆解（IK）：scipy L-BFGS-B + MuJoCo FK，多初始猜测
  - 独立 IK 模型，与渲染循环不冲突
"""

import numpy as np
import mujoco
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap
from typing import Optional, List, Dict, Any

try:
    from utils.tcp_config import load_tcp_settings
except Exception:
    def load_tcp_settings():
        return {"tcp_offset_mm": 75.0}

try:
    from utils.world_config import load_world_settings, normalize_world_settings
except Exception:
    def load_world_settings():
        return {
            "origin_x_mm": 0.0,
            "origin_y_mm": 0.0,
            "origin_z_mm": 0.0,
            "yaw_deg": 0.0,
            "work_surface_z_mm": 0.0,
            "min_tcp_clearance_mm": 35.0,
            "min_arm_clearance_mm": 20.0,
            "base_guard_radius_mm": 110.0,
            "base_guard_height_mm": 145.0,
            "workspace_box_enabled": False,
            "workspace_min": {"x_mm": -400.0, "y_mm": -80.0, "z_mm": 0.0},
            "workspace_max": {"x_mm": 400.0, "y_mm": 450.0, "z_mm": 700.0},
        }

    def normalize_world_settings(settings):
        return dict(load_world_settings())

# ── scipy 可用性检测 ──────────────────────────────────────────────────────────
try:
    from scipy.optimize import minimize as _scipy_minimize
    _SCIPY_OK = True
except ImportError:
    _SCIPY_OK = False


# ── 固件角度 → URDF 关节角度映射 ──────────────────────────────────────────────
#
#   joint1 axis="0 0 -1"  → urdf_j1 = -deg2rad(fw_j1)
#   joint2 axis="1 0 0"   → urdf_j2 = -deg2rad(fw_j2)  [符号相反]
#   joint3 axis="-1 0 0"  → urdf_j3 =  deg2rad(fw_j3 - 90)
#                            固件 J3=90° = URDF 零位（L 形臂）
#   joint4 axis="0 -1 0"  → urdf_j4 = -deg2rad(fw_j4)
#   joint5 axis="-1 0 0"  → urdf_j5 = -deg2rad(fw_j5)  [实机方向与旧仿真相反]
#   joint6 axis="0 -1 0"  → urdf_j6 = -deg2rad(fw_j6)
# ────────────────────────────────────────────────────────────────────────────

def firmware_to_urdf(fw_angles_deg: List[float]) -> List[float]:
    """将固件角度（度）转换为 URDF MJCF 关节角度（弧度）。"""
    j1, j2, j3, j4, j5, j6 = fw_angles_deg[:6]
    r = np.deg2rad
    return [
        -r(j1),       # joint1 axis=-Z
        -r(j2),       # joint2 axis=+X（符号取反）
         r(j3 - 90),  # joint3 axis=-X，偏移 90°（URDF 零位 = 固件 L 形 90°）
        -r(j4),       # joint4 axis=-Y
        -r(j5),       # joint5 axis=-X，实机方向与旧仿真相反
        -r(j6),       # joint6 axis=-Y
    ]


_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
_REST_FW_DEG = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
_GRIPPER_MAX_OPEN_M = 0.028


def normalize_gripper_opening(value: object) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if v > 1.0:
        return 1.0 if v >= 15.0 else 0.0
    return 1.0 if v >= 0.5 else 0.0


class MuJoCoWidget(QWidget):
    """
    PyQt6 MuJoCo 3D 可视化组件

    三套 model/data：
      main  — 主渲染（跟随真机或空闲姿态）
      ghost — 影子渲染（仿真规划预览，蓝色半透明叠加）
      ik    — 数值逆解专用（不参与渲染，避免冲突）
    """

    end_effector_updated = pyqtSignal(list)   # [x, y, z, roll, pitch, yaw] mm/deg
    FLOOR_GUARD_GEOMS = (
        "link3_geom",
        "link4_geom",
        "link5_geom",
        "link6_geom",
        "figer1_geom",
        "figer2_geom",
    )
    BASE_GUARD_GEOMS = (
        "link4_geom",
        "link5_geom",
        "link6_geom",
        "figer1_geom",
        "figer2_geom",
    )

    def __init__(self, parent=None, width: int = 640, height: int = 480):
        super().__init__(parent)

        self.render_width  = width
        self.render_height = height

        # ── 主 ──
        self.model:    Optional[mujoco.MjModel]  = None
        self.data:     Optional[mujoco.MjData]   = None
        self.renderer: Optional[mujoco.Renderer] = None

        # ── 影子 ──
        self.ghost_model:    Optional[mujoco.MjModel]  = None
        self.ghost_data:     Optional[mujoco.MjData]   = None
        self.ghost_renderer: Optional[mujoco.Renderer] = None
        self.ghost_enabled:  bool                      = False
        self.target_model:    Optional[mujoco.MjModel]  = None
        self.target_data:     Optional[mujoco.MjData]   = None
        self.target_renderer: Optional[mujoco.Renderer] = None
        self.target_enabled:  bool                      = False

        # ── IK ──
        self.ik_model: Optional[mujoco.MjModel] = None
        self.ik_data:  Optional[mujoco.MjData]  = None

        self.scene  = None
        self.camera = None
        self.main_site_id = -1
        self.ghost_site_id = -1
        self.target_site_id = -1
        self.ik_site_id = -1
        self.tcp_offset_mm = float(load_tcp_settings()["tcp_offset_mm"])
        self.world_settings = normalize_world_settings(load_world_settings())

        # 当前目标角度（固件度数）
        self.fw_angles_deg:  List[float] = list(_REST_FW_DEG)
        self.gripper_opening: float = 0.0
        self.ghost_gripper_opening: float = 0.0
        self.target_gripper_opening: float = 0.0
        # 平滑显示角度（低通滤波，α=0.20 ≈ 0.4s 收敛）
        self._disp_fw_deg:   List[float] = list(_REST_FW_DEG)
        self._disp_gripper_opening: float = 0.0
        self.smooth_alpha:   float       = 0.20

        self._init_ui()
        self._init_mujoco()

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self._update_frame)
        self.update_timer.start(33)   # ~30 FPS

    # ── UI ──────────────────────────────────────────────────────────────────

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.image_label = QLabel()
        self.image_label.setFixedSize(self.render_width, self.render_height)
        self.image_label.setStyleSheet("background-color: #1a1a1a;")
        layout.addWidget(self.image_label)

        info = QHBoxLayout()
        self.pos_label   = QLabel("末端位置: --")
        self.angle_label = QLabel("关节角度: --")
        for lbl in (self.pos_label, self.angle_label):
            lbl.setStyleSheet("font-family:monospace;font-size:10px;")
            info.addWidget(lbl)
        layout.addLayout(info)

    # ── MuJoCo 初始化 ────────────────────────────────────────────────────────

    def _init_mujoco(self):
        try:
            import os
            model_path = os.path.abspath(os.path.join(
                os.path.dirname(__file__), "..", "models", "dummy_robot.xml"
            ))

            # ── 主模型 ──
            self.model    = mujoco.MjModel.from_xml_path(model_path)
            self.data     = mujoco.MjData(self.model)
            self.renderer = mujoco.Renderer(self.model, self.render_height, self.render_width)

            # ── 影子模型（同 XML，蓝色半透明外观） ──
            self.ghost_model    = mujoco.MjModel.from_xml_path(model_path)
            self.ghost_data     = mujoco.MjData(self.ghost_model)
            self.ghost_renderer = mujoco.Renderer(
                self.ghost_model, self.render_height, self.render_width
            )
            # 将所有几何体改为蓝色
            for i in range(self.ghost_model.ngeom):
                self.ghost_model.geom_rgba[i] = [0.15, 0.45, 1.0, 1.0]

            # ── 直接控制目标影子（暖色半透明） ──
            self.target_model    = mujoco.MjModel.from_xml_path(model_path)
            self.target_data     = mujoco.MjData(self.target_model)
            self.target_renderer = mujoco.Renderer(
                self.target_model, self.render_height, self.render_width
            )
            for i in range(self.target_model.ngeom):
                self.target_model.geom_rgba[i] = [0.98, 0.69, 0.29, 1.0]

            # ── IK 专用模型 ──
            self.ik_model = mujoco.MjModel.from_xml_path(model_path)
            self.ik_data  = mujoco.MjData(self.ik_model)

            # 相机
            self.scene  = mujoco.MjvScene(self.model, maxgeom=1000)
            self.camera = mujoco.MjvCamera()
            self.camera.type      = mujoco.mjtCamera.mjCAMERA_FREE
            self.camera.lookat[0] =  0.0
            self.camera.lookat[1] =  0.15
            self.camera.lookat[2] =  0.25
            self.camera.distance  =  1.1
            self.camera.azimuth   = -45
            self.camera.elevation = -20

            self.main_site_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
            )
            self.ghost_site_id = mujoco.mj_name2id(
                self.ghost_model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
            )
            self.target_site_id = mujoco.mj_name2id(
                self.target_model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
            )
            self.ik_site_id = mujoco.mj_name2id(
                self.ik_model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
            )
            self.set_tcp_offset_mm(self.tcp_offset_mm)

            # 初始姿态
            self._apply_angles()
            mujoco.mj_forward(self.model, self.data)
            self._apply_ghost_pose(list(_REST_FW_DEG))
            self._apply_target_pose(list(_REST_FW_DEG))

            print("[MuJoCo Widget] 初始化成功（含影子渲染 & IK）")
            if not _SCIPY_OK:
                print("[MuJoCo Widget] 警告：scipy 未安装，逆解功能不可用")

        except Exception as e:
            import traceback
            print(f"[MuJoCo Widget] 初始化失败: {e}")
            traceback.print_exc()
            self.image_label.setText(f"MuJoCo 初始化失败\n{e}")

    def _apply_tcp_offset_to_model(self, model, site_id: int):
        if model is None or site_id < 0:
            return
        model.site_pos[site_id] = np.array(
            [0.0, self.tcp_offset_mm / 1000.0, 0.0],
            dtype=np.float64,
        )

    def _apply_work_surface_to_model(self, model):
        if model is None:
            return
        floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        if floor_id < 0:
            return
        local_floor_z_m = (
            float(self.world_settings["work_surface_z_mm"])
            - float(self.world_settings["origin_z_mm"])
        ) / 1000.0
        model.geom_pos[floor_id] = np.array([0.0, 0.0, local_floor_z_m], dtype=np.float64)

    def set_tcp_offset_mm(self, offset_mm: float):
        self.tcp_offset_mm = float(offset_mm)
        self._apply_tcp_offset_to_model(self.model, self.main_site_id)
        self._apply_tcp_offset_to_model(self.ghost_model, self.ghost_site_id)
        self._apply_tcp_offset_to_model(self.target_model, self.target_site_id)
        self._apply_tcp_offset_to_model(self.ik_model, self.ik_site_id)
        self._apply_work_surface_to_model(self.model)
        self._apply_work_surface_to_model(self.ghost_model)
        self._apply_work_surface_to_model(self.target_model)
        self._apply_work_surface_to_model(self.ik_model)

        if self.model is not None and self.data is not None:
            mujoco.mj_forward(self.model, self.data)
        if self.ghost_model is not None and self.ghost_data is not None:
            mujoco.mj_forward(self.ghost_model, self.ghost_data)
        if self.target_model is not None and self.target_data is not None:
            mujoco.mj_forward(self.target_model, self.target_data)
        if self.ik_model is not None and self.ik_data is not None:
            mujoco.mj_forward(self.ik_model, self.ik_data)

    def get_tcp_offset_mm(self) -> float:
        return float(self.tcp_offset_mm)

    def set_world_settings(self, settings: Dict[str, Any]):
        self.world_settings = normalize_world_settings(settings)
        self._apply_work_surface_to_model(self.model)
        self._apply_work_surface_to_model(self.ghost_model)
        self._apply_work_surface_to_model(self.target_model)
        self._apply_work_surface_to_model(self.ik_model)

        if self.model is not None and self.data is not None:
            mujoco.mj_forward(self.model, self.data)
        if self.ghost_model is not None and self.ghost_data is not None:
            mujoco.mj_forward(self.ghost_model, self.ghost_data)
        if self.target_model is not None and self.target_data is not None:
            mujoco.mj_forward(self.target_model, self.target_data)
        if self.ik_model is not None and self.ik_data is not None:
            mujoco.mj_forward(self.ik_model, self.ik_data)

    def get_world_settings(self) -> Dict[str, Any]:
        return dict(self.world_settings)

    def _world_yaw_matrix(self) -> np.ndarray:
        yaw = np.deg2rad(float(self.world_settings["yaw_deg"]))
        c = float(np.cos(yaw))
        s = float(np.sin(yaw))
        return np.array(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def _model_xyz_to_world_mm(self, pos_xyz_m: np.ndarray) -> List[float]:
        pos_mm = np.array(pos_xyz_m, dtype=np.float64) * 1000.0
        mapped = self._world_yaw_matrix() @ pos_mm
        mapped[0] += float(self.world_settings["origin_x_mm"])
        mapped[1] += float(self.world_settings["origin_y_mm"])
        mapped[2] += float(self.world_settings["origin_z_mm"])
        return mapped.astype(float).tolist()

    def _world_xyz_to_model_m(self, xyz_mm: List[float]) -> np.ndarray:
        pt = np.array(xyz_mm, dtype=np.float64)
        pt[0] -= float(self.world_settings["origin_x_mm"])
        pt[1] -= float(self.world_settings["origin_y_mm"])
        pt[2] -= float(self.world_settings["origin_z_mm"])
        return (self._world_yaw_matrix().T @ pt) / 1000.0

    def _model_rot_to_world(self, rotation: np.ndarray) -> np.ndarray:
        return self._world_yaw_matrix() @ rotation

    # ── 主姿态控制 ────────────────────────────────────────────────────────────

    def _apply_angles(self):
        """低通平滑后写入主模型 qpos"""
        if self.model is None or self.data is None:
            return
        a = self.smooth_alpha
        for i in range(6):
            self._disp_fw_deg[i] += a * (self.fw_angles_deg[i] - self._disp_fw_deg[i])
        self._disp_gripper_opening += a * (
            self.gripper_opening - self._disp_gripper_opening
        )
        urdf_rad = firmware_to_urdf(self._disp_fw_deg)
        for name, rad in zip(_JOINT_NAMES, urdf_rad):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                adr = self.model.jnt_qposadr[jid]
                lo  = self.model.jnt_range[jid, 0]
                hi  = self.model.jnt_range[jid, 1]
                self.data.qpos[adr] = float(np.clip(rad, lo, hi))
        self._apply_gripper_to_model(
            self.model,
            self.data,
            self._disp_gripper_opening,
        )

    def snap_to_angles(self):
        """立即跳转到目标角度（跳过平滑，用于加载预设）"""
        self._disp_fw_deg = list(self.fw_angles_deg)
        self._disp_gripper_opening = float(self.gripper_opening)
        self._apply_angles()

    # ── 影子姿态控制 ──────────────────────────────────────────────────────────

    def _apply_gripper_to_model(self, model, data, opening: object):
        if model is None or data is None:
            return
        opening = normalize_gripper_opening(opening)
        targets = {
            "figer1": opening * _GRIPPER_MAX_OPEN_M,
            "figer2": -opening * _GRIPPER_MAX_OPEN_M,
        }
        for name, qpos in targets.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                adr = model.jnt_qposadr[jid]
                lo = model.jnt_range[jid, 0]
                hi = model.jnt_range[jid, 1]
                data.qpos[adr] = float(np.clip(qpos, lo, hi))

    def _apply_ghost_pose(self, fw_deg: List[float], gripper_opening: object = None):
        if self.ghost_model is None or self.ghost_data is None:
            return
        urdf_rad = firmware_to_urdf(fw_deg)
        for name, rad in zip(_JOINT_NAMES, urdf_rad):
            jid = mujoco.mj_name2id(self.ghost_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                adr = self.ghost_model.jnt_qposadr[jid]
                lo  = self.ghost_model.jnt_range[jid, 0]
                hi  = self.ghost_model.jnt_range[jid, 1]
                self.ghost_data.qpos[adr] = float(np.clip(rad, lo, hi))
        if gripper_opening is not None:
            self.ghost_gripper_opening = normalize_gripper_opening(gripper_opening)
        self._apply_gripper_to_model(
            self.ghost_model,
            self.ghost_data,
            self.ghost_gripper_opening,
        )
        mujoco.mj_forward(self.ghost_model, self.ghost_data)

    def set_ghost(self, fw_deg: List[float], gripper_opening: object = None):
        """更新影子姿态并启用叠加显示"""
        self.ghost_enabled = True
        self._apply_ghost_pose(fw_deg, gripper_opening)

    def clear_ghost(self):
        """关闭影子叠加"""
        self.ghost_enabled = False

    def _apply_target_pose(self, fw_deg: List[float], gripper_opening: object = None):
        if self.target_model is None or self.target_data is None:
            return
        urdf_rad = firmware_to_urdf(fw_deg)
        for name, rad in zip(_JOINT_NAMES, urdf_rad):
            jid = mujoco.mj_name2id(self.target_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                adr = self.target_model.jnt_qposadr[jid]
                lo = self.target_model.jnt_range[jid, 0]
                hi = self.target_model.jnt_range[jid, 1]
                self.target_data.qpos[adr] = float(np.clip(rad, lo, hi))
        if gripper_opening is not None:
            self.target_gripper_opening = normalize_gripper_opening(gripper_opening)
        self._apply_gripper_to_model(
            self.target_model,
            self.target_data,
            self.target_gripper_opening,
        )
        mujoco.mj_forward(self.target_model, self.target_data)

    def set_target_ghost(self, fw_deg: List[float], gripper_opening: object = None):
        """更新直接控制目标影子并启用叠加显示。"""
        self.target_enabled = True
        self._apply_target_pose(fw_deg, gripper_opening)

    def clear_target_ghost(self):
        """关闭直接控制目标影子。"""
        self.target_enabled = False

    # ── 渲染循环 ─────────────────────────────────────────────────────────────

    def _update_frame(self):
        if self.renderer is None or self.data is None:
            return
        try:
            self._apply_angles()
            mujoco.mj_forward(self.model, self.data)

            # 渲染主体（真机/空闲姿态）
            self.renderer.update_scene(self.data, self.camera)
            rgb_main = self.renderer.render()

            rgb = rgb_main.astype(np.float32)

            if self.ghost_enabled and self.ghost_renderer is not None:
                self.ghost_renderer.update_scene(self.ghost_data, self.camera)
                rgb += self.ghost_renderer.render().astype(np.float32) * 0.38
                rgb *= 0.88

            if self.target_enabled and self.target_renderer is not None:
                self.target_renderer.update_scene(self.target_data, self.camera)
                rgb += self.target_renderer.render().astype(np.float32) * 0.30
                rgb *= 0.92

            rgb = np.clip(rgb, 0, 255).astype(np.uint8)

            h, w, c = rgb.shape
            q_img = QImage(rgb.tobytes(), w, h, c * w, QImage.Format.Format_RGB888)
            self.image_label.setPixmap(QPixmap.fromImage(q_img))
            self._update_info()

        except Exception as e:
            import traceback
            print(f"[MuJoCo Widget] 渲染错误: {e}")
            traceback.print_exc()

    def _update_info(self):
        try:
            sid = self.main_site_id
            if sid >= 0:
                pos = self._model_xyz_to_world_mm(self.data.site_xpos[sid])
                self.pos_label.setText(
                    f"末端: X={pos[0]:.1f} Y={pos[1]:.1f} Z={pos[2]:.1f} mm"
                )
                rot = self._model_rot_to_world(self.data.site_xmat[sid].reshape(3, 3))
                roll  = np.arctan2(rot[2, 1], rot[2, 2])
                pitch = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2 + rot[2, 2]**2))
                yaw   = np.arctan2(rot[1, 0], rot[0, 0])
                self.end_effector_updated.emit([
                    pos[0], pos[1], pos[2],
                    np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)
                ])
        except Exception:
            pass

        angles_str = "  ".join(
            [f"J{i+1}={a:.1f}°" for i, a in enumerate(self.fw_angles_deg)]
        )
        grip = "打开" if self.gripper_opening >= 0.5 else "夹紧"
        angles_str += f"  G={grip}"
        self.angle_label.setText(angles_str)

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def update_joint_angles(self, angles: List[float], gripper_opening: object = None):
        """更新目标角度（固件度数），平滑收敛到目标"""
        if len(angles) >= 6:
            self.fw_angles_deg = list(angles[:6])
        if gripper_opening is not None:
            self.update_gripper(gripper_opening)

    def update_gripper(self, opening: object):
        """更新夹爪开合状态，0=夹紧，1=打开。"""
        self.gripper_opening = normalize_gripper_opening(opening)

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

    def get_ee_position(self) -> Optional[List[float]]:
        """返回主体当前末端位置 [x, y, z] mm"""
        if self.model is None or self.data is None:
            return None
        try:
            sid = self.main_site_id
            if sid >= 0:
                return self._model_xyz_to_world_mm(self.data.site_xpos[sid])
        except Exception:
            pass
        return None

    def get_last_ik_ee(self) -> Optional[List[float]]:
        """返回最近一次 compute_ik 计算后 IK 模型的末端位置 [x, y, z] mm"""
        if self.ik_model is None or self.ik_data is None:
            return None
        try:
            sid = self.ik_site_id
            if sid >= 0:
                return self._model_xyz_to_world_mm(self.ik_data.site_xpos[sid])
        except Exception:
            pass
        return None

    def _set_pose_on_model(self, model, data, fw_deg: List[float]):
        """将固件角度写入指定模型/数据对。"""
        urdf_rad = firmware_to_urdf(list(fw_deg[:6]))
        for name, rad in zip(_JOINT_NAMES, urdf_rad):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                adr = model.jnt_qposadr[jid]
                lo = model.jnt_range[jid, 0]
                hi = model.jnt_range[jid, 1]
                data.qpos[adr] = float(np.clip(rad, lo, hi))
        mujoco.mj_forward(model, data)

    def analyze_pose_safety(
        self,
        fw_deg: List[float],
        min_tcp_height_mm: Optional[float] = None,
        min_arm_clearance_mm: Optional[float] = None,
        base_radius_mm: Optional[float] = None,
        base_guard_height_mm: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        基于 MuJoCo 几何中心做轻量安全分析。

        当前优先覆盖两类风险：
        1. 末端 / 前臂过低，容易碰地或擦工作台
        2. 前臂 / 腕部过于贴近基座顶部区域
        """
        if self.ik_model is None or self.ik_data is None or self.ik_site_id < 0:
            return {
                "available": False,
                "safe": True,
                "issues": [],
                "summary": "MuJoCo 未就绪，跳过安全分析",
            }

        self._set_pose_on_model(self.ik_model, self.ik_data, fw_deg)

        if min_tcp_height_mm is None:
            min_tcp_height_mm = float(self.world_settings["min_tcp_clearance_mm"])
        if min_arm_clearance_mm is None:
            min_arm_clearance_mm = float(self.world_settings["min_arm_clearance_mm"])
        if base_radius_mm is None:
            base_radius_mm = float(self.world_settings["base_guard_radius_mm"])
        if base_guard_height_mm is None:
            base_guard_height_mm = float(self.world_settings["base_guard_height_mm"])

        tcp_mm = self._model_xyz_to_world_mm(self.ik_data.site_xpos[self.ik_site_id].copy())
        geom_metrics: Dict[str, Dict[str, float]] = {}
        tracked_geoms = set(self.FLOOR_GUARD_GEOMS) | set(self.BASE_GUARD_GEOMS)

        for geom_name in tracked_geoms:
            gid = mujoco.mj_name2id(self.ik_model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            if gid < 0:
                continue
            pos = self.ik_data.geom_xpos[gid]
            world_xyz = self._model_xyz_to_world_mm(pos)
            geom_metrics[geom_name] = {
                "world_x_mm": float(world_xyz[0]),
                "world_y_mm": float(world_xyz[1]),
                "world_z_mm": float(world_xyz[2]),
                "base_z_mm": float(pos[2] * 1000.0),
                "base_radial_mm": float(np.linalg.norm(pos[:2]) * 1000.0),
                "rbound_mm": float(self.ik_model.geom_rbound[gid] * 1000.0),
            }

        issues: List[str] = []
        floor_hits = []
        work_surface_z_mm = float(self.world_settings["work_surface_z_mm"])
        for geom_name in self.FLOOR_GUARD_GEOMS:
            metric = geom_metrics.get(geom_name)
            if metric is None:
                continue
            clearance = metric["world_z_mm"] - work_surface_z_mm
            if clearance < min_arm_clearance_mm:
                floor_hits.append((geom_name, clearance))

        base_hits = []
        for geom_name in self.BASE_GUARD_GEOMS:
            metric = geom_metrics.get(geom_name)
            if metric is None:
                continue
            if (
                metric["base_radial_mm"] < base_radius_mm
                and metric["base_z_mm"] < base_guard_height_mm
            ):
                base_hits.append((geom_name, metric["base_radial_mm"], metric["base_z_mm"]))

        tcp_clearance = float(tcp_mm[2] - work_surface_z_mm)
        if tcp_clearance < min_tcp_height_mm:
            issues.append(
                f"末端离工作面仅 {tcp_clearance:.1f} mm，低于安全阈值 {min_tcp_height_mm:.1f} mm"
            )

        if floor_hits:
            worst_geom, worst_clearance = min(floor_hits, key=lambda item: item[1])
            issues.append(
                f"{worst_geom} 离工作面仅 {worst_clearance:.1f} mm，存在碰地/擦台风险"
            )

        if base_hits:
            closest_geom, closest_r, closest_z = min(base_hits, key=lambda item: item[1])
            issues.append(
                f"{closest_geom} 贴近基座（半径 {closest_r:.1f} mm，高度 {closest_z:.1f} mm）"
            )

        if bool(self.world_settings.get("workspace_box_enabled")):
            box_min = self.world_settings["workspace_min"]
            box_max = self.world_settings["workspace_max"]
            if not (box_min["x_mm"] <= tcp_mm[0] <= box_max["x_mm"]):
                issues.append(
                    f"末端 X={tcp_mm[0]:.1f} mm 超出工作空间 [{box_min['x_mm']:.1f}, {box_max['x_mm']:.1f}]"
                )
            if not (box_min["y_mm"] <= tcp_mm[1] <= box_max["y_mm"]):
                issues.append(
                    f"末端 Y={tcp_mm[1]:.1f} mm 超出工作空间 [{box_min['y_mm']:.1f}, {box_max['y_mm']:.1f}]"
                )
            if not (box_min["z_mm"] <= tcp_mm[2] <= box_max["z_mm"]):
                issues.append(
                    f"末端 Z={tcp_mm[2]:.1f} mm 超出工作空间 [{box_min['z_mm']:.1f}, {box_max['z_mm']:.1f}]"
                )

        floor_min_z_mm = (
            min(
                (metric["world_z_mm"] - work_surface_z_mm for metric in geom_metrics.values()),
                default=float("inf"),
            )
        )
        summary = "安全" if not issues else "；".join(issues)

        return {
            "available": True,
            "safe": len(issues) == 0,
            "issues": issues,
            "summary": summary,
            "tcp_mm": tcp_mm,
            "tcp_clearance_mm": tcp_clearance,
            "floor_min_z_mm": float(floor_min_z_mm),
            "geom_metrics": geom_metrics,
            "floor_hits": floor_hits,
            "base_hits": base_hits,
        }

    # ── 数值逆解 ─────────────────────────────────────────────────────────────

    def compute_ik(
        self,
        target_xyz_mm: List[float],
        initial_fw_deg: Optional[List[float]] = None,
        tolerance_mm: float = 8.0,
    ) -> Optional[List[float]]:
        """
        数值逆解：给定末端目标位置（毫米），返回关节角度（固件度数）。

        使用 scipy L-BFGS-B + 多初始猜测，求位置误差最小的解。
        成功时同步更新 ik_data（可通过 get_last_ik_ee() 验证正解）。

        Returns:
            fw_deg list (6 values) 或 None（不可达）
        """
        if not _SCIPY_OK:
            print("[IK] scipy 未安装，无法计算逆解")
            return None
        if self.ik_model is None:
            return None

        try:
            from utils.config import JOINT_LIMITS, JointLimitChecker
        except ImportError:
            return None

        target_m = self._world_xyz_to_model_m(target_xyz_mm)
        sid = self.ik_site_id
        if sid < 0:
            return None

        if initial_fw_deg is None:
            initial_fw_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

        def _fk(fw_deg) -> np.ndarray:
            urdf_rad = firmware_to_urdf(list(fw_deg))
            for name, rad in zip(_JOINT_NAMES, urdf_rad):
                jid = mujoco.mj_name2id(
                    self.ik_model, mujoco.mjtObj.mjOBJ_JOINT, name
                )
                if jid >= 0:
                    adr = self.ik_model.jnt_qposadr[jid]
                    lo  = self.ik_model.jnt_range[jid, 0]
                    hi  = self.ik_model.jnt_range[jid, 1]
                    self.ik_data.qpos[adr] = float(np.clip(rad, lo, hi))
            mujoco.mj_forward(self.ik_model, self.ik_data)
            return self.ik_data.site_xpos[sid].copy()

        def cost(fw_deg) -> float:
            return float(np.sum((_fk(fw_deg) - target_m) ** 2))

        bounds = [(lo, hi) for lo, hi in JOINT_LIMITS]

        # 多初始猜测：覆盖典型工作空间姿态
        guesses = [
            initial_fw_deg,
            [0.0,   0.0, 90.0, 0.0, 0.0, 0.0],   # L-Pose
            [0.0, -30.0, 120.0, 0.0, 0.0, 0.0],
            [90.0,  0.0,  90.0, 0.0, 0.0, 0.0],   # J1=90° L-Pose
            [-90.0, 0.0,  90.0, 0.0, 0.0, 0.0],
            [0.0, -50.0, 150.0, 0.0, 0.0, 0.0],
        ]

        best_x    = None
        best_cost = float("inf")

        for guess in guesses:
            try:
                r = _scipy_minimize(
                    cost, guess, method="L-BFGS-B", bounds=bounds,
                    options={"maxiter": 500, "ftol": 1e-13, "gtol": 1e-9},
                )
                if r.fun < best_cost:
                    best_cost = r.fun
                    best_x    = r.x.tolist()
            except Exception:
                continue

        threshold = (tolerance_mm * 1e-3) ** 2
        if best_x is not None and best_cost < threshold:
            _fk(best_x)   # 让 ik_data 停在最优解上，供 get_last_ik_ee() 读取
            return JointLimitChecker.clamp_angles(best_x)
        return None

    def compute_posture_biased_ik(
        self,
        target_xyz_mm: List[float],
        preferred_fw_deg: List[float],
        initial_fw_deg: Optional[List[float]] = None,
        tolerance_mm: float = 2.0,
        position_weight: float = 28.0,
        joint_weights: Optional[List[float]] = None,
    ) -> Optional[List[float]]:
        """
        带姿态偏好的数值逆解。

        目标仍然是优先锁定末端 XYZ，只是在多个可行解里偏向 preferred_fw_deg，
        适合做“鸡头稳定”这类末端位置基本不动、手臂姿态在变化的演示。
        """
        if not _SCIPY_OK:
            print("[IK] scipy 未安装，无法计算姿态偏好逆解")
            return None
        if self.ik_model is None or self.ik_data is None:
            return None

        try:
            from utils.config import JOINT_LIMITS, JointLimitChecker
        except ImportError:
            return None

        preferred = JointLimitChecker.clamp_angles(list(preferred_fw_deg[:6]))
        initial = (
            JointLimitChecker.clamp_angles(list(initial_fw_deg[:6]))
            if initial_fw_deg is not None
            else list(preferred)
        )
        weights = np.array(
            joint_weights or [0.4, 0.5, 0.5, 1.4, 1.1, 0.25],
            dtype=np.float64,
        )
        target_mm = np.array(target_xyz_mm, dtype=np.float64)

        sid = self.ik_site_id
        if sid < 0:
            return None

        def _fk_mm(fw_deg) -> np.ndarray:
            urdf_rad = firmware_to_urdf(list(fw_deg))
            for name, rad in zip(_JOINT_NAMES, urdf_rad):
                jid = mujoco.mj_name2id(
                    self.ik_model, mujoco.mjtObj.mjOBJ_JOINT, name
                )
                if jid >= 0:
                    adr = self.ik_model.jnt_qposadr[jid]
                    lo = self.ik_model.jnt_range[jid, 0]
                    hi = self.ik_model.jnt_range[jid, 1]
                    self.ik_data.qpos[adr] = float(np.clip(rad, lo, hi))
            mujoco.mj_forward(self.ik_model, self.ik_data)
            return np.array(self._model_xyz_to_world_mm(self.ik_data.site_xpos[sid].copy()))

        def cost(fw_deg) -> float:
            q = np.array(fw_deg, dtype=np.float64)
            pos_mm = _fk_mm(q)
            pos_err = float(np.sum((pos_mm - target_mm) ** 2))
            posture_err = float(np.sum(weights * ((q - preferred) ** 2)))
            return position_weight * pos_err + posture_err

        bounds = [(lo, hi) for lo, hi in JOINT_LIMITS]
        best_x = None
        best_err_mm = float("inf")

        guesses = [
            initial,
            preferred,
            self.compute_ik(target_xyz_mm, initial_fw_deg=initial, tolerance_mm=max(6.0, tolerance_mm * 3.0)),
        ]

        for guess in guesses:
            if guess is None:
                continue
            try:
                r = _scipy_minimize(
                    cost, guess, method="L-BFGS-B", bounds=bounds,
                    options={"maxiter": 400, "ftol": 1e-12, "gtol": 1e-8},
                )
                ee_mm = _fk_mm(r.x)
                err_mm = float(np.linalg.norm(ee_mm - target_mm))
                if err_mm < best_err_mm:
                    best_err_mm = err_mm
                    best_x = r.x.tolist()
            except Exception:
                continue

        if best_x is not None and best_err_mm <= tolerance_mm:
            _fk_mm(best_x)  # 让 ik_data 停在最终解上，供外部读取验证
            return JointLimitChecker.clamp_angles(best_x)
        return None

    # ── closeEvent ───────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self.update_timer.stop()
        if self.renderer:
            self.renderer.close()
        if self.ghost_renderer:
            self.ghost_renderer.close()
        if self.target_renderer:
            self.target_renderer.close()
        event.accept()


# ════════════════════════════════════════════════════════════════════════════
class MujocoDialog(QWidget):
    """独立 MuJoCo 可视化对话框（可单独弹出）"""

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

    def update_angles(self, angles: List[float], gripper_opening: object = None):
        self.mujoco_widget.update_joint_angles(angles, gripper_opening)


# ════════════════════════════════════════════════════════════════════════════
# 独立运行入口（含演示动画）
# ════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import sys, glob, serial, threading, time
    from PyQt6.QtWidgets import (
        QApplication, QDialog, QVBoxLayout, QHBoxLayout,
        QLabel, QComboBox, QPushButton, QMessageBox
    )
    from PyQt6.QtCore import QTimer

    _ser = None
    _serial_lock = threading.Lock()

    def _auto_detect_port():
        return glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')

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
                    dialog_ref.update_angles([float(parts[i+1]) for i in range(6)])
            except Exception:
                pass

    app = QApplication(sys.argv)
    root = QDialog()
    root.setWindowTitle("MuJoCo 仿真 — 独立运行")
    root.setMinimumSize(860, 680)
    root_layout = QVBoxLayout(root)

    conn_bar = QHBoxLayout()
    conn_bar.addWidget(QLabel("串口:"))
    port_combo = QComboBox()
    port_combo.setMinimumWidth(160)
    port_combo.addItems(_auto_detect_port())
    conn_bar.addWidget(port_combo)
    conn_bar.addWidget(QPushButton("刷新", clicked=lambda: (
        port_combo.clear(), port_combo.addItems(_auto_detect_port())
    )))

    connect_btn = QPushButton("连接")
    connect_btn.setStyleSheet("background-color:#4CAF50;color:white;padding:4px 12px;")
    conn_status  = QLabel("● 未连接")
    conn_status.setStyleSheet("color:red;font-weight:bold;")

    def toggle_connect():
        global _ser
        if _ser and _ser.is_open:
            with _serial_lock: _ser.close()
            _ser = None
            connect_btn.setText("连接")
            connect_btn.setStyleSheet("background-color:#4CAF50;color:white;padding:4px 12px;")
            conn_status.setText("● 未连接")
            conn_status.setStyleSheet("color:red;font-weight:bold;")
        else:
            port = port_combo.currentText()
            if not port:
                QMessageBox.warning(root, "错误", "未检测到串口"); return
            try:
                _ser = serial.Serial(port, 115200, timeout=2)
                time.sleep(0.2)
                connect_btn.setText("断开")
                connect_btn.setStyleSheet("background-color:#F44336;color:white;padding:4px 12px;")
                conn_status.setText(f"● 已连接 {port}")
                conn_status.setStyleSheet("color:green;font-weight:bold;")
            except Exception as e:
                QMessageBox.critical(root, "错误", f"连接失败: {e}")

    connect_btn.clicked.connect(toggle_connect)
    conn_bar.addWidget(connect_btn)
    conn_bar.addWidget(conn_status)
    conn_bar.addStretch()
    root_layout.addLayout(conn_bar)

    dialog = MujocoDialog(root)
    root_layout.addWidget(dialog)

    threading.Thread(target=_read_loop, args=(dialog,), daemon=True).start()

    _demo_angles = [0.0, -75.0, 180.0, 0.0, 0.0, 0.0]
    _demo_t = 0.0

    def _demo_animate():
        global _demo_angles, _demo_t
        if _ser and _ser.is_open:
            return
        _demo_t += 0.025
        s = (np.sin(_demo_t) + 1.0) * 0.5
        _demo_angles[0] = 60.0 * np.sin(_demo_t * 0.4)
        _demo_angles[1] = -75.0 * (1.0 - s)
        _demo_angles[2] = 90.0 + 90.0 * (1.0 - s)
        dialog.update_angles(_demo_angles)

    demo_timer = QTimer()
    demo_timer.timeout.connect(_demo_animate)
    demo_timer.start(50)

    root.show()
    sys.exit(app.exec())
