"""Forward-kinematics helpers for LeRobot-style export."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

try:
    import mujoco
except ImportError:  # pragma: no cover - optional dependency
    mujoco = None

from utils.tcp_config import load_tcp_settings
from utils.world_config import load_world_settings, normalize_world_settings
from core.gripper import normalize_gripper_value


def firmware_to_urdf(fw_angles_deg: List[float]) -> List[float]:
    """Convert firmware joint angles (deg) to MJCF/URDF joint angles (rad)."""
    j1, j2, j3, j4, j5, j6 = fw_angles_deg[:6]
    r = np.deg2rad
    return [
        -r(j1),
        -r(j2),
        r(j3 - 90),
        -r(j4),
        -r(j5),
        -r(j6),
    ]


def rotation_matrix_to_rot6d(rotation: np.ndarray) -> List[float]:
    """Convert a 3x3 rotation matrix into the common 6D representation."""
    return rotation[:, :2].reshape(6, order="F").astype(float).tolist()


@dataclass
class EndEffectorState:
    xyz_mm: List[float]
    rot6d: List[float]
    gripper: float
    rpy_deg: Optional[List[float]] = None

    def state_vector(self) -> List[float]:
        return self.xyz_mm + self.rot6d + [float(self.gripper)]

    def to_dict(self) -> Dict[str, object]:
        data: Dict[str, object] = {
            "x_mm": self.xyz_mm[0],
            "y_mm": self.xyz_mm[1],
            "z_mm": self.xyz_mm[2],
            "rot6d": list(self.rot6d),
            "gripper": float(self.gripper),
        }
        if self.rpy_deg is not None:
            data["rpy_deg"] = list(self.rpy_deg)
        return data


class DummyKinematics:
    """Standalone FK helper that does not depend on the live Qt widget."""

    JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    GRIPPER_MAX_OPEN_M = 0.028

    def __init__(self):
        self.model = None
        self.data = None
        self.site_id = -1
        self.tcp_offset_mm = float(load_tcp_settings()["tcp_offset_mm"])
        self.world_settings = normalize_world_settings(load_world_settings())

        if mujoco is None:
            return

        model_path = (
            Path(__file__).resolve().parents[2] / "mujoco_sim" / "models" / "dummy_robot.xml"
        )
        try:
            self.model = mujoco.MjModel.from_xml_path(str(model_path))
            self.data = mujoco.MjData(self.model)
            self.site_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector"
            )
            self.set_tcp_offset_mm(self.tcp_offset_mm)
            self.set_world_settings(self.world_settings)
        except Exception:
            self.model = None
            self.data = None
            self.site_id = -1

    @property
    def available(self) -> bool:
        return self.model is not None and self.data is not None and self.site_id >= 0

    def set_tcp_offset_mm(self, offset_mm: float):
        self.tcp_offset_mm = float(offset_mm)
        if not self.available:
            return
        self.model.site_pos[self.site_id] = np.array(
            [0.0, self.tcp_offset_mm / 1000.0, 0.0],
            dtype=np.float64,
        )

    def set_world_settings(self, settings: Dict[str, object]):
        self.world_settings = normalize_world_settings(settings)
        if not self.available:
            return
        floor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        if floor_id >= 0:
            local_floor_z_m = (
                float(self.world_settings["work_surface_z_mm"])
                - float(self.world_settings["origin_z_mm"])
            ) / 1000.0
            self.model.geom_pos[floor_id] = np.array([0.0, 0.0, local_floor_z_m], dtype=np.float64)
        mujoco.mj_forward(self.model, self.data)

    def get_world_settings(self) -> Dict[str, object]:
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

    def _model_rot_to_world(self, rotation: np.ndarray) -> np.ndarray:
        return self._world_yaw_matrix() @ rotation

    def get_tcp_offset_mm(self) -> float:
        return float(self.tcp_offset_mm)

    def compute_pose(
        self, fw_angles_deg: List[float], gripper: float = 0.0
    ) -> Optional[EndEffectorState]:
        if not self.available or len(fw_angles_deg) < 6:
            return None

        urdf_rad = firmware_to_urdf(list(fw_angles_deg[:6]))

        for name, rad in zip(self.JOINT_NAMES, urdf_rad):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                continue
            qpos_addr = self.model.jnt_qposadr[joint_id]
            lo = self.model.jnt_range[joint_id, 0]
            hi = self.model.jnt_range[joint_id, 1]
            self.data.qpos[qpos_addr] = float(np.clip(rad, lo, hi))

        opening = normalize_gripper_value(gripper)
        for name, qpos in {
            "figer1": opening * self.GRIPPER_MAX_OPEN_M,
            "figer2": -opening * self.GRIPPER_MAX_OPEN_M,
        }.items():
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                continue
            qpos_addr = self.model.jnt_qposadr[joint_id]
            lo = self.model.jnt_range[joint_id, 0]
            hi = self.model.jnt_range[joint_id, 1]
            self.data.qpos[qpos_addr] = float(np.clip(qpos, lo, hi))

        mujoco.mj_forward(self.model, self.data)

        pos = self.data.site_xpos[self.site_id]
        rot = self._model_rot_to_world(self.data.site_xmat[self.site_id].reshape(3, 3))

        roll = math.degrees(math.atan2(rot[2, 1], rot[2, 2]))
        pitch = math.degrees(
            math.atan2(-rot[2, 0], math.sqrt(rot[2, 1] ** 2 + rot[2, 2] ** 2))
        )
        yaw = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))

        return EndEffectorState(
            xyz_mm=self._model_xyz_to_world_mm(pos),
            rot6d=rotation_matrix_to_rot6d(rot),
            gripper=opening,
            rpy_deg=[roll, pitch, yaw],
        )
