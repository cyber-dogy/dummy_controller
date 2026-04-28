"""Persistent world-frame calibration and workspace boundary settings."""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path

try:
    from utils.config import J6_FIRMWARE_REDUCTION, J6_OUTPUT_REDUCTION_DEFAULT, JOINT_LIMITS
except Exception:
    from .config import J6_FIRMWARE_REDUCTION, J6_OUTPUT_REDUCTION_DEFAULT, JOINT_LIMITS


PROJECT_DIR = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_DIR / "config"
WORLD_CONFIG_PATH = CONFIG_DIR / "world_settings.json"

DEFAULT_WORLD_SETTINGS = {
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
    "workspace_min": {
        "x_mm": -400.0,
        "y_mm": -80.0,
        "z_mm": 0.0,
    },
    "workspace_max": {
        "x_mm": 400.0,
        "y_mm": 450.0,
        "z_mm": 700.0,
    },
    # MuJoCo floor/base/workspace guard is kept available but disabled by
    # default because the physical fixture can differ from the model while
    # debugging.
    "mujoco_guard_enabled": False,
    # User-defined soft boundary: independent from firmware hard limits.
    "user_boundary_enabled": False,
    "joint_boundary_enabled": False,
    "joint_boundary_min": [float(lo) for lo, _ in JOINT_LIMITS],
    "joint_boundary_max": [float(hi) for _, hi in JOINT_LIMITS],
    "world_boundary_enabled": False,
    "world_boundary_min": {
        "x_mm": -400.0,
        "y_mm": -80.0,
        "z_mm": 0.0,
    },
    "world_boundary_max": {
        "x_mm": 400.0,
        "y_mm": 450.0,
        "z_mm": 700.0,
    },
    # Real J6 output angle = firmware angle * firmware reduction / mechanical ratio.
    # Commands still use firmware angle because the controller firmware owns the motor-side angle.
    "j6_firmware_reduction": float(J6_FIRMWARE_REDUCTION),
    "j6_output_reduction": float(J6_OUTPUT_REDUCTION_DEFAULT),
}


def _as_float(value, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _normalize_axis_box(settings: dict, key_min: str, key_max: str, merged: dict) -> None:
    raw_min = settings.get(key_min, {}) if isinstance(settings.get(key_min), dict) else {}
    raw_max = settings.get(key_max, {}) if isinstance(settings.get(key_max), dict) else {}
    for axis in ("x_mm", "y_mm", "z_mm"):
        lo = _as_float(raw_min.get(axis), merged[key_min][axis])
        hi = _as_float(raw_max.get(axis), merged[key_max][axis])
        if lo > hi:
            lo, hi = hi, lo
        merged[key_min][axis] = lo
        merged[key_max][axis] = hi


def _normalize_joint_boundary(settings: dict, merged: dict) -> None:
    raw_min = settings.get("joint_boundary_min")
    raw_max = settings.get("joint_boundary_max")
    min_vals = raw_min if isinstance(raw_min, list) else []
    max_vals = raw_max if isinstance(raw_max, list) else []

    normalized_min = []
    normalized_max = []
    for i, (fw_lo, fw_hi) in enumerate(JOINT_LIMITS):
        lo_default = merged["joint_boundary_min"][i]
        hi_default = merged["joint_boundary_max"][i]
        lo = _as_float(min_vals[i], lo_default) if i < len(min_vals) else lo_default
        hi = _as_float(max_vals[i], hi_default) if i < len(max_vals) else hi_default
        lo = max(float(fw_lo), min(float(fw_hi), lo))
        hi = max(float(fw_lo), min(float(fw_hi), hi))
        if lo > hi:
            lo, hi = hi, lo
        normalized_min.append(lo)
        normalized_max.append(hi)

    merged["joint_boundary_min"] = normalized_min
    merged["joint_boundary_max"] = normalized_max


def normalize_world_settings(settings: dict | None) -> dict:
    merged = deepcopy(DEFAULT_WORLD_SETTINGS)
    if not isinstance(settings, dict):
        return merged

    merged["origin_x_mm"] = _as_float(settings.get("origin_x_mm"), merged["origin_x_mm"])
    merged["origin_y_mm"] = _as_float(settings.get("origin_y_mm"), merged["origin_y_mm"])
    merged["origin_z_mm"] = _as_float(settings.get("origin_z_mm"), merged["origin_z_mm"])
    merged["yaw_deg"] = max(-180.0, min(180.0, _as_float(settings.get("yaw_deg"), merged["yaw_deg"])))
    merged["work_surface_z_mm"] = _as_float(
        settings.get("work_surface_z_mm"),
        merged["work_surface_z_mm"],
    )
    merged["min_tcp_clearance_mm"] = max(
        0.0,
        min(300.0, _as_float(settings.get("min_tcp_clearance_mm"), merged["min_tcp_clearance_mm"])),
    )
    merged["min_arm_clearance_mm"] = max(
        0.0,
        min(300.0, _as_float(settings.get("min_arm_clearance_mm"), merged["min_arm_clearance_mm"])),
    )
    merged["base_guard_radius_mm"] = max(
        0.0,
        min(500.0, _as_float(settings.get("base_guard_radius_mm"), merged["base_guard_radius_mm"])),
    )
    merged["base_guard_height_mm"] = max(
        0.0,
        min(500.0, _as_float(settings.get("base_guard_height_mm"), merged["base_guard_height_mm"])),
    )
    merged["workspace_box_enabled"] = bool(
        settings.get("workspace_box_enabled", merged["workspace_box_enabled"])
    )
    merged["mujoco_guard_enabled"] = bool(
        settings.get("mujoco_guard_enabled", merged["mujoco_guard_enabled"])
    )
    merged["user_boundary_enabled"] = bool(
        settings.get("user_boundary_enabled", merged["user_boundary_enabled"])
    )
    merged["joint_boundary_enabled"] = bool(
        settings.get("joint_boundary_enabled", merged["joint_boundary_enabled"])
    )
    merged["world_boundary_enabled"] = bool(
        settings.get("world_boundary_enabled", merged["world_boundary_enabled"])
    )
    merged["j6_output_reduction"] = max(
        1.0,
        min(200.0, _as_float(settings.get("j6_output_reduction"), merged["j6_output_reduction"])),
    )
    merged["j6_firmware_reduction"] = max(
        1.0,
        min(200.0, _as_float(settings.get("j6_firmware_reduction"), merged["j6_firmware_reduction"])),
    )

    _normalize_axis_box(settings, "workspace_min", "workspace_max", merged)
    _normalize_axis_box(settings, "world_boundary_min", "world_boundary_max", merged)
    _normalize_joint_boundary(settings, merged)

    return merged


def save_world_settings(settings: dict) -> Path:
    normalized = normalize_world_settings(settings)
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    WORLD_CONFIG_PATH.write_text(
        json.dumps(normalized, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return WORLD_CONFIG_PATH


def load_world_settings() -> dict:
    if not WORLD_CONFIG_PATH.exists():
        save_world_settings(DEFAULT_WORLD_SETTINGS)
        return deepcopy(DEFAULT_WORLD_SETTINGS)

    try:
        data = json.loads(WORLD_CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        save_world_settings(DEFAULT_WORLD_SETTINGS)
        return deepcopy(DEFAULT_WORLD_SETTINGS)

    normalized = normalize_world_settings(data)
    if normalized != data:
        save_world_settings(normalized)
    return normalized
