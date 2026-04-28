"""Persistent light-ring configuration and mode definitions."""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Dict


PROJECT_DIR = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_DIR / "config"
LIGHT_RING_CONFIG_PATH = CONFIG_DIR / "light_ring_settings.json"


LIGHT_RING_MODES: Dict[str, dict] = {
    "off": {"label": "关闭", "mode_id": 6, "rgb_enabled": False, "swatch": "#2f3437"},
    "rainbow": {"label": "莫兰迪流彩", "mode_id": 0, "rgb_enabled": True, "swatch": "#9da7b1"},
    "fade": {"label": "灰雾呼吸", "mode_id": 1, "rgb_enabled": True, "swatch": "#b3a69a"},
    "blink": {"label": "陶光闪烁", "mode_id": 2, "rgb_enabled": True, "swatch": "#bcaea1"},
    "red": {"label": "烟粉红", "mode_id": 3, "rgb_enabled": True, "swatch": "#927370"},
    "green": {"label": "鼠尾草绿", "mode_id": 4, "rgb_enabled": True, "swatch": "#85937b"},
    "blue": {"label": "雾霾蓝", "mode_id": 5, "rgb_enabled": True, "swatch": "#748599"},
}


LIGHT_RING_STATE_LABELS: Dict[str, str] = {
    "disconnected": "未连接机械臂",
    "connected": "已连接未使能",
    "enabled": "电机已使能",
    "moving": "单次运动中",
    "teach": "示教模式",
    "recording": "录制中",
    "playback": "回放中",
    "estop": "急停/报警",
    "error": "命令错误",
}


LIGHT_RING_STATE_PRIORITY = [
    "error",
    "estop",
    "playback",
    "recording",
    "teach",
    "moving",
    "enabled",
    "connected",
    "disconnected",
]


DEFAULT_LIGHT_RING_SETTINGS = {
    "enabled": True,
    "brightness": 24,
    "state_modes": {
        "disconnected": "off",
        "connected": "blue",
        "enabled": "green",
        "moving": "fade",
        "teach": "blue",
        "recording": "blink",
        "playback": "rainbow",
        "estop": "red",
        "error": "blink",
    },
}


def normalize_light_ring_settings(settings: dict | None) -> dict:
    """Merge settings with defaults and sanitize unknown keys."""
    merged = deepcopy(DEFAULT_LIGHT_RING_SETTINGS)
    if not isinstance(settings, dict):
        return merged

    merged["enabled"] = bool(settings.get("enabled", merged["enabled"]))
    try:
        brightness = int(settings.get("brightness", merged["brightness"]))
    except Exception:
        brightness = merged["brightness"]
    merged["brightness"] = max(0, min(45, brightness))

    state_modes = settings.get("state_modes", {})
    if isinstance(state_modes, dict):
        for state_key in LIGHT_RING_STATE_LABELS:
            mode_key = state_modes.get(state_key, merged["state_modes"][state_key])
            if mode_key in LIGHT_RING_MODES:
                merged["state_modes"][state_key] = mode_key

    return merged


def load_light_ring_settings() -> dict:
    """Load settings from disk, creating the default file on first run."""
    if not LIGHT_RING_CONFIG_PATH.exists():
        save_light_ring_settings(DEFAULT_LIGHT_RING_SETTINGS)
        return deepcopy(DEFAULT_LIGHT_RING_SETTINGS)

    try:
        data = json.loads(LIGHT_RING_CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        save_light_ring_settings(DEFAULT_LIGHT_RING_SETTINGS)
        return deepcopy(DEFAULT_LIGHT_RING_SETTINGS)

    normalized = normalize_light_ring_settings(data)
    if normalized != data:
        save_light_ring_settings(normalized)
    return normalized


def save_light_ring_settings(settings: dict) -> Path:
    """Persist settings to disk and return the file path."""
    normalized = normalize_light_ring_settings(settings)
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    LIGHT_RING_CONFIG_PATH.write_text(
        json.dumps(normalized, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return LIGHT_RING_CONFIG_PATH
