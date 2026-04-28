"""Persistent TCP offset configuration for the effective tool center point."""

from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_DIR / "config"
TCP_CONFIG_PATH = CONFIG_DIR / "tcp_settings.json"

DEFAULT_TCP_SETTINGS = {
    "tcp_offset_mm": 75.0,
}


def normalize_tcp_settings(settings: dict | None) -> dict:
    merged = deepcopy(DEFAULT_TCP_SETTINGS)
    if not isinstance(settings, dict):
        return merged

    try:
        offset_mm = float(settings.get("tcp_offset_mm", merged["tcp_offset_mm"]))
    except Exception:
        offset_mm = merged["tcp_offset_mm"]

    merged["tcp_offset_mm"] = max(0.0, min(150.0, offset_mm))
    return merged


def save_tcp_settings(settings: dict) -> Path:
    normalized = normalize_tcp_settings(settings)
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    TCP_CONFIG_PATH.write_text(
        json.dumps(normalized, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return TCP_CONFIG_PATH


def load_tcp_settings() -> dict:
    if not TCP_CONFIG_PATH.exists():
        save_tcp_settings(DEFAULT_TCP_SETTINGS)
        return deepcopy(DEFAULT_TCP_SETTINGS)

    try:
        data = json.loads(TCP_CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        save_tcp_settings(DEFAULT_TCP_SETTINGS)
        return deepcopy(DEFAULT_TCP_SETTINGS)

    normalized = normalize_tcp_settings(data)
    if normalized != data:
        save_tcp_settings(normalized)
    return normalized
