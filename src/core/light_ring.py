"""REF native-USB light-ring transport wrapper."""

from __future__ import annotations

import threading
from typing import Dict, Optional, Tuple

from core.ref_native_usb import RefNativeUsbLink
from utils.light_config import LIGHT_RING_MODES


class LightRingController:
    """Talks to the REF native USB fibre interface for RGB ring control."""

    def __init__(self, link: Optional[RefNativeUsbLink] = None):
        self._lock = threading.Lock()
        self._enabled = True
        self._link = link or RefNativeUsbLink("light-ring")
        self._brightness = 24
        self._last_mode_key: Optional[str] = None
        self._last_brightness: Optional[int] = None
        self._last_error = ""
        self._brightness_supported: Optional[bool] = None

    def set_enabled(self, enabled: bool) -> Tuple[bool, str]:
        self._enabled = bool(enabled)
        if not self._enabled:
            return self.apply_mode_key("off", force=True)
        return True, "灯环联动已启用"

    def set_brightness(self, brightness: int) -> Tuple[bool, str]:
        with self._lock:
            self._brightness = max(0, min(45, int(brightness)))
            if self._brightness_supported is False:
                return True, f"亮度已保存为 {self._brightness}/45（当前固件未暴露亮度接口）"
            return True, f"亮度已设置为 {self._brightness}/45"

    def apply_mode_key(self, mode_key: str, force: bool = False) -> Tuple[bool, str]:
        mode_key = mode_key if mode_key in LIGHT_RING_MODES else "off"

        with self._lock:
            if not self._enabled and mode_key != "off":
                mode_key = "off"

            if (
                not force
                and mode_key == self._last_mode_key
                and self._last_brightness == self._brightness
            ):
                return True, f"灯环保持 {LIGHT_RING_MODES[mode_key]['label']}"

            for _ in range(2):
                try:
                    mode_cfg = LIGHT_RING_MODES[mode_key]

                    def _apply(remote):
                        if self._brightness_supported is not False:
                            try:
                                remote.robot.set_rgb_brightness(int(self._brightness))
                                self._brightness_supported = True
                                self._last_brightness = self._brightness
                            except Exception:
                                self._brightness_supported = False

                        remote.robot.set_rgb_mode(mode_cfg["mode_id"])
                        if mode_cfg["rgb_enabled"]:
                            remote.robot.set_rgb_enable(True)
                        else:
                            remote.robot.set_rgb_enable(False)
                        return True

                    self._link.call(_apply)

                    self._last_mode_key = mode_key
                    self._last_brightness = self._brightness
                    self._last_error = ""
                    return True, f"灯环已切换为 {mode_cfg['label']}"
                except Exception as exc:
                    self._last_error = str(exc)

            return False, f"灯环控制失败: {self._last_error or '未知错误'}"

    def get_status_snapshot(self) -> Dict[str, Optional[str]]:
        with self._lock:
            return {
                "enabled": self._enabled,
                "connected": self._link.is_connected(),
                "last_mode_key": self._last_mode_key,
                "last_mode_label": LIGHT_RING_MODES.get(
                    self._last_mode_key or "", {}
                ).get("label"),
                "brightness": self._brightness,
                "brightness_supported": self._brightness_supported,
                "last_error": self._last_error,
            }

    def close(self):
        with self._lock:
            self._link.close()
