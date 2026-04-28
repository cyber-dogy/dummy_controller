"""Line-rail gripper control over the REF ASCII serial protocol."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple


GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 1.0


def normalize_gripper_value(value: object, default: float = GRIPPER_CLOSED) -> float:
    """Normalize legacy angle values and new binary values to 0=closed, 1=open."""
    if value is None:
        return float(default)
    try:
        v = float(value)
    except (TypeError, ValueError):
        return float(default)

    # Legacy GUI used 0-30 degrees, where 30 meant open.
    if v > 1.0:
        return GRIPPER_OPEN if v >= 15.0 else GRIPPER_CLOSED
    return GRIPPER_OPEN if v >= 0.5 else GRIPPER_CLOSED


def gripper_state_name(value: object) -> str:
    return "open" if normalize_gripper_value(value) >= 0.5 else "closed"


def gripper_state_label(value: object) -> str:
    return "打开" if gripper_state_name(value) == "open" else "夹紧"


@dataclass
class GripperSettings:
    """Runtime settings for the line-rail gripper current-control sequence."""

    open_current_a: float = 0.70
    close_current_a: float = 0.70
    hold_current_a: float = 0.25
    open_pulse_s: float = 1.00
    close_pulse_s: float = 0.50
    hold_after_close: bool = True
    max_hold_s: float = 60.0
    hold_refresh_s: float = 5.0

    def normalized(self) -> "GripperSettings":
        return GripperSettings(
            open_current_a=_clamp(self.open_current_a, 0.05, 1.20),
            close_current_a=_clamp(self.close_current_a, 0.05, 1.20),
            hold_current_a=_clamp(self.hold_current_a, 0.0, 0.80),
            open_pulse_s=_clamp(self.open_pulse_s, 0.05, 2.50),
            close_pulse_s=_clamp(self.close_pulse_s, 0.05, 2.50),
            hold_after_close=bool(self.hold_after_close),
            max_hold_s=_clamp(self.max_hold_s, 1.0, 300.0),
            hold_refresh_s=_clamp(self.hold_refresh_s, 1.0, 30.0),
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


class GripperController:
    """
    Current-mode controller for the line-rail gripper firmware.

    Firmware commands used:
      !HAND_I <ampere>, !HAND_EN, !HAND_O, !HAND_C, !HAND_DIS

    High current is applied only for a bounded motion pulse. If configured, the
    closed state then switches to a lower holding current and an auto-disable
    watchdog.
    """

    def __init__(self, command_fn: Callable[[str, float], List[str]]):
        self._command_fn = command_fn
        self._lock = threading.RLock()
        self._settings = GripperSettings()
        self._target_value = GRIPPER_CLOSED
        self._last_value = GRIPPER_CLOSED
        self._last_error = ""
        self._enabled = False
        self._holding = False
        self._hold_deadline: Optional[float] = None
        self._last_hold_refresh = 0.0
        self._hold_timer: Optional[threading.Timer] = None

    def configure(self, **kwargs) -> GripperSettings:
        with self._lock:
            data = self._settings.__dict__.copy()
            data.update(kwargs)
            self._settings = GripperSettings(**data).normalized()
            if self._holding:
                self._schedule_hold_timeout_locked()
            return self._settings

    def get_settings(self) -> GripperSettings:
        with self._lock:
            return GripperSettings(**self._settings.__dict__)

    def set_state(self, state: object) -> Tuple[bool, str]:
        target = normalize_gripper_value(state)
        if target >= 0.5:
            return self.open()
        return self.close()

    def open(self) -> Tuple[bool, str]:
        settings = self.get_settings()
        with self._lock:
            self._target_value = GRIPPER_OPEN
            self._cancel_hold_timeout_locked()
            try:
                self._disable_locked()
                self._set_current_locked(settings.open_current_a)
                self._send_locked("!HAND_EN")
                self._enabled = True
                self._send_locked("!HAND_O")
                time.sleep(settings.open_pulse_s)
                self._disable_locked()
                self._last_value = GRIPPER_OPEN
                self._last_error = ""
                return True, (
                    f"夹爪已打开（{settings.open_current_a:.2f}A, "
                    f"{settings.open_pulse_s:.2f}s，已断使能）"
                )
            except Exception as exc:
                self._last_error = str(exc)
                self._best_effort_disable_locked()
                return False, f"夹爪打开失败: {self._last_error}"

    def close(self) -> Tuple[bool, str]:
        settings = self.get_settings()
        with self._lock:
            self._target_value = GRIPPER_CLOSED
            self._cancel_hold_timeout_locked()
            try:
                self._disable_locked()
                self._set_current_locked(settings.close_current_a)
                self._send_locked("!HAND_EN")
                self._enabled = True
                self._send_locked("!HAND_C")
                time.sleep(settings.close_pulse_s)

                if settings.hold_after_close and settings.hold_current_a > 0.0:
                    self._set_current_locked(settings.hold_current_a)
                    self._send_locked("!HAND_C")
                    self._holding = True
                    self._last_hold_refresh = time.monotonic()
                    self._schedule_hold_timeout_locked()
                    hold_msg = (
                        f"转入 {settings.hold_current_a:.2f}A 保持，"
                        f"最长 {settings.max_hold_s:.0f}s"
                    )
                else:
                    self._disable_locked()
                    hold_msg = "已断使能"

                self._last_value = GRIPPER_CLOSED
                self._last_error = ""
                return True, (
                    f"夹爪已夹紧（{settings.close_current_a:.2f}A, "
                    f"{settings.close_pulse_s:.2f}s，{hold_msg}）"
                )
            except Exception as exc:
                self._last_error = str(exc)
                self._best_effort_disable_locked()
                return False, f"夹爪夹紧失败: {self._last_error}"

    def disable(self) -> Tuple[bool, str]:
        with self._lock:
            self._cancel_hold_timeout_locked()
            try:
                self._disable_locked()
                self._last_error = ""
                return True, "夹爪已断使能"
            except Exception as exc:
                self._last_error = str(exc)
                return False, f"夹爪断使能失败: {self._last_error}"

    def refresh_hold(self) -> Tuple[bool, str]:
        with self._lock:
            if not self._holding:
                return True, "夹爪未处于保持状态"

            settings = self._settings
            now = time.monotonic()
            if now - self._last_hold_refresh < settings.hold_refresh_s:
                return True, "夹爪保持仍有效"

            try:
                self._set_current_locked(settings.hold_current_a)
                self._send_locked("!HAND_C")
                self._last_hold_refresh = now
                self._schedule_hold_timeout_locked()
                self._last_error = ""
                return True, f"夹爪保持已刷新（{settings.hold_current_a:.2f}A）"
            except Exception as exc:
                self._last_error = str(exc)
                self._best_effort_disable_locked()
                return False, f"夹爪保持刷新失败: {self._last_error}"

    def get_status_snapshot(self) -> Dict[str, object]:
        with self._lock:
            remaining = None
            if self._hold_deadline is not None:
                remaining = max(0.0, self._hold_deadline - time.monotonic())
            return {
                "enabled": self._enabled,
                "holding": self._holding,
                "target": self._target_value,
                "value": self._last_value,
                "state": gripper_state_name(self._last_value),
                "target_state": gripper_state_name(self._target_value),
                "last_error": self._last_error,
                "hold_remaining_s": remaining,
                "settings": self.get_settings().__dict__,
            }

    def close_link(self):
        self.disable()

    def _send_locked(self, cmd: str, wait_s: float = 0.12) -> List[str]:
        return self._command_fn(cmd, wait_s)

    def _set_current_locked(self, current_a: float):
        self._send_locked(f"!HAND_I {current_a:.3f}")

    def _disable_locked(self):
        self._send_locked("!HAND_DIS")
        self._send_locked("!HAND_DIS")
        self._enabled = False
        self._holding = False
        self._hold_deadline = None

    def _best_effort_disable_locked(self):
        try:
            self._disable_locked()
        except Exception:
            self._enabled = False
            self._holding = False
            self._hold_deadline = None

    def _schedule_hold_timeout_locked(self):
        self._cancel_hold_timeout_locked()
        settings = self._settings
        self._hold_deadline = time.monotonic() + settings.max_hold_s
        self._hold_timer = threading.Timer(settings.max_hold_s, self._hold_timeout)
        self._hold_timer.daemon = True
        self._hold_timer.start()

    def _cancel_hold_timeout_locked(self):
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self._hold_timer = None
        self._hold_deadline = None

    def _hold_timeout(self):
        with self._lock:
            if not self._holding:
                return
            self._best_effort_disable_locked()
            self._last_error = "夹爪保持达到最大时长，已自动断使能"
