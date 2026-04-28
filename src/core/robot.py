"""机械臂控制核心"""

import serial
import time
import threading
from copy import deepcopy
from typing import List, Optional, Callable, Tuple

from core.gripper import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    GripperController,
    normalize_gripper_value,
    gripper_state_label,
)
from core.light_ring import LightRingController
from core.ref_native_usb import RefNativeUsbLink
from utils.light_config import (
    LIGHT_RING_STATE_LABELS,
    LIGHT_RING_STATE_PRIORITY,
    load_light_ring_settings,
    normalize_light_ring_settings,
    save_light_ring_settings,
)


class DummyRobot:
    """Dummy V2 机械臂控制器"""

    MOVE_LIGHT_HOLD_SECONDS = 0.8
    
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.connected = False
        self.enabled = False
        self.current_angles = [0.0] * 6
        self.target_angles = [0.0] * 6
        self.current_gripper = GRIPPER_CLOSED
        self.target_gripper = GRIPPER_CLOSED
        self.gripper_placeholder_mode = True
        self.on_position_update: Optional[Callable] = None
        self.ref_native_link = RefNativeUsbLink("dummy-controller")
        self.light_ring = LightRingController(self.ref_native_link)
        self.gripper = GripperController(self._send_gripper_ascii)
        self._light_state_lock = threading.Lock()
        self._moving_timer: Optional[threading.Timer] = None
        self._active_light_state = "disconnected"
        self._light_settings = load_light_ring_settings()
        self._light_runtime_flags = {
            "moving": False,
            "teach": False,
            "recording": False,
            "playback": False,
            "estop": False,
            "error": False,
        }
        self.light_ring.set_enabled(self._light_settings["enabled"])
        self.light_ring.set_brightness(self._light_settings["brightness"])
        self._apply_light_state(force=True)
        self.position_read_timeout_s = 0.35
        self.position_poll_min_interval_s = 0.25
        self.position_error_backoff_s = 0.8
        self.position_after_move_quiet_s = 0.25
        self._last_position_read_s = 0.0
        self._position_fail_until_s = 0.0
        self._suspend_position_poll_until_s = 0.0
        
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """连接串口"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=2)
            time.sleep(0.2)
            with self.lock:
                self.ser.reset_input_buffer()
            self.connected = True
            self._clear_action_error()
            self._set_runtime_state("estop", False)
            self._apply_light_state(force=True)
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            self.connected = False
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.connected:
            try:
                self.gripper.disable()
            except Exception as e:
                print(f"夹爪断使能失败: {e}")
        if self.ser:
            with self.lock:
                self.ser.close()
            self.ser = None
        self.connected = False
        self.enabled = False
        self._cancel_moving_timer()
        with self._light_state_lock:
            for key in self._light_runtime_flags:
                self._light_runtime_flags[key] = False
        self._apply_light_state(force=True)
    
    def enable(self) -> bool:
        """使能电机"""
        if not self.connected:
            return False
        try:
            with self.lock:
                self.ser.write(b'!START\n')
                time.sleep(0.3)
                self.ser.read(self.ser.in_waiting)
            self.enabled = True
            self._clear_action_error()
            self._set_runtime_state("estop", False)
            self._set_runtime_state("teach", False)
            self._apply_light_state(force=True)
            return True
        except Exception as e:
            print(f"使能失败: {e}")
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
            return False
    
    def disable(self):
        """禁用电机"""
        if not self.connected:
            return
        try:
            self.gripper.disable()
            with self.lock:
                self.ser.write(b'!DISABLE\n')
            self.enabled = False
            self._clear_action_error()
            self._cancel_moving_timer()
            self._set_runtime_state("moving", False)
            self._set_runtime_state("playback", False)
            self._apply_light_state(force=True)
        except Exception as e:
            print(f"禁用失败: {e}")
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
    
    def emergency_stop(self):
        """急停"""
        try:
            if self.connected:
                with self.lock:
                    self.ser.write(b'!STOP\n')
                self.gripper.disable()
        except Exception as e:
            print(f"急停失败: {e}")
            self._set_runtime_state("error", True)
        self._cancel_moving_timer()
        self._set_runtime_state("moving", False)
        self._set_runtime_state("playback", False)
        self._set_runtime_state("estop", True)
        self._apply_light_state(force=True)
    
    def get_position(self) -> Optional[List[float]]:
        """获取当前位置"""
        if not self.connected:
            return None
        now = time.monotonic()
        if now < self._position_fail_until_s or now < self._suspend_position_poll_until_s:
            return None
        if now - self._last_position_read_s < self.position_poll_min_interval_s:
            return None
        self._last_position_read_s = now

        try:
            lines: List[str] = []
            with self.lock:
                if self.ser is None:
                    return None
                old_timeout = self.ser.timeout
                self.ser.timeout = 0.04
                try:
                    self.ser.write(b'#GETJPOS\n')
                    self.ser.flush()
                    deadline = time.monotonic() + self.position_read_timeout_s
                    while time.monotonic() < deadline:
                        raw = self.ser.readline()
                        if not raw:
                            continue
                        line = raw.decode(errors='replace').strip()
                        if line:
                            lines.append(line)
                            parts = line.split()
                            if len(parts) >= 7 and parts[0] == 'ok':
                                break
                finally:
                    self.ser.timeout = old_timeout

            for line in reversed(lines):
                parts = line.split()
                if len(parts) >= 7 and parts[0] == 'ok':
                    angles = [float(parts[i + 1]) for i in range(6)]
                    self.current_angles = angles
                    if self.on_position_update:
                        self.on_position_update(angles)
                    self._clear_action_error()
                    return angles
            if lines:
                print(f"读取位置失败: 未收到完整关节位置响应: {' | '.join(lines[-3:])}")
        except (serial.SerialException, OSError) as e:
            self._position_fail_until_s = time.monotonic() + self.position_error_backoff_s
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
            print(f"读取位置失败: {e}")
        except Exception as e:
            print(f"读取位置失败: {e}")
        return None

    def _send_gripper_ascii(self, cmd: str, wait_s: float = 0.12) -> List[str]:
        """Send one line-rail gripper ASCII command through the main CDC serial."""
        if not self.connected or self.ser is None:
            raise RuntimeError("机械臂串口未连接")

        with self.lock:
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()
            time.sleep(max(0.0, float(wait_s)))
            raw = self.ser.read(self.ser.in_waiting).decode(errors='replace')

        return [line.strip() for line in raw.splitlines() if line.strip()]
    
    def move_to(self, angles: List[float], speed: int = 30, 
                check_limits: bool = True) -> Tuple[bool, str]:
        """
        运动到指定角度，自动限制在关节限位内
        
        Args:
            angles: 目标角度列表 [J1, J2, J3, J4, J5, J6]
            speed: 运动速度
            check_limits: 是否检查关节限位
        
        Returns:
            (success, message): 是否成功，以及状态信息
        """
        if not self.connected:
            return False, "未连接"
        
        if not self.enabled:
            return False, "电机未使能"
        
        violations = []
        
        # 检查并限制角度范围
        if check_limits:
            from utils.config import JointLimitChecker, JOINT_LIMITS
            original_angles = angles.copy()
            angles = JointLimitChecker.clamp_angles(angles)
            
            # 检查是否有被限制的角度
            for i in range(6):
                if abs(original_angles[i] - angles[i]) > 0.1:
                    min_val, max_val = JOINT_LIMITS[i]
                    violations.append(f"J{i+1}: {original_angles[i]:.1f}° → {angles[i]:.1f}° (限位[{min_val}, {max_val}])")
            
            if violations:
                print(f"[警告] 角度已限制到合法范围:")
                for v in violations:
                    print(f"  {v}")
        
        try:
            cmd_str = f">{','.join(str(int(a)) for a in angles)},{speed}"
            print(f"[DEBUG] move_to: {cmd_str}")  # 调试用
            with self.lock:
                self.ser.write((cmd_str + '\n').encode())
                self.ser.flush()
            self._suspend_position_poll_until_s = time.monotonic() + self.position_after_move_quiet_s
            self.target_angles = list(angles[:6])
            
            msg = "运动指令已发送"
            if violations:
                msg += f" (已限制{len(violations)}个关节)"
            self._clear_action_error()
            self._pulse_moving_state()
            return True, msg
            
        except Exception as e:
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
            return False, f"运动指令失败: {e}"

    def set_gripper(self, value: float) -> Tuple[bool, str]:
        """Set gripper state. Compatible with legacy 0-30 angle values."""
        return self.set_gripper_state(value)

    def set_gripper_state(self, state: object) -> Tuple[bool, str]:
        """Set the line-rail gripper to closed(0) or open(1)."""
        value = normalize_gripper_value(state)
        if self.gripper_placeholder_mode:
            self.current_gripper = value
            self.target_gripper = value
            self._clear_action_error()
            return True, f"夹爪占位状态已更新为{gripper_state_label(value)}（未下发硬件）"

        ok, msg = self.gripper.set_state(value)
        if ok:
            self.current_gripper = value
            self.target_gripper = value
            self._clear_action_error()
            return True, msg

        self._set_runtime_state("error", True)
        self._apply_light_state(force=True)
        return False, msg

    def open_gripper(self) -> Tuple[bool, str]:
        return self.set_gripper_state(GRIPPER_OPEN)

    def close_gripper(self) -> Tuple[bool, str]:
        return self.set_gripper_state(GRIPPER_CLOSED)

    def disable_gripper(self) -> Tuple[bool, str]:
        ok, msg = self.gripper.disable()
        if ok:
            self._clear_action_error()
        else:
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
        return ok, msg

    def configure_gripper(self, **kwargs):
        return self.gripper.configure(**kwargs)

    def refresh_gripper_hold(self) -> Tuple[bool, str]:
        return self.gripper.refresh_hold()

    def get_gripper_status_snapshot(self) -> dict:
        snapshot = self.gripper.get_status_snapshot()
        snapshot["placeholder_mode"] = self.gripper_placeholder_mode
        snapshot["current_value"] = self.current_gripper
        snapshot["target_value"] = self.target_gripper
        return snapshot

    def get_gripper(self) -> float:
        """Return the latest known normalized gripper state (0=closed, 1=open)."""
        return float(self.current_gripper)

    def get_target_angles(self) -> List[float]:
        """Return the latest commanded joint target."""
        return list(self.target_angles)

    def get_target_gripper(self) -> float:
        """Return the latest commanded gripper target."""
        return float(self.target_gripper)

    def set_gripper_placeholder_mode(self, enabled: bool) -> Tuple[bool, str]:
        """Enable or disable gripper placeholder mode."""
        self.gripper_placeholder_mode = bool(enabled)
        mode = "占位记录模式" if self.gripper_placeholder_mode else "真实硬件模式"
        return True, f"夹爪已切换到{mode}"

    def is_gripper_placeholder_mode(self) -> bool:
        """Whether the gripper currently runs in placeholder mode."""
        return bool(self.gripper_placeholder_mode)
    
    def move_j(self, angles: List[float], speed: int = 30) -> Tuple[bool, str]:
        """关节运动（兼容接口）"""
        return self.move_to(angles, speed)
    
    def check_joint_limits(self, angles: List[float]) -> Tuple[bool, List[str]]:
        """
        检查关节角度是否在限位内
        
        Args:
            angles: 6个关节的角度
        
        Returns:
            (is_valid, violations): 是否全部合法，违规信息列表
        """
        from utils.config import JointLimitChecker
        violations = JointLimitChecker.get_violations(angles)
        is_valid = len(violations) == 0
        
        violation_msgs = []
        for joint_idx, angle, min_val, max_val in violations:
            violation_msgs.append(
                f"J{joint_idx+1}: {angle:.1f}° 超出范围 [{min_val}, {max_val}]"
            )
        
        return is_valid, violation_msgs
    
    def clamp_angles(self, angles: List[float]) -> List[float]:
        """
        将角度限制在合法范围内
        
        Args:
            angles: 原始角度
        
        Returns:
            限制后的角度
        """
        from utils.config import JointLimitChecker
        return JointLimitChecker.clamp_angles(angles)
    
    def send_command(self, cmd: str) -> bool:
        """发送原始命令"""
        if not self.connected:
            return False
        try:
            with self.lock:
                self.ser.write((cmd + '\n').encode())
                self.ser.flush()
            self._clear_action_error()
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
            return False

    def set_joint_current_limit(self, joint: int, current_a: float) -> bool:
        """设置指定关节电流限制。node 7 夹爪不要走这个通用关节接口。"""
        joint = int(joint)
        if joint < 1 or joint > 6:
            print(f"非法关节号: {joint}")
            return False
        current_a = max(0.05, min(3.0, float(current_a)))
        return self.send_command(f"#I_LIMIT_J {joint} {current_a:.3f}")

    def set_joint_speed_limit(self, joint: int, speed: float) -> bool:
        """设置指定关节速度上限。"""
        joint = int(joint)
        if joint < 1 or joint > 6:
            print(f"非法关节号: {joint}")
            return False
        speed = max(1.0, min(200.0, float(speed)))
        return self.send_command(f"#SPEED_J {joint} {speed:.3f}")

    def set_joint_acceleration(self, joint: int, acceleration: float) -> bool:
        """设置指定关节加速度。"""
        joint = int(joint)
        if joint < 1 or joint > 6:
            print(f"非法关节号: {joint}")
            return False
        acceleration = max(1.0, min(200.0, float(acceleration)))
        return self.send_command(f"#ACC_J {joint} {acceleration:.3f}")
    
    def set_dce_kp(self, joint: int, val: int) -> bool:
        """设置指定关节的 DCE Kp"""
        return self.send_command(f"#SET_DCE_KP {joint} {val}")
    
    def set_dce_ki(self, joint: int, val: int) -> bool:
        """设置指定关节的 DCE Ki"""
        return self.send_command(f"#SET_DCE_KI {joint} {val}")
    
    def set_dce_kd(self, joint: int, val: int) -> bool:
        """设置指定关节的 DCE Kd"""
        return self.send_command(f"#SET_DCE_KD {joint} {val}")
    
    def set_dce_kv(self, joint: int, val: int) -> bool:
        """设置指定关节的 DCE Kv"""
        return self.send_command(f"#SET_DCE_KV {joint} {val}")
    
    def reboot_joint(self, joint: int) -> bool:
        """重启指定关节的电机驱动板"""
        return self.send_command(f"#REBOOT {joint}")
    
    def reboot(self) -> bool:
        """重启主控板"""
        return self.send_command("!REBOOT")
    
    def enter_teach_mode(self, current_limit: float = 1.5) -> bool:
        """
        进入示教模式：发送 !DISABLE，停止位置控制（MODE_STOP）。
        步进电机降低 Kp 无效（驱动芯片仍施加保持电流），必须完全禁用才能拖动。
        剩余阻力仅为减速器机械摩擦，与断电拖动手感一致。
        """
        if not self.connected:
            return False
        try:
            with self.lock:
                self.ser.write(b'!DISABLE\n')
                time.sleep(0.3)
                self.ser.read(self.ser.in_waiting)
            self.enabled = False
            self._clear_action_error()
            self._cancel_moving_timer()
            self._set_runtime_state("moving", False)
            self._set_runtime_state("teach", True)
            self._apply_light_state(force=True)
            return True
        except Exception as e:
            print(f"进入示教模式失败: {e}")
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
            return False

    def exit_teach_mode(self) -> bool:
        """
        退出示教模式：读当前位置 → !START 使能 → 立即保持当前位置。
        必须先保位，否则电机上电会猛地回到上次记忆的目标位置。
        """
        if not self.connected:
            return False
        try:
            # 1. 在禁用状态下读当前位置（编码器仍有效）
            current_pos = self.get_position()

            # 2. 使能电机
            with self.lock:
                self.ser.write(b'!START\n')
                time.sleep(0.3)
                self.ser.read(self.ser.in_waiting)
            self.enabled = True

            # 3. 立即保持当前位置，防止突然跳位
            if current_pos:
                self.move_to(current_pos, speed=10, check_limits=False)

            self._clear_action_error()
            self._set_runtime_state("teach", False)
            self._apply_light_state(force=True)
            return True
        except Exception as e:
            print(f"退出示教模式失败: {e}")
            self._set_runtime_state("error", True)
            self._apply_light_state(force=True)
            return False

    def get_light_settings(self) -> dict:
        """Return a copy of the current light-ring settings."""
        return deepcopy(self._light_settings)

    def update_light_settings(self, settings: dict, persist: bool = False) -> Tuple[bool, str]:
        """Update and optionally persist light-ring settings."""
        self._light_settings = normalize_light_ring_settings(settings)
        if persist:
            path = save_light_ring_settings(self._light_settings)
            prefix = f"灯环设置已保存到 {path}"
        else:
            prefix = "灯环设置已更新"

        self.light_ring.set_enabled(self._light_settings["enabled"])
        self.light_ring.set_brightness(self._light_settings["brightness"])
        ok, msg = self._apply_light_state(force=True)
        snapshot = self.light_ring.get_status_snapshot()
        if ok and snapshot.get("brightness_supported") is False:
            return True, f"{prefix}；亮度配置已保存，重刷支持亮度接口的 REF 固件后生效"
        if ok:
            return True, prefix
        return False, f"{prefix}，但应用当前状态失败: {msg}"

    def set_runtime_state(self, state_key: str, active: bool = True):
        """Expose runtime state changes to GUI flows such as record/playback."""
        self._set_runtime_state(state_key, active)
        self._apply_light_state(force=True)

    def preview_light_state(self, state_key: str, settings: Optional[dict] = None) -> Tuple[bool, str]:
        """Preview the light effect mapped to a state without changing runtime flags."""
        normalized = normalize_light_ring_settings(settings or self._light_settings)
        mode_key = normalized["state_modes"].get(state_key, "off")
        self.light_ring.set_enabled(normalized["enabled"])
        self.light_ring.set_brightness(normalized["brightness"])
        return self.light_ring.apply_mode_key(mode_key, force=True)

    def restore_light_state(self) -> Tuple[bool, str]:
        """Re-apply the current automatic light state."""
        self.light_ring.set_enabled(self._light_settings["enabled"])
        self.light_ring.set_brightness(self._light_settings["brightness"])
        return self._apply_light_state(force=True)

    def get_light_status_snapshot(self) -> dict:
        """Expose light-ring status for the UI."""
        snapshot = self.light_ring.get_status_snapshot()
        snapshot["active_state"] = self._active_light_state
        snapshot["active_state_label"] = LIGHT_RING_STATE_LABELS.get(
            self._active_light_state, self._active_light_state
        )
        return snapshot

    def _pulse_moving_state(self):
        self._set_runtime_state("moving", True)
        self._apply_light_state(force=True)
        self._cancel_moving_timer()
        self._moving_timer = threading.Timer(
            self.MOVE_LIGHT_HOLD_SECONDS, self._clear_moving_state
        )
        self._moving_timer.daemon = True
        self._moving_timer.start()

    def _clear_moving_state(self):
        self._set_runtime_state("moving", False)
        self._apply_light_state(force=True)

    def _cancel_moving_timer(self):
        if self._moving_timer is not None:
            self._moving_timer.cancel()
            self._moving_timer = None

    def _set_runtime_state(self, state_key: str, active: bool):
        if state_key not in self._light_runtime_flags:
            return
        with self._light_state_lock:
            self._light_runtime_flags[state_key] = bool(active)

    def _clear_action_error(self):
        self._set_runtime_state("error", False)

    def _compute_active_light_state(self) -> str:
        base_flags = {
            "disconnected": not self.connected,
            "connected": self.connected and not self.enabled,
            "enabled": self.connected and self.enabled,
        }

        with self._light_state_lock:
            runtime_flags = dict(self._light_runtime_flags)

        for state_key in LIGHT_RING_STATE_PRIORITY:
            if state_key in runtime_flags and runtime_flags[state_key]:
                return state_key
            if base_flags.get(state_key, False):
                return state_key

        return "disconnected"

    def _apply_light_state(self, force: bool = False) -> Tuple[bool, str]:
        active_state = self._compute_active_light_state()
        self._active_light_state = active_state
        mode_key = self._light_settings["state_modes"].get(active_state, "off")
        return self.light_ring.apply_mode_key(mode_key, force=force)
