"""机械臂控制核心"""

import serial
import time
import threading
from typing import List, Optional, Callable, Tuple


class DummyRobot:
    """Dummy V2 机械臂控制器"""
    
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.connected = False
        self.enabled = False
        self.current_angles = [0.0] * 6
        self.on_position_update: Optional[Callable] = None
        
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """连接串口"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=2)
            time.sleep(0.2)
            with self.lock:
                self.ser.reset_input_buffer()
            self.connected = True
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.ser:
            with self.lock:
                self.ser.close()
            self.ser = None
        self.connected = False
        self.enabled = False
    
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
            return True
        except Exception as e:
            print(f"使能失败: {e}")
            return False
    
    def disable(self):
        """禁用电机"""
        if not self.connected:
            return
        try:
            with self.lock:
                self.ser.write(b'!DISABLE\n')
            self.enabled = False
        except Exception as e:
            print(f"禁用失败: {e}")
    
    def emergency_stop(self):
        """急停"""
        if self.connected:
            with self.lock:
                self.ser.write(b'!STOP\n')
    
    def get_position(self) -> Optional[List[float]]:
        """获取当前位置"""
        if not self.connected:
            return None
        try:
            with self.lock:
                self.ser.reset_input_buffer()
                self.ser.write(b'#GETJPOS\n')
                time.sleep(0.15)
                response = self.ser.read(self.ser.in_waiting).decode()
            
            parts = response.strip().split()
            if len(parts) >= 7 and parts[0] == 'ok':
                angles = [float(parts[i+1]) for i in range(6)]
                self.current_angles = angles
                if self.on_position_update:
                    self.on_position_update(angles)
                return angles
        except Exception as e:
            print(f"读取位置失败: {e}")
        return None
    
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
            
            msg = "运动指令已发送"
            if violations:
                msg += f" (已限制{len(violations)}个关节)"
            return True, msg
            
        except Exception as e:
            return False, f"运动指令失败: {e}"
    
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
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False
    
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
            return True
        except Exception as e:
            print(f"进入示教模式失败: {e}")
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

            return True
        except Exception as e:
            print(f"退出示教模式失败: {e}")
            return False
