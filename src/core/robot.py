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
    
    @staticmethod
    def _fix_j4_j6(angles: List[float]) -> List[float]:
        """修正 J4 和 J6 的互换问题
        用户输入: [J1, J2, J3, J4(小臂), J5, J6(末端)]
        固件实际: [J1, J2, J3, J6(末端), J5, J4(小臂)]
        """
        fixed = angles.copy()
        fixed[3], fixed[5] = angles[5], angles[3]
        return fixed

    @staticmethod
    def _unfix_j4_j6(angles: List[float]) -> List[float]:
        """从固件读数转回用户坐标系"""
        fixed = angles.copy()
        fixed[3], fixed[5] = angles[5], angles[3]
        return fixed

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
                angles = self._unfix_j4_j6(angles)
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
            fixed_angles = self._fix_j4_j6(angles)
            cmd_str = f">{','.join(str(int(a)) for a in fixed_angles)},{speed}"
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
        进入示教模式（电流环模式）
        
        通过发送 !TEACH <current_mA> 命令，主控板会将所有关节切换到电流模式。
        注意：固件通常接收整数单位的 mA，因此内部会将 A 转换为 mA。
        
        Args:
            current_limit: 电流限制（A），推荐值 1.0~2.5
                          1.0A = 较轻，可轻松拖动
                          1.5A = 适中（默认）
                          2.5A = 较重，接近正常工作负载
        
        Returns:
            bool: 是否成功
        """
        if not self.connected:
            return False
        try:
            # 固件通常期望 mA 整数（例如 1500 表示 1.5A）
            current_mA = int(current_limit * 1000)
            cmd = f"!TEACH {current_mA}"
            with self.lock:
                self.ser.reset_input_buffer()
                self.ser.write((cmd + '\n').encode())
                time.sleep(0.3)
                response = self.ser.read(self.ser.in_waiting).decode().strip()
            
            success = "Teach mode ON" in response or "OK" in response.upper() or response.startswith("ok")
            if not success and not response:
                # 部分固件不返回响应，认为成功
                success = True
            
            self.enabled = False  # 示教模式下标记为未使能
            return success
        except Exception as e:
            print(f"进入示教模式失败: {e}")
            # 降级方案：直接禁用电机（无助力但可拖动）
            try:
                self.disable()
                return True
            except:
                return False
    
    def exit_teach_mode(self) -> bool:
        """
        退出示教模式，恢复位置控制
        
        发送 !TEACHOFF 命令，主控板会将当前位置设为目标位置并保持。
        """
        if not self.connected:
            return False
        try:
            # 发送退出示教模式命令
            with self.lock:
                self.ser.write(b'!TEACHOFF\n')
                time.sleep(0.2)
                self.ser.read(self.ser.in_waiting)
            
            self.enabled = True
            return True
        except Exception as e:
            print(f"退出示教模式失败: {e}")
            # 降级方案：直接使能
            return self.enable()
