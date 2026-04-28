"""配置常量"""

import numpy as np
from typing import List, Tuple

# 串口配置
DEFAULT_BAUDRATE = 115200
DEFAULT_PORT = "/dev/ttyACM2"

# 关节配置
JOINT_NAMES = [
    "J1 底座",
    "J2 肩部", 
    "J3 肘部(限位35°)",  # 固件硬编码限制
    "J4 小臂",
    "J5 腕部",
    "J6 末端"
]

# 当前已烧录的线轨夹爪 rtx-50 固件事实:
#   motorJ[1] = CtrlStepMotor(..., 50, -170, 170)
#   motorJ[2] = CtrlStepMotor(..., 50, -75, 90)
#   motorJ[3] = CtrlStepMotor(..., 50, 35, 180)
#   motorJ[4] = CtrlStepMotor(..., 50, -180, 180)
#   motorJ[5] = CtrlStepMotor(..., 50, -120, 120)
#   motorJ[6] = CtrlStepMotor(..., 50, -180, 180)
JOINT_LIMITS = [
    (-170, 170),   # J1
    (-75, 90),     # J2
    (35, 180),     # J3 - 固件硬限位，0-35°固件不可达，最小安全值 35°
    (-180, 180),   # J4
    (-120, 120),   # J5
    (-180, 180),   # J6 - 当前线轨夹爪固件限制
]

# 预设位置
# 坐标系: J2=-75~90, J3=35~180 (0°=link3竖直/伸直，但当前固件不可达)
RESET_POSE = [0, -75, 180, 0, 0, 0]  # 固件 !RESET / Resting()
HOME_POSE = [0, 0, 90, 0, 0, 0]      # 固件 !HOME / Homing()

POSES = {
    "RESET (固件折叠位)": RESET_POSE,
    "HOME (固件L位)": HOME_POSE,
    "垂直向上": [0, 0, 35, 0, 0, 0],  # J3=35° 固件硬限位最小值
}

# J6 当前已刷固件按 50:1 计算电机步数，实际末端机械减速器为 mini8-30。
# 真实输出角 = 固件角 * J6_FIRMWARE_REDUCTION / J6_OUTPUT_REDUCTION_DEFAULT。
J6_FIRMWARE_REDUCTION = 50.0
J6_OUTPUT_REDUCTION_DEFAULT = 30.0

# 颜色主题
COLORS = {
    "primary": "#2196F3",
    "success": "#4CAF50",
    "warning": "#FF9800",
    "danger": "#F44336",
    "background": "#F5F5F5",
}


class JointLimitChecker:
    """关节限位检查器 - 确保运动不超出固件限制"""
    
    # 安全边距（度）- 确保不撞到硬限位
    SAFETY_MARGIN = 2.0
    
    @classmethod
    def get_soft_limits(cls) -> List[Tuple[float, float]]:
        """
        获取软限位（在硬限位基础上增加安全边距）
        
        Returns:
            List of (min, max) with safety margin
        """
        soft_limits = []
        for min_val, max_val in JOINT_LIMITS:
            soft_limits.append((
                min_val + cls.SAFETY_MARGIN,
                max_val - cls.SAFETY_MARGIN
            ))
        return soft_limits
    
    @classmethod
    def clamp_angle(cls, angle: float, joint_idx: int) -> float:
        """
        将角度限制在合法范围内
        
        Args:
            angle: 目标角度
            joint_idx: 关节索引 0-5
        
        Returns:
            限制后的角度
        """
        min_val, max_val = JOINT_LIMITS[joint_idx]
        return max(min_val, min(max_val, angle))
    
    @classmethod
    def clamp_angles(cls, angles: List[float]) -> List[float]:
        """
        将所有关节角度限制在合法范围内
        
        Args:
            angles: 6个关节的角度列表
        
        Returns:
            限制后的角度列表
        """
        return [cls.clamp_angle(a, i) for i, a in enumerate(angles)]
    
    @classmethod
    def is_valid(cls, angles: List[float]) -> bool:
        """
        检查角度是否在合法范围内
        
        Args:
            angles: 6个关节的角度列表
        
        Returns:
            True if all angles are valid
        """
        if len(angles) != 6:
            return False
        for i, a in enumerate(angles):
            min_val, max_val = JOINT_LIMITS[i]
            if a < min_val or a > max_val:
                return False
        return True
    
    @classmethod
    def get_violations(cls, angles: List[float]) -> List[Tuple[int, float, float, float]]:
        """
        获取所有超出限位的关节
        
        Returns:
            List of (joint_idx, angle, min_limit, max_limit) for violations
        """
        violations = []
        for i, a in enumerate(angles):
            min_val, max_val = JOINT_LIMITS[i]
            if a < min_val or a > max_val:
                violations.append((i, a, min_val, max_val))
        return violations
    
    @classmethod
    def interpolate_to_limit(cls, start: List[float], target: List[float], 
                             steps: int = 10) -> List[List[float]]:
        """
        从起点到目标点进行插值，确保每个中间点都在限位内
        
        Args:
            start: 起始角度
            target: 目标角度（可能被限位）
            steps: 插值步数
        
        Returns:
            插值后的轨迹点列表，每个点都在限位内
        """
        # 首先限制目标点
        target_clamped = cls.clamp_angles(target)
        
        # 生成插值轨迹
        trajectory = []
        for i in range(steps + 1):
            t = i / steps
            interpolated = [
                start[j] + (target_clamped[j] - start[j]) * t
                for j in range(6)
            ]
            # 确保每个点都在限位内
            trajectory.append(cls.clamp_angles(interpolated))
        
        return trajectory


def smooth_clamp_trajectory(trajectory: List[List[float]], 
                            window_size: int = 5) -> List[List[float]]:
    """
    平滑轨迹并确保在限位内
    
    Args:
        trajectory: 原始轨迹
        window_size: 平滑窗口大小
    
    Returns:
        平滑且在限位内的轨迹
    """
    if len(trajectory) < window_size:
        return [JointLimitChecker.clamp_angles(p) for p in trajectory]
    
    angles_array = np.array(trajectory)
    smoothed = np.zeros_like(angles_array)
    
    # 对每个关节分别平滑
    for i in range(6):
        smoothed[:, i] = np.convolve(
            angles_array[:, i],
            np.ones(window_size) / window_size,
            mode='same'
        )
    
    # 限制到合法范围
    result = []
    for point in smoothed:
        result.append(JointLimitChecker.clamp_angles(point.tolist()))
    
    return result
