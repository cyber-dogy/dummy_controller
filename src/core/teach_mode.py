#!/usr/bin/env python3
"""
示教功能核心模块
支持拖动示教、轨迹记录、平滑处理、保存导出
"""

import time
import json
import csv
import numpy as np
from datetime import datetime
from typing import List, Dict, Optional, Callable, Tuple
from dataclasses import dataclass, asdict
import threading

from core.kinematics import DummyKinematics
from core.gripper import normalize_gripper_value


@dataclass
class TrajectoryPoint:
    """轨迹点数据结构"""
    timestamp: float          # 时间戳（秒）
    angles: List[float]       # 6轴角度 [J1, J2, J3, J4, J5, J6]
    velocity: Optional[List[float]] = None  # 速度（可选）
    acceleration: Optional[List[float]] = None  # 加速度（可选）
    playback_speed: Optional[int] = None  # 回放关节速度（度/秒）
    target_angles: Optional[List[float]] = None  # 目标角度（GUI 实时跟随录制时）
    gripper: float = 0.0      # 当前夹爪角度
    target_gripper: Optional[float] = None  # 目标夹爪角度
    source: str = "teach"     # 数据来源

    def to_dict(self):
        return {
            'timestamp': self.timestamp,
            'angles': self.angles,
            'velocity': self.velocity,
            'acceleration': self.acceleration,
            'playback_speed': self.playback_speed,
            'target_angles': self.target_angles,
            'gripper': normalize_gripper_value(self.gripper),
            'target_gripper': (
                normalize_gripper_value(self.target_gripper)
                if self.target_gripper is not None
                else None
            ),
            'source': self.source,
        }

    def playback_angles(self) -> List[float]:
        """回放优先使用目标轨迹；没有目标时退回观测轨迹。"""
        return list(self.target_angles if self.target_angles is not None else self.angles)

    def playback_gripper(self) -> float:
        return normalize_gripper_value(
            self.target_gripper if self.target_gripper is not None else self.gripper
        )

    def playback_speed_value(self, default_speed: int = 30) -> int:
        speed = self.playback_speed if self.playback_speed is not None else default_speed
        return max(1, int(speed))


class TeachMode:
    """示教模式控制器"""

    DEFAULT_SAMPLE_INTERVAL = 0.1   # 默认采样间隔 100ms（10Hz），与串口延迟匹配
    MAX_TRAJECTORY_POINTS = 5000

    def __init__(self, robot):
        self.robot = robot
        self.recording = False
        self.trajectory: List[TrajectoryPoint] = []
        self.record_thread = None
        self.sample_interval = self.DEFAULT_SAMPLE_INTERVAL
        self.on_record_callback: Optional[Callable] = None
        self._action_provider: Optional[Callable[[], Optional[Dict[str, object]]]] = None
        self._record_source = "teach"
        self.kinematics = DummyKinematics()
        
    def enter_teach_mode(self, current_limit: float = 0.5) -> bool:
        """
        进入示教模式（电流环模式）
        
        Args:
            current_limit: 电流限制（A），0.5A=可轻松拖动
        
        Returns:
            bool: 是否成功
        """
        return self.robot.enter_teach_mode(current_limit)
    
    def exit_teach_mode(self) -> bool:
        """退出示教模式，恢复位置控制"""
        return self.robot.exit_teach_mode()

    def set_tcp_offset_mm(self, offset_mm: float):
        self.kinematics.set_tcp_offset_mm(offset_mm)

    def get_tcp_offset_mm(self) -> float:
        return self.kinematics.get_tcp_offset_mm()

    def set_world_settings(self, settings: Dict[str, object]):
        self.kinematics.set_world_settings(settings)

    def get_world_settings(self) -> Dict[str, object]:
        return self.kinematics.get_world_settings()
    
    def start_recording(self,
                        action_provider: Optional[Callable[[], Optional[Dict[str, object]]]] = None,
                        source: str = "teach"):
        """开始记录轨迹"""
        if self.recording:
            return
        
        self._action_provider = action_provider
        self._record_source = source
        self.recording = True
        self.trajectory = []
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.record_thread.start()
    
    def stop_recording(self) -> List[TrajectoryPoint]:
        """停止记录，返回轨迹数据"""
        self.recording = False
        if self.record_thread:
            self.record_thread.join(timeout=1.0)
        
        return self.trajectory.copy()
    
    def _record_loop(self):
        """记录循环（后台线程）"""
        start_time = time.time()
        consecutive_errors = 0

        while self.recording:
            t0 = time.time()
            try:
                angles = self.robot.get_position()

                if angles:
                    consecutive_errors = 0
                    obs_gripper = normalize_gripper_value(
                        self.robot.get_gripper() if hasattr(self.robot, "get_gripper") else 0.0
                    )
                    target_angles = None
                    target_gripper = None

                    if self._action_provider:
                        try:
                            action_payload = self._action_provider() or {}
                            payload_angles = action_payload.get("angles")
                            if payload_angles is not None:
                                target_angles = list(payload_angles[:6])
                            payload_gripper = action_payload.get("gripper")
                            if payload_gripper is not None:
                                target_gripper = normalize_gripper_value(payload_gripper)
                        except Exception as exc:
                            print(f"[示教] 读取 action_provider 失败: {exc}")

                    point = TrajectoryPoint(
                        timestamp=t0 - start_time,
                        angles=angles,
                        target_angles=target_angles,
                        gripper=normalize_gripper_value(obs_gripper),
                        target_gripper=target_gripper,
                        source=self._record_source,
                    )
                    self.trajectory.append(point)

                    if self.on_record_callback:
                        self.on_record_callback(point)

                    if len(self.trajectory) >= self.MAX_TRAJECTORY_POINTS:
                        print(f"[示教] 达到最大轨迹点数 {self.MAX_TRAJECTORY_POINTS}，停止记录")
                        self.recording = False
                        break
                else:
                    consecutive_errors += 1
                    if consecutive_errors > 5:
                        print("[示教] 连续5次读取位置失败，停止记录")
                        self.recording = False
                        break

            except Exception as e:
                print(f"[示教] 记录出错: {e}")
                consecutive_errors += 1
                if consecutive_errors > 5:
                    self.recording = False
                    break

            # 控制采样率（扣除本次读取耗时）
            elapsed = time.time() - t0
            sleep_time = self.sample_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _nearest_gripper_pair(self, trajectory: List[TrajectoryPoint], timestamp: float) -> Tuple[float, Optional[float]]:
        if not trajectory:
            return 0.0, None
        nearest = min(trajectory, key=lambda p: abs(float(p.timestamp) - float(timestamp)))
        return (
            normalize_gripper_value(nearest.gripper),
            (
                normalize_gripper_value(nearest.target_gripper)
                if nearest.target_gripper is not None
                else None
            ),
        )
    
    def smooth_trajectory_adaptive(self, trajectory: List[TrajectoryPoint],
                                   strength: int = 5) -> List[TrajectoryPoint]:
        """
        自适应平滑：专门针对拖动示教卡顿轨迹。
        步骤：
          1. 去除停滞段（连续相同位置，用户没有移动时产生的冗余采样）
          2. Gaussian 加权移动平均（比均匀平均更平滑且边缘不失真）
          3. 三次样条重采样到均匀时间步长

        Args:
            trajectory: 原始轨迹
            strength: 平滑强度 1-10（1=轻微，10=极度平滑）
        """
        if len(trajectory) < 4:
            return trajectory

        # --- 步骤 1：去停滞段 ---
        # 相邻点最大角度变化 < 0.3° 认为是停滞，合并为一个点
        STALL_THRESH = 0.3
        deduped = [trajectory[0]]
        for pt in trajectory[1:]:
            prev = deduped[-1]
            max_delta = max(abs(a - b) for a, b in zip(pt.angles, prev.angles))
            if max_delta >= STALL_THRESH:
                deduped.append(pt)
        # 始终保留最后一个点
        if deduped[-1] is not trajectory[-1]:
            deduped.append(trajectory[-1])

        if len(deduped) < 4:
            return deduped

        # --- 步骤 2：Gaussian 加权平均 ---
        # 窗口大小由 strength 决定：strength=1 → window=3，strength=10 → window=21
        window = max(3, strength * 2 + 1)
        # 构建 Gaussian 核
        sigma = window / 6.0
        half = window // 2
        kernel = np.array([
            np.exp(-0.5 * ((i - half) / sigma) ** 2)
            for i in range(window)
        ])
        kernel /= kernel.sum()

        angles_arr = np.array([p.angles for p in deduped])  # (N, 6)
        smoothed_angles = np.zeros_like(angles_arr)
        N = len(deduped)

        for j in range(6):
            col = angles_arr[:, j]
            # 用边缘反射填充，避免边缘失真
            padded = np.pad(col, half, mode='reflect')
            smoothed_angles[:, j] = np.convolve(padded, kernel, mode='valid')[:N]

        # --- 步骤 3：均匀时间重采样（100ms 间隔） ---
        times = np.array([p.timestamp for p in deduped])
        new_times = np.arange(0, times[-1], self.sample_interval)

        result = []
        for t in new_times:
            interp_angles = [
                float(np.interp(t, times, smoothed_angles[:, j]))
                for j in range(6)
            ]
            gripper, target_gripper = self._nearest_gripper_pair(deduped, float(t))
            result.append(TrajectoryPoint(
                timestamp=float(t),
                angles=interp_angles,
                gripper=gripper,
                target_gripper=target_gripper,
                source=deduped[0].source,
            ))

        # 补上最后一个点（避免截断）
        if result[-1].timestamp < times[-1] - 0.01:
            gripper, target_gripper = self._nearest_gripper_pair(deduped, float(times[-1]))
            result.append(TrajectoryPoint(
                timestamp=float(times[-1]),
                angles=smoothed_angles[-1].tolist(),
                gripper=gripper,
                target_gripper=target_gripper,
                source=deduped[-1].source,
            ))

        return result

    def smooth_trajectory(self, trajectory: List[TrajectoryPoint],
                          window_size: int = 5) -> List[TrajectoryPoint]:
        """
        轨迹平滑处理（移动平均滤波），并确保在限位内
        
        Args:
            trajectory: 原始轨迹
            window_size: 平滑窗口大小
        
        Returns:
            平滑后且在限位内的轨迹
        """
        from utils.config import JointLimitChecker
        
        if len(trajectory) < window_size:
            # 即使不平滑也要限制范围
            return [
                TrajectoryPoint(
                    timestamp=p.timestamp,
                    angles=JointLimitChecker.clamp_angles(p.angles),
                    velocity=p.velocity,
                    acceleration=p.acceleration,
                    playback_speed=p.playback_speed,
                    target_angles=p.target_angles,
                    gripper=normalize_gripper_value(p.gripper),
                    target_gripper=(
                        normalize_gripper_value(p.target_gripper)
                        if p.target_gripper is not None
                        else None
                    ),
                    source=p.source,
                )
                for p in trajectory
            ]
        
        smoothed = []
        angles_array = np.array([p.angles for p in trajectory])
        
        # 对每个关节分别平滑
        smoothed_angles = np.zeros_like(angles_array)
        for i in range(6):
            smoothed_angles[:, i] = np.convolve(
                angles_array[:, i], 
                np.ones(window_size)/window_size, 
                mode='same'
            )
        
        # 构建新的轨迹点，并限制到合法范围
        for idx, point in enumerate(trajectory):
            clamped_angles = JointLimitChecker.clamp_angles(
                smoothed_angles[idx].tolist()
            )
            smoothed.append(TrajectoryPoint(
                timestamp=point.timestamp,
                angles=clamped_angles,
                velocity=point.velocity,
                acceleration=point.acceleration,
                playback_speed=point.playback_speed,
                target_angles=point.target_angles,
                gripper=normalize_gripper_value(point.gripper),
                target_gripper=(
                    normalize_gripper_value(point.target_gripper)
                    if point.target_gripper is not None
                    else None
                ),
                source=point.source,
            ))
        
        return smoothed
    
    def interpolate_trajectory(self, trajectory: List[TrajectoryPoint],
                               target_interval: float = 0.05) -> List[TrajectoryPoint]:
        """
        轨迹插值（统一时间间隔），并确保在限位内
        
        Args:
            trajectory: 原始轨迹
            target_interval: 目标时间间隔（秒）
        
        Returns:
            插值后且在限位内的轨迹
        """
        from utils.config import JointLimitChecker
        
        if len(trajectory) < 2:
            return [
                TrajectoryPoint(
                    timestamp=p.timestamp,
                    angles=JointLimitChecker.clamp_angles(p.angles),
                    playback_speed=p.playback_speed,
                    target_angles=p.target_angles,
                    gripper=normalize_gripper_value(p.gripper),
                    target_gripper=(
                        normalize_gripper_value(p.target_gripper)
                        if p.target_gripper is not None
                        else None
                    ),
                    source=p.source,
                )
                for p in trajectory
            ]
        
        # 提取时间和角度（先用原始轨迹的限制版本）
        times = np.array([p.timestamp for p in trajectory])
        angles = np.array([
            JointLimitChecker.clamp_angles(p.angles) 
            for p in trajectory
        ])
        
        # 生成统一的时间点
        duration = times[-1]
        new_times = np.arange(0, duration, target_interval)
        
        # 对每个关节插值
        new_angles = np.zeros((len(new_times), 6))
        for i in range(6):
            new_angles[:, i] = np.interp(new_times, times, angles[:, i])
        
        # 构建新轨迹，并限制到合法范围
        interpolated = []
        for idx, t in enumerate(new_times):
            clamped_angles = JointLimitChecker.clamp_angles(
                new_angles[idx].tolist()
            )
            gripper, target_gripper = self._nearest_gripper_pair(trajectory, float(t))
            interpolated.append(TrajectoryPoint(
                timestamp=t,
                angles=clamped_angles,
                gripper=gripper,
                target_gripper=target_gripper,
                source=trajectory[0].source,
            ))
        
        return interpolated
    
    def decimate_trajectory(self, trajectory: List[TrajectoryPoint], 
                           tolerance: float = 1.0) -> List[TrajectoryPoint]:
        """
        轨迹稀疏化 - 使用Ramer-Douglas-Peucker算法减少点数
        
        减少轨迹点数以降低CAN总线负载，同时保持轨迹形状
        
        Args:
            trajectory: 原始轨迹
            tolerance: 简化容差（度），越大点数越少
        
        Returns:
            稀疏化后的轨迹
        """
        if len(trajectory) <= 2:
            return trajectory
        
        def perpendicular_distance(point, line_start, line_end):
            """计算点到线段的垂直距离"""
            if np.allclose(line_start, line_end):
                return np.linalg.norm(point - line_start)
            
            line_vec = line_end - line_start
            point_vec = point - line_start
            line_len_sq = np.dot(line_vec, line_vec)
            
            # 投影参数
            t = max(0, min(1, np.dot(point_vec, line_vec) / line_len_sq))
            projection = line_start + t * line_vec
            
            return np.linalg.norm(point - projection)
        
        def rdp(points, epsilon):
            """Ramer-Douglas-Peucker算法"""
            if len(points) <= 2:
                return points
            
            # 找到距离线段最远的点
            first_point = points[0]
            last_point = points[-1]
            
            max_dist = 0
            max_idx = 0
            
            for i in range(1, len(points) - 1):
                dist = perpendicular_distance(points[i], first_point, last_point)
                if dist > max_dist:
                    max_dist = dist
                    max_idx = i
            
            # 如果最大距离大于容差，递归简化
            if max_dist > epsilon:
                left = rdp(points[:max_idx + 1], epsilon)
                right = rdp(points[max_idx:], epsilon)
                return np.vstack([left[:-1], right])
            else:
                return np.array([first_point, last_point])
        
        # 将轨迹转换为numpy数组
        points = np.array([p.angles for p in trajectory])
        
        # 应用RDP算法
        simplified = rdp(points, tolerance)
        
        # 重建轨迹点列表
        result = []
        for angles in simplified:
            # 找到最近的时间戳
            idx = np.argmin(np.linalg.norm(points - angles, axis=1))
            result.append(TrajectoryPoint(
                timestamp=trajectory[idx].timestamp,
                angles=angles.tolist(),
                playback_speed=trajectory[idx].playback_speed,
                target_angles=trajectory[idx].target_angles,
                gripper=normalize_gripper_value(trajectory[idx].gripper),
                target_gripper=(
                    normalize_gripper_value(trajectory[idx].target_gripper)
                    if trajectory[idx].target_gripper is not None
                    else None
                ),
                source=trajectory[idx].source,
            ))
        
        return result
    
    def optimize_for_can_bus(self, trajectory: List[TrajectoryPoint],
                            max_points_per_second: float = 20.0) -> List[TrajectoryPoint]:
        """
        优化轨迹以适应CAN总线带宽
        
        Args:
            trajectory: 原始轨迹
            max_points_per_second: 最大点数/秒（默认20Hz，即50ms间隔）
        
        Returns:
            优化后的轨迹
        """
        if not trajectory:
            return trajectory
        
        duration = trajectory[-1].timestamp - trajectory[0].timestamp
        current_rate = len(trajectory) / duration if duration > 0 else 0
        
        print(f"[CAN优化] 原始轨迹: {len(trajectory)}点, "
              f"{duration:.2f}s, "
              f"{current_rate:.1f}点/秒")
        
        if current_rate <= max_points_per_second:
            print(f"[CAN优化] 轨迹频率正常，无需优化")
            return trajectory
        
        # 需要优化
        # 1. 先尝试RDP稀疏化
        tolerance = 0.5
        while tolerance <= 5.0:
            simplified = self.decimate_trajectory(trajectory, tolerance)
            new_rate = len(simplified) / duration if duration > 0 else 0
            
            if new_rate <= max_points_per_second:
                print(f"[CAN优化] 使用RDP稀疏化(容差{tolerance}°): "
                      f"{len(trajectory)} -> {len(simplified)}点, "
                      f"{new_rate:.1f}点/秒")
                return simplified
            
            tolerance += 0.5
        
        # 2. 如果RDP不够，使用均匀降采样
        target_count = int(duration * max_points_per_second)
        step = max(1, len(trajectory) // target_count)
        
        downsampled = trajectory[::step]
        # 确保包含最后一个点
        if downsampled[-1] != trajectory[-1]:
            downsampled.append(trajectory[-1])
        
        new_rate = len(downsampled) / duration if duration > 0 else 0
        print(f"[CAN优化] 使用均匀降采样(步长{step}): "
              f"{len(trajectory)} -> {len(downsampled)}点, "
              f"{new_rate:.1f}点/秒")
        
        return downsampled
    
    def save_trajectory(self, trajectory: List[TrajectoryPoint], 
                       filepath: str, 
                       format: str = 'json',
                       metadata: Optional[Dict] = None):
        """
        保存轨迹到文件
        
        Args:
            trajectory: 轨迹数据
            filepath: 文件路径
            format: 格式 ('json' 或 'csv')
            metadata: 元数据（名称、描述等）
        """
        metadata = metadata or {}
        data = {
            'metadata': {
                **metadata,
                'schema_version': 'dummy_lerobot_ready_v1',
                'state_encoding': 'xyz_mm + rot6d(6) + binary_gripper_open',
                'action_encoding': 'xyz_mm + rot6d(6) + binary_gripper_open',
                'gripper_encoding': '0.0=closed/clamped, 1.0=open',
                'kinematics_available': self.kinematics.available,
                'tcp_offset_mm': self.kinematics.get_tcp_offset_mm(),
                'world_settings': self.kinematics.get_world_settings(),
            },
            'created_at': datetime.now().isoformat(),
            'duration': trajectory[-1].timestamp if trajectory else 0,
            'num_points': len(trajectory),
            'joint_names': ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'],
            'trajectory': [self._serialize_point(p) for p in trajectory]
        }
        
        if format == 'json':
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        
        elif format == 'csv':
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['# Dummy Robot Trajectory (LeRobot-ready)'])
                writer.writerow([f'# Created: {data["created_at"]}'])
                writer.writerow([f'# Duration: {data["duration"]:.3f}s'])
                writer.writerow([f'# Points: {data["num_points"]}'])
                writer.writerow(['# observation.state/action = [x_mm, y_mm, z_mm, rot6d(6), gripper_open_binary]'])
                writer.writerow([])
                header = (
                    ['timestamp', 'source', 'playback_speed', 'observation.gripper', 'action.gripper']
                    + [f'observation.joint.J{i+1}' for i in range(6)]
                    + [f'action.joint.J{i+1}' for i in range(6)]
                    + [f'observation.state.{i}' for i in range(10)]
                    + [f'action.{i}' for i in range(10)]
                )
                writer.writerow(header)
                for p in trajectory:
                    row = self._serialize_point(p)
                    obs_state = row.get('observation.state') or [None] * 10
                    action_state = row.get('action') or [None] * 10
                    action_angles = row.get('action.joints') or row['angles']
                    action_gripper = row.get('target_gripper')
                    if action_gripper is None:
                        action_gripper = row.get('gripper', 0.0)
                    writer.writerow(
                        [f'{p.timestamp:.6f}', row.get('source', 'teach'),
                         row.get('playback_speed') or '',
                         f"{row.get('gripper', 0.0):.4f}",
                         f"{float(action_gripper):.4f}"]
                        + [f'{a:.4f}' for a in row['angles']]
                        + [f'{a:.4f}' for a in action_angles]
                        + [self._fmt_csv_value(v) for v in obs_state]
                        + [self._fmt_csv_value(v) for v in action_state]
                    )

    def load_trajectory(self, filepath: str, 
                       validate_limits: bool = True) -> Optional[List[TrajectoryPoint]]:
        """
        从文件加载轨迹，并进行边界验证
        
        Args:
            filepath: 文件路径
            validate_limits: 是否验证关节限位
        
        Returns:
            轨迹点列表，或在加载失败时返回None
        """
        from utils.config import JointLimitChecker
        
        try:
            if filepath.lower().endswith('.csv'):
                return self._load_csv_trajectory(filepath, validate_limits=validate_limits)

            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            trajectory = []
            violations_count = 0
            
            for p in data['trajectory']:
                angles = p.get('angles') or p.get('observation.joints')
                if angles is None:
                    continue
                target_angles = p.get('target_angles') or p.get('action.joints')
                gripper = self._extract_gripper_value(
                    p.get('gripper'),
                    p.get('observation.state'),
                )
                target_gripper = self._extract_gripper_value(
                    p.get('target_gripper'),
                    p.get('action'),
                )
                
                # 验证并限制角度
                if validate_limits:
                    clamped_angles = JointLimitChecker.clamp_angles(angles)
                    if not JointLimitChecker.is_valid(angles):
                        violations_count += 1
                    angles = clamped_angles
                    if target_angles is not None:
                        target_angles = JointLimitChecker.clamp_angles(target_angles)
                
                trajectory.append(TrajectoryPoint(
                    timestamp=p['timestamp'],
                    angles=angles,
                    velocity=p.get('velocity'),
                    acceleration=p.get('acceleration'),
                    playback_speed=p.get('playback_speed'),
                    target_angles=target_angles,
                    gripper=normalize_gripper_value(gripper),
                    target_gripper=normalize_gripper_value(target_gripper),
                    source=p.get('source', 'loaded'),
                ))
            
            if violations_count > 0:
                print(f"[警告] 加载的轨迹中有 {violations_count} 个点超出关节限位，已自动限制")
            
            return trajectory
        except Exception as e:
            print(f"加载轨迹失败: {e}")
            return None
    
    def validate_trajectory(self, trajectory: List[TrajectoryPoint]) -> Tuple[bool, List[str]]:
        """
        验证轨迹是否全部在关节限位内
        
        Args:
            trajectory: 轨迹数据
        
        Returns:
            (is_valid, violations): 是否全部合法，违规信息列表
        """
        from utils.config import JointLimitChecker
        
        violations = []
        for idx, point in enumerate(trajectory):
            point_violations = JointLimitChecker.get_violations(point.angles)
            for joint_idx, angle, min_val, max_val in point_violations:
                violations.append(
                    f"点{idx} (t={point.timestamp:.2f}s): "
                    f"J{joint_idx+1}={angle:.1f}° 超出 [{min_val}, {max_val}]"
                )
        
        return len(violations) == 0, violations
    
    def fix_trajectory_limits(self, trajectory: List[TrajectoryPoint],
                             smooth: bool = True) -> List[TrajectoryPoint]:
        """
        修复轨迹中的超限点，可选平滑处理
        
        Args:
            trajectory: 原始轨迹
            smooth: 是否进行平滑处理
        
        Returns:
            修复后的轨迹
        """
        from utils.config import JointLimitChecker, smooth_clamp_trajectory
        
        # 首先限制所有点
        fixed = [
            TrajectoryPoint(
                timestamp=p.timestamp,
                angles=JointLimitChecker.clamp_angles(p.angles),
                velocity=p.velocity,
                acceleration=p.acceleration,
                playback_speed=p.playback_speed,
                target_angles=p.target_angles,
                gripper=normalize_gripper_value(p.gripper),
                target_gripper=(
                    normalize_gripper_value(p.target_gripper)
                    if p.target_gripper is not None
                    else None
                ),
                source=p.source,
            )
            for p in trajectory
        ]
        
        # 如果需要，进行平滑处理
        if smooth and len(fixed) > 5:
            angles_only = [p.angles for p in fixed]
            smoothed_angles = smooth_clamp_trajectory(angles_only, window_size=5)
            
            fixed = [
                TrajectoryPoint(
                    timestamp=fixed[i].timestamp,
                    angles=smoothed_angles[i],
                    velocity=fixed[i].velocity,
                    acceleration=fixed[i].acceleration,
                    playback_speed=fixed[i].playback_speed,
                    target_angles=fixed[i].target_angles,
                    gripper=fixed[i].gripper,
                    target_gripper=fixed[i].target_gripper,
                    source=fixed[i].source,
                )
                for i in range(len(fixed))
            ]
        
        return fixed
    
    def playback_trajectory(self, trajectory: List[TrajectoryPoint],
                           speed_ratio: float = 1.0,
                           on_progress: Optional[Callable] = None,
                           check_limits: bool = True,
                           optimize_can: bool = True):
        """
        回放轨迹，自动处理边界限制和CAN总线保护
        
        Args:
            trajectory: 轨迹数据
            speed_ratio: 速度比例（1.0=原速，0.5=半速，2.0=倍速）
            on_progress: 进度回调函数(percentage)
            check_limits: 是否检查关节限位
            optimize_can: 是否优化CAN总线负载
        """
        from utils.config import JointLimitChecker
        
        if not trajectory:
            return
        
        # CAN总线优化：如果点太多，先稀疏化
        if optimize_can:
            trajectory = self.optimize_for_can_bus(trajectory)
        
        # 先限制所有轨迹点
        if check_limits:
            valid_trajectory = []
            violations_total = 0
            for point in trajectory:
                clamped = JointLimitChecker.clamp_angles(point.angles)
                if not JointLimitChecker.is_valid(point.angles):
                    violations_total += 1
                valid_trajectory.append(TrajectoryPoint(
                    timestamp=point.timestamp,
                    angles=clamped,
                    target_angles=point.target_angles,
                    gripper=normalize_gripper_value(point.gripper),
                    target_gripper=(
                        normalize_gripper_value(point.target_gripper)
                        if point.target_gripper is not None
                        else None
                    ),
                    source=point.source,
                ))
            
            if violations_total > 0:
                print(f"[警告] 轨迹中有 {violations_total} 个点超出关节限位，已自动限制")
            
            trajectory = valid_trajectory
        
        # 获取当前位置用于平滑插值
        current_pos = self.robot.get_position()
        if current_pos and check_limits:
            current_pos = JointLimitChecker.clamp_angles(current_pos)
        
        # CAN总线保护参数
        min_tx_interval = 0.05 / speed_ratio  # 最小发送间隔（考虑速度比例）
        last_tx_time = 0
        tx_count = 0
        skip_count = 0
        last_gripper: Optional[float] = None
        
        print(f"[CAN回放] 开始回放 {len(trajectory)} 点，速度 {speed_ratio}x")
        
        for idx, point in enumerate(trajectory):
            current_time = time.time()
            
            # 限流保护：如果距离上次发送时间太短，跳过此点
            if current_time - last_tx_time < min_tx_interval:
                skip_count += 1
                continue
            
            # 如果是第一个点，进行平滑过渡
            if idx == 0 and current_pos:
                # 检查是否需要插入过渡点
                playback_angles = point.playback_angles()
                max_diff = max(abs(a - b) for a, b in zip(playback_angles, current_pos))
                if max_diff > 10:  # 如果差距大于10度，插入过渡
                    transition = JointLimitChecker.interpolate_to_limit(
                        current_pos, playback_angles, steps=5
                    )
                    for t_angles in transition[:-1]:  # 不执行最后一个点（会被下面执行）
                        self.robot.move_to(t_angles, check_limits=False)
                        time.sleep(0.05)
            
            # 运动到目标点
            success, msg = self.robot.move_to(point.playback_angles(), check_limits=False)
            if not success:
                self.stats['can_errors'] += 1
                print(f"[警告] 第 {idx} 点发送失败: {msg}")
            else:
                tx_count += 1

            if hasattr(self.robot, "set_gripper_state"):
                target_gripper = normalize_gripper_value(point.playback_gripper())
                if last_gripper is None or abs(last_gripper - target_gripper) >= 1e-6:
                    self.robot.set_gripper_state(target_gripper)
                    last_gripper = target_gripper
                elif target_gripper < 0.5 and hasattr(self.robot, "refresh_gripper_hold"):
                    self.robot.refresh_gripper_hold()
            
            last_tx_time = time.time()
            
            # 计算等待时间
            if idx < len(trajectory) - 1:
                next_time = trajectory[idx + 1].timestamp
                point_time = point.timestamp
                wait_time = (next_time - point_time) / speed_ratio
                time.sleep(max(0.01, wait_time))
            
            # 回调进度
            if on_progress:
                progress = (idx + 1) / len(trajectory) * 100
                on_progress(progress)
        
        print(f"[CAN回放] 完成: 发送 {tx_count} 点, 跳过 {skip_count} 点")
        if skip_count > 0:
            print(f"[CAN回放] 提示: 跳过的点过多，建议降低回放速度或减少轨迹点数")

    def _serialize_point(self, point: TrajectoryPoint) -> Dict[str, object]:
        action_angles = point.playback_angles()
        gripper = normalize_gripper_value(point.gripper)
        action_gripper = normalize_gripper_value(point.playback_gripper())
        obs_pose = self.kinematics.compute_pose(point.angles, gripper)
        action_pose = self.kinematics.compute_pose(action_angles, action_gripper)

        return {
            'timestamp': float(point.timestamp),
            'source': point.source,
            'angles': [float(a) for a in point.angles],
            'target_angles': [float(a) for a in point.target_angles] if point.target_angles is not None else None,
            'gripper': gripper,
            'target_gripper': action_gripper if point.target_gripper is not None else None,
            'velocity': point.velocity,
            'acceleration': point.acceleration,
            'playback_speed': point.playback_speed,
            'observation.joints': [float(a) for a in point.angles],
            'action.joints': [float(a) for a in action_angles],
            'observation.state': obs_pose.state_vector() if obs_pose else None,
            'action': action_pose.state_vector() if action_pose else None,
            'observation.eef': obs_pose.to_dict() if obs_pose else None,
            'action.eef': action_pose.to_dict() if action_pose else None,
        }

    def _load_csv_trajectory(self, filepath: str, validate_limits: bool = True) -> Optional[List[TrajectoryPoint]]:
        from utils.config import JointLimitChecker

        rows: List[TrajectoryPoint] = []
        with open(filepath, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(
                line for line in f
                if line.strip() and not self._is_csv_comment_line(line)
            )

            for row in reader:
                if not row:
                    continue

                if 'observation.joint.J1' in row:
                    angles = [float(row[f'observation.joint.J{i+1}']) for i in range(6)]
                    target_angles = [
                        float(row.get(f'action.joint.J{i+1}') or row[f'observation.joint.J{i+1}'])
                        for i in range(6)
                    ]
                    playback_speed = int(float(row.get('playback_speed') or 0)) or None
                    gripper = normalize_gripper_value(row.get('observation.gripper') or 0.0)
                    target_gripper = normalize_gripper_value(row.get('action.gripper') or gripper)
                    source = row.get('source') or 'loaded_csv'
                else:
                    angles = [float(row[f'J{i+1}']) for i in range(6)]
                    target_angles = None
                    playback_speed = None
                    gripper = 0.0
                    target_gripper = None
                    source = 'legacy_csv'

                if validate_limits:
                    angles = JointLimitChecker.clamp_angles(angles)
                    if target_angles is not None:
                        target_angles = JointLimitChecker.clamp_angles(target_angles)

                rows.append(TrajectoryPoint(
                    timestamp=float(row['timestamp']),
                    angles=angles,
                    playback_speed=playback_speed,
                    target_angles=target_angles,
                    gripper=normalize_gripper_value(gripper),
                    target_gripper=(
                        normalize_gripper_value(target_gripper)
                        if target_gripper is not None
                        else None
                    ),
                    source=source,
                ))

        return rows

    def _extract_gripper_value(self, direct_value, state_vector) -> float:
        if direct_value is not None:
            return normalize_gripper_value(direct_value)
        if isinstance(state_vector, list) and len(state_vector) >= 10:
            return normalize_gripper_value(state_vector[9])
        return 0.0

    def _fmt_csv_value(self, value) -> str:
        if value is None:
            return ''
        return f'{float(value):.6f}'

    def _is_csv_comment_line(self, line: str) -> bool:
        stripped = line.lstrip()
        return stripped.startswith('#') or stripped.startswith('"#')
