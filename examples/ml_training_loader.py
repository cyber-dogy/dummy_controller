#!/usr/bin/env python3
"""
ML训练数据加载示例
演示如何加载示教轨迹数据用于强化学习/模仿学习
"""

import json
import csv
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader


class TrajectoryDataset(Dataset):
    """
    轨迹数据集 - 用于PyTorch训练
    
    支持:
    - 行为克隆 (Behavior Cloning)
    - 模仿学习 (Imitation Learning)
    - 强化学习预训练
    """
    
    def __init__(self, json_file: str, seq_len: int = 10, predict_next: int = 1):
        """
        Args:
            json_file: 轨迹JSON文件路径
            seq_len: 输入序列长度（历史帧数）
            predict_next: 预测未来帧数
        """
        self.seq_len = seq_len
        self.predict_next = predict_next
        
        # 加载数据
        with open(json_file, 'r') as f:
            data = json.load(f)
        
        # 提取轨迹
        trajectory = data['trajectory']
        self.timestamps = np.array([p['timestamp'] for p in trajectory])
        self.angles = np.array([p['angles'] for p in trajectory])  # [N, 6]
        
        # 计算速度和加速度（如果未提供）
        self.velocities = self._compute_velocity()
        self.accelerations = self._compute_acceleration()
        
        # 归一化参数
        self.angle_mean = np.mean(self.angles, axis=0)
        self.angle_std = np.std(self.angles, axis=0) + 1e-8
        
        print(f"[Dataset] 加载 {len(self.angles)} 帧轨迹数据")
        print(f"[Dataset] 角度范围: [{self.angles.min():.2f}, {self.angles.max():.2f}]")
    
    def _compute_velocity(self) -> np.ndarray:
        """计算关节速度"""
        dt = np.diff(self.timestamps, prepend=self.timestamps[0])
        dt = np.maximum(dt, 0.001)  # 避免除零
        velocity = np.diff(self.angles, axis=0, prepend=self.angles[:1]) / dt[:, None]
        return velocity
    
    def _compute_acceleration(self) -> np.ndarray:
        """计算关节加速度"""
        dt = np.diff(self.timestamps, prepend=self.timestamps[0])
        dt = np.maximum(dt, 0.001)
        acc = np.diff(self.velocities, axis=0, prepend=self.velocities[:1]) / dt[:, None]
        return acc
    
    def normalize_angles(self, angles: np.ndarray) -> np.ndarray:
        """归一化角度"""
        return (angles - self.angle_mean) / self.angle_std
    
    def denormalize_angles(self, angles_norm: np.ndarray) -> np.ndarray:
        """反归一化角度"""
        return angles_norm * self.angle_std + self.angle_mean
    
    def __len__(self) -> int:
        return max(0, len(self.angles) - self.seq_len - self.predict_next + 1)
    
    def __getitem__(self, idx: int) -> dict:
        """
        获取一个训练样本
        
        Returns:
            dict: {
                'obs': [seq_len, 6] 历史角度,
                'vel': [seq_len, 6] 历史速度,
                'target': [predict_next, 6] 目标角度
            }
        """
        # 输入序列
        obs = self.angles[idx:idx + self.seq_len]
        vel = self.velocities[idx:idx + self.seq_len]
        
        # 目标（下一帧或多帧）
        target_start = idx + self.seq_len
        target_end = target_start + self.predict_next
        target = self.angles[target_start:target_end]
        
        return {
            'obs': torch.FloatTensor(self.normalize_angles(obs)),
            'vel': torch.FloatTensor(vel / 1000.0),  # 缩放速度
            'target': torch.FloatTensor(self.normalize_angles(target))
        }


class CSVLoader:
    """CSV格式轨迹加载器（更简单的格式）"""
    
    @staticmethod
    def load(csv_file: str) -> np.ndarray:
        """
        加载CSV轨迹
        
        Returns:
            np.ndarray: [N, 7] array (timestamp + 6 joints)
        """
        data = []
        with open(csv_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'):
                    continue
                try:
                    data.append([float(x) for x in row])
                except ValueError:
                    continue
        
        return np.array(data)
    
    @staticmethod
    def split_by_keypoints(csv_file: str, keyframe_indices: list) -> list:
        """
        按关键帧分割轨迹
        
        Args:
            csv_file: CSV文件路径
            keyframe_indices: 关键帧索引列表
        
        Returns:
            list: 分段后的轨迹列表
        """
        data = CSVLoader.load(csv_file)
        segments = []
        
        for i in range(len(keyframe_indices) - 1):
            start = keyframe_indices[i]
            end = keyframe_indices[i + 1]
            segment = data[start:end]
            segments.append(segment)
        
        return segments


def demo_behavior_cloning():
    """
    行为克隆训练示例
    """
    print("=" * 50)
    print("行为克隆训练示例")
    print("=" * 50)
    
    # 加载数据
    dataset = TrajectoryDataset('trajectory_example.json', seq_len=5, predict_next=1)
    dataloader = DataLoader(dataset, batch_size=4, shuffle=True)
    
    # 简单的策略网络
    class PolicyNetwork(torch.nn.Module):
        def __init__(self, input_dim: int = 12, hidden_dim: int = 64, output_dim: int = 6):
            super().__init__()
            self.net = torch.nn.Sequential(
                torch.nn.Linear(input_dim * 5, hidden_dim),  # 5帧历史
                torch.nn.ReLU(),
                torch.nn.Linear(hidden_dim, hidden_dim),
                torch.nn.ReLU(),
                torch.nn.Linear(hidden_dim, output_dim)
            )
        
        def forward(self, obs_seq, vel_seq):
            # obs_seq: [B, 5, 6], vel_seq: [B, 5, 6]
            x = torch.cat([obs_seq, vel_seq], dim=-1)  # [B, 5, 12]
            x = x.reshape(x.size(0), -1)  # [B, 60]
            return self.net(x)
    
    # 训练
    model = PolicyNetwork()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    criterion = torch.nn.MSELoss()
    
    print("\n训练开始...")
    for epoch in range(3):
        total_loss = 0
        for batch in dataloader:
            obs = batch['obs']  # [B, 5, 6]
            vel = batch['vel']  # [B, 5, 6]
            target = batch['target'][:, 0, :]  # [B, 6]
            
            pred = model(obs, vel)
            loss = criterion(pred, target)
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
        
        avg_loss = total_loss / len(dataloader)
        print(f"Epoch {epoch+1}, Loss: {avg_loss:.6f}")
    
    print("\n✅ 训练完成！模型可以预测下一帧关节角度。")
    print("=" * 50)


def demo_statistics():
    """轨迹统计分析示例"""
    print("\n" + "=" * 50)
    print("轨迹统计分析")
    print("=" * 50)
    
    # 加载CSV数据
    data = CSVLoader.load('trajectory_example.csv')
    timestamps = data[:, 0]
    angles = data[:, 1:]
    
    print(f"\n轨迹统计:")
    print(f"  总时长: {timestamps[-1]:.3f}秒")
    print(f"  总帧数: {len(angles)}")
    print(f"  平均帧率: {len(angles) / timestamps[-1]:.1f} FPS")
    
    print(f"\n关节角度范围:")
    joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    for i, name in enumerate(joint_names):
        print(f"  {name}: [{angles[:, i].min():.2f}°, {angles[:, i].max():.2f}°]")
    
    # 计算运动平滑度（加速度变化率）
    velocities = np.diff(angles, axis=0) / np.diff(timestamps)[:, None]
    accelerations = np.diff(velocities, axis=0) / np.diff(timestamps[:-1])[:, None]
    jerk = np.diff(accelerations, axis=0)
    
    print(f"\n运动平滑度 (Jerk):")
    print(f"  平均: {np.abs(jerk).mean():.2f} °/s³")
    print(f"  最大: {np.abs(jerk).max():.2f} °/s³")
    print("=" * 50)


if __name__ == '__main__':
    import os
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    demo_statistics()
    demo_behavior_cloning()
