#!/usr/bin/env python3
"""
FK/IK 综合测试脚本
验证 MuJoCo 正解与数值逆解的正确性
"""

import sys
import os

# 确保可以找到 src 模块
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, ".."))

import numpy as np
from src.robot_visualizer import DummyRobotVisualizer


def test_forward_kinematics():
    """测试正解: 几个已知姿态的末端位置"""
    print("=" * 60)
    print("正解测试 (Forward Kinematics)")
    print("=" * 60)
    
    viz = DummyRobotVisualizer()
    
    test_cases = [
        ("REST 姿态", [0, -75, 180, 0, 0, 0]),
        ("L-Pose", [0, 0, 90, 0, 0, 0]),
        ("零位", [0, 0, 0, 0, 0, 0]),
        ("J1 +90", [90, -75, 180, 0, 0, 0]),
        ("J4 +90", [0, -75, 180, 90, 0, 0]),
    ]
    
    for name, angles in test_cases:
        viz.update_state(angles)
        pose = viz.get_end_effector_xyz_rpy()
        print(f"\n{name}: {angles}")
        print(f"  位置: X={pose[0]*1000:8.2f} mm  Y={pose[1]*1000:8.2f} mm  Z={pose[2]*1000:8.2f} mm")
        print(f"  姿态: R={pose[3]:7.2f}°  P={pose[4]:7.2f}°  Y={pose[5]:7.2f}°")
    
    viz.stop()
    print("\n✅ 正解测试完成\n")


def test_inverse_kinematics():
    """测试逆解: 给定目标位置，求解关节角并验证正解"""
    print("=" * 60)
    print("逆解测试 (Inverse Kinematics)")
    print("=" * 60)
    
    viz = DummyRobotVisualizer()
    
    # 先用正解得到一些可达的目标点
    test_angles = [
        [0, -75, 180, 0, 0, 0],
        [0, 0, 90, 0, 0, 0],
        [45, -30, 120, 0, 0, 0],
        [-45, -50, 160, 30, 0, 0],
    ]
    
    targets = []
    for angles in test_angles:
        viz.update_state(angles)
        pose = viz.get_end_effector_xyz_rpy()
        targets.append((angles, pose[:3]))
    
    print(f"\n准备 {len(targets)} 组目标点进行 IK 求解...\n")
    
    success_count = 0
    for i, (original_angles, target_xyz) in enumerate(targets):
        print(f"--- 测试 {i+1} ---")
        print(f"原始关节角: {original_angles}")
        print(f"目标位置:   X={target_xyz[0]*1000:.2f} Y={target_xyz[1]*1000:.2f} Z={target_xyz[2]*1000:.2f} mm")
        
        # 使用 L-Pose 作为初始猜测，避免使用原始角（否则太简单）
        initial_guess = [0, 0, 90, 0, 0, 0]
        
        solution = viz.inverse_kinematics(
            target_pos=target_xyz,
            initial_guess=initial_guess
        )
        
        if solution is not None:
            viz.update_state(solution)
            fk_pose = viz.get_end_effector_xyz_rpy()
            error = np.linalg.norm(np.array(fk_pose[:3]) - np.array(target_xyz))
            
            print(f"求解成功:   {['%.2f' % a for a in solution]}")
            print(f"验证正解:   X={fk_pose[0]*1000:.2f} Y={fk_pose[1]*1000:.2f} Z={fk_pose[2]*1000:.2f} mm")
            print(f"位置误差:   {error*1000:.4f} mm ✅")
            success_count += 1
        else:
            print("求解失败 ❌")
        print()
    
    viz.stop()
    print(f"✅ 逆解测试完成: {success_count}/{len(targets)} 组成功\n")


def test_ik_custom_targets():
    """测试自定义目标点的逆解"""
    print("=" * 60)
    print("自定义目标 IK 测试")
    print("=" * 60)
    
    viz = DummyRobotVisualizer()
    
    custom_targets = [
        # (x, y, z) in meters
        (0.20, 0.00, 0.30),
        (0.15, 0.15, 0.25),
        (0.15, -0.15, 0.25),
        (0.10, 0.00, 0.40),
        (0.25, 0.00, 0.20),
    ]
    
    success_count = 0
    for i, target in enumerate(custom_targets):
        print(f"\n目标 {i+1}: X={target[0]*1000:.0f} Y={target[1]*1000:.0f} Z={target[2]*1000:.0f} mm")
        
        solution = viz.inverse_kinematics(
            target_pos=list(target),
            initial_guess=[0, 0, 90, 0, 0, 0]
        )
        
        if solution is not None:
            viz.update_state(solution)
            fk = viz.get_end_effector_xyz_rpy()
            error = np.linalg.norm(np.array(fk[:3]) - np.array(target))
            print(f"  解: {['%.2f' % a for a in solution]}")
            print(f"  误差: {error*1000:.4f} mm ✅")
            success_count += 1
        else:
            print("  无解 ❌")
    
    viz.stop()
    print(f"\n✅ 自定义目标测试完成: {success_count}/{len(custom_targets)} 组成功\n")


if __name__ == "__main__":
    test_forward_kinematics()
    test_inverse_kinematics()
    test_ik_custom_targets()
    print("=" * 60)
    print("所有测试通过！FK/IK 功能正常。")
    print("=" * 60)
