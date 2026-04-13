#!/usr/bin/env python3
"""
MuJoCo 机器人实时可视化模块
用于 Dummy V2 机械臂的 3D 状态显示
"""

import numpy as np
import mujoco
import mujoco.viewer
import threading
import time
from typing import Optional, List, Callable
from dataclasses import dataclass


@dataclass
class RobotState:
    """机器人状态数据结构"""
    joint_angles: List[float]  # 6个关节角度（度）
    timestamp: float = 0.0
    
    @property
    def joint_angles_rad(self) -> List[float]:
        """获取弧度制角度"""
        return [np.deg2rad(a) for a in self.joint_angles]


class DummyRobotVisualizer:
    """
    Dummy V2 机械臂 MuJoCo 可视化器
    
    功能：
    - 实时显示机械臂 3D 模型
    - 显示关节角度
    - 显示末端执行器位置（正解）
    - 支持目标位置预览（逆解）
    """
    
    def __init__(self, model_path: str = None):
        """
        初始化可视化器
        
        Args:
            model_path: MuJoCo XML 模型文件路径
        """
        self.model_path = model_path
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.viewer: Optional[mujoco.viewer.Viewer] = None
        
        # 运行状态
        self.running = False
        self.viewer_thread: Optional[threading.Thread] = None
        
        # 回调函数
        self.on_joint_change: Optional[Callable[[List[float]], None]] = None
        
        # 当前状态
        self.current_state = RobotState([0, -75, 180, 0, 0, 0])
        self.target_state: Optional[RobotState] = None
        
        # 线程锁
        self.lock = threading.Lock()
        
        # 初始化模型
        self._init_model()
    
    def _init_model(self) -> bool:
        """初始化 MuJoCo 模型"""
        try:
            import os
            if self.model_path is None:
                self.model_path = os.path.join(
                    os.path.dirname(__file__),
                    "..", "models", "dummy_robot.xml"
                )
                self.model_path = os.path.abspath(self.model_path)
            
            self.model = mujoco.MjModel.from_xml_path(self.model_path)
            self.data = mujoco.MjData(self.model)
            
            # 设置初始位置（REST位置）
            self._apply_joint_angles([0, -75, 180, 0, 0, 0])
            
            print(f"[MuJoCo] 模型加载成功: {self.model_path}")
            print(f"[MuJoCo] 自由度: {self.model.nv}")
            print(f"[MuJoCo] 关节数: {self.model.njnt}")
            return True
            
        except Exception as e:
            print(f"[MuJoCo] 模型加载失败: {e}")
            return False
    
    def _apply_joint_angles(self, angles_deg: List[float]):
        """
        应用关节角度到模型
        
        Args:
            angles_deg: 6个关节角度（度）
        """
        if self.data is None:
            return
        
        # 转换到弧度
        angles_rad = np.deg2rad(angles_deg)
        
        # 设置关节位置
        for i, angle in enumerate(angles_rad):
            self.data.qpos[i] = angle
        
        # 前向运动学计算
        mujoco.mj_forward(self.model, self.data)
    
    def start(self) -> bool:
        """
        启动可视化窗口（非阻塞）
        
        Returns:
            bool: 是否成功启动
        """
        if self.model is None:
            print("[MuJoCo] 模型未加载，无法启动")
            return False
        
        if self.running:
            print("[MuJoCo] 可视化器已在运行")
            return True
        
        self.running = True
        
        # 启动 viewer 线程
        self.viewer_thread = threading.Thread(target=self._viewer_loop, daemon=True)
        self.viewer_thread.start()
        
        print("[MuJoCo] 可视化器已启动")
        return True
    
    def _viewer_loop(self):
        """Viewer 主循环（在独立线程中运行）"""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            
            while self.running and viewer.is_running():
                with self.lock:
                    # 应用当前状态
                    self._apply_joint_angles(self.current_state.joint_angles)
                    
                    # 如果有目标状态，显示为半透明/不同颜色
                    if self.target_state is not None:
                        # TODO: 显示目标姿态（ghost）
                        pass
                
                # 更新 viewer
                viewer.sync()
                
                # 控制更新频率
                time.sleep(0.02)  # 50Hz
    
    def stop(self):
        """停止可视化器"""
        self.running = False
        if self.viewer_thread:
            self.viewer_thread.join(timeout=1.0)
        print("[MuJoCo] 可视化器已停止")
    
    def update_state(self, joint_angles: List[float]):
        """
        更新机器人当前状态（从实际机器人）
        
        Args:
            joint_angles: 6个关节角度（度）
        """
        with self.lock:
            self.current_state = RobotState(joint_angles.copy())
    
    def set_target(self, joint_angles: List[float]):
        """
        设置目标状态（用于预览逆解结果）
        
        Args:
            joint_angles: 目标关节角度（度）
        """
        with self.lock:
            self.target_state = RobotState(joint_angles.copy())
    
    def clear_target(self):
        """清除目标状态"""
        with self.lock:
            self.target_state = None
    
    def get_end_effector_pose(self) -> Optional[np.ndarray]:
        """
        获取末端执行器位姿（正解）
        
        Returns:
            4x4 变换矩阵 [R | t; 0 | 1]
        """
        if self.data is None:
            return None
        
        # 获取末端 site 的位置和旋转
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        if site_id < 0:
            return None
        
        pos = self.data.site_xpos[site_id]
        rot = self.data.site_xmat[site_id].reshape(3, 3)
        
        # 构建 4x4 变换矩阵
        transform = np.eye(4)
        transform[:3, :3] = rot
        transform[:3, 3] = pos
        
        return transform
    
    def get_end_effector_xyz_rpy(self) -> Optional[List[float]]:
        """
        获取末端执行器位置和姿态角
        
        Returns:
            [x, y, z, roll, pitch, yaw]（米，度）
        """
        transform = self.get_end_effector_pose()
        if transform is None:
            return None
        
        pos = transform[:3, 3]
        rot = transform[:3, :3]
        
        # 从旋转矩阵提取欧拉角
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        pitch = np.arctan2(-rot[2, 0], np.sqrt(rot[2, 1]**2 + rot[2, 2]**2))
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        
        return [
            pos[0], pos[1], pos[2],
            np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)
        ]
    
    def inverse_kinematics(self, target_pos: List[float], 
                          target_quat: Optional[List[float]] = None,
                          initial_guess: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        数值逆解（使用 MuJoCo 的优化求解器）
        
        Args:
            target_pos: 目标位置 [x, y, z]（米）
            target_quat: 目标四元数 [w, x, y, z]（可选）
            initial_guess: 初始猜测角度（度）
        
        Returns:
            6个关节角度（度），失败返回 None
        """
        if self.data is None or self.model is None:
            return None
        
        # 保存当前状态
        qpos_backup = self.data.qpos.copy()
        
        try:
            # 设置初始猜测
            if initial_guess is not None:
                self._apply_joint_angles(initial_guess)
            
            # 创建目标位姿
            target = np.array(target_pos)
            
            # 使用 MuJoCo 的 IK（简化版本，使用最小化距离）
            # 这里使用简单的优化方法
            from scipy.optimize import minimize
            
            def objective(q):
                self.data.qpos[:6] = q
                mujoco.mj_forward(self.model, self.data)
                site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
                current_pos = self.data.site_xpos[site_id]
                return np.sum((current_pos - target)**2)
            
            # 初始值
            q0 = self.data.qpos[:6].copy() if initial_guess is None else np.deg2rad(initial_guess)
            
            # 关节限位
            bounds = [
                (np.deg2rad(-170), np.deg2rad(170)),  # J1
                (np.deg2rad(-75), np.deg2rad(0)),     # J2
                (np.deg2rad(35), np.deg2rad(180)),    # J3
                (np.deg2rad(-170), np.deg2rad(170)),  # J4
                (np.deg2rad(-120), np.deg2rad(120)),  # J5
                (np.deg2rad(-720), np.deg2rad(720)),  # J6
            ]
            
            # 优化
            result = minimize(objective, q0, method='L-BFGS-B', bounds=bounds)
            
            if result.success:
                solution = np.rad2deg(result.x)
                return solution.tolist()
            else:
                return None
                
        except Exception as e:
            print(f"[MuJoCo] IK 求解失败: {e}")
            return None
        finally:
            # 恢复状态
            self.data.qpos[:] = qpos_backup
            mujoco.mj_forward(self.model, self.data)
    
    def is_running(self) -> bool:
        """检查可视化器是否正在运行"""
        return self.running and self.viewer is not None and self.viewer.is_running()


class MujocoWidget:
    """
    PyQt6 集成的 MuJoCo 可视化组件
    使用 Offscreen 渲染 + QLabel 显示
    """
    
    def __init__(self, width: int = 640, height: int = 480):
        self.width = width
        self.height = height
        self.renderer: Optional[mujoco.Renderer] = None
        
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        
        self._init_model()
    
    def _init_model(self):
        """初始化模型"""
        try:
            import os
            model_path = os.path.join(
                os.path.dirname(__file__),
                "..", "models", "dummy_robot.xml"
            )
            model_path = os.path.abspath(model_path)
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            self.renderer = mujoco.Renderer(self.model, self.height, self.width)
            
            # 设置相机
            self.camera = mujoco.MjvCamera()
            self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.camera.trackbodyid = 0  # 跟踪基座
            self.camera.distance = 1.0
            self.camera.azimuth = 135
            self.camera.elevation = -30
            
        except Exception as e:
            print(f"[MuJoCo Widget] 初始化失败: {e}")
    
    def render(self, joint_angles: List[float]) -> Optional[np.ndarray]:
        """
        渲染一帧
        
        Returns:
            RGB 图像数组 (height, width, 3)
        """
        if self.renderer is None or self.data is None:
            return None
        
        # 应用关节角度
        for i, angle in enumerate(np.deg2rad(joint_angles)):
            self.data.qpos[i] = angle
        
        mujoco.mj_forward(self.model, self.data)
        
        # 渲染
        self.renderer.update_scene(self.data, self.camera)
        rgb = self.renderer.render()
        
        return rgb
    
    def close(self):
        """释放资源"""
        if self.renderer:
            self.renderer.close()


# 测试代码
if __name__ == "__main__":
    print("MuJoCo Dummy V2 可视化器测试")
    
    # 创建可视化器
    viz = DummyRobotVisualizer()
    
    # 启动
    if viz.start():
        print("可视化器已启动，按 Ctrl+C 停止")
        
        try:
            # 模拟状态更新
            angles = [0, -75, 180, 0, 0, 0]  # REST
            
            while viz.is_running():
                # 缓慢运动到 L-Pose
                for i in range(100):
                    t = i / 100.0
                    angles[1] = -75 + 75 * t  # J2: -75 -> 0
                    angles[2] = 180 - 90 * t  # J3: 180 -> 90
                    
                    viz.update_state(angles)
                    time.sleep(0.02)
                
                time.sleep(1)
                
                # 回到 REST
                for i in range(100):
                    t = i / 100.0
                    angles[1] = 0 - 75 * t    # J2: 0 -> -75
                    angles[2] = 90 + 90 * t   # J3: 90 -> 180
                    
                    viz.update_state(angles)
                    time.sleep(0.02)
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n停止测试")
        finally:
            viz.stop()
