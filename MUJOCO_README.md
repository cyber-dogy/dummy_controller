# MuJoCo 3D 可视化功能

## 概述

本功能为 Dummy V2 机械臂添加了基于 MuJoCo 的实时 3D 可视化，支持：
- 实时显示机械臂 3D 模型
- 正解计算（显示末端位姿）
- 数值逆解（点击目标位置计算关节角）
- 多视角切换

## 文件结构

```
src/mujoco_viewer/
├── __init__.py              # 模块入口
├── dummy_robot.xml          # MuJoCo 模型文件
├── robot_visualizer.py      # 核心可视化类
└── qt_integration.py        # PyQt6 集成

src/gui/
└── main_window_with_mujoco.py  # 集成版本的主窗口
```

## 安装

```bash
# 安装依赖
pip install mujoco numpy scipy PyQt6 pyserial

# 验证 MuJoCo 安装
python -c "import mujoco; print(mujoco.__version__)"
```

## 使用方法

### 方法1：集成到主 GUI

```bash
python -m src.gui.main_window_with_mujoco
```

界面布局：
- 左侧：关节控制和串口连接
- 中间：MuJoCo 3D 可视化（实时显示）
- 右侧：电机控制、示教功能、日志

### 方法2：独立可视化窗口

在原有 GUI 中点击 "🔳 打开独立 MuJoCo 窗口" 按钮

### 方法3：作为组件使用

```python
from mujoco_viewer import MuJoCoWidget

# 创建 widget
mujoco_widget = MuJoCoWidget(parent, width=640, height=480)

# 更新关节角度
mujoco_widget.update_joint_angles([0, -75, 180, 0, 0, 0])

# 切换视角
mujoco_widget.set_camera_view("front")  # front, side, top, iso, back
```

## 功能说明

### 1. 实时显示

MuJoCo 以 30 FPS 实时渲染机械臂状态：
- 关节角度从实际机器人读取
- 末端位置实时计算并显示

### 2. 视角切换

| 视角 | 说明 |
|------|------|
| 正面 | 从 Y 轴正方向看 |
| 侧面 | 从 X 轴正方向看 |
| 俯视 | 从 Z 轴正方向看 |
| 等轴测 | 默认视角（推荐） |
| 背面 | 从 Y 轴负方向看 |

### 3. 正解显示

末端位置显示格式：
```
末端: X=123.4 Y=45.6 Z=78.9 mm | 姿态: R=10.0° P=20.0° Y=30.0°
```

### 4. 数值逆解（开发中）

```python
from mujoco_viewer import DummyRobotVisualizer

viz = DummyRobotVisualizer()

# 目标位置（米）
target_pos = [0.2, 0.1, 0.3]

# 求解逆解
joint_angles = viz.inverse_kinematics(target_pos)
```

## 模型参数

模型文件：`dummy_robot.xml`

DH 参数（与实物一致）：
```
Joint 1: base旋转    range=[-170, 170]°
Joint 2: 肩部俯仰    range=[-75, 0]°
Joint 3: 肘部俯仰    range=[35, 180]°
Joint 4: 小臂旋转    range=[-170, 170]°
Joint 5: 腕部俯仰    range=[-120, 120]°
Joint 6: 末端旋转    range=[-720, 720]°
```

连杆长度：
```
Base -> J2: 109mm
J2 -> J3: 35mm
J3 -> J4: 146mm
J4 -> J5: 115mm
J5 -> J6: 52mm
J6 -> EE: 72mm
```

## Git 分支

本功能在独立分支开发：

```bash
# 切换到 MuJoCo 分支
git checkout feature/mujoco-visualization

# 查看分支
git branch -a

# 合并到 main（测试稳定后）
git checkout main
git merge feature/mujoco-visualization
```

## 故障排除

### 问题1：MuJoCo 初始化失败

```
[警告] MuJoCo 未安装或初始化失败
```

**解决：**
```bash
pip install mujoco
# 或
pip install mujoco --upgrade
```

### 问题2：3D 窗口黑屏

**原因：** OpenGL 驱动问题

**解决：**
```bash
# 更新显卡驱动
# 或设置软件渲染
export LIBGL_ALWAYS_SOFTWARE=1
```

### 问题3：模型加载失败

**解决：**
```bash
# 检查模型文件路径
ls src/mujoco_viewer/dummy_robot.xml

# 检查 STL 文件路径
ls ../4.Model/木子晓汶版本3D模型/*.stl
```

## 开发计划

- [x] 基础 3D 显示
- [x] 多视角切换
- [x] 末端位置显示
- [x] Qt 集成
- [ ] 逆解交互（点击目标位置）
- [ ] 轨迹可视化
- [ ] 碰撞检测显示
- [ ] 力/力矩可视化

## 参考资料

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo Python Bindings](https://mujoco.readthedocs.io/en/stable/python.html)
- Dummy V2 原始项目：基于任同学/木子晓汶版本
