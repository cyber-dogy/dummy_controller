# MuJoCo Sim - Dummy V2 3D 仿真与可视化

独立的 MuJoCo 仿真环境，用于 Dummy V2 机械臂的 3D 可视化、正逆运动学计算。

## 目录结构

```
mujoco_sim/
├── src/
│   ├── __init__.py              # 模块入口
│   ├── robot_visualizer.py      # 核心可视化器
│   └── qt_integration.py        # PyQt6 集成组件
├── models/
│   └── dummy_robot.xml          # MuJoCo 模型文件
├── requirements.txt             # 依赖列表
├── setup_env.sh                 # 环境安装脚本
└── README.md                    # 本文件
```

## 环境配置

### 自动安装

```bash
cd mujoco_sim
bash setup_env.sh
```

### 手动安装

```bash
cd mujoco_sim
python3 -m venv .venv_mujoco
source .venv_mujoco/bin/activate
pip install -r requirements.txt
```

## 使用方法

### 1. 独立运行可视化器

```bash
cd mujoco_sim
source .venv_mujoco/bin/activate
python3 src/robot_visualizer.py
```

### 2. 集成到主 GUI

主 GUI 会自动检测 `mujoco_sim` 目录并加载：

```bash
cd dummy-controller
python3 -m src.gui.main_window_with_mujoco
```

### 3. 作为模块导入

```python
import sys
sys.path.insert(0, "../mujoco_sim")

from src.qt_integration import MuJoCoWidget, MujocoDialog
from src.robot_visualizer import DummyRobotVisualizer
```

## 模型说明

- **模型文件**: `models/dummy_robot.xml`
- **网格来源**: 木子晓汶版本 3D 模型（STL 格式）
- **DH 参数**: 与 Dummy V2 实物完全一致

关节范围：
| 关节 | 范围 | 说明 |
|------|------|------|
| J1 | [-170°, 170°] | 底座旋转 |
| J2 | [-75°, 0°] | 肩部俯仰 |
| J3 | [35°, 180°] | 肘部俯仰 |
| J4 | [-170°, 170°] | 小臂旋转 |
| J5 | [-120°, 120°] | 腕部俯仰 |
| J6 | [-720°, 720°] | 末端旋转 |

## 功能特性

- ✅ 实时 3D 可视化（30 FPS）
- ✅ 正运动学计算
- ✅ 数值逆运动学
- ✅ 多视角切换
- ✅ PyQt6 无缝集成

## 依赖

- mujoco >= 2.3.0
- numpy >= 1.21.0
- scipy >= 1.7.0
- PyQt6 >= 6.4.0
