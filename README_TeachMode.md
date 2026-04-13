# Dummy V2 示教功能使用指南

## 📋 功能概述

本系统实现了完整的**拖动示教**功能，支持：

| 功能 | 描述 |
|------|------|
| 🔓 电流环示教 | 一键进入零扭矩模式，可手动拖动机械臂 |
| 📝 轨迹记录 | 实时记录6轴角度，50ms采样 |
| 🎨 平滑处理 | 移动平均滤波，去除抖动 |
| ▶️ 轨迹回放 | 支持变速回放（0.25x ~ 4x） |
| 💾 数据导出 | JSON完整格式 / CSV训练格式 |

---

## 🚀 快速开始

### 1. 启动程序

```bash
cd dummy-controller/src
python3 main.py
```

### 2. 进入示教模式

1. **连接串口** → 点击"连接"按钮
2. **使能电机** → 点击"使能电机"按钮  
3. **打开示教面板** → 点击右侧"🎓 打开示教面板"

### 3. 示教操作流程

```
┌─────────────────────────────────────────────────────┐
│  1. 点击 "🔓 进入示教模式"                           │
│     ↓                                               │
│  2. 手动拖动机械臂到你想要的位置                      │
│     ↓                                               │
│  3. 点击 "⏺️ 开始记录"                               │
│     ↓                                               │
│  4. 继续拖动完成动作轨迹                             │
│     ↓                                               │
│  5. 点击 "⏹️ 停止记录"                               │
│     ↓                                               │
│  6. 数据自动平滑处理                                 │
│     ↓                                               │
│  7. 点击 "💾 保存轨迹"                               │
└─────────────────────────────────────────────────────┘
```

---

## 📊 数据格式说明

### JSON 格式（完整数据）

适合：完整存档、轨迹分析、可视化

```json
{
  "metadata": {
    "name": "抓取动作_v1",
    "description": "从A点抓取物体放到B点",
    "robot_model": "Dummy V2",
    "sample_interval": 0.05,
    "smoothed": true,
    "creator": "操作员张三",
    "tags": ["抓取", "搬运"]
  },
  "created_at": "2026-04-12T14:30:00",
  "duration": 3.5,
  "num_points": 70,
  "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
  "trajectory": [
    {
      "timestamp": 0.0,
      "angles": [0, -75, 180, 0, 0, 0],
      "velocity": [0, 0, 0, 0, 0, 0],
      "acceleration": [0, 0, 0, 0, 0, 0]
    },
    {
      "timestamp": 0.05,
      "angles": [0, -60, 160, 5, 10, 0],
      "velocity": [0, 300, 400, 100, 200, 0],
      "acceleration": [0, 6000, 8000, 2000, 4000, 0]
    }
  ],
  "keyframes": [
    {"idx": 0, "name": "起点", "angles": [0, -75, 180, 0, 0, 0]},
    {"idx": 20, "name": "到达物体", "angles": [0, 0, 90, 0, 0, 0]},
    {"idx": 69, "name": "终点", "angles": [45, 0, 90, 0, 0, 0]}
  ]
}
```

### CSV 格式（ML训练格式）

适合：机器学习、强化学习、数据科学

```csv
# Dummy Robot Trajectory
# Created: 2026-04-12T14:30:00
# Duration: 3.500s
# Points: 70

timestamp,J1,J2,J3,J4,J5,J6
0.000000,0.0000,-75.0000,180.0000,0.0000,0.0000,0.0000
0.050000,0.0000,-60.0000,160.0000,5.0000,10.0000,0.0000
0.100000,0.0000,-40.0000,140.0000,10.0000,20.0000,5.0000
...
```

---

## 🤖 机器学习使用指南

### 加载数据用于训练

```python
from examples.ml_training_loader import TrajectoryDataset
from torch.utils.data import DataLoader

# 加载数据集
dataset = TrajectoryDataset(
    '轨迹文件.json',
    seq_len=10,        # 使用过去10帧
    predict_next=1     # 预测下一帧
)

dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

# 训练循环
for batch in dataloader:
    obs = batch['obs']      # [B, 10, 6] 历史角度
    vel = batch['vel']      # [B, 10, 6] 历史速度
    target = batch['target'] # [B, 1, 6] 目标角度
    
    # 你的模型训练代码...
    pred = model(obs, vel)
    loss = criterion(pred, target)
    ...
```

### 行为克隆 (Behavior Cloning)

```python
# 完整的训练脚本见 examples/ml_training_loader.py
demo_behavior_cloning()
```

### 轨迹统计分析

```python
from examples.ml_training_loader import CSVLoader
import numpy as np

# 加载CSV数据
data = CSVLoader.load('轨迹文件.csv')
timestamps = data[:, 0]
angles = data[:, 1:]

# 分析
print(f"总时长: {timestamps[-1]:.3f}秒")
print(f"角度范围: {angles.min():.2f}° ~ {angles.max():.2f}°")

# 计算平滑度
velocities = np.diff(angles, axis=0) / np.diff(timestamps)[:, None]
accelerations = np.diff(velocities, axis=0) / np.diff(timestamps[:-1])[:, None]
jerk = np.diff(accelerations, axis=0)
print(f"运动平滑度 (Jerk): {np.abs(jerk).mean():.2f} °/s³")
```

---

## ⚙️ 高级设置

### 采样间隔调整

| 间隔 | 适用场景 |
|------|----------|
| 20ms (50Hz) | 快速动作，精确控制 |
| 50ms (20Hz) | 常规动作（默认推荐） |
| 100ms (10Hz) | 慢速动作，数据量小 |

### 平滑窗口大小

- **小窗口 (3-5)**：轻微平滑，保留细节
- **中窗口 (7-9)**：平衡平滑度和精度（推荐）
- **大窗口 (11-21)**：重度平滑，适合抖动大的数据

### 回放速度

- **0.25x - 0.5x**：慢动作，精确调试
- **1.0x**：原速回放（默认）
- **2.0x - 4.0x**：快速预览

---

## ⚠️ 注意事项

### 安全提示

1. **进入示教模式前**：确保机械臂周围没有障碍物
2. **拖动时**：用双手稳住机械臂，避免突然松手
3. **电流限制**：默认0.5A可以舒适拖动，太重可再调小
4. **紧急停止**：遇到异常立即点击"⏹ 急停"

### 技术限制

1. **J3关节**：固件限制最小35°，示教时不可突破
2. **采样间隔**：受串口通信限制，最小约20ms
3. **数据精度**：角度记录精度0.01°，时间精度0.001s

### 常见问题

**Q: 为什么拖动时感觉很重？**
A: 检查是否已进入示教模式，或尝试调小电流限制。

**Q: 记录的轨迹抖动很大？**
A: 增大平滑窗口，或在录制时更稳定地拖动。

**Q: 回放时机械臂运动不平滑？**
A: 尝试减小回放速度，或检查原始记录是否有突变点。

---

## 📁 文件结构

```
dummy-controller/
├── src/
│   ├── core/
│   │   ├── robot.py          # 机器人控制
│   │   └── teach_mode.py     # 示教核心逻辑
│   ├── gui/
│   │   ├── main_window.py    # 主窗口
│   │   └── teach_dialog.py   # 示教对话框
│   └── main.py
├── examples/
│   ├── trajectory_example.json      # JSON示例
│   ├── trajectory_example.csv       # CSV示例
│   └── ml_training_loader.py        # ML加载器
└── README_TeachMode.md              # 本文件
```

---

## 🔬 技术原理

### 电流环 (Current Loop)

```
正常模式: 电机 = 位置控制器 → 硬邦邦（掰不动）
示教模式: 电机 = 电流限制器 → 有弹性（可拖动）

┌─────────────┐      ┌─────────────┐
│  目标位置   │      │  电流限制   │
└──────┬──────┘      └──────┬──────┘
       │                    │
       ▼                    ▼
┌─────────────┐      ┌─────────────┐
│  PID控制器  │      │  电流环     │
└──────┬──────┘      └──────┬──────┘
       │                    │
       ▼                    ▼
┌─────────────┐      ┌─────────────┐
│  电机驱动   │      │  电机驱动   │
└─────────────┘      └─────────────┘
  （位置控制）         （力矩控制）
```

### 轨迹平滑算法

```python
# 移动平均滤波
smoothed[i] = mean(trajectory[i-k:i+k+1])

# 其中 k = window_size // 2
```

### 时间归一化

```python
# 原始时间戳不均匀 → 插值为固定间隔
new_times = [0, 0.05, 0.10, 0.15, ...]
new_angles = interpolate(original_times, original_angles, new_times)
```

---

## 📝 更新日志

- **v1.0** (2026-04-12)
  - 初始版本发布
  - 支持基础示教、记录、回放、保存
  - 支持 JSON / CSV 双格式导出
  - 提供 ML 训练数据加载示例

---

**有问题？** 查看 `examples/` 目录中的示例代码或联系开发者。
