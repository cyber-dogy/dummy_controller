# 示教功能快速参考

## 5分钟上手

```bash
# 1. 启动
cd dummy-controller/src && python3 main.py

# 2. 连接 → 使能 → 打开示教面板

# 3. 一键示教流程
进入示教模式 → 拖动机械臂 → 开始记录 → 完成动作 → 停止记录 → 保存
```

## 常用操作速查

| 操作 | 步骤 |
|------|------|
| **拖动示教** | 进入示教模式 → 手动拖动 |
| **记录轨迹** | 开始记录 → 做动作 → 停止记录 |
| **平滑轨迹** | 自动平滑（默认开启） |
| **回放轨迹** | 调整速度 → 点击回放 |
| **保存数据** | 命名 → 选格式(JSON/CSV) → 保存 |
| **用于训练** | 保存为CSV → 用`ml_training_loader.py`加载 |

## 数据格式对比

```
JSON格式                        CSV格式
────────────                   ────────────
完整元数据                      纯数据表格
包含速度/加速度                  仅角度
适合：存档、可视化               适合：ML训练

文件大小：较大                  文件大小：较小
可读性：好                      可读性：一般
```

## 关键参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 电流限制 | 0.5A | 越小越容易拖动 |
| 采样间隔 | 50ms | 平衡精度和数据量 |
| 平滑窗口 | 5 | 去除抖动 |
| 回放速度 | 1.0x | 原速 |

## 代码示例

### 加载训练数据

```python
from examples.ml_training_loader import TrajectoryDataset

dataset = TrajectoryDataset('my_trajectory.json', seq_len=10)
print(f"加载了 {len(dataset)} 个训练样本")
```

### 读取CSV

```python
import numpy as np
data = np.loadtxt('my_trajectory.csv', delimiter=',', skiprows=6)
angles = data[:, 1:]  # 6轴角度
```

## 故障排除

| 问题 | 解决方案 |
|------|----------|
| 拖不动 | 检查是否进入示教模式，或调小电流 |
| 记录抖动大 | 增大平滑窗口 |
| 回放卡 | 减小速度或检查轨迹 |
| 数据丢失 | JSON格式更可靠 |

---

**详细文档**: 查看 `README_TeachMode.md`
**示例代码**: 查看 `examples/` 目录
