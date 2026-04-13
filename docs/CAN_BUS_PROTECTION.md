# CAN总线通信保护机制

## 背景

Dummy V2 使用 CAN 总线连接主控板（STM32F4）和 6 个关节驱动板（STM32F1）。CAN 总线虽然有硬件流控，但存在以下风险：

1. **发送缓冲区满**：STM32 有 3 个发送邮箱，如果发送频率过高，新的消息可能丢失
2. **总线负载过高**：高频率通信会导致总线拥堵，影响实时性
3. **信号量阻塞**：固件中使用信号量等待发送完成，过快发送会导致阻塞

## 保护措施

### 1. 采样率限制（记录阶段）

```python
MIN_SAMPLE_INTERVAL = 0.05  # 最小采样间隔 50ms (20Hz)
MAX_SAMPLE_INTERVAL = 0.5   # 最大采样间隔 500ms (2Hz)
```

- 示教记录时，采样间隔不能小于 50ms
- 防止记录阶段就产生过多数据点

### 2. 轨迹稀疏化（RDP算法）

```python
def decimate_trajectory(trajectory, tolerance=1.0):
    """使用Ramer-Douglas-Peucker算法减少点数"""
```

**作用：**
- 在保持轨迹形状的前提下减少点数
- 容差 1.0° 时，通常可以减少 30-50% 的点

### 3. CAN总线优化回放

```python
def optimize_for_can_bus(trajectory, max_points_per_second=20.0):
    """优化轨迹以适应CAN总线带宽"""
```

**自动优化流程：**
1. 检查轨迹频率
2. 如果 > 20Hz，使用RDP稀疏化
3. 如果还不够，使用均匀降采样
4. 确保不丢失关键形状

### 4. 回放限流保护

```python
MIN_TX_INTERVAL = 0.03  # 最小发送间隔 30ms（约33Hz）

# 如果距离上次发送时间太短，跳过此点
if current_time - last_tx_time < MIN_TX_INTERVAL:
    skip_count += 1
    continue
```

### 5. 超时保护

```python
def _safe_get_position(self, timeout=0.5):
    """带超时的位置读取"""
    thread.join(timeout)
    if thread.is_alive():
        print("[警告] 读取位置超时")
        return None
```

## 参数设置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| MIN_SAMPLE_INTERVAL | 50ms | 最小采样间隔 |
| MIN_TX_INTERVAL | 30ms | 最小发送间隔 |
| MAX_POINTS_PER_SECOND | 20 | 最大点数/秒 |
| CAN_TX_TIMEOUT | 100ms | CAN发送超时 |
| MAX_TRAJECTORY_POINTS | 10000 | 最大轨迹点数 |

## 故障诊断

### 回放卡顿

**症状：** 机械臂运动不流畅

**排查：**
```
[CAN回放] 完成: 发送 50点, 限流跳过 200点
```

**解决：** 降低采样率或增加回放速度

### CAN通信超时

**症状：** 日志频繁出现 "读取位置超时"

**排查：**
- 检查CAN线连接
- 降低采样率
- 重启主控板

## 总结

CAN总线保护机制确保了通信稳定性，防止数据丢失和系统阻塞。
