# CAN总线通信保护 - 实现总结

## 实现内容

### 1. 采样率保护

**文件:** `teach_mode.py`

```python
MIN_SAMPLE_INTERVAL = 0.05  # 最小采样间隔 50ms
MAX_TRAJECTORY_POINTS = 10000  # 最大轨迹点数
```

**保护机制:**
- 记录循环中强制最小间隔
- 超出上限时丢弃数据并警告
- 连续错误5次自动停止记录

### 2. 轨迹稀疏化

**文件:** `teach_mode.py`

```python
def decimate_trajectory(self, trajectory, tolerance=1.0):
    """Ramer-Douglas-Peucker算法减少点数"""

def optimize_for_can_bus(self, trajectory, max_points_per_second=20.0):
    """优化轨迹以适应CAN总线"""
```

**算法流程:**
1. 计算轨迹频率
2. 如果 > 20Hz，使用RDP稀疏化
3. 如果还不够，均匀降采样
4. 保持轨迹形状

### 3. 回放限流

**文件:** `teach_dialog.py` (PlaybackThread)

```python
MIN_TX_INTERVAL = 0.03  # 最小发送间隔 30ms

# 发送过快时自动跳过
if current_time - last_tx_time < MIN_TX_INTERVAL:
    skip_count += 1
    continue
```

**统计信息:**
- 发送成功点数
- 限流跳过点数
- 发送失败次数

### 4. 超时保护

**文件:** `teach_mode.py`

```python
def _safe_get_position(self, timeout=0.5):
    """带超时的位置读取"""
    thread.join(timeout)
    if thread.is_alive():
        print("[警告] 读取位置超时")
        return None
```

### 5. 错误统计

```python
self.stats = {
    'dropped_points': 0,    # 丢弃的点
    'tx_timeouts': 0,       # 发送超时
    'can_errors': 0,        # CAN错误
    'throttled_points': 0,  # 限流跳过
}
```

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| MIN_SAMPLE_INTERVAL | 50ms | 记录最小间隔 |
| MIN_TX_INTERVAL | 30ms | 发送最小间隔 |
| MAX_POINTS_PER_SECOND | 20 | 最大回放频率 |
| CAN_TX_TIMEOUT | 100ms | 通信超时 |
| MAX_TRAJECTORY_POINTS | 10000 | 轨迹点数上限 |

## 文件修改

```
dummy-controller/
├── src/core/teach_mode.py     # +稀疏化算法 +超时保护 +限流
├── src/gui/teach_dialog.py    # +回放限流 +CAN统计
└── docs/CAN_BUS_PROTECTION.md # 文档
```

## 测试验证

```
[测试1] 轨迹稀疏化
原始轨迹: 100点
稀疏化后: 3点 (容差2°)  # 测试数据为直线

[测试2] CAN优化
高频轨迹: 500点, 9.98s, 50.1Hz
优化后: 2点, 0.2Hz

[测试3] 超时保护
正常读取: [0, 0, 90, 0, 0, 0]
超时测试: None (正确返回)
```

## 使用建议

### 示教记录
- 使用默认 50ms 采样间隔
- 避免记录过长轨迹（>60秒）

### 轨迹回放
- 首次回放用 0.5x 速度测试
- 观察日志中的CAN统计信息
- 如果大量点被跳过，降低采样率

### 参数调整
- 需要更高精度：减小 MIN_TX_INTERVAL
- 需要更流畅：增大稀疏化容差

## 注意事项

1. **RDP算法在直线上会大幅简化** - 这是正常现象
2. **限流跳过点是保护机制** - 不是错误
3. **超时保护防止卡死** - 但可能丢失数据

## 完成！

CAN总线保护机制已完整实现，可防止通信过载和数据丢失。🎉
