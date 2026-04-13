# 示教模式边界处理详解

## 概述

在示教模式下，机械臂通过**电流环控制**，电机只输出恒定的阻力电流，位置是自由的。用户可以拖动机械臂到任何位置，**包括超出固件限制的范围**。

为了确保安全，系统需要在以下时机处理边界：
1. **示教过程中**：实时警告接近边界
2. **记录轨迹时**：自动限制记录的角度到合法范围
3. **退出示教模式时**：如果当前位置超限，平滑回到合法位置
4. **回放轨迹时**：确保所有轨迹点在限位内

---

## 示教时拖动的范围

### 实际可拖动范围

在电流环模式下，机械臂**理论上可以拖动到任何位置**（机械结构允许范围内）。但是：

```
物理范围：由机械结构决定
        J1: 360° 旋转
        J2: -90° ~ 10° (机械限位)
        J3: 0° ~ 200° (机械限位)
        ...

固件范围：由软件限制（固件硬编码）
        J1: [-170°, 170°]
        J2: [-75°, 0°]      ← 比机械范围小
        J3: [35°, 180°]    ← 比机械范围小！重要
        J4: [-170°, 170°]
        J5: [-120°, 120°]
        J6: [-720°, 720°]
```

### 关键问题

**用户可以拖动到固件限制之外！**

例如：
- 用户可以把 J3 拖到 20°（固件限制是 35°）
- 用户可以把 J2 拖到 -80°（固件限制是 -75°）

**风险：**
- 退出示教模式时，如果直接设置当前位置为目标，固件可能拒绝或出错
- 回放轨迹时，超出限位的点会导致机械臂停止或异常

---

## 边界保护机制

### 1. 示教过程中的实时警告

```python
# GUI 定时器每 100ms 检查一次
def update_live_data(self):
    angles = robot.get_position()
    
    for i, angle in enumerate(angles):
        min_val, max_val = JOINT_LIMITS[i]
        
        # 距离边界小于 5° 时警告
        if angle < min_val + 5:
            log(f"⚠️ {JOINT_NAMES[i]}接近下限({angle:.1f}°)")
        elif angle > max_val - 5:
            log(f"⚠️ {JOINT_NAMES[i]}接近上限({angle:.1f}°)")
```

**效果：**
- 当用户拖动接近边界时，日志会显示警告
- 每 5 秒显示一次，避免刷屏

### 2. 记录轨迹时的边界限制

```python
def _record_loop(self):
    angles = robot.get_position()
    
    # 即使示教时超出范围，记录时自动限制
    clamped_angles = JointLimitChecker.clamp_angles(angles)
    
    trajectory.append(TrajectoryPoint(
        timestamp=time.time(),
        angles=clamped_angles  # 记录的是限制后的角度
    ))
```

**效果：**
- 用户拖到了 J3=20°，但记录的是 J3=35°
- 回放时不会超出限位

### 3. 退出示教模式时的边界处理（重点）

这是最复杂也是最重要的情况：

#### 场景 A：当前位置在合法范围内
```
当前位置: J3=45° (合法)
退出示教: 直接切换回位置模式
结果: 机械臂保持在 45°
```

#### 场景 B：当前位置超出范围
```
当前位置: J3=20° (超出 35° 下限)
         
退出示教:
  1. 检测到 J3 超限
  2. 询问用户是否平滑过渡
  3. 计算合法位置: J3=35°
  4. 生成过渡轨迹: 20° → 25° → 30° → 35°
  5. 执行过渡运动（低速）
  6. 到达合法位置后，切换回位置模式
```

#### 代码实现

```cpp
// 固件端退出示教模式
void DummyRobot::SetTeachMode(false)
{
    // 获取当前角度（可能超限）
    UpdateJointAngles();
    
    // 限制到合法范围
    clampedJoints[0] = fmaxf(-170.0f, fminf(170.0f, currentJoints.a[0]));
    clampedJoints[1] = fmaxf(-73.0f, fminf(90.0f, currentJoints.a[1]));
    clampedJoints[2] = fmaxf(35.0f, fminf(180.0f, currentJoints.a[2]));  // J3 限制
    // ...
    
    // 使用限制后的角度
    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAngle(clampedJoints[i-1] - initPose.a[i-1]);
}
```

```python
# PC端退出示教模式
def exit_teach_mode(self):
    current_pos = robot.get_position()  # 获取当前位置
    
    if not JointLimitChecker.is_valid(current_pos):
        # 弹出对话框询问
        reply = QMessageBox.question(
            "位置超限",
            "当前位置超出关节限位。是否平滑移动回合法位置？"
        )
        
        if reply == Yes:
            # 计算合法位置
            target_pos = JointLimitChecker.clamp_angles(current_pos)
            
            # 生成过渡轨迹
            transition = JointLimitChecker.interpolate_to_limit(
                current_pos, target_pos, steps=10
            )
            
            # 执行过渡
            for angles in transition:
                robot.move_to(angles, speed=10)
                time.sleep(0.1)
    
    # 完成退出
    robot.exit_teach_mode()
```

### 4. 轨迹回放时的边界检查

```python
def playback_trajectory(self, trajectory):
    # 验证所有轨迹点
    for point in trajectory:
        if not JointLimitChecker.is_valid(point.angles):
            # 提示用户
            QMessageBox.warning("轨迹中有超出限位的点")
            
    # 自动修复
    valid_trajectory = [
        TrajectoryPoint(
            timestamp=p.timestamp,
            angles=JointLimitChecker.clamp_angles(p.angles)
        )
        for p in trajectory
    ]
    
    # 执行回放
    for point in valid_trajectory:
        robot.move_to(point.angles)
```

---

## 使用建议

### 示教时的注意事项

1. **注意 GUI 日志中的边界警告**
   ```
   ⚠️ 边界警告: J3接近下限(38.5°)
   → 不要再往下拖了，J3 下限是 35°
   ```

2. **如果已经超出边界**
   - 不用担心，记录的数据会自动限制
   - 退出时会提示平滑回到合法位置
   - 选择"是"让机械臂自动平滑过渡

3. **避免的操作**
   - 不要强行拖动到机械极限（可能损坏）
   - 示教时保持各关节在中间位置附近
   - 特别注意 J3，最小只能到 35°

### 退出示教模式的流程

```
1. 点击 "退出示教模式"
     ↓
2. 系统读取当前位置
     ↓
3. 检查是否超出限位
     ├── 未超出 → 直接退出
     └── 超出 → 提示用户
              ↓
         选择"平滑过渡"
              ↓
         生成过渡轨迹
              ↓
         低速运动到合法位置
              ↓
         切换回位置模式
```

---

## 故障排除

### 问题1：退出示教时机械臂突然跳动

**原因：**
- 当前位置超出限位，固件自动限制了目标位置
- 位置跳跃导致速度突变

**解决：**
- 在 GUI 中选择"平滑过渡"
- 或者在示教时就不要超出限位

### 问题2：示教时拖不动到想要的位置

**原因：**
- 可能已经碰到了机械限位
- 或者电流设置太大，阻力太强

**解决：**
- 检查当前位置是否接近限位
- 减小电流值（如 0.3A）让拖动更轻松

### 问题3：记录的轨迹回放时和示教时不一致

**原因：**
- 示教时超出了限位，记录时被限制到合法范围

**检查：**
```python
# 查看记录的轨迹范围
print(f"J3范围: {min(p.angles[2] for p in trajectory):.1f}° ~ {max(p.angles[2] for p in trajectory):.1f}°")
# 如果最小值是 35°，说明示教时曾超出下限
```

---

## 技术细节

### 插值算法

当需要从超限位置平滑回到合法位置时：

```python
def interpolate_to_limit(start, target, steps=10):
    """
    从起点到目标点插值，确保每个点都在限位内
    """
    # 先限制目标点
    target_clamped = clamp_angles(target)
    
    trajectory = []
    for i in range(steps + 1):
        t = i / steps
        # 线性插值
        interpolated = [
            start[j] + (target_clamped[j] - start[j]) * t
            for j in range(6)
        ]
        # 双重保险：再次限制
        trajectory.append(clamp_angles(interpolated))
    
    return trajectory
```

### 固件端的限制

```cpp
// 关节限位（与PC端保持一致）
const float JOINT_MIN[6] = {-170.0f, -75.0f, 35.0f, -170.0f, -120.0f, -720.0f};
const float JOINT_MAX[6] = {170.0f, 0.0f, 180.0f, 170.0f, 120.0f, 720.0f};

// 限制函数
float ClampJointAngle(int joint_idx, float angle) {
    return fmaxf(JOINT_MIN[joint_idx], fminf(JOINT_MAX[joint_idx], angle));
}
```

---

## 总结

示教模式的边界处理确保了：

1. **示教自由**：用户可以拖动到任何位置（电流环模式）
2. **记录安全**：记录的数据自动限制在合法范围
3. **退出平滑**：超出限位时自动平滑过渡回合法位置
4. **回放可靠**：轨迹回放不会触发限位保护

这样用户既可以自由地示教，又不用担心边界问题！
