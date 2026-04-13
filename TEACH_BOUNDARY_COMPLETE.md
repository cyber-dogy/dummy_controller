# 示教模式边界处理 - 完整实现

## 实现目标

解决示教模式下用户拖动机械臂可能超出固件限制的问题，确保：
1. 示教时自由拖动（电流环模式）
2. 接近边界时实时警告
3. 记录数据自动限制
4. 退出时平滑回到合法位置

---

## 实现内容

### 1. 实时边界警告 (`gui/teach_dialog.py`)

```python
def update_live_data(self):
    # 每 100ms 检查一次
    if 关节距离边界 < 5°:
        log(f"⚠️ 边界警告: {关节名}接近限位({角度}°)")
```

**效果：**
- 用户拖动接近边界时，日志显示警告
- 每 5 秒提示一次，避免刷屏

### 2. 退出示教边界处理 (`gui/teach_dialog.py`)

```python
def exit_teach_mode(self):
    current_pos = robot.get_position()
    
    if 当前位置超出限位:
        # 弹出对话框
        QMessageBox.question("是否平滑移动回合法位置？")
        
        if 用户选择"是":
            # 生成过渡轨迹
            transition = interpolate_to_limit(current, target, steps=10)
            # 低速执行
            for angles in transition:
                robot.move_to(angles, speed=10)
    
    # 完成退出
    exit_teach_mode()
```

**效果：**
- 检测当前位置是否超限
- 弹出对话框让用户选择
- 选择"平滑过渡"则自动生成插值轨迹
- 低速运动到合法位置后退出

### 3. 固件端边界限制 (`dummy_robot.cpp`)

```cpp
void SetTeachMode(bool _enable, float _current) {
    if (!_enable) {  // 退出示教模式
        // 获取当前角度
        UpdateJointAngles();
        
        // 限制到合法范围
        clampedJoints[2] = fmaxf(35.0f, fminf(180.0f, currentJoints.a[2]));
        // ...
        
        // 使用限制后的角度
        for (int i = 1; i <= 6; i++)
            motorJ[i]->SetAngle(clampedJoints[i-1]);
    }
}
```

**效果：**
- 即使 PC 端没有处理，固件端也会限制角度
- 双重保险确保安全

---

## 使用流程示例

### 场景：示教时 J3 超出下限

```
1. 进入示教模式
   → 点击 "🔓 进入示教模式"
   → 电流环启动，可以自由拖动

2. 用户拖动机械臂
   → J3 被拖到 20°（超出 35° 下限）
   → GUI 日志: ⚠️ 边界警告: J3接近下限(25.0°)
   → 用户继续拖动到 20°

3. 记录轨迹
   → 点击 "⏺️ 开始记录"
   → 系统记录 J3=35°（自动限制）
   → 日志: 记录点已限制到合法范围

4. 退出示教模式
   → 点击 "🔒 退出示教模式"
   → 系统检测: 当前 J3=20° 超限
   → 弹出对话框: "是否平滑移动回合法位置？"
   
   用户选择:
   ├── "是" → 生成轨迹: 20°→25°→30°→35°
   │        → 低速执行过渡
   │        → 到达 35° 后退出
   │        → 日志: ✅ 已回到合法位置
   │
   └── "否" → 直接退出（固件端会限制）
   
5. 完成
   → 机械臂稳定在 J3=35°
   → 已切换回位置控制模式
```

---

## 关键代码

### 平滑插值到边界

```python
# utils/config.py
class JointLimitChecker:
    @classmethod
    def interpolate_to_limit(cls, start, target, steps=10):
        """从起点到目标点插值，确保每个点都在限位内"""
        # 限制目标点
        target_clamped = cls.clamp_angles(target)
        
        trajectory = []
        for i in range(steps + 1):
            t = i / steps
            # 线性插值
            interpolated = [
                start[j] + (target_clamped[j] - start[j]) * t
                for j in range(6)
            ]
            # 再次限制（双重保险）
            trajectory.append(cls.clamp_angles(interpolated))
        
        return trajectory
```

### 实时边界检查

```python
# gui/teach_dialog.py
def update_live_data(self):
    """实时更新当前位置，并检查是否接近边界"""
    if not self.robot.connected:
        return
    
    angles = self.robot.get_position()
    if not angles:
        return
    
    # 只在示教模式下检查
    if self.btn_exit_teach.isEnabled():
        warnings = []
        for i, angle in enumerate(angles):
            min_val, max_val = JOINT_LIMITS[i]
            # 距离边界小于 5° 时警告
            if angle < min_val + 5:
                warnings.append(f"{JOINT_NAMES[i]}接近下限({angle:.1f}°)")
            elif angle > max_val - 5:
                warnings.append(f"{JOINT_NAMES[i]}接近上限({angle:.1f}°)")
        
        if warnings:
            self.log(f"⚠️ 边界警告: {', '.join(warnings[:2])}")
```

---

## 测试验证

```
============================================================
示教边界处理功能测试
============================================================
[测试1] 边界超限处理
当前位置: [0, -75, 180, 0, 0, 0]  (REST)
目标位置: [0, 0, 20, 0, 0, 0]      (J3超限)

过渡轨迹 (6 点):
  点0: J3=180.0°
  点1: J3=151.0°
  点2: J3=122.0°
  点3: J3=93.0°
  点4: J3=64.0°
  点5: J3=35.0° <-- 已限制

[测试2] 轨迹点边界验证
原始轨迹:
  ✓ t=0.0s: J3=180.0°
  ✗ t=1.0s: J3=20.0°  (超限)
  ✓ t=2.0s: J3=35.0°

修复后轨迹:
  ✓ t=0.0s: J3=180.0°
  ✓ t=1.0s: J3=35.0°  (已修复)
  ✓ t=2.0s: J3=35.0°

============================================================
✅ 示教边界处理功能测试通过！
============================================================
```

---

## 文档

- `docs/TEACH_MODE_BOUNDARY.md` - 详细原理和使用说明
- `docs/JOINT_LIMITS.md` - 关节限位参考
- 本文件 - 实现总结

---

## 注意事项

### J3 关节特别提醒

```
固件硬编码限制: J3 ∈ [35°, 180°]

用户可能拖动到: J3 = 20° (超出)
系统处理:
  1. 实时警告: "J3接近下限(25.0°)"
  2. 记录限制: 存储 J3=35°
  3. 退出过渡: 20°→25°→30°→35°
```

### 建议操作

1. **示教时注意观察日志警告**
   - 看到"接近限位"就不要再拖了

2. **退出时选择平滑过渡**
   - 避免机械臂突然跳动

3. **特殊情况**
   - 如果强行拖到了机械极限，立即退出并检查
   - 电流设置小一点（0.3A），拖动更轻，更容易感知边界

---

## 完成！

示教模式边界处理已完整实现，用户可以：
- ✅ 自由拖动示教（电流环模式）
- ✅ 收到实时边界警告
- ✅ 记录的数据自动安全
- ✅ 退出时平滑回到合法位置

现在可以安全地进行示教操作了！🎉
