# Dummy V2 关节限位说明与边界保护

## 关节限位配置

### 固件硬编码限制

| 关节 | 名称 | 最小值 | 最大值 | 备注 |
|------|------|--------|--------|------|
| J1 | 底座 | -170° | 170° | 无特殊限制 |
| J2 | 肩部 | -75° | 90° | 当前线轨夹爪固件限制 |
| J3 | 肘部 | **35°** | 180° | **固件硬编码限制！** |
| J4 | 小臂 | -180° | 180° | 当前线轨夹爪固件限制 |
| J5 | 腕部 | -120° | 120° | 无特殊限制 |
| J6 | 末端 | -180° | 180° | 当前线轨夹爪固件限制 |

### 关键限制说明

**⚠️ J3 关节特别警告：**
- 固件代码中硬编码限制：`motorJ[3] = new CtrlStepMotor(..., 35, 180)`
- 如果命令 J3 < 35°，固件会拒绝执行或发生意外行为
- 所有轨迹点在回放时都会被限制到 [35°, 180°] 范围内

**当前线轨夹爪固件事实：**

- `!RESET` / `Resting()` = `[0, -75, 180, 0, 0, 0]`
- `!HOME` / `Homing()` = `[0, 0, 90, 0, 0, 0]`
- J6 固件角限位是 `-180° ~ 180°`，旧 GUI/旧文档中的 `-720° ~ 720°` 不适用于当前已烧录的线轨夹爪固件。
- 当前已烧录固件按 `50:1` 计算 J6 电机步数，末端实际减速器为 `mini8-30`，GUI 显示/仿真输出角按 `输出角 = 固件角 × 固件比 / 实际比` 换算，默认是 `输出角 = 固件角 × 50 / 30`。
- 因此减速比差异不会导致 J6 完全不动；若一点都不动，应优先排查 J6 CAN 节点、使能、接线、电机相序或机械锁紧。

## 边界保护机制

### 1. 实时限位检查

```python
from utils.config import JointLimitChecker

# 检查角度是否合法
angles = [0, -50, 20, 0, 0, 0]  # J3=20 超限
is_valid = JointLimitChecker.is_valid(angles)
print(is_valid)  # False

# 获取违规信息
violations = JointLimitChecker.get_violations(angles)
# [(2, 20.0, 35, 180)]  # J3 超限
```

### 2. 自动限幅

```python
# 将角度限制到合法范围
angles = [0, -50, 20, 0, 0, 0]  # J3=20 超限
clamped = JointLimitChecker.clamp_angles(angles)
print(clamped)  # [0, -50, 35, 0, 0, 0]  # J3被限制到35
```

### 3. 平滑插值到边界

```python
# 从当前位置平滑移动到目标位置（自动处理超限）
current = [0, -75, 180, 0, 0, 0]  # REST
target = [0, 0, 20, 0, 0, 0]       # J3 超限

# 生成平滑过渡轨迹
trajectory = JointLimitChecker.interpolate_to_limit(current, target, steps=10)
# 每个点都在限位内
```

## 各阶段边界保护

### 运动控制阶段 (`robot.py`)

```python
# 所有 move_to 调用自动进行边界检查
success, msg = robot.move_to([0, 0, 20, 0, 0, 0])
# 输出: [警告] 角度已限制到合法范围: J3: 20.0° → 35.0° (限位[35, 180])
```

### 轨迹记录阶段 (`teach_mode.py`)

```python
# 记录时自动限制角度
angles = robot.get_position()  # 用户可能拖动到边界外
clamped = JointLimitChecker.clamp_angles(angles)  # 立即限制
```

### 轨迹平滑阶段

```python
# 平滑处理后再次限制
smoothed = teach_mode.smooth_trajectory(trajectory)
# 内部调用 clamp_angles 确保不超出限位
```

### 轨迹回放阶段

```python
# 回放前验证整个轨迹
is_valid, violations = teach_mode.validate_trajectory(trajectory)

# 自动修复超限点
fixed_trajectory = teach_mode.fix_trajectory_limits(trajectory)
```

## 使用示例

### 示例1：安全移动到目标位置

```python
from core.robot import DummyRobot
from utils.config import JointLimitChecker

robot = DummyRobot()
robot.connect("/dev/ttyACM0")
robot.enable()

# 目标位置（J3可能超限）
target = [0, -30, 10, 0, 0, 0]

# 方法1：让 move_to 自动处理
success, msg = robot.move_to(target)
print(msg)  # 会提示哪个关节被限制了

# 方法2：预先检查并警告
target_clamped = JointLimitChecker.clamp_angles(target)
if target_clamped != target:
    print("警告：目标位置已调整")
    print(f"  原始: {target}")
    print(f"  调整后: {target_clamped}")

robot.move_to(target_clamped)
```

### 示例2：处理记录的轨迹

```python
from core.teach_mode import TeachMode

# 假设从文件加载了轨迹
trajectory = teach_mode.load_trajectory("my_traj.json")

# 验证轨迹
is_valid, violations = teach_mode.validate_trajectory(trajectory)
if not is_valid:
    print(f"发现 {len(violations)} 个违规点:")
    for v in violations[:5]:  # 只显示前5个
        print(f"  {v}")

# 修复轨迹
trajectory_fixed = teach_mode.fix_trajectory_limits(trajectory, smooth=True)

# 现在可以安全回放
teach_mode.playback_trajectory(trajectory_fixed)
```

### 示例3：示教时实时监控

```python
# 在示教模式下，即使拖动超出限位，记录的数据也是安全的
teaching_record = []

while recording:
    angles = robot.get_position()
    
    # 自动限制到合法范围
    from utils.config import JointLimitChecker
    safe_angles = JointLimitChecker.clamp_angles(angles)
    
    teaching_record.append({
        'timestamp': time.time(),
        'angles': safe_angles
    })
```

## 故障排除

### 问题1：回放时机械臂不按照预期运动

**可能原因：**
- 轨迹中有超出限位的点，被自动限制

**排查：**
```python
# 检查轨迹是否有超限点
from utils.config import JointLimitChecker

for idx, point in enumerate(trajectory):
    if not JointLimitChecker.is_valid(point.angles):
        print(f"点 {idx} 超限: {point.angles}")
```

### 问题2：J3 关节突然停止

**可能原因：**
- 命令的 J3 角度 < 35°

**解决：**
```python
# 确保 J3 >= 35
target = [0, 0, max(35, desired_j3), 0, 0, 0]
```

### 问题3：示教时感觉某个关节"卡住"

**可能原因：**
- 接近关节限位，固件自动保护

**解决：**
- 查看当前位置，确认是否在限位边缘
- 调整示教姿势，避开限位区域

## 安全建议

### 软件层面

1. **始终保留固件限位检查**（默认已启用）
2. **保存轨迹前验证**
3. **回放前检查**是否有大量超限点
4. **空间安全边界使用 GUI“安全边界”页单独启用**
   - 旧 MuJoCo 碰台/基座检查当前默认关闭，避免模型不准时阻碍装配调试。
   - 用户软边界保存后可独立限制关节范围和末端世界坐标盒。

### 操作层面

1. **示教时避免极限位置**
   - 建议保持 J3 > 40°（留5°安全边距）
   - J2 当前固件范围为 -75°~90°，按实际装配空间设置用户软边界

2. **测试新轨迹**
   - 先用慢速（0.5x）回放
   - 观察是否有异常停止

3. **了解机械限制**
   - J3=35° 是最小安全角度，不是最优工作角度
   - 正常工作建议 J3 >= 60°

## 技术细节

### 安全边距

```python
# JointLimitChecker 内部使用 2° 安全边距
SAFETY_MARGIN = 2.0

# 实际软限位
soft_limits = [
    (-168, 168),   # J1: 硬限位 (-170, 170)
    (-73, -2),     # J2: 硬限位 (-75, 0)
    (37, 178),     # J3: 硬限位 (35, 180)
    (-168, 168),   # J4: 硬限位 (-170, 170)
    (-118, 118),   # J5: 硬限位 (-120, 120)
    (-718, 718),   # J6: 硬限位 (-720, 720)
]
```

### 平滑处理时的边界保持

```python
# 平滑滤波可能导致超出限位
smoothed = np.convolve(angles, window, mode='same')

# 必须再次限制
clamped = JointLimitChecker.clamp_angles(smoothed)
```

---

**记住：边界保护是最后一道防线，示教时尽量避开极限位置！**
