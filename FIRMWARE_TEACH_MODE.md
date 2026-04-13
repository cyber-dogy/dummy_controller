# 电流环示教模式 - 固件修改指南

## 概述

本修改为主控板固件添加了完整的电流环示教模式支持，让机械臂可以轻松拖动示教。

## 修改内容

### 1. 新增文件/修改文件

```
_repos/Dummy-Robot/2.Firmware/Core-STM32F4-fw/
├── Robot/instances/dummy_robot.h          # 添加 SetTeachMode() 声明
├── Robot/instances/dummy_robot.cpp        # 实现 SetTeachMode() 方法
└── UserApp/protocols/ascii_protocol.cpp   # 添加 !TEACH 和 !TEACHOFF 命令
```

### 2. 新增命令

| 命令 | 参数 | 说明 |
|------|------|------|
| `!TEACH <current>` | current: 电流值(A) | 进入示教模式，设置电流限制 |
| `!TEACHOFF` | 无 | 退出示教模式，恢复位置控制 |

**示例：**
```
!TEACH 0.5      # 进入示教模式，电流限制0.5A
!TEACH 0.3      # 进入示教模式，电流限制0.3A（更轻松）
!TEACHOFF       # 退出示教模式
```

## 编译步骤

### 前提条件

1. 安装 **STM32CubeIDE** 或 **STM32CubeMX + GCC ARM**
2. 安装 **STM32CubeProgrammer**（用于烧录）

### 编译流程

```bash
# 1. 进入固件目录
cd _repos/Dummy-Robot/2.Firmware/Core-STM32F4-fw

# 2. 使用 STM32CubeIDE 打开项目
#    文件 -> 打开项目 -> 选择 Core-STM32F4-fw 目录

# 3. 在 IDE 中点击 "Build" 按钮编译
#    或使用命令行：
make clean
make all

# 4. 编译成功后，生成文件位于：
#    Core-STM32F4-fw/build/Core-STM32F4-fw.elf
#    Core-STM32F4-fw/build/Core-STM32F4-fw.bin
#    Core-STM32F4-fw/build/Core-STM32F4-fw.hex
```

## 烧录步骤

### 方法一：使用 STM32CubeProgrammer（推荐）

```bash
# 1. 连接 ST-Link 到主控板（Core Board）的 SWD 接口

# 2. 打开 STM32CubeProgrammer

# 3. 选择连接方式：ST-Link -> Connect

# 4. 打开编译好的 .bin 或 .hex 文件

# 5. 点击 "Download" 按钮烧录

# 6. 烧录完成后，点击 "Run" 启动程序
```

### 方法二：使用命令行

```bash
# 使用 st-flash（需要安装 stlink 工具）
st-flash write Core-STM32F4-fw.bin 0x08000000

# 或使用 STM32_Programmer_CLI
STM32_Programmer_CLI -c port=SWD -w Core-STM32F4-fw.bin 0x08000000 -v
```

### 方法三：使用 DFU 模式（USB）

```bash
# 1. 将主控板 BOOT0 引脚接高电平（3.3V）
# 2. 复位主控板，进入 DFU 模式
# 3. 使用 USB 线连接主控板到电脑

# 4. 使用 dfu-util 烧录
dfu-util -a 0 -s 0x08000000:leave -D Core-STM32F4-fw.bin

# 5. 烧录完成后，断开 BOOT0 连接，重新上电
```

## 验证是否成功

```bash
# 1. 使用串口工具连接主控板（波特率 115200）

# 2. 发送测试命令
!TEACH 0.5
# 预期响应: "Teach mode ON, current=0.50A"

# 3. 尝试拖动机械臂，应该可以轻松移动

# 4. 退出示教模式
!TEACHOFF
# 预期响应: "Teach mode OFF"

# 5. 机械臂应该保持在当前位置
```

## 技术原理

### 电流环工作流程

```
PC 端 (Python)
    ↓ 串口命令: "!TEACH 0.5"
主控板 (STM32F4)
    ↓ 解析命令
    ↓ 调用 SetTeachMode(true, 0.5)
    ↓ CAN 命令 0x03 (SetCurrentSetPoint)
6个关节驱动板 (STM32F1)
    ↓ 切换到 MODE_COMMAND_CURRENT
    ↓ 设置电流设定点为 0.5A
电机驱动芯片 (TB67H450)
    ↓ 输出恒定电流
步进电机
    ↓ 产生可预测的阻力矩
```

### 关键代码解析

```cpp
// dummy_robot.cpp - SetTeachMode 实现
void DummyRobot::SetTeachMode(bool _enable, float _current)
{
    if (_enable)
    {
        // 1. 标记为未使能状态，避免与运动指令冲突
        isEnabled = false;
        
        // 2. 使能电机驱动器（但切换到电流模式）
        motorJ[ALL]->SetEnable(true);
        
        // 3. 给每个关节设置电流设定点
        //    这会触发关节驱动板切换到电流模式
        for (int i = 1; i <= 6; i++)
        {
            motorJ[i]->SetCurrentSetPoint(_current);
        }
    }
    else
    {
        // 退出示教模式：
        // 1. 获取当前角度
        UpdateJointAngles();
        
        // 2. 将当前位置设为目标位置（保持不动）
        for (int i = 1; i <= 6; i++)
        {
            motorJ[i]->SetAngle(currentJoints.a[i - 1] - initPose.a[i - 1]);
        }
        
        // 3. 自动切换回位置模式（SetAngle 触发）
        isEnabled = true;
    }
}
```

### 关节驱动板电流模式

```cpp
// Ctrl-Step-Driver-STM32F1-fw/Ctrl/Motor/motor.cpp

void Motor::Controller::SetCurrentSetPoint(int32_t _cur)
{
    // 如果当前不是电流模式，切换到电流模式
    if (modeRunning != Motor::MODE_COMMAND_CURRENT)
        SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
    
    // 设置电流设定点（单位：mA）
    goalCurrent = _cur;
}

// 在主循环中，电流模式的处理：
void Motor::CloseLoopControlTick()
{
    switch (controller->modeRunning)
    {
        case MODE_COMMAND_CURRENT:
            // 直接计算电流输出
            controller->CalcCurrentToOutput(controller->softCurrent);
            break;
        // ... 其他模式
    }
}
```

## 故障排除

### 问题1：无法进入示教模式

**症状：** 发送 `!TEACH 0.5` 后没有响应

**排查：**
1. 检查固件是否已正确烧录
2. 检查串口连接是否正常
3. 确认命令格式正确（注意大小写和空格）

### 问题2：进入示教模式后仍然拖不动

**症状：** 发送 `!TEACH 0.5` 后，机械臂还是很重

**排查：**
1. 尝试更小的电流值：`!TEACH 0.2`
2. 检查关节驱动板固件是否支持电流模式
3. 检查电机驱动芯片是否正常工作

### 问题3：退出示教模式后位置偏移

**症状：** `!TEACHOFF` 后，机械臂突然跳动

**原因：** 位置环和电流环切换时的瞬间误差

**解决：** 这是正常现象，轻微跳动是允许的。如果跳动过大，检查：
1. 编码器是否正常工作
2. 位置更新是否及时

### 问题4：电流设置不生效

**症状：** 调整电流值后，阻力没有明显变化

**排查：**
1. 确认电流值在有效范围内（0.1A ~ 2.0A）
2. 检查关节驱动板固件版本
3. 某些关节可能有不同的减速比，感受会不同

## 注意事项

### 安全警告

1. **进入示教模式前**：确保机械臂周围没有障碍物
2. **拖动时**：用双手稳住机械臂，避免突然松手
3. **电流设置**：不要设置过大的电流（>1.5A），可能导致电机发热
4. **紧急停止**：遇到异常立即发送 `!STOP`

### 性能提示

1. **电流越小**：越容易拖动，但可能无法保持位置
2. **电流越大**：阻力越大，但位置保持能力越强
3. **推荐值**：
   - 快速示教：0.2 ~ 0.4 A
   - 精确示教：0.5 ~ 0.8 A
   - 正常工作：1.0 ~ 2.0 A

## 相关文件参考

### 主控板固件
- `Core-STM32F4-fw/Robot/instances/dummy_robot.cpp` - 机器人控制逻辑
- `Core-STM32F4-fw/Robot/actuators/ctrl_step/ctrl_step.cpp` - CAN 通信协议
- `Core-STM32F4-fw/UserApp/protocols/ascii_protocol.cpp` - 串口命令解析

### 关节驱动板固件
- `Ctrl-Step-Driver-STM32F1-fw/Ctrl/Motor/motor.cpp` - 电机控制核心
- `Ctrl-Step-Driver-STM32F1-fw/UserApp/protocols/interface_can.cpp` - CAN 命令处理

---

**完成！** 烧录新固件后，即可使用 PC 端控制器的示教功能。
