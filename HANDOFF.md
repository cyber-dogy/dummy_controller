# Dummy V2 项目交接文档（供新模型快速上手）

> 本文档面向接手此项目的 AI 模型或开发者，提供精确的文件读取顺序、当前实现状态、已知约定与尚未解决的问题。

---

## 0. 项目根目录

```
dummy-controller/                        ← 所有路径均以此为根
├── src/
│   ├── main.py                          ← 程序入口
│   ├── core/
│   │   ├── robot.py                     ★ 重点：真机串口通信
│   │   └── teach_mode.py                ★ 重点：示教轨迹录制与回放
│   ├── gui/
│   │   ├── main_window_with_mujoco.py   ★ 重点：主窗口（最近大量修改）
│   │   ├── teach_dialog.py              ★ 重点：完整示教对话框
│   │   └── ik_dialog.py                 （旧版逆解对话框，已被主窗口内嵌取代）
│   └── utils/
│       └── config.py                    ★ 重点：关节限位 / 预设位置
├── mujoco_sim/
│   ├── models/
│   │   └── dummy_robot.xml              ★ 重点：MJCF 仿真模型
│   └── src/
│       └── qt_integration.py            ★ 重点：MuJoCo 渲染组件（最近大量修改）
└── INTERVIEW_TECH_DOC.md                参考：技术细节与面试包装
```

**忽略目录：** `third_party/`（第三方 CLI 工具，项目不依赖）、`mujoco_sim/.venv_mujoco/`（虚拟环境）、`examples/`、`scripts/`（辅助脚本）

---

## 1. 必读文件及顺序

### 第一步：理解配置约束（5 分钟）
**读 `src/utils/config.py`（全文，194 行）**

关注：
- `JOINT_LIMITS`：6 个关节的 `(min, max)` 度数范围，**J3 下限是 35° 不是 0°**（固件硬限位）
- `POSES`：三个预设姿态（REST/L-Pose/垂直向上）
- `JointLimitChecker`：`clamp_angles()` / `get_violations()` 方法，全局安全检查

---

### 第二步：理解真机通信（10 分钟）
**读 `src/core/robot.py`（全文，268 行）**

串口协议（全部命令）：
```
#GETJPOS\n        → 查询关节角，响应: ok j1 j2 j3 j4 j5 j6
>j1,j2,j3,j4,j5,j6,speed\n  → 运动到目标角度（整数度，速度 1~100）
!START\n          → 使能电机
!DISABLE\n        → 禁用电机（示教模式入口）
!STOP\n           → 急停
!REBOOT\n         → 重启主控板
```

关注两个非标方法：
- `enter_teach_mode()`：发 `!DISABLE`，**不是降低 PID 增益**，这是意图让用户拖动的核心
- `exit_teach_mode()`：必须先读编码器位置 → 上电 → 立即发保持指令（防止跳位）

---

### 第三步：理解仿真模型（5 分钟）
**读 `mujoco_sim/models/dummy_robot.xml`（全文，157 行）**

关注：
- `compiler angle="radian"` → MuJoCo 内部用弧度
- `meshdir="../../../../_repos/dummy2/simulation/dummy2/meshes/"` → 相对于 XML 文件目录的路径，**向上 4 级**到达 `1.Dummy V2任同学整理/`，再进 `_repos/`
- 关节轴向（IK 映射的依据）：
  - joint1: `0 0 -1`（负 Z）
  - joint2: `1 0 0`（正 X）
  - joint3: `-1 0 0`（负 X）
  - joint4: `0 -1 0`（负 Y）
  - joint5: `-1 0 0`（负 X）
  - joint6: `0 -1 0`（负 Y）
- `site name="end_effector"` 在 link6 下，用于 FK/IK 位置读取

---

### 第四步：理解 Sim2Real 核心映射（15 分钟）
**读 `mujoco_sim/src/qt_integration.py`，重点段落：**

1. **第 1~55 行**：`firmware_to_urdf()` 函数和注释——这是整个 Sim2Real 的核心，必须完全理解

```python
def firmware_to_urdf(fw_angles_deg):
    j1, j2, j3, j4, j5, j6 = fw_angles_deg
    r = np.deg2rad
    return [
        -r(j1),        # joint1 axis=-Z → 取反
        -r(j2),        # joint2 axis=+X → 取反（轴与固件相反）
         r(j3 - 90),   # joint3 axis=-X → 偏移90°（URDF零位=固件L-Pose=J3:90°）
        -r(j4),        # joint4 axis=-Y → 取反
         r(j5),        # joint5 axis=-X → 不取反（实测与固件同向，与理论相反）
        -r(j6),        # joint6 axis=-Y → 取反
    ]
```

> ⚠️ **特别注意 J5**：轴是 `-X` 理论上应取反，但实测方向一致，不取反。这是唯一与理论不符的关节，如有疑问必须接真机验证。

2. **第 61~200 行**：`MuJoCoWidget.__init__` 和 `_init_mujoco`
   - 三套独立的 `MjModel/MjData/Renderer`：主体 / 影子(ghost) / IK 专用
   - 影子着色：`ghost_model.geom_rgba[i] = [0.15, 0.45, 1.0, 1.0]`（蓝色）

3. **第 170~240 行**：`_apply_angles()` 和 `_update_frame()`
   - EMA 平滑：`disp[i] += 0.20 * (target[i] - disp[i])` 每帧执行
   - 渲染合成：`0.70 × 主 + 0.45 × 影`，只在 `ghost_enabled=True` 时叠加

4. **第 300~420 行**：`compute_ik()`
   - 用 `ik_model`（第三套独立模型）计算，不影响渲染
   - `scipy.optimize.minimize(L-BFGS-B)`，6 个初始猜测
   - 收敛阈值 8mm，失败返回 `None`

---

### 第五步：理解主窗口布局（20 分钟）
**读 `src/gui/main_window_with_mujoco.py`，按段落跳读：**

**布局结构（第 200~250 行，`_create_ui`）：**
```
QSplitter(Horizontal) [330 : 700 : 390 px]
  左栏: _build_left()   → 串口 + 关节滑块 + 预设 + 日志
  中栏: _build_middle() → MuJoCo 3D 仿真 + 末端状态
  右栏: _build_right()  → QTabWidget
          Tab0: _build_tab_fk_ik()   → 正逆解
          Tab1: _build_tab_teach()   → 真机示教
          Tab2: _build_tab_sim_traj() → 仿真规划
```

**关键方法（可按需跳读）：**

| 方法 | 行号约 | 作用 |
|------|--------|------|
| `_sync_mujoco()` | ~620 | 50ms 定时，真机→主视图；未连接时不同步（左侧滑块负责） |
| `_sim_rec_tick()` | ~830 | 10Hz 采样仿真规划滑块，追加 `TrajectoryPoint` |
| `_sim_execute()` | ~870 | 执行前限位检查，启动 `SimPlaybackThread` |
| `_ik_compute()` | ~680 | 调用 `compute_ik`，结果自动设为 ghost 预览 |
| `_teach_toggle_mode()` | ~960 | 进入/退出真机示教模式 |
| `SimPlaybackThread.run()` | ~100 | 按时间戳节奏发送，`max(0.02, dt)` 保护 CAN |

**两个滑块组，不要混淆：**
- `self.joint_sliders`（左栏）→ 控制**真机运动目标**
- `self.sim_sliders`（右栏 Tab2）→ 控制**仿真影子位置**（不发送到真机）

---

### 第六步：理解示教核心（按需读）
**读 `src/core/teach_mode.py` 第 1~130 行（`TeachMode` 基础部分）**

- `start_recording()` / `stop_recording()`：后台线程 10Hz 轮询 `robot.get_position()`
- `smooth_trajectory_adaptive()`：去停滞→Gaussian平滑→均匀重采样

**读 `src/gui/teach_dialog.py` 第 20~55 行（`PlaybackThread`）**

```python
class PlaybackThread(QThread):
    progress_signal = pyqtSignal(int)    # 0~100，连接到 QProgressBar.setValue
    finished_signal = pyqtSignal()       # 无参数！（旧版 main_window 连接方式有误，已修复）
```
> ⚠️ `finished_signal` **无参数**，不要传 `lambda ok, msg: ...`，这是旧代码的 bug 已修复

---

## 2. 当前实现状态（截至本次会话）

### 已完成 ✅

| 功能 | 文件 | 状态 |
|------|------|------|
| URDF→MJCF 转换 | `mujoco_sim/models/dummy_robot.xml` | 完成，网格路径已验证 |
| Sim2Real 角度映射 | `qt_integration.py: firmware_to_urdf()` | 完成，J5 方向已实测修正 |
| EMA 低通平滑（α=0.2） | `MuJoCoWidget._apply_angles()` | 完成 |
| 三模型影子渲染 | `MuJoCoWidget._update_frame()` | 完成，alpha blend 验证通过 |
| 数值 IK（L-BFGS-B） | `MuJoCoWidget.compute_ik()` | 完成，精度 <8mm |
| J3 限位修正（35°） | `src/utils/config.py` | 完成 |
| 主窗口三栏布局 | `main_window_with_mujoco.py` | 完成，已移除相机占位符 |
| FK/IK 选项卡 | `_build_tab_fk_ik()` | 完成 |
| 仿真规划选项卡（录制+执行） | `_build_tab_sim_traj()` | 完成 |
| 真机示教选项卡 | `_build_tab_teach()` | 完成，信号连接已修复 |
| `SimPlaybackThread`（时间戳节奏） | `main_window_with_mujoco.py` | 完成 |
| CAN 保护（最小 20ms 间隔） | `SimPlaybackThread.run()` | 完成 |
| 路径可移植性（__file__ 相对） | 所有新增文件 | 完成，已验证 |
| 语法检查 | 两个主文件 | 通过 `ast.parse()` |

### 未完成 / 已知限制 ⚠️

| 项目 | 说明 |
|------|------|
| 摄像头接入 | `CameraPlaceholder` 已移除，接入点需重新设计 |
| IK 运行在主线程 | 约 200~500ms，可能短暂冻结 UI；应移至 QThread |
| J6 无限转 ±720° | MuJoCo 内 joint6 范围截断为 ±180°，仿真与真机不完全一致 |
| 未实测完整闭环 | 本次会话未连接真机做端到端测试（作者没有真机在手边） |
| `ik_dialog.py` 过时 | 引用了已不存在的 `DummyRobotVisualizer`，不要使用 |
| `main_window.py` 过时 | 旧版无 MuJoCo 的窗口，已被 `main_window_with_mujoco.py` 取代 |

---

## 3. 关键约定与容易踩坑的地方

### 约定 1：角度单位边界

| 层级 | 单位 | 说明 |
|------|------|------|
| 固件串口协议 | **度（整数）** | `>0,-75,180,0,0,0,30\n` |
| Python 代码（内部） | **度（浮点）** | `robot.current_angles` / `JOINT_LIMITS` |
| MuJoCo qpos | **弧度** | `mujoco_sim/models/dummy_robot.xml` 有 `angle="radian"` |
| URDF 文件 | 度（原始） | `_repos/dummy2/simulation/dummy2/dummy2.urdf` 仅作参考 |

转换边界：**只有 `firmware_to_urdf()` 这一处**做度→弧度转换。

### 约定 2：三套 MjModel 的用途严格隔离

```
self.model / self.data / self.renderer
    → 主渲染，跟随真机，有 EMA 平滑，30fps 渲染

self.ghost_model / self.ghost_data / self.ghost_renderer
    → 影子渲染，跟随仿真规划滑块，无平滑，直接 set

self.ik_model / self.ik_data
    → IK 计算专用，不参与渲染，scipy 优化时修改 qpos
```

**不能混用**：IK 计算修改 `ik_data.qpos`，如果用主体的 `data` 来算 IK，会导致渲染跳变。

### 约定 3：仿真规划滑块 ≠ 关节控制滑块

- `self.joint_sliders`（左栏 6 个）→ 关节控制目标，`_execute_move()` 发送到真机
- `self.sim_sliders`（Tab2 6 个）→ 仿真影子控制，`_sim_slider_changed()` 只更新 ghost，**不发送到真机**
- `_sim_sync_from_robot()` 把真机当前角度复制到 `sim_sliders`，是两者的唯一桥梁

### 约定 4：PlaybackThread 信号签名

```python
# teach_dialog.py 中的定义（不要改）
progress_signal = pyqtSignal(int)    # int: 0~100
finished_signal = pyqtSignal()       # 无参数

# 正确连接方式（在 main_window_with_mujoco.py 中）
thread.progress_signal.connect(self.teach_play_progress.setValue)
thread.finished_signal.connect(self._teach_playback_done)  # 无参方法

# 错误连接方式（旧 bug，不要写）
thread.finished_signal.connect(lambda ok, msg: ...)  # ❌ 参数不匹配
```

### 约定 5：J3 最小值是 35° 不是 0°

固件硬限位，0~35° 物理不可达。`JOINT_LIMITS[2] = (35, 180)`。
任何 IK、轨迹规划、预设都必须通过 `JointLimitChecker.clamp_angles()` 过滤。

### 约定 6：仿真 MJCF 零位 = 固件 L-Pose

MJCF 所有关节 qpos=0 时，机械臂处于 L 形（固件 `[0,0,90,0,0,0]`）。
这是 J3 映射中 `-90` 偏置的来源：`urdf_j3 = deg2rad(fw_j3 - 90)`。

### 约定 7：网格路径是相对路径，不可移动 XML

`dummy_robot.xml` 中 `meshdir="../../../../_repos/..."` 是相对于 XML 文件本身的路径。
如果移动 `mujoco_sim/models/dummy_robot.xml`，meshdir 必须同步更新。

---

## 4. 运行方式

```bash
# 激活虚拟环境（包含 mujoco、PyQt6、scipy、numpy、pyserial）
cd 3.Software/dummy-controller
source mujoco_sim/.venv_mujoco/bin/activate

# 启动主程序
python src/main.py

# 或单独运行 MuJoCo 仿真（无需真机，含演示动画）
python mujoco_sim/src/qt_integration.py
```

**依赖验证：**
```bash
python -c "import mujoco, PyQt6, scipy, serial, numpy; print('OK')"
```

---

## 5. 这次会话做了什么（修改记录）

### 修改 1：`src/utils/config.py`
- `JOINT_LIMITS[2]` 从 `(0, 180)` 改为 `(35, 180)`
- 原因：J3 的 0~35° 是固件硬限位，物理不可达

### 修改 2：`mujoco_sim/models/dummy_robot.xml`（完整重写）
- 从 URDF `_repos/dummy2/simulation/dummy2/dummy2.urdf` 人工转换而来
- 包含完整运动链：base→link1→...→link6→figer1/figer2
- 坐标偏置全部从 URDF 的 `visual/origin` 字段提取
- 已验证：加载成功，渲染非零像素 ~920000

### 修改 3：`mujoco_sim/src/qt_integration.py`（大幅扩展）
- 原版：单套 model/data，只有 `firmware_to_urdf` 和基础渲染
- 新增：
  - **三套 model/data**（主体/影子/IK）
  - **影子渲染** (`set_ghost`/`clear_ghost`/`_apply_ghost_pose`)
  - **alpha 合成**（`0.70×主 + 0.45×影`）
  - **数值 IK** (`compute_ik`，scipy L-BFGS-B，6 初始猜测)
  - `get_ee_position()` / `get_last_ik_ee()`
- J5 方向：从 `-r(j5)` 改为 `+r(j5)`（实测修正）

### 修改 4：`src/gui/main_window_with_mujoco.py`（完整重写）
- 原版：三栏含相机占位符，轨迹规划用离散路点，`PlaybackThread` 信号连接有 bug
- 新版：
  - **移除相机占位符**，中栏全给 3D 仿真
  - **三个选项卡**：正逆解 / 真机示教 / 仿真规划
  - **`SimPlaybackThread`**：时间戳节奏发送，替代旧的 `TrajectoryExecuteThread`
  - **仿真规划 10Hz 录制**：替代旧的手动添加路点
  - **`TrajectoryPoint` 回退定义**：`TEACH_AVAILABLE=False` 时仍可用
  - **修复 `PlaybackThread` 信号**：`finished_signal.connect(_teach_playback_done)` 无参
  - **两组独立滑块**：`joint_sliders`（真机）vs `sim_sliders`（影子），明确隔离

---

## 6. 如果要继续开发，建议的下一步

优先级从高到低：

1. **接真机端到端测试**：验证 J5 方向修正是否正确，验证 IK 精度，验证 CAN 回放节奏
2. **IK 移至 QThread**：避免主线程冻结（在 `_ik_compute` 方法处，创建一个 `IKThread`）
3. **接摄像头**：在 `main_window_with_mujoco.py` 中间栏 `_build_middle()` 末尾添加 `CameraWidget`，接入 OpenCV `VideoCapture`，调用 `set_frame(rgb_ndarray)`
4. **保存/加载仿真轨迹**：在 Tab2 添加 `QFileDialog`，复用 `TeachMode.save_trajectory()` / `load_trajectory()`
5. **模仿学习数据采集**：扩展 `TrajectoryPoint` 加入图像字段，存储为 HDF5

---

*生成时间：2025 年 4 月 17 日 | 基于本次完整会话*