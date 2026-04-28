# Dummy V2 六轴机械臂控制系统 — 面试技术文档

> 目标岗位：具身智能 / 机器人控制 / Sim-to-Real  
> 项目定位：从零构建具备 **仿真-真机闭环、示教-回放、数值逆解** 能力的六轴臂控制栈

---

## 一、项目全景

| 维度 | 内容 |
|------|------|
| 硬件 | Dummy V2 六轴串联机械臂，步进电机 + 减速器，CAN 总线通信，串口桥接（ACM） |
| 仿真 | MuJoCo 3.x，MJCF 由厂商 URDF 人工转换，含网格/关节/执行器/传感器完整定义 |
| 软件栈 | Python + PyQt6 GUI，MuJoCo Python binding，scipy 数值优化，串口协议自实现 |
| 代码规模 | ~2600 行，3 大核心模块：qt_integration / main_window / teach_mode |
| 核心能力 | Sim2Real 映射、影子渲染、连续轨迹示教、数值 IK、CAN 总线保护 |

---

## 二、核心技术一：Sim2Real — 仿真到真实的运动映射与控制

### 2.1 问题背景

Sim2Real 最常被提及的挑战是 **动力学 gap**（摩擦、惯量、柔性）；但在运动学层面，同样存在一个更基础却容易被忽视的 gap：**坐标系约定不一致**。

本项目的固件与 URDF 之间存在三类系统性偏差：

| 偏差类型 | 具体表现 |
|----------|----------|
| 单位制 | 固件：角度制（°）；URDF/MuJoCo：弧度制（rad） |
| 轴向符号 | URDF 各关节轴方向与固件正方向不完全对应（6 轴中有 5 轴需要处理符号） |
| 零位偏置 | URDF 的 J3 零位 = 固件的 L-Pose（J3=90°），存在 90° 系统偏置 |

### 2.2 映射函数 `firmware_to_urdf()`

```python
def firmware_to_urdf(fw_angles_deg):
    j1, j2, j3, j4, j5, j6 = fw_angles_deg
    r = np.deg2rad
    return [
        -r(j1),        # joint1 axis=−Z：取反
        -r(j2),        # joint2 axis=+X：URDF 轴向与固件相反，取反
         r(j3 - 90),   # joint3 axis=−X：URDF 零位=固件 L-Pose，补偿 90°
        -r(j4),        # joint4 axis=−Y：取反
         r(j5),        # joint5 axis=−X：实测与固件同向，不取反
        -r(j6),        # joint6 axis=−Y：取反
    ]
```

**推导方法论：**
1. 从 URDF 的 `<axis xyz>` 字段读取各关节旋转轴
2. 判断轴向与固件正方向是否同向（靠硬件实测确认，不能纯靠理论）
3. J3 的 90° 偏置来自 URDF 零位几何分析：URDF 零位时 link3 竖直（即固件的 L-Pose 姿态 J3=90°）
4. J5 **是本项目中唯一与轴向理论推导不符的关节**（axis=−X 理论应取反，但实测同向），体现了 Sim2Real 必须靠实测验证的原则

**技术亮点：** 这套映射是可微的纯线性变换，可以直接嵌入梯度优化（如 IK、轨迹优化），也可以作为强化学习 obs/action 空间的归一化层。

### 2.3 低通滤波平滑：解决真机抖动传导到仿真

**问题：** 真机位置以 2 Hz（串口轮询）更新，直接写入仿真会导致动画跳变。

**方案：指数移动平均（EMA）**

```python
# 每帧（33ms）执行
for i in range(6):
    disp[i] += α * (target[i] - disp[i])   # α = 0.20
```

- `α = 0.20`：约 0.4 秒收敛到目标（5 帧内达到 67%，15 帧达到 95%）
- 本质是一阶 IIR 低通滤波器，截止频率 $f_c = -\ln(1-\alpha) / (2\pi T) \approx 1.06$ Hz
- `snap_to_angles()` 方法强制 `disp = target`，用于加载预设姿态时的瞬时跳转

**与业界对比：** ROS2 的 `joint_state_publisher` 用滑动平均；Isaac Gym 用 PD 控制器隐式平滑；本实现轻量但逻辑相同。

### 2.4 真机通信协议

```
查询位置：  #GETJPOS\n
响应格式：  ok j1 j2 j3 j4 j5 j6
运动指令：  >j1,j2,j3,j4,j5,j6,speed\n
使能/禁用：  !START\n  /  !DISABLE\n
急停：      !STOP\n
```

- 串口波特率 115200，轮询周期 500ms（UpdateThread 后台线程）
- 仿真同步定时器 50ms（20 Hz），与串口轮询解耦——仿真以 EMA 平滑内插，真机以 500ms 更新
- **线程安全：** 所有串口读写加 `threading.Lock()`，避免 GUI 线程与轮询线程竞争

### 2.5 关节限位系统

```python
JOINT_LIMITS = [
    (-170, 170),   # J1
    (-75,    0),   # J2
    (35,   180),   # J3 — 固件硬限位，0~35° 物理不可达
    (-170, 170),   # J4
    (-120, 120),   # J5
    (-720, 720),   # J6
]
```

J3 下限 **35°** 而非 0° 是关键细节：固件层面存在硬件限位，软件必须遵守，否则会触发固件保护导致通信中断。这与工业机器人的软限位 + 硬限位双层保护架构完全一致。

**小结（面试口述版）：**
> Sim2Real 的映射核心是一个六维线性变换，解决单位制、轴向和零位三类 gap。
> 真机→仿真用 EMA 滤波消除抖动，仿真→真机用限位截断保护硬件。
> 全链路实现了 20 Hz 的闭环同步，真机动作在仿真中毫秒级可见。

---

## 三、核心技术二：仿真运动规划与示教

### 3.1 设计哲学：影子机制（Ghost Rendering）

**核心问题：** 轨迹规划时，仿真视图既要显示真机实时状态（监控），又要显示规划轨迹（预览），两者不能互相覆盖。

**解决方案：双模型分层渲染**

```
┌─────────────────────────────────────────┐
│         MuJoCo 渲染帧（最终输出）         │
│                                          │
│  70% × 主体渲染（灰色，跟随真机）          │
│ +45% × 影子渲染（蓝色，跟随规划滑块）      │
│ = Alpha 合成（additive blending）         │
└─────────────────────────────────────────┘
```

**实现细节：**

```python
# 三套独立的 MjModel/MjData/Renderer
self.model    = mujoco.MjModel.from_xml_path(xml)   # 主体
self.ghost_model = mujoco.MjModel.from_xml_path(xml) # 影子
self.ik_model    = mujoco.MjModel.from_xml_path(xml) # IK 专用

# 影子着色（全部几何体改为蓝色）
for i in range(self.ghost_model.ngeom):
    self.ghost_model.geom_rgba[i] = [0.15, 0.45, 1.0, 1.0]

# 渲染合成
rgb_main  = self.renderer.render()       # 主体帧
rgb_ghost = self.ghost_renderer.render() # 影子帧
rgb_out   = np.clip(rgb_main * 0.70 + rgb_ghost * 0.45, 0, 255).astype(np.uint8)
```

**为什么加载三份模型而不共享：**
- MuJoCo 的 `MjData` 是 `MjModel` 的运行时状态，FK/IK 计算会修改 `qpos`
- 三个任务（主渲染、影子渲染、IK 优化）并发运行且互相异步，必须隔离状态
- 这与 Isaac Lab 中多环境并行仿真的"环境隔离"思想一致

### 3.2 仿真示教录制：连续轨迹而非离散路点

**旧方案问题（离散路点）：**
- 用户手动添加路点 → 机械臂直线跳跃（关节空间线性插值，无过渡）
- 路点间缺乏时间信息 → 无法控制运动节奏
- CAN 总线收到多条指令堆积 → 丢步

**新方案：10 Hz 连续轨迹录制**

```python
class TrajectoryPoint:
    timestamp: float       # 相对时间（秒）
    angles:    List[float] # 6 个关节角度

# 录制循环（Qt 定时器，100ms 周期）
def _sim_rec_tick(self):
    angles = [slider.value() for slider in self.sim_sliders]
    t = time.time() - self.sim_rec_start
    self.sim_trajectory.append(TrajectoryPoint(t, angles))
```

**类比：** 这与机器人示教器（Teach Pendant）的轨迹录制原理完全相同，也与模仿学习（Imitation Learning）的数据采集格式兼容。录制的 `List[TrajectoryPoint]` 可以直接作为行为克隆（Behavior Cloning）的训练数据。

### 3.3 真机示教：拖动示教（Kinesthetic Teaching）

```
流程：
  !DISABLE → 电机断开位置控制 → 用户手动拖动关节
     ↓ 10 Hz 采样 → TrajectoryPoint 列表
  !START → 读编码器当前位置 → 立即保持（防跳位）
     ↓
  PlaybackThread → 按时间戳节奏回放
```

**关键技术细节：**

1. **示教模式入口：** 发送 `!DISABLE` 而非降低 PID 增益——步进电机驱动器在降增益时仍施加保持电流，必须完全断电才能轻松拖动（只剩减速器机械摩擦，约等于断电手感）

2. **退出时防跳位序列：**
```python
def exit_teach_mode(self):
    current_pos = self.get_position()   # Step1: 断电状态下读编码器
    self.ser.write(b'!START\n')         # Step2: 上电使能
    time.sleep(0.3)                     # Step3: 等待驱动器稳定
    self.move_to(current_pos, speed=10) # Step4: 立即发送保持指令
    # 如果跳过 Step4，电机会猛地回到上次记忆的目标位置
```

3. **轨迹平滑（adaptive smooth）：**
```
原始轨迹 → 去停滞段（Δ<0.3°视为静止）
         → Gaussian 加权移动平均（strength 1~10）
         → 均匀时间重采样（100ms 间隔）
```
平滑核：$K(i) = e^{-\frac{1}{2}\left(\frac{i-h}{\sigma}\right)^2}$，$\sigma = w/6$

### 3.4 CAN 总线保护：按时间戳回放

**问题：** 如果逐帧以最快速度发送指令，CAN 总线（或串口 FIFO）会堆积，导致：
- 后续指令覆盖前序（丢步）
- 电机接收乱序指令（抽搐）

**方案：`SimPlaybackThread` 严格按录制时间戳等待**

```python
def run(self):
    for idx, pt in enumerate(self.trajectory):
        self.robot.move_to(pt.angles)        # 发送当前点
        if idx < n - 1:
            dt = (traj[idx+1].timestamp - pt.timestamp) / self.speed_ratio
            time.sleep(max(0.02, dt))        # 严格等待下一点的时间间隔
```

- `max(0.02, dt)` 保证最小 20ms 间隔（50 Hz 上限），防止速度倍率过高时塞满 CAN
- 速度倍率（0.1×~5×）通过缩放时间间隔实现，而非修改速度参数
- 这与 ROS2 `FollowJointTrajectory` action 的执行机制本质相同

### 3.5 轨迹优化：Ramer-Douglas-Peucker 稀疏化

```python
def decimate_trajectory(trajectory, tolerance=1.0):
    # RDP 算法：保留关键拐点，去除冗余直线段上的采样点
    # tolerance: 允许偏差（度），越大点数越少
```

对于 10 Hz 采样的 10 秒轨迹（100 点），RDP 可在保持形状的前提下压缩到 30~50 点，降低 CAN 总线负载 50~70%。

**小结（面试口述版）：**
> 规划层用"影子渲染"分离真机监控与轨迹预览，双 MjModel 隔离状态互不干扰。
> 录制层模仿示教器，以 10 Hz 连续采样关节角度+时间戳，直接兼容模仿学习数据格式。
> 执行层严格遵循录制节奏发送指令，用 RDP 稀疏化降低总线负载，从根源解决 CAN 丢步。

---

## 四、核心技术三：正/逆运动学控制

### 4.1 正运动学（FK）：MuJoCo 物理引擎作为运动学解算器

```python
mujoco.mj_forward(model, data)   # 全量正向传播
ee_pos = data.site_xpos[site_id] # 末端执行器位置（米）
ee_mat = data.site_xmat[site_id] # 旋转矩阵 3×3
```

**从旋转矩阵提取 RPY：**
```python
roll  = arctan2(R[2,1], R[2,2])
pitch = arctan2(-R[2,0], sqrt(R[2,1]²+R[2,2]²))
yaw   = arctan2(R[1,0], R[0,0])
```

**优势：** 相比手写 DH 参数正解，MuJoCo 的 FK 是基于完整 MJCF 模型（含精确几何偏置）计算，精度更高，且与仿真渲染的几何完全一致，不存在"解算器与可视化不匹配"的问题。

**实测关键位置正解验证：**
```
REST  [0,-75,180,0,0,0] → EE = (12.8, 186.1, 147.3) mm
L-Pose [0,0,90,0,0,0]  → EE = (12.8, 343.1, 353.7) mm
臂直立 [0,0,0,0,0,0]   → EE = (12.8,  -5.1, 604.6) mm
```

### 4.2 逆运动学（IK）：scipy 数值优化 + 多初始猜测

**方法：** 将 IK 转化为约束优化问题

$$\min_{\theta \in \Theta} \| \text{FK}(\theta) - p_{\text{target}} \|^2$$

其中 $\Theta$ 是关节限位构成的超矩形约束集。

**实现：**

```python
def compute_ik(self, target_xyz_mm, initial_fw_deg=None, tolerance_mm=8.0):
    target_m = np.array(target_xyz_mm) / 1000.0

    def cost(fw_deg):
        pos = self._fk_on_ik_model(fw_deg)   # 用隔离的 ik_model 计算 FK
        return float(np.sum((pos - target_m) ** 2))

    bounds = [(lo, hi) for lo, hi in JOINT_LIMITS]

    # 多初始猜测：覆盖典型工作空间姿态
    guesses = [
        initial_fw_deg,          # 用户当前姿态（最可能接近）
        [0, 0, 90, 0, 0, 0],    # L-Pose
        [0, -30, 120, 0, 0, 0], # 低位
        [90, 0, 90, 0, 0, 0],   # J1旋转90°
        [-90, 0, 90, 0, 0, 0],
        [0, -50, 150, 0, 0, 0],
    ]
    best = min(
        (minimize(cost, g, method='L-BFGS-B', bounds=bounds) for g in guesses),
        key=lambda r: r.fun
    )
    return best.x if best.fun < (tolerance_mm/1000)**2 else None
```

**L-BFGS-B 选择原因：**
- 支持盒约束（Box constraints）—— 直接处理关节限位，无需惩罚项
- 拟牛顿法，收敛速度比梯度下降快 10~50 倍
- 无需解析雅可比，自动数值差分（MuJoCo FK 不暴露雅可比 API 时）

**多初始猜测的必要性：** IK 代价函数存在多个局部极小值（冗余自由度 + 非凸约束），单次优化依赖初始点，多猜测可覆盖工作空间主要配置。

**性能：** 单次 IK 约 200~500ms（Python 层，六轮优化），在 UI 层通过 `processEvents()` 保持响应性；生产环境可移至 QThread 或用 C++ 扩展加速。

**可达性判断：** `tolerance_mm=8.0`（8mm 阈值），超出则返回 `None` 并提示用户调整目标点。工作空间约为：X ±380mm，Y −20~400mm，Z 100~640mm（由正解枚举关节限位端点推算）。

### 4.3 IK 结果流转

```
计算逆解
  → 自动显示影子（仿真预览目标姿态）
  → 「应用到控制」→ 更新左侧关节滑块
  → 「发送到真机」→ robot.move_to(ik_angles)
  → 验证正解误差标注在结果面板
```

**小结（面试口述版）：**
> FK 直接复用 MuJoCo 物理引擎，与仿真渲染几何完全对齐，实测验证过三个关键姿态。
> IK 用 scipy L-BFGS-B 将"找关节角"转化为有界约束优化，多初始猜测覆盖冗余解空间。
> 完整链路：用户输 XYZ → IK 解 → 影子预览 → 一键下发真机，形成闭环。

---

## 五、业界包装：如何讲这个项目

### 5.1 项目定位关键词（对标 JD）

| 你做了什么 | 业界术语 | 对标技术 |
|-----------|---------|---------|
| URDF→MJCF转换+轴向映射 | **Sim2Real 运动学对齐** | IsaacGym domain randomization |
| EMA 滤波平滑 | **状态估计与滤波** | Kalman Filter / Complementary Filter |
| 拖动示教+时间戳录制 | **Kinesthetic Teaching / Teleoperation** | ACT, DexMimic 数据采集 |
| 双模型影子渲染 | **数字孪生（Digital Twin）** | NVIDIA Omniverse |
| 数值 IK（L-BFGS-B） | **运动规划** | MoveIt2 KDL/TRAC-IK |
| CAN 时间戳节奏回放 | **实时控制 / 轨迹追踪** | ROS2 FollowJointTrajectory |
| TrajectoryPoint 格式 | **Demonstration Data Collection** | HDF5 / RLDS 格式预备 |

### 5.2 面试中的"亮点包装"话术

**话术 1：Sim2Real**
> "这个项目核心解决了仿真与真机之间的运动学 gap。我分析了每个关节的 URDF 轴向与固件约定的差异，设计了一个映射函数，发现 J5 与理论推导方向相反，必须靠实测来校准——这正是 Sim2Real 的核心难点：不能只信仿真，要用真机数据验证。"

**话术 2：数字孪生**
> "仿真界面实现了双模型分层渲染，主体实时跟随真机，蓝色影子跟随规划轨迹，两者同屏可见。这是数字孪生的基本形态：真实与虚拟并行运行，互相参照。"

**话术 3：示教数据**
> "我把仿真规划的轨迹直接存成 TrajectoryPoint 列表（时间戳+关节角），这个格式一步之遥就是行为克隆（Behavior Cloning）的训练数据。下一步接上 Diffusion Policy，这套系统就变成了端到端模仿学习的采集平台。"

**话术 4：工程可靠性**
> "CAN 总线通信是这个系统的脆弱点。我用严格的时间戳节奏发送指令，而不是尽可能快地发——这防止了指令堆积导致的丢步和抖动。这个思路和 ROS2 的轨迹动作服务器完全一致。"

---

## 六、下一步：向世界模型与具身智能进发

### 6.1 当前系统的数据价值

本系统已经具备：
- 结构化轨迹数据：`List[TrajectoryPoint(timestamp, joint_angles)]`
- 末端执行器位置（6D pose：XYZ + RPY）
- 视觉接口预留（`CameraPlaceholder.set_frame(rgb_ndarray)`）

**一步到位接 HDF5 存储：**
```python
import h5py
with h5py.File('demo_001.hdf5', 'w') as f:
    f['obs/joint_pos']  = [[p.angles]  for p in traj]
    f['obs/ee_pos']     = [[p.ee_xyz]  for p in traj]
    f['obs/image']      = rgb_frames
    f['action/joint']   = [[p.angles]  for p in traj[1:]]
    f.attrs['duration'] = traj[-1].timestamp
```
这就是 **ACT / Diffusion Policy / RoboAgent** 所用的标准数据格式。

### 6.2 从示教数据到模仿学习

```
当前系统（已实现）
  拖动示教 → TrajectoryPoint 列表

第一步扩展（1~2周）
  → 接入 RGB 摄像头 → obs = (joint_angles, ee_pos, image)
  → 存储为 RLDS / HDF5
  → 送入 Behavior Cloning 训练

第二步（1个月）
  → 替换 BC 为 Diffusion Policy
  → 利用 MuJoCo 仿真做数据增强（domain randomization）
  → Sim2Real 验证：仿真训练 → 真机零样本迁移
```

### 6.3 世界模型接入路径

**世界模型定义（RSSM/Dreamer 框架）：**
$$h_t = f(h_{t-1}, z_{t-1}, a_{t-1})$$
$$z_t \sim q(z_t | h_t, o_t)$$

本系统可以作为世界模型的**环境接口**：

| 世界模型组件 | 本系统对应 |
|------------|---------|
| 观测 $o_t$ | RGB图像 + 关节角 + EE位置 |
| 动作 $a_t$ | 关节角增量 / IK 目标位置 |
| 环境步进 | MuJoCo `mj_step()` |
| 真实 rollout | 串口指令 → 真机反馈 |

**具体路径：**
1. 用 MuJoCo 作为**快速想象（imagination）**环境，在仿真中做 model-based RL 规划
2. 仿真规划出的轨迹通过本系统的`SimPlaybackThread` 执行到真机
3. 真机反馈（关节角+图像）回填世界模型，做在线适应

### 6.4 VLA（Vision-Language-Action）接入

```python
# 预留接口：自然语言 → IK 目标
def execute_language_command(command: str):
    # Step1: VLM 解析语义目标
    target_xyz = vlm.grounding(command, camera_frame)
    # Step2: IK 求解
    joint_angles = mujoco_widget.compute_ik(target_xyz)
    # Step3: 执行
    robot.move_to(joint_angles)
```

本系统的 IK 模块就是 VLA 的**运动原语（motor primitive）**层，上接语言/视觉，下接真机。

### 6.5 技术演进路线图

```
当前 ────────────────────────────────────────────► 未来
  │                                                  │
运动学 Sim2Real     +示教数据采集     +世界模型规划     具身智能 Agent
  │                      │                  │              │
映射函数+IK         模仿学习BC/DP      RSSM/Dreamer    VLA端到端
MuJoCo闭环         HDF5数据格式      仿真想象规划     语言指令执行
```

---

## 七、一页 PPT — 珍格格项目

---

### 六轴机械臂全栈控制系统
#### Dummy V2 · Sim2Real · 仿真规划 · 数值 IK

---

**【项目定位】**
从零构建具备仿真-真机闭环的机械臂控制栈，覆盖运动学建模 → 仿真可视化 → 轨迹规划 → 真机执行全链路

---

**【核心亮点】**

🔵 **Sim2Real 运动学对齐**
- 分析 URDF 轴向与固件约定差异，实现 6 轴精确映射（含 J3 的 90° 零位补偿）
- EMA 低通滤波（α=0.2）消除串口抖动，真机运动毫秒级映射到仿真

🔵 **数字孪生影子渲染**
- 三套独立 MjModel 隔离渲染状态：主体跟随真机、蓝色影子跟随规划轨迹
- Alpha 合成实现双机器人同屏，规划与监控互不干扰

🔵 **连续轨迹示教（Kinesthetic Teaching）**
- 10 Hz 连续采样 `{timestamp, joint_angles}`，与 ACT/Diffusion Policy 数据格式兼容
- 拖动示教实现：`!DISABLE` 断电拖动 → 编码器采样 → 上电保位防跳位
- 严格时间戳节奏回放，解决 CAN 总线丢步问题

🔵 **数值逆运动学**
- 复用 MuJoCo FK 作为代价函数，L-BFGS-B 有界优化（6 个初始猜测覆盖冗余解空间）
- XYZ 目标 → 影子预览 → 一键下发，形成 IK 完整闭环

---

**【技术栈】**
`MuJoCo 3.x` · `PyQt6` · `scipy L-BFGS-B` · `串口协议` · `CAN 总线保护`  
`Python 多线程` · `EMA 滤波` · `RDP 轨迹稀疏化` · `URDF/MJCF 转换`

---

**【下一步】**

```
当前：运动学 Sim2Real + 示教数据采集
  ↓
+摄像头 → 模仿学习（Behavior Cloning / Diffusion Policy）
  ↓
+世界模型（RSSM/Dreamer）→ 仿真想象规划 + Sim2Real 迁移
  ↓
+VLA → 语言指令驱动末端执行（IK 作为运动原语层）
```

> 本系统已是具身智能数据采集平台的核心骨架，距离接入大模型只差摄像头与训练脚本

---

**【指标】**
| 维度 | 数值 |
|------|------|
| 仿真同步频率 | 20 Hz |
| 轨迹录制频率 | 10 Hz |
| IK 求解精度 | < 8 mm |
| 代码规模 | ~2600 行，3 核心模块 |
| 关节数 | 6 轴（+夹爪接口预留） |

---

*构建者：[你的名字] · 2024~2025 · 具身智能方向*

---

## 附：面试高频问题 Q&A

**Q: 你的 Sim2Real 和 IsaacGym 的 domain randomization 有什么区别？**
> A: DR 主要解决动力学 gap（摩擦、质量误差），我这里主要解决运动学 gap（坐标系约定）。两者都是 Sim2Real 必须解决的，是正交的两个维度。DR 是下一步要加的，可以在 MuJoCo 里随机化关节摩擦和末端载荷。

**Q: 为什么用数值 IK 而不是解析 IK？**
> A: 六轴串联臂理论上有封闭解，但需要精确的 DH 参数表；而本项目的几何来自 URDF 文件（CAD 软件导出），偏置很多，手写 DH 容易出错。数值 IK 直接复用 MuJoCo FK，几何保证与仿真一致，代价是每次求解 200~500ms，生产环境可换成 TRAC-IK 的 C++ 实现。

**Q: 你的示教数据怎么用于训练？**
> A: 目前存储为 `List[TrajectoryPoint(timestamp, joint_angles)]`，下一步加摄像头后扩展成 `(image, joint_angles, ee_pose)` 三元组，直接存 HDF5，可以送入 ACT 或 Diffusion Policy 的 dataloader。

**Q: CAN 总线问题是怎么发现的？**
> A: 早期版本用路点模式，连续快速发送指令时机械臂出现抽搐和跳位。排查发现是串口 FIFO 缓冲区溢出，后续指令覆盖了前一条还没执行完的指令。改用时间戳节奏发送后问题消失，这和 ROS2 的 `FollowJointTrajectory` 解决方案本质相同。

**Q: 这个项目最难的地方是什么？**
> A: J5 关节的方向问题。URDF 标注 `axis="-1 0 0"`，按理论应该取反，但实测仿真方向与真机相反，说明固件对该关节的正方向定义恰好与 URDF 的负轴方向一致。这让我意识到 Sim2Real 不能只靠理论推导，必须有闭环验证机制。

**Q: 为什么选 MuJoCo 而不是 PyBullet 或 Gazebo？**
> A: MuJoCo 在接触力学上比 PyBullet 精准，Python API 比 Gazebo 轻量，且是 DeepMind/Google 支持的主流 RL 仿真环境（OpenAI Gym、dm_control 都用它）。最重要的是，它的 `mj_forward()` 可以当高精度 FK 计算器用，不需要单独实现正运动学。
