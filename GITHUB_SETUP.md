# GitHub 仓库设置完成

## 仓库地址

**GitHub Repository:** https://github.com/cyber-dogy/dummy_controller

## 分支结构

```
main (默认分支)
├── 所有核心功能
├── MuJoCo 可视化（已合并）
└── 文档和示例

feature/mujoco-visualization (功能分支)
├── 与 main 同步
└── 用于开发新功能
```

## 推送的文件

```
dummy-controller/
├── src/
│   ├── core/
│   │   ├── robot.py              # 机器人控制核心
│   │   └── teach_mode.py         # 示教功能核心
│   ├── gui/
│   │   ├── main_window.py        # 主窗口
│   │   ├── main_window_with_mujoco.py  # 集成MuJoCo版本
│   │   └── teach_dialog.py       # 示教对话框
│   ├── mujoco_viewer/            # MuJoCo 3D可视化
│   │   ├── __init__.py
│   │   ├── dummy_robot.xml       # MuJoCo模型
│   │   ├── robot_visualizer.py   # 核心可视化
│   │   └── qt_integration.py     # Qt集成
│   └── utils/
│       └── config.py             # 配置文件
├── docs/                         # 文档
├── examples/                     # 示例代码
├── requirements.txt              # 依赖
└── *.md                          # 各种README
```

## 使用方法

### 1. 克隆仓库

```bash
git clone https://github.com/cyber-dogy/dummy_controller.git
cd dummy_controller
```

### 2. 安装依赖

```bash
pip install -r requirements.txt
```

### 3. 运行基础版本

```bash
python -m src.main
```

### 4. 运行 MuJoCo 版本

```bash
python -m src.gui.main_window_with_mujoco
```

## 主要功能

### 基础功能
- ✅ PyQt6 GUI 控制界面
- ✅ 串口通信（USB-CAN）
- ✅ 电流环示教模式
- ✅ 轨迹记录与回放
- ✅ 关节限位保护
- ✅ CAN总线通信保护

### MuJoCo 可视化（新增）
- ✅ 实时 3D 模型显示
- ✅ 正解计算（末端位姿）
- ✅ 数值逆解
- ✅ 多视角切换
- ✅ Qt 集成显示

## Git 操作指南

### 查看分支
```bash
git branch -a
```

### 切换分支
```bash
git checkout feature/mujoco-visualization
```

### 拉取最新代码
```bash
git pull origin main
```

### 提交更改
```bash
git add .
git commit -m "Your commit message"
git push origin main
```

### 创建新分支
```bash
git checkout -b feature/your-new-feature
git push -u origin feature/your-new-feature
```

## MuJoCo 模型说明

模型文件：`src/mujoco_viewer/dummy_robot.xml`

使用木子晓汶版本的 STL 文件：
- base.stl
- joint1.stl
- joint23.stl
- joint4.stl
- joint5.stl
- joint6.stl

DH 参数与实物一致，确保正解准确。

## 后续开发

建议的 Git 工作流：

1. **开发新功能**
   ```bash
   git checkout -b feature/new-feature
   # 开发代码
   git add .
   git commit -m "Add new feature"
   git push -u origin feature/new-feature
   ```

2. **合并到 main**
   ```bash
   git checkout main
   git merge feature/new-feature
   git push origin main
   ```

3. **发布版本**
   ```bash
   git tag -a v1.0.0 -m "Release version 1.0.0"
   git push origin v1.0.0
   ```

## 文档索引

- `README.md` - 项目总览
- `MUJOCO_README.md` - MuJoCo 可视化说明
- `README_TeachMode.md` - 示教功能指南
- `FIRMWARE_TEACH_MODE.md` - 固件修改指南
- `BOUNDARY_PROTECTION_SUMMARY.md` - 边界保护说明
- `CAN_PROTECTION_SUMMARY.md` - CAN总线保护说明

## 完成！

项目已成功推送到 GitHub，包含完整的 MuJoCo 3D 可视化功能！🎉
