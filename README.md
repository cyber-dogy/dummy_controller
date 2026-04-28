# Dummy V2 机械臂控制器 (PyQt6)

美观的跨平台可视化控制界面

## 系统要求

- Ubuntu 22.04 LTS
- Python 3.10+

## 安装

```bash
pip install -r requirements.txt
```

## 运行

```bash
python src/main.py
```

或

```bash
python -m src.main
```

如果项目使用内置 MuJoCo 环境：

```bash
source mujoco_sim/.venv_mujoco/bin/activate
python scripts/verify_baseline.py
```

## 功能

- 6轴关节独立控制
- 预设位置快速切换
- 实时位置反馈
- 串口自动检测
- 紧急停止保护
- MuJoCo 主视图 + ghost 影子预览
- 数值 IK
- 仿真轨迹录制 / 保存 / 加载 / 回放
