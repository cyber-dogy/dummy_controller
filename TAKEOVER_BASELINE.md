# Dummy Controller 接手基线

这份文档把当前项目的默认基线固定下来，后续开发、验收和问题排查都以这里为准。

## 当前正式入口

- 主入口：`src/main.py`
- 主窗口：`src/gui/main_window_with_mujoco.py`
- MuJoCo 独立入口：`mujoco_sim/src/qt_integration.py`

以下文件视为历史遗留，不作为继续开发基线：

- `src/gui/main_window.py`
- `src/gui/ik_dialog.py`

## 当前核心约束

- Sim2Real 映射以 `mujoco_sim/src/qt_integration.py::firmware_to_urdf()` 为准
- 全局关节限位以 `src/utils/config.py::JOINT_LIMITS` 为准
- `J3` 最小值固定为 `35°`
- 真机示教模式以 `src/core/robot.py::enter_teach_mode()` / `exit_teach_mode()` 的当前协议行为为准
- 退出示教模式的顺序固定为：读当前位置 → 使能 → 立即保持当前位置

## 当前工程结论

- 主功能链已经成型：串口控制、MuJoCo 仿真、ghost 影子、IK、仿真轨迹规划、真机示教都已接上
- 当前最大风险不在功能缺失，而在真机闭环验证不足
- `J5` 方向映射基于实机经验修正，后续改动前必须再做单轴验证
- MuJoCo `joint6` 范围与真机 `J6 ±720°` 不完全一致，仿真结果不能直接等同于真机无限转

## 本轮固化内容

- IK 已迁移到后台线程，避免主线程短时冻结
- 仿真轨迹页已支持保存/加载，默认复用 `TeachMode` 的轨迹文件格式
- 根目录 `requirements.txt` 已补齐 `mujoco` 依赖
- 新增 `scripts/verify_baseline.py` 作为接手后的快速基线检查脚本

## 验收建议

先执行：

```bash
cd 3.Software/dummy-controller
source mujoco_sim/.venv_mujoco/bin/activate
python scripts/verify_baseline.py
```

再做人工验收：

- `python src/main.py`
- `python mujoco_sim/src/qt_integration.py`
- 真机串口连接、示教模式进出、急停、轨迹发送

## 暂不进入实现的事项

- 不在本轮接摄像头
- 不重构 MuJoCo XML 与 meshdir 路径
- 不恢复历史窗口/对话框实现
