#!/bin/bash
# MuJoCo 环境安装脚本

set -e

echo "========================================="
echo "  MuJoCo Sim 环境配置"
echo "========================================="

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到 python3"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "Python 版本: $PYTHON_VERSION"

# 创建虚拟环境
echo ""
echo "[1/4] 创建虚拟环境 .venv_mujoco ..."
if [ -d ".venv_mujoco" ]; then
    echo "      虚拟环境已存在，跳过创建"
else
    python3 -m venv .venv_mujoco
    echo "      虚拟环境创建成功"
fi

# 激活虚拟环境
echo ""
echo "[2/4] 激活虚拟环境 ..."
source .venv_mujoco/bin/activate

# 升级 pip
echo ""
echo "[3/4] 升级 pip ..."
pip install --upgrade pip

# 安装依赖
echo ""
echo "[4/4] 安装依赖 ..."
pip install -r requirements.txt

# 验证安装
echo ""
echo "验证安装 ..."
python3 -c "import mujoco; print(f'MuJoCo 版本: {mujoco.__version__}')"
python3 -c "import numpy; print(f'NumPy 版本: {numpy.__version__}')"
python3 -c "import scipy; print(f'SciPy 版本: {scipy.__version__}')"
python3 -c "import PyQt6; print(f'PyQt6 已安装')"

echo ""
echo "========================================="
echo "  ✅ 环境配置完成！"
echo "========================================="
echo ""
echo "使用方法:"
echo "  source .venv_mujoco/bin/activate"
echo "  python3 src/robot_visualizer.py"
echo ""
