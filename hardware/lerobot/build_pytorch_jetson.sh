#!/bin/bash
set -e

echo "🚀 [1/6] 更新系统并安装依赖..."
# sudo apt update
# sudo apt install -y \
#     git cmake ninja-build build-essential \
#     python3-dev python3-pip libopenblas-dev libblas-dev \
#     libeigen3-dev gfortran libffi-dev

echo "🚀 [2/6] 升级 pip/setuptools/wheel..."
pip3 install --upgrade pip setuptools wheel

echo "🚀 [3/6] 克隆 PyTorch 源码..."
if [ ! -d "pytorch" ]; then
    git clone --recursive https://github.com/pytorch/pytorch
    cd pytorch
else
    cd pytorch
    # git pull
    git submodule sync
    git submodule update --init --recursive
fi

# 你可以切换到特定版本，比如：
git checkout v2.6.0
git submodule update --init --recursive

echo "🚀 [4/6] 设置编译环境变量 (Orin: compute capability 8.7)..."
export USE_NCCL=0
export USE_CUDA=1
export USE_CUDNN=1
export USE_MKLDNN=0
export TORCH_CUDA_ARCH_LIST="8.7"
export MAX_JOBS=6   # 自动检测 CPU 核心

echo "🚀 [5/6] 开始编译 PyTorch (可能需要 1-3 小时)..."
python3 setup.py bdist_wheel

echo "🚀 [6/6] 安装生成的 wheel..."
cd dist
pip install torch-*.whl

echo "✅ 编译和安装完成！验证 PyTorch..."
python3 - <<'EOF'
import torch
print("PyTorch version:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
print("CUDA version:", torch.version.cuda)
EOF

