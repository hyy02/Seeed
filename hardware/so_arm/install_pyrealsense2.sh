#!/bin/bash
set -e

echo "🚀 [1/6] 安装依赖..."
# sudo apt-get update
# sudo apt-get install -y \
#    git cmake build-essential pkg-config \
#    libusb-1.0-0-dev libssl-dev libudev-dev \
#    libgtk-3-dev
# sudo apt-get install -y freeglut3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

echo "🚀 [2/6] 克隆 librealsense 源码..."
# if [ ! -d librealsense ]; then
#    git clone https://github.com/IntelRealSense/librealsense.git
# fi
cd librealsense
git pull

echo "🚀 [3/6] 设置 udev 规则..."
./scripts/setup_udev_rules.sh

echo "🚀 [4/6] 开始编译 (带 CUDA + Python 绑定)..."
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=true \
    -DFORCE_RSUSB_BACKEND=false \
    -DBUILD_WITH_CUDA=true \
    -DBUILD_PYTHON_BINDINGS=bool:true

make -j$(nproc)

echo "🚀 [5/6] 安装 librealsense..."
sudo make install

ln -sf /usr/lib/aarch64-linux-gnu/libstdc++.so.6 /home/cics/miniconda3/envs/lerobot/lib/libstdc++.so.6

echo "✅ 安装完成！现在可以运行: python3 -c 'import pyrealsense2 as rs; print(rs.__version__)'"

