et -e

echo "🚀 [1/6] 更新系统并安装依赖..."
# sudo apt update
# sudo apt install -y \
#	    build-essential cmake git \
#	        libjpeg-dev zlib1g-dev libpng-dev \
#               libavcodec-dev libavformat-dev libswscale-dev \
#		        python3-dev python3-pip

echo "🚀 [2/6] 升级 pip/setuptools/wheel..."
# pip3 install --upgrade pip setuptools wheel

echo "🚀 [3/6] 克隆 torchvision 源码..."
if [ ! -d "vision" ]; then
	    git clone https://github.com/pytorch/vision.git
	        cd vision
	else
		    cd vision
		        git fetch
fi

# torchvision 版本需要与 torch 2.6.0 匹配 (>=0.21.0,<0.23.0)
git checkout v0.21.0

echo "🚀 [4/6] 设置编译环境变量..."
export CUDA_HOME=/usr/local/cuda-12.6
export TORCH_CUDA_ARCH_LIST="8.7"
export MAX_JOBS=8

echo "🚀 [5/6] 开始编译 torchvision..."
# 方式一：开发模式 (推荐，方便调试)
# pip install -v -e . --no-deps

# 如果需要 wheel 包，请用：
python3 setup.py bdist_wheel
pip install dist/torchvision-*.whl

echo "🚀 [6/6] 验证 torchvision..."
python3 - <<'EOF'
import torchvision, torch
print("Torchvision version:", torchvision.__version__)
print("PyTorch version:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
from torchvision.ops import nms
import torch as t
print("Test NMS:", nms(t.randn(10,4,device='cuda'), t.rand(10,device='cuda'), 0.5))
EOF

echo "✅ torchvision 编译安装完成！"

