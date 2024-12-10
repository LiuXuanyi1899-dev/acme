#!/bin/bash

# 设置脚本在发生错误时立即退出
set -e

echo "======================================================================="
echo "Please make sure this script is located inside the project root folder and a Python virtual environment."
echo "======================================================================="
read -p "Proceed? (y/n): " IN_VENV
if [[ "$IN_VENV" != "y" && "$IN_VENV" != "Y" ]]; then
    exit 1
fi

echo "Installing required system tools and libraries..."
sudo apt-get update
sudo apt-get install -y libglew-dev libgl1-mesa-dev libosmesa6-dev mesa-utils cmake build-essential zlib1g-dev ffmpeg

echo "Upgrading pip, setuptools, and wheel..."
python3 -m pip install --upgrade pip setuptools wheel

echo "Installing acme wheel"
python3 -m pip install "https://github.com/LiuXuanyi1899-dev/acme/releases/download/publish/dm_acme-0.4.1-py3-none-any.whl"

echo "Installing dm-acme[envs] and related dependencies..."
#安装修复后的依赖版本
echo "Installing jax==0.4.25..."
python3 -m pip install jax==0.4.25
JAXLIB_URL="https://storage.googleapis.com/jax-releases/cuda11/jaxlib-0.4.25+cuda11.cudnn86-cp39-cp39-manylinux2014_x86_64.whl"
echo "Installing jaxlib from $JAXLIB_URL..."
python3 -m pip install "$JAXLIB_URL"

python3 -m pip install "numpy<2.0"  # 降级 numpy
python3 -m pip install dm-launchpad==0.5.2  # 升级 dm-launchpad
python3 -m pip install cython==0.29.36  # 降级 cython
python3 -m pip install dm-reverb==0.7.2  # 升级 dm-reverb

# 降级 pip、setuptools 和 wheel
echo "Downgrading pip, setuptools, and wheel..."
python3 -m pip install pip==21.3.1 setuptools==59.6.0 wheel==0.37.0
# 安装 gym==0.19.0
echo "Installing gym==0.19.0..."
python3 -m pip install gym==0.19.0

read -p "Install MuJoCo and mujoco-py automatically? the .bashrc will be modified, make sure to checkout after the installation (don't proceed if you have already installed) (y/n): " INSTALL_MUJOCO
if [[ "$INSTALL_MUJOCO" == "y" || "$INSTALL_MUJOCO" == "Y" ]]; then
    echo "Installing MuJoCo and mujoco-py..."
    MUJOCO_DIR=~/.mujoco
    mkdir -p $MUJOCO_DIR

    # 下载并安装 MuJoCo 210
    if [ ! -f mujoco210-linux-x86_64.tar.gz ]; then
        wget https://github.com/deepmind/mujoco/releases/download/2.1/mujoco210-linux-x86_64.tar.gz -O mujoco210-linux-x86_64.tar.gz
    fi
    tar -xzvf mujoco210-linux-x86_64.tar.gz -C $MUJOCO_DIR

    # 设置环境变量
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco210/bin' >> ~/.bashrc
    source ~/.bashrc

    # 下载并安装 MuJoCo 200
    if [ ! -f mujoco200.zip ]; then
        wget https://roboti.us/download/mujoco200_linux.zip -O $MUJOCO_DIR/mujoco200.zip
    fi
    unzip -o $MUJOCO_DIR/mujoco200.zip -d $MUJOCO_DIR

    # 下载 MuJoCo license
    if [ ! -f $MUJOCO_DIR/mjkey.txt ]; then
        wget https://roboti.us/file/mjkey.txt -P $MUJOCO_DIR
    fi

    # 安装 mujoco-py
    python3 -m pip install -U 'mujoco-py<2.2,>=2.1'
else
    echo "Skipping MuJoCo and mujoco-py installation."
fi

echo "Installation completed successfully!"
