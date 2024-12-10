#!/bin/bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate duper
PYTHON_PATH=$(which python3)
echo "Current Python Path: $PYTHON_PATH"
if [[ -z "$CONDA_PREFIX" ]]; then
    echo "Not in a Conda environment."
else
    echo "In Conda environment (should match the python path): $CONDA_PREFIX"
fi
SITE_PACKAGES_DIR=$(python3 -c "import site; print(site.getsitepackages()[0])")
echo "Site-Package Path: $SITE_PACKAGES_DIR"

echo "======================================================================="
echo -e "Please make sure:\n1. This script is located inside the project root folder (duper-agent/) and a Python virtual environment.\n2. Site-Package is the correct path with respect to the Python venv."
echo "======================================================================="
read -p "Proceed? (y/n): " IN_VENV
if [[ "$IN_VENV" != "y" && "$IN_VENV" != "Y" ]]; then
    exit 1
fi
set -e
echo "Installing required system tools and libraries..."
sudo apt-get update
sudo apt-get install -y libglew-dev libgl1-mesa-dev libosmesa6-dev mesa-utils cmake build-essential zlib1g-dev ffmpeg

echo "Upgrading pip, setuptools, and wheel..."
python3 -m pip install --upgrade pip setuptools wheel

python3 -m pip uninstall -y gym

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

# 替换 np.bool -> np.bool_ 和其他类型
echo "Performing replacements in dm_control..."

DM_CONTROL_DIR="$SITE_PACKAGES_DIR/dm_control"

if [ -d "$DM_CONTROL_DIR" ]; then
    echo "Found dm_control in $DM_CONTROL_DIR. Starting replacements..."
    find "$DM_CONTROL_DIR" -type f -name "*.py" -exec sed -i \
        -e 's/\bnp.bool\b/np.bool_/g' \
        -e 's/\bnp.float\b/np.float64/g' \
        -e 's/\bnp.float6464\b/np.float64/g' \
        -e 's/\bnp.float6432\b/np.float32/g' {} +

    echo "Replacements completed in dm_control."
else
    echo "dm_control directory not found in $SITE_PACKAGES_DIR. Skipping replacements."
fi


read -p "Install MuJoCo (both 200&210) and mujoco-py automatically? The .bashrc will be modified. Make sure to check it after installation (don't proceed if you have already installed) (y/n): " INSTALL_MUJOCO
if [[ "$INSTALL_MUJOCO" == "y" || "$INSTALL_MUJOCO" == "Y" ]]; then
    echo "Installing MuJoCo and mujoco-py..."
    MUJOCO_DIR="$HOME/.mujoco"
    mkdir -p "$MUJOCO_DIR"

    # 下载并安装 MuJoCo 210
    if [ ! -f mujoco210-linux-x86_64.tar.gz ]; then
        wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz -O mujoco210-linux-x86_64.tar.gz
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
