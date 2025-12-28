#!/bin/bash

# 退出conda环境
# 初始化conda环境（如果已初始化则跳过）
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
elif [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
    source "/opt/conda/etc/profile.d/conda.sh"
fi

# 退出conda环境（如果当前在conda环境中）
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    conda deactivate
fi


# 设置库路径
# export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

# 检查是否设置了SUDO_PASSWORD
if [ -z "$SUDO_PASSWORD" ]; then
    echo "错误：请先设置SUDO_PASSWORD环境变量"
    echo "例如：export SUDO_PASSWORD='your_password'"
    exit 1
fi

# 函数：使用密码执行sudo命令
sudo_with_password() {
    echo "$SUDO_PASSWORD" | sudo -S "$@"
}

# 修改设备权限
sudo_with_password chmod 666 /dev/video*
sudo_with_password chmod 666 /dev/ttyUSB*


cd /home/yaoyh-4090/IsaacLab/third/linkerta_cpp
# 设置ROS环境
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 run linkerta linkerta calibrate
