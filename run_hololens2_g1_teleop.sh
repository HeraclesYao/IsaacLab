#!/bin/bash

# HoloLens2控制G1机器人遥操作启动脚本

echo "启动HoloLens2 G1遥操作..."

# 设置HoloLens2 IP地址（根据实际情况修改）
HOLOLENS_IP="192.168.11.121"

# 运行遥操作脚本
./isaaclab.sh -p scripts/environments/teleoperation/hololens2_g1_teleop.py \
    --task Isaac-PickPlace-G1-InspireFTP-Abs-v0 \
    --num_envs 1 \
    --hololens_host $HOLOLENS_IP \
    --enable_visualization

echo "遥操作已结束"