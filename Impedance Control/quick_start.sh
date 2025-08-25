#!/bin/bash

# 快速启动脚本 - 适用于重启后的日常使用

echo "=========================================="
echo "FR3 Demo 快速启动"
echo "=========================================="

# 1. 加载工作空间环境
echo "加载工作空间环境..."
source install/setup.bash

# 2. 检查可执行文件
echo "检查可执行文件..."
if [ -f "install/mujoco_ros2_control_demos/lib/mujoco_ros2_control_demos/fr3_impedance_controller" ]; then
    echo "✓ fr3_impedance_controller 存在"
else
    echo "✗ fr3_impedance_controller 不存在，请先编译"
    exit 1
fi

if [ -f "install/mujoco_ros2_control/lib/mujoco_ros2_control/mujoco_ros2_control" ]; then
    echo "✓ mujoco_ros2_control 存在"
else
    echo "✗ mujoco_ros2_control 不存在，请先编译"
    exit 1
fi

# 3. 启动演示
echo "=========================================="
echo "启动FR3 Demo..."
echo "按 Ctrl+C 停止程序"
echo "=========================================="

ros2 launch mujoco_ros2_control_demos fr3_demo.launch.py
