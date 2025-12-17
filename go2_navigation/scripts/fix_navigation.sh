#!/bin/bash

# 修复导航系统的脚本
# 用于Go2机器狗导航启动后的手动修复

echo "正在修复导航系统..."

# 1. 杀死现有的map_server
echo "步骤1: 停止现有map_server..."
pkill -f map_server
sleep 1

# 2. 启动新的map_server
echo "步骤2: 启动map_server..."
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/unitree/go2_ws1/map/map.yaml &
MAP_SERVER_PID=$!
sleep 2

# 3. 配置map_server lifecycle
echo "步骤3: 配置map_server lifecycle..."
ros2 lifecycle set /map_server configure
sleep 1
ros2 lifecycle set /map_server activate
sleep 1

# 4. 激活AMCL
echo "步骤4: 激活AMCL..."
ros2 lifecycle set /amcl activate
sleep 1

# 5. 设置初始位置
echo "步骤5: 设置初始位置..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

echo "导航系统修复完成！"
echo "现在可以发送导航目标了"
