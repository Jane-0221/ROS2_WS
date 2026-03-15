#!/bin/bash

# 切换到工作目录
cd /home/jszn/robot_ros2_ws

# 配置ROS2环境
source /opt/ros/humble/setup.bash

# 给Python脚本添加执行权限（如果还没有）
chmod +x src/robot_hardware/robot_hardware/*.py

echo "启动机器人节点系统..."
echo "========================"

# 启动BMS SOC读取节点（后台运行）
echo "1. 启动BMS SOC读取节点..."
python3 src/robot_hardware/robot_hardware/bms_soc_reader_node.py &
BMS_PID=$!
sleep 2

# 启动STM32串口节点（后台运行）
echo "2. 启动STM32串口节点..."
python3 src/robot_hardware/robot_hardware/stm32_serial_node.py &
STM32_PID=$!
sleep 2

# 启动高度控制器节点（前台运行）
# echo "3. 启动高度控制器节点..."
# echo "========================"
# echo "系统启动完成！"
# echo "电流控制升降功能已启用："
# echo "  电流 > -1.5A → 高度300mm"
# echo "  电流 ≤ -1.5A → 高度600mm"
# echo "========================"

# 前台运行高度控制器节点（可以查看实时日志）
# python3 src/robot_hardware/robot_hardware/bms_height_controller_node.py

# 当高度控制器节点停止时，停止其他节点
# echo "正在停止所有节点..."
# kill $BMS_PID 2>/dev/null
# kill $STM32_PID 2>/dev/null
# echo "所有节点已停止"

echo "========================"
echo "系统启动完成！"
echo "========================"

# 保持脚本运行，防止立即退出
wait $BMS_PID $STM32_PID
