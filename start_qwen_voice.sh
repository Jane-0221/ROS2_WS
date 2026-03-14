#!/bin/bash
# 千问语音控制系统启动脚本
# 使用方法: ./start_qwen_voice.sh

echo "========================================="
echo "  千问语音控制系统"
echo "========================================="

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source ~/robot_ros2_ws/install/setup.bash

# 设置API Key环境变量
export DASHSCOPE_API_KEY="sk-2cab9b4a77914400b0f504817b8fc0ae"

# 启动各个节点
echo "启动千问ASR语音识别节点..."
ros2 run robot_hardware qwen_asr_node &

echo "启动意图理解节点..."
ros2 run robot_hardware intent_understanding_node &

echo "启动TTS语音合成节点..."
ros2 run robot_hardware qwen_tts_player_node &

echo "启动指令分发节点..."
ros2 run robot_hardware command_dispatcher_node &

echo ""
echo "所有节点已启动！"
echo "对着麦克风说话测试，例如：'打开气泵'"
echo "按Ctrl+C停止所有节点"
echo "========================================="

# 等待
wait
