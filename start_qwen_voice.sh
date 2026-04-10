#!/bin/bash
# 千问语音控制系统启动脚本
# 使用方法: ./start_qwen_voice.sh

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================="
echo "  千问语音控制系统"
echo "========================================="

# 保存用户Python包路径
USER_PYTHONPATH="/home/jszn/.local/lib/python3.10/site-packages"

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# 重要：在ROS2环境加载后恢复PYTHONPATH
# 因为ROS2的setup.bash会覆盖PYTHONPATH
export PYTHONPATH="$USER_PYTHONPATH:$PYTHONPATH"

# 设置API Key环境变量
export DASHSCOPE_API_KEY="sk-dd0137dc3fec464d86de6c57406ed624"

# 启动各个节点 - 直接使用python3运行
echo "启动千问ASR语音识别节点..."
python3 -m robot_hardware.voice_control.qwen_asr_node &

echo "启动意图理解节点..."
python3 -m robot_hardware.voice_control.intent_understanding_node &

echo "启动TTS语音合成节点..."
python3 -m robot_hardware.voice_control.qwen_tts_player_node &

echo "启动指令分发节点..."
python3 -m robot_hardware.voice_control.command_dispatcher_node &

echo ""
echo "所有节点已启动！"
echo "对着麦克风说话测试，例如：'打开气泵'"
echo "按Ctrl+C停止所有节点"
echo "========================================="

# 等待
wait
