#!/usr/bin/env python3
"""
千问语音控制系统启动文件

启动以下节点:
1. 千问ASR语音识别节点
2. 意图理解节点（LLM）
3. TTS语音合成播放节点
4. 指令分发节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述"""
    
    # 千问ASR语音识别节点
    qwen_asr_node = Node(
        package='robot_hardware',
        executable='qwen_asr_node',
        name='qwen_asr_node',
        output='screen',
        parameters=[{
            'api_key': 'sk-2cab9b4a77914400b0f504817b8fc0ae',
            'sample_rate': 16000,
            'device_index': 0,
        }]
    )
    
    # 意图理解节点
    intent_node = Node(
        package='robot_hardware',
        executable='intent_understanding_node',
        name='intent_understanding_node',
        output='screen',
        parameters=[{
            'api_key': 'sk-2cab9b4a77914400b0f504817b8fc0ae',
            'llm_model': 'qwen-plus',
        }]
    )
    
    # TTS播放节点
    tts_node = Node(
        package='robot_hardware',
        executable='qwen_tts_player_node',
        name='qwen_tts_player_node',
        output='screen',
        parameters=[{
            'api_key': 'sk-2cab9b4a77914400b0f504817b8fc0ae',
            'voice': 'Cherry',
        }]
    )
    
    # 指令分发节点
    dispatcher_node = Node(
        package='robot_hardware',
        executable='command_dispatcher_node',
        name='command_dispatcher_node',
        output='screen',
    )
    
    return LaunchDescription([
        qwen_asr_node,
        intent_node,
        tts_node,
        dispatcher_node,
    ])
