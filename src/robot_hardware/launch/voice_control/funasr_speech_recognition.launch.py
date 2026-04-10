#!/usr/bin/env python3
"""
FunASR语音识别系统启动文件

同时启动语音识别节点和订阅节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # FunASR语音识别节点
    funasr_node = Node(
        package='robot_hardware',
        executable='funasr_speech_recognition_node',
        name='funasr_speech_recognition_node',
        output='screen',
        parameters=[{
            'sample_rate': 16000,
            'record_duration': 3,
            'device_index': 0
        }]
    )
    
    # 语音识别结果订阅节点
    subscriber_node = Node(
        package='robot_hardware',
        executable='speech_subscriber_node',
        name='speech_subscriber_node',
        output='screen'
    )
    
    return LaunchDescription([
        funasr_node,
        subscriber_node
    ])