#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable


def generate_launch_description():
    return LaunchDescription([
        # 语音识别节点
        Node(
            package='robot_hardware',
            executable='simple_speech_recognition_node',
            name='speech_recognition_node',
            output='screen',
            parameters=[{
                'sample_rate': 16000,
                'chunk_size': 4000,
                'model_path': '/usr/local/share/vosk/models/vosk-model-small-cn-0.22'
            }]
        ),
        
        # 可选：显示音频设备信息
        ExecuteProcess(
            cmd=[[FindExecutable(name='python3')], '-c', 'import sounddevice as sd; print("可用音频设备:"); print(sd.query_devices())'],
            shell=True
        )
    ])