#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # BMS SOC读取节点
        Node(
            package='robot_hardware',
            executable='bms_soc_reader_node',
            name='bms_soc_reader',
            output='screen',
            parameters=[]
        ),
        
        # STM32串口通讯节点
        Node(
            package='robot_hardware',
            executable='stm32_serial_node',
            name='stm32_serial',
            output='screen',
            parameters=[]
        ),
        
        # BMS智能高度控制器节点
        Node(
            package='robot_hardware',
            executable='bms_height_controller_node',
            name='bms_height_controller',
            output='screen',
            parameters=[]
        )
    ])