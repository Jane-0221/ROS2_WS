from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/launch/voice_control', glob('launch/voice_control/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jszn',
    maintainer_email='jszn@todo.todo',
    description='ROS2 hardware interface package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bms_soc_reader_node = robot_hardware.bms_soc_reader_node:main',
            'stm32_serial_node = robot_hardware.stm32_serial_node:main',
            'bms_height_controller_node = robot_hardware.bms_height_controller_node:main',
            'wheeltec_chassis_node = robot_hardware.wheeltec_chassis_node:main',
            'funasr_speech_recognition_node = robot_hardware.voice_control.funasr_speech_recognition_node:main',
            'simple_speech_recognition_node = robot_hardware.voice_control.simple_speech_recognition_node:main',
            'speech_recognition_node = robot_hardware.voice_control.speech_recognition_node:main',
            'speech_control_node = robot_hardware.voice_control.speech_control_node:main',
            'speech_subscriber_node = robot_hardware.voice_control.speech_subscriber_node:main',
            'qwen_asr_node = robot_hardware.voice_control.qwen_asr_node:main',
            'intent_understanding_node = robot_hardware.voice_control.intent_understanding_node:main',
            'qwen_tts_player_node = robot_hardware.voice_control.qwen_tts_player_node:main',
            'command_dispatcher_node = robot_hardware.voice_control.command_dispatcher_node:main',
            'arm_trajectory_controller_bridge = robot_hardware.arm_trajectory_controller_bridge:main',
            'motor_arm_trajectory_controller_bridge = robot_hardware.arm_trajectory_controller_bridge:main',
            'lift_command_adapter = robot_hardware.lift_command_adapter:main',
            'pump_command_adapter = robot_hardware.pump_command_adapter:main',
            'base_cmd_mux = robot_hardware.base_cmd_mux:main',
            'mapping_app_bridge = robot_hardware.mapping_app_bridge:main',
            'camera_target_pose_relay = robot_hardware.camera_target_pose_relay:main',
            'camera_pointcloud_target_estimator = robot_hardware.camera_pointcloud_target_estimator:main',
            'local_pick_pose_resolver = robot_hardware.local_pick_pose_resolver:main',
            'manual_target_pose_publisher = robot_hardware.manual_target_pose_publisher:main',
            'base_pose_joint_state_bridge = robot_hardware.base_pose_joint_state_bridge:main',
            'joint_state_mux = robot_hardware.joint_state_mux:main',
            'apriltag_anchor_localizer = robot_hardware.apriltag_anchor_localizer:main',
        ],
    },
)
