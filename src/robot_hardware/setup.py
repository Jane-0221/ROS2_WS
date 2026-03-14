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
            'funasr_speech_recognition_node = robot_hardware.funasr_speech_recognition_node:main',
            'speech_subscriber_node = robot_hardware.speech_subscriber_node:main',
        ],
    },

)