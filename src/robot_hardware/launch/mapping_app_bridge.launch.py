from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8000"),
            DeclareLaunchArgument("default_database_path", default_value="~/.ros/medipick_scene_mapping.db"),
            DeclareLaunchArgument("device_name", default_value=""),
            DeclareLaunchArgument("app_max_linear_x", default_value="3.0"),
            DeclareLaunchArgument("app_max_linear_y", default_value="3.0"),
            DeclareLaunchArgument("app_max_angular_z", default_value="3.0"),
            DeclareLaunchArgument("start_bms_reader", default_value="false"),
            Node(
                package="robot_hardware",
                executable="mapping_app_bridge",
                name="medipick_mapping_app_bridge",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                        "default_database_path": LaunchConfiguration("default_database_path"),
                        "device_name": LaunchConfiguration("device_name"),
                        "app_max_linear_x": ParameterValue(
                            LaunchConfiguration("app_max_linear_x"),
                            value_type=float,
                        ),
                        "app_max_linear_y": ParameterValue(
                            LaunchConfiguration("app_max_linear_y"),
                            value_type=float,
                        ),
                        "app_max_angular_z": ParameterValue(
                            LaunchConfiguration("app_max_angular_z"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="robot_hardware",
                executable="bms_soc_reader_node",
                name="bms_soc_reader_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_bms_reader")),
                parameters=[
                    {
                        "port": "/dev/ttyBattery",
                        "battery_state_topic": "battery",
                    }
                ],
            ),
        ]
    )
