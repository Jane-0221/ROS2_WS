import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joy_config = LaunchConfiguration("joy_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("joy_config", default_value="ps3-holonomic"),
            DeclareLaunchArgument("joy_dev", default_value="0"),
            DeclareLaunchArgument("deadzone", default_value="0.3"),
            DeclareLaunchArgument("autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("publish_stamped_twist", default_value="false"),
            DeclareLaunchArgument(
                "config_filepath",
                default_value=[
                    TextSubstitution(
                        text=os.path.join(get_package_share_directory("teleop_twist_joy"), "config", "")
                    ),
                    joy_config,
                    TextSubstitution(text=".config.yaml"),
                ],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                parameters=[
                    {
                        "device_id": ParameterValue(LaunchConfiguration("joy_dev"), value_type=int),
                        "deadzone": ParameterValue(LaunchConfiguration("deadzone"), value_type=float),
                        "autorepeat_rate": ParameterValue(LaunchConfiguration("autorepeat_rate"), value_type=float),
                    },
                    LaunchConfiguration("config_filepath"),
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[
                    LaunchConfiguration("config_filepath"),
                    {
                        "publish_stamped_twist": ParameterValue(
                            LaunchConfiguration("publish_stamped_twist"),
                            value_type=bool,
                        )
                    },
                ],
                remappings=[
                    ("/cmd_vel", LaunchConfiguration("cmd_vel_topic")),
                ],
            ),
        ]
    )
