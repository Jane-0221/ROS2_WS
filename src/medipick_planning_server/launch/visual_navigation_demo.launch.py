from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_stm32", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_apriltag", default_value="true"),
            DeclareLaunchArgument("start_local_pick_estimator", default_value="true"),
            DeclareLaunchArgument("start_local_pick_resolver", default_value="true"),
            DeclareLaunchArgument("start_semantic_registry", default_value="true"),
            DeclareLaunchArgument("start_fetch_coordinator", default_value="true"),
            DeclareLaunchArgument("base_control_mode", default_value="nav"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("medipick_planning_server"),
                            "launch",
                            "medipick_runtime.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "mode": "nav_pick",
                    "rviz": LaunchConfiguration("rviz"),
                    "start_camera": LaunchConfiguration("start_camera"),
                    "start_stm32": LaunchConfiguration("start_stm32"),
                    "start_base": LaunchConfiguration("start_base"),
                    "start_slam": LaunchConfiguration("start_slam"),
                    "start_nav2": LaunchConfiguration("start_nav2"),
                    "base_control_mode": LaunchConfiguration("base_control_mode"),
                }.items(),
            ),
        ]
    )
