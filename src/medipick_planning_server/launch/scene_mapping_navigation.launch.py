from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    default_db = str(Path.home() / ".ros" / "medipick_scene_mapping.db")

    return LaunchDescription(
        [
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_stm32", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="false"),
            DeclareLaunchArgument("start_rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("delete_db_on_start", default_value="false"),
            DeclareLaunchArgument("base_control_mode", default_value="nav"),
            DeclareLaunchArgument("rtabmap_database_path", default_value=default_db),
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
                    "mode": "nav",
                    "localization": LaunchConfiguration("localization"),
                    "start_camera": LaunchConfiguration("start_camera"),
                    "start_stm32": LaunchConfiguration("start_stm32"),
                    "start_base": LaunchConfiguration("start_base"),
                    "start_slam": LaunchConfiguration("start_slam"),
                    "start_nav2": LaunchConfiguration("start_nav2"),
                    "start_rtabmap_viz": LaunchConfiguration("start_rtabmap_viz"),
                    "delete_db_on_start": LaunchConfiguration("delete_db_on_start"),
                    "base_control_mode": LaunchConfiguration("base_control_mode"),
                    "rtabmap_database_path": LaunchConfiguration("rtabmap_database_path"),
                }.items(),
            ),
        ]
    )
