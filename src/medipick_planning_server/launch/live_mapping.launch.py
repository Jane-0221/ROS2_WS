from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("start_rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("delete_db_on_start", default_value="true"),
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
                    "mode": "mapping",
                    "start_camera": LaunchConfiguration("start_camera"),
                    "start_slam": LaunchConfiguration("start_slam"),
                    "start_rtabmap_viz": LaunchConfiguration("start_rtabmap_viz"),
                    "delete_db_on_start": LaunchConfiguration("delete_db_on_start"),
                }.items(),
            ),
        ]
    )
