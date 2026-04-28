from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def _build_forwarded_arguments(mode: str):
    mode_args = {
        "mapping": (
            "start_camera",
            "start_slam",
            "start_rtabmap_viz",
            "delete_db_on_start",
            "rtabmap_database_path",
        ),
        "nav": (
            "localization",
            "start_camera",
            "start_slam",
            "start_nav2",
            "start_rtabmap_viz",
            "start_stm32",
            "start_base",
            "delete_db_on_start",
            "base_control_mode",
            "rtabmap_database_path",
        ),
        "nav_pick": (
            "rviz",
            "start_camera",
            "start_slam",
            "start_nav2",
            "start_stm32",
            "start_base",
            "base_control_mode",
        ),
    }

    return [(name, LaunchConfiguration(name)) for name in mode_args[mode]]


def _build_mode_launch(context):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in {"mapping", "nav", "nav_pick"}:
        raise RuntimeError(
            f"Unsupported medipick runtime mode '{mode}'. Expected mapping, nav, or nav_pick."
        )

    runtime_package = "medipick_planning_server"
    impl_file = {
        "mapping": "live_mapping_impl.launch.py",
        "nav": "scene_mapping_navigation_impl.launch.py",
        "nav_pick": "visual_navigation_demo_impl.launch.py",
    }[mode]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(runtime_package),
                        "launch",
                        impl_file,
                    ]
                )
            ),
            launch_arguments=_build_forwarded_arguments(mode),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_db = str(Path.home() / ".ros" / "medipick_scene_mapping.db")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="nav_pick"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_stm32", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("delete_db_on_start", default_value="false"),
            DeclareLaunchArgument("base_control_mode", default_value="nav"),
            DeclareLaunchArgument("rtabmap_database_path", default_value=default_db),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=_build_mode_launch),
        ]
    )
