from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name: str, relative_path: str) -> str:
    package_path = Path(get_package_share_directory(package_name))
    return (package_path / relative_path).read_text(encoding="utf-8")


def load_yaml(package_name: str, relative_path: str) -> dict:
    package_path = Path(get_package_share_directory(package_name))
    with open(package_path / relative_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    moveit_package = "medipick_moveit_config"
    description_package = "medipick_simple3_description"
    planning_server_package = "medipick_planning_server"

    moveit_launch_file = Path(get_package_share_directory(moveit_package)) / "launch" / "demo.launch.py"
    planning_server_config = Path(get_package_share_directory(planning_server_package)) / "config" / "planning_server.yaml"

    robot_description = {
        "robot_description": load_file(description_package, "urdf/simple3_moveit.urdf"),
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(moveit_package, "config/medipick.srdf"),
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(moveit_package, "config/kinematics.yaml"),
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(moveit_package, "config/joint_limits.yaml"),
    }
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load_yaml(moveit_package, "config/ompl_planning.yaml"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="auto"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("with_moveit_demo", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(moveit_launch_file)),
                launch_arguments={"rviz": LaunchConfiguration("rviz")}.items(),
                condition=IfCondition(LaunchConfiguration("with_moveit_demo")),
            ),
            Node(
                package=planning_server_package,
                executable="planning_server.py",
                name="medipick_planning_server",
                output="screen",
                parameters=[
                    str(planning_server_config),
                    {"backend": LaunchConfiguration("backend")},
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                ],
            ),
        ]
    )
