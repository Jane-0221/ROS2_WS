from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_package = "medipick_simple3_description"
    moveit_package = "medipick_moveit_config"
    planning_server_package = "medipick_planning_server"

    urdf_path = Path(get_package_share_directory(description_package)) / "urdf" / "simple3_moveit.urdf"
    rviz_path = Path(get_package_share_directory(planning_server_package)) / "rviz" / "mock_perception.rviz"
    robot_description = urdf_path.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            Node(
                package=moveit_package,
                executable="default_joint_state_publisher.py",
                name="default_joint_state_publisher",
                parameters=[{"publish_rate": 20.0}],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package=planning_server_package,
                executable="mock_vision_publisher.py",
                name="medipick_mock_vision_publisher",
                output="screen",
            ),
            Node(
                package=planning_server_package,
                executable="mock_pick_path_publisher.py",
                name="medipick_mock_pick_path_publisher",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_path)],
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
        ]
    )
