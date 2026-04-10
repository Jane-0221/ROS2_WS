from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_file(package_name: str, relative_path: str) -> str:
    package_path = Path(get_package_share_directory(package_name))
    return (package_path / relative_path).read_text(encoding="utf-8")


def load_yaml(package_name: str, relative_path: str) -> dict:
    package_path = Path(get_package_share_directory(package_name))
    with open(package_path / relative_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    description_package = "medipick_simple3_description"
    moveit_package = "medipick_moveit_config"
    planning_server_package = "medipick_planning_server"
    hardware_package = "robot_hardware"

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
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(load_yaml(moveit_package, "config/ompl_planning.yaml"))
    moveit_controllers = load_yaml(moveit_package, "config/moveit_controllers.yaml")
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    calibration_file = str(
        Path(get_package_share_directory(hardware_package)) / "config" / "medipick_hardware.yaml"
    )
    planning_server_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "planning_server.yaml"
    )
    task_manager_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "hardware_task_manager.yaml"
    )
    rviz_config_file = str(Path(get_package_share_directory(moveit_package)) / "rviz" / "moveit.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("start_stm32", default_value="true"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttySTM32"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("calibration_file", default_value=calibration_file),
            DeclareLaunchArgument(
                "motor_arm_joint_command_topic",
                default_value="/medipick/hardware/motor_arm_joint_command",
            ),
            DeclareLaunchArgument(
                "enable_legacy_servo_control_topic",
                default_value="false",
            ),
            DeclareLaunchArgument("publish_test_target", default_value="false"),
            DeclareLaunchArgument("target_frame", default_value="world"),
            DeclareLaunchArgument("target_x", default_value="0.72"),
            DeclareLaunchArgument("target_y", default_value="-0.22"),
            DeclareLaunchArgument("target_z", default_value="1.05"),
            DeclareLaunchArgument("target_qx", default_value="0.0"),
            DeclareLaunchArgument("target_qy", default_value="0.70710678"),
            DeclareLaunchArgument("target_qz", default_value="0.0"),
            DeclareLaunchArgument("target_qw", default_value="0.70710678"),
            Node(
                package=hardware_package,
                executable="stm32_serial_node",
                name="stm32_serial_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_stm32")),
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "motor_arm_joint_command_topic": LaunchConfiguration("motor_arm_joint_command_topic"),
                        "legacy_arm_joint_command_topic": "",
                        "enable_legacy_servo_control_topic": ParameterValue(
                            LaunchConfiguration("enable_legacy_servo_control_topic"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package=hardware_package,
                executable="motor_arm_trajectory_controller_bridge",
                name="medipick_motor_arm_trajectory_controller_bridge",
                output="screen",
                parameters=[
                    {
                        "motor_arm_joint_command_topic": LaunchConfiguration("motor_arm_joint_command_topic"),
                    }
                ],
            ),
            Node(
                package=hardware_package,
                executable="lift_command_adapter",
                name="medipick_lift_command_adapter",
                output="screen",
                parameters=[{"calibration_file": LaunchConfiguration("calibration_file")}],
            ),
            Node(
                package=hardware_package,
                executable="pump_command_adapter",
                name="medipick_pump_command_adapter",
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor_parameters,
                ],
            ),
            Node(
                package=planning_server_package,
                executable="planning_server.py",
                name="medipick_planning_server",
                output="screen",
                parameters=[
                    planning_server_config,
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    {
                        "planning_pipelines": ["ompl"],
                        "default_planning_pipeline": "ompl",
                        "ompl": load_yaml(moveit_package, "config/ompl_planning.yaml"),
                    },
                ],
            ),
            Node(
                package=planning_server_package,
                executable="pick_task_manager.py",
                name="medipick_pick_task_manager",
                output="screen",
                parameters=[task_manager_config],
            ),
            Node(
                package=hardware_package,
                executable="manual_target_pose_publisher",
                name="medipick_manual_target_pose_publisher",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_test_target")),
                parameters=[
                    {
                        "frame_id": LaunchConfiguration("target_frame"),
                        "x": ParameterValue(LaunchConfiguration("target_x"), value_type=float),
                        "y": ParameterValue(LaunchConfiguration("target_y"), value_type=float),
                        "z": ParameterValue(LaunchConfiguration("target_z"), value_type=float),
                        "qx": ParameterValue(LaunchConfiguration("target_qx"), value_type=float),
                        "qy": ParameterValue(LaunchConfiguration("target_qy"), value_type=float),
                        "qz": ParameterValue(LaunchConfiguration("target_qz"), value_type=float),
                        "qw": ParameterValue(LaunchConfiguration("target_qw"), value_type=float),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
