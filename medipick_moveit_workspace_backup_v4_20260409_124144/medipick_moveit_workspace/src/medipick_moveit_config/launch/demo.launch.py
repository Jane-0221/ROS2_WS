from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError


def load_file(package_name: str, relative_path: str) -> str:
    package_path = Path(get_package_share_directory(package_name))
    return (package_path / relative_path).read_text(encoding="utf-8")


def load_yaml(package_name: str, relative_path: str) -> dict:
    package_path = Path(get_package_share_directory(package_name))
    with open(package_path / relative_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def package_exists(package_name: str) -> bool:
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    description_package = "medipick_simple3_description"
    moveit_package = "medipick_moveit_config"
    ros2_control_available = all(
        package_exists(package_name)
        for package_name in (
            "controller_manager",
            "joint_trajectory_controller",
            "joint_state_broadcaster",
        )
    )
    use_ros2_control = LaunchConfiguration("use_ros2_control")

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
    sensors_3d = load_yaml(moveit_package, "config/sensors_3d.yaml")
    trajectory_execution = {
        "allow_trajectory_execution": ParameterValue(use_ros2_control, value_type=bool),
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    use_pointcloud_octomap = LaunchConfiguration("use_pointcloud_octomap")
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    octomap_frame = LaunchConfiguration("octomap_frame")
    octomap_resolution = LaunchConfiguration("octomap_resolution")
    octomap_max_range = LaunchConfiguration("octomap_max_range")
    sensor_name = sensors_3d["sensors"][0]
    sensor_config = sensors_3d[sensor_name]
    octomap_sensor_parameters = {
        "sensors": sensors_3d["sensors"],
        f"{sensor_name}.sensor_plugin": sensor_config["sensor_plugin"],
        f"{sensor_name}.point_cloud_topic": point_cloud_topic,
        f"{sensor_name}.max_range": ParameterValue(octomap_max_range, value_type=float),
        f"{sensor_name}.point_subsample": sensor_config["point_subsample"],
        f"{sensor_name}.padding_offset": sensor_config["padding_offset"],
        f"{sensor_name}.padding_scale": sensor_config["padding_scale"],
        f"{sensor_name}.max_update_rate": sensor_config["max_update_rate"],
        f"{sensor_name}.filtered_cloud_topic": sensor_config["filtered_cloud_topic"],
    }
    octomap_parameters = {
        "octomap_frame": octomap_frame,
        "octomap_resolution": ParameterValue(octomap_resolution, value_type=float),
        "max_range": ParameterValue(octomap_max_range, value_type=float),
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    rviz_config_file = str(Path(get_package_share_directory(moveit_package)) / "rviz" / "moveit.rviz")
    ros2_controllers_file = str(
        Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_pointcloud_octomap", default_value="false"),
            DeclareLaunchArgument("point_cloud_topic", default_value="/medipick/mock_pointcloud"),
            DeclareLaunchArgument("octomap_frame", default_value="world"),
            DeclareLaunchArgument("octomap_resolution", default_value="0.03"),
            DeclareLaunchArgument("octomap_max_range", default_value="3.0"),
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true" if ros2_control_available else "false",
            ),
            LogInfo(
                msg=(
                    "ros2_control packages not found; MoveIt execution stays disabled by default. "
                    "Install controller_manager, joint_state_broadcaster, and joint_trajectory_controller "
                    "to enable Plan&Execute."
                ),
                condition=UnlessCondition(use_ros2_control),
            ),
            Node(
                package="medipick_moveit_config",
                executable="default_joint_state_publisher.py",
                name="default_joint_state_publisher",
                parameters=[{"publish_rate": 20.0}],
                condition=UnlessCondition(use_ros2_control),
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
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, ros2_controllers_file],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "mobile_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    ros2_controllers_file,
                ],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "tool_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    ros2_controllers_file,
                ],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "head_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    ros2_controllers_file,
                ],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
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
                condition=UnlessCondition(use_pointcloud_octomap),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    octomap_parameters,
                    octomap_sensor_parameters,
                    planning_scene_monitor_parameters,
                ],
                condition=IfCondition(use_pointcloud_octomap),
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
