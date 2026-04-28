from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
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
    sensors_3d = load_yaml(moveit_package, "config/sensors_3d.yaml")
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
        Path(get_package_share_directory(planning_server_package)) / "config" / "planning_server_hardware.yaml"
    )
    task_manager_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "hardware_task_manager.yaml"
    )
    rviz_config_file = str(Path(get_package_share_directory(moveit_package)) / "rviz" / "moveit.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("start_stm32", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttySTM32"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("calibration_file", default_value=calibration_file),
            DeclareLaunchArgument("camera_launch_file", default_value="gemini_330_series.launch.py"),
            DeclareLaunchArgument("camera_model", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_depth_registration", default_value="false"),
            DeclareLaunchArgument("camera_serial_number", default_value=""),
            DeclareLaunchArgument("camera_usb_port", default_value=""),
            DeclareLaunchArgument("camera_device_num", default_value="1"),
            DeclareLaunchArgument("camera_config_file_path", default_value=""),
            DeclareLaunchArgument("camera_enable_color", default_value="true"),
            DeclareLaunchArgument("camera_enable_depth", default_value="true"),
            DeclareLaunchArgument("camera_enable_ir", default_value="false"),
            DeclareLaunchArgument("camera_enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("camera_enable_colored_point_cloud", default_value="false"),
            DeclareLaunchArgument("camera_time_domain", default_value="device"),
            DeclareLaunchArgument("camera_publish_tf", default_value="true"),
            DeclareLaunchArgument("camera_tf_publish_rate", default_value="0.0"),
            DeclareLaunchArgument("camera_log_level", default_value="info"),
            DeclareLaunchArgument("start_camera_world_tf", default_value="true"),
            DeclareLaunchArgument("camera_world_parent_frame", default_value="world"),
            DeclareLaunchArgument("camera_world_child_frame", default_value="camera_link"),
            DeclareLaunchArgument("camera_world_x", default_value="0.44"),
            DeclareLaunchArgument("camera_world_y", default_value="-0.20"),
            DeclareLaunchArgument("camera_world_z", default_value="0.95"),
            DeclareLaunchArgument("camera_world_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_world_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_world_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_pointcloud_octomap", default_value="true"),
            DeclareLaunchArgument("point_cloud_topic", default_value="/camera/depth/points"),
            DeclareLaunchArgument("octomap_frame", default_value="world"),
            DeclareLaunchArgument("octomap_resolution", default_value="0.03"),
            DeclareLaunchArgument("octomap_max_range", default_value="3.0"),
            DeclareLaunchArgument("start_camera_target_estimator", default_value="true"),
            DeclareLaunchArgument("camera_target_estimator_input_topic", default_value="/camera/depth/points"),
            DeclareLaunchArgument("start_target_pose_relay", default_value="true"),
            DeclareLaunchArgument("camera_target_pose_topic", default_value="/camera/target_pose"),
            DeclareLaunchArgument("target_pose_output_frame", default_value="world"),
            DeclareLaunchArgument("target_pose_replace_stamp_with_now", default_value="false"),
            DeclareLaunchArgument("target_pose_passthrough_if_tf_unavailable", default_value="false"),
            DeclareLaunchArgument("target_estimator_publish_rate", default_value="1.0"),
            DeclareLaunchArgument("target_estimator_sample_step", default_value="2"),
            DeclareLaunchArgument("target_roi_min_x", default_value="-0.12"),
            DeclareLaunchArgument("target_roi_max_x", default_value="0.18"),
            DeclareLaunchArgument("target_roi_min_y", default_value="-0.20"),
            DeclareLaunchArgument("target_roi_max_y", default_value="0.05"),
            DeclareLaunchArgument("target_roi_min_z", default_value="0.18"),
            DeclareLaunchArgument("target_roi_max_z", default_value="0.60"),
            DeclareLaunchArgument("target_estimator_voxel_size", default_value="0.04"),
            DeclareLaunchArgument("target_estimator_cluster_radius", default_value="0.06"),
            DeclareLaunchArgument("target_estimator_min_cluster_points", default_value="40"),
            DeclareLaunchArgument("target_estimator_surface_front_ratio", default_value="0.25"),
            DeclareLaunchArgument("target_estimator_depth_weight", default_value="0.35"),
            DeclareLaunchArgument("target_estimator_offset_x", default_value="0.0"),
            DeclareLaunchArgument("target_estimator_offset_y", default_value="0.0"),
            DeclareLaunchArgument("target_estimator_offset_z", default_value="0.07"),
            DeclareLaunchArgument("target_estimator_qx", default_value="0.0"),
            DeclareLaunchArgument("target_estimator_qy", default_value="0.0"),
            DeclareLaunchArgument("target_estimator_qz", default_value="0.70710678"),
            DeclareLaunchArgument("target_estimator_qw", default_value="0.70710678"),
            DeclareLaunchArgument("target_estimator_publish_debug_cloud", default_value="false"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("orbbec_camera"),
                            "launch",
                            LaunchConfiguration("camera_launch_file"),
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_camera")),
                launch_arguments={
                    "device_type": LaunchConfiguration("camera_model"),
                    "camera_model": LaunchConfiguration("camera_model"),
                    "camera_name": LaunchConfiguration("camera_name"),
                    "depth_registration": LaunchConfiguration("camera_depth_registration"),
                    "serial_number": LaunchConfiguration("camera_serial_number"),
                    "usb_port": LaunchConfiguration("camera_usb_port"),
                    "device_num": LaunchConfiguration("camera_device_num"),
                    "config_file_path": LaunchConfiguration("camera_config_file_path"),
                    "enable_color": LaunchConfiguration("camera_enable_color"),
                    "enable_depth": LaunchConfiguration("camera_enable_depth"),
                    "enable_ir": LaunchConfiguration("camera_enable_ir"),
                    "enable_left_ir": LaunchConfiguration("camera_enable_ir"),
                    "enable_right_ir": LaunchConfiguration("camera_enable_ir"),
                    "enable_point_cloud": LaunchConfiguration("camera_enable_point_cloud"),
                    "enable_colored_point_cloud": LaunchConfiguration("camera_enable_colored_point_cloud"),
                    "time_domain": LaunchConfiguration("camera_time_domain"),
                    "publish_tf": LaunchConfiguration("camera_publish_tf"),
                    "tf_publish_rate": LaunchConfiguration("camera_tf_publish_rate"),
                    "log_level": LaunchConfiguration("camera_log_level"),
                }.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="medipick_camera_world_tf",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_camera_world_tf")),
                arguments=[
                    "--x",
                    LaunchConfiguration("camera_world_x"),
                    "--y",
                    LaunchConfiguration("camera_world_y"),
                    "--z",
                    LaunchConfiguration("camera_world_z"),
                    "--roll",
                    LaunchConfiguration("camera_world_roll"),
                    "--pitch",
                    LaunchConfiguration("camera_world_pitch"),
                    "--yaw",
                    LaunchConfiguration("camera_world_yaw"),
                    "--frame-id",
                    LaunchConfiguration("camera_world_parent_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("camera_world_child_frame"),
                ],
            ),
            Node(
                package=hardware_package,
                executable="camera_pointcloud_target_estimator",
                name="medipick_camera_pointcloud_target_estimator",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_camera_target_estimator")),
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("camera_target_estimator_input_topic"),
                        "output_topic": LaunchConfiguration("camera_target_pose_topic"),
                        "publish_rate": ParameterValue(
                            LaunchConfiguration("target_estimator_publish_rate"),
                            value_type=float,
                        ),
                        "sample_step": ParameterValue(
                            LaunchConfiguration("target_estimator_sample_step"),
                            value_type=int,
                        ),
                        "min_x": ParameterValue(LaunchConfiguration("target_roi_min_x"), value_type=float),
                        "max_x": ParameterValue(LaunchConfiguration("target_roi_max_x"), value_type=float),
                        "min_y": ParameterValue(LaunchConfiguration("target_roi_min_y"), value_type=float),
                        "max_y": ParameterValue(LaunchConfiguration("target_roi_max_y"), value_type=float),
                        "min_z": ParameterValue(LaunchConfiguration("target_roi_min_z"), value_type=float),
                        "max_z": ParameterValue(LaunchConfiguration("target_roi_max_z"), value_type=float),
                        "voxel_size": ParameterValue(
                            LaunchConfiguration("target_estimator_voxel_size"),
                            value_type=float,
                        ),
                        "cluster_radius": ParameterValue(
                            LaunchConfiguration("target_estimator_cluster_radius"),
                            value_type=float,
                        ),
                        "min_cluster_points": ParameterValue(
                            LaunchConfiguration("target_estimator_min_cluster_points"),
                            value_type=int,
                        ),
                        "surface_front_ratio": ParameterValue(
                            LaunchConfiguration("target_estimator_surface_front_ratio"),
                            value_type=float,
                        ),
                        "depth_weight": ParameterValue(
                            LaunchConfiguration("target_estimator_depth_weight"),
                            value_type=float,
                        ),
                        "offset_x": ParameterValue(LaunchConfiguration("target_estimator_offset_x"), value_type=float),
                        "offset_y": ParameterValue(LaunchConfiguration("target_estimator_offset_y"), value_type=float),
                        "offset_z": ParameterValue(LaunchConfiguration("target_estimator_offset_z"), value_type=float),
                        "orientation_x": ParameterValue(
                            LaunchConfiguration("target_estimator_qx"),
                            value_type=float,
                        ),
                        "orientation_y": ParameterValue(
                            LaunchConfiguration("target_estimator_qy"),
                            value_type=float,
                        ),
                        "orientation_z": ParameterValue(
                            LaunchConfiguration("target_estimator_qz"),
                            value_type=float,
                        ),
                        "orientation_w": ParameterValue(
                            LaunchConfiguration("target_estimator_qw"),
                            value_type=float,
                        ),
                        "publish_debug_cloud": ParameterValue(
                            LaunchConfiguration("target_estimator_publish_debug_cloud"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package=hardware_package,
                executable="camera_target_pose_relay",
                name="medipick_camera_target_pose_relay",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_target_pose_relay")),
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("camera_target_pose_topic"),
                        "output_topic": "/medipick/task/target_pose",
                        "output_frame": LaunchConfiguration("target_pose_output_frame"),
                        "replace_stamp_with_now": ParameterValue(
                            LaunchConfiguration("target_pose_replace_stamp_with_now"),
                            value_type=bool,
                        ),
                        "passthrough_if_tf_unavailable": ParameterValue(
                            LaunchConfiguration("target_pose_passthrough_if_tf_unavailable"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
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
                condition=UnlessCondition(use_pointcloud_octomap),
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
                    octomap_parameters,
                    octomap_sensor_parameters,
                    planning_scene_monitor_parameters,
                ],
                condition=IfCondition(use_pointcloud_octomap),
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
                        "topic": "/medipick/task/target_pose",
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
