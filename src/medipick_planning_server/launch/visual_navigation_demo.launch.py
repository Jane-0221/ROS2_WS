from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
    sensor_name = sensors_3d["sensors"][0]
    sensor_config = sensors_3d[sensor_name]

    rtabmap_database_path = str(Path.home() / ".ros" / "medipick_rtabmap.db")
    registry_autosave_file = str(Path.home() / ".ros" / "medipick_medicine_locations.yaml")

    calibration_file = str(Path(get_package_share_directory(hardware_package)) / "config" / "medipick_hardware.yaml")
    planning_server_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "planning_server.yaml"
    )
    task_manager_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "navigation_task_manager.yaml"
    )
    ekf_config = str(Path(get_package_share_directory(planning_server_package)) / "config" / "ekf_visual.yaml")
    rtabmap_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "rtabmap_visual_nav.yaml"
    )
    nav2_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "nav2_holonomic.yaml"
    )
    apriltag_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "apriltag_ros.yaml"
    )
    apriltag_anchors = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "apriltag_anchors.yaml"
    )
    registry_seed = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "medicine_registry_seed.yaml"
    )
    fetch_coordinator_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "fetch_medicine_coordinator.yaml"
    )
    rviz_config_file = str(Path(get_package_share_directory(moveit_package)) / "rviz" / "moveit.rviz")

    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    octomap_parameters = {
        "octomap_frame": LaunchConfiguration("octomap_frame"),
        "octomap_resolution": ParameterValue(LaunchConfiguration("octomap_resolution"), value_type=float),
        "max_range": ParameterValue(LaunchConfiguration("octomap_max_range"), value_type=float),
    }
    octomap_sensor_parameters = {
        "sensors": sensors_3d["sensors"],
        f"{sensor_name}.sensor_plugin": sensor_config["sensor_plugin"],
        f"{sensor_name}.point_cloud_topic": point_cloud_topic,
        f"{sensor_name}.max_range": ParameterValue(LaunchConfiguration("octomap_max_range"), value_type=float),
        f"{sensor_name}.point_subsample": sensor_config["point_subsample"],
        f"{sensor_name}.padding_offset": sensor_config["padding_offset"],
        f"{sensor_name}.padding_scale": sensor_config["padding_scale"],
        f"{sensor_name}.max_update_rate": sensor_config["max_update_rate"],
        f"{sensor_name}.filtered_cloud_topic": sensor_config["filtered_cloud_topic"],
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
            DeclareLaunchArgument("serial_port", default_value="/dev/ttySTM32"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("wheeltec_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("wheeltec_baudrate", default_value="115200"),
            DeclareLaunchArgument("calibration_file", default_value=calibration_file),
            DeclareLaunchArgument("camera_launch_file", default_value="gemini_330_series.launch.py"),
            DeclareLaunchArgument("camera_model", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_time_domain", default_value="device"),
            DeclareLaunchArgument("camera_enable_color", default_value="true"),
            DeclareLaunchArgument("camera_enable_depth", default_value="true"),
            DeclareLaunchArgument("camera_enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("camera_enable_ir", default_value="false"),
            DeclareLaunchArgument("camera_enable_accel", default_value="true"),
            DeclareLaunchArgument("camera_enable_gyro", default_value="true"),
            DeclareLaunchArgument("camera_enable_sync_imu", default_value="true"),
            DeclareLaunchArgument("camera_publish_tf", default_value="true"),
            DeclareLaunchArgument("camera_tf_publish_rate", default_value="0.0"),
            DeclareLaunchArgument("camera_log_level", default_value="info"),
            DeclareLaunchArgument("camera_frame_parent", default_value="base_link"),
            DeclareLaunchArgument("camera_frame_child", default_value="camera_link"),
            DeclareLaunchArgument("camera_mount_x", default_value="0.18"),
            DeclareLaunchArgument("camera_mount_y", default_value="0.0"),
            DeclareLaunchArgument("camera_mount_z", default_value="1.20"),
            DeclareLaunchArgument("camera_mount_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_mount_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_mount_yaw", default_value="0.0"),
            DeclareLaunchArgument("point_cloud_topic", default_value="/camera/depth/points"),
            DeclareLaunchArgument("local_pick_pose_topic", default_value="/camera/local_pick_pose"),
            DeclareLaunchArgument("local_pick_resolver_output_topic", default_value="/medipick/vision/local_pick_pose"),
            DeclareLaunchArgument("local_pick_resolver_output_frame", default_value="base_link"),
            DeclareLaunchArgument("local_pick_max_pose_age_sec", default_value="2.0"),
            DeclareLaunchArgument("local_pick_nominal_confidence", default_value="0.85"),
            DeclareLaunchArgument("local_pick_allow_reference_fallback", default_value="false"),
            DeclareLaunchArgument("local_pick_fallback_confidence", default_value="0.30"),
            DeclareLaunchArgument("octomap_frame", default_value="world"),
            DeclareLaunchArgument("octomap_resolution", default_value="0.03"),
            DeclareLaunchArgument("octomap_max_range", default_value="3.0"),
            DeclareLaunchArgument("ekf_config_file", default_value=ekf_config),
            DeclareLaunchArgument("rtabmap_config_file", default_value=rtabmap_config),
            DeclareLaunchArgument("rtabmap_database_path", default_value=rtabmap_database_path),
            DeclareLaunchArgument("nav2_config_file", default_value=nav2_config),
            DeclareLaunchArgument("apriltag_config_file", default_value=apriltag_config),
            DeclareLaunchArgument("apriltag_anchors_file", default_value=apriltag_anchors),
            DeclareLaunchArgument("medicine_registry_seed_file", default_value=registry_seed),
            DeclareLaunchArgument("medicine_registry_autosave_file", default_value=registry_autosave_file),
            DeclareLaunchArgument("fetch_coordinator_config_file", default_value=fetch_coordinator_config),
            DeclareLaunchArgument(
                "motor_arm_joint_command_topic",
                default_value="/medipick/hardware/motor_arm_joint_command",
            ),
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
                    "enable_color": LaunchConfiguration("camera_enable_color"),
                    "enable_depth": LaunchConfiguration("camera_enable_depth"),
                    "enable_point_cloud": LaunchConfiguration("camera_enable_point_cloud"),
                    "enable_left_ir": LaunchConfiguration("camera_enable_ir"),
                    "enable_right_ir": LaunchConfiguration("camera_enable_ir"),
                    "enable_accel": LaunchConfiguration("camera_enable_accel"),
                    "enable_gyro": LaunchConfiguration("camera_enable_gyro"),
                    "enable_sync_output_accel_gyro": LaunchConfiguration("camera_enable_sync_imu"),
                    "time_domain": LaunchConfiguration("camera_time_domain"),
                    "publish_tf": LaunchConfiguration("camera_publish_tf"),
                    "tf_publish_rate": LaunchConfiguration("camera_tf_publish_rate"),
                    "log_level": LaunchConfiguration("camera_log_level"),
                }.items(),
            ),
            Node(
                package=hardware_package,
                executable="camera_pointcloud_target_estimator",
                name="medipick_camera_pointcloud_target_estimator",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_local_pick_estimator")),
                parameters=[
                    {
                        "input_topic": point_cloud_topic,
                        "output_topic": LaunchConfiguration("local_pick_pose_topic"),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="medipick_base_to_camera_tf",
                output="screen",
                arguments=[
                    "--x", LaunchConfiguration("camera_mount_x"),
                    "--y", LaunchConfiguration("camera_mount_y"),
                    "--z", LaunchConfiguration("camera_mount_z"),
                    "--roll", LaunchConfiguration("camera_mount_roll"),
                    "--pitch", LaunchConfiguration("camera_mount_pitch"),
                    "--yaw", LaunchConfiguration("camera_mount_yaw"),
                    "--frame-id", LaunchConfiguration("camera_frame_parent"),
                    "--child-frame-id", LaunchConfiguration("camera_frame_child"),
                ],
            ),
            Node(
                package=hardware_package,
                executable="wheeltec_chassis_node",
                name="wheeltec_chassis_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_base")),
                parameters=[
                    {
                        "robot_type": "omni",
                        "protocol": "serial",
                        "port": LaunchConfiguration("wheeltec_port"),
                        "baudrate": ParameterValue(LaunchConfiguration("wheeltec_baudrate"), value_type=int),
                        "publish_odom": False,
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
                        "publish_full_joint_states": False,
                        "publish_base_joints": False,
                        "arm_joint_states_topic": "/medipick/hardware/arm_joint_states",
                    }
                ],
            ),
            Node(
                package=hardware_package,
                executable="base_pose_joint_state_bridge",
                name="medipick_base_pose_joint_state_bridge",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/odometry/filtered",
                        "output_topic": "/medipick/hardware/base_joint_states",
                    }
                ],
            ),
            Node(
                package=hardware_package,
                executable="joint_state_mux",
                name="medipick_joint_state_mux",
                output="screen",
                parameters=[
                    {
                        "input_topics": [
                            "/medipick/hardware/arm_joint_states",
                            "/medipick/hardware/base_joint_states",
                        ],
                        "output_topic": "/joint_states",
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
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_slam")),
                parameters=[LaunchConfiguration("ekf_config_file")],
            ),
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_slam")),
                parameters=[
                    LaunchConfiguration("rtabmap_config_file"),
                    {
                        "publish_tf": False,
                    },
                ],
                remappings=[
                    ("rgb/image", "/camera/color/image_raw"),
                    ("depth/image", "/camera/depth/image_raw"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("odom", "/visual_odom"),
                ],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_slam")),
                parameters=[
                    LaunchConfiguration("rtabmap_config_file"),
                    {
                        "database_path": LaunchConfiguration("rtabmap_database_path"),
                    },
                ],
                remappings=[
                    ("rgb/image", "/camera/color/image_raw"),
                    ("depth/image", "/camera/depth/image_raw"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("odom", "/odometry/filtered"),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
                ),
                condition=IfCondition(LaunchConfiguration("start_nav2")),
                launch_arguments={
                    "use_sim_time": "False",
                    "params_file": LaunchConfiguration("nav2_config_file"),
                    "autostart": "True",
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_apriltag")),
                parameters=[LaunchConfiguration("apriltag_config_file")],
                remappings=[
                    ("image_rect", "/camera/color/image_raw"),
                    ("camera_info", "/camera/color/camera_info"),
                ],
            ),
            Node(
                package=hardware_package,
                executable="apriltag_anchor_localizer",
                name="medipick_apriltag_anchor_localizer",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_apriltag")),
                parameters=[
                    {
                        "anchors_file": LaunchConfiguration("apriltag_anchors_file"),
                        "camera_frame": "camera_color_optical_frame",
                        "base_frame": "base_link",
                        "map_frame": "map",
                    }
                ],
            ),
            Node(
                package=hardware_package,
                executable="local_pick_pose_resolver",
                name="medipick_local_pick_pose_resolver",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_local_pick_resolver")),
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("local_pick_pose_topic"),
                        "output_topic": LaunchConfiguration("local_pick_resolver_output_topic"),
                        "output_frame": LaunchConfiguration("local_pick_resolver_output_frame"),
                        "max_pose_age_sec": ParameterValue(
                            LaunchConfiguration("local_pick_max_pose_age_sec"),
                            value_type=float,
                        ),
                        "nominal_confidence": ParameterValue(
                            LaunchConfiguration("local_pick_nominal_confidence"),
                            value_type=float,
                        ),
                        "allow_reference_pose_fallback": ParameterValue(
                            LaunchConfiguration("local_pick_allow_reference_fallback"),
                            value_type=bool,
                        ),
                        "fallback_confidence": ParameterValue(
                            LaunchConfiguration("local_pick_fallback_confidence"),
                            value_type=float,
                        ),
                    }
                ],
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
                    octomap_parameters,
                    octomap_sensor_parameters,
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
                package=planning_server_package,
                executable="medicine_location_registry.py",
                name="medipick_medicine_location_registry",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_semantic_registry")),
                parameters=[
                    {
                        "seed_file": LaunchConfiguration("medicine_registry_seed_file"),
                        "autosave_file": LaunchConfiguration("medicine_registry_autosave_file"),
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="fetch_medicine_coordinator.py",
                name="medipick_fetch_medicine_coordinator",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_fetch_coordinator")),
                parameters=[
                    LaunchConfiguration("fetch_coordinator_config_file"),
                    {
                        "output_target_frame": "world",
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
