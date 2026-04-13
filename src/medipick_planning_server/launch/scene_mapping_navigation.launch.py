from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _is_true(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() == "true"


def build_wheeltec_chassis_node(context):
    base_control_mode = LaunchConfiguration("base_control_mode").perform(context).strip()
    if base_control_mode not in {"nav", "stm32"}:
        raise RuntimeError(
            f"Invalid base_control_mode '{base_control_mode}'. Expected 'nav' or 'stm32'."
        )

    cmd_vel_topic = "/cmd_vel"
    if base_control_mode == "stm32":
        cmd_vel_topic = LaunchConfiguration("stm32_base_cmd_vel_topic").perform(context).strip()
    identical_nonzero_cmd_guard_enabled = base_control_mode == "stm32"

    return [
        Node(
            package="robot_hardware",
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
                    "cmd_vel_topic": cmd_vel_topic,
                    "identical_nonzero_cmd_guard_enabled": identical_nonzero_cmd_guard_enabled,
                    "identical_nonzero_cmd_guard_timeout": ParameterValue(
                        LaunchConfiguration("stm32_identical_nonzero_cmd_guard_timeout"),
                        value_type=float,
                    ),
                    "identical_nonzero_cmd_guard_epsilon": ParameterValue(
                        LaunchConfiguration("stm32_identical_nonzero_cmd_guard_epsilon"),
                        value_type=float,
                    ),
                }
            ],
        )
    ]


def build_rtabmap_node(context):
    localization = _is_true(context, "localization")
    delete_db_on_start = _is_true(context, "delete_db_on_start")

    arguments = []
    if delete_db_on_start and not localization:
        arguments.append("-d")

    return [
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_slam")),
            arguments=arguments,
            parameters=[
                LaunchConfiguration("rtabmap_config_file"),
                {
                    "database_path": LaunchConfiguration("rtabmap_database_path"),
                    "Mem/IncrementalMemory": "false" if localization else "true",
                    "Mem/InitWMWithAllNodes": "true" if localization else "false",
                },
            ],
            remappings=[
                ("imu", "/camera/gyro_accel/sample"),
                ("rgb/image", "/camera/color/image_raw"),
                ("depth/image", "/camera/depth/image_raw"),
                ("rgb/camera_info", "/camera/color/camera_info"),
                ("odom", "/odometry/filtered"),
            ],
        )
    ]


def generate_launch_description():
    planning_server_package = "medipick_planning_server"
    hardware_package = "robot_hardware"

    ekf_config = str(Path(get_package_share_directory(planning_server_package)) / "config" / "ekf_visual.yaml")
    rtabmap_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "rtabmap_visual_nav.yaml"
    )
    nav2_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "nav2_holonomic.yaml"
    )
    rtabmap_database_path = str(Path.home() / ".ros" / "medipick_scene_mapping.db")
    calibration_file = str(Path(get_package_share_directory(hardware_package)) / "config" / "medipick_hardware.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_stm32", default_value="true"),
            DeclareLaunchArgument("start_base", default_value="true"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="false"),
            DeclareLaunchArgument("start_rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("delete_db_on_start", default_value="false"),
            DeclareLaunchArgument("base_control_mode", default_value="nav"),
            DeclareLaunchArgument(
                "stm32_base_cmd_vel_topic",
                default_value="/medipick/hardware/stm32_base_cmd_vel",
            ),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttySTM32"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("calibration_file", default_value=calibration_file),
            DeclareLaunchArgument(
                "wheeltec_port",
                default_value="/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00",
            ),
            DeclareLaunchArgument("wheeltec_baudrate", default_value="115200"),
            DeclareLaunchArgument("stm32_identical_nonzero_cmd_guard_timeout", default_value="2.0"),
            DeclareLaunchArgument("stm32_identical_nonzero_cmd_guard_epsilon", default_value="0.0001"),
            DeclareLaunchArgument("camera_launch_file", default_value="gemini_330_series.launch.py"),
            DeclareLaunchArgument("camera_model", default_value="camera"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("camera_time_domain", default_value="system"),
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
            DeclareLaunchArgument("ekf_config_file", default_value=ekf_config),
            DeclareLaunchArgument("rtabmap_config_file", default_value=rtabmap_config),
            DeclareLaunchArgument("rtabmap_database_path", default_value=rtabmap_database_path),
            DeclareLaunchArgument("nav2_config_file", default_value=nav2_config),
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
            OpaqueFunction(function=build_wheeltec_chassis_node),
            Node(
                package="robot_hardware",
                executable="stm32_serial_node",
                name="stm32_serial_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_stm32")),
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "stm32_base_cmd_vel_topic": LaunchConfiguration("stm32_base_cmd_vel_topic"),
                        "publish_full_joint_states": False,
                        "publish_base_joints": False,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_slam")),
                parameters=[
                    LaunchConfiguration("ekf_config_file"),
                    {
                        "publish_tf": True,
                    },
                ],
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
                    ("imu", "/camera/gyro_accel/sample"),
                    ("rgb/image", "/camera/color/image_raw"),
                    ("depth/image", "/camera/depth/image_raw"),
                    ("rgb/camera_info", "/camera/color/camera_info"),
                    ("odom", "/visual_odom"),
                ],
            ),
            OpaqueFunction(function=build_rtabmap_node),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                name="rtabmap_viz",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rtabmap_viz")),
                parameters=[
                    {
                        "frame_id": "base_link",
                        "odom_frame_id": "world",
                        "subscribe_rgb": True,
                        "subscribe_depth": True,
                        "subscribe_odom_info": False,
                        "approx_sync": True,
                        "approx_sync_max_interval": 0.03,
                        "topic_queue_size": 30,
                        "sync_queue_size": 30,
                    }
                ],
                remappings=[
                    ("imu", "/camera/gyro_accel/sample"),
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
        ]
    )
