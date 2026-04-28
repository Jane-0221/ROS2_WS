from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    moveit_package = "medipick_moveit_config"
    description_package = "medipick_simple3_description"
    planning_server_package = "medipick_planning_server"

    default_world = (
        Path(get_package_share_directory(description_package))
        / "worlds"
        / "medipick_pharmacy_textured.world.sdf"
    )
    gazebo_trial_demo_launch = (
        Path(get_package_share_directory(moveit_package))
        / "launch"
        / "gazebo_trial_demo.launch.py"
    )
    slam_toolbox_params_file = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "slam_toolbox_mapper_params_gazebo.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("with_move_group", default_value="false"),
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("spawn_delay", default_value="2.0"),
            DeclareLaunchArgument("state_publish_delay", default_value="2.5"),
            DeclareLaunchArgument("depth_to_scan", default_value="true"),
            DeclareLaunchArgument("use_cmd_vel_bridge", default_value="true"),
            DeclareLaunchArgument("slam_start_delay", default_value="16.0"),
            DeclareLaunchArgument("slam_toolbox_params_file", default_value=str(slam_toolbox_params_file)),
            DeclareLaunchArgument("target_seed", default_value="0"),
            DeclareLaunchArgument("target_entity_name", default_value=""),
            DeclareLaunchArgument("target_pose_frame", default_value="map"),
            DeclareLaunchArgument("camera_frame", default_value="cam_sensor_link"),
            DeclareLaunchArgument("scan_generator", default_value="pointcloud_to_laserscan"),
            DeclareLaunchArgument("scan_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_min_height", default_value="0.30"),
            DeclareLaunchArgument("scan_max_height", default_value="2.00"),
            DeclareLaunchArgument("scan_transform_tolerance", default_value="0.20"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("slam_map_topic", default_value="/rtabmap/grid_prob_map"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gazebo_trial_demo_launch)),
                launch_arguments={
                    "rviz": LaunchConfiguration("rviz"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "with_move_group": LaunchConfiguration("with_move_group"),
                    "world_file": LaunchConfiguration("world_file"),
                    "spawn_delay": LaunchConfiguration("spawn_delay"),
                    "state_publish_delay": LaunchConfiguration("state_publish_delay"),
                }.items(),
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_cmd_vel_bridge.py",
                name="medipick_gazebo_cmd_vel_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_cmd_vel_bridge")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "trajectory_topic": "/base_controller/joint_trajectory",
                        "control_rate_hz": 15.0,
                        "lookahead_time": 0.25,
                        "command_timeout": 0.40,
                        "max_linear_speed": 0.35,
                        "max_lateral_speed": 0.35,
                        "max_angular_speed": 0.80,
                        "trajectory_linear_gain": 6.0,
                        "trajectory_lateral_gain": 6.0,
                        "trajectory_angular_gain": 4.0,
                        "max_target_linear_speed": 2.0,
                        "max_target_lateral_speed": 2.0,
                        "max_target_angular_speed": 2.5,
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_ground_truth_localization.py",
                name="medipick_gazebo_ground_truth_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "amcl_pose_topic": "/amcl_pose",
                        "publish_rate_hz": 20.0,
                        "publish_static_tf": True,
                        "publish_map_to_odom_tf": False,
                        "publish_odom_to_world_tf": True,
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_box_target_publisher.py",
                name="medipick_gazebo_box_target_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_file": LaunchConfiguration("world_file"),
                        "frame_id": LaunchConfiguration("target_pose_frame"),
                        "world_frame": "world",
                        "camera_frame": LaunchConfiguration("camera_frame"),
                        "target_seed": LaunchConfiguration("target_seed"),
                        "target_entity_name": LaunchConfiguration("target_entity_name"),
                        "start_delay": LaunchConfiguration("slam_start_delay"),
                        "publish_world_pose": True,
                        "publish_camera_pose": True,
                    }
                ],
            ),
            Node(
                package="depthimage_to_laserscan",
                executable="depthimage_to_laserscan_node",
                name="medipick_depthimage_to_laserscan",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("depth_to_scan"),
                            "' == 'true' and '",
                            LaunchConfiguration("scan_generator"),
                            "' == 'depthimage_to_laserscan'",
                        ]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": True,
                        "output_frame": LaunchConfiguration("camera_frame"),
                        "scan_time": 0.1,
                        "range_min": 0.10,
                        "range_max": 3.50,
                        "scan_height": 10,
                    }
                ],
                remappings=[
                    ("depth", "/medipick/depth_camera/depth_image"),
                    ("depth_camera_info", "/medipick/depth_camera/camera_info"),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="medipick_pointcloud_to_laserscan",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("depth_to_scan"),
                            "' == 'true' and '",
                            LaunchConfiguration("scan_generator"),
                            "' == 'pointcloud_to_laserscan'",
                        ]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": True,
                        "target_frame": LaunchConfiguration("scan_frame"),
                        "transform_tolerance": LaunchConfiguration("scan_transform_tolerance"),
                        "min_height": LaunchConfiguration("scan_min_height"),
                        "max_height": LaunchConfiguration("scan_max_height"),
                        "angle_min": -1.05,
                        "angle_max": 1.05,
                        "angle_increment": 0.0058,
                        "scan_time": 0.1,
                        "range_min": 0.10,
                        "range_max": 3.50,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                remappings=[
                    ("cloud_in", "/medipick/depth_camera/points"),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("slam_start_delay"),
                actions=[
                    Node(
                        package="slam_toolbox",
                        executable="async_slam_toolbox_node",
                        name="slam_toolbox",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("slam_toolbox_params_file"),
                            {
                                "use_sim_time": True,
                            },
                        ],
                        remappings=[
                            ("scan", LaunchConfiguration("scan_topic")),
                            ("map", LaunchConfiguration("slam_map_topic")),
                        ],
                    )
                ],
            ),
        ]
    )
