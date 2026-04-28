from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    description_package = "medipick_simple3_description"
    moveit_package = "medipick_moveit_config"
    planning_server_package = "medipick_planning_server"

    world_file = str(
        Path(get_package_share_directory(description_package)) / "worlds" / "medipick_pharmacy_textured.world.sdf"
    )
    gazebo_trial_demo_launch = str(
        Path(get_package_share_directory(moveit_package)) / "launch" / "gazebo_trial_demo.launch.py"
    )
    rtabmap_launch = str(Path(get_package_share_directory("rtabmap_launch")) / "launch" / "rtabmap.launch.py")

    resource_paths = [
        str(Path(get_package_share_directory(description_package))),
        str(Path(get_package_share_directory(description_package)) / "models"),
        str(Path(get_package_share_directory(moveit_package))),
    ]

    controllers_ready_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_controllers_ready_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "topic": LaunchConfiguration("controllers_ready_topic"),
                "required_value": True,
                "log_period_sec": 5.0,
            }
        ],
    )

    rtabmap_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            "use_sim_time": TextSubstitution(text="true"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "rviz": TextSubstitution(text="false"),
            "depth": TextSubstitution(text="true"),
            "frame_id": TextSubstitution(text="base_link"),
            "publish_tf_map": TextSubstitution(text="true"),
            "visual_odometry": LaunchConfiguration("rtabmap_use_visual_odometry"),
            "icp_odometry": TextSubstitution(text="false"),
            "vo_frame_id": LaunchConfiguration("rtabmap_vo_frame_id"),
            "publish_tf_odom": TextSubstitution(text="true"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "wait_imu_to_init": LaunchConfiguration("rtabmap_wait_imu_to_init"),
            "approx_sync_max_interval": LaunchConfiguration("rtabmap_approx_sync_max_interval"),
            "topic_queue_size": LaunchConfiguration("rtabmap_topic_queue_size"),
            "sync_queue_size": LaunchConfiguration("rtabmap_sync_queue_size"),
            "wait_for_transform": LaunchConfiguration("rtabmap_wait_for_transform"),
            "subscribe_scan": LaunchConfiguration("rtabmap_subscribe_scan"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "rgbd_sync": TextSubstitution(text="false"),
            "approx_sync": TextSubstitution(text="true"),
            "approx_rgbd_sync": TextSubstitution(text="true"),
            "qos": TextSubstitution(text="2"),
            "qos_image": TextSubstitution(text="2"),
            "qos_camera_info": TextSubstitution(text="2"),
            "qos_scan": TextSubstitution(text="2"),
            "qos_odom": TextSubstitution(text="2"),
            "database_path": LaunchConfiguration("rtabmap_database_path"),
            "gui_cfg": LaunchConfiguration("rtabmap_gui_cfg"),
            "args": [
                TextSubstitution(text="--delete_db_on_start --Rtabmap/DetectionRate "),
                LaunchConfiguration("rtabmap_detection_rate"),
                TextSubstitution(
                    text=" --Vis/MinInliers "
                ),
                LaunchConfiguration("rtabmap_vis_min_inliers"),
                TextSubstitution(
                    text=" --Vis/MinInliersDistribution "
                ),
                LaunchConfiguration("rtabmap_vis_min_inliers_distribution"),
                TextSubstitution(
                    text=" --Vis/GridRows "
                ),
                LaunchConfiguration("rtabmap_vis_grid_rows"),
                TextSubstitution(
                    text=" --Vis/GridCols "
                ),
                LaunchConfiguration("rtabmap_vis_grid_cols"),
                TextSubstitution(
                    text=" --Kp/GridRows "
                ),
                LaunchConfiguration("rtabmap_kp_grid_rows"),
                TextSubstitution(
                    text=" --Kp/GridCols "
                ),
                LaunchConfiguration("rtabmap_kp_grid_cols"),
                TextSubstitution(
                    text=" --Kp/MaxFeatures "
                ),
                LaunchConfiguration("rtabmap_kp_max_features"),
                TextSubstitution(
                    text=" --Kp/MinDepth "
                ),
                LaunchConfiguration("rtabmap_kp_min_depth"),
                TextSubstitution(
                    text=" --Kp/MaxDepth "
                ),
                LaunchConfiguration("rtabmap_kp_max_depth"),
                TextSubstitution(
                    text=" --Vis/CorNNDR "
                ),
                LaunchConfiguration("rtabmap_vis_cor_nndr"),
                TextSubstitution(
                    text=" --Rtabmap/LoopThr "
                ),
                LaunchConfiguration("rtabmap_loop_thr"),
                TextSubstitution(
                    text=
                    " --Rtabmap/ImageBufferSize 1"
                    " --Rtabmap/MemoryThr 0"
                    " --RGBD/LinearUpdate "
                ),
                LaunchConfiguration("rtabmap_linear_update"),
                TextSubstitution(
                    text=
                    " --RGBD/AngularUpdate "
                ),
                LaunchConfiguration("rtabmap_angular_update"),
                TextSubstitution(
                    text=
                    " --Mem/DepthCompressionFormat .png"
                    " --Reg/Strategy "
                ),
                LaunchConfiguration("rtabmap_reg_strategy"),
                PythonExpression(
                    [
                        "' --RGBD/OptimizeMaxError 0 --Optimizer/Robust true'"
                        " if '",
                        LaunchConfiguration("rtabmap_optimizer_robust"),
                        "' == 'true' else ' --RGBD/OptimizeMaxError ",
                        LaunchConfiguration("rtabmap_optimize_max_error"),
                        "'",
                    ]
                ),
                TextSubstitution(
                    text=" --RGBD/ProximityBySpace "
                ),
                LaunchConfiguration("rtabmap_proximity_by_space"),
                TextSubstitution(
                    text=
                    " --RGBD/ProximityPathMaxNeighbors 1"
                    " --Mem/RehearsalSimilarity 0.40"
                    " --Mem/BadSignaturesIgnored "
                ),
                LaunchConfiguration("rtabmap_mem_bad_signatures_ignored"),
                TextSubstitution(
                    text=
                    " --Vis/DepthMaskFloorThr "
                ),
                LaunchConfiguration("rtabmap_vis_depth_mask_floor_thr"),
                TextSubstitution(
                    text=
                    " --Grid/FromDepth true"
                    " --Grid/RangeMax 6.0"
                ),
                PythonExpression(
                    [
                        "' --Reg/Force3DoF true --Optimizer/Slam2D true --RGBD/OptimizeSlam2D true'"
                        " if '",
                        LaunchConfiguration("rtabmap_force_3dof"),
                        "' == 'true' else ''",
                    ]
                ),
                PythonExpression(
                    [
                        "' --Grid/3D true'"
                        " if '",
                        LaunchConfiguration("rtabmap_grid_3d"),
                        "' == 'true' else ' --Grid/3D false'",
                    ]
                ),
                PythonExpression(
                    [
                        "' ' + '",
                        LaunchConfiguration("rtabmap_extra_args"),
                        "' if '",
                        LaunchConfiguration("rtabmap_extra_args"),
                        "' else ''",
                    ]
                ),
            ],
        }.items(),
    )

    rtabmap_param_tuner = Node(
        package=planning_server_package,
        executable="rtabmap_param_tuner.py",
        name="medipick_rtabmap_param_tuner",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "target_node": "/rtabmap/rtabmap",
                "tf_delay": LaunchConfiguration("rtabmap_tf_delay"),
                "tf_tolerance": LaunchConfiguration("rtabmap_tf_tolerance"),
                "wait_timeout_sec": 20.0,
                "ready_topic": LaunchConfiguration("rtabmap_ready_topic"),
            }
        ],
    )

    latched_map_republisher = Node(
        package=planning_server_package,
        executable="latched_occupancy_grid_republisher.py",
        name="medipick_latched_slam_map_republisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": LaunchConfiguration("slam_map_input_topic"),
                "output_topic": LaunchConfiguration("slam_map_output_topic"),
                "ready_topic": LaunchConfiguration("slam_map_ready_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("with_move_group", default_value="false"),
            DeclareLaunchArgument("spawn_mobile_arm_controller", default_value="true"),
            DeclareLaunchArgument("spawn_head_controller", default_value="true"),
            DeclareLaunchArgument("spawn_tool_controller", default_value="true"),
            DeclareLaunchArgument("controllers_ready_after", default_value=""),
            DeclareLaunchArgument("world_file", default_value=world_file),
            DeclareLaunchArgument("spawn_delay", default_value="2.0"),
            DeclareLaunchArgument("state_publish_delay", default_value="2.5"),
            DeclareLaunchArgument("depth_to_scan", default_value="true"),
            DeclareLaunchArgument("scan_generator", default_value="depthimage_to_laserscan"),
            DeclareLaunchArgument("scan_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_min_height", default_value="0.30"),
            DeclareLaunchArgument("scan_max_height", default_value="2.00"),
            DeclareLaunchArgument("scan_transform_tolerance", default_value="0.50"),
            DeclareLaunchArgument("rtabmap_subscribe_scan", default_value="false"),
            DeclareLaunchArgument("imu_topic", default_value="/medipick/imu"),
            DeclareLaunchArgument("rtabmap_wait_imu_to_init", default_value="true"),
            DeclareLaunchArgument("rtabmap_force_3dof", default_value="false"),
            DeclareLaunchArgument("rtabmap_grid_3d", default_value="false"),
            DeclareLaunchArgument("rtabmap_loop_thr", default_value="0.15"),
            DeclareLaunchArgument("rtabmap_vis_min_inliers", default_value="20"),
            DeclareLaunchArgument("rtabmap_vis_min_inliers_distribution", default_value="0.0125"),
            DeclareLaunchArgument("rtabmap_vis_grid_rows", default_value="2"),
            DeclareLaunchArgument("rtabmap_vis_grid_cols", default_value="2"),
            DeclareLaunchArgument("rtabmap_kp_grid_rows", default_value="2"),
            DeclareLaunchArgument("rtabmap_kp_grid_cols", default_value="2"),
            DeclareLaunchArgument("rtabmap_kp_max_features", default_value="500"),
            DeclareLaunchArgument("rtabmap_kp_min_depth", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_kp_max_depth", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_vis_cor_nndr", default_value="0.8"),
            DeclareLaunchArgument("rtabmap_vis_depth_mask_floor_thr", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_reg_strategy", default_value="0"),
            DeclareLaunchArgument("rtabmap_optimizer_robust", default_value="false"),
            DeclareLaunchArgument("rtabmap_mem_bad_signatures_ignored", default_value="false"),
            DeclareLaunchArgument("rtabmap_optimize_max_error", default_value="3.0"),
            DeclareLaunchArgument("rtabmap_proximity_by_space", default_value="true"),
            DeclareLaunchArgument("rtabmap_extra_args", default_value=""),
            DeclareLaunchArgument("sim_base_mode", default_value="omni_driver"),
            DeclareLaunchArgument("use_cmd_vel_bridge", default_value="true"),
            DeclareLaunchArgument("controllers_ready_topic", default_value="/medipick/controllers_ready"),
            DeclareLaunchArgument("rtabmap_ready_topic", default_value="/medipick/rtabmap_ready"),
            DeclareLaunchArgument("slam_map_input_topic", default_value="/rtabmap/grid_prob_map"),
            DeclareLaunchArgument("slam_map_output_topic", default_value="/medipick/slam_map_latched"),
            DeclareLaunchArgument("slam_map_ready_topic", default_value="/medipick/slam_map_ready"),
            DeclareLaunchArgument("rtabmap_start_delay", default_value="1.0"),
            DeclareLaunchArgument("rtabmap_database_path", default_value="~/.ros/medipick_gazebo_rtabmap.db"),
            DeclareLaunchArgument("rtabmap_gui_cfg", default_value="~/.ros/rtabmapGUI.ini"),
            DeclareLaunchArgument("rtabmap_detection_rate", default_value="4.0"),
            DeclareLaunchArgument("rtabmap_linear_update", default_value="0.08"),
            DeclareLaunchArgument("rtabmap_angular_update", default_value="0.08"),
            DeclareLaunchArgument("rtabmap_use_visual_odometry", default_value="false"),
            DeclareLaunchArgument("rtabmap_vo_frame_id", default_value="odom"),
            DeclareLaunchArgument("rtabmap_topic_queue_size", default_value="30"),
            DeclareLaunchArgument("rtabmap_sync_queue_size", default_value="30"),
            DeclareLaunchArgument("rtabmap_approx_sync_max_interval", default_value="0.20"),
            DeclareLaunchArgument("rtabmap_wait_for_transform", default_value="0.40"),
            DeclareLaunchArgument("rtabmap_tf_delay", default_value="0.30"),
            DeclareLaunchArgument("rtabmap_tf_tolerance", default_value="0.40"),
            DeclareLaunchArgument("target_seed", default_value="0"),
            DeclareLaunchArgument("target_entity_name", default_value=""),
            DeclareLaunchArgument("target_pose_frame", default_value="map"),
            DeclareLaunchArgument("publish_target_box", default_value="true"),
            DeclareLaunchArgument("raw_rgb_topic", default_value="/medipick/depth_camera/image"),
            DeclareLaunchArgument("raw_depth_topic", default_value="/medipick/depth_camera/depth_image"),
            DeclareLaunchArgument("rgb_topic", default_value="/medipick/depth_camera/image_synced"),
            DeclareLaunchArgument("depth_topic", default_value="/medipick/depth_camera/depth_image_synced"),
            DeclareLaunchArgument("raw_camera_info_topic", default_value="/medipick/depth_camera/camera_info"),
            DeclareLaunchArgument("camera_info_topic", default_value="/medipick/depth_camera/camera_info_synced"),
            DeclareLaunchArgument("camera_frame", default_value="medipick/h2_link/head_rgbd"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("image_stamp_backoff_sec", default_value="0.25"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join(resource_paths)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_trial_demo_launch),
                launch_arguments={
                    "rviz": LaunchConfiguration("rviz"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "with_move_group": LaunchConfiguration("with_move_group"),
                    "world_file": LaunchConfiguration("world_file"),
                    "spawn_delay": LaunchConfiguration("spawn_delay"),
                    "state_publish_delay": LaunchConfiguration("state_publish_delay"),
                    "bridge_pointcloud": TextSubstitution(text="false"),
                    "spawn_mobile_arm_controller": LaunchConfiguration("spawn_mobile_arm_controller"),
                    "spawn_head_controller": LaunchConfiguration("spawn_head_controller"),
                    "spawn_tool_controller": LaunchConfiguration("spawn_tool_controller"),
                    "controllers_ready_after": LaunchConfiguration("controllers_ready_after"),
                }.items(),
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_omni_base_driver.py",
                name="medipick_gazebo_omni_base_driver",
                output="screen",
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("sim_base_mode"), "' == 'omni_driver'"])
                ),
                parameters=[
                    {
                        "use_sim_time": True,
                        "joint_state_topic": "/medipick/joint_states_sim",
                        "trajectory_topic": "/base_controller/joint_trajectory",
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "amcl_pose_topic": "/amcl_pose",
                        "control_rate_hz": 15.0,
                        "publish_rate_hz": 60.0,
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
                        "publish_identity_tf": True,
                        "publish_map_to_odom_tf": False,
                        "publish_odom_to_world_tf": True,
                        "publish_amcl_pose": True,
                        "odom_stamp_backoff_sec": 0.12,
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_cmd_vel_bridge.py",
                name="medipick_gazebo_cmd_vel_bridge",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("use_cmd_vel_bridge"),
                            "' == 'true' and '",
                            LaunchConfiguration("sim_base_mode"),
                            "' == 'legacy'",
                        ]
                    )
                ),
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
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("rtabmap_use_visual_odometry"),
                            "' != 'true' and '",
                            LaunchConfiguration("sim_base_mode"),
                            "' == 'legacy'",
                        ]
                    )
                ),
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
                condition=IfCondition(LaunchConfiguration("publish_target_box")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_file": LaunchConfiguration("world_file"),
                        "frame_id": LaunchConfiguration("target_pose_frame"),
                        "world_frame": "world",
                        "camera_frame": LaunchConfiguration("camera_frame"),
                        "target_seed": LaunchConfiguration("target_seed"),
                        "target_entity_name": LaunchConfiguration("target_entity_name"),
                        "start_delay": LaunchConfiguration("rtabmap_start_delay"),
                        "publish_world_pose": True,
                        "publish_camera_pose": True,
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="image_stamp_republisher.py",
                name="medipick_rgb_image_stamp_republisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": LaunchConfiguration("raw_rgb_topic"),
                        "output_topic": LaunchConfiguration("rgb_topic"),
                        "stamp_backoff_sec": LaunchConfiguration("image_stamp_backoff_sec"),
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="image_stamp_republisher.py",
                name="medipick_depth_image_stamp_republisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": LaunchConfiguration("raw_depth_topic"),
                        "output_topic": LaunchConfiguration("depth_topic"),
                        "stamp_backoff_sec": LaunchConfiguration("image_stamp_backoff_sec"),
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="camera_info_stamp_republisher.py",
                name="medipick_camera_info_stamp_republisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "image_topic": LaunchConfiguration("rgb_topic"),
                        "camera_info_topic": LaunchConfiguration("raw_camera_info_topic"),
                        "output_topic": LaunchConfiguration("camera_info_topic"),
                        "horizontal_fov_rad": 1.5707963267948966,
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
                        "range_max": 6.00,
                        "scan_height": 10,
                    }
                ],
                remappings=[
                    ("depth", LaunchConfiguration("depth_topic")),
                    ("depth_camera_info", LaunchConfiguration("camera_info_topic")),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
            controllers_ready_gate,
            latched_map_republisher,
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
                        "range_max": 6.00,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                remappings=[
                    ("cloud_in", "/medipick/depth_camera/points"),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controllers_ready_gate,
                    on_exit=[
                        TimerAction(
                            period=LaunchConfiguration("rtabmap_start_delay"),
                            actions=[rtabmap_include],
                        ),
                        TimerAction(
                            period=PythonExpression([LaunchConfiguration("rtabmap_start_delay"), " + 1.5"]),
                            actions=[rtabmap_param_tuner],
                        ),
                    ],
                )
            ),
        ]
    )
