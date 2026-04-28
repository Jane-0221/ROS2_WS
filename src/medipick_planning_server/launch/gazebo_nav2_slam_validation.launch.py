from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    description_package = "medipick_simple3_description"
    planning_server_package = "medipick_planning_server"

    default_world = (
        Path(get_package_share_directory(description_package))
        / "worlds"
        / "medipick_pharmacy_textured.world.sdf"
    )
    rtabmap_launch = (
        Path(get_package_share_directory(planning_server_package))
        / "launch"
        / "gazebo_rtabmap_slam.launch.py"
    )
    slam_toolbox_launch = (
        Path(get_package_share_directory(planning_server_package))
        / "launch"
        / "gazebo_slam_toolbox.launch.py"
    )
    cartographer_launch = (
        Path(get_package_share_directory(planning_server_package))
        / "launch"
        / "gazebo_cartographer.launch.py"
    )
    nav2_params_file = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "nav2_params.yaml"
    )
    slam_toolbox_params_file = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "slam_toolbox_mapper_params_gazebo.yaml"
    )

    configured_nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration("nav2_params_file"),
        param_rewrites={
            "global_costmap.global_costmap.ros__parameters.static_layer.map_topic": LaunchConfiguration(
                "slam_map_topic"
            ),
            "global_costmap.global_costmap.ros__parameters.static_layer.map_subscribe_transient_local": "true",
        },
        convert_types=True,
    )

    controllers_ready_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_validation_controllers_ready_gate",
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

    mapping_done_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_validation_mapping_done_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "topic": LaunchConfiguration("mapping_done_topic"),
                "required_value": True,
                "log_period_sec": 5.0,
            }
        ],
    )

    rtabmap_ready_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_validation_rtabmap_ready_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "topic": LaunchConfiguration("rtabmap_ready_topic"),
                "required_value": True,
                "log_period_sec": 5.0,
                "durability_transient_local": True,
            }
        ],
    )

    mapping_route_runner = Node(
        package=planning_server_package,
        executable="gazebo_mapping_route_runner.py",
        name="medipick_gazebo_mapping_route_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "odom_topic": "/odom",
                "cmd_vel_topic": "/cmd_vel",
                "done_topic": LaunchConfiguration("mapping_done_topic"),
                "start_delay_sec": 0.0,
                "route_xs": [0.35, 2.15, 2.15, 2.15, 3.50, 4.85, 4.85, 4.85, 6.55, 6.55, 6.55, 4.85, 3.50, 2.15, 2.15, 2.15, 0.35],
                "route_ys": [0.0, 0.0, 3.35, 0.0, 0.0, 0.0, 3.35, 0.0, 0.0, -3.35, 0.0, 0.0, 0.0, 0.0, -3.35, 0.0, 0.0],
                "route_yaws_deg": [0.0, 0.0, 90.0, -90.0, 0.0, 0.0, 90.0, -90.0, 0.0, -90.0, 90.0, 180.0, 180.0, 180.0, -90.0, 90.0, 180.0],
                "waypoint_position_tolerance": 0.24,
                "waypoint_yaw_tolerance_deg": 14.0,
                "kp_linear": 0.9,
                "kp_lateral": 0.9,
                "kp_angular": 1.5,
                "max_linear_speed": 0.36,
                "max_lateral_speed": 0.36,
                "max_angular_speed": 0.80,
                "slowdown_radius": 0.20,
                "yaw_alignment_waypoint_indices": [-1],
                "enable_head_scan": True,
                "head_scan_waypoint_indices": [1, 2, 5, 8, 11, 14],
                "head_scan_h1_sequence_deg": [-25.0, 0.0, 25.0],
                "head_scan_h2_deg": -30.0,
                "head_scan_move_sec": 0.12,
                "head_scan_hold_sec": 0.05,
            }
        ],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[configured_nav2_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[configured_nav2_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_nav2_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[configured_nav2_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_nav2_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_nav2_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[configured_nav2_params],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }
        ],
    )

    slam_loop_validator = Node(
        package=planning_server_package,
        executable="slam_loop_validator.py",
        name="medipick_slam_loop_validator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "map_topic": LaunchConfiguration("slam_map_topic"),
                "map_topic_transient_local": True,
                "navigate_action_name": "/navigate_to_pose",
                "mapping_done_topic": LaunchConfiguration("mapping_done_topic"),
                "rtabmap_info_topic": PythonExpression(
                    [
                        "'/rtabmap/info' if '",
                        LaunchConfiguration("slam_backend"),
                        "' == 'rtabmap' else ''",
                    ]
                ),
                "route_xs": [0.35, 2.15, 2.15, 2.15, 3.50, 4.85, 4.85, 4.85, 6.55, 6.55, 6.55, 4.85, 3.50, 2.15, 2.15, 2.15, 0.35],
                "route_ys": [0.0, 0.0, 3.35, 0.0, 0.0, 0.0, 3.35, 0.0, 0.0, -3.35, 0.0, 0.0, 0.0, 0.0, -3.35, 0.0, 0.0],
                "route_yaws_deg": [180.0, 0.0, 90.0, -90.0, 0.0, 0.0, 90.0, -90.0, 0.0, -90.0, 90.0, 180.0, 180.0, 180.0, -90.0, 90.0, 180.0],
                "waypoint_timeout_sec": 90.0,
                "wait_after_map_ready_sec": 2.0,
                "wait_after_each_goal_sec": 0.6,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("slam_backend", default_value="rtabmap"),
            DeclareLaunchArgument("slam_map_topic", default_value="/medipick/slam_map_latched"),
            DeclareLaunchArgument("nav2_params_file", default_value=str(nav2_params_file)),
            DeclareLaunchArgument("slam_toolbox_params_file", default_value=str(slam_toolbox_params_file)),
            DeclareLaunchArgument("cartographer_config_basename", default_value="cartographer_2d_gazebo.lua"),
            DeclareLaunchArgument("target_seed", default_value="0"),
            DeclareLaunchArgument("target_entity_name", default_value=""),
            DeclareLaunchArgument("rtabmap_gui_cfg", default_value="~/.ros/rtabmapGUI.ini"),
            DeclareLaunchArgument("rtabmap_use_visual_odometry", default_value="false"),
            DeclareLaunchArgument("rtabmap_vo_frame_id", default_value="odom"),
            DeclareLaunchArgument("rtabmap_subscribe_scan", default_value="false"),
            DeclareLaunchArgument("rtabmap_scan_generator", default_value="depthimage_to_laserscan"),
            DeclareLaunchArgument("rtabmap_scan_frame", default_value="base_link"),
            DeclareLaunchArgument("rtabmap_scan_min_height", default_value="0.30"),
            DeclareLaunchArgument("rtabmap_scan_max_height", default_value="2.00"),
            DeclareLaunchArgument("rtabmap_scan_transform_tolerance", default_value="0.20"),
            DeclareLaunchArgument("imu_topic", default_value="/medipick/imu"),
            DeclareLaunchArgument("rtabmap_wait_imu_to_init", default_value="true"),
            DeclareLaunchArgument("rtabmap_force_3dof", default_value="true"),
            DeclareLaunchArgument("rtabmap_grid_3d", default_value="false"),
            DeclareLaunchArgument("rtabmap_loop_thr", default_value="0.15"),
            DeclareLaunchArgument("rtabmap_vis_min_inliers", default_value="15"),
            DeclareLaunchArgument("rtabmap_vis_min_inliers_distribution", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_kp_max_features", default_value="800"),
            DeclareLaunchArgument("rtabmap_kp_min_depth", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_kp_max_depth", default_value="4.0"),
            DeclareLaunchArgument("rtabmap_vis_cor_nndr", default_value="0.8"),
            DeclareLaunchArgument("rtabmap_vis_depth_mask_floor_thr", default_value="0.10"),
            DeclareLaunchArgument("rtabmap_reg_strategy", default_value="2"),
            DeclareLaunchArgument("rtabmap_optimizer_robust", default_value="true"),
            DeclareLaunchArgument("rtabmap_mem_bad_signatures_ignored", default_value="true"),
            DeclareLaunchArgument("rtabmap_optimize_max_error", default_value="0"),
            DeclareLaunchArgument("rtabmap_subscribe_scan", default_value="true"),
            DeclareLaunchArgument("rtabmap_extra_args", default_value=""),
            DeclareLaunchArgument("controllers_ready_topic", default_value="/medipick/controllers_ready"),
            DeclareLaunchArgument("rtabmap_ready_topic", default_value="/medipick/rtabmap_ready"),
            DeclareLaunchArgument("mapping_done_topic", default_value="/medipick/slam_mapping_route_done"),
            DeclareLaunchArgument("mapping_runner_start_delay", default_value="1.0"),
            DeclareLaunchArgument("nav2_start_delay", default_value="8.0"),
            DeclareLaunchArgument("validator_start_delay", default_value="4.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rtabmap_launch)),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                    "rviz": LaunchConfiguration("rviz"),
                    "target_seed": LaunchConfiguration("target_seed"),
                    "target_entity_name": LaunchConfiguration("target_entity_name"),
                    "target_pose_frame": "world",
                    "rtabmap_gui_cfg": LaunchConfiguration("rtabmap_gui_cfg"),
                    "rtabmap_use_visual_odometry": LaunchConfiguration("rtabmap_use_visual_odometry"),
                    "rtabmap_vo_frame_id": LaunchConfiguration("rtabmap_vo_frame_id"),
                    "rtabmap_subscribe_scan": LaunchConfiguration("rtabmap_subscribe_scan"),
                    "scan_generator": LaunchConfiguration("rtabmap_scan_generator"),
                    "scan_frame": LaunchConfiguration("rtabmap_scan_frame"),
                    "scan_min_height": LaunchConfiguration("rtabmap_scan_min_height"),
                    "scan_max_height": LaunchConfiguration("rtabmap_scan_max_height"),
                    "scan_transform_tolerance": LaunchConfiguration("rtabmap_scan_transform_tolerance"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "rtabmap_wait_imu_to_init": LaunchConfiguration("rtabmap_wait_imu_to_init"),
                    "rtabmap_force_3dof": LaunchConfiguration("rtabmap_force_3dof"),
                    "rtabmap_grid_3d": LaunchConfiguration("rtabmap_grid_3d"),
                    "rtabmap_loop_thr": LaunchConfiguration("rtabmap_loop_thr"),
                    "rtabmap_vis_min_inliers": LaunchConfiguration("rtabmap_vis_min_inliers"),
                    "rtabmap_vis_min_inliers_distribution": LaunchConfiguration("rtabmap_vis_min_inliers_distribution"),
                    "rtabmap_kp_max_features": LaunchConfiguration("rtabmap_kp_max_features"),
                    "rtabmap_kp_min_depth": LaunchConfiguration("rtabmap_kp_min_depth"),
                    "rtabmap_kp_max_depth": LaunchConfiguration("rtabmap_kp_max_depth"),
                    "rtabmap_vis_cor_nndr": LaunchConfiguration("rtabmap_vis_cor_nndr"),
                    "rtabmap_vis_depth_mask_floor_thr": LaunchConfiguration("rtabmap_vis_depth_mask_floor_thr"),
                    "rtabmap_reg_strategy": LaunchConfiguration("rtabmap_reg_strategy"),
                    "rtabmap_optimizer_robust": LaunchConfiguration("rtabmap_optimizer_robust"),
                    "rtabmap_mem_bad_signatures_ignored": LaunchConfiguration("rtabmap_mem_bad_signatures_ignored"),
                    "rtabmap_optimize_max_error": LaunchConfiguration("rtabmap_optimize_max_error"),
                    "rtabmap_subscribe_scan": LaunchConfiguration("rtabmap_subscribe_scan"),
                    "rtabmap_extra_args": LaunchConfiguration("rtabmap_extra_args"),
                    "controllers_ready_topic": LaunchConfiguration("controllers_ready_topic"),
                    "rtabmap_ready_topic": LaunchConfiguration("rtabmap_ready_topic"),
                    "sim_base_mode": "omni_driver",
                    "use_cmd_vel_bridge": "true",
                    "spawn_mobile_arm_controller": "false",
                    "spawn_head_controller": "true",
                    "spawn_tool_controller": "false",
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("slam_backend"), "' == 'rtabmap'"])
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(slam_toolbox_launch)),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "with_move_group": "false",
                    "target_seed": LaunchConfiguration("target_seed"),
                    "target_entity_name": LaunchConfiguration("target_entity_name"),
                    "target_pose_frame": "world",
                    "slam_start_delay": "16.0",
                    "slam_toolbox_params_file": LaunchConfiguration("slam_toolbox_params_file"),
                    "slam_map_topic": LaunchConfiguration("slam_map_topic"),
                    "use_cmd_vel_bridge": "true",
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("slam_backend"), "' == 'slam_toolbox'"])
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(cartographer_launch)),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "with_move_group": "false",
                    "target_seed": LaunchConfiguration("target_seed"),
                    "target_entity_name": LaunchConfiguration("target_entity_name"),
                    "target_pose_frame": "world",
                    "slam_start_delay": "16.0",
                    "slam_map_topic": LaunchConfiguration("slam_map_topic"),
                    "cartographer_config_basename": LaunchConfiguration("cartographer_config_basename"),
                    "use_cmd_vel_bridge": "true",
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("slam_backend"), "' == 'cartographer'"])
                ),
            ),
            controllers_ready_gate,
            rtabmap_ready_gate,
            mapping_done_gate,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rtabmap_ready_gate,
                    on_exit=[
                        TimerAction(
                            period=LaunchConfiguration("mapping_runner_start_delay"),
                            actions=[mapping_route_runner],
                        ),
                        TimerAction(
                            period=LaunchConfiguration("validator_start_delay"),
                            actions=[slam_loop_validator],
                        ),
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=mapping_done_gate,
                    on_exit=[
                        TimerAction(
                            period=LaunchConfiguration("nav2_start_delay"),
                            actions=[
                                controller_server,
                                smoother_server,
                                planner_server,
                                behavior_server,
                                bt_navigator,
                                waypoint_follower,
                                velocity_smoother,
                                lifecycle_manager,
                            ],
                        ),
                    ],
                )
            ),
        ]
    )
