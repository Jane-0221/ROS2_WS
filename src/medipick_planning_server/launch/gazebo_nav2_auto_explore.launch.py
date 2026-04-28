from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    nav2_params_file = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "nav2_params.yaml"
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

    slam_map_ready_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_auto_explore_slam_map_ready_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "topic": LaunchConfiguration("slam_map_ready_topic"),
                "required_value": True,
                "log_period_sec": 5.0,
                "durability_transient_local": False,
            }
        ],
    )

    nav2_ready_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_auto_explore_nav2_ready_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "topic": LaunchConfiguration("nav2_ready_topic"),
                "required_value": True,
                "log_period_sec": 5.0,
                "durability_transient_local": False,
            }
        ],
    )

    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_nav2_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[configured_nav2_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_nav2_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_nav2_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_nav2_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[configured_nav2_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
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
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": False,
                    "bond_timeout": 20.0,
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
        ),
    ]

    nav2_activation_gate = Node(
        package=planning_server_package,
        executable="nav2_activation_gate.py",
        name="medipick_auto_explore_nav2_activation_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "mapping_done_topic": LaunchConfiguration("slam_map_ready_topic"),
                "lifecycle_service": "/lifecycle_manager_navigation/manage_nodes",
                "post_mapping_delay_sec": 2.0,
                "retry_period_sec": 1.0,
                "ready_topic": LaunchConfiguration("nav2_ready_topic"),
            }
        ],
    )

    explorer_node = Node(
        package=planning_server_package,
        executable="frontier_explorer.py",
        name="medipick_frontier_explorer",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "map_topic": "/rtabmap/grid_prob_map",
                "navigate_action_name": "/navigate_to_pose",
                "goal_frame": "map",
                "robot_base_frame": "base_link",
                "done_topic": "/medipick/slam_auto_explore_done",
                "map_ready_min_known_cells": LaunchConfiguration("frontier_map_ready_min_known_cells"),
                "require_robot_inside_map": False,
                "frontier_min_cluster_size": LaunchConfiguration("frontier_min_cluster_size"),
                "frontier_min_goal_distance": LaunchConfiguration("frontier_min_goal_distance"),
                "frontier_max_goal_distance": LaunchConfiguration("frontier_max_goal_distance"),
                "frontier_goal_standoff_distance": LaunchConfiguration("frontier_goal_standoff_distance"),
                "frontier_use_geodesic_distance": LaunchConfiguration("frontier_use_geodesic_distance"),
                "frontier_max_path_distance": LaunchConfiguration("frontier_max_path_distance"),
                "frontier_gain_weight": LaunchConfiguration("frontier_gain_weight"),
                "no_frontier_confirmations": LaunchConfiguration("frontier_done_confirmations"),
                "bootstrap_head_scan_before_map_ready": True,
                "bootstrap_head_scan_period_sec": 6.0,
                "enable_head_scan": True,
                "head_scan_h1_sequence_deg": [-25.0, 0.0, 25.0],
                "head_scan_h2_deg": -30.0,
                "head_scan_move_sec": 0.12,
                "head_scan_hold_sec": 0.05,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("slam_map_topic", default_value="/medipick/slam_map_latched"),
            DeclareLaunchArgument("slam_map_ready_topic", default_value="/medipick/slam_map_ready"),
            DeclareLaunchArgument("nav2_ready_topic", default_value="/medipick/nav2_ready"),
            DeclareLaunchArgument("nav2_params_file", default_value=str(nav2_params_file)),
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
            DeclareLaunchArgument("rtabmap_vis_min_inliers", default_value="20"),
            DeclareLaunchArgument("rtabmap_vis_min_inliers_distribution", default_value="0.0125"),
            DeclareLaunchArgument("rtabmap_kp_max_features", default_value="500"),
            DeclareLaunchArgument("rtabmap_kp_min_depth", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_kp_max_depth", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_vis_cor_nndr", default_value="0.8"),
            DeclareLaunchArgument("rtabmap_vis_depth_mask_floor_thr", default_value="0.0"),
            DeclareLaunchArgument("rtabmap_reg_strategy", default_value="0"),
            DeclareLaunchArgument("rtabmap_optimizer_robust", default_value="false"),
            DeclareLaunchArgument("rtabmap_mem_bad_signatures_ignored", default_value="false"),
            DeclareLaunchArgument("rtabmap_optimize_max_error", default_value="3.0"),
            DeclareLaunchArgument("rtabmap_subscribe_scan", default_value="false"),
            DeclareLaunchArgument("rtabmap_extra_args", default_value=""),
            DeclareLaunchArgument("nav2_start_delay", default_value="2.0"),
            DeclareLaunchArgument("explorer_start_delay", default_value="6.0"),
            DeclareLaunchArgument("frontier_min_cluster_size", default_value="10"),
            DeclareLaunchArgument("frontier_min_goal_distance", default_value="0.55"),
            DeclareLaunchArgument("frontier_max_goal_distance", default_value="0.0"),
            DeclareLaunchArgument("frontier_goal_standoff_distance", default_value="0.0"),
            DeclareLaunchArgument("frontier_use_geodesic_distance", default_value="true"),
            DeclareLaunchArgument("frontier_max_path_distance", default_value="4.0"),
            DeclareLaunchArgument("frontier_gain_weight", default_value="0.20"),
            DeclareLaunchArgument("frontier_done_confirmations", default_value="6"),
            DeclareLaunchArgument("frontier_map_ready_min_known_cells", default_value="20"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rtabmap_launch)),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                    "rviz": LaunchConfiguration("rviz"),
                    "slam_map_ready_topic": LaunchConfiguration("slam_map_ready_topic"),
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
                    "sim_base_mode": "omni_driver",
                    "use_cmd_vel_bridge": "true",
                    "controllers_ready_after": "base_controller",
                    "publish_target_box": "false",
                    "spawn_mobile_arm_controller": "false",
                    "spawn_head_controller": "true",
                    "spawn_tool_controller": "false",
                }.items(),
            ),
            slam_map_ready_gate,
            nav2_ready_gate,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=slam_map_ready_gate,
                    on_exit=[
                        TimerAction(
                            period=LaunchConfiguration("nav2_start_delay"),
                            actions=nav2_nodes + [nav2_activation_gate],
                        ),
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=nav2_ready_gate,
                    on_exit=[
                        TimerAction(
                            period=LaunchConfiguration("explorer_start_delay"),
                            actions=[explorer_node],
                        ),
                    ],
                )
            ),
        ]
    )
