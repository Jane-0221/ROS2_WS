from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def load_file(package_name: str, relative_path: str) -> str:
    package_path = Path(get_package_share_directory(package_name))
    return (package_path / relative_path).read_text(encoding="utf-8")


def load_yaml(package_name: str, relative_path: str) -> dict:
    package_path = Path(get_package_share_directory(package_name))
    with open(package_path / relative_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    moveit_package = "medipick_moveit_config"
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
    planning_server_config = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "planning_server.yaml"
    )
    nav2_params_file = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "nav2_params.yaml"
    )

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
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load_yaml(moveit_package, "config/ompl_planning.yaml"),
    }
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("slam_backend", default_value="rtabmap"),
            DeclareLaunchArgument("slam_map_topic", default_value="/medipick/slam_map_latched"),
            DeclareLaunchArgument("target_seed", default_value="0"),
            DeclareLaunchArgument("target_entity_name", default_value=""),
            DeclareLaunchArgument("trial_base_offset", default_value="0.68"),
            DeclareLaunchArgument("trial_base_lateral_offset", default_value="0.18"),
            DeclareLaunchArgument("trial_base_yaw_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("nav2_params_file", default_value=str(nav2_params_file)),
            DeclareLaunchArgument(
                "slam_toolbox_params_file",
                default_value=str(
                    Path(get_package_share_directory(planning_server_package))
                    / "config"
                    / "slam_toolbox_mapper_params_gazebo.yaml"
                ),
            ),
            DeclareLaunchArgument("rtabmap_gui_cfg", default_value="~/.ros/rtabmapGUI.ini"),
            DeclareLaunchArgument("rtabmap_use_visual_odometry", default_value="false"),
            DeclareLaunchArgument("rtabmap_vo_frame_id", default_value="odom"),
            DeclareLaunchArgument("rtabmap_subscribe_scan", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/medipick/imu"),
            DeclareLaunchArgument("rtabmap_wait_imu_to_init", default_value="false"),
            DeclareLaunchArgument("rtabmap_force_3dof", default_value="true"),
            DeclareLaunchArgument("rtabmap_grid_3d", default_value="false"),
            DeclareLaunchArgument("rtabmap_extra_args", default_value=""),
            DeclareLaunchArgument("nav2_start_delay", default_value="28.0"),
            DeclareLaunchArgument("planning_stack_start_delay", default_value="30.0"),
            DeclareLaunchArgument("task_start_delay_sec", default_value="1.5"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rtabmap_launch)),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                    "rviz": LaunchConfiguration("rviz"),
                    "with_move_group": "true",
                    "target_seed": LaunchConfiguration("target_seed"),
                    "target_entity_name": LaunchConfiguration("target_entity_name"),
                    "target_pose_frame": "map",
                    "rtabmap_gui_cfg": LaunchConfiguration("rtabmap_gui_cfg"),
                    "rtabmap_use_visual_odometry": LaunchConfiguration("rtabmap_use_visual_odometry"),
                    "rtabmap_vo_frame_id": LaunchConfiguration("rtabmap_vo_frame_id"),
                    "rtabmap_subscribe_scan": LaunchConfiguration("rtabmap_subscribe_scan"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "rtabmap_wait_imu_to_init": LaunchConfiguration("rtabmap_wait_imu_to_init"),
                    "rtabmap_force_3dof": LaunchConfiguration("rtabmap_force_3dof"),
                    "rtabmap_grid_3d": LaunchConfiguration("rtabmap_grid_3d"),
                    "rtabmap_extra_args": LaunchConfiguration("rtabmap_extra_args"),
                    "sim_base_mode": "omni_driver",
                    "use_cmd_vel_bridge": "true",
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
                    "with_move_group": "true",
                    "target_seed": LaunchConfiguration("target_seed"),
                    "target_entity_name": LaunchConfiguration("target_entity_name"),
                    "target_pose_frame": "map",
                    "slam_start_delay": "16.0",
                    "slam_toolbox_params_file": LaunchConfiguration("slam_toolbox_params_file"),
                    "slam_map_topic": LaunchConfiguration("slam_map_topic"),
                    "use_cmd_vel_bridge": "true",
                }.items(),
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("slam_backend"), "' == 'slam_toolbox'"])
                ),
            ),
            TimerAction(
                period=LaunchConfiguration("nav2_start_delay"),
                actions=[
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
                ],
            ),
            TimerAction(
                period=PythonExpression([LaunchConfiguration("nav2_start_delay"), " + 2.0"]),
                actions=[
                    Node(
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
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("planning_stack_start_delay"),
                actions=[
                    Node(
                        package=planning_server_package,
                        executable="planning_server.py",
                        name="medipick_planning_server",
                        output="screen",
                        parameters=[
                            str(planning_server_config),
                            {"backend": "auto", "use_sim_time": True},
                            robot_description,
                            robot_description_semantic,
                            robot_description_kinematics,
                            robot_description_planning,
                            ompl_planning_pipeline_config,
                        ],
                    ),
                    Node(
                        package=planning_server_package,
                        executable="pick_task_manager.py",
                        name="medipick_pick_task_manager",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "frame_id": "world",
                                "manipulation_target_frame_id": "odom",
                                "manipulation_target_tracking_enabled": True,
                                "manipulation_target_tracking_position_alpha": 0.35,
                                "manipulation_target_tracking_orientation_alpha": 0.25,
                                "manipulation_target_tracking_max_hold_sec": 4.0,
                                "freeze_manipulation_target_on_pre_insert": True,
                                "prefer_latest_target_on_manipulation_relock": True,
                                "auto_start_on_target": False,
                                "require_initialization_ready": False,
                                "initialization_ready_topic": "",
                                "auto_accept_base_arrival": True,
                                "base_lateral_offset": 0.0,
                                "base_yaw_offset_deg": 0.0,
                                "acquire_target_stable_age": 0.0,
                                "stage_timeout": 90.0,
                                "pre_insert_group_name": "mobile_arm",
                                "insert_group_name": "arm",
                                "insert_fallback_group_name": "mobile_arm",
                                "safe_retreat_group_name": "arm",
                                "pre_insert_try_arm_first": False,
                                "pre_insert_select_pose_only": True,
                                "pre_insert_candidate_count": 14,
                                "candidate_max_evaluations": 14,
                                "candidate_shortlist_size": 8,
                                "allowed_planning_time": 8.0,
                                "pre_insert_base_translation_limit_m": 0.90,
                                "pre_insert_base_rotation_limit_deg": 45.0,
                                "insert_base_translation_limit_m": 0.45,
                                "insert_base_rotation_limit_deg": 35.0,
                                "candidate_planning_time": 2.0,
                                "candidate_num_planning_attempts": 1,
                                "candidate_retry_on_failure": False,
                                "candidate_retry_planning_time": 5.0,
                                "candidate_retry_num_planning_attempts": 3,
                                "max_velocity_scaling": 0.60,
                                "max_acceleration_scaling": 0.60,
                                "pre_insert_offset": 0.08,
                                "pre_insert_offset_min": 0.07,
                                "pre_insert_offset_max": 0.14,
                                "prepare_projection_tolerance": 0.06,
                                "lift_use_end_effector_height_alignment": True,
                                "lift_end_effector_target_offset": 0.0,
                                "lift_band_half_width": 0.06,
                                "lift_arrival_below_band_tolerance": 0.01,
                                "lift_probe_insert_before_arrival": True,
                                "lift_probe_insert_delay_sec": 6.0,
                                "lift_allow_pre_insert_without_arrival": True,
                                "lift_pre_insert_probe_delay_sec": 10.0,
                                "direct_final_try_before_prepare": False,
                                "final_planner_id": "RRTConnectkConfigDefault",
                                "final_prefer_seeded_ik": True,
                                "final_motion_duration": 2.0,
                                "final_cartesian_min_fraction": 0.85,
                                "retreat_cartesian_min_fraction": 0.85,
                                "mock_lift_motion_duration": 3.5,
                                "stow_motion_duration": 1.2,
                                "trajectory_execution_timeout_margin": 30.0,
                                "controller_goal_retry_count": 4,
                                "controller_retry_delay": 2.0,
                                "controller_settle_correction_passes": 3,
                                "controller_settle_correction_min_duration": 1.0,
                                "controller_settle_correction_max_duration": 3.0,
                                "allow_return_to_stow_failure_after_retreat": True,
                                "base_controller_action_name": "/base_controller/follow_joint_trajectory",
                                "controller_joint_names": [
                                    "r1_joint",
                                    "r2_joint",
                                    "r3_joint",
                                    "r4_joint",
                                    "r5_joint",
                                    "r6_joint",
                                ],
                                "base_controller_joint_names": [
                                    "base_x",
                                    "base_y",
                                    "base_theta",
                                ],
                                "discrete_lift_mode": True,
                                "discrete_arm_group_name": "arm_no_lift",
                                "stow_preserve_raise_joint": True,
                                "lift_segmented_execution_enabled": False,
                                "lift_segment_step_m": 0.06,
                                "lift_segment_min_duration": 0.8,
                                "lift_segment_speed_mps": 0.10,
                            }
                        ],
                    ),
                    Node(
                        package=planning_server_package,
                        executable="nav2_target_navigator.py",
                        name="medipick_nav2_target_navigator",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "target_pose_topic": "/medipick/task/target_pose",
                                "map_topic": LaunchConfiguration("slam_map_topic"),
                                "navigate_action_name": "/navigate_to_pose",
                                "task_start_service": "/medipick/task/start",
                                "base_offset": LaunchConfiguration("trial_base_offset"),
                                "base_lateral_offset": LaunchConfiguration("trial_base_lateral_offset"),
                                "base_yaw_offset_deg": LaunchConfiguration("trial_base_yaw_offset_deg"),
                                "navigation_route_mode": "central_corridor",
                                "navigation_corridor_y": 0.0,
                                "waypoint_position_tolerance": 0.16,
                                "max_stage_travel_distance_m": 0.45,
                                "task_start_delay_sec": LaunchConfiguration("task_start_delay_sec"),
                                "min_known_map_cells": 10,
                                "retry_delay_sec": 5.0,
                                "max_retry_count": 8,
                                "stage_goal_completion_distance_threshold_m": 0.12,
                                "coarse_success_distance_threshold_m": 0.12,
                                "head_motion_duration_sec": 0.5,
                                "arm_motion_duration_sec": 0.7,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
