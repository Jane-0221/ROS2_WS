from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_file(package_name: str, relative_path: str) -> str:
    package_path = Path(get_package_share_directory(package_name))
    return (package_path / relative_path).read_text(encoding="utf-8")


def load_yaml(package_name: str, relative_path: str) -> dict:
    package_path = Path(get_package_share_directory(package_name))
    with open(package_path / relative_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def _set_nested(mapping, path, value):
    current = mapping
    for key in path[:-1]:
        current = current.setdefault(key, {})
    current[path[-1]] = value


def _build_saved_map_nav2_params_file(source_path: Path) -> Path:
    with source_path.open("r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream)

    updates = {
        ("bt_navigator", "ros__parameters", "bt_loop_duration"): 200,
        ("bt_navigator", "ros__parameters", "default_server_timeout"): 1000,
        (
            "bt_navigator",
            "ros__parameters",
            "default_nav_to_pose_bt_xml",
        ): "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_only_if_path_becomes_invalid.xml",
        ("controller_server", "ros__parameters", "controller_frequency"): 12.0,
        ("controller_server", "ros__parameters", "general_goal_checker", "stateful"): False,
        ("controller_server", "ros__parameters", "general_goal_checker", "xy_goal_tolerance"): 0.18,
        ("controller_server", "ros__parameters", "general_goal_checker", "yaw_goal_tolerance"): 3.14,
        ("controller_server", "ros__parameters", "progress_checker", "required_movement_radius"): 0.03,
        ("controller_server", "ros__parameters", "progress_checker", "movement_time_allowance"): 120.0,
        ("controller_server", "ros__parameters", "FollowPath", "xy_goal_tolerance"): 0.18,
        ("controller_server", "ros__parameters", "FollowPath", "RotateToGoal.scale"): 4.0,
        ("controller_server", "ros__parameters", "FollowPath", "vx_samples"): 8,
        ("controller_server", "ros__parameters", "FollowPath", "vy_samples"): 8,
        ("controller_server", "ros__parameters", "FollowPath", "vtheta_samples"): 10,
        ("controller_server", "ros__parameters", "FollowPath", "sim_time"): 0.7,
        ("controller_server", "ros__parameters", "FollowPath", "max_vel_x"): 1.20,
        ("controller_server", "ros__parameters", "FollowPath", "max_vel_y"): 1.20,
        ("controller_server", "ros__parameters", "FollowPath", "max_speed_xy"): 1.30,
        ("local_costmap", "local_costmap", "ros__parameters", "update_frequency"): 8.0,
        ("local_costmap", "local_costmap", "ros__parameters", "publish_frequency"): 4.0,
        ("global_costmap", "global_costmap", "ros__parameters", "update_frequency"): 2.0,
        ("global_costmap", "global_costmap", "ros__parameters", "static_layer", "map_topic"): "/map",
        (
            "global_costmap",
            "global_costmap",
            "ros__parameters",
            "static_layer",
            "map_subscribe_transient_local",
        ): True,
        ("planner_server", "ros__parameters", "expected_planner_frequency"): 2.0,
        ("waypoint_follower", "ros__parameters", "loop_rate"): 20,
        ("velocity_smoother", "ros__parameters", "smoothing_frequency"): 20.0,
        ("velocity_smoother", "ros__parameters", "max_velocity"): [1.20, 1.20, 1.20],
        ("velocity_smoother", "ros__parameters", "min_velocity"): [-0.80, -0.80, -1.20],
        ("velocity_smoother", "ros__parameters", "max_accel"): [3.0, 3.0, 3.0],
        ("velocity_smoother", "ros__parameters", "max_decel"): [-3.0, -3.0, -3.0],
    }
    for path, value in updates.items():
        _set_nested(params, path, value)

    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix="medipick_saved_map_pick_nav2_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    ) as stream:
        yaml.safe_dump(params, stream, sort_keys=False)
        return Path(stream.name)


def generate_launch_description():
    description_package = "medipick_simple3_description"
    moveit_package = "medipick_moveit_config"
    planning_server_package = "medipick_planning_server"

    default_world = (
        Path(get_package_share_directory(description_package))
        / "worlds"
        / "medipick_pharmacy_textured.world.sdf"
    )
    default_saved_map = Path("/home/lzoolloozl/work/medipick/map/medipick_gazebo_rtabmap.yaml")
    default_medicine_memory_db = Path.home() / ".local" / "share" / "medipick" / "medicine_memory" / "medicine_memory.sqlite3"
    gazebo_trial_demo_launch = (
        Path(get_package_share_directory(moveit_package))
        / "launch"
        / "gazebo_trial_demo.launch.py"
    )
    nav2_params_file = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "nav2_params.yaml"
    )
    planning_server_config = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "planning_server.yaml"
    )

    configured_nav2_params = _build_saved_map_nav2_params_file(nav2_params_file)

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

    controllers_ready_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_saved_map_pick_controllers_ready_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "topic": "/medipick/controllers_ready",
                "required_value": True,
                "log_period_sec": 5.0,
            }
        ],
    )

    runtime_actions = [
        Node(
            package=planning_server_package,
            executable="gazebo_omni_base_driver.py",
            name="medipick_gazebo_omni_base_driver",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "joint_state_topic": "/medipick/joint_states_sim",
                    "trajectory_topic": "/base_controller/joint_trajectory",
                    "odom_topic": "/odom",
                    "amcl_pose_topic": "/amcl_pose",
                    "control_rate_hz": 30.0,
                    "publish_rate_hz": 60.0,
                    "lookahead_time": 0.18,
                    "command_timeout": 0.30,
                    "max_linear_speed": 1.20,
                    "max_lateral_speed": 1.20,
                    "max_angular_speed": 1.20,
                    "trajectory_linear_gain": 6.0,
                    "trajectory_lateral_gain": 6.0,
                    "trajectory_angular_gain": 4.0,
                    "max_target_linear_speed": 2.0,
                    "max_target_lateral_speed": 2.0,
                    "max_target_angular_speed": 2.5,
                    "publish_identity_tf": True,
                    "publish_map_to_odom_tf": True,
                    "publish_odom_to_world_tf": True,
                    "publish_amcl_pose": True,
                    "odom_stamp_backoff_sec": 0.12,
                }
            ],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[str(configured_nav2_params)],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[str(configured_nav2_params)],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[str(configured_nav2_params)],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[str(configured_nav2_params)],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[str(configured_nav2_params)],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[str(configured_nav2_params)],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[str(configured_nav2_params)],
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
        ),
        TimerAction(
            period=4.0,
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
                            "insert_local_position_threshold": 0.40,
                            "insert_local_axial_threshold": 0.32,
                            "insert_local_lateral_threshold": 0.25,
                            "insert_local_orientation_threshold": 0.25,
                            "prepare_head_positions": [0.0, -0.5236],
                            "prepare_head_motion_duration": 1.6,
                            "prepare_head_max_step_rad": 0.20,
                            "enable_pick_head_tracking": False,
                            "pick_head_tracking_stages": [
                                "lift_to_band",
                                "select_pre_insert",
                                "plan_to_pre_insert",
                                "insert_and_suction",
                            ],
                            "pick_head_tracking_h1_offset_rad": 0.0,
                            "pick_head_tracking_h2_offset_rad": -0.5236,
                            "pick_head_tracking_h1_min_rad": -1.05,
                            "pick_head_tracking_h1_max_rad": 1.05,
                            "pick_head_tracking_h2_min_rad": -1.10,
                            "pick_head_tracking_h2_max_rad": -0.05,
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
                            "trajectory_execution_timeout_margin": 60.0,
                            "controller_goal_retry_count": 4,
                            "controller_retry_delay": 2.0,
                            "controller_settle_correction_passes": 3,
                            "controller_settle_correction_min_duration": 1.0,
                            "controller_settle_correction_max_duration": 3.0,
                            "pre_insert_segment_max_base_translation_m": 0.10,
                            "pre_insert_segment_max_base_rotation_deg": 10.0,
                            "pre_insert_segment_max_arm_joint_motion": 0.50,
                            "pre_insert_segment_min_duration": 0.8,
                            "pre_insert_segment_max_duration": 1.8,
                            "pre_insert_segment_step_count": 4,
                            "insert_segment_max_base_translation_m": 0.08,
                            "insert_segment_max_base_rotation_deg": 8.0,
                            "insert_segment_max_arm_joint_motion": 0.45,
                            "insert_segment_min_duration": 0.8,
                            "insert_segment_max_duration": 1.6,
                            "insert_segment_step_count": 4,
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
                            "map_topic": "/map",
                            "navigate_action_name": "/navigate_to_pose",
                            "task_start_service": "/medipick/task/start",
                            "base_offset": LaunchConfiguration("trial_base_offset"),
                            "base_lateral_offset": LaunchConfiguration("trial_base_lateral_offset"),
                            "base_yaw_offset_deg": LaunchConfiguration("trial_base_yaw_offset_deg"),
                            "navigation_route_mode": "direct",
                            "navigation_corridor_y": 0.0,
                            "waypoint_position_tolerance": 0.16,
                            "max_stage_travel_distance_m": 0.0,
                            "task_start_delay_sec": LaunchConfiguration("task_start_delay_sec"),
                            "min_known_map_cells": 10,
                            "retry_delay_sec": 5.0,
                            "max_retry_count": 8,
                            "stage_goal_completion_distance_threshold_m": 0.06,
                            "coarse_success_distance_threshold_m": 0.22,
                            "final_orientation_distance_threshold_m": 100.0,
                            "enable_head_posture_control": False,
                            "enable_nav_arm_stow": False,
                            "head_motion_duration_sec": 0.5,
                            "arm_motion_duration_sec": 0.7,
                        }
                    ],
                ),
                Node(
                    package=planning_server_package,
                    executable="gazebo_box_target_publisher.py",
                    name="medipick_gazebo_box_target_publisher",
                    output="screen",
                    condition=UnlessCondition(LaunchConfiguration("use_medicine_memory_target")),
                    parameters=[
                        {
                            "use_sim_time": True,
                            "world_file": LaunchConfiguration("world_file"),
                            "target_seed": LaunchConfiguration("target_seed"),
                            "target_entity_name": LaunchConfiguration("target_entity_name"),
                            "start_delay": 0.5,
                            "frame_id": "map",
                            "world_frame": "world",
                            "camera_frame": "cam_link",
                            "publish_world_pose": True,
                            "publish_camera_pose": True,
                        }
                    ],
                ),
                Node(
                    package=planning_server_package,
                    executable="medicine_memory_target_publisher.py",
                    name="medipick_medicine_memory_target_publisher",
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("use_medicine_memory_target")),
                    parameters=[
                        {
                            "db_path": LaunchConfiguration("medicine_memory_db_path"),
                            "medicine_id": LaunchConfiguration("medicine_id"),
                            "location_key": LaunchConfiguration("medicine_location_key"),
                            "target_pose_topic": "/medipick/task/target_pose",
                            "target_name_topic": "/medipick/gazebo_target_box/name",
                            "target_shelf_info_topic": "/medipick/task/target_shelf_info",
                            "publish_period_sec": 0.5,
                        }
                    ],
                ),
                TimerAction(
                    period=30.0,
                    actions=[
                        Node(
                            package=planning_server_package,
                            executable="post_pick_delivery_manager.py",
                            name="medipick_post_pick_delivery_manager",
                            output="screen",
                            condition=IfCondition(LaunchConfiguration("post_pick_delivery_enabled")),
                            parameters=[
                                {
                                    "delivery_goal_frame": "map",
                                    "delivery_goal_x": LaunchConfiguration("delivery_goal_x"),
                                    "delivery_goal_y": LaunchConfiguration("delivery_goal_y"),
                                    "delivery_goal_yaw_deg": LaunchConfiguration("delivery_goal_yaw_deg"),
                                    "release_joint_position": LaunchConfiguration("delivery_release_joint_position"),
                                    "release_motion_duration_sec": LaunchConfiguration("delivery_release_motion_duration_sec"),
                                }
                            ],
                        ),
                    ],
                ),
            ],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("map_yaml_file", default_value=str(default_saved_map)),
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("camera_view", default_value="false"),
            DeclareLaunchArgument("camera_view_topic", default_value="/medipick/depth_camera/image"),
            DeclareLaunchArgument("with_move_group", default_value="true"),
            DeclareLaunchArgument("target_seed", default_value="0"),
            DeclareLaunchArgument("target_entity_name", default_value=""),
            DeclareLaunchArgument("use_medicine_memory_target", default_value="false"),
            DeclareLaunchArgument("medicine_memory_db_path", default_value=str(default_medicine_memory_db)),
            DeclareLaunchArgument("medicine_id", default_value=""),
            DeclareLaunchArgument("medicine_location_key", default_value=""),
            DeclareLaunchArgument("trial_base_offset", default_value="0.82"),
            DeclareLaunchArgument("trial_base_lateral_offset", default_value="0.12"),
            DeclareLaunchArgument("trial_base_yaw_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("task_start_delay_sec", default_value="1.5"),
            DeclareLaunchArgument("post_pick_delivery_enabled", default_value="false"),
            DeclareLaunchArgument("delivery_goal_x", default_value="2.15"),
            DeclareLaunchArgument("delivery_goal_y", default_value="0.0"),
            DeclareLaunchArgument("delivery_goal_yaw_deg", default_value="180.0"),
            DeclareLaunchArgument("delivery_release_joint_position", default_value="0.0"),
            DeclareLaunchArgument("delivery_release_motion_duration_sec", default_value="1.0"),
            DeclareLaunchArgument("initial_base_x", default_value="2.15"),
            DeclareLaunchArgument("initial_base_y", default_value="0.0"),
            DeclareLaunchArgument("initial_base_theta", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gazebo_trial_demo_launch)),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "rviz": LaunchConfiguration("rviz"),
                    "with_move_group": LaunchConfiguration("with_move_group"),
                    "bridge_pointcloud": "false",
                    "initial_base_x": LaunchConfiguration("initial_base_x"),
                    "initial_base_y": LaunchConfiguration("initial_base_y"),
                    "initial_base_theta": LaunchConfiguration("initial_base_theta"),
                }.items(),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "yaml_filename": LaunchConfiguration("map_yaml_file"),
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            Node(
                package="image_tools",
                executable="showimage",
                name="medipick_pick_camera_view",
                output="screen",
                condition=IfCondition(LaunchConfiguration("camera_view")),
                remappings=[
                    ("image", LaunchConfiguration("camera_view_topic")),
                ],
            ),
            controllers_ready_gate,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controllers_ready_gate,
                    on_exit=runtime_actions,
                )
            ),
        ]
    )
