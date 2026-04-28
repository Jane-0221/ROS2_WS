from pathlib import Path
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        ("controller_server", "ros__parameters", "controller_frequency"): 6.0,
        ("controller_server", "ros__parameters", "general_goal_checker", "stateful"): False,
        ("controller_server", "ros__parameters", "general_goal_checker", "xy_goal_tolerance"): 0.18,
        ("controller_server", "ros__parameters", "general_goal_checker", "yaw_goal_tolerance"): 3.14,
        ("controller_server", "ros__parameters", "progress_checker", "required_movement_radius"): 0.03,
        ("controller_server", "ros__parameters", "progress_checker", "movement_time_allowance"): 120.0,
        ("controller_server", "ros__parameters", "FollowPath", "xy_goal_tolerance"): 0.18,
        ("controller_server", "ros__parameters", "FollowPath", "RotateToGoal.scale"): 4.0,
        ("controller_server", "ros__parameters", "FollowPath", "vx_samples"): 10,
        ("controller_server", "ros__parameters", "FollowPath", "vy_samples"): 10,
        ("controller_server", "ros__parameters", "FollowPath", "vtheta_samples"): 12,
        ("controller_server", "ros__parameters", "FollowPath", "sim_time"): 1.2,
        ("local_costmap", "local_costmap", "ros__parameters", "update_frequency"): 4.0,
        ("local_costmap", "local_costmap", "ros__parameters", "publish_frequency"): 1.0,
        ("local_costmap", "local_costmap", "ros__parameters", "plugins"): ["inflation_layer"],
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
        ("waypoint_follower", "ros__parameters", "loop_rate"): 10,
        ("velocity_smoother", "ros__parameters", "smoothing_frequency"): 10.0,
    }
    for path, value in updates.items():
        _set_nested(params, path, value)

    local_costmap_params = params["local_costmap"]["local_costmap"]["ros__parameters"]
    local_costmap_params.pop("obstacle_layer", None)

    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix="medipick_saved_map_nav2_",
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

    configured_nav2_params = _build_saved_map_nav2_params_file(nav2_params_file)

    controllers_ready_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_saved_map_validation_controllers_ready_gate",
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
        Node(
            package=planning_server_package,
            executable="slam_loop_validator.py",
            name="medipick_saved_map_nav_validator",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "map_topic": "/map",
                    "map_topic_transient_local": True,
                    "rtabmap_info_topic": "",
                    "navigate_action_name": "/navigate_to_pose",
                    "route_xs": [
                        2.15, 2.15, 2.15, 3.50, 4.85, 4.85, 4.85, 6.55, 6.55, 6.55, 4.85, 3.50, 2.15
                    ],
                    "route_ys": [
                        0.0, 3.05, 0.0, 0.0, 0.0, 3.05, 0.0, 0.0, 3.05, 0.0, 0.0, 0.0, 0.0
                    ],
                    "route_yaws_deg": [
                        0.0, 90.0, -90.0, 0.0, 0.0, 90.0, -90.0, 0.0, 90.0, -90.0, 180.0, 180.0, 180.0
                    ],
                    "waypoint_timeout_sec": 75.0,
                    "wait_after_map_ready_sec": 1.0,
                    "wait_after_action_server_ready_sec": 4.0,
                    "wait_after_each_goal_sec": 0.4,
                    "continue_on_failure": True,
                }
            ],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("map_yaml_file", default_value=str(default_saved_map)),
            DeclareLaunchArgument("gazebo_gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("with_move_group", default_value="false"),
            DeclareLaunchArgument("initial_base_x", default_value="2.15"),
            DeclareLaunchArgument("initial_base_y", default_value="0.0"),
            DeclareLaunchArgument("initial_base_theta", default_value="0.0"),
            DeclareLaunchArgument("nav2_params_file", default_value=str(nav2_params_file)),
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
            controllers_ready_gate,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controllers_ready_gate,
                    on_exit=runtime_actions,
                )
            ),
        ]
    )
