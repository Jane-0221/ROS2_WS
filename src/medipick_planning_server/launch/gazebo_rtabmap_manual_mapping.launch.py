from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planning_server_package = "medipick_planning_server"
    description_package = "medipick_simple3_description"

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

    controllers_ready_gate = Node(
        package=planning_server_package,
        executable="bool_topic_gate.py",
        name="medipick_manual_mapping_controllers_ready_gate",
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

    posture_keeper = Node(
        package=planning_server_package,
        executable="manual_mapping_posture_keeper.py",
        name="medipick_manual_mapping_posture_keeper",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "lift_height": 0.302,
                "hold_head": LaunchConfiguration("hold_head"),
                "head_positions": [0.0, -0.5236],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("depth_to_scan", default_value="false"),
            DeclareLaunchArgument("rtabmap_force_3dof", default_value="true"),
            DeclareLaunchArgument("rtabmap_detection_rate", default_value="6.0"),
            DeclareLaunchArgument("rtabmap_linear_update", default_value="0.08"),
            DeclareLaunchArgument("rtabmap_angular_update", default_value="0.08"),
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
            DeclareLaunchArgument("controllers_ready_topic", default_value="/medipick/controllers_ready"),
            DeclareLaunchArgument("posture_keeper_start_delay", default_value="0.5"),
            DeclareLaunchArgument("hold_head", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rtabmap_launch)),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
                    "rviz": LaunchConfiguration("rviz"),
                    "depth_to_scan": LaunchConfiguration("depth_to_scan"),
                    "rtabmap_force_3dof": LaunchConfiguration("rtabmap_force_3dof"),
                    "rtabmap_detection_rate": LaunchConfiguration("rtabmap_detection_rate"),
                    "rtabmap_linear_update": LaunchConfiguration("rtabmap_linear_update"),
                    "rtabmap_angular_update": LaunchConfiguration("rtabmap_angular_update"),
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
                    "controllers_ready_topic": LaunchConfiguration("controllers_ready_topic"),
                    "sim_base_mode": "omni_driver",
                    "use_cmd_vel_bridge": "true",
                }.items(),
            ),
            controllers_ready_gate,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=controllers_ready_gate,
                    on_exit=[
                        TimerAction(
                            period=LaunchConfiguration("posture_keeper_start_delay"),
                            actions=[posture_keeper],
                        )
                    ],
                )
            ),
        ]
    )
