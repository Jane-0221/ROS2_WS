from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("auto_start_on_target", default_value="false"),
            DeclareLaunchArgument("auto_accept_base_arrival", default_value="false"),
            DeclareLaunchArgument("prepare_group_name", default_value="arm"),
            DeclareLaunchArgument("pre_insert_group_name", default_value="mobile_arm"),
            DeclareLaunchArgument("candidate_planning_time", default_value="1.0"),
            DeclareLaunchArgument("candidate_num_planning_attempts", default_value="1"),
            DeclareLaunchArgument("candidate_retry_on_failure", default_value="true"),
            DeclareLaunchArgument("candidate_retry_planning_time", default_value="3.0"),
            DeclareLaunchArgument("candidate_retry_num_planning_attempts", default_value="2"),
            DeclareLaunchArgument("lift_use_end_effector_height_alignment", default_value="true"),
            DeclareLaunchArgument("lift_end_effector_target_offset", default_value="0.0"),
            DeclareLaunchArgument("final_group_name", default_value="mobile_arm"),
            DeclareLaunchArgument("insert_group_name", default_value="arm"),
            DeclareLaunchArgument("retreat_group_name", default_value="mobile_arm"),
            DeclareLaunchArgument("safe_retreat_group_name", default_value="arm"),
            Node(
                package="medipick_planning_server",
                executable="pick_task_manager.py",
                output="screen",
                parameters=[
                    {
                        "auto_start_on_target": LaunchConfiguration("auto_start_on_target"),
                        "auto_accept_base_arrival": LaunchConfiguration("auto_accept_base_arrival"),
                        "prepare_group_name": LaunchConfiguration("prepare_group_name"),
                        "pre_insert_group_name": LaunchConfiguration("pre_insert_group_name"),
                        "candidate_planning_time": LaunchConfiguration("candidate_planning_time"),
                        "candidate_num_planning_attempts": LaunchConfiguration("candidate_num_planning_attempts"),
                        "candidate_retry_on_failure": LaunchConfiguration("candidate_retry_on_failure"),
                        "candidate_retry_planning_time": LaunchConfiguration("candidate_retry_planning_time"),
                        "candidate_retry_num_planning_attempts": LaunchConfiguration(
                            "candidate_retry_num_planning_attempts"
                        ),
                        "lift_use_end_effector_height_alignment": LaunchConfiguration(
                            "lift_use_end_effector_height_alignment"
                        ),
                        "lift_end_effector_target_offset": LaunchConfiguration(
                            "lift_end_effector_target_offset"
                        ),
                        "final_group_name": LaunchConfiguration("final_group_name"),
                        "insert_group_name": LaunchConfiguration("insert_group_name"),
                        "retreat_group_name": LaunchConfiguration("retreat_group_name"),
                        "safe_retreat_group_name": LaunchConfiguration("safe_retreat_group_name"),
                    }
                ],
            ),
        ]
    )
