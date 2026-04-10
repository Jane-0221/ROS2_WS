from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


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
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    planning_server_config = str(
        Path(get_package_share_directory(planning_server_package)) / "config" / "planning_server.yaml"
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
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(load_yaml(moveit_package, "config/ompl_planning.yaml"))

    moveit_controllers = load_yaml(moveit_package, "config/moveit_controllers.yaml")
    sensors_3d = load_yaml(moveit_package, "config/sensors_3d.yaml")
    trajectory_execution = {
        "allow_trajectory_execution": ParameterValue(use_ros2_control, value_type=bool),
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    octomap_parameters = {
        "octomap_frame": "world",
        "octomap_resolution": 0.03,
        "max_range": 3.0,
    }
    sensor_name = sensors_3d["sensors"][0]
    sensor_config = sensors_3d[sensor_name]
    octomap_sensor_parameters = {
        "sensors": sensors_3d["sensors"],
        f"{sensor_name}.sensor_plugin": sensor_config["sensor_plugin"],
        f"{sensor_name}.point_cloud_topic": sensor_config["point_cloud_topic"],
        f"{sensor_name}.max_range": sensor_config["max_range"],
        f"{sensor_name}.point_subsample": sensor_config["point_subsample"],
        f"{sensor_name}.padding_offset": sensor_config["padding_offset"],
        f"{sensor_name}.padding_scale": sensor_config["padding_scale"],
        f"{sensor_name}.max_update_rate": sensor_config["max_update_rate"],
        f"{sensor_name}.filtered_cloud_topic": sensor_config["filtered_cloud_topic"],
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    rviz_config_file = str(
        Path(get_package_share_directory(planning_server_package)) / "rviz" / "moveit_mock_perception.rviz"
    )
    ros2_controllers_file = str(
        Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_ros2_control", default_value="true"),
            DeclareLaunchArgument("randomize_initial_pose", default_value="false"),
            DeclareLaunchArgument("initial_random_seed", default_value="-1"),
            DeclareLaunchArgument("initial_motion_duration", default_value="2.5"),
            DeclareLaunchArgument("random_seed", default_value="42"),
            DeclareLaunchArgument("shelf_levels", default_value="5"),
            DeclareLaunchArgument("shelf_center_x", default_value="0.88"),
            DeclareLaunchArgument("shelf_center_y", default_value="0.0"),
            DeclareLaunchArgument("shelf_depth", default_value="0.24"),
            DeclareLaunchArgument("shelf_width", default_value="0.72"),
            DeclareLaunchArgument("shelf_bottom_z", default_value="0.48"),
            DeclareLaunchArgument("shelf_level_gap", default_value="0.24"),
            DeclareLaunchArgument("shelf_board_thickness", default_value="0.03"),
            DeclareLaunchArgument("shelf_side_thickness", default_value="0.04"),
            DeclareLaunchArgument("shelf_back_thickness", default_value="0.03"),
            DeclareLaunchArgument("clutter_count", default_value="8"),
            DeclareLaunchArgument("point_spacing", default_value="0.025"),
            DeclareLaunchArgument("noise_amplitude", default_value="0.0025"),
            DeclareLaunchArgument("min_point_z", default_value="0.20"),
            DeclareLaunchArgument("publish_rate", default_value="1.0"),
            DeclareLaunchArgument("target_box_size_x", default_value="0.08"),
            DeclareLaunchArgument("target_box_size_y", default_value="0.12"),
            DeclareLaunchArgument("target_box_size_z", default_value="0.08"),
            DeclareLaunchArgument("target_level", default_value="-1"),
            DeclareLaunchArgument("target_gap_index", default_value="-1"),
            DeclareLaunchArgument("target_depth_ratio", default_value="0.05"),
            DeclareLaunchArgument("target_lateral_margin", default_value="0.10"),
            DeclareLaunchArgument("target_lateral_span", default_value="0.06"),
            DeclareLaunchArgument("publish_target_box_points", default_value="false"),
            DeclareLaunchArgument("publish_clutter_points", default_value="false"),
            DeclareLaunchArgument("publish_floor_points", default_value="false"),
            DeclareLaunchArgument("run_task_manager", default_value="true"),
            DeclareLaunchArgument("task_auto_start_on_target", default_value="true"),
            DeclareLaunchArgument("task_auto_accept_base_arrival", default_value="true"),
            DeclareLaunchArgument("use_simplified_pipeline", default_value="true"),
            DeclareLaunchArgument("discrete_lift_mode", default_value="false"),
            DeclareLaunchArgument("task_auto_accept_lift_arrival", default_value="true"),
            DeclareLaunchArgument("task_stage_timeout", default_value="60.0"),
            DeclareLaunchArgument("mock_lift_motion_duration", default_value="2.0"),
            DeclareLaunchArgument("base_standoff", default_value="0.60"),
            DeclareLaunchArgument("base_lateral_offset", default_value="0.18"),
            DeclareLaunchArgument("adaptive_workspace_enabled", default_value="true"),
            DeclareLaunchArgument("prepare_offset", default_value="0.06"),
            DeclareLaunchArgument("pre_insert_offset", default_value="0.06"),
            DeclareLaunchArgument("pre_insert_candidate_count", default_value="3"),
            DeclareLaunchArgument("pre_insert_first_candidate_only", default_value="false"),
            DeclareLaunchArgument("pre_insert_select_pose_only", default_value="true"),
            DeclareLaunchArgument("pre_insert_try_arm_first", default_value="true"),
            DeclareLaunchArgument("pre_insert_arm_group_name", default_value="arm"),
            DeclareLaunchArgument("pre_insert_limit_base_motion", default_value="true"),
            DeclareLaunchArgument("pre_insert_base_translation_limit_m", default_value="0.30"),
            DeclareLaunchArgument("pre_insert_base_rotation_limit_deg", default_value="25.0"),
            DeclareLaunchArgument("prepare_group_name", default_value="arm"),
            DeclareLaunchArgument("pre_insert_group_name", default_value="mobile_arm"),
            DeclareLaunchArgument("candidate_planner_id", default_value="RRTConnectkConfigDefault"),
            DeclareLaunchArgument("candidate_planning_time", default_value="1.0"),
            DeclareLaunchArgument("candidate_num_planning_attempts", default_value="1"),
            DeclareLaunchArgument("candidate_retry_on_failure", default_value="true"),
            DeclareLaunchArgument("candidate_retry_planning_time", default_value="3.0"),
            DeclareLaunchArgument("candidate_retry_num_planning_attempts", default_value="2"),
            DeclareLaunchArgument("direct_final_try_before_prepare", default_value="true"),
            DeclareLaunchArgument("allow_partial_direct_prepare", default_value="false"),
            DeclareLaunchArgument("use_entry_plane_prepare", default_value="true"),
            DeclareLaunchArgument("cabinet_entry_margin", default_value="0.05"),
            DeclareLaunchArgument("retreat_retrace_insert_first", default_value="true"),
            DeclareLaunchArgument("final_use_cartesian", default_value="false"),
            DeclareLaunchArgument("final_group_name", default_value="mobile_arm"),
            DeclareLaunchArgument("insert_group_name", default_value="arm"),
            DeclareLaunchArgument("insert_fallback_group_name", default_value="mobile_arm"),
            DeclareLaunchArgument("retreat_group_name", default_value="mobile_arm"),
            DeclareLaunchArgument("safe_retreat_group_name", default_value="arm"),
            DeclareLaunchArgument("final_planner_id", default_value=""),
            DeclareLaunchArgument("final_prefer_seeded_ik", default_value="false"),
            DeclareLaunchArgument("direct_prepare_min_fraction", default_value="0.75"),
            DeclareLaunchArgument("final_joint_interp_steps", default_value="10"),
            DeclareLaunchArgument("final_joint_motion_limit", default_value="1.4"),
            DeclareLaunchArgument("final_roll_motion_limit", default_value="1.2"),
            DeclareLaunchArgument("final_step_joint_motion_limit", default_value="0.45"),
            DeclareLaunchArgument("final_step_roll_motion_limit", default_value="0.35"),
            DeclareLaunchArgument("final_motion_duration", default_value="1.2"),
            DeclareLaunchArgument("insert_force_local_when_close", default_value="true"),
            DeclareLaunchArgument("insert_local_position_threshold", default_value="0.12"),
            DeclareLaunchArgument("insert_local_axial_threshold", default_value="0.12"),
            DeclareLaunchArgument("insert_local_lateral_threshold", default_value="0.03"),
            DeclareLaunchArgument("insert_local_orientation_threshold", default_value="0.20"),
            DeclareLaunchArgument("insert_limit_base_motion", default_value="true"),
            DeclareLaunchArgument("insert_base_translation_limit_m", default_value="0.07"),
            DeclareLaunchArgument("insert_base_rotation_limit_deg", default_value="8.0"),
            DeclareLaunchArgument("retreat_use_cartesian", default_value="false"),
            DeclareLaunchArgument("retreat_planner_id", default_value=""),
            DeclareLaunchArgument("lift_band_half_width", default_value="0.03"),
            DeclareLaunchArgument("lift_use_end_effector_height_alignment", default_value="true"),
            DeclareLaunchArgument("lift_end_effector_target_offset", default_value="0.0"),
            Node(
                package=moveit_package,
                executable="default_joint_state_publisher.py",
                name="default_joint_state_publisher",
                parameters=[{"publish_rate": 20.0}],
                condition=UnlessCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, ros2_controllers_file],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "mobile_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    ros2_controllers_file,
                ],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "tool_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    ros2_controllers_file,
                ],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "head_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "joint_trajectory_controller/JointTrajectoryController",
                    "--param-file",
                    ros2_controllers_file,
                ],
                condition=IfCondition(use_ros2_control),
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    octomap_parameters,
                    octomap_sensor_parameters,
                    planning_scene_monitor_parameters,
                ],
            ),
            Node(
                package=planning_server_package,
                executable="planning_server.py",
                name="medipick_planning_server",
                output="screen",
                parameters=[
                    planning_server_config,
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    {
                        "planning_pipelines": ["ompl"],
                        "default_planning_pipeline": "ompl",
                        "ompl": load_yaml(moveit_package, "config/ompl_planning.yaml"),
                    },
                ],
            ),
            Node(
                package=planning_server_package,
                executable="mock_vision_publisher.py",
                name="medipick_mock_vision_publisher",
                parameters=[
                    {
                        "publish_rate": ParameterValue(LaunchConfiguration("publish_rate"), value_type=float),
                        "random_seed": ParameterValue(LaunchConfiguration("random_seed"), value_type=int),
                        "shelf_levels": ParameterValue(LaunchConfiguration("shelf_levels"), value_type=int),
                        "shelf_center_x": ParameterValue(LaunchConfiguration("shelf_center_x"), value_type=float),
                        "shelf_center_y": ParameterValue(LaunchConfiguration("shelf_center_y"), value_type=float),
                        "shelf_depth": ParameterValue(LaunchConfiguration("shelf_depth"), value_type=float),
                        "shelf_width": ParameterValue(LaunchConfiguration("shelf_width"), value_type=float),
                        "shelf_bottom_z": ParameterValue(LaunchConfiguration("shelf_bottom_z"), value_type=float),
                        "shelf_level_gap": ParameterValue(LaunchConfiguration("shelf_level_gap"), value_type=float),
                        "shelf_board_thickness": ParameterValue(
                            LaunchConfiguration("shelf_board_thickness"), value_type=float
                        ),
                        "shelf_side_thickness": ParameterValue(
                            LaunchConfiguration("shelf_side_thickness"), value_type=float
                        ),
                        "shelf_back_thickness": ParameterValue(
                            LaunchConfiguration("shelf_back_thickness"), value_type=float
                        ),
                        "clutter_count": ParameterValue(LaunchConfiguration("clutter_count"), value_type=int),
                        "point_spacing": ParameterValue(LaunchConfiguration("point_spacing"), value_type=float),
                        "noise_amplitude": ParameterValue(LaunchConfiguration("noise_amplitude"), value_type=float),
                        "min_point_z": ParameterValue(LaunchConfiguration("min_point_z"), value_type=float),
                        "target_box_size_x": ParameterValue(
                            LaunchConfiguration("target_box_size_x"), value_type=float
                        ),
                        "target_box_size_y": ParameterValue(
                            LaunchConfiguration("target_box_size_y"), value_type=float
                        ),
                        "target_box_size_z": ParameterValue(
                            LaunchConfiguration("target_box_size_z"), value_type=float
                        ),
                        "target_level": ParameterValue(LaunchConfiguration("target_level"), value_type=int),
                        "target_gap_index": ParameterValue(LaunchConfiguration("target_gap_index"), value_type=int),
                        "target_depth_ratio": ParameterValue(LaunchConfiguration("target_depth_ratio"), value_type=float),
                        "target_lateral_margin": ParameterValue(
                            LaunchConfiguration("target_lateral_margin"), value_type=float
                        ),
                        "target_lateral_span": ParameterValue(
                            LaunchConfiguration("target_lateral_span"), value_type=float
                        ),
                        "publish_target_box_points": ParameterValue(
                            LaunchConfiguration("publish_target_box_points"), value_type=bool
                        ),
                        "publish_clutter_points": ParameterValue(
                            LaunchConfiguration("publish_clutter_points"), value_type=bool
                        ),
                        "publish_floor_points": ParameterValue(
                            LaunchConfiguration("publish_floor_points"), value_type=bool
                        ),
                        "task_target_pose_topic": "/medipick/task/target_pose",
                    }
                ],
                output="screen",
            ),
            Node(
                package=planning_server_package,
                executable="mock_initial_pose_manager.py",
                name="medipick_mock_initial_pose_manager",
                output="screen",
                condition=IfCondition(LaunchConfiguration("randomize_initial_pose")),
                parameters=[
                    {
                        "use_ros2_control": ParameterValue(use_ros2_control, value_type=bool),
                        "random_seed": ParameterValue(LaunchConfiguration("initial_random_seed"), value_type=int),
                        "initial_motion_duration": ParameterValue(
                            LaunchConfiguration("initial_motion_duration"), value_type=float
                        ),
                        "target_pose_topic": "/medipick/task/target_pose",
                        "task_start_service": "/medipick/task/start",
                        "auto_start_task": True,
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="pick_task_manager.py",
                name="medipick_pick_task_manager",
                output="screen",
                condition=IfCondition(LaunchConfiguration("run_task_manager")),
                parameters=[
                    {
                        "use_simplified_pipeline": ParameterValue(
                            LaunchConfiguration("use_simplified_pipeline"), value_type=bool
                        ),
                        "auto_start_on_target": ParameterValue(
                            LaunchConfiguration("task_auto_start_on_target"), value_type=bool
                        ),
                        "auto_accept_base_arrival": ParameterValue(
                            LaunchConfiguration("task_auto_accept_base_arrival"), value_type=bool
                        ),
                        "discrete_lift_mode": ParameterValue(
                            LaunchConfiguration("discrete_lift_mode"), value_type=bool
                        ),
                        "auto_accept_lift_arrival": ParameterValue(
                            LaunchConfiguration("task_auto_accept_lift_arrival"), value_type=bool
                        ),
                        "stage_timeout": ParameterValue(
                            LaunchConfiguration("task_stage_timeout"), value_type=float
                        ),
                        "mock_lift_motion_duration": ParameterValue(
                            LaunchConfiguration("mock_lift_motion_duration"), value_type=float
                        ),
                        "prepare_use_planning": False,
                        "prepare_offset": ParameterValue(
                            LaunchConfiguration("prepare_offset"), value_type=float
                        ),
                        "pre_insert_offset": ParameterValue(
                            LaunchConfiguration("pre_insert_offset"), value_type=float
                        ),
                        "shelf_center_x": ParameterValue(LaunchConfiguration("shelf_center_x"), value_type=float),
                        "shelf_center_y": ParameterValue(LaunchConfiguration("shelf_center_y"), value_type=float),
                        "shelf_depth": ParameterValue(LaunchConfiguration("shelf_depth"), value_type=float),
                        "adaptive_workspace_enabled": ParameterValue(
                            LaunchConfiguration("adaptive_workspace_enabled"), value_type=bool
                        ),
                        "candidate_planner_id": LaunchConfiguration("candidate_planner_id"),
                        "candidate_planning_time": ParameterValue(
                            LaunchConfiguration("candidate_planning_time"), value_type=float
                        ),
                        "candidate_num_planning_attempts": ParameterValue(
                            LaunchConfiguration("candidate_num_planning_attempts"), value_type=int
                        ),
                        "candidate_retry_on_failure": ParameterValue(
                            LaunchConfiguration("candidate_retry_on_failure"), value_type=bool
                        ),
                        "candidate_retry_planning_time": ParameterValue(
                            LaunchConfiguration("candidate_retry_planning_time"), value_type=float
                        ),
                        "candidate_retry_num_planning_attempts": ParameterValue(
                            LaunchConfiguration("candidate_retry_num_planning_attempts"), value_type=int
                        ),
                        "pre_insert_candidate_count": ParameterValue(
                            LaunchConfiguration("pre_insert_candidate_count"), value_type=int
                        ),
                        "pre_insert_first_candidate_only": ParameterValue(
                            LaunchConfiguration("pre_insert_first_candidate_only"), value_type=bool
                        ),
                        "pre_insert_select_pose_only": ParameterValue(
                            LaunchConfiguration("pre_insert_select_pose_only"), value_type=bool
                        ),
                        "pre_insert_try_arm_first": ParameterValue(
                            LaunchConfiguration("pre_insert_try_arm_first"), value_type=bool
                        ),
                        "pre_insert_arm_group_name": ParameterValue(
                            LaunchConfiguration("pre_insert_arm_group_name"), value_type=str
                        ),
                        "pre_insert_limit_base_motion": ParameterValue(
                            LaunchConfiguration("pre_insert_limit_base_motion"), value_type=bool
                        ),
                        "pre_insert_base_translation_limit_m": ParameterValue(
                            LaunchConfiguration("pre_insert_base_translation_limit_m"), value_type=float
                        ),
                        "pre_insert_base_rotation_limit_deg": ParameterValue(
                            LaunchConfiguration("pre_insert_base_rotation_limit_deg"), value_type=float
                        ),
                        "direct_final_try_before_prepare": ParameterValue(
                            LaunchConfiguration("direct_final_try_before_prepare"), value_type=bool
                        ),
                        "allow_partial_direct_prepare": ParameterValue(
                            LaunchConfiguration("allow_partial_direct_prepare"), value_type=bool
                        ),
                        "use_entry_plane_prepare": ParameterValue(
                            LaunchConfiguration("use_entry_plane_prepare"), value_type=bool
                        ),
                        "cabinet_entry_margin": ParameterValue(
                            LaunchConfiguration("cabinet_entry_margin"), value_type=float
                        ),
                        "retreat_retrace_insert_first": ParameterValue(
                            LaunchConfiguration("retreat_retrace_insert_first"), value_type=bool
                        ),
                        "final_use_cartesian": ParameterValue(
                            LaunchConfiguration("final_use_cartesian"), value_type=bool
                        ),
                        "final_planner_id": LaunchConfiguration("final_planner_id"),
                        "final_prefer_seeded_ik": ParameterValue(
                            LaunchConfiguration("final_prefer_seeded_ik"), value_type=bool
                        ),
                        "direct_prepare_min_fraction": ParameterValue(
                            LaunchConfiguration("direct_prepare_min_fraction"), value_type=float
                        ),
                        "final_joint_interp_steps": ParameterValue(
                            LaunchConfiguration("final_joint_interp_steps"), value_type=int
                        ),
                        "final_joint_motion_limit": ParameterValue(
                            LaunchConfiguration("final_joint_motion_limit"), value_type=float
                        ),
                        "final_roll_motion_limit": ParameterValue(
                            LaunchConfiguration("final_roll_motion_limit"), value_type=float
                        ),
                        "final_step_joint_motion_limit": ParameterValue(
                            LaunchConfiguration("final_step_joint_motion_limit"), value_type=float
                        ),
                        "final_step_roll_motion_limit": ParameterValue(
                            LaunchConfiguration("final_step_roll_motion_limit"), value_type=float
                        ),
                        "final_motion_duration": ParameterValue(
                            LaunchConfiguration("final_motion_duration"), value_type=float
                        ),
                        "insert_force_local_when_close": ParameterValue(
                            LaunchConfiguration("insert_force_local_when_close"), value_type=bool
                        ),
                        "insert_local_position_threshold": ParameterValue(
                            LaunchConfiguration("insert_local_position_threshold"), value_type=float
                        ),
                        "insert_local_axial_threshold": ParameterValue(
                            LaunchConfiguration("insert_local_axial_threshold"), value_type=float
                        ),
                        "insert_local_lateral_threshold": ParameterValue(
                            LaunchConfiguration("insert_local_lateral_threshold"), value_type=float
                        ),
                        "insert_local_orientation_threshold": ParameterValue(
                            LaunchConfiguration("insert_local_orientation_threshold"), value_type=float
                        ),
                        "insert_limit_base_motion": ParameterValue(
                            LaunchConfiguration("insert_limit_base_motion"), value_type=bool
                        ),
                        "insert_base_translation_limit_m": ParameterValue(
                            LaunchConfiguration("insert_base_translation_limit_m"), value_type=float
                        ),
                        "insert_base_rotation_limit_deg": ParameterValue(
                            LaunchConfiguration("insert_base_rotation_limit_deg"), value_type=float
                        ),
                        "final_cartesian_max_step": 0.01,
                        "final_cartesian_min_fraction": 0.99,
                        "retreat_use_cartesian": ParameterValue(
                            LaunchConfiguration("retreat_use_cartesian"), value_type=bool
                        ),
                        "retreat_planner_id": LaunchConfiguration("retreat_planner_id"),
                        "retreat_cartesian_max_step": 0.01,
                        "retreat_cartesian_min_fraction": 0.99,
                        "prepare_reference_target_z": 1.08,
                        "prepare_reference_raise_joint": 0.495,
                        "lift_reference_target_z": 1.08,
                        "lift_reference_raise_joint": 0.495,
                        "lift_band_half_width": ParameterValue(
                            LaunchConfiguration("lift_band_half_width"), value_type=float
                        ),
                        "lift_use_end_effector_height_alignment": ParameterValue(
                            LaunchConfiguration("lift_use_end_effector_height_alignment"), value_type=bool
                        ),
                        "lift_end_effector_target_offset": ParameterValue(
                            LaunchConfiguration("lift_end_effector_target_offset"), value_type=float
                        ),
                        "base_standoff": ParameterValue(LaunchConfiguration("base_standoff"), value_type=float),
                        "base_lateral_offset": ParameterValue(
                            LaunchConfiguration("base_lateral_offset"), value_type=float
                        ),
                        "base_yaw_offset_deg": 0.0,
                        "prepare_group_name": LaunchConfiguration("prepare_group_name"),
                        "pre_insert_group_name": LaunchConfiguration("pre_insert_group_name"),
                        "final_group_name": LaunchConfiguration("final_group_name"),
                        "insert_group_name": LaunchConfiguration("insert_group_name"),
                        "insert_fallback_group_name": LaunchConfiguration("insert_fallback_group_name"),
                        "retreat_group_name": LaunchConfiguration("retreat_group_name"),
                        "safe_retreat_group_name": LaunchConfiguration("safe_retreat_group_name"),
                        "discrete_arm_group_name": "arm_no_lift",
                        "execute_with_controller": ParameterValue(use_ros2_control, value_type=bool),
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="mock_pick_path_publisher.py",
                name="medipick_mock_pick_path_publisher",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    robot_description_kinematics,
                    robot_description_planning,
                    ompl_planning_pipeline_config,
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
