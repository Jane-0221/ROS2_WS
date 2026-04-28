import math
import random
import re
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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


def parse_pose_text(pose_text: str) -> tuple[float, float, float, float, float, float]:
    return tuple(map(float, pose_text.split()))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_world_includes(world_file: Path) -> list[dict]:
    root = ET.parse(world_file).getroot()
    includes = []
    for include in root.findall(".//include"):
        includes.append(
            {
                "name": (include.findtext("name") or "").strip(),
                "uri": (include.findtext("uri") or "").strip(),
                "pose": parse_pose_text((include.findtext("pose") or "0 0 0 0 0 0").strip()),
            }
        )
    return includes


def load_model_size(description_package: str, model_name: str) -> tuple[float, float, float]:
    sdf_path = (
        Path(get_package_share_directory(description_package))
        / "models"
        / model_name
        / "model.sdf"
    )
    text = sdf_path.read_text(encoding="utf-8")
    match = re.search(r"<size>([0-9.eE+-]+)\s+([0-9.eE+-]+)\s+([0-9.eE+-]+)</size>", text)
    if not match:
        raise RuntimeError(f"无法从 {sdf_path} 读取药盒尺寸")
    return float(match.group(1)), float(match.group(2)), float(match.group(3))


def compute_trial_spawn_state(
    *,
    description_package: str,
    world_file: Path,
    target_seed: int,
    target_entity_name: str,
    target_contact_inset: float,
    base_offset: float,
    base_lateral_offset: float,
    base_yaw_offset_deg: float,
) -> tuple[float, float, float]:
    includes = parse_world_includes(world_file)
    shelf_map: dict[str, dict] = {}
    drug_entities: list[dict] = []
    for item in includes:
        uri = item["uri"]
        if uri.startswith("model://textured_drug_"):
            drug_entities.append(item)
        elif uri.startswith("model://") and not uri.startswith("model://medipick"):
            shelf_map[item["name"]] = item

    if not drug_entities:
        raise RuntimeError("当前 world 中没有找到可用于纯抓取试验的药盒目标")

    if target_entity_name:
        matches = [item for item in drug_entities if item["name"] == target_entity_name]
        if not matches:
            raise RuntimeError(f"指定目标药盒不存在：{target_entity_name}")
        selected = matches[0]
    else:
        selected = random.Random(target_seed).choice(drug_entities)

    entity_name = selected["name"]
    model_name = selected["uri"].replace("model://", "", 1)
    x, y, z, _roll, _pitch, _entity_yaw = selected["pose"]
    shelf_name = re.sub(r"_l\d+_s\d+$", "", entity_name)
    if shelf_name not in shelf_map:
        raise RuntimeError(f"找不到目标药盒对应的货架：{shelf_name}")

    _sx, _sy, _sz, _sroll, _spitch, shelf_yaw = shelf_map[shelf_name]["pose"]
    _width, depth, _height = load_model_size(description_package, model_name)
    inward_x = math.cos(shelf_yaw)
    inward_y = math.sin(shelf_yaw)
    target_x = x - inward_x * max(0.0, depth / 2.0 - target_contact_inset)
    target_y = y - inward_y * max(0.0, depth / 2.0 - target_contact_inset)
    yaw = shelf_yaw

    lateral_dx = -math.sin(yaw) * base_lateral_offset
    lateral_dy = math.cos(yaw) * base_lateral_offset
    base_x = target_x - math.cos(yaw) * base_offset + lateral_dx
    base_y = target_y - math.sin(yaw) * base_offset + lateral_dy
    base_theta = normalize_angle(yaw + math.radians(base_yaw_offset_deg))
    return base_x, base_y, base_theta


def build_gazebo_trial_include(context, *args, **kwargs):
    moveit_package = "medipick_moveit_config"
    description_package = "medipick_simple3_description"
    gazebo_launch = (
        Path(get_package_share_directory(moveit_package))
        / "launch"
        / "gazebo_trial_demo.launch.py"
    )

    spawn_near_target = LaunchConfiguration("trial_spawn_near_target").perform(context).lower() == "true"
    world_file = Path(LaunchConfiguration("world_file").perform(context))
    initial_base_x = 0.0
    initial_base_y = 0.0
    initial_base_theta = 0.0
    if spawn_near_target:
        initial_base_x, initial_base_y, initial_base_theta = compute_trial_spawn_state(
            description_package=description_package,
            world_file=world_file,
            target_seed=int(LaunchConfiguration("target_seed").perform(context)),
            target_entity_name=str(LaunchConfiguration("target_entity_name").perform(context)).strip(),
            target_contact_inset=0.002,
            base_offset=float(LaunchConfiguration("trial_base_offset").perform(context)),
            base_lateral_offset=float(LaunchConfiguration("trial_base_lateral_offset").perform(context)),
            base_yaw_offset_deg=float(LaunchConfiguration("trial_base_yaw_offset_deg").perform(context)),
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_launch)),
            launch_arguments={
                "world_file": str(world_file),
                "rviz": LaunchConfiguration("rviz").perform(context),
                "gazebo_gui": LaunchConfiguration("gazebo_gui").perform(context),
                "with_move_group": "true",
                "initial_base_x": f"{initial_base_x:.6f}",
                "initial_base_y": f"{initial_base_y:.6f}",
                "initial_base_theta": f"{initial_base_theta:.6f}",
            }.items(),
        )
    ]


def generate_launch_description():
    moveit_package = "medipick_moveit_config"
    description_package = "medipick_simple3_description"
    planning_server_package = "medipick_planning_server"

    gazebo_launch = (
        Path(get_package_share_directory(moveit_package))
        / "launch"
        / "gazebo_trial_demo.launch.py"
    )
    planning_server_config = (
        Path(get_package_share_directory(planning_server_package))
        / "config"
        / "planning_server.yaml"
    )
    default_world = (
        Path(get_package_share_directory(description_package))
        / "worlds"
        / "medipick_pharmacy_textured.world.sdf"
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("target_seed", default_value="0"),
            DeclareLaunchArgument("target_entity_name", default_value=""),
            DeclareLaunchArgument("target_pose_frame", default_value="map"),
            DeclareLaunchArgument("auto_accept_base_arrival", default_value="true"),
            DeclareLaunchArgument("trial_spawn_near_target", default_value="true"),
            DeclareLaunchArgument("trial_base_offset", default_value="0.95"),
            DeclareLaunchArgument("trial_base_lateral_offset", default_value="0.0"),
            DeclareLaunchArgument("trial_base_yaw_offset_deg", default_value="0.0"),
            DeclareLaunchArgument("trial_min_motion_duration", default_value="4.0"),
            DeclareLaunchArgument("trial_base_linear_speed", default_value="0.20"),
            DeclareLaunchArgument("trial_base_angular_speed", default_value="0.35"),
            DeclareLaunchArgument("trial_arm_joint_speed", default_value="0.45"),
            DeclareLaunchArgument("trial_motion_duration_scale", default_value="2.5"),
            DeclareLaunchArgument("trial_motion_duration_margin", default_value="1.0"),
            DeclareLaunchArgument("trial_post_arrival_settle_time", default_value="2.0"),
            DeclareLaunchArgument("trial_navigation_route_mode", default_value="central_corridor"),
            DeclareLaunchArgument("trial_navigation_corridor_y", default_value="0.0"),
            DeclareLaunchArgument("trial_navigation_segment_translation_limit_m", default_value="0.20"),
            DeclareLaunchArgument("trial_navigation_segment_rotation_limit_deg", default_value="15.0"),
            DeclareLaunchArgument("trial_translate_then_rotate", default_value="true"),
            DeclareLaunchArgument("trial_task_start_delay", default_value="18.0"),
            DeclareLaunchArgument("use_cmd_vel_navigation", default_value="false"),
            DeclareLaunchArgument("use_discrete_lift", default_value="true"),
            DeclareLaunchArgument("use_legacy_lift_velocity_bridge", default_value="false"),
            DeclareLaunchArgument("cmd_vel_control_rate_hz", default_value="15.0"),
            DeclareLaunchArgument("cmd_vel_lookahead_time", default_value="0.25"),
            DeclareLaunchArgument("cmd_vel_command_timeout", default_value="0.40"),
            DeclareLaunchArgument("odom_publish_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("map_resolution", default_value="0.05"),
            DeclareLaunchArgument("nav_waypoint_position_tolerance", default_value="0.05"),
            DeclareLaunchArgument("nav_waypoint_yaw_tolerance_deg", default_value="8.0"),
            DeclareLaunchArgument("nav_final_position_tolerance", default_value="0.04"),
            DeclareLaunchArgument("nav_final_yaw_tolerance_deg", default_value="6.0"),
            DeclareLaunchArgument("nav_max_linear_speed", default_value="0.22"),
            DeclareLaunchArgument("nav_max_lateral_speed", default_value="0.22"),
            DeclareLaunchArgument("nav_max_angular_speed", default_value="0.55"),
            OpaqueFunction(function=build_gazebo_trial_include),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "lift_velocity_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-type",
                    "forward_command_controller/ForwardCommandController",
                    "--param-file",
                    str(Path(get_package_share_directory(moveit_package)) / "config" / "ros2_controllers_gazebo_trial.yaml"),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_legacy_lift_velocity_bridge")),
            ),
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
                        "manipulation_target_frame_id": "world",
                        "manipulation_target_tracking_enabled": True,
                        "manipulation_target_tracking_position_alpha": 0.35,
                        "manipulation_target_tracking_orientation_alpha": 0.25,
                        "manipulation_target_tracking_max_hold_sec": 4.0,
                        "freeze_manipulation_target_on_pre_insert": True,
                        "prefer_latest_target_on_manipulation_relock": True,
                        "auto_start_on_target": False,
                        "require_initialization_ready": False,
                        "initialization_ready_topic": "",
                        "auto_accept_base_arrival": LaunchConfiguration("auto_accept_base_arrival"),
                        "stage_timeout": 90.0,
                        "pre_insert_group_name": "mobile_arm",
                        "insert_group_name": "arm",
                        "insert_fallback_group_name": "mobile_arm",
                        "safe_retreat_group_name": "arm",
                        "pre_insert_try_arm_first": False,
                        "pre_insert_select_pose_only": True,
                        "pre_insert_candidate_count": 8,
                        "candidate_max_evaluations": 8,
                        "candidate_shortlist_size": 5,
                        "pre_insert_base_translation_limit_m": 0.90,
                        "pre_insert_base_rotation_limit_deg": 45.0,
                        "insert_base_translation_limit_m": 0.45,
                        "insert_base_rotation_limit_deg": 35.0,
                        "candidate_planning_time": 2.0,
                        "candidate_num_planning_attempts": 1,
                        "candidate_retry_on_failure": False,
                        "candidate_retry_planning_time": 5.0,
                        "candidate_retry_num_planning_attempts": 3,
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
                        "mock_lift_motion_duration": 10.0,
                        "stow_motion_duration": 6.0,
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
                        "discrete_lift_mode": LaunchConfiguration("use_discrete_lift"),
                        "discrete_arm_group_name": "arm_no_lift",
                        "stow_preserve_raise_joint": True,
                        "lift_segmented_execution_enabled": False,
                        "lift_segment_step_m": 0.06,
                        "lift_segment_min_duration": 2.0,
                        "lift_segment_speed_mps": 0.04,
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_discrete_lift_controller.py",
                name="medipick_gazebo_discrete_lift_controller",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_legacy_lift_velocity_bridge")),
                parameters=[
                    {
                        "joint_state_topic": "/joint_states",
                        "target_height_topic": "/medipick/task/lift_target_height",
                        "command_topic": "/lift_velocity_controller/commands",
                        "control_rate_hz": 20.0,
                        "height_tolerance": 0.008,
                        "slow_zone": 0.05,
                        "max_up_velocity": 0.10,
                        "max_down_velocity": 0.10,
                        "proportional_gain": 3.0,
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_trial_initial_pose_manager.py",
                name="medipick_gazebo_trial_initial_pose_manager",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("use_cmd_vel_navigation"),
                            "' == 'false' and '",
                            LaunchConfiguration("trial_spawn_near_target"),
                            "' == 'false'",
                        ]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": True,
                        "target_pose_topic": "/medipick/task/target_pose",
                        "world_file": LaunchConfiguration("world_file"),
                        "target_seed": LaunchConfiguration("target_seed"),
                        "target_entity_name": LaunchConfiguration("target_entity_name"),
                        "target_contact_inset": 0.002,
                        "controller_action_name": "/base_controller/follow_joint_trajectory",
                        "controller_name": "base_controller",
                        "controller_joint_names": [
                            "base_x",
                            "base_y",
                            "base_theta",
                        ],
                        "task_start_service": "/medipick/task/start",
                        "initialization_ready_topic": "/medipick/task/initialization_ready",
                        "base_offset": ParameterValue(LaunchConfiguration("trial_base_offset"), value_type=float),
                        "base_lateral_offset": ParameterValue(LaunchConfiguration("trial_base_lateral_offset"), value_type=float),
                        "base_yaw_offset_deg": ParameterValue(LaunchConfiguration("trial_base_yaw_offset_deg"), value_type=float),
                        "min_motion_duration": ParameterValue(LaunchConfiguration("trial_min_motion_duration"), value_type=float),
                        "base_linear_speed": ParameterValue(LaunchConfiguration("trial_base_linear_speed"), value_type=float),
                        "base_angular_speed": ParameterValue(LaunchConfiguration("trial_base_angular_speed"), value_type=float),
                        "arm_joint_speed": ParameterValue(LaunchConfiguration("trial_arm_joint_speed"), value_type=float),
                        "motion_duration_scale": ParameterValue(LaunchConfiguration("trial_motion_duration_scale"), value_type=float),
                        "motion_duration_margin": ParameterValue(LaunchConfiguration("trial_motion_duration_margin"), value_type=float),
                        "post_arrival_settle_time": ParameterValue(LaunchConfiguration("trial_post_arrival_settle_time"), value_type=float),
                        "auto_start_task": True,
                        "stow_arm_during_initialization": False,
                        "navigation_route_mode": LaunchConfiguration("trial_navigation_route_mode"),
                        "navigation_corridor_y": ParameterValue(LaunchConfiguration("trial_navigation_corridor_y"), value_type=float),
                        "navigation_segment_translation_limit_m": ParameterValue(LaunchConfiguration("trial_navigation_segment_translation_limit_m"), value_type=float),
                        "navigation_segment_rotation_limit_deg": ParameterValue(LaunchConfiguration("trial_navigation_segment_rotation_limit_deg"), value_type=float),
                        "translate_then_rotate": ParameterValue(LaunchConfiguration("trial_translate_then_rotate"), value_type=bool),
                    }
                ],
            ),
            TimerAction(
                period=LaunchConfiguration("trial_task_start_delay"),
                condition=IfCondition(LaunchConfiguration("trial_spawn_near_target")),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "bash",
                            "-lc",
                            "ros2 service call /medipick/task/start std_srvs/srv/Trigger '{}' >/tmp/medipick_pick_trial_start.log 2>&1",
                        ],
                        output="screen",
                    )
                ],
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_cmd_vel_bridge.py",
                name="medipick_gazebo_cmd_vel_bridge",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_cmd_vel_navigation")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "trajectory_topic": "/base_controller/joint_trajectory",
                        "control_rate_hz": LaunchConfiguration("cmd_vel_control_rate_hz"),
                        "lookahead_time": LaunchConfiguration("cmd_vel_lookahead_time"),
                        "command_timeout": LaunchConfiguration("cmd_vel_command_timeout"),
                        "max_linear_speed": LaunchConfiguration("nav_max_linear_speed"),
                        "max_lateral_speed": LaunchConfiguration("nav_max_lateral_speed"),
                        "max_angular_speed": LaunchConfiguration("nav_max_angular_speed"),
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
                        "publish_rate_hz": LaunchConfiguration("odom_publish_rate_hz"),
                        "publish_map_to_odom_tf": True,
                        "publish_odom_to_world_tf": False,
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_map_publisher.py",
                name="medipick_gazebo_map_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_file": LaunchConfiguration("world_file"),
                        "resolution": LaunchConfiguration("map_resolution"),
                    }
                ],
            ),
            Node(
                package=planning_server_package,
                executable="gazebo_pre_navigation_manager.py",
                name="medipick_gazebo_pre_navigation_manager",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_cmd_vel_navigation")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "target_pose_topic": "/medipick/task/target_pose",
                        "odom_topic": "/odom",
                        "cmd_vel_topic": "/cmd_vel",
                        "task_start_service": "/medipick/task/start",
                        "base_offset": LaunchConfiguration("trial_base_offset"),
                        "base_lateral_offset": LaunchConfiguration("trial_base_lateral_offset"),
                        "base_yaw_offset_deg": LaunchConfiguration("trial_base_yaw_offset_deg"),
                        "navigation_route_mode": LaunchConfiguration("trial_navigation_route_mode"),
                        "navigation_corridor_y": LaunchConfiguration("trial_navigation_corridor_y"),
                        "waypoint_position_tolerance": LaunchConfiguration("nav_waypoint_position_tolerance"),
                        "waypoint_yaw_tolerance_deg": LaunchConfiguration("nav_waypoint_yaw_tolerance_deg"),
                        "final_position_tolerance": LaunchConfiguration("nav_final_position_tolerance"),
                        "final_yaw_tolerance_deg": LaunchConfiguration("nav_final_yaw_tolerance_deg"),
                        "max_linear_speed": LaunchConfiguration("nav_max_linear_speed"),
                        "max_lateral_speed": LaunchConfiguration("nav_max_lateral_speed"),
                        "max_angular_speed": LaunchConfiguration("nav_max_angular_speed"),
                        "settle_time": LaunchConfiguration("trial_post_arrival_settle_time"),
                        "auto_start_task": True,
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
                        "target_seed": LaunchConfiguration("target_seed"),
                        "target_entity_name": LaunchConfiguration("target_entity_name"),
                        "start_delay": 0.5,
                        "frame_id": "world",
                        "world_frame": "world",
                        "camera_frame": "cam_link",
                        "publish_world_pose": True,
                        "publish_camera_pose": True,
                    }
                ],
            ),
        ]
    )
