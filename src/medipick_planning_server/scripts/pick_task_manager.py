#!/usr/bin/python3

from __future__ import annotations

import json
import math
import subprocess
import time
from typing import Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray

from medipick_planning_interfaces.srv import PlanToPose
from pick_task_flow import PickTaskFlowMixin
from pick_task_runtime import PickTaskRuntimeMixin
from pick_task_services import PickTaskServicesMixin
from pick_task_shared import (
    CandidateDebugSample,
    DEFAULT_PREPARE_ARM_POSITIONS,
    DEFAULT_STOW_STATE_POSITIONS,
    PickStage,
    PlannedStage,
    PrepareCandidate,
)
from pick_task_utils import (
    clone_pose_stamped,
    duration_to_msg,
    joint_position,
    joint_state_from_trajectory_end,
    joint_states_to_interpolated_trajectory,
    rotate_vector_by_quaternion,
    seconds_since,
    trajectory_effective_playback_duration,
)


def quaternion_multiply(a, b) -> tuple[float, float, float, float]:
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


def quaternion_normalize(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in quaternion)


def quaternion_slerp(
    start: tuple[float, float, float, float],
    end: tuple[float, float, float, float],
    ratio: float,
) -> tuple[float, float, float, float]:
    clamped_ratio = max(0.0, min(1.0, ratio))
    if clamped_ratio <= 0.0:
        return quaternion_normalize(start)
    if clamped_ratio >= 1.0:
        return quaternion_normalize(end)

    start_norm = quaternion_normalize(start)
    end_norm = quaternion_normalize(end)
    dot = sum(lhs * rhs for lhs, rhs in zip(start_norm, end_norm))
    if dot < 0.0:
        end_norm = tuple(-component for component in end_norm)
        dot = -dot

    if dot > 0.9995:
        blended = tuple(lhs + (rhs - lhs) * clamped_ratio for lhs, rhs in zip(start_norm, end_norm))
        return quaternion_normalize(blended)

    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    if abs(sin_theta_0) <= 1e-9:
        return quaternion_normalize(start_norm)

    theta = theta_0 * clamped_ratio
    sin_theta = math.sin(theta)
    scale_start = math.sin(theta_0 - theta) / sin_theta_0
    scale_end = sin_theta / sin_theta_0
    return quaternion_normalize(
        tuple(scale_start * lhs + scale_end * rhs for lhs, rhs in zip(start_norm, end_norm))
    )


class PickTaskManager(PickTaskFlowMixin, PickTaskServicesMixin, PickTaskRuntimeMixin, Node):
    def __init__(self) -> None:
        super().__init__("medipick_pick_task_manager")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("pose_link", "sucker_contact_link")
        self.declare_parameter("use_simplified_pipeline", True)
        self.declare_parameter("prepare_group_name", "arm")
        self.declare_parameter("final_group_name", "mobile_arm")
        self.declare_parameter("retreat_group_name", "mobile_arm")
        self.declare_parameter("pre_insert_group_name", "mobile_arm")
        self.declare_parameter("insert_group_name", "arm")
        self.declare_parameter("insert_fallback_group_name", "mobile_arm")
        self.declare_parameter("safe_retreat_group_name", "arm")
        self.declare_parameter("discrete_arm_group_name", "arm_no_lift")
        self.declare_parameter("ik_pose_link", "r6_link")
        self.declare_parameter("tool_reference_link", "sucker_contact_link")
        self.declare_parameter("tool_to_ik_offset_x", 0.1540)
        self.declare_parameter("tool_to_ik_offset_y", -0.0235)
        self.declare_parameter("tool_to_ik_offset_z", 0.0)
        self.declare_parameter("prepare_use_planning", False)
        self.declare_parameter("discrete_lift_mode", False)
        self.declare_parameter("lift_target_topic", "/medipick/task/lift_target_height")
        self.declare_parameter("auto_accept_lift_arrival", True)
        self.declare_parameter("mock_lift_motion_duration", 2.0)
        self.declare_parameter("final_use_cartesian", False)
        self.declare_parameter("final_planner_id", "")
        self.declare_parameter("final_prefer_seeded_ik", False)
        self.declare_parameter("final_seeded_ik_timeout", 0.3)
        self.declare_parameter("final_joint_interp_steps", 10)
        self.declare_parameter("final_joint_motion_limit", 1.4)
        self.declare_parameter("final_roll_motion_limit", 1.2)
        self.declare_parameter("final_step_joint_motion_limit", 0.45)
        self.declare_parameter("final_step_roll_motion_limit", 0.35)
        self.declare_parameter("final_motion_duration", 1.2)
        self.declare_parameter("insert_force_local_when_close", True)
        self.declare_parameter("insert_local_position_threshold", 0.12)
        self.declare_parameter("insert_local_axial_threshold", 0.12)
        self.declare_parameter("insert_local_lateral_threshold", 0.03)
        self.declare_parameter("insert_local_orientation_threshold", 0.20)
        self.declare_parameter("insert_limit_base_motion", True)
        self.declare_parameter("insert_base_translation_limit_m", 0.07)
        self.declare_parameter("insert_base_rotation_limit_deg", 8.0)
        self.declare_parameter("direct_final_try_before_prepare", True)
        self.declare_parameter("allow_partial_direct_prepare", False)
        self.declare_parameter("direct_prepare_min_fraction", 0.75)
        self.declare_parameter("prepare_projection_tolerance", 0.04)
        self.declare_parameter("prepare_axial_tolerance", 0.05)
        self.declare_parameter("prepare_orientation_tolerance", 0.16)
        self.declare_parameter("final_target_position_tolerance", 0.03)
        self.declare_parameter("final_target_orientation_tolerance", 0.45)
        self.declare_parameter("final_cartesian_max_step", 0.01)
        self.declare_parameter("final_cartesian_min_fraction", 0.99)
        self.declare_parameter("final_cartesian_revolute_jump_threshold", 0.45)
        self.declare_parameter("retreat_use_cartesian", False)
        self.declare_parameter("retreat_planner_id", "")
        self.declare_parameter("retreat_cartesian_max_step", 0.01)
        self.declare_parameter("retreat_cartesian_min_fraction", 0.99)
        self.declare_parameter("retreat_cartesian_revolute_jump_threshold", 0.45)
        self.declare_parameter("return_to_stow_after_retreat", True)
        self.declare_parameter("allow_return_to_stow_failure_after_retreat", False)
        self.declare_parameter("observe_duration", 0.2)
        self.declare_parameter("stage_timeout", 20.0)
        self.declare_parameter("target_pose_timeout", 3.0)
        self.declare_parameter("lock_target_after_acquire", True)
        self.declare_parameter("manipulation_target_frame_id", "")
        self.declare_parameter("manipulation_target_tracking_enabled", True)
        self.declare_parameter("manipulation_target_tracking_position_alpha", 0.35)
        self.declare_parameter("manipulation_target_tracking_orientation_alpha", 0.25)
        self.declare_parameter("manipulation_target_tracking_max_hold_sec", 4.0)
        self.declare_parameter("freeze_manipulation_target_on_pre_insert", True)
        self.declare_parameter("prefer_latest_target_on_manipulation_relock", True)
        self.declare_parameter("target_transform_timeout", 0.5)
        self.declare_parameter("require_initialization_ready", False)
        self.declare_parameter("initialization_ready_topic", "")
        self.declare_parameter("target_shelf_info_topic", "/medipick/task/target_shelf_info")
        self.declare_parameter("target_name_topic", "/medipick/gazebo_target_box/name")
        self.declare_parameter("acquire_target_stable_age", 0.4)
        self.declare_parameter("acquire_scene_settle_time", 0.6)
        self.declare_parameter("acquire_service_probe_timeout", 0.05)
        self.declare_parameter("acquire_fk_timeout", 1.0)
        self.declare_parameter("gazebo_world_name", "medipick_pharmacy_textured")
        self.declare_parameter("gazebo_remove_target_on_pick_success", True)
        self.declare_parameter("prepare_offset", 0.06)
        self.declare_parameter("pre_insert_offset", 0.06)
        self.declare_parameter("prepare_motion_duration", 2.5)
        self.declare_parameter("prepare_reference_target_z", 1.08)
        self.declare_parameter("prepare_reference_raise_joint", 0.495)
        self.declare_parameter("prepare_raise_gain", 1.0)
        self.declare_parameter("prepare_raise_min", 0.05)
        self.declare_parameter("prepare_raise_max", 0.75)
        self.declare_parameter("lift_reference_target_z", 1.08)
        self.declare_parameter("lift_reference_raise_joint", 0.495)
        self.declare_parameter("lift_height_gain", 1.0)
        self.declare_parameter("lift_height_min", 0.05)
        self.declare_parameter("lift_height_max", 0.75)
        self.declare_parameter("lift_band_half_width", 0.03)
        self.declare_parameter("lift_arrival_below_band_tolerance", 0.0)
        self.declare_parameter("lift_use_end_effector_height_alignment", True)
        self.declare_parameter("lift_end_effector_target_offset", 0.0)
        self.declare_parameter("lift_segmented_execution_enabled", False)
        self.declare_parameter("lift_segment_step_m", 0.08)
        self.declare_parameter("lift_segment_min_duration", 1.5)
        self.declare_parameter("lift_segment_speed_mps", 0.08)
        self.declare_parameter("lift_probe_insert_before_arrival", False)
        self.declare_parameter("lift_probe_insert_delay_sec", 6.0)
        self.declare_parameter("lift_allow_pre_insert_without_arrival", False)
        self.declare_parameter("lift_pre_insert_probe_delay_sec", 10.0)
        self.declare_parameter("low_shelf_target_z_threshold", 0.45)
        self.declare_parameter("lift_stage_stow_safe_min_height", 0.22)
        self.declare_parameter("enable_low_shelf_prepare_seed", True)
        self.declare_parameter("low_shelf_prepare_seed_target_z_threshold", 0.45)
        self.declare_parameter(
            "low_shelf_prepare_arm_positions",
            [DEFAULT_PREPARE_ARM_POSITIONS[joint_name] for joint_name in DEFAULT_PREPARE_ARM_POSITIONS.keys()],
        )
        self.declare_parameter("low_shelf_prepare_motion_duration", 1.0)
        self.declare_parameter("low_shelf_post_unfold_lift_duration", 1.0)
        self.declare_parameter("use_entry_plane_prepare", True)
        self.declare_parameter("cabinet_entry_margin", 0.05)
        self.declare_parameter("shelf_center_x", 0.88)
        self.declare_parameter("shelf_center_y", 0.0)
        self.declare_parameter("shelf_depth", 0.24)
        self.declare_parameter("retreat_offset", 0.18)
        self.declare_parameter("retreat_lift", 0.08)
        self.declare_parameter("base_standoff", 0.60)
        self.declare_parameter("base_lateral_offset", 0.18)
        self.declare_parameter("base_yaw_offset_deg", 0.0)
        self.declare_parameter("adaptive_workspace_enabled", True)
        self.declare_parameter("base_standoff_reference_depth", 0.24)
        self.declare_parameter("base_standoff_depth_gain", 0.35)
        self.declare_parameter("base_standoff_lateral_gain", 0.10)
        self.declare_parameter("base_standoff_min", 0.50)
        self.declare_parameter("base_standoff_max", 0.68)
        self.declare_parameter("pre_insert_offset_reference_depth", 0.24)
        self.declare_parameter("pre_insert_offset_depth_gain", 0.18)
        self.declare_parameter("pre_insert_offset_lateral_gain", 0.10)
        self.declare_parameter("pre_insert_offset_min", 0.05)
        self.declare_parameter("pre_insert_offset_max", 0.12)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 3)
        self.declare_parameter("candidate_planning_time", 1.0)
        self.declare_parameter("candidate_num_planning_attempts", 1)
        self.declare_parameter("candidate_retry_on_failure", True)
        self.declare_parameter("candidate_retry_planning_time", 3.0)
        self.declare_parameter("candidate_retry_num_planning_attempts", 2)
        self.declare_parameter("candidate_good_enough_score", 12.0)
        self.declare_parameter("candidate_max_evaluations", 1)
        self.declare_parameter("candidate_shortlist_size", 1)
        self.declare_parameter("candidate_pose_planning_fallback_enabled", False)
        self.declare_parameter("candidate_planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("pre_insert_first_candidate_only", False)
        self.declare_parameter("pre_insert_select_pose_only", True)
        self.declare_parameter("pre_insert_candidate_count", 3)
        self.declare_parameter("pre_insert_center_bias_max", 0.03)
        self.declare_parameter("pre_insert_center_bias_yaw_deg", 4.0)
        self.declare_parameter("pre_insert_try_arm_first", True)
        self.declare_parameter("pre_insert_arm_group_name", "arm")
        self.declare_parameter("pre_insert_limit_base_motion", True)
        self.declare_parameter("pre_insert_base_translation_limit_m", 0.30)
        self.declare_parameter("pre_insert_base_rotation_limit_deg", 25.0)
        self.declare_parameter("r1_stage_motion_limit_deg", 100.0)
        self.declare_parameter("r1_stage_motion_soft_penalty_start_deg", 70.0)
        self.declare_parameter("pre_insert_outside_margin", 0.01)
        self.declare_parameter("pre_insert_seeded_allow_invalid_start", True)
        self.declare_parameter("pre_insert_seeded_skip_invalid_prefix_points", 4)
        self.declare_parameter("retreat_retrace_insert_first", True)
        self.declare_parameter("max_velocity_scaling", 0.2)
        self.declare_parameter("max_acceleration_scaling", 0.2)
        self.declare_parameter("auto_start_on_target", False)
        self.declare_parameter("auto_accept_stow", True)
        self.declare_parameter("stow_joint_tolerance", 0.08)
        self.declare_parameter("stow_raise_tolerance", 0.03)
        self.declare_parameter("stow_preserve_raise_joint", False)
        self.declare_parameter("auto_accept_base_arrival", False)
        self.declare_parameter("skip_base_execution", False)
        self.declare_parameter("auto_publish_visualization_trajectory", True)
        self.declare_parameter("mock_base_motion_duration", 2.0)
        self.declare_parameter("post_base_settle_time", 0.5)
        self.declare_parameter("execute_with_controller", True)
        self.declare_parameter("controller_action_name", "/mobile_arm_controller/follow_joint_trajectory")
        self.declare_parameter("base_controller_action_name", "/base_controller/follow_joint_trajectory")
        self.declare_parameter("tool_controller_action_name", "/tool_controller/follow_joint_trajectory")
        self.declare_parameter("head_controller_action_name", "/head_controller/follow_joint_trajectory")
        self.declare_parameter("prepare_head_positions", [0.0, -0.5236])
        self.declare_parameter("prepare_head_motion_duration", 0.6)
        self.declare_parameter("prepare_head_max_step_rad", 0.35)
        self.declare_parameter("enable_pick_head_tracking", True)
        self.declare_parameter("pick_head_tracking_frame", "raise_link")
        self.declare_parameter(
            "pick_head_tracking_stages",
            [
                PickStage.LIFT_TO_BAND.value,
                PickStage.SELECT_PRE_INSERT.value,
                PickStage.PLAN_TO_PRE_INSERT.value,
                PickStage.INSERT_AND_SUCTION.value,
            ],
        )
        self.declare_parameter("pick_head_tracking_h1_offset_rad", 0.0)
        self.declare_parameter("pick_head_tracking_h2_offset_rad", -0.5236)
        self.declare_parameter("pick_head_tracking_h1_min_rad", -1.05)
        self.declare_parameter("pick_head_tracking_h1_max_rad", 1.05)
        self.declare_parameter("pick_head_tracking_h2_min_rad", -1.10)
        self.declare_parameter("pick_head_tracking_h2_max_rad", -0.05)
        self.declare_parameter("pick_head_tracking_update_period_sec", 0.35)
        self.declare_parameter("pick_head_tracking_trigger_error_rad", 0.08)
        self.declare_parameter("pick_head_tracking_motion_duration_sec", 0.35)
        self.declare_parameter("prepare_selection_retry_limit", 8)
        self.declare_parameter("controller_joint_names", [""])
        self.declare_parameter("base_controller_joint_names", ["base_x", "base_y", "base_theta"])
        self.declare_parameter("controller_goal_retry_count", 3)
        self.declare_parameter("controller_retry_delay", 2.0)
        self.declare_parameter("controller_settle_correction_passes", 2)
        self.declare_parameter("controller_settle_correction_min_duration", 0.9)
        self.declare_parameter("controller_settle_correction_max_duration", 2.5)
        self.declare_parameter("pre_insert_segment_max_base_translation_m", 0.05)
        self.declare_parameter("pre_insert_segment_max_base_rotation_deg", 5.0)
        self.declare_parameter("pre_insert_segment_max_arm_joint_motion", 0.40)
        self.declare_parameter("pre_insert_segment_max_count", 14)
        self.declare_parameter("pre_insert_segment_min_duration", 1.4)
        self.declare_parameter("pre_insert_segment_max_duration", 3.0)
        self.declare_parameter("pre_insert_segment_step_count", 6)
        self.declare_parameter("insert_segment_max_base_translation_m", 0.05)
        self.declare_parameter("insert_segment_max_base_rotation_deg", 5.0)
        self.declare_parameter("insert_segment_max_arm_joint_motion", 0.40)
        self.declare_parameter("insert_segment_max_count", 8)
        self.declare_parameter("insert_segment_min_duration", 1.4)
        self.declare_parameter("insert_segment_max_duration", 2.8)
        self.declare_parameter("insert_segment_step_count", 5)
        self.declare_parameter("whole_body_segment_max_base_translation_m", 0.10)
        self.declare_parameter("whole_body_segment_max_base_rotation_deg", 8.0)
        self.declare_parameter("whole_body_segment_max_arm_joint_motion", 0.85)
        self.declare_parameter("whole_body_segment_max_count", 6)
        self.declare_parameter("whole_body_segment_min_duration", 1.8)
        self.declare_parameter("whole_body_segment_max_duration", 4.0)
        self.declare_parameter("whole_body_segment_step_count", 5)
        self.declare_parameter("stow_motion_duration", 2.0)
        self.declare_parameter("trajectory_execution_timeout_margin", 4.0)
        self.declare_parameter("pump_command_topic", "/medipick/task/pump_command")
        self.declare_parameter("suction_state_topic", "/medipick/hardware/suction_state")
        self.declare_parameter("pump_after_insert", False)
        self.declare_parameter("require_suction_for_success", False)
        self.declare_parameter("suction_wait_timeout", 1.5)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._pose_link = str(self.get_parameter("pose_link").value)
        self._use_simplified_pipeline = bool(self.get_parameter("use_simplified_pipeline").value)
        self._prepare_group_name = str(self.get_parameter("prepare_group_name").value)
        self._final_group_name = str(self.get_parameter("final_group_name").value)
        self._retreat_group_name = str(self.get_parameter("retreat_group_name").value)
        self._pre_insert_group_name = str(self.get_parameter("pre_insert_group_name").value)
        self._insert_group_name = str(self.get_parameter("insert_group_name").value)
        self._insert_fallback_group_name = str(self.get_parameter("insert_fallback_group_name").value)
        self._safe_retreat_group_name = str(self.get_parameter("safe_retreat_group_name").value)
        self._discrete_arm_group_name = str(self.get_parameter("discrete_arm_group_name").value)
        self._ik_pose_link = str(self.get_parameter("ik_pose_link").value)
        self._tool_reference_link = str(self.get_parameter("tool_reference_link").value)
        self._tool_to_ik_offset = (
            float(self.get_parameter("tool_to_ik_offset_x").value),
            float(self.get_parameter("tool_to_ik_offset_y").value),
            float(self.get_parameter("tool_to_ik_offset_z").value),
        )
        self._discrete_lift_mode = bool(self.get_parameter("discrete_lift_mode").value)
        self._lift_target_topic = str(self.get_parameter("lift_target_topic").value)
        self._auto_accept_lift_arrival = bool(self.get_parameter("auto_accept_lift_arrival").value)
        self._mock_lift_motion_duration = float(self.get_parameter("mock_lift_motion_duration").value)
        self._final_use_cartesian = bool(self.get_parameter("final_use_cartesian").value)
        self._final_planner_id = str(self.get_parameter("final_planner_id").value)
        self._final_prefer_seeded_ik = bool(self.get_parameter("final_prefer_seeded_ik").value)
        self._final_seeded_ik_timeout = float(self.get_parameter("final_seeded_ik_timeout").value)
        self._final_joint_interp_steps = max(2, int(self.get_parameter("final_joint_interp_steps").value))
        self._final_joint_motion_limit = float(self.get_parameter("final_joint_motion_limit").value)
        self._final_roll_motion_limit = float(self.get_parameter("final_roll_motion_limit").value)
        self._final_step_joint_motion_limit = float(self.get_parameter("final_step_joint_motion_limit").value)
        self._final_step_roll_motion_limit = float(self.get_parameter("final_step_roll_motion_limit").value)
        self._final_motion_duration = float(self.get_parameter("final_motion_duration").value)
        self._insert_force_local_when_close = bool(self.get_parameter("insert_force_local_when_close").value)
        self._insert_local_position_threshold = float(self.get_parameter("insert_local_position_threshold").value)
        self._insert_local_axial_threshold = float(self.get_parameter("insert_local_axial_threshold").value)
        self._insert_local_lateral_threshold = float(self.get_parameter("insert_local_lateral_threshold").value)
        self._insert_local_orientation_threshold = float(
            self.get_parameter("insert_local_orientation_threshold").value
        )
        self._insert_limit_base_motion = bool(self.get_parameter("insert_limit_base_motion").value)
        self._insert_base_translation_limit_m = float(
            self.get_parameter("insert_base_translation_limit_m").value
        )
        self._insert_base_rotation_limit_deg = float(
            self.get_parameter("insert_base_rotation_limit_deg").value
        )
        self._direct_final_try_before_prepare = bool(self.get_parameter("direct_final_try_before_prepare").value)
        self._prepare_projection_tolerance = float(self.get_parameter("prepare_projection_tolerance").value)
        self._prepare_axial_tolerance = float(self.get_parameter("prepare_axial_tolerance").value)
        self._prepare_orientation_tolerance = float(self.get_parameter("prepare_orientation_tolerance").value)
        self._final_target_position_tolerance = float(self.get_parameter("final_target_position_tolerance").value)
        self._final_target_orientation_tolerance = float(self.get_parameter("final_target_orientation_tolerance").value)
        self._final_cartesian_max_step = float(self.get_parameter("final_cartesian_max_step").value)
        self._final_cartesian_min_fraction = float(self.get_parameter("final_cartesian_min_fraction").value)
        self._final_cartesian_revolute_jump_threshold = float(self.get_parameter("final_cartesian_revolute_jump_threshold").value)
        self._retreat_use_cartesian = bool(self.get_parameter("retreat_use_cartesian").value)
        self._retreat_planner_id = str(self.get_parameter("retreat_planner_id").value)
        self._retreat_cartesian_max_step = float(self.get_parameter("retreat_cartesian_max_step").value)
        self._retreat_cartesian_min_fraction = float(self.get_parameter("retreat_cartesian_min_fraction").value)
        self._retreat_cartesian_revolute_jump_threshold = float(self.get_parameter("retreat_cartesian_revolute_jump_threshold").value)
        self._return_to_stow_after_retreat = bool(self.get_parameter("return_to_stow_after_retreat").value)
        self._allow_return_to_stow_failure_after_retreat = bool(
            self.get_parameter("allow_return_to_stow_failure_after_retreat").value
        )
        self._observe_duration = float(self.get_parameter("observe_duration").value)
        self._stage_timeout = float(self.get_parameter("stage_timeout").value)
        self._target_pose_timeout = float(self.get_parameter("target_pose_timeout").value)
        self._lock_target_after_acquire = bool(self.get_parameter("lock_target_after_acquire").value)
        self._manipulation_target_frame_id = str(self.get_parameter("manipulation_target_frame_id").value).strip()
        self._manipulation_target_tracking_enabled = bool(
            self.get_parameter("manipulation_target_tracking_enabled").value
        )
        self._manipulation_target_tracking_position_alpha = max(
            0.0,
            min(1.0, float(self.get_parameter("manipulation_target_tracking_position_alpha").value)),
        )
        self._manipulation_target_tracking_orientation_alpha = max(
            0.0,
            min(1.0, float(self.get_parameter("manipulation_target_tracking_orientation_alpha").value)),
        )
        self._manipulation_target_tracking_max_hold_sec = max(
            0.0,
            float(self.get_parameter("manipulation_target_tracking_max_hold_sec").value),
        )
        self._freeze_manipulation_target_on_pre_insert = bool(
            self.get_parameter("freeze_manipulation_target_on_pre_insert").value
        )
        self._prefer_latest_target_on_manipulation_relock = bool(
            self.get_parameter("prefer_latest_target_on_manipulation_relock").value
        )
        self._target_transform_timeout = Duration(
            seconds=max(0.05, float(self.get_parameter("target_transform_timeout").value))
        )
        self._require_initialization_ready = bool(self.get_parameter("require_initialization_ready").value)
        self._initialization_ready_topic = str(self.get_parameter("initialization_ready_topic").value).strip()
        self._target_shelf_info_topic = str(self.get_parameter("target_shelf_info_topic").value)
        self._target_name_topic = str(self.get_parameter("target_name_topic").value)
        self._acquire_target_stable_age = float(self.get_parameter("acquire_target_stable_age").value)
        self._acquire_scene_settle_time = float(self.get_parameter("acquire_scene_settle_time").value)
        self._acquire_service_probe_timeout = float(self.get_parameter("acquire_service_probe_timeout").value)
        self._acquire_fk_timeout = float(self.get_parameter("acquire_fk_timeout").value)
        self._gazebo_world_name = str(self.get_parameter("gazebo_world_name").value).strip()
        self._gazebo_remove_target_on_pick_success = bool(
            self.get_parameter("gazebo_remove_target_on_pick_success").value
        )
        self._prepare_offset = float(self.get_parameter("prepare_offset").value)
        self._pre_insert_offset = float(self.get_parameter("pre_insert_offset").value)
        self._prepare_motion_duration = float(self.get_parameter("prepare_motion_duration").value)
        self._prepare_reference_target_z = float(self.get_parameter("prepare_reference_target_z").value)
        self._prepare_reference_raise_joint = float(self.get_parameter("prepare_reference_raise_joint").value)
        self._prepare_raise_gain = float(self.get_parameter("prepare_raise_gain").value)
        self._prepare_raise_min = float(self.get_parameter("prepare_raise_min").value)
        self._prepare_raise_max = float(self.get_parameter("prepare_raise_max").value)
        self._lift_reference_target_z = float(self.get_parameter("lift_reference_target_z").value)
        self._lift_reference_raise_joint = float(self.get_parameter("lift_reference_raise_joint").value)
        self._lift_height_gain = float(self.get_parameter("lift_height_gain").value)
        self._lift_height_min = float(self.get_parameter("lift_height_min").value)
        self._lift_height_max = float(self.get_parameter("lift_height_max").value)
        self._lift_band_half_width = float(self.get_parameter("lift_band_half_width").value)
        self._lift_arrival_below_band_tolerance = max(
            0.0,
            float(self.get_parameter("lift_arrival_below_band_tolerance").value),
        )
        self._lift_use_end_effector_height_alignment = bool(
            self.get_parameter("lift_use_end_effector_height_alignment").value
        )
        self._lift_end_effector_target_offset = float(
            self.get_parameter("lift_end_effector_target_offset").value
        )
        self._lift_segmented_execution_enabled = bool(
            self.get_parameter("lift_segmented_execution_enabled").value
        )
        self._lift_segment_step_m = float(self.get_parameter("lift_segment_step_m").value)
        self._lift_segment_min_duration = float(self.get_parameter("lift_segment_min_duration").value)
        self._lift_segment_speed_mps = float(self.get_parameter("lift_segment_speed_mps").value)
        self._lift_probe_insert_before_arrival = bool(
            self.get_parameter("lift_probe_insert_before_arrival").value
        )
        self._lift_probe_insert_delay_sec = float(self.get_parameter("lift_probe_insert_delay_sec").value)
        self._lift_allow_pre_insert_without_arrival = bool(
            self.get_parameter("lift_allow_pre_insert_without_arrival").value
        )
        self._lift_pre_insert_probe_delay_sec = float(
            self.get_parameter("lift_pre_insert_probe_delay_sec").value
        )
        self._low_shelf_target_z_threshold = float(self.get_parameter("low_shelf_target_z_threshold").value)
        self._lift_stage_stow_safe_min_height = float(
            self.get_parameter("lift_stage_stow_safe_min_height").value
        )
        self._enable_low_shelf_prepare_seed = bool(self.get_parameter("enable_low_shelf_prepare_seed").value)
        self._low_shelf_prepare_seed_target_z_threshold = float(
            self.get_parameter("low_shelf_prepare_seed_target_z_threshold").value
        )
        self._low_shelf_prepare_arm_joint_names = tuple(DEFAULT_PREPARE_ARM_POSITIONS.keys())
        self._low_shelf_prepare_arm_positions = tuple(
            float(value) for value in self.get_parameter("low_shelf_prepare_arm_positions").value
        )
        self._low_shelf_prepare_motion_duration = float(
            self.get_parameter("low_shelf_prepare_motion_duration").value
        )
        self._low_shelf_post_unfold_lift_duration = float(
            self.get_parameter("low_shelf_post_unfold_lift_duration").value
        )
        self._use_entry_plane_prepare = bool(self.get_parameter("use_entry_plane_prepare").value)
        self._cabinet_entry_margin = float(self.get_parameter("cabinet_entry_margin").value)
        self._shelf_center_x = float(self.get_parameter("shelf_center_x").value)
        self._shelf_center_y = float(self.get_parameter("shelf_center_y").value)
        self._shelf_depth = float(self.get_parameter("shelf_depth").value)
        self._retreat_offset = float(self.get_parameter("retreat_offset").value)
        self._retreat_lift = float(self.get_parameter("retreat_lift").value)
        self._base_standoff = float(self.get_parameter("base_standoff").value)
        self._base_lateral_offset = float(self.get_parameter("base_lateral_offset").value)
        self._base_yaw_offset_deg = float(self.get_parameter("base_yaw_offset_deg").value)
        self._adaptive_workspace_enabled = bool(self.get_parameter("adaptive_workspace_enabled").value)
        self._base_standoff_reference_depth = float(self.get_parameter("base_standoff_reference_depth").value)
        self._base_standoff_depth_gain = float(self.get_parameter("base_standoff_depth_gain").value)
        self._base_standoff_lateral_gain = float(self.get_parameter("base_standoff_lateral_gain").value)
        self._base_standoff_min = float(self.get_parameter("base_standoff_min").value)
        self._base_standoff_max = float(self.get_parameter("base_standoff_max").value)
        self._pre_insert_offset_reference_depth = float(self.get_parameter("pre_insert_offset_reference_depth").value)
        self._pre_insert_offset_depth_gain = float(self.get_parameter("pre_insert_offset_depth_gain").value)
        self._pre_insert_offset_lateral_gain = float(self.get_parameter("pre_insert_offset_lateral_gain").value)
        self._pre_insert_offset_min = float(self.get_parameter("pre_insert_offset_min").value)
        self._pre_insert_offset_max = float(self.get_parameter("pre_insert_offset_max").value)
        self._allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self._num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self._candidate_planning_time = float(self.get_parameter("candidate_planning_time").value)
        self._candidate_num_planning_attempts = int(self.get_parameter("candidate_num_planning_attempts").value)
        self._candidate_retry_on_failure = bool(self.get_parameter("candidate_retry_on_failure").value)
        self._candidate_retry_planning_time = float(self.get_parameter("candidate_retry_planning_time").value)
        self._candidate_retry_num_planning_attempts = int(
            self.get_parameter("candidate_retry_num_planning_attempts").value
        )
        self._candidate_good_enough_score = float(self.get_parameter("candidate_good_enough_score").value)
        self._candidate_max_evaluations = max(1, int(self.get_parameter("candidate_max_evaluations").value))
        self._candidate_shortlist_size = max(1, int(self.get_parameter("candidate_shortlist_size").value))
        self._candidate_planner_id = str(self.get_parameter("candidate_planner_id").value)
        self._pre_insert_first_candidate_only = bool(self.get_parameter("pre_insert_first_candidate_only").value)
        self._pre_insert_select_pose_only = bool(self.get_parameter("pre_insert_select_pose_only").value)
        self._pre_insert_candidate_count = max(1, int(self.get_parameter("pre_insert_candidate_count").value))
        self._pre_insert_center_bias_max = float(self.get_parameter("pre_insert_center_bias_max").value)
        self._pre_insert_center_bias_yaw_deg = float(self.get_parameter("pre_insert_center_bias_yaw_deg").value)
        self._pre_insert_try_arm_first = bool(self.get_parameter("pre_insert_try_arm_first").value)
        self._pre_insert_arm_group_name = str(self.get_parameter("pre_insert_arm_group_name").value)
        self._pre_insert_limit_base_motion = bool(self.get_parameter("pre_insert_limit_base_motion").value)
        self._pre_insert_base_translation_limit_m = float(
            self.get_parameter("pre_insert_base_translation_limit_m").value
        )
        self._pre_insert_base_rotation_limit_deg = float(
            self.get_parameter("pre_insert_base_rotation_limit_deg").value
        )
        self._r1_stage_motion_limit_deg = float(self.get_parameter("r1_stage_motion_limit_deg").value)
        self._r1_stage_motion_soft_penalty_start_deg = float(
            self.get_parameter("r1_stage_motion_soft_penalty_start_deg").value
        )
        self._pre_insert_outside_margin = float(self.get_parameter("pre_insert_outside_margin").value)
        self._pre_insert_seeded_allow_invalid_start = bool(
            self.get_parameter("pre_insert_seeded_allow_invalid_start").value
        )
        self._pre_insert_seeded_skip_invalid_prefix_points = max(
            0,
            int(self.get_parameter("pre_insert_seeded_skip_invalid_prefix_points").value),
        )
        self._retreat_retrace_insert_first = bool(self.get_parameter("retreat_retrace_insert_first").value)
        self._max_velocity_scaling = float(self.get_parameter("max_velocity_scaling").value)
        self._max_acceleration_scaling = float(self.get_parameter("max_acceleration_scaling").value)
        self._auto_start_on_target = bool(self.get_parameter("auto_start_on_target").value)
        self._auto_accept_stow = bool(self.get_parameter("auto_accept_stow").value)
        self._stow_joint_tolerance = float(self.get_parameter("stow_joint_tolerance").value)
        self._stow_raise_tolerance = float(self.get_parameter("stow_raise_tolerance").value)
        self._stow_preserve_raise_joint = bool(self.get_parameter("stow_preserve_raise_joint").value)
        self._auto_accept_base_arrival = bool(self.get_parameter("auto_accept_base_arrival").value)
        self._skip_base_execution = bool(self.get_parameter("skip_base_execution").value)
        self._auto_publish_visualization_trajectory = bool(self.get_parameter("auto_publish_visualization_trajectory").value)
        self._mock_base_motion_duration = float(self.get_parameter("mock_base_motion_duration").value)
        self._post_base_settle_time = float(self.get_parameter("post_base_settle_time").value)
        self._execute_with_controller = bool(self.get_parameter("execute_with_controller").value)
        self._controller_action_name = str(self.get_parameter("controller_action_name").value)
        self._base_controller_action_name = str(self.get_parameter("base_controller_action_name").value)
        self._tool_controller_action_name = str(self.get_parameter("tool_controller_action_name").value)
        self._head_controller_action_name = str(self.get_parameter("head_controller_action_name").value)
        self._prepare_head_positions = tuple(
            float(value) for value in self.get_parameter("prepare_head_positions").value
        )
        self._prepare_head_motion_duration = float(self.get_parameter("prepare_head_motion_duration").value)
        self._prepare_head_max_step_rad = float(self.get_parameter("prepare_head_max_step_rad").value)
        self._enable_pick_head_tracking = bool(self.get_parameter("enable_pick_head_tracking").value)
        self._pick_head_tracking_frame = str(self.get_parameter("pick_head_tracking_frame").value).strip()
        self._pick_head_tracking_stages = {
            str(value).strip()
            for value in self.get_parameter("pick_head_tracking_stages").value
            if str(value).strip()
        }
        self._pick_head_tracking_h1_offset_rad = float(
            self.get_parameter("pick_head_tracking_h1_offset_rad").value
        )
        self._pick_head_tracking_h2_offset_rad = float(
            self.get_parameter("pick_head_tracking_h2_offset_rad").value
        )
        self._pick_head_tracking_h1_min_rad = float(self.get_parameter("pick_head_tracking_h1_min_rad").value)
        self._pick_head_tracking_h1_max_rad = float(self.get_parameter("pick_head_tracking_h1_max_rad").value)
        self._pick_head_tracking_h2_min_rad = float(self.get_parameter("pick_head_tracking_h2_min_rad").value)
        self._pick_head_tracking_h2_max_rad = float(self.get_parameter("pick_head_tracking_h2_max_rad").value)
        self._pick_head_tracking_update_period_sec = max(
            0.05,
            float(self.get_parameter("pick_head_tracking_update_period_sec").value),
        )
        self._pick_head_tracking_trigger_error_rad = max(
            0.01,
            float(self.get_parameter("pick_head_tracking_trigger_error_rad").value),
        )
        self._pick_head_tracking_motion_duration_sec = max(
            0.10,
            float(self.get_parameter("pick_head_tracking_motion_duration_sec").value),
        )
        self._prepare_selection_retry_limit = max(
            1, int(self.get_parameter("prepare_selection_retry_limit").value)
        )
        self._controller_joint_names_override = tuple(
            str(joint_name)
            for joint_name in self.get_parameter("controller_joint_names").value
            if str(joint_name).strip()
        )
        self._base_controller_joint_names_override = tuple(
            str(joint_name)
            for joint_name in self.get_parameter("base_controller_joint_names").value
            if str(joint_name).strip()
        )
        self._controller_goal_retry_count = max(1, int(self.get_parameter("controller_goal_retry_count").value))
        self._controller_retry_delay = float(self.get_parameter("controller_retry_delay").value)
        self._controller_settle_correction_passes = max(
            0, int(self.get_parameter("controller_settle_correction_passes").value)
        )
        self._controller_settle_correction_min_duration = float(
            self.get_parameter("controller_settle_correction_min_duration").value
        )
        self._controller_settle_correction_max_duration = float(
            self.get_parameter("controller_settle_correction_max_duration").value
        )
        self._pre_insert_segment_max_base_translation_m = float(
            self.get_parameter("pre_insert_segment_max_base_translation_m").value
        )
        self._pre_insert_segment_max_base_rotation_deg = float(
            self.get_parameter("pre_insert_segment_max_base_rotation_deg").value
        )
        self._pre_insert_segment_max_arm_joint_motion = float(
            self.get_parameter("pre_insert_segment_max_arm_joint_motion").value
        )
        self._pre_insert_segment_max_count = int(self.get_parameter("pre_insert_segment_max_count").value)
        self._pre_insert_segment_min_duration = float(
            self.get_parameter("pre_insert_segment_min_duration").value
        )
        self._pre_insert_segment_max_duration = float(
            self.get_parameter("pre_insert_segment_max_duration").value
        )
        self._pre_insert_segment_step_count = int(self.get_parameter("pre_insert_segment_step_count").value)
        self._insert_segment_max_base_translation_m = float(
            self.get_parameter("insert_segment_max_base_translation_m").value
        )
        self._insert_segment_max_base_rotation_deg = float(
            self.get_parameter("insert_segment_max_base_rotation_deg").value
        )
        self._insert_segment_max_arm_joint_motion = float(
            self.get_parameter("insert_segment_max_arm_joint_motion").value
        )
        self._insert_segment_max_count = int(self.get_parameter("insert_segment_max_count").value)
        self._insert_segment_min_duration = float(self.get_parameter("insert_segment_min_duration").value)
        self._insert_segment_max_duration = float(self.get_parameter("insert_segment_max_duration").value)
        self._insert_segment_step_count = int(self.get_parameter("insert_segment_step_count").value)
        self._whole_body_segment_max_base_translation_m = float(
            self.get_parameter("whole_body_segment_max_base_translation_m").value
        )
        self._whole_body_segment_max_base_rotation_deg = float(
            self.get_parameter("whole_body_segment_max_base_rotation_deg").value
        )
        self._whole_body_segment_max_arm_joint_motion = float(
            self.get_parameter("whole_body_segment_max_arm_joint_motion").value
        )
        self._whole_body_segment_max_count = int(self.get_parameter("whole_body_segment_max_count").value)
        self._whole_body_segment_min_duration = float(
            self.get_parameter("whole_body_segment_min_duration").value
        )
        self._whole_body_segment_max_duration = float(
            self.get_parameter("whole_body_segment_max_duration").value
        )
        self._whole_body_segment_step_count = int(self.get_parameter("whole_body_segment_step_count").value)
        self._stow_motion_duration = float(self.get_parameter("stow_motion_duration").value)
        self._trajectory_execution_timeout_margin = float(self.get_parameter("trajectory_execution_timeout_margin").value)
        self._pump_command_topic = str(self.get_parameter("pump_command_topic").value)
        self._suction_state_topic = str(self.get_parameter("suction_state_topic").value)
        self._pump_after_insert = bool(self.get_parameter("pump_after_insert").value)
        self._require_suction_for_success = bool(self.get_parameter("require_suction_for_success").value)
        self._suction_wait_timeout = float(self.get_parameter("suction_wait_timeout").value)
        self._stow_state_positions = dict(DEFAULT_STOW_STATE_POSITIONS)
        if not self._pre_insert_arm_group_name:
            self._pre_insert_arm_group_name = self._prepare_group_name
        if not self._pre_insert_group_name:
            self._pre_insert_group_name = self._prepare_group_name
        if not self._insert_group_name:
            self._insert_group_name = self._final_group_name
        if not self._insert_fallback_group_name:
            self._insert_fallback_group_name = self._final_group_name
        if not self._safe_retreat_group_name:
            self._safe_retreat_group_name = self._retreat_group_name

        self._stage = PickStage.IDLE
        self._active = False
        self._stage_started_at = self.get_clock().now()
        self._last_observe_wait_log_wall_time = 0.0
        self._last_target_transform_warning_time = 0.0
        self._last_target_tracking_wait_log_wall_time = 0.0
        self._last_target_hold_log_wall_time = 0.0
        self._latest_target_pose: Optional[PoseStamped] = None
        self._latest_target_received_at = self.get_clock().now()
        self._locked_target_pose: Optional[PoseStamped] = None
        self._locked_target_acquired_at = self.get_clock().now()
        self._locked_target_last_measurement_at = self.get_clock().now()
        self._locked_target_frozen = False
        self._manipulation_target_relocked = False
        self._initialization_ready = not self._require_initialization_ready
        self._latest_target_shelf_name = ""
        self._latest_target_entity_name = ""
        self._picked_target_removed = False
        self._shelf_entry_x: Optional[float] = None
        self._shelf_entry_y: Optional[float] = None
        self._shelf_inward_axis_xy: Optional[tuple[float, float]] = None
        self._shelf_lateral_axis_xy: Optional[tuple[float, float]] = None
        self._current_joint_state = JointState()
        self._base_goal_pose: Optional[PoseStamped] = None
        self._prepare_pose: Optional[PoseStamped] = None
        self._prepare_target_distance = self._pre_insert_offset
        self._retreat_pose: Optional[PoseStamped] = None
        self._prepare_candidate: Optional[PrepareCandidate] = None
        self._prepare_candidate_queue: list[PrepareCandidate] = []
        self._prepare_selection_attempt_count = 0
        self._failed_prepare_pose_keys: set[tuple[float, float, float, float]] = set()
        self._candidate_debug_samples: list[CandidateDebugSample] = []
        self._target_lift_height: Optional[float] = None
        self._lift_target_center: Optional[float] = None
        self._lift_target_band_min: Optional[float] = None
        self._lift_target_band_max: Optional[float] = None
        self._base_arrived = False
        self._lift_arrived = False
        self._stow_arrived = False
        self._stow_motion_executed = False
        self._lift_motion_executed = False
        self._lift_insert_probe_attempted = False
        self._lift_prepare_probe_attempted = False
        self._base_motion_executed = False
        self._prepare_executed = False
        self._prepare_head_cleared = False
        self._low_shelf_prepare_executed = False
        self._low_shelf_target_lift_restored = False
        self._direct_insert_attempted = False
        self._last_prepare_result: Optional[PlannedStage] = None
        self._last_final_result: Optional[PlannedStage] = None
        self._last_insert_trajectory = JointTrajectory()
        self._selected_candidate_ready = False
        self._pick_head_tracking_goal_in_flight = False
        self._pick_head_tracking_requested_positions: Optional[tuple[float, float]] = None
        self._pick_head_tracking_last_sent_wall_time = 0.0
        self._last_pick_head_tracking_log_wall_time = 0.0
        self._pump_enabled = False
        self._suction_state: Optional[bool] = None
        self._suction_state_received_at = self.get_clock().now()

        self._client_callback_group = ReentrantCallbackGroup()
        self._subscription_callback_group = ReentrantCallbackGroup()
        self._timer_callback_group = MutuallyExclusiveCallbackGroup()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._stage_pub = self.create_publisher(String, "/medipick/task/stage", 10)
        self._event_pub = self.create_publisher(String, "/medipick/task/event", 10)
        self._pump_command_pub = self.create_publisher(Bool, self._pump_command_topic, 10)
        self._base_goal_pub = self.create_publisher(PoseStamped, "/medipick/task/base_goal", 10)
        self._lift_target_pub = self.create_publisher(Float64, self._lift_target_topic, 10)
        self._prepare_pose_pub = self.create_publisher(PoseStamped, "/medipick/task/prepare_pose", 10)
        self._pre_insert_pose_pub = self.create_publisher(PoseStamped, "/medipick/task/pre_insert_pose", 10)
        self._final_pose_pub = self.create_publisher(PoseStamped, "/medipick/task/final_pose", 10)
        self._pick_pose_pub = self.create_publisher(PoseStamped, "/medipick/task/pick_pose", 10)
        self._retreat_pose_pub = self.create_publisher(PoseStamped, "/medipick/task/retreat_pose", 10)
        self._achieved_pose_pub = self.create_publisher(PoseStamped, "/medipick/task/achieved_pose", 10)
        self._stage_path_pub = self.create_publisher(Path, "/medipick/task/stage_path", 10)
        self._stow_joint_state_pub = self.create_publisher(JointState, "/medipick/task/stow_joint_state", 10)
        self._trajectory_pub = self.create_publisher(JointTrajectory, "/medipick/task/planned_trajectory", 10)
        self._visual_trajectory_pub = self.create_publisher(JointTrajectory, "/medipick/visualization_trajectory", 10)
        self._visualization_joint_state_pub = self.create_publisher(
            JointState,
            "/medipick/visualization_joint_state",
            10,
        )
        self._debug_markers_pub = self.create_publisher(MarkerArray, "/medipick/task/debug_markers", 10)

        self.create_subscription(
            PoseStamped,
            "/medipick/task/target_pose",
            self._on_target_pose,
            10,
            callback_group=self._subscription_callback_group,
        )
        self.create_subscription(
            String,
            self._target_name_topic,
            self._on_target_name,
            10,
            callback_group=self._subscription_callback_group,
        )
        self.create_subscription(
            String,
            self._target_shelf_info_topic,
            self._on_target_shelf_info,
            10,
            callback_group=self._subscription_callback_group,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            20,
            callback_group=self._subscription_callback_group,
        )
        self.create_subscription(
            Bool,
            self._suction_state_topic,
            self._on_suction_state,
            20,
            callback_group=self._subscription_callback_group,
        )
        if self._initialization_ready_topic:
            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                Bool,
                self._initialization_ready_topic,
                self._on_initialization_ready,
                latched_qos,
                callback_group=self._subscription_callback_group,
            )

        self.create_service(Trigger, "/medipick/task/start", self._handle_start)
        self.create_service(Trigger, "/medipick/task/reset", self._handle_reset)
        self.create_service(Trigger, "/medipick/task/mark_base_arrived", self._handle_mark_base_arrived)
        self.create_service(Trigger, "/medipick/task/mark_lift_arrived", self._handle_mark_lift_arrived)

        self._plan_client = self.create_client(PlanToPose, "/medipick_planning_server/plan_to_pose", callback_group=self._client_callback_group)
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk", callback_group=self._client_callback_group)
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self._client_callback_group)
        self._state_validity_client = self.create_client(GetStateValidity, "/check_state_validity", callback_group=self._client_callback_group)
        self._cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path", callback_group=self._client_callback_group)
        self._trajectory_action = ActionClient(self, FollowJointTrajectory, self._controller_action_name, callback_group=self._client_callback_group)
        self._base_trajectory_action = ActionClient(
            self,
            FollowJointTrajectory,
            self._base_controller_action_name,
            callback_group=self._client_callback_group,
        )
        self._tool_trajectory_action = ActionClient(
            self,
            FollowJointTrajectory,
            self._tool_controller_action_name,
            callback_group=self._client_callback_group,
        )
        self._head_trajectory_action = ActionClient(
            self,
            FollowJointTrajectory,
            self._head_controller_action_name,
            callback_group=self._client_callback_group,
        )

        self.create_timer(0.1, self._tick, callback_group=self._timer_callback_group)
        self.get_logger().info("Pick task manager refactored pipeline ready.")
        if self._enable_pick_head_tracking:
            self.get_logger().info(
                "抓取阶段头部跟踪已启用："
                f"frame={self._pick_head_tracking_frame or self._frame_id}, "
                f"stages={sorted(self._pick_head_tracking_stages)}"
            )

    def _duration_to_msg(self, seconds: float):
        return duration_to_msg(seconds)

    def _task_target_pose(self) -> Optional[PoseStamped]:
        if self._locked_target_pose is not None:
            return self._locked_target_pose
        return self._latest_target_pose

    def _transform_pose_to_frame(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        if not target_frame:
            transformed = clone_pose_stamped(pose)
            transformed.header.stamp = self.get_clock().now().to_msg()
            return transformed

        source_frame = pose.header.frame_id or target_frame
        if source_frame == target_frame:
            transformed = clone_pose_stamped(pose)
            transformed.header.stamp = self.get_clock().now().to_msg()
            transformed.header.frame_id = target_frame
            return transformed

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=self._target_transform_timeout,
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self._last_target_transform_warning_time >= 2.0:
                self._last_target_transform_warning_time = now_sec
                self.get_logger().warning(
                    f"抓取任务等待目标位姿 TF：{source_frame} -> {target_frame}，原因：{exc}"
                )
            return None

        transformed = PoseStamped()
        transformed.header.stamp = self.get_clock().now().to_msg()
        transformed.header.frame_id = target_frame
        rotated = rotate_vector_by_quaternion(
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ),
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ),
        )
        transformed.pose.position.x = transform.transform.translation.x + rotated[0]
        transformed.pose.position.y = transform.transform.translation.y + rotated[1]
        transformed.pose.position.z = transform.transform.translation.z + rotated[2]
        transformed.pose.orientation.x, transformed.pose.orientation.y, transformed.pose.orientation.z, transformed.pose.orientation.w = quaternion_multiply(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ),
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ),
        )
        return transformed

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _pick_head_tracking_stage_active(self) -> bool:
        if not self._enable_pick_head_tracking:
            return False
        return self._stage.value in self._pick_head_tracking_stages

    def _compute_pick_head_tracking_positions(self) -> Optional[tuple[float, float]]:
        target_pose = self._task_target_pose()
        if target_pose is None:
            return None

        tracking_frame = self._pick_head_tracking_frame or self._frame_id
        tracked_pose = self._transform_pose_to_frame(target_pose, tracking_frame)
        if tracked_pose is None:
            return None

        target_x = tracked_pose.pose.position.x
        target_y = tracked_pose.pose.position.y
        target_z = tracked_pose.pose.position.z
        planar_distance = math.hypot(target_x, target_y)
        if planar_distance <= 1.0e-4:
            planar_distance = 1.0e-4

        desired_h1 = -math.atan2(target_y, target_x) + self._pick_head_tracking_h1_offset_rad
        desired_h2 = math.atan2(target_z, planar_distance) + self._pick_head_tracking_h2_offset_rad
        desired_h1 = self._clamp(
            desired_h1,
            self._pick_head_tracking_h1_min_rad,
            self._pick_head_tracking_h1_max_rad,
        )
        desired_h2 = self._clamp(
            desired_h2,
            self._pick_head_tracking_h2_min_rad,
            self._pick_head_tracking_h2_max_rad,
        )
        return desired_h1, desired_h2

    def _send_pick_head_tracking_goal(self, desired_positions: tuple[float, float]) -> None:
        if self._pick_head_tracking_goal_in_flight:
            return
        if not self._head_trajectory_action.wait_for_server(timeout_sec=0.02):
            now = time.monotonic()
            if now - self._last_pick_head_tracking_log_wall_time >= 2.0:
                self._last_pick_head_tracking_log_wall_time = now
                self.get_logger().info("抓取阶段头部跟踪等待 head_controller action server。")
            return

        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ["h1_joint", "h2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [desired_positions[0], desired_positions[1]]
        point.time_from_start = self._duration_to_msg(self._pick_head_tracking_motion_duration_sec)
        trajectory.points = [point]
        goal.trajectory = trajectory

        self._pick_head_tracking_goal_in_flight = True
        self._pick_head_tracking_requested_positions = desired_positions
        self._pick_head_tracking_last_sent_wall_time = time.monotonic()
        future = self._head_trajectory_action.send_goal_async(goal)
        future.add_done_callback(self._on_pick_head_tracking_goal_response)

    def _on_pick_head_tracking_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._pick_head_tracking_goal_in_flight = False
            self.get_logger().warning(f"抓取阶段头部跟踪目标发送失败：{exc}")
            return
        if not goal_handle.accepted:
            self._pick_head_tracking_goal_in_flight = False
            self.get_logger().warning("抓取阶段头部跟踪目标被 head_controller 拒绝。")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_pick_head_tracking_goal_result)

    def _on_pick_head_tracking_goal_result(self, future) -> None:
        self._pick_head_tracking_goal_in_flight = False
        try:
            _ = future.result()
        except Exception as exc:
            self.get_logger().warning(f"抓取阶段头部跟踪执行失败：{exc}")

    def _maybe_update_pick_head_tracking(self) -> None:
        if not self._pick_head_tracking_stage_active():
            return
        if self._pick_head_tracking_goal_in_flight:
            return
        now = time.monotonic()
        if now - self._pick_head_tracking_last_sent_wall_time < self._pick_head_tracking_update_period_sec:
            return

        desired_positions = self._compute_pick_head_tracking_positions()
        if desired_positions is None:
            return

        current_h1 = joint_position(self._current_joint_state, "h1_joint")
        current_h2 = joint_position(self._current_joint_state, "h2_joint")
        reference_h1, reference_h2 = desired_positions
        if math.isfinite(current_h1) and math.isfinite(current_h2):
            max_error = max(abs(reference_h1 - current_h1), abs(reference_h2 - current_h2))
            if max_error < self._pick_head_tracking_trigger_error_rad:
                return

        requested = self._pick_head_tracking_requested_positions
        if requested is not None:
            max_delta = max(
                abs(desired_positions[0] - requested[0]),
                abs(desired_positions[1] - requested[1]),
            )
            if max_delta < 0.02:
                return

        self._send_pick_head_tracking_goal(desired_positions)

    def _set_locked_target_pose(
        self,
        source_pose: PoseStamped,
        target_frame: str,
        reason: str,
        *,
        from_measurement: bool = True,
    ) -> bool:
        locked_target_pose = self._transform_pose_to_frame(source_pose, target_frame)
        if locked_target_pose is None:
            return False
        now = self.get_clock().now()
        previous_measurement_at = self._locked_target_last_measurement_at
        self._locked_target_pose = locked_target_pose
        self._locked_target_acquired_at = now
        self._locked_target_last_measurement_at = now if from_measurement else previous_measurement_at
        self._locked_target_frozen = False
        self._final_pose_pub.publish(self._locked_target_pose)
        self._pick_pose_pub.publish(self._locked_target_pose)
        frame_id = self._locked_target_pose.header.frame_id or target_frame
        position = self._locked_target_pose.pose.position
        self.get_logger().info(
            f"锁定抓取目标位姿（{reason}）：frame={frame_id}, "
            f"x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}"
        )
        self._event(
            f"Locked target pose in '{frame_id}' at "
            f"({position.x:.3f}, {position.y:.3f}, {position.z:.3f})."
        )
        return True

    def _lock_current_target_pose(self, reason: str) -> bool:
        if self._latest_target_pose is None:
            return False
        return self._set_locked_target_pose(
            self._latest_target_pose,
            self._frame_id,
            reason,
            from_measurement=True,
        )

    def _relock_task_target_pose_for_manipulation(self, reason: str) -> bool:
        if not self._manipulation_target_frame_id:
            return True
        if self._manipulation_target_relocked:
            return True
        use_latest_measurement = (
            self._prefer_latest_target_on_manipulation_relock
            and self._latest_target_pose is not None
            and self._has_fresh_target()
        )
        source_pose = self._latest_target_pose if use_latest_measurement else self._task_target_pose()
        if source_pose is None:
            return False
        if not self._set_locked_target_pose(
            source_pose,
            self._manipulation_target_frame_id,
            reason,
            from_measurement=use_latest_measurement,
        ):
            return False
        self._manipulation_target_relocked = True
        return True

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._latest_target_pose = msg
        self._latest_target_received_at = self.get_clock().now()
        self._final_pose_pub.publish(msg)
        self._pick_pose_pub.publish(msg)
        self._event(f"Received target pose in frame '{msg.header.frame_id}'.")
        if (
            self._auto_start_on_target
            and not self._active
            and self._stage == PickStage.IDLE
            and self._can_start_task()
        ):
            self._start_task()

    def _locked_target_hold_age_sec(self) -> float:
        return seconds_since(self.get_clock().now(), self._locked_target_last_measurement_at)

    def _locked_target_hold_is_valid(self) -> bool:
        if self._locked_target_pose is None:
            return False
        if self._locked_target_frozen:
            return True
        if self._manipulation_target_tracking_max_hold_sec <= 0.0:
            return False
        return self._locked_target_hold_age_sec() <= self._manipulation_target_tracking_max_hold_sec

    def _log_target_tracking_wait(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_target_tracking_wait_log_wall_time < 2.0:
            return
        self._last_target_tracking_wait_log_wall_time = now
        self.get_logger().info(f"manipulation_target: {message}")

    def _freeze_locked_target_pose(self, reason: str) -> None:
        if self._locked_target_pose is None or self._locked_target_frozen:
            return
        self._locked_target_frozen = True
        frame_id = self._locked_target_pose.header.frame_id or self._frame_id
        self.get_logger().info(f"冻结操作阶段抓取目标位姿（{reason}）：frame={frame_id}")
        self._event(f"Manipulation target frozen in '{frame_id}' during {reason}.")

    def _unfreeze_locked_target_pose(self, reason: str) -> None:
        if not self._locked_target_frozen:
            return
        self._locked_target_frozen = False
        self.get_logger().info(f"恢复操作阶段抓取目标跟踪（{reason}）。")
        self._event(f"Manipulation target tracking resumed during {reason}.")

    def _refresh_manipulation_target_tracking(self, reason: str, *, refresh_geometry: bool) -> bool:
        if not self._manipulation_target_tracking_enabled or not self._manipulation_target_frame_id:
            return True
        if self._locked_target_frozen:
            return True

        if self._latest_target_pose is None or not self._has_fresh_target():
            if self._locked_target_hold_is_valid():
                now = time.monotonic()
                if now - self._last_target_hold_log_wall_time >= 2.0:
                    self._last_target_hold_log_wall_time = now
                    self.get_logger().info(
                        "manipulation_target: 当前使用短时保持位姿，"
                        f"hold_age={self._locked_target_hold_age_sec():.2f}s/"
                        f"{self._manipulation_target_tracking_max_hold_sec:.2f}s."
                    )
                return True
            self._log_target_tracking_wait(
                f"waiting for fresh target pose or valid hold in '{self._manipulation_target_frame_id}'."
            )
            return False

        observed_pose = self._transform_pose_to_frame(self._latest_target_pose, self._manipulation_target_frame_id)
        if observed_pose is None:
            self._log_target_tracking_wait(
                f"waiting for target pose transform into '{self._manipulation_target_frame_id}'."
            )
            return False

        now = self.get_clock().now()
        current_locked_pose = self._locked_target_pose
        if (
            current_locked_pose is None
            or (current_locked_pose.header.frame_id or self._manipulation_target_frame_id)
            != self._manipulation_target_frame_id
        ):
            if not self._set_locked_target_pose(
                self._latest_target_pose,
                self._manipulation_target_frame_id,
                reason,
                from_measurement=True,
            ):
                return False
            if refresh_geometry and not self._refresh_task_geometry():
                return False
            return True

        updated_pose = PoseStamped()
        updated_pose.header.stamp = now.to_msg()
        updated_pose.header.frame_id = self._manipulation_target_frame_id
        position_alpha = self._manipulation_target_tracking_position_alpha
        orientation_alpha = self._manipulation_target_tracking_orientation_alpha
        updated_pose.pose.position.x = current_locked_pose.pose.position.x + (
            observed_pose.pose.position.x - current_locked_pose.pose.position.x
        ) * position_alpha
        updated_pose.pose.position.y = current_locked_pose.pose.position.y + (
            observed_pose.pose.position.y - current_locked_pose.pose.position.y
        ) * position_alpha
        updated_pose.pose.position.z = current_locked_pose.pose.position.z + (
            observed_pose.pose.position.z - current_locked_pose.pose.position.z
        ) * position_alpha
        blended_orientation = quaternion_slerp(
            (
                current_locked_pose.pose.orientation.x,
                current_locked_pose.pose.orientation.y,
                current_locked_pose.pose.orientation.z,
                current_locked_pose.pose.orientation.w,
            ),
            (
                observed_pose.pose.orientation.x,
                observed_pose.pose.orientation.y,
                observed_pose.pose.orientation.z,
                observed_pose.pose.orientation.w,
            ),
            orientation_alpha,
        )
        updated_pose.pose.orientation.x = blended_orientation[0]
        updated_pose.pose.orientation.y = blended_orientation[1]
        updated_pose.pose.orientation.z = blended_orientation[2]
        updated_pose.pose.orientation.w = blended_orientation[3]
        self._locked_target_pose = updated_pose
        self._locked_target_last_measurement_at = now
        self._final_pose_pub.publish(updated_pose)
        self._pick_pose_pub.publish(updated_pose)
        if refresh_geometry and not self._refresh_task_geometry():
            return False
        return True

    def _on_target_shelf_info(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"目标货架几何 JSON 解析失败：{exc}")
            return

        try:
            self._shelf_center_x = float(payload["shelf_center_x"])
            self._shelf_center_y = float(payload["shelf_center_y"])
            self._shelf_depth = float(payload["shelf_depth"])
            self._shelf_entry_x = float(payload["shelf_entry_x"])
            self._shelf_entry_y = float(payload["shelf_entry_y"])
            inward_x = float(payload["shelf_inward_axis_x"])
            inward_y = float(payload["shelf_inward_axis_y"])
            lateral_x = float(payload["shelf_lateral_axis_x"])
            lateral_y = float(payload["shelf_lateral_axis_y"])
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"目标货架几何字段不完整：{exc}")
            return

        self._shelf_inward_axis_xy = (inward_x, inward_y)
        self._shelf_lateral_axis_xy = (lateral_x, lateral_y)
        shelf_name = str(payload.get("shelf_name", "")).strip()
        if shelf_name and shelf_name != self._latest_target_shelf_name:
            self._latest_target_shelf_name = shelf_name
            self._event(
                "目标货架几何已更新："
                f"{shelf_name}, center=({self._shelf_center_x:.3f}, {self._shelf_center_y:.3f}), "
                f"entry=({self._shelf_entry_x:.3f}, {self._shelf_entry_y:.3f}), depth={self._shelf_depth:.3f}"
            )

    def _on_target_name(self, msg: String) -> None:
        target_name = msg.data.strip()
        if target_name:
            self._latest_target_entity_name = target_name

    def _on_joint_state(self, msg: JointState) -> None:
        self._current_joint_state = msg

    def _on_suction_state(self, msg: Bool) -> None:
        self._suction_state = bool(msg.data)
        self._suction_state_received_at = self.get_clock().now()

    def _on_initialization_ready(self, msg: Bool) -> None:
        previous = self._initialization_ready
        self._initialization_ready = bool(msg.data)
        if self._initialization_ready and not previous:
            self.get_logger().info("初始化完成信号已收到，允许启动抓取任务。")
            self._event("Initialization ready signal received.")

    def _can_start_task(self) -> bool:
        return (not self._require_initialization_ready) or self._initialization_ready

    def _handle_start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._has_fresh_target():
            response.success = False
            response.message = "No fresh target_pose received yet."
            return response
        if not self._can_start_task():
            response.success = False
            response.message = "Initialization is not complete yet."
            return response
        self._start_task()
        response.success = True
        response.message = "Pick task started."
        return response

    def _handle_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._reset_state()
        response.success = True
        response.message = "Pick task manager reset."
        return response

    def _handle_mark_base_arrived(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._base_arrived = True
        response.success = True
        response.message = "Base arrival acknowledged."
        return response

    def _handle_mark_lift_arrived(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._lift_arrived = True
        response.success = True
        response.message = "Lift arrival acknowledged."
        return response

    def _set_pump(self, enabled: bool, reason: Optional[str] = None) -> None:
        self._pump_enabled = enabled
        self._pump_command_pub.publish(Bool(data=enabled))
        if reason:
            self._event(reason)

    def _wait_for_suction_state(self, desired_state: bool, timeout_sec: float) -> bool:
        if timeout_sec <= 0.0:
            return self._suction_state is desired_state

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._suction_state is desired_state:
                return True
            time.sleep(0.05)
        return self._suction_state is desired_state

    def _start_task(self) -> None:
        if not self._can_start_task():
            self.get_logger().warning("忽略启动请求：初始化尚未完成。")
            return
        self._set_pump(False, "Starting new pick task with pump disabled.")
        self._suction_state = None
        self._active = True
        self._locked_target_pose = None
        self._locked_target_frozen = False
        self._manipulation_target_relocked = False
        self._base_arrived = False
        self._lift_arrived = False
        self._stow_arrived = False
        self._stow_motion_executed = False
        self._lift_motion_executed = False
        self._base_motion_executed = False
        self._prepare_executed = False
        self._prepare_head_cleared = False
        self._low_shelf_prepare_executed = False
        self._low_shelf_target_lift_restored = False
        self._direct_insert_attempted = False
        self._prepare_candidate = None
        self._prepare_candidate_queue = []
        self._prepare_selection_attempt_count = 0
        self._failed_prepare_pose_keys = set()
        self._candidate_debug_samples = []
        self._target_lift_height = None
        self._lift_target_center = None
        self._lift_target_band_min = None
        self._lift_target_band_max = None
        self._last_prepare_result = None
        self._last_final_result = None
        self._last_insert_trajectory = JointTrajectory()
        self._selected_candidate_ready = False
        self._picked_target_removed = False
        self._transition_to(PickStage.ACQUIRE_TARGET)

    def _reset_state(self) -> None:
        self._set_pump(False, "Resetting pick task manager with pump disabled.")
        self._suction_state = None
        self._active = False
        self._locked_target_pose = None
        self._locked_target_frozen = False
        self._manipulation_target_relocked = False
        self._base_arrived = False
        self._lift_arrived = False
        self._stow_arrived = False
        self._stow_motion_executed = False
        self._lift_motion_executed = False
        self._base_motion_executed = False
        self._prepare_executed = False
        self._prepare_head_cleared = False
        self._direct_insert_attempted = False
        self._base_goal_pose = None
        self._prepare_pose = None
        self._retreat_pose = None
        self._prepare_candidate = None
        self._prepare_candidate_queue = []
        self._prepare_selection_attempt_count = 0
        self._failed_prepare_pose_keys = set()
        self._candidate_debug_samples = []
        self._target_lift_height = None
        self._lift_target_center = None
        self._lift_target_band_min = None
        self._lift_target_band_max = None
        self._last_prepare_result = None
        self._last_final_result = None
        self._last_insert_trajectory = JointTrajectory()
        self._selected_candidate_ready = False
        self._picked_target_removed = False
        self._transition_to(PickStage.IDLE)

    def _remove_picked_target_from_gazebo(self) -> None:
        if (
            not self._gazebo_remove_target_on_pick_success
            or self._picked_target_removed
            or not self._gazebo_world_name
            or not self._latest_target_entity_name
        ):
            return

        request = f'name: "{self._latest_target_entity_name}" type: MODEL'
        command = [
            "ign",
            "service",
            "-s",
            f"/world/{self._gazebo_world_name}/remove",
            "--reqtype",
            "ignition.msgs.Entity",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
        except (subprocess.SubprocessError, OSError) as exc:
            self.get_logger().warning(
                f"Failed to remove picked target '{self._latest_target_entity_name}' from Gazebo: {exc}"
            )
            return

        if result.returncode != 0 or "data: true" not in result.stdout:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            detail = stderr or stdout or f"returncode={result.returncode}"
            self.get_logger().warning(
                "Gazebo target removal did not report success for "
                f"'{self._latest_target_entity_name}': {detail}"
            )
            return

        self._picked_target_removed = True
        self.get_logger().info(
            f"Gazebo target '{self._latest_target_entity_name}' removed after suction success."
        )
        self._event(f"Gazebo target '{self._latest_target_entity_name}' removed after suction success.")

    def _tick(self) -> None:
        self._stage_pub.publish(String(data=self._stage.value))
        self._publish_current_goals()
        self._publish_debug_markers()
        if not self._active:
            return
        if self._stage in (PickStage.COMPLETED, PickStage.FAILED):
            self._active = False
            return
        if self._stage_timed_out():
            self._fail(f"Stage {self._stage.value} timed out.")
            return
        self._maybe_update_pick_head_tracking()

        if self._stage == PickStage.ACQUIRE_TARGET:
            self._tick_observe()
        elif self._stage == PickStage.ARM_STOW_SAFE:
            self._tick_stow_arm()
        elif self._stage == PickStage.BASE_ENTER_WORKSPACE:
            self._tick_base_approach()
        elif self._stage == PickStage.LIFT_TO_BAND:
            self._tick_lift_align()
        elif self._stage == PickStage.SELECT_PRE_INSERT:
            self._tick_select_pre_insert()
        elif self._stage == PickStage.PLAN_TO_PRE_INSERT:
            self._tick_plan_to_pre_insert()
        elif self._stage == PickStage.INSERT_AND_SUCTION:
            self._tick_final_approach()
        elif self._stage == PickStage.SAFE_RETREAT:
            self._tick_retreat()

    def _planning_services_ready_for_acquire(self) -> bool:
        timeout_sec = max(0.0, self._acquire_service_probe_timeout)
        clients = (
            self._plan_client,
            self._fk_client,
            self._ik_client,
            self._state_validity_client,
            self._cartesian_client,
        )
        return all(client.wait_for_service(timeout_sec=timeout_sec) for client in clients)

    def _acquire_target_is_stable(self) -> bool:
        if self._acquire_target_stable_age <= 0.0:
            return True
        target_age = seconds_since(self.get_clock().now(), self._latest_target_received_at)
        if target_age >= self._acquire_target_stable_age:
            return True

        # In simulation the target box is often republished continuously even after it
        # has settled, so avoid waiting forever once we've already seen a fresh target
        # for a short, stable observation window.
        if self._has_fresh_target():
            stage_age = seconds_since(self.get_clock().now(), self._stage_started_at)
            return stage_age >= max(1.0, self._acquire_scene_settle_time)
        return False

    def _acquire_scene_is_settled(self) -> bool:
        stage_age = seconds_since(self.get_clock().now(), self._stage_started_at)
        return stage_age >= self._acquire_scene_settle_time

    def _log_observe_wait(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_observe_wait_log_wall_time < 2.0:
            return
        self._last_observe_wait_log_wall_time = now
        self.get_logger().info(f"acquire_target: {message}")

    def _tick_observe(self) -> None:
        if not self._has_fresh_target():
            self._log_observe_wait("waiting for a fresh target pose.")
            return
        if len(self._current_joint_state.name) == 0:
            self._log_observe_wait("waiting for joint_states.")
            return
        if not self._acquire_target_is_stable():
            target_age = seconds_since(self.get_clock().now(), self._latest_target_received_at)
            self._log_observe_wait(
                f"waiting for target stability ({target_age:.2f}s/{self._acquire_target_stable_age:.2f}s)."
            )
            return
        if not self._acquire_scene_is_settled():
            stage_age = seconds_since(self.get_clock().now(), self._stage_started_at)
            self._log_observe_wait(
                f"waiting for scene settle ({stage_age:.2f}s/{self._acquire_scene_settle_time:.2f}s)."
            )
            return
        if not self._planning_services_ready_for_acquire():
            self._log_observe_wait("waiting for planning/FK/IK services.")
            return
        if self._lock_target_after_acquire and self._locked_target_pose is None:
            if not self._lock_current_target_pose("acquire_target"):
                self._log_observe_wait(f"waiting for target pose transform into '{self._frame_id}'.")
                return
        if not self._refresh_task_geometry():
            self._fail("Unable to compute task geometry from current target.")
            return
        self._candidate_debug_samples = []
        self._prepare_candidate = None
        self._prepare_candidate_queue = []
        self._event("Task geometry refreshed from target pose.")
        self._transition_to(PickStage.ARM_STOW_SAFE)

    def _tick_stow_arm(self) -> None:
        stow_joint_state = self._stow_joint_state()
        self._stow_joint_state_pub.publish(stow_joint_state)
        if self._discrete_lift_mode and "raise_joint" in stow_joint_state.name:
            stow_positions = dict(zip(stow_joint_state.name, stow_joint_state.position))
            self._lift_target_pub.publish(Float64(data=stow_positions["raise_joint"]))

        if not self._stow_motion_executed:
            trajectory = self._build_stow_trajectory()
            if not self._execute_trajectory(trajectory, "stow_arm"):
                self._fail("Failed to execute stow trajectory.")
                return
            self._stow_motion_executed = True

        if self._stow_arrival_reached():
            self._stow_arrived = True
        elif self._auto_accept_stow and self._stow_motion_executed and not self._execute_with_controller:
            self._stow_arrived = True

        if self._stow_arrived:
            self._transition_to(PickStage.BASE_ENTER_WORKSPACE)

    def _tick_base_approach(self) -> None:
        if self._base_goal_pose is None:
            self._fail("Base goal pose is not available.")
            return
        if self._skip_base_execution:
            if not self._base_motion_executed:
                self._event("Base execution skipped; publishing base goal for external navigation.")
                self._base_motion_executed = True
                self._base_arrived = True
            if self._base_arrived:
                if self._manipulation_target_frame_id and not self._manipulation_target_relocked:
                    if not self._relock_task_target_pose_for_manipulation("base_arrived"):
                        self._log_observe_wait(
                            f"waiting for target pose transform into '{self._manipulation_target_frame_id}'."
                        )
                        return
                    if not self._refresh_task_geometry():
                        self._fail("Unable to refresh task geometry after relocking target pose for manipulation.")
                        return
                    self._event(
                        f"Relocked manipulation target into '{self._manipulation_target_frame_id}' after base arrival."
                    )
                self._unfreeze_locked_target_pose("base_arrived")
                self._transition_to(PickStage.LIFT_TO_BAND)
            return
        if not self._base_motion_executed:
            base_trajectory = self._build_base_trajectory(self._base_goal_pose, self._mock_base_motion_duration)
            if not self._execute_trajectory(base_trajectory, "base_approach"):
                self._fail("Failed to execute base approach trajectory.")
                return
            self._base_motion_executed = True
            if self._auto_accept_base_arrival or self._base_arrival_reached():
                self._base_arrived = True
            time.sleep(max(0.0, self._post_base_settle_time))

        if self._base_arrived:
            if self._manipulation_target_frame_id and not self._manipulation_target_relocked:
                if not self._relock_task_target_pose_for_manipulation("base_arrived"):
                    self._log_observe_wait(
                        f"waiting for target pose transform into '{self._manipulation_target_frame_id}'."
                    )
                    return
                if not self._refresh_task_geometry():
                    self._fail("Unable to refresh task geometry after relocking target pose for manipulation.")
                    return
                self._event(
                    f"Relocked manipulation target into '{self._manipulation_target_frame_id}' after base arrival."
                )
            self._unfreeze_locked_target_pose("base_arrived")
            self._transition_to(PickStage.LIFT_TO_BAND)

    def _tick_lift_align(self) -> None:
        if not self._refresh_manipulation_target_tracking("lift_align", refresh_geometry=True):
            return
        if self._lift_target_center is None or self._lift_target_band_min is None or self._lift_target_band_max is None:
            self._fail("Lift target band is not available.")
            return
        self._lift_target_pub.publish(Float64(data=self._lift_target_center))
        current_height = joint_position(self._current_joint_state, "raise_joint", self._lift_target_center)
        if self._auto_accept_lift_arrival and self._lift_arrival_reached():
            self._lift_arrived = True

        if not self._lift_arrived and not self._discrete_lift_mode:
            if self._lift_segmented_execution_enabled:
                next_height, segment_duration = self._next_lift_segment()
                lift_trajectory = self._build_lift_trajectory(next_height, segment_duration)
                if not self._execute_trajectory(
                    lift_trajectory,
                    f"lift_align_segment({next_height:.3f})",
                ):
                    self._fail("Failed to execute segmented lift alignment trajectory.")
                    return
                self._lift_motion_executed = True
            elif not self._lift_motion_executed:
                lift_trajectory = self._build_lift_trajectory(self._lift_target_center, self._mock_lift_motion_duration)
                if not self._execute_trajectory(lift_trajectory, "lift_align"):
                    self._fail("Failed to execute lift alignment trajectory.")
                    return
                self._lift_motion_executed = True

        if not self._lift_arrived and self._discrete_lift_mode and self._lift_segmented_execution_enabled:
            next_height, segment_duration = self._next_lift_segment()
            if abs(next_height - current_height) > 1e-4:
                lift_trajectory = self._build_lift_trajectory(next_height, segment_duration)
                if not self._execute_trajectory(
                    lift_trajectory,
                    f"lift_align_segment({next_height:.3f})",
                ):
                    self._fail("Failed to execute discrete lift alignment trajectory.")
                    return
                self._lift_motion_executed = True

        if not self._lift_arrived and self._discrete_lift_mode:
            stage_age = seconds_since(self.get_clock().now(), self._stage_started_at)
            if (
                self._lift_probe_insert_before_arrival
                and not self._lift_insert_probe_attempted
                and stage_age >= self._lift_probe_insert_delay_sec
            ):
                self._lift_insert_probe_attempted = True
                self.get_logger().warn(
                    "lift_align: 升降仍未进入目标带，先从当前高度尝试直接插入。"
                    f" current={current_height:.3f}, target={self._lift_target_center:.3f}, "
                    f"band=[{self._lift_target_band_min:.3f}, {self._lift_target_band_max:.3f}]"
                )
                if self._direct_final_try_before_prepare and not self._direct_insert_attempted:
                    self._direct_insert_attempted = True
                    if self._try_direct_insert_from_current_state():
                        return

            if (
                self._lift_allow_pre_insert_without_arrival
                and not self._lift_prepare_probe_attempted
                and stage_age >= self._lift_pre_insert_probe_delay_sec
            ):
                self._lift_prepare_probe_attempted = True
                self.get_logger().warn(
                    "lift_align: 升降仍未进入目标带，改为从当前高度继续 pre-insert 搜索。"
                    f" current={current_height:.3f}, target={self._lift_target_center:.3f}, "
                    f"band=[{self._lift_target_band_min:.3f}, {self._lift_target_band_max:.3f}]"
                )
                if not self._ensure_low_shelf_prepare_seed():
                    return
                if not self._ensure_low_shelf_target_lift_restored():
                    return
                if not self._ensure_prepare_head_clearance():
                    return
                self._transition_to(PickStage.SELECT_PRE_INSERT)
                return

        if self._auto_accept_lift_arrival and self._lift_arrival_reached():
            self._lift_arrived = True

        if self._lift_arrived:
            if not self._ensure_low_shelf_prepare_seed():
                return
            if not self._ensure_low_shelf_target_lift_restored():
                return
            if not self._ensure_prepare_head_clearance():
                return
            if self._direct_final_try_before_prepare and not self._direct_insert_attempted:
                self._direct_insert_attempted = True
                if self._try_direct_insert_from_current_state():
                    return
            self._transition_to(PickStage.SELECT_PRE_INSERT)

    def _ensure_prepare_head_clearance(self) -> bool:
        if self._prepare_head_cleared or self._prepare_head_clearance_reached():
            self._prepare_head_cleared = True
            return True
        head_trajectory = self._build_prepare_head_trajectory()
        if len(head_trajectory.joint_names) == 0 or len(head_trajectory.points) == 0:
            self._prepare_head_cleared = True
            return True
        if not self._execute_trajectory(head_trajectory, "prepare_head_clearance"):
            self._fail("Failed to move head to the manipulation clearance pose.")
            return False
        self._prepare_head_cleared = self._prepare_head_clearance_reached()
        if not self._prepare_head_cleared:
            self._fail("Head did not reach the manipulation clearance pose.")
            return False
        return True

    def _ensure_low_shelf_prepare_seed(self) -> bool:
        if not self._should_use_low_shelf_prepare_seed():
            self._low_shelf_prepare_executed = True
            return True
        if self._low_shelf_prepare_executed or self._low_shelf_prepare_seed_reached():
            self._low_shelf_prepare_executed = True
            return True
        seed_trajectory = self._build_low_shelf_prepare_trajectory()
        if len(seed_trajectory.joint_names) == 0 or len(seed_trajectory.points) == 0:
            self._low_shelf_prepare_executed = True
            return True
        if not self._execute_trajectory(seed_trajectory, "low_shelf_prepare_seed"):
            self._fail("Failed to move the arm into the low-shelf manipulation seed pose.")
            return False
        self._low_shelf_prepare_executed = self._low_shelf_prepare_seed_reached()
        if not self._low_shelf_prepare_executed:
            self._fail("Arm did not reach the low-shelf manipulation seed pose.")
            return False
        return True

    def _ensure_low_shelf_target_lift_restored(self) -> bool:
        if not self._should_restore_low_shelf_target_lift():
            self._low_shelf_target_lift_restored = True
            return True
        if self._low_shelf_target_lift_restored or self._low_shelf_target_lift_reached():
            self._low_shelf_target_lift_restored = True
            return True
        lift_trajectory = self._build_low_shelf_target_lift_trajectory()
        if len(lift_trajectory.joint_names) == 0 or len(lift_trajectory.points) == 0:
            self._low_shelf_target_lift_restored = True
            return True
        if not self._execute_trajectory(lift_trajectory, "low_shelf_target_lift_restore"):
            self._fail("Failed to restore the low-shelf lift height after unfolding the arm.")
            return False
        self._low_shelf_target_lift_restored = self._low_shelf_target_lift_reached()
        if not self._low_shelf_target_lift_restored:
            self._fail("Lift did not reach the low-shelf working height after unfolding the arm.")
            return False
        return True

    def _try_direct_insert_from_current_state(self) -> bool:
        if not self._refresh_manipulation_target_tracking("direct_insert_try", refresh_geometry=True):
            return False
        if self._freeze_manipulation_target_on_pre_insert:
            self._freeze_locked_target_pose("direct_insert_try")
        task_target_pose = self._task_target_pose()
        if task_target_pose is None:
            return False

        start_joint_state = self._current_joint_state
        result = self._plan_insert_stage(start_joint_state)
        self._last_final_result = result
        if not result.success:
            self.get_logger().info(f"direct_insert_try: planning failed: {result.message}")
            return False

        if not self._final_stage_reached(result, start_joint_state, task_target_pose, "direct_insert_check"):
            self.get_logger().info("direct_insert_try: final reach check failed; falling back to pre-insert selection.")
            return False

        if not self._trajectory_respects_joint_preferences(result, start_joint_state, "direct_insert_r1_limit"):
            self.get_logger().info("direct_insert_try: trajectory violates joint preference limits; falling back to pre-insert selection.")
            return False

        execution_trajectory = self._compress_dense_insert_trajectory(
            result.trajectory,
            start_joint_state,
            "direct_insert_and_suction",
        )
        if not self._execute_trajectory(execution_trajectory, "direct_insert_and_suction"):
            self._fail("Direct insert trajectory execution failed.")
            return True

        self._last_insert_trajectory = execution_trajectory
        self._event("Lift 对齐后直接插入成功，跳过 pre-insert 阶段。")
        self._transition_to(PickStage.SAFE_RETREAT)
        return True

    def _compress_dense_insert_trajectory(
        self,
        trajectory: JointTrajectory,
        start_joint_state: JointState,
        label: str,
    ) -> JointTrajectory:
        final_joint_state = joint_state_from_trajectory_end(start_joint_state, trajectory)
        joint_names = list(trajectory.joint_names)
        if len(joint_names) == 0:
            return trajectory

        start_positions = [joint_position(start_joint_state, joint_name) for joint_name in joint_names]
        end_positions = [joint_position(final_joint_state, joint_name) for joint_name in joint_names]
        max_delta = max((abs(end - start) for start, end in zip(start_positions, end_positions)), default=0.0)
        original_duration_sec = trajectory_effective_playback_duration(trajectory)
        step_count = max(
            4,
            min(
                10,
                max(
                    int(self._final_joint_interp_steps),
                    int(math.ceil(max_delta / 0.10)) + 2,
                ),
            ),
        )
        duration_sec = max(
            2.0,
            float(self._final_motion_duration),
            min(6.0, 1.2 + 2.4 * max_delta),
        )
        if len(trajectory.points) <= 12 and original_duration_sec <= max(6.0, duration_sec * 1.5):
            return trajectory
        self.get_logger().info(
            f"{label}: compressing dense insert trajectory for controller stability "
            f"({len(trajectory.points)} -> {step_count} points, "
            f"original_duration={original_duration_sec:.2f}s, duration={duration_sec:.2f}s, "
            f"max_delta={max_delta:.3f})."
        )
        return joint_states_to_interpolated_trajectory(
            joint_names=joint_names,
            start_positions=start_positions,
            end_positions=end_positions,
            duration_sec=duration_sec,
            step_count=step_count,
        )

    def _tick_select_pre_insert(self) -> None:
        self._prepare_selection_attempt_count += 1
        if self._prepare_selection_attempt_count > self._prepare_selection_retry_limit:
            self._fail(
                "Pre-insert candidate search exceeded the retry limit "
                f"({self._prepare_selection_retry_limit})."
            )
            return
        self._unfreeze_locked_target_pose("select_pre_insert")
        if not self._refresh_manipulation_target_tracking("select_pre_insert", refresh_geometry=True):
            return
        candidates = self._select_prepare_candidates()
        if not candidates:
            candidates = self._select_prepare_candidates_with_relaxed_limits()
        if not candidates:
            self._fail("No feasible outside-cabinet pre-insert candidate found.")
            return
        self._prepare_candidate = candidates[0]
        self._prepare_candidate_queue = list(candidates[1:])
        self._prepare_pose = self._prepare_candidate.prepare_pose
        self._last_prepare_result = self._prepare_candidate.prepare_result
        self._selected_candidate_ready = True
        total = len(candidates)
        self._event(
            f"Selected pre-insert candidate 1/{total} "
            f"(selection round {self._prepare_selection_attempt_count}/{self._prepare_selection_retry_limit})"
            + (f"; queued {total - 1} backup candidate(s)." if total > 1 else ".")
        )
        if self._freeze_manipulation_target_on_pre_insert:
            self._freeze_locked_target_pose("selected_pre_insert")
        self._transition_to(PickStage.PLAN_TO_PRE_INSERT)

    def _select_prepare_candidates_with_relaxed_limits(self) -> list[PrepareCandidate]:
        self.get_logger().warn(
            "No pre-insert candidate found with nominal limits; retrying with relaxed whole-body limits."
        )
        original_values = (
            self._pre_insert_base_translation_limit_m,
            self._pre_insert_base_rotation_limit_deg,
            self._r1_stage_motion_limit_deg,
            self._prepare_projection_tolerance,
            self._pre_insert_candidate_count,
            self._candidate_max_evaluations,
            self._candidate_shortlist_size,
            self._pre_insert_select_pose_only,
            self._pre_insert_try_arm_first,
            self._candidate_retry_on_failure,
        )
        try:
            self._pre_insert_base_translation_limit_m = max(self._pre_insert_base_translation_limit_m, 0.90)
            self._pre_insert_base_rotation_limit_deg = max(self._pre_insert_base_rotation_limit_deg, 45.0)
            self._r1_stage_motion_limit_deg = max(self._r1_stage_motion_limit_deg, 180.0)
            self._prepare_projection_tolerance = max(self._prepare_projection_tolerance, 0.06)
            self._pre_insert_candidate_count = max(self._pre_insert_candidate_count, 8)
            self._candidate_max_evaluations = max(self._candidate_max_evaluations, 8)
            self._candidate_shortlist_size = max(self._candidate_shortlist_size, 5)
            self._pre_insert_select_pose_only = True
            self._pre_insert_try_arm_first = False
            self._candidate_retry_on_failure = False
            return self._select_prepare_candidates()
        finally:
            (
                self._pre_insert_base_translation_limit_m,
                self._pre_insert_base_rotation_limit_deg,
                self._r1_stage_motion_limit_deg,
                self._prepare_projection_tolerance,
                self._pre_insert_candidate_count,
                self._candidate_max_evaluations,
                self._candidate_shortlist_size,
                self._pre_insert_select_pose_only,
                self._pre_insert_try_arm_first,
                self._candidate_retry_on_failure,
            ) = original_values

    def _advance_to_next_prepare_candidate(self, reason: str) -> bool:
        if not self._prepare_candidate_queue:
            return False
        next_candidate = self._prepare_candidate_queue.pop(0)
        self._prepare_candidate = next_candidate
        self._prepare_pose = next_candidate.prepare_pose
        self._last_prepare_result = next_candidate.prepare_result
        self._selected_candidate_ready = True
        self._event(
            f"{reason} Trying next queued pre-insert candidate; "
            f"{len(self._prepare_candidate_queue)} backup candidate(s) remain."
        )
        return True

    def _tick_plan_to_pre_insert(self) -> None:
        if self._prepare_candidate is None or not self._selected_candidate_ready:
            self._fail("Pre-insert candidate is not available.")
            return
        if len(self._prepare_candidate.prepare_result.trajectory.points) == 0:
            successful_plan, selected_group_name, group_failures = self._plan_pre_insert_candidate_from_state(
                prepare_pose=self._prepare_candidate.prepare_pose,
                preferred_group_name=self._prepare_candidate.preferred_group_name,
                start_joint_state=self._current_joint_state,
                log_prefix="plan_to_pre_insert",
            )
            if successful_plan is None:
                failure_message = "Pre-insert planning failed"
                if group_failures:
                    failure_message += ": " + "; ".join(group_failures)
                if self._advance_to_next_prepare_candidate(failure_message):
                    return
                self._fail(failure_message)
                return

            self._last_prepare_result = successful_plan
            self._prepare_candidate.prepare_result = successful_plan
            if selected_group_name:
                self._event(f"Pre-insert plan accepted with group '{selected_group_name}'.")
        if not self._execute_trajectory(self._prepare_candidate.prepare_result.trajectory, "plan_to_pre_insert"):
            if self._advance_to_next_prepare_candidate("Failed to execute selected pre-insert trajectory."):
                return
            self._fail("Failed to execute selected pre-insert trajectory.")
            return
        if not self._current_prepare_pose_reached(self._prepare_candidate.prepare_pose, "plan_to_pre_insert_actual_check"):
            if self._advance_to_next_prepare_candidate("Executed pre-insert trajectory did not reach the selected pose."):
                return
            self._fail("Executed pre-insert trajectory did not reach the selected pose.")
            return
        self._prepare_executed = True
        self._selected_candidate_ready = False
        self._transition_to(PickStage.INSERT_AND_SUCTION)

    def _tick_final_approach(self) -> None:
        start_joint_state = self._current_joint_state
        if not self._prepare_executed:
            self._fail("Insert stage cannot start before reaching a selected pre-insert pose.")
            return

        result = self._plan_insert_stage(start_joint_state)
        self._last_final_result = result
        task_target_pose = self._task_target_pose()
        if (
            task_target_pose is not None
            and
            result.success
            and self._final_stage_reached(result, start_joint_state, task_target_pose, "insert_check")
            and self._trajectory_respects_joint_preferences(result, start_joint_state, "insert_execute_r1_limit")
        ):
            execution_trajectory = self._compress_dense_insert_trajectory(
                result.trajectory,
                start_joint_state,
                "insert_and_suction",
            )
            if not self._execute_trajectory(execution_trajectory, "insert_and_suction"):
                self._fail("Failed to execute insert trajectory.")
                return
            if not self._current_final_pose_reached(task_target_pose, "insert_actual_check"):
                self._event("Insert execution ended outside the target tolerance; retrying pre-insert selection.")
                if self._prepare_pose is not None:
                    self._failed_prepare_pose_keys.add(self._prepare_pose_key(self._prepare_pose))
                if self._advance_to_next_prepare_candidate("Insert execution did not reach the target pose."):
                    self._prepare_executed = False
                    self._transition_to(PickStage.PLAN_TO_PRE_INSERT)
                    return
                self._event("Insert execution miss had no backup candidate; retrying candidate selection.")
                self._prepare_executed = False
                self._prepare_candidate = None
                self._prepare_candidate_queue = []
                self._prepare_pose = None
                self._transition_to(PickStage.SELECT_PRE_INSERT)
                return
            self._remove_picked_target_from_gazebo()
            if self._pump_after_insert:
                self._set_pump(True, "Insert trajectory finished; enabling pump.")
                suction_observed = self._wait_for_suction_state(True, self._suction_wait_timeout)
                if self._require_suction_for_success and not suction_observed:
                    self._fail("Pump enabled but suction feedback did not assert in time.")
                    return
                if suction_observed:
                    self._event("Suction feedback observed.")
                else:
                    self.get_logger().warning(
                        "Pump enabled but suction feedback was not observed before timeout."
                    )
                    self._event("Pump enabled but suction feedback not observed before timeout.")
            self._last_insert_trajectory = execution_trajectory
            self._transition_to(PickStage.SAFE_RETREAT)
            return

        if self._prepare_pose is not None:
            self._failed_prepare_pose_keys.add(self._prepare_pose_key(self._prepare_pose))
        if self._advance_to_next_prepare_candidate("Insert planning failed from current pre-insert pose."):
            self._prepare_executed = False
            self._transition_to(PickStage.PLAN_TO_PRE_INSERT)
            return
        self._event("Insert planning failed from current pre-insert pose; retrying candidate selection.")
        self._prepare_executed = False
        self._prepare_candidate = None
        self._prepare_candidate_queue = []
        self._prepare_pose = None
        self._transition_to(PickStage.SELECT_PRE_INSERT)

    def _tick_retreat(self) -> None:
        if self._retreat_pose is None:
            self._fail("Retreat pose is not available.")
            return
        start_joint_state = self._current_joint_state
        result = self._plan_retreat_stage(start_joint_state)
        if not result.success:
            self._fail(f"Retreat planning failed: {result.message}")
            return
        if not self._trajectory_respects_joint_preferences(result, start_joint_state, "retreat_r1_limit"):
            self._fail("Retreat trajectory violates the R1 reasonable-angle limit.")
            return
        if not self._execute_trajectory(result.trajectory, "safe_retreat"):
            if self._picked_target_removed:
                self.get_logger().warn("Retreat execution failed after suction success，按已完成抓取处理。")
                self._event("Retreat execution failed after suction success，按已完成抓取处理。")
                self._transition_to(PickStage.COMPLETED)
                self._event("Pick task finished.")
                return
            self._fail("Retreat execution failed.")
            return
        if self._return_to_stow_after_retreat:
            if self._discrete_lift_mode and "raise_joint" in self._stow_state_positions:
                self._lift_target_pub.publish(Float64(data=self._stow_state_positions["raise_joint"]))
            return_trajectory = self._build_stow_trajectory()
            if not self._execute_trajectory(return_trajectory, "return_to_stow"):
                if self._allow_return_to_stow_failure_after_retreat:
                    self.get_logger().warn("Return-to-stow execution failed after retreat，按已完成抓取处理。")
                    self._event("Return-to-stow execution failed after retreat，按已完成抓取处理。")
                else:
                    self._fail("Return-to-stow execution failed after retreat.")
                    return
        self._transition_to(PickStage.COMPLETED)
        self._event("Pick task finished.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickTaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
