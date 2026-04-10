#!/usr/bin/env python3

from __future__ import annotations

import time
from typing import Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import MarkerArray

from medipick_planning_interfaces.srv import PlanToPose
from pick_task_flow import PickTaskFlowMixin
from pick_task_runtime import PickTaskRuntimeMixin
from pick_task_services import PickTaskServicesMixin
from pick_task_shared import (
    CandidateDebugSample,
    DEFAULT_STOW_STATE_POSITIONS,
    PickStage,
    PlannedStage,
    PrepareCandidate,
)
from pick_task_utils import duration_to_msg, joint_state_from_trajectory_end, seconds_since


class PickTaskManager(PickTaskFlowMixin, PickTaskServicesMixin, PickTaskRuntimeMixin, Node):
    def __init__(self) -> None:
        super().__init__("medipick_pick_task_manager")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("pose_link", "sucker_link")
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
        self.declare_parameter("tool_reference_link", "sucker_link")
        self.declare_parameter("tool_to_ik_offset_x", 0.1112)
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
        self.declare_parameter("observe_duration", 0.2)
        self.declare_parameter("stage_timeout", 20.0)
        self.declare_parameter("target_pose_timeout", 3.0)
        self.declare_parameter("acquire_target_stable_age", 0.4)
        self.declare_parameter("acquire_scene_settle_time", 0.6)
        self.declare_parameter("acquire_service_probe_timeout", 0.05)
        self.declare_parameter("acquire_fk_timeout", 1.0)
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
        self.declare_parameter("lift_use_end_effector_height_alignment", True)
        self.declare_parameter("lift_end_effector_target_offset", 0.0)
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
        self.declare_parameter("auto_accept_base_arrival", False)
        self.declare_parameter("skip_base_execution", False)
        self.declare_parameter("auto_publish_visualization_trajectory", True)
        self.declare_parameter("mock_base_motion_duration", 2.0)
        self.declare_parameter("post_base_settle_time", 0.5)
        self.declare_parameter("execute_with_controller", True)
        self.declare_parameter("controller_action_name", "/mobile_arm_controller/follow_joint_trajectory")
        self.declare_parameter("stow_motion_duration", 2.0)
        self.declare_parameter("trajectory_execution_timeout_margin", 4.0)
        self.declare_parameter("pump_command_topic", "/medipick/task/pump_command")
        self.declare_parameter("suction_state_topic", "/medipick/hardware/suction_state")
        self.declare_parameter("pump_after_insert", True)
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
        self._observe_duration = float(self.get_parameter("observe_duration").value)
        self._stage_timeout = float(self.get_parameter("stage_timeout").value)
        self._target_pose_timeout = float(self.get_parameter("target_pose_timeout").value)
        self._acquire_target_stable_age = float(self.get_parameter("acquire_target_stable_age").value)
        self._acquire_scene_settle_time = float(self.get_parameter("acquire_scene_settle_time").value)
        self._acquire_service_probe_timeout = float(self.get_parameter("acquire_service_probe_timeout").value)
        self._acquire_fk_timeout = float(self.get_parameter("acquire_fk_timeout").value)
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
        self._lift_use_end_effector_height_alignment = bool(
            self.get_parameter("lift_use_end_effector_height_alignment").value
        )
        self._lift_end_effector_target_offset = float(
            self.get_parameter("lift_end_effector_target_offset").value
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
        self._auto_accept_base_arrival = bool(self.get_parameter("auto_accept_base_arrival").value)
        self._skip_base_execution = bool(self.get_parameter("skip_base_execution").value)
        self._auto_publish_visualization_trajectory = bool(self.get_parameter("auto_publish_visualization_trajectory").value)
        self._mock_base_motion_duration = float(self.get_parameter("mock_base_motion_duration").value)
        self._post_base_settle_time = float(self.get_parameter("post_base_settle_time").value)
        self._execute_with_controller = bool(self.get_parameter("execute_with_controller").value)
        self._controller_action_name = str(self.get_parameter("controller_action_name").value)
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
        self._latest_target_pose: Optional[PoseStamped] = None
        self._latest_target_received_at = self.get_clock().now()
        self._current_joint_state = JointState()
        self._base_goal_pose: Optional[PoseStamped] = None
        self._prepare_pose: Optional[PoseStamped] = None
        self._prepare_target_distance = self._pre_insert_offset
        self._retreat_pose: Optional[PoseStamped] = None
        self._prepare_candidate: Optional[PrepareCandidate] = None
        self._prepare_candidate_queue: list[PrepareCandidate] = []
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
        self._base_motion_executed = False
        self._prepare_executed = False
        self._last_prepare_result: Optional[PlannedStage] = None
        self._last_final_result: Optional[PlannedStage] = None
        self._last_insert_trajectory = JointTrajectory()
        self._selected_candidate_ready = False
        self._pump_enabled = False
        self._suction_state: Optional[bool] = None
        self._suction_state_received_at = self.get_clock().now()

        self._client_callback_group = ReentrantCallbackGroup()

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

        self.create_subscription(PoseStamped, "/medipick/task/target_pose", self._on_target_pose, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            Bool,
            self._suction_state_topic,
            self._on_suction_state,
            20,
            callback_group=self._client_callback_group,
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

        self.create_timer(0.1, self._tick)
        self.get_logger().info("Pick task manager refactored pipeline ready.")

    def _duration_to_msg(self, seconds: float):
        return duration_to_msg(seconds)

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._latest_target_pose = msg
        self._latest_target_received_at = self.get_clock().now()
        self._final_pose_pub.publish(msg)
        self._pick_pose_pub.publish(msg)
        self._event(f"Received target pose in frame '{msg.header.frame_id}'.")
        if self._auto_start_on_target and not self._active and self._stage == PickStage.IDLE:
            self._start_task()

    def _on_joint_state(self, msg: JointState) -> None:
        self._current_joint_state = msg

    def _on_suction_state(self, msg: Bool) -> None:
        self._suction_state = bool(msg.data)
        self._suction_state_received_at = self.get_clock().now()

    def _handle_start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if not self._has_fresh_target():
            response.success = False
            response.message = "No fresh target_pose received yet."
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
        self._set_pump(False, "Starting new pick task with pump disabled.")
        self._suction_state = None
        self._active = True
        self._base_arrived = False
        self._lift_arrived = False
        self._stow_arrived = False
        self._stow_motion_executed = False
        self._lift_motion_executed = False
        self._base_motion_executed = False
        self._prepare_executed = False
        self._prepare_candidate = None
        self._prepare_candidate_queue = []
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
        self._transition_to(PickStage.ACQUIRE_TARGET)

    def _reset_state(self) -> None:
        self._set_pump(False, "Resetting pick task manager with pump disabled.")
        self._suction_state = None
        self._active = False
        self._base_arrived = False
        self._lift_arrived = False
        self._stow_arrived = False
        self._stow_motion_executed = False
        self._lift_motion_executed = False
        self._base_motion_executed = False
        self._prepare_executed = False
        self._base_goal_pose = None
        self._prepare_pose = None
        self._retreat_pose = None
        self._prepare_candidate = None
        self._prepare_candidate_queue = []
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
        self._transition_to(PickStage.IDLE)

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
        target_age = seconds_since(self.get_clock().now(), self._latest_target_received_at)
        return target_age >= self._acquire_target_stable_age

    def _acquire_scene_is_settled(self) -> bool:
        stage_age = seconds_since(self.get_clock().now(), self._stage_started_at)
        return stage_age >= self._acquire_scene_settle_time

    def _tick_observe(self) -> None:
        if not self._has_fresh_target():
            return
        if len(self._current_joint_state.name) == 0:
            return
        if not self._acquire_target_is_stable():
            return
        if not self._acquire_scene_is_settled():
            return
        if not self._planning_services_ready_for_acquire():
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
        if self._discrete_lift_mode and "raise_joint" in self._stow_state_positions:
            self._lift_target_pub.publish(Float64(data=self._stow_state_positions["raise_joint"]))

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
                self._event("Base execution skipped; publishing base_goal for later base integration.")
                self._base_motion_executed = True
                self._base_arrived = True
            if self._base_arrived:
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
            self._transition_to(PickStage.LIFT_TO_BAND)

    def _tick_lift_align(self) -> None:
        if self._lift_target_center is None or self._lift_target_band_min is None or self._lift_target_band_max is None:
            self._fail("Lift target band is not available.")
            return
        self._lift_target_pub.publish(Float64(data=self._lift_target_center))
        if not self._lift_motion_executed and not self._discrete_lift_mode:
            lift_trajectory = self._build_lift_trajectory(self._lift_target_center, self._mock_lift_motion_duration)
            if not self._execute_trajectory(lift_trajectory, "lift_align"):
                self._fail("Failed to execute lift alignment trajectory.")
                return
            self._lift_motion_executed = True

        if self._auto_accept_lift_arrival and self._lift_arrival_reached():
            self._lift_arrived = True

        if self._lift_arrived:
            self._transition_to(PickStage.SELECT_PRE_INSERT)

    def _tick_select_pre_insert(self) -> None:
        candidates = self._select_prepare_candidates()
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
            f"Selected pre-insert candidate 1/{total}"
            + (f"; queued {total - 1} backup candidate(s)." if total > 1 else ".")
        )
        self._transition_to(PickStage.PLAN_TO_PRE_INSERT)

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
            group_failures: list[str] = []
            selected_group_name = ""
            successful_plan: Optional[PlannedStage] = None

            preferred_group_name = self._prepare_candidate.preferred_group_name
            for group_name in self._pre_insert_group_attempt_order(preferred_group_name):
                used_pose_fallback = False

                def plan_with_pose_goal() -> PlannedStage:
                    result = self._plan_stage(
                        stage=PickStage.PLAN_TO_PRE_INSERT,
                        group_name=group_name,
                        pose=self._prepare_candidate.prepare_pose,
                        start_joint_state=self._current_joint_state,
                        planner_id=self._candidate_planner_id,
                        allowed_planning_time=self._candidate_planning_time,
                        num_planning_attempts=self._candidate_num_planning_attempts,
                    )
                    retry_time = max(self._candidate_planning_time, self._candidate_retry_planning_time)
                    retry_attempts = max(
                        self._candidate_num_planning_attempts,
                        self._candidate_retry_num_planning_attempts,
                    )
                    if result.success or not self._candidate_retry_on_failure:
                        return result
                    if (
                        retry_time <= self._candidate_planning_time + 1e-6
                        and retry_attempts <= self._candidate_num_planning_attempts
                    ):
                        return result
                    self.get_logger().info(
                        "pre_insert_pose_retry: initial pose planning failed "
                        f"for group '{group_name}' ('{result.message}'); retrying with "
                        f"{retry_time:.1f}s and {retry_attempts} attempt(s)."
                    )
                    retry_result = self._plan_stage(
                        stage=PickStage.PLAN_TO_PRE_INSERT,
                        group_name=group_name,
                        pose=self._prepare_candidate.prepare_pose,
                        start_joint_state=self._current_joint_state,
                        planner_id=self._candidate_planner_id,
                        allowed_planning_time=retry_time,
                        num_planning_attempts=retry_attempts,
                    )
                    if retry_result.success:
                        return retry_result
                    retry_result.message = f"{result.message}; retry failed: {retry_result.message}"
                    return retry_result

                group_label = f"pre_insert_seeded[{group_name}]"
                plan_result = self._plan_stage_via_seeded_ik(
                    stage=PickStage.PLAN_TO_PRE_INSERT,
                    group_name=group_name,
                    pose=self._prepare_candidate.prepare_pose,
                    start_joint_state=self._current_joint_state,
                    label=group_label,
                )
                if not plan_result.success:
                    self.get_logger().info(
                        f"{group_label}: {plan_result.message} Falling back to pose planning."
                    )
                    plan_result = plan_with_pose_goal()
                    used_pose_fallback = True
                if not plan_result.success:
                    group_failures.append(f"{group_name}: {plan_result.message}")
                    continue

                if not self._trajectory_respects_joint_preferences(
                    plan_result,
                    self._current_joint_state,
                    f"plan_to_pre_insert_r1_limit[{group_name}]",
                ):
                    if not used_pose_fallback:
                        self.get_logger().info(
                            f"{group_label}: joint preference limit violated. Falling back to pose planning."
                        )
                        plan_result = plan_with_pose_goal()
                        used_pose_fallback = True
                    if not plan_result.success:
                        group_failures.append(f"{group_name}: {plan_result.message}")
                        continue
                    if not self._trajectory_respects_joint_preferences(
                        plan_result,
                        self._current_joint_state,
                        f"plan_to_pre_insert_r1_limit[{group_name}]",
                    ):
                        group_failures.append(f"{group_name}: joint preference limit")
                        continue

                if not self._prepare_stage_reached(
                    plan_result,
                    self._current_joint_state,
                    self._prepare_candidate.prepare_pose,
                    f"plan_to_pre_insert_check[{group_name}]",
                ):
                    if not used_pose_fallback:
                        self.get_logger().info(
                            f"{group_label}: selected pose not reached tightly enough. Falling back to pose planning."
                        )
                        plan_result = plan_with_pose_goal()
                        used_pose_fallback = True
                    if not plan_result.success:
                        group_failures.append(f"{group_name}: {plan_result.message}")
                        continue
                    if not self._prepare_stage_reached(
                        plan_result,
                        self._current_joint_state,
                        self._prepare_candidate.prepare_pose,
                        f"plan_to_pre_insert_check[{group_name}]",
                    ):
                        group_failures.append(f"{group_name}: reach check")
                        continue

                successful_plan = plan_result
                selected_group_name = group_name
                break

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
        if (
            result.success
            and self._final_stage_reached(result, start_joint_state, self._latest_target_pose, "insert_check")
            and self._trajectory_respects_joint_preferences(result, start_joint_state, "insert_execute_r1_limit")
        ):
            if not self._execute_trajectory(result.trajectory, "insert_and_suction"):
                self._fail("Failed to execute insert trajectory.")
                return
            self._last_insert_trajectory = result.trajectory
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
            self._fail("Retreat execution failed.")
            return
        if self._return_to_stow_after_retreat:
            if self._discrete_lift_mode and "raise_joint" in self._stow_state_positions:
                self._lift_target_pub.publish(Float64(data=self._stow_state_positions["raise_joint"]))
            return_trajectory = self._build_stow_trajectory()
            if not self._execute_trajectory(return_trajectory, "return_to_stow"):
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
