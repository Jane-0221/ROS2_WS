#!/usr/bin/python3

from __future__ import annotations

import math
from threading import Event
from typing import Optional

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from medipick_planning_interfaces.srv import PlanToPose
from pick_task_shared import PickStage, PlannedStage
from pick_task_utils import (
    canonicalize_revolute_joint_state,
    clone_pose,
    clone_pose_stamped,
    duration_to_msg,
    joint_state_base_planar_motion_extent,
    joint_position,
    joint_state_motion_extent,
    joint_state_from_trajectory_end,
    joint_state_positions,
    joint_states_to_interpolated_trajectory,
    joint_state_to_trajectory,
    merge_joint_states,
    normalize_xy,
    pose_error,
    prepare_alignment_errors,
    quaternion_to_yaw,
    rotate_vector_by_quaternion,
    yaw_to_quaternion,
)


class PickTaskServicesMixin:
    _REVOLUTE_JOINT_NAMES = {
        "base_theta",
        "r1_joint",
        "r2_joint",
        "r3_joint",
        "r4_joint",
        "r5_joint",
        "r6_joint",
        "sucker_joint",
        "h1_joint",
        "h2_joint",
        "l1_joint",
        "l2_joint",
        "l3_joint",
        "l4_joint",
        "l5_joint",
    }
    _LOCAL_INSERT_ROLL_JOINT_NAMES = {"r6_joint", "sucker_joint"}

    def _plan_stage(
        self,
        stage: PickStage,
        group_name: str,
        pose: PoseStamped,
        start_joint_state: Optional[JointState],
        planner_id: str = "",
        allowed_planning_time: Optional[float] = None,
        num_planning_attempts: Optional[int] = None,
    ) -> PlannedStage:
        request = PlanToPose.Request()
        request.group_name = group_name
        request.pose_link = self._pose_link
        request.planner_id = planner_id
        request.target_pose = pose
        request.allowed_planning_time = (
            self._allowed_planning_time if allowed_planning_time is None else allowed_planning_time
        )
        request.num_planning_attempts = (
            self._num_planning_attempts if num_planning_attempts is None else num_planning_attempts
        )
        request.max_velocity_scaling = self._max_velocity_scaling
        request.max_acceleration_scaling = self._max_acceleration_scaling
        request.use_start_joint_state = bool(start_joint_state is not None and len(start_joint_state.name) > 0)
        if request.use_start_joint_state:
            request.start_joint_state = start_joint_state

        response = self._call_plan_service(request, stage.value)
        if response is None:
            return PlannedStage(stage, False, "Planning service unavailable or timed out.", 0.0, JointTrajectory(), JointState())

        success = response.success and len(response.joint_trajectory.points) > 0
        return PlannedStage(
            stage=stage,
            success=success,
            message=response.message,
            planning_time=response.planning_time,
            trajectory=response.joint_trajectory,
            final_joint_state=response.final_joint_state,
        )

    def _plan_stage_via_seeded_ik(
        self,
        stage: PickStage,
        group_name: str,
        pose: PoseStamped,
        start_joint_state: Optional[JointState],
        label: str,
    ) -> PlannedStage:
        if start_joint_state is None or len(start_joint_state.name) == 0:
            return PlannedStage(
                stage,
                False,
                "Seeded IK planning requires a non-empty start joint state.",
                0.0,
                JointTrajectory(),
                JointState(),
            )

        ik_solution = self._call_ik_service(
            group_name=group_name,
            target_pose=pose,
            start_joint_state=start_joint_state,
            label=f"{label}_ik",
        )
        if ik_solution is None:
            return PlannedStage(stage, False, "Seeded IK could not find a solution.", 0.0, JointTrajectory(), JointState())

        merged_seed = merge_joint_states(self._current_joint_state, start_joint_state)
        normalized_goal = canonicalize_revolute_joint_state(
            ik_solution,
            merged_seed,
            self._REVOLUTE_JOINT_NAMES,
        )
        if not self._check_state_validity(normalized_goal, group_name, f"{label}_goal_validity"):
            return PlannedStage(
                stage,
                False,
                "Seeded IK produced a goal state that is invalid.",
                0.0,
                JointTrajectory(),
                JointState(),
            )
        if not self._seeded_goal_respects_joint_preferences(
            normalized_goal,
            merged_seed,
            f"{label}_goal_preference",
        ):
            return PlannedStage(
                stage,
                False,
                "Seeded IK goal violates the R1 stage-motion limit.",
                0.0,
                JointTrajectory(),
                JointState(),
            )

        ordered_joint_names = list(normalized_goal.name)
        seed_positions = joint_state_positions(merged_seed)
        goal_positions = joint_state_positions(normalized_goal)
        start_positions = [seed_positions.get(name, goal_positions[name]) for name in ordered_joint_names]
        end_positions = [goal_positions[name] for name in ordered_joint_names]
        max_delta = max((abs(end - start) for start, end in zip(start_positions, end_positions)), default=0.0)
        step_count = max(8, min(28, int(math.ceil(max_delta / 0.08)) + 2))
        duration_sec = max(1.2, min(4.0, 1.0 + 1.2 * max_delta))
        trajectory = joint_states_to_interpolated_trajectory(
            joint_names=ordered_joint_names,
            start_positions=start_positions,
            end_positions=end_positions,
            duration_sec=duration_sec,
            step_count=step_count,
        )
        tolerate_invalid_start = (
            stage == PickStage.PLAN_TO_PRE_INSERT and self._pre_insert_seeded_allow_invalid_start
        )
        if not self._validate_trajectory_states(
            trajectory,
            start_joint_state,
            group_name,
            f"{label}_trajectory_validity",
        ):
            if tolerate_invalid_start and self._validate_trajectory_states(
                trajectory,
                start_joint_state,
                group_name,
                f"{label}_trajectory_validity_relaxed",
                allow_invalid_start=True,
                allowed_invalid_prefix_points=self._pre_insert_seeded_skip_invalid_prefix_points,
            ):
                return PlannedStage(
                    stage=stage,
                    success=True,
                    message="Seeded IK interpolation escaped a lightly colliding start state.",
                    planning_time=0.0,
                    trajectory=trajectory,
                    final_joint_state=normalized_goal,
                )
            return PlannedStage(
                stage,
                False,
                "Seeded IK joint interpolation collided or left the valid state space.",
                0.0,
                JointTrajectory(),
                JointState(),
            )

        return PlannedStage(
            stage=stage,
            success=True,
            message="Seeded IK joint interpolation succeeded.",
            planning_time=0.0,
            trajectory=trajectory,
            final_joint_state=normalized_goal,
        )

    def _plan_stage_via_local_seeded_ik(
        self,
        stage: PickStage,
        group_name: str,
        pose: PoseStamped,
        start_joint_state: Optional[JointState],
        label: str,
    ) -> PlannedStage:
        if start_joint_state is None or len(start_joint_state.name) == 0:
            return PlannedStage(
                stage,
                False,
                "Local seeded IK planning requires a non-empty start joint state.",
                0.0,
                JointTrajectory(),
                JointState(),
            )

        ik_solution = self._call_ik_service(
            group_name=group_name,
            target_pose=pose,
            start_joint_state=start_joint_state,
            label=f"{label}_ik",
        )
        if ik_solution is None:
            return PlannedStage(stage, False, "Local seeded IK could not find a solution.", 0.0, JointTrajectory(), JointState())

        merged_seed = merge_joint_states(self._current_joint_state, start_joint_state)
        normalized_goal = canonicalize_revolute_joint_state(
            ik_solution,
            merged_seed,
            self._REVOLUTE_JOINT_NAMES,
        )
        if not self._check_state_validity(normalized_goal, group_name, f"{label}_goal_validity"):
            return PlannedStage(
                stage,
                False,
                "Local seeded IK produced a goal state that is invalid.",
                0.0,
                JointTrajectory(),
                JointState(),
            )
        if not self._seeded_goal_respects_joint_preferences(
            normalized_goal,
            merged_seed,
            f"{label}_goal_preference",
            stage=stage,
        ):
            return PlannedStage(
                stage,
                False,
                "Local seeded IK goal violates stage motion preferences.",
                0.0,
                JointTrajectory(),
                JointState(),
            )

        ordered_joint_names = list(normalized_goal.name)
        seed_positions = joint_state_positions(merged_seed)
        goal_positions = joint_state_positions(normalized_goal)
        start_positions = [seed_positions.get(name, goal_positions[name]) for name in ordered_joint_names]
        end_positions = [goal_positions[name] for name in ordered_joint_names]

        required_steps = max(2, self._final_joint_interp_steps)
        max_joint_extent = 0.0
        max_roll_extent = 0.0
        for joint_name, start_position, end_position in zip(ordered_joint_names, start_positions, end_positions):
            delta = abs(end_position - start_position)
            if joint_name in self._LOCAL_INSERT_ROLL_JOINT_NAMES:
                if delta > self._final_roll_motion_limit:
                    return PlannedStage(
                        stage,
                        False,
                        f"Local seeded IK would rotate {joint_name} by {delta:.3f}, exceeding the roll limit.",
                        0.0,
                        JointTrajectory(),
                        JointState(),
                    )
                max_roll_extent = max(max_roll_extent, delta)
                step_limit = max(1e-6, self._final_step_roll_motion_limit)
            elif joint_name in {"base_x", "base_y", "base_theta"}:
                if joint_name == "base_theta":
                    step_limit = max(1e-6, math.radians(self._insert_base_rotation_limit_deg))
                else:
                    step_limit = max(1e-6, self._insert_base_translation_limit_m)
            else:
                if delta > self._final_joint_motion_limit:
                    return PlannedStage(
                        stage,
                        False,
                        f"Local seeded IK would move {joint_name} by {delta:.3f}, exceeding the joint limit.",
                        0.0,
                        JointTrajectory(),
                        JointState(),
                    )
                max_joint_extent = max(max_joint_extent, delta)
                step_limit = max(1e-6, self._final_step_joint_motion_limit)
            required_steps = max(required_steps, int(math.ceil(delta / step_limit)) + 1)

        duration_sec = max(
            self._final_motion_duration,
            0.8 + 0.6 * max_joint_extent + 0.4 * max_roll_extent,
        )
        trajectory = joint_states_to_interpolated_trajectory(
            joint_names=ordered_joint_names,
            start_positions=start_positions,
            end_positions=end_positions,
            duration_sec=duration_sec,
            step_count=required_steps,
        )
        if not self._validate_trajectory_states(
            trajectory,
            start_joint_state,
            group_name,
            f"{label}_trajectory_validity",
        ):
            return PlannedStage(
                stage,
                False,
                "Local seeded IK interpolation collided or left the valid state space.",
                0.0,
                JointTrajectory(),
                JointState(),
            )

        return PlannedStage(
            stage=stage,
            success=True,
            message="Local seeded IK joint interpolation succeeded.",
            planning_time=0.0,
            trajectory=trajectory,
            final_joint_state=normalized_goal,
        )

    def _plan_cartesian_stage(
        self,
        stage: PickStage,
        group_name: str,
        pose: PoseStamped,
        start_joint_state: Optional[JointState],
        max_step: Optional[float] = None,
        min_fraction: Optional[float] = None,
        revolute_jump_threshold: Optional[float] = None,
    ) -> PlannedStage:
        if start_joint_state is None or len(start_joint_state.name) == 0:
            return PlannedStage(
                stage,
                False,
                "Cartesian planning requires a non-empty start joint state.",
                0.0,
                JointTrajectory(),
                JointState(),
            )

        response = self._call_cartesian_service(
            group_name=group_name,
            target_pose=pose,
            start_joint_state=start_joint_state,
            label=stage.value,
            max_step=self._final_cartesian_max_step if max_step is None else max_step,
            revolute_jump_threshold=(
                self._final_cartesian_revolute_jump_threshold
                if revolute_jump_threshold is None
                else revolute_jump_threshold
            ),
        )
        if response is None:
            return PlannedStage(stage, False, "Cartesian planning service unavailable or timed out.", 0.0, JointTrajectory(), JointState())

        fraction = float(response.fraction)
        trajectory = response.solution.joint_trajectory
        required_fraction = self._final_cartesian_min_fraction if min_fraction is None else min_fraction
        success = int(response.error_code.val) == 1 and fraction >= required_fraction and len(trajectory.points) > 0
        message = (
            f"Cartesian path fraction={fraction:.3f}, points={len(trajectory.points)}, code={int(response.error_code.val)}"
        )
        final_joint_state = joint_state_from_trajectory_end(start_joint_state, trajectory)
        return PlannedStage(
            stage=stage,
            success=success,
            message=message,
            planning_time=0.0,
            trajectory=trajectory,
            final_joint_state=final_joint_state,
        )

    def _call_plan_service(self, request: PlanToPose.Request, label: str):
        if not self._plan_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{label}: /medipick_planning_server/plan_to_pose unavailable.")
            return None

        future = self._plan_client.call_async(request)
        done = Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self._stage_timeout):
            future.cancel()
            self.get_logger().error(f"{label}: planning request timed out.")
            return None
        return future.result()

    def _call_fk_service(
        self,
        joint_state: JointState,
        link_name: str,
        label: str,
        timeout_sec: Optional[float] = None,
        wait_for_service_timeout_sec: float = 3.0,
    ) -> Optional[PoseStamped]:
        if not self._fk_client.wait_for_service(timeout_sec=wait_for_service_timeout_sec):
            self.get_logger().error(f"{label}: /compute_fk unavailable.")
            return None

        request = GetPositionFK.Request()
        request.header.frame_id = self._frame_id
        request.fk_link_names = [link_name]
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state

        future = self._fk_client.call_async(request)
        done = Event()
        future.add_done_callback(lambda _: done.set())
        effective_timeout = self._stage_timeout if timeout_sec is None else timeout_sec
        if not done.wait(timeout=effective_timeout):
            future.cancel()
            self.get_logger().error(f"{label}: FK request timed out.")
            return None

        response = future.result()
        if response is None or len(response.pose_stamped) == 0:
            self.get_logger().error(f"{label}: FK response was empty.")
            return None
        if int(response.error_code.val) != 1:
            self.get_logger().error(f"{label}: FK failed with code={int(response.error_code.val)}.")
            return None
        return response.pose_stamped[0]

    def _stage_base_motion_limits(self, stage: PickStage) -> tuple[bool, float, float]:
        if stage == PickStage.PLAN_TO_PRE_INSERT:
            extra_base_yaw_deg = 0.0
            target_pose = self._task_target_pose()
            if target_pose is not None:
                if target_pose.pose.position.z < 0.45:
                    extra_base_yaw_deg = 120.0
                elif target_pose.pose.position.z < 0.60:
                    extra_base_yaw_deg = 35.0
            return (
                self._pre_insert_limit_base_motion,
                self._pre_insert_base_translation_limit_m,
                math.radians(self._pre_insert_base_rotation_limit_deg + extra_base_yaw_deg),
            )
        if stage == PickStage.INSERT_AND_SUCTION:
            return (
                self._insert_limit_base_motion,
                self._insert_base_translation_limit_m,
                math.radians(self._insert_base_rotation_limit_deg),
            )
        return False, 0.0, 0.0

    def _effective_r1_stage_motion_limit_deg(self, stage: PickStage) -> float:
        limit_deg = self._r1_stage_motion_limit_deg
        if stage == PickStage.PLAN_TO_PRE_INSERT:
            target_pose = self._task_target_pose()
            if target_pose is not None:
                if target_pose.pose.position.z < 0.45:
                    limit_deg += 45.0
                elif target_pose.pose.position.z < 0.60:
                    limit_deg += 15.0
        return limit_deg

    def _seeded_goal_respects_joint_preferences(
        self,
        goal_joint_state: JointState,
        seed_joint_state: JointState,
        label: str,
        stage: PickStage = PickStage.PLAN_TO_PRE_INSERT,
    ) -> bool:
        enforce_base_motion, base_translation_limit, base_theta_limit = self._stage_base_motion_limits(stage)
        if enforce_base_motion:
            base_planar_extent = joint_state_base_planar_motion_extent(seed_joint_state, goal_joint_state)
            if base_planar_extent > base_translation_limit:
                self.get_logger().warning(
                    f"{label}: rejecting seeded goal because base planar stage motion would reach "
                    f"{base_planar_extent:.3f} m "
                    f"(limit {base_translation_limit:.3f} m)"
                )
                return False

            base_theta_extent = joint_state_motion_extent(
                seed_joint_state,
                goal_joint_state,
                "base_theta",
                revolute_joint_names={"base_theta"},
            )
            if base_theta_extent > base_theta_limit:
                self.get_logger().warning(
                    f"{label}: rejecting seeded goal because base yaw stage motion would reach "
                    f"{math.degrees(base_theta_extent):.1f} deg "
                    f"(limit {math.degrees(base_theta_limit):.1f} deg)"
                )
                return False

        effective_r1_limit_deg = self._effective_r1_stage_motion_limit_deg(stage)
        r1_motion_limit = math.radians(effective_r1_limit_deg)
        r1_motion_extent = joint_state_motion_extent(
            seed_joint_state,
            goal_joint_state,
            "r1_joint",
            revolute_joint_names={"r1_joint"},
        )
        if r1_motion_extent > r1_motion_limit:
            start_deg = math.degrees(joint_position(seed_joint_state, "r1_joint", 0.0))
            self.get_logger().warning(
                f"{label}: rejecting seeded goal because r1 stage motion would reach "
                f"{math.degrees(r1_motion_extent):.1f} deg from start {start_deg:.1f} deg "
                f"(limit {effective_r1_limit_deg:.1f} deg)"
            )
            return False
        return True

    def _insert_local_window_metrics(
        self,
        start_joint_state: JointState,
        target_pose: PoseStamped,
        label: str,
    ) -> tuple[bool, str]:
        current_pose = self._call_fk_service(start_joint_state, self._pose_link, label)
        if current_pose is None:
            return False, "FK unavailable"

        position_error, orientation_error = pose_error(target_pose.pose, current_pose.pose)
        projection_error, axial_error, _ = prepare_alignment_errors(target_pose.pose, current_pose.pose)
        approach_axis = rotate_vector_by_quaternion(
            (1.0, 0.0, 0.0),
            (
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ),
        )
        axis_norm = math.sqrt(approach_axis[0] ** 2 + approach_axis[1] ** 2 + approach_axis[2] ** 2)
        signed_axial_gap = 0.0
        if axis_norm > 1e-6:
            approach_unit = (
                approach_axis[0] / axis_norm,
                approach_axis[1] / axis_norm,
                approach_axis[2] / axis_norm,
            )
            delta = (
                target_pose.pose.position.x - current_pose.pose.position.x,
                target_pose.pose.position.y - current_pose.pose.position.y,
                target_pose.pose.position.z - current_pose.pose.position.z,
            )
            signed_axial_gap = (
                delta[0] * approach_unit[0]
                + delta[1] * approach_unit[1]
                + delta[2] * approach_unit[2]
            )

        is_local = (
            position_error <= self._insert_local_position_threshold
            and axial_error <= self._insert_local_axial_threshold
            and projection_error <= self._insert_local_lateral_threshold
            and orientation_error <= self._insert_local_orientation_threshold
        )
        message = (
            f"position_error={position_error:.4f}, axial_gap={signed_axial_gap:.4f}, "
            f"lateral_error={projection_error:.4f}, orientation_error={orientation_error:.4f}"
        )
        return is_local, message

    def _call_ik_service(
        self,
        group_name: str,
        target_pose: PoseStamped,
        start_joint_state: JointState,
        label: str,
    ) -> Optional[JointState]:
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{label}: /compute_ik unavailable.")
            return None

        resolved_pose, resolved_link_name = self._resolve_cartesian_goal_pose(target_pose, self._pose_link)
        request = GetPositionIK.Request()
        ik_request = PositionIKRequest()
        ik_request.group_name = group_name
        ik_request.robot_state = RobotState()
        ik_request.robot_state.joint_state = merge_joint_states(self._current_joint_state, start_joint_state)
        ik_request.avoid_collisions = True
        ik_request.ik_link_name = resolved_link_name
        ik_request.pose_stamped = resolved_pose
        ik_request.timeout = duration_to_msg(self._final_seeded_ik_timeout)
        request.ik_request = ik_request

        future = self._ik_client.call_async(request)
        done = Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self._stage_timeout):
            future.cancel()
            self.get_logger().error(f"{label}: IK request timed out.")
            return None

        response = future.result()
        if response is None:
            self.get_logger().error(f"{label}: IK response unavailable.")
            return None
        if int(response.error_code.val) != 1:
            self.get_logger().warning(f"{label}: IK failed with code={int(response.error_code.val)}.")
            return None
        return response.solution.joint_state

    def _check_state_validity(
        self,
        joint_state: JointState,
        group_name: str,
        label: str,
    ) -> bool:
        if not self._state_validity_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{label}: /check_state_validity unavailable.")
            return False

        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = joint_state
        request.group_name = group_name

        future = self._state_validity_client.call_async(request)
        done = Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self._stage_timeout):
            future.cancel()
            self.get_logger().error(f"{label}: state validity request timed out.")
            return False

        response = future.result()
        return bool(response is not None and response.valid)

    def _validate_trajectory_states(
        self,
        trajectory: JointTrajectory,
        seed_joint_state: JointState,
        group_name: str,
        label: str,
        allow_invalid_start: bool = False,
        allowed_invalid_prefix_points: int = 0,
    ) -> bool:
        merged_seed = merge_joint_states(self._current_joint_state, seed_joint_state)
        start_valid = self._check_state_validity(merged_seed, group_name, f"{label}_start")
        if not start_valid and not allow_invalid_start:
            return False
        if not start_valid and allow_invalid_start:
            self.get_logger().info(
                f"{label}: tolerating invalid start state while checking whether the first "
                f"{allowed_invalid_prefix_points} interpolation point(s) move away from collision."
            )

        for index, point in enumerate(trajectory.points):
            sample_joint_state = JointState()
            sample_joint_state.name = list(trajectory.joint_names)
            sample_joint_state.position = list(point.positions)
            merged_state = merge_joint_states(merged_seed, sample_joint_state)
            if not self._check_state_validity(merged_state, group_name, f"{label}_{index}"):
                if allow_invalid_start and index <= allowed_invalid_prefix_points:
                    continue
                return False
        return True

    def _call_cartesian_service(
        self,
        group_name: str,
        target_pose: PoseStamped,
        start_joint_state: JointState,
        label: str,
        max_step: float,
        revolute_jump_threshold: float,
    ):
        if not self._cartesian_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f"{label}: /compute_cartesian_path unavailable.")
            return None

        request = GetCartesianPath.Request()
        resolved_pose, resolved_link_name = self._resolve_cartesian_goal_pose(target_pose, self._pose_link)
        request.header.frame_id = resolved_pose.header.frame_id or self._frame_id
        request.start_state = RobotState()
        request.start_state.joint_state = merge_joint_states(self._current_joint_state, start_joint_state)
        request.group_name = group_name
        request.link_name = resolved_link_name
        request.waypoints = [clone_pose(resolved_pose.pose)]
        request.max_step = max_step
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = max(0.0, revolute_jump_threshold)
        request.avoid_collisions = True

        future = self._cartesian_client.call_async(request)
        done = Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self._stage_timeout):
            future.cancel()
            self.get_logger().error(f"{label}: Cartesian planning request timed out.")
            return None
        return future.result()

    def _resolve_cartesian_goal_pose(self, target_pose: PoseStamped, pose_link: str) -> tuple[PoseStamped, str]:
        if (
            pose_link != self._tool_reference_link
            or not self._ik_pose_link
            or self._ik_pose_link == self._tool_reference_link
        ):
            return clone_pose_stamped(target_pose), pose_link

        resolved = clone_pose_stamped(target_pose)
        offset_world = rotate_vector_by_quaternion(
            self._tool_to_ik_offset,
            (
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ),
        )
        resolved.pose.position.x -= offset_world[0]
        resolved.pose.position.y -= offset_world[1]
        resolved.pose.position.z -= offset_world[2]
        return resolved, self._ik_pose_link

    def _compute_base_goal(self, target_pose: PoseStamped) -> PoseStamped:
        approach_axis = rotate_vector_by_quaternion(
            (1.0, 0.0, 0.0),
            (
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ),
        )
        approach_xy = normalize_xy((approach_axis[0], approach_axis[1], 0.0), fallback=(1.0, 0.0, 0.0))
        lateral_xy = (-approach_xy[1], approach_xy[0], 0.0)

        goal = PoseStamped()
        goal.header.frame_id = target_pose.header.frame_id or self._frame_id
        effective_base_standoff = self._compute_effective_base_standoff(target_pose)
        effective_lateral_offset = self._base_lateral_offset
        if target_pose.pose.position.z < 0.60:
            effective_lateral_offset *= 0.60
        if target_pose.pose.position.z < 0.45:
            effective_lateral_offset *= 0.50
        goal.pose.position.x = (
            target_pose.pose.position.x
            - effective_base_standoff * approach_xy[0]
            + effective_lateral_offset * lateral_xy[0]
        )
        goal.pose.position.y = (
            target_pose.pose.position.y
            - effective_base_standoff * approach_xy[1]
            + effective_lateral_offset * lateral_xy[1]
        )
        goal.pose.position.z = 0.0

        target_dx = target_pose.pose.position.x - goal.pose.position.x
        target_dy = target_pose.pose.position.y - goal.pose.position.y
        yaw = math.atan2(target_dy, target_dx) + math.radians(self._base_yaw_offset_deg)
        goal.pose.orientation = yaw_to_quaternion(yaw)
        return goal

    def _shelf_lateral_offset(self, target_pose: PoseStamped) -> float:
        if self._shelf_lateral_axis_xy is None:
            return target_pose.pose.position.y - self._shelf_center_y
        delta_x = target_pose.pose.position.x - self._shelf_center_x
        delta_y = target_pose.pose.position.y - self._shelf_center_y
        return delta_x * self._shelf_lateral_axis_xy[0] + delta_y * self._shelf_lateral_axis_xy[1]

    def _entry_plane_signed_distance(self, pose: PoseStamped) -> float:
        if self._shelf_entry_x is None or self._shelf_entry_y is None or self._shelf_inward_axis_xy is None:
            cabinet_entry_x = self._shelf_center_x - self._shelf_depth / 2.0
            return pose.pose.position.x - cabinet_entry_x
        delta_x = pose.pose.position.x - self._shelf_entry_x
        delta_y = pose.pose.position.y - self._shelf_entry_y
        return delta_x * self._shelf_inward_axis_xy[0] + delta_y * self._shelf_inward_axis_xy[1]

    def _compute_effective_base_standoff(self, target_pose: PoseStamped) -> float:
        if not self._adaptive_workspace_enabled:
            return self._base_standoff
        depth_delta = self._shelf_depth - self._base_standoff_reference_depth
        lateral_offset = abs(self._shelf_lateral_offset(target_pose))
        dynamic = (
            self._base_standoff
            - depth_delta * self._base_standoff_depth_gain
            + lateral_offset * self._base_standoff_lateral_gain
        )
        if target_pose.pose.position.z < 0.60:
            dynamic += 0.04
        if target_pose.pose.position.z < 0.45:
            dynamic += 0.06
        return max(self._base_standoff_min, min(self._base_standoff_max, dynamic))

    def _candidate_prepare_offsets(self) -> list[float]:
        base_distance = max(0.03, self._prepare_target_distance)
        offsets = [
            base_distance,
            max(0.03, base_distance - 0.02),
            base_distance + 0.02,
            base_distance + 0.04,
        ]
        unique_offsets: list[float] = []
        for value in offsets:
            rounded = round(value, 3)
            if rounded not in unique_offsets:
                unique_offsets.append(rounded)
        return unique_offsets

    def _compute_prepare_target_distance(self, target_pose: PoseStamped) -> float:
        effective_pre_insert_offset = self._compute_effective_pre_insert_offset(target_pose)
        if not self._use_entry_plane_prepare:
            return effective_pre_insert_offset

        approach_axis = rotate_vector_by_quaternion(
            (1.0, 0.0, 0.0),
            (
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ),
        )
        axis_norm = math.sqrt(approach_axis[0] ** 2 + approach_axis[1] ** 2 + approach_axis[2] ** 2)
        if axis_norm <= 1e-6:
            return effective_pre_insert_offset
        approach_unit = (
            approach_axis[0] / axis_norm,
            approach_axis[1] / axis_norm,
            approach_axis[2] / axis_norm,
        )

        plane_point_x = self._shelf_entry_x if self._shelf_entry_x is not None else self._shelf_center_x - self._shelf_depth / 2.0
        plane_point_y = self._shelf_entry_y if self._shelf_entry_y is not None else self._shelf_center_y
        inward_axis = self._shelf_inward_axis_xy if self._shelf_inward_axis_xy is not None else (1.0, 0.0)
        axis_dot_plane_normal = approach_unit[0] * inward_axis[0] + approach_unit[1] * inward_axis[1]
        if abs(axis_dot_plane_normal) <= 1e-6:
            return effective_pre_insert_offset

        target_to_plane_along_normal = (
            (target_pose.pose.position.x - plane_point_x) * inward_axis[0]
            + (target_pose.pose.position.y - plane_point_y) * inward_axis[1]
        )
        distance_to_entry_plane = target_to_plane_along_normal / axis_dot_plane_normal
        if distance_to_entry_plane < 0.0:
            return effective_pre_insert_offset
        return max(effective_pre_insert_offset, distance_to_entry_plane + self._cabinet_entry_margin)

    def _compute_effective_pre_insert_offset(self, target_pose: PoseStamped) -> float:
        if not self._adaptive_workspace_enabled:
            return self._pre_insert_offset
        depth_delta = max(0.0, self._shelf_depth - self._pre_insert_offset_reference_depth)
        lateral_offset = abs(self._shelf_lateral_offset(target_pose))
        dynamic = (
            self._pre_insert_offset
            + depth_delta * self._pre_insert_offset_depth_gain
            + lateral_offset * self._pre_insert_offset_lateral_gain
        )
        if target_pose.pose.position.z < 0.60:
            dynamic += 0.02
        if target_pose.pose.position.z < 0.45:
            dynamic += 0.03
        return max(self._pre_insert_offset_min, min(self._pre_insert_offset_max, dynamic))

    def _build_base_trajectory(self, base_goal_pose: PoseStamped, duration_sec: float) -> JointTrajectory:
        current_positions = {name: pos for name, pos in zip(self._current_joint_state.name, self._current_joint_state.position)}
        yaw = quaternion_to_yaw(
            base_goal_pose.pose.orientation.x,
            base_goal_pose.pose.orientation.y,
            base_goal_pose.pose.orientation.z,
            base_goal_pose.pose.orientation.w,
        )
        joint_state = JointState()
        joint_state.name = ["base_x", "base_y", "base_theta"]
        joint_state.position = [
            current_positions.get("base_x", base_goal_pose.pose.position.x),
            current_positions.get("base_y", base_goal_pose.pose.position.y),
            current_positions.get("base_theta", 0.0),
        ]
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_state.name)

        start_point = JointTrajectoryPoint()
        start_point.positions = list(joint_state.position)
        start_point.time_from_start = duration_to_msg(0.0)

        end_point = JointTrajectoryPoint()
        end_point.positions = [base_goal_pose.pose.position.x, base_goal_pose.pose.position.y, yaw]
        end_point.time_from_start = duration_to_msg(duration_sec)

        trajectory.points = [start_point, end_point]
        return trajectory

    def _build_lift_trajectory(self, lift_height: float, duration_sec: float) -> JointTrajectory:
        current_positions = joint_state_positions(self._current_joint_state)
        joint_state = JointState()
        joint_state.name = ["raise_joint"]
        joint_state.position = [lift_height]
        return joint_state_to_trajectory(
            joint_state,
            duration_sec,
            start_positions=current_positions,
        )

    def _offset_pose_along_tool_axis(self, pose: PoseStamped, distance: float) -> PoseStamped:
        approach_vector = rotate_vector_by_quaternion(
            (1.0, 0.0, 0.0),
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ),
        )
        shifted = clone_pose_stamped(pose)
        shifted.pose.position.x += approach_vector[0] * distance
        shifted.pose.position.y += approach_vector[1] * distance
        shifted.pose.position.z += approach_vector[2] * distance
        return shifted

    def _make_retreat_pose(self, pose: PoseStamped) -> PoseStamped:
        retreat = self._offset_pose_along_tool_axis(pose, distance=-self._retreat_offset)
        retreat.pose.position.z += self._retreat_lift
        return retreat
