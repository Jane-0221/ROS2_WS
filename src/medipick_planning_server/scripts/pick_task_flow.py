#!/usr/bin/python3

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from pick_task_shared import CandidateDebugSample, PickStage, PlannedStage, PrepareCandidate
from pick_task_utils import (
    clone_pose_stamped,
    joint_state_base_planar_motion_extent,
    joint_states_to_interpolated_trajectory,
    joint_state_motion_extent,
    joint_state_from_trajectory_end,
    joint_state_to_trajectory,
    joint_position,
    joint_state_positions,
    merge_joint_states,
    multiply_quaternions,
    pose_error,
    pose_from_base_joint_state,
    predict_executed_final_joint_state,
    prepare_alignment_errors,
    rotate_vector_by_quaternion,
    reverse_joint_trajectory,
    trajectory_base_planar_motion_extent,
    trajectory_joint_abs_max,
    trajectory_joint_motion_extent,
    trajectory_motion_metrics,
)


class PickTaskFlowMixin:
    def _prepare_pose_key(self, prepare_pose: PoseStamped) -> tuple[float, float, float, float]:
        return (
            round(prepare_pose.pose.position.x, 3),
            round(prepare_pose.pose.position.y, 3),
            round(prepare_pose.pose.position.z, 3),
            round(
                math.atan2(
                    2.0
                    * (
                        prepare_pose.pose.orientation.w * prepare_pose.pose.orientation.z
                        + prepare_pose.pose.orientation.x * prepare_pose.pose.orientation.y
                    ),
                    1.0
                    - 2.0
                    * (
                        prepare_pose.pose.orientation.y * prepare_pose.pose.orientation.y
                        + prepare_pose.pose.orientation.z * prepare_pose.pose.orientation.z
                    ),
                ),
                3,
            ),
        )

    def _pre_insert_group_attempt_order(self, preferred_group_name: str = "") -> list[str]:
        groups: list[str] = []
        if preferred_group_name:
            groups.append(preferred_group_name)
        if self._pre_insert_try_arm_first and self._pre_insert_arm_group_name:
            groups.append(self._effective_group_name(PickStage.PLAN_TO_PRE_INSERT, self._pre_insert_arm_group_name))
        groups.append(self._effective_group_name(PickStage.PLAN_TO_PRE_INSERT, self._pre_insert_group_name))

        unique_groups: list[str] = []
        for group_name in groups:
            if group_name and group_name not in unique_groups:
                unique_groups.append(group_name)
        return unique_groups

    def _target_lateral_offset(self, target_pose: PoseStamped) -> float:
        return self._shelf_lateral_offset(target_pose)

    def _pending_stage(self, stage: PickStage, message: str) -> PlannedStage:
        return PlannedStage(
            stage=stage,
            success=False,
            message=message,
            planning_time=0.0,
            trajectory=JointTrajectory(),
            final_joint_state=JointState(),
        )

    def _refresh_task_geometry(self) -> bool:
        task_target_pose = self._task_target_pose()
        if task_target_pose is None:
            return False

        target_pose = clone_pose_stamped(task_target_pose)
        self._prepare_target_distance = self._compute_prepare_target_distance(target_pose)
        self._base_goal_pose = self._compute_base_goal(target_pose)
        self._target_lift_height = self._compute_target_lift_height(target_pose)
        self._lift_target_center = self._compute_lift_stage_target_height(
            target_pose,
            self._target_lift_height,
        )
        self._lift_target_band_min, self._lift_target_band_max = self._compute_lift_target_band(
            self._lift_target_center
        )
        self._prepare_pose = self._build_prepare_pose(target_pose, self._prepare_target_distance, 0.0, 0.0, 0.0)
        self._retreat_pose = self._make_retreat_pose(target_pose)
        self._publish_stage_path()
        return True

    def _compute_target_lift_height(self, target_pose: PoseStamped) -> float:
        if self._lift_use_end_effector_height_alignment:
            reference_joint_state = merge_joint_states(self._current_joint_state, self._stow_joint_state())
            reference_raise = joint_position(reference_joint_state, "raise_joint", self._lift_reference_raise_joint)
            reference_pose = self._call_fk_service(
                reference_joint_state,
                self._pose_link,
                "lift_reference_tool_height",
                timeout_sec=self._acquire_fk_timeout,
                wait_for_service_timeout_sec=self._acquire_service_probe_timeout,
            )
            if reference_pose is not None:
                desired_tool_z = target_pose.pose.position.z + self._lift_end_effector_target_offset
                raw_height = reference_raise + (desired_tool_z - reference_pose.pose.position.z) * self._lift_height_gain
                return max(self._lift_height_min, min(self._lift_height_max, raw_height))

        delta_z = target_pose.pose.position.z - self._lift_reference_target_z
        raw_height = self._lift_reference_raise_joint + delta_z * self._lift_height_gain
        return max(self._lift_height_min, min(self._lift_height_max, raw_height))

    def _compute_lift_stage_target_height(self, target_pose: PoseStamped, desired_height: float) -> float:
        target_height = desired_height
        if (
            target_pose.pose.position.z < self._low_shelf_target_z_threshold
            and desired_height < self._lift_stage_stow_safe_min_height
        ):
            target_height = self._lift_stage_stow_safe_min_height
            self.get_logger().info(
                "lift_align: low shelf target keeps stowed arm above the self-collision floor "
                f"({desired_height:.3f} -> {target_height:.3f} m)."
            )
        return max(self._lift_height_min, min(self._lift_height_max, target_height))

    def _compute_lift_target_band(self, target_center: float) -> tuple[float, float]:
        band_min = max(self._lift_height_min, target_center - self._lift_band_half_width)
        band_max = min(self._lift_height_max, target_center + self._lift_band_half_width)
        return (band_min, band_max)

    def _next_lift_segment(self) -> tuple[float, float]:
        current_height = joint_position(self._current_joint_state, "raise_joint", self._lift_height_min)
        target_height = self._lift_target_center if self._lift_target_center is not None else current_height
        delta = target_height - current_height
        step_limit = max(1e-3, self._lift_segment_step_m)
        step = max(-step_limit, min(step_limit, delta))
        next_height = current_height + step
        duration = max(
            self._lift_segment_min_duration,
            abs(step) / max(1e-3, self._lift_segment_speed_mps),
        )
        return next_height, duration

    def _build_prepare_pose(
        self,
        target_pose: PoseStamped,
        retreat_distance: float,
        lateral_offset: float,
        vertical_offset: float,
        yaw_offset_deg: float,
    ) -> PoseStamped:
        prepare_pose = self._offset_pose_along_tool_axis(target_pose, distance=-retreat_distance)
        lateral_axis = rotate_vector_by_quaternion(
            (0.0, 1.0, 0.0),
            (
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ),
        )
        vertical_axis = rotate_vector_by_quaternion(
            (0.0, 0.0, 1.0),
            (
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ),
        )
        prepare_pose.pose.position.x += lateral_axis[0] * lateral_offset + vertical_axis[0] * vertical_offset
        prepare_pose.pose.position.y += lateral_axis[1] * lateral_offset + vertical_axis[1] * vertical_offset
        prepare_pose.pose.position.z += lateral_axis[2] * lateral_offset + vertical_axis[2] * vertical_offset
        if abs(yaw_offset_deg) > 1e-6:
            yaw_quat = (
                0.0,
                0.0,
                math.sin(math.radians(yaw_offset_deg) * 0.5),
                math.cos(math.radians(yaw_offset_deg) * 0.5),
            )
            current_quat = (
                prepare_pose.pose.orientation.x,
                prepare_pose.pose.orientation.y,
                prepare_pose.pose.orientation.z,
                prepare_pose.pose.orientation.w,
            )
            rotated_quat = multiply_quaternions(yaw_quat, current_quat)
            prepare_pose.pose.orientation.x = rotated_quat[0]
            prepare_pose.pose.orientation.y = rotated_quat[1]
            prepare_pose.pose.orientation.z = rotated_quat[2]
            prepare_pose.pose.orientation.w = rotated_quat[3]
        return prepare_pose

    def _candidate_prepare_pose_specs(self) -> list[tuple[float, float, float, float]]:
        distances = self._candidate_prepare_offsets()
        target_pose = self._task_target_pose()
        if target_pose is None:
            return []

        lateral_offset = self._target_lateral_offset(target_pose)
        bias_sign = 0.0
        if abs(lateral_offset) > 1e-6:
            bias_sign = -1.0 if lateral_offset > 0.0 else 1.0
        center_bias = min(self._pre_insert_center_bias_max, abs(lateral_offset) * 0.5)
        biased_lateral = bias_sign * center_bias
        biased_yaw = bias_sign * self._pre_insert_center_bias_yaw_deg if bias_sign != 0.0 else 0.0
        sweep_lateral = min(
            self._pre_insert_center_bias_max,
            max(abs(biased_lateral), 0.02),
        )
        sweep_yaw = max(abs(biased_yaw), self._pre_insert_center_bias_yaw_deg)

        prioritized_specs: list[tuple[float, float, float, float]] = []
        if target_pose.pose.position.z < 0.45:
            moderate_distance = max(distances[0], self._prepare_target_distance + 0.04)
            farther_distance = max(distances[-1], self._prepare_target_distance + 0.10)
            widest_distance = farther_distance + 0.06
            signed_yaw = sweep_yaw if bias_sign >= 0.0 else -sweep_yaw
            wider_yaw = max(12.0, sweep_yaw + 8.0)

            prioritized_specs.extend(
                [
                    (moderate_distance, biased_lateral, 0.10, 0.0),
                    (moderate_distance, 0.0, 0.10, 0.0),
                    (farther_distance, biased_lateral, 0.16, 0.0),
                    (widest_distance, 0.0, 0.16, 0.0),
                    (farther_distance, biased_lateral, 0.22, 0.0),
                    (widest_distance, 0.0, 0.22, 0.0),
                ]
            )
            if sweep_lateral > 1e-6:
                prioritized_specs.extend(
                    [
                        (moderate_distance, sweep_lateral, 0.10, signed_yaw),
                        (moderate_distance, -sweep_lateral, 0.10, -signed_yaw),
                        (farther_distance, sweep_lateral, 0.16, signed_yaw),
                        (farther_distance, -sweep_lateral, 0.16, -signed_yaw),
                        (widest_distance, sweep_lateral, 0.22, wider_yaw),
                        (widest_distance, -sweep_lateral, 0.22, -wider_yaw),
                    ]
                )
        elif target_pose.pose.position.z < 0.60:
            base_distance = max(distances[0], self._prepare_target_distance + 0.02)
            for vertical_offset in (0.08, 0.12):
                prioritized_specs.append((base_distance, biased_lateral, vertical_offset, 0.0))
                prioritized_specs.append((base_distance + 0.04, 0.0, vertical_offset, 0.0))
                if sweep_lateral > 1e-6:
                    prioritized_specs.append((base_distance + 0.02, sweep_lateral, vertical_offset, 0.0))
                    prioritized_specs.append((base_distance + 0.02, -sweep_lateral, vertical_offset, 0.0))

        for index, retreat_distance in enumerate(distances):
            prioritized_specs.append((retreat_distance, biased_lateral, 0.0, 0.0))
            prioritized_specs.append((retreat_distance, 0.0, 0.0, 0.0))

            if sweep_lateral > 1e-6:
                signed_yaw = sweep_yaw if bias_sign >= 0.0 else -sweep_yaw
                prioritized_specs.append((retreat_distance, sweep_lateral, 0.0, signed_yaw))
                prioritized_specs.append((retreat_distance, -sweep_lateral, 0.0, -signed_yaw))

            if index == 0 and sweep_lateral > 1e-6:
                prioritized_specs.append((retreat_distance, sweep_lateral, 0.0, 0.0))
                prioritized_specs.append((retreat_distance, -sweep_lateral, 0.0, 0.0))

        unique_specs: list[tuple[float, float, float, float]] = []
        for spec in prioritized_specs:
            rounded = tuple(round(value, 4) for value in spec)
            if rounded not in unique_specs:
                unique_specs.append(rounded)

        if self._pre_insert_first_candidate_only and unique_specs:
            return [unique_specs[0]]

        limit = max(1, self._pre_insert_candidate_count)
        return unique_specs[:limit]

    def _select_prepare_candidates(self) -> list[PrepareCandidate]:
        task_target_pose = self._task_target_pose()
        if task_target_pose is None:
            return []

        if self._pre_insert_select_pose_only:
            specs = self._candidate_prepare_pose_specs()
            if not specs:
                return []
            debug_samples: list[CandidateDebugSample] = []
            accepted_candidates: list[PrepareCandidate] = []
            group_name = self._effective_group_name(PickStage.PLAN_TO_PRE_INSERT, self._pre_insert_group_name)
            arm_group_name = self._effective_group_name(PickStage.PLAN_TO_PRE_INSERT, self._pre_insert_arm_group_name)
            whole_body_group_name = ""
            if self._discrete_lift_mode and self._pre_insert_group_name and self._pre_insert_group_name != group_name:
                whole_body_group_name = self._pre_insert_group_name

            for retreat_distance, lateral_offset, vertical_offset, yaw_offset_deg in specs:
                prepare_pose = self._build_prepare_pose(
                    task_target_pose,
                    retreat_distance=retreat_distance,
                    lateral_offset=lateral_offset,
                    vertical_offset=vertical_offset,
                    yaw_offset_deg=yaw_offset_deg,
                )
                accepted = self._pre_insert_pose_is_outside_cabinet(prepare_pose)
                label = "outside"
                score = abs(lateral_offset) + 0.2 * abs(yaw_offset_deg) + abs(retreat_distance - self._prepare_target_distance)
                preferred_group_name = group_name
                prepare_result = self._pending_stage(
                    PickStage.PLAN_TO_PRE_INSERT,
                    "Planning deferred to plan stage.",
                )

                if accepted:
                    ik_solution = self._call_ik_service(
                        group_name=group_name,
                        target_pose=prepare_pose,
                        start_joint_state=self._current_joint_state,
                        label="pre_insert_select_ik",
                    )
                    fallback_whole_body = False
                    if ik_solution is None or not self._check_state_validity(
                        ik_solution,
                        group_name,
                        "pre_insert_select_validity",
                    ):
                        accepted = False
                        label = "ik/collision"
                        if whole_body_group_name:
                            whole_body_solution = self._call_ik_service(
                                group_name=whole_body_group_name,
                                target_pose=prepare_pose,
                                start_joint_state=self._current_joint_state,
                                label="pre_insert_select_whole_body_ik",
                            )
                            if (
                                whole_body_solution is not None
                                and self._check_state_validity(
                                    whole_body_solution,
                                    whole_body_group_name,
                                    "pre_insert_select_whole_body_validity",
                                )
                                and self._seeded_goal_respects_joint_preferences(
                                    whole_body_solution,
                                    self._current_joint_state,
                                    "pre_insert_select_whole_body_goal_preference",
                                    stage=PickStage.PLAN_TO_PRE_INSERT,
                                )
                            ):
                                ik_solution = whole_body_solution
                                accepted = True
                                fallback_whole_body = True
                                preferred_group_name = whole_body_group_name
                                label = "whole-body"
                    if accepted:
                        r1_extent = joint_state_motion_extent(
                            self._current_joint_state,
                            ik_solution,
                            "r1_joint",
                            revolute_joint_names={"r1_joint"},
                        )
                        score += math.degrees(r1_extent) * 0.01
                        if (not fallback_whole_body) and not self._seeded_goal_respects_joint_preferences(
                            ik_solution,
                            self._current_joint_state,
                            "pre_insert_select_goal_preference",
                            stage=PickStage.PLAN_TO_PRE_INSERT,
                        ):
                            accepted = False
                            label = "r1 limit"
                        else:
                            if fallback_whole_body:
                                score += joint_state_base_planar_motion_extent(
                                    self._current_joint_state,
                                    ik_solution,
                                )
                                label = "whole-body"
                            else:
                                label = "seeded ok"
                            if arm_group_name and arm_group_name != group_name:
                                arm_only_solution = self._call_ik_service(
                                    group_name=arm_group_name,
                                    target_pose=prepare_pose,
                                    start_joint_state=self._current_joint_state,
                                    label="pre_insert_select_arm_ik",
                                )
                                if (
                                    arm_only_solution is not None
                                    and self._check_state_validity(
                                        arm_only_solution,
                                        arm_group_name,
                                        "pre_insert_select_arm_validity",
                                    )
                                    and self._seeded_goal_respects_joint_preferences(
                                        arm_only_solution,
                                        self._current_joint_state,
                                        "pre_insert_select_arm_goal_preference",
                                        stage=PickStage.PLAN_TO_PRE_INSERT,
                                    )
                                ):
                                    arm_insert_group = self._effective_group_name(
                                        PickStage.INSERT_AND_SUCTION,
                                        self._insert_group_name,
                                    )
                                    final_arm_solution = self._call_ik_service(
                                        group_name=arm_insert_group,
                                        target_pose=task_target_pose,
                                        start_joint_state=arm_only_solution,
                                        label="pre_insert_select_arm_insert_ik",
                                    )
                                    if (
                                        final_arm_solution is not None
                                        and self._check_state_validity(
                                            final_arm_solution,
                                            arm_insert_group,
                                            "pre_insert_select_arm_insert_validity",
                                        )
                                        and self._seeded_goal_respects_joint_preferences(
                                            final_arm_solution,
                                            arm_only_solution,
                                            "pre_insert_select_arm_insert_goal_preference",
                                            stage=PickStage.INSERT_AND_SUCTION,
                                        )
                                    ):
                                        score -= 0.35
                                        label = "arm-first"
                                        preferred_group_name = arm_group_name

                if accepted:
                    prepare_result, selected_group_name, group_failures = self._plan_pre_insert_candidate_from_state(
                        prepare_pose=prepare_pose,
                        preferred_group_name=preferred_group_name,
                        start_joint_state=self._current_joint_state,
                        log_prefix="pre_insert_select",
                    )
                    if prepare_result is None:
                        accepted = False
                        label = group_failures[-1] if group_failures else "prepare plan"
                    else:
                        preferred_group_name = selected_group_name or preferred_group_name
                        score += 0.5 * prepare_result.planning_time

                debug_samples.append(
                    CandidateDebugSample(
                        base_goal_pose=self._candidate_marker_pose(prepare_pose),
                        lift_height=self._lift_target_center or 0.0,
                        score=score if accepted else 1e5,
                        accepted=accepted,
                        label=label,
                    )
                )
                if self._prepare_pose_key(prepare_pose) in self._failed_prepare_pose_keys:
                    continue
                if accepted:
                    accepted_candidates.append(
                        PrepareCandidate(
                            score=score,
                            prepare_pose=prepare_pose,
                            base_goal_pose=self._candidate_marker_pose(prepare_pose),
                            lift_height=self._lift_target_center or 0.0,
                            prepare_result=prepare_result,
                            final_result=self._pending_stage(
                                PickStage.INSERT_AND_SUCTION,
                                "Insert not evaluated yet.",
                            ),
                            preferred_group_name=preferred_group_name,
                        )
                    )

            self._candidate_debug_samples = debug_samples
            accepted_candidates.sort(key=lambda candidate: candidate.score)
            return accepted_candidates[: max(1, self._pre_insert_candidate_count)]

        start_joint_state = self._current_joint_state
        accepted_candidates: list[PrepareCandidate] = []
        debug_samples: list[CandidateDebugSample] = []
        evaluations = 0

        group_name = self._effective_group_name(PickStage.PLAN_TO_PRE_INSERT, self._pre_insert_group_name)

        for retreat_distance, lateral_offset, vertical_offset, yaw_offset_deg in self._candidate_prepare_pose_specs():
            if evaluations >= self._candidate_max_evaluations:
                break
            evaluations += 1
            prepare_pose = self._build_prepare_pose(
                task_target_pose,
                retreat_distance=retreat_distance,
                lateral_offset=lateral_offset,
                vertical_offset=vertical_offset,
                yaw_offset_deg=yaw_offset_deg,
            )
            ik_solution = self._call_ik_service(
                group_name=group_name,
                target_pose=prepare_pose,
                start_joint_state=start_joint_state,
                label="pre_insert_candidate_ik",
            )
            if ik_solution is None or not self._check_state_validity(ik_solution, group_name, "pre_insert_candidate_validity"):
                debug_samples.append(
                    CandidateDebugSample(
                        base_goal_pose=self._candidate_marker_pose(prepare_pose),
                        lift_height=self._lift_target_center or 0.0,
                        score=1e7,
                        accepted=False,
                        label="ik/collision fail",
                    )
                )
                continue

            if not self._pre_insert_pose_is_outside_cabinet(prepare_pose):
                debug_samples.append(
                    CandidateDebugSample(
                        base_goal_pose=self._candidate_marker_pose(prepare_pose),
                        lift_height=self._lift_target_center or 0.0,
                        score=1e7,
                        accepted=False,
                        label="inside cabinet",
                    )
                )
                continue
            if self._prepare_pose_key(prepare_pose) in self._failed_prepare_pose_keys:
                debug_samples.append(
                    CandidateDebugSample(
                        base_goal_pose=self._candidate_marker_pose(prepare_pose),
                        lift_height=self._lift_target_center or 0.0,
                        score=1e6,
                        accepted=False,
                        label="previously failed",
                    )
                )
                continue

            prepare_result = self._plan_stage(
                stage=PickStage.PLAN_TO_PRE_INSERT,
                group_name=group_name,
                pose=prepare_pose,
                start_joint_state=start_joint_state,
                planner_id=self._candidate_planner_id,
                allowed_planning_time=self._candidate_planning_time,
                num_planning_attempts=self._candidate_num_planning_attempts,
            )
            if not prepare_result.success:
                debug_samples.append(
                    CandidateDebugSample(
                        base_goal_pose=self._candidate_marker_pose(prepare_pose),
                        lift_height=self._lift_target_center or 0.0,
                        score=1e6,
                        accepted=False,
                        label="plan fail",
                    )
                )
                continue

            if not self._trajectory_respects_joint_preferences(
                prepare_result,
                start_joint_state,
                "pre_insert_candidate_r1_limit",
            ):
                debug_samples.append(
                    CandidateDebugSample(
                        base_goal_pose=self._candidate_marker_pose(prepare_pose),
                        lift_height=self._lift_target_center or 0.0,
                        score=1e5,
                        accepted=False,
                        label="r1 limit",
                    )
                )
                continue

            if not self._prepare_stage_reached(prepare_result, start_joint_state, prepare_pose, "pre_insert_candidate"):
                debug_samples.append(
                    CandidateDebugSample(
                        base_goal_pose=self._candidate_marker_pose(prepare_pose),
                        lift_height=self._lift_target_center or 0.0,
                        score=1e5,
                        accepted=False,
                        label="reach fail",
                    )
                )
                continue

            predicted_prepare_state = predict_executed_final_joint_state(prepare_result.trajectory, start_joint_state)
            final_result = self._plan_insert_stage(predicted_prepare_state)
            if not final_result.success or not self._final_stage_reached(
                final_result,
                predicted_prepare_state,
                task_target_pose,
                "pre_insert_candidate_final",
            ) or not self._trajectory_respects_joint_preferences(
                final_result,
                predicted_prepare_state,
                "insert_candidate_r1_limit",
            ):
                debug_samples.append(
                    CandidateDebugSample(
                        base_goal_pose=self._candidate_marker_pose(prepare_pose),
                        lift_height=self._lift_target_center or 0.0,
                        score=1e4,
                        accepted=False,
                        label="insert fail",
                    )
                )
                continue

            score = self._score_prepare_candidate(prepare_pose, prepare_result, final_result)
            candidate = PrepareCandidate(
                score=score,
                prepare_pose=prepare_pose,
                base_goal_pose=self._candidate_marker_pose(prepare_pose),
                lift_height=self._lift_target_center or 0.0,
                prepare_result=prepare_result,
                final_result=final_result,
                preferred_group_name=group_name,
            )
            debug_samples.append(
                CandidateDebugSample(
                    base_goal_pose=candidate.base_goal_pose,
                    lift_height=candidate.lift_height,
                    score=score,
                    accepted=True,
                    label=f"{score:.2f}",
                )
            )
            accepted_candidates.append(candidate)
            if candidate.score <= self._candidate_good_enough_score:
                break

        self._candidate_debug_samples = debug_samples
        accepted_candidates.sort(key=lambda candidate: candidate.score)
        shortlist_limit = max(1, self._candidate_shortlist_size)
        return accepted_candidates[:shortlist_limit]

    def _select_prepare_candidate(self) -> Optional[PrepareCandidate]:
        candidates = self._select_prepare_candidates()
        if not candidates:
            return None
        return candidates[0]

    def _candidate_marker_pose(self, prepare_pose: PoseStamped) -> PoseStamped:
        marker_pose = clone_pose_stamped(prepare_pose)
        marker_pose.pose.position.z = 0.0
        return marker_pose

    def _pre_insert_pose_is_outside_cabinet(self, prepare_pose: PoseStamped) -> bool:
        signed_distance = self._entry_plane_signed_distance(prepare_pose)
        return signed_distance <= -self._pre_insert_outside_margin

    def _score_prepare_candidate(
        self,
        prepare_pose: PoseStamped,
        prepare_result: PlannedStage,
        final_result: PlannedStage,
    ) -> float:
        projection_error = 0.0
        axial_error = 0.0
        orientation_error = 0.0
        prepare_joint_state = prepare_result.final_joint_state
        if len(prepare_joint_state.name) == 0:
            prepare_joint_state = joint_state_from_trajectory_end(
                self._current_joint_state,
                prepare_result.trajectory,
            )
        achieved_prepare_pose = self._call_fk_service(prepare_joint_state, self._pose_link, "pre_insert_candidate_score")
        if achieved_prepare_pose is not None:
            projection_error, axial_error, orientation_error = prepare_alignment_errors(
                prepare_pose.pose,
                achieved_prepare_pose.pose,
            )

        prepare_motion, _ = trajectory_motion_metrics(
            prepare_result.trajectory,
            {"raise_joint", "r1_joint", "r2_joint", "r3_joint", "r4_joint", "r5_joint", "r6_joint"},
        )
        final_motion, _ = trajectory_motion_metrics(
            final_result.trajectory,
            {"base_x", "base_y", "base_theta", "raise_joint", "r1_joint", "r2_joint", "r3_joint", "r4_joint", "r5_joint", "r6_joint"},
        )
        r1_soft_threshold = math.radians(self._r1_stage_motion_soft_penalty_start_deg)
        prepare_r1_motion = trajectory_joint_motion_extent(
            prepare_result.trajectory,
            self._current_joint_state,
            "r1_joint",
            revolute_joint_names={"r1_joint"},
        )
        final_seed_state = (
            prepare_result.final_joint_state
            if len(prepare_result.final_joint_state.name) > 0
            else predict_executed_final_joint_state(prepare_result.trajectory, self._current_joint_state)
        )
        final_r1_motion = trajectory_joint_motion_extent(
            final_result.trajectory,
            final_seed_state,
            "r1_joint",
            revolute_joint_names={"r1_joint"},
        )
        r1_penalty = 0.0
        if prepare_r1_motion > r1_soft_threshold:
            r1_penalty += 6.0 * (prepare_r1_motion - r1_soft_threshold)
        if final_r1_motion > r1_soft_threshold:
            r1_penalty += 8.0 * (final_r1_motion - r1_soft_threshold)
        return (
            prepare_result.planning_time
            + final_result.planning_time
            + 0.2 * prepare_motion
            + 0.1 * final_motion
            + 25.0 * projection_error
            + 10.0 * axial_error
            + 8.0 * orientation_error
            + r1_penalty
        )

    def _plan_pre_insert_candidate_from_state(
        self,
        prepare_pose: PoseStamped,
        preferred_group_name: str,
        start_joint_state: JointState,
        log_prefix: str,
    ) -> tuple[Optional[PlannedStage], str, list[str]]:
        group_failures: list[str] = []
        selected_group_name = ""

        for group_name in self._pre_insert_group_attempt_order(preferred_group_name):
            used_pose_fallback = False

            def plan_with_pose_goal() -> PlannedStage:
                result = self._plan_stage(
                    stage=PickStage.PLAN_TO_PRE_INSERT,
                    group_name=group_name,
                    pose=prepare_pose,
                    start_joint_state=start_joint_state,
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
                    f"{log_prefix}_pose_retry[{group_name}]: initial pose planning failed "
                    f"('{result.message}'); retrying with {retry_time:.1f}s and "
                    f"{retry_attempts} attempt(s)."
                )
                retry_result = self._plan_stage(
                    stage=PickStage.PLAN_TO_PRE_INSERT,
                    group_name=group_name,
                    pose=prepare_pose,
                    start_joint_state=start_joint_state,
                    planner_id=self._candidate_planner_id,
                    allowed_planning_time=retry_time,
                    num_planning_attempts=retry_attempts,
                )
                if retry_result.success:
                    return retry_result
                retry_result.message = f"{result.message}; retry failed: {retry_result.message}"
                return retry_result

            group_label = f"{log_prefix}_seeded[{group_name}]"
            plan_result = self._plan_stage_via_seeded_ik(
                stage=PickStage.PLAN_TO_PRE_INSERT,
                group_name=group_name,
                pose=prepare_pose,
                start_joint_state=start_joint_state,
                label=group_label,
            )
            if not plan_result.success:
                self.get_logger().info(f"{group_label}: {plan_result.message} Falling back to pose planning.")
                plan_result = plan_with_pose_goal()
                used_pose_fallback = True
            if not plan_result.success:
                group_failures.append(f"{group_name}: {plan_result.message}")
                continue

            if not self._trajectory_respects_joint_preferences(
                plan_result,
                start_joint_state,
                f"{log_prefix}_r1_limit[{group_name}]",
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
                    start_joint_state,
                    f"{log_prefix}_r1_limit[{group_name}]",
                ):
                    group_failures.append(f"{group_name}: joint preference limit")
                    continue

            if not self._prepare_stage_reached(
                plan_result,
                start_joint_state,
                prepare_pose,
                f"{log_prefix}_reach[{group_name}]",
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
                    start_joint_state,
                    prepare_pose,
                    f"{log_prefix}_reach[{group_name}]",
                ):
                    group_failures.append(f"{group_name}: reach check")
                    continue

            selected_group_name = group_name
            return plan_result, selected_group_name, group_failures

        return None, selected_group_name, group_failures

    def _plan_insert_stage(self, start_joint_state: JointState) -> PlannedStage:
        task_target_pose = self._task_target_pose()
        assert task_target_pose is not None
        group_name = self._effective_group_name(PickStage.INSERT_AND_SUCTION, self._insert_group_name)
        fallback_group_name = self._effective_group_name(
            PickStage.INSERT_AND_SUCTION,
            self._insert_fallback_group_name,
        )
        if (
            self._discrete_lift_mode
            and self._insert_fallback_group_name
            and self._insert_fallback_group_name != group_name
        ):
            fallback_group_name = self._insert_fallback_group_name
        local_insert_only, local_window_message = self._insert_local_window_metrics(
            start_joint_state,
            task_target_pose,
            "insert_local_window",
        )
        if local_insert_only:
            self.get_logger().info(
                "insert_local_window: forcing local insert strategy because "
                f"{local_window_message}"
            )
        else:
            self.get_logger().info(
                "insert_local_window: target not yet in strict local window; "
                f"{local_window_message}"
            )
        prefer_whole_body_first = (
            (not local_insert_only)
            and bool(fallback_group_name)
            and fallback_group_name != group_name
        )
        if prefer_whole_body_first:
            self.get_logger().info(
                "insert_local_window: preferring whole-body insert fallback group "
                f"'{fallback_group_name}' before '{group_name}' while outside the strict local window."
            )

        strategy_failures: list[str] = []

        def plan_cartesian() -> PlannedStage:
            return self._plan_cartesian_stage(
                stage=PickStage.INSERT_AND_SUCTION,
                group_name=group_name,
                pose=task_target_pose,
                start_joint_state=start_joint_state,
                max_step=self._final_cartesian_max_step,
                min_fraction=self._final_cartesian_min_fraction,
                revolute_jump_threshold=self._final_cartesian_revolute_jump_threshold,
            )

        def plan_local_seeded() -> PlannedStage:
            return self._plan_stage_via_local_seeded_ik(
                stage=PickStage.INSERT_AND_SUCTION,
                group_name=group_name,
                pose=task_target_pose,
                start_joint_state=start_joint_state,
                label=f"insert_local_seeded[{group_name}]",
            )

        def plan_fallback_local_seeded() -> PlannedStage:
            if not fallback_group_name or fallback_group_name == group_name:
                return PlannedStage(
                    stage=PickStage.INSERT_AND_SUCTION,
                    success=False,
                    message="No distinct local seeded fallback group is configured.",
                    planning_time=0.0,
                    trajectory=JointTrajectory(),
                    final_joint_state=JointState(),
                )
            return self._plan_stage_via_local_seeded_ik(
                stage=PickStage.INSERT_AND_SUCTION,
                group_name=fallback_group_name,
                pose=task_target_pose,
                start_joint_state=start_joint_state,
                label=f"insert_local_seeded[{fallback_group_name}]",
            )

        local_strategies = [
            ("cartesian", plan_cartesian),
            ("seeded local", plan_local_seeded),
        ]
        if self._final_prefer_seeded_ik:
            local_strategies = [
                ("seeded local", plan_local_seeded),
                ("cartesian", plan_cartesian),
            ]
        if fallback_group_name and fallback_group_name != group_name:
            fallback_strategy = (f"seeded local {fallback_group_name}", plan_fallback_local_seeded)
            if prefer_whole_body_first:
                local_strategies = [fallback_strategy] + local_strategies
            else:
                local_strategies.append(fallback_strategy)

        for strategy_name, plan_fn in local_strategies:
            result = plan_fn()
            if result.success:
                if local_insert_only:
                    result.message = (
                        f"{strategy_name} succeeded inside the local insert window "
                        f"({local_window_message})."
                    )
                return result
            strategy_failures.append(f"{strategy_name} failed ({result.message})")
            if self._final_use_cartesian and strategy_name == "cartesian":
                return result

        if local_insert_only and self._insert_force_local_when_close:
            return PlannedStage(
                stage=PickStage.INSERT_AND_SUCTION,
                success=False,
                message=(
                    "Local insert window active; refusing free OMPL fallback after "
                    + "; ".join(strategy_failures)
                ),
                planning_time=0.0,
                trajectory=JointTrajectory(),
                final_joint_state=JointState(),
            )

        primary_ompl_group_name = group_name
        secondary_ompl_group_name = fallback_group_name
        if prefer_whole_body_first and fallback_group_name and fallback_group_name != group_name:
            primary_ompl_group_name = fallback_group_name
            secondary_ompl_group_name = group_name

        primary_ompl_result = self._plan_stage(
            stage=PickStage.INSERT_AND_SUCTION,
            group_name=primary_ompl_group_name,
            pose=task_target_pose,
            start_joint_state=start_joint_state,
            planner_id=self._final_planner_id,
        )
        if primary_ompl_result.success:
            primary_ompl_result.message = (
                "; ".join(strategy_failures) + f"; OMPL group '{primary_ompl_group_name}' succeeded."
                if strategy_failures
                else primary_ompl_result.message
            )
            return primary_ompl_result

        if secondary_ompl_group_name and secondary_ompl_group_name != primary_ompl_group_name:
            fallback_result = self._plan_stage(
                stage=PickStage.INSERT_AND_SUCTION,
                group_name=secondary_ompl_group_name,
                pose=task_target_pose,
                start_joint_state=start_joint_state,
                planner_id=self._final_planner_id,
            )
            if fallback_result.success:
                fallback_result.message = (
                    "; ".join(strategy_failures)
                    + f"; primary OMPL group '{primary_ompl_group_name}' failed ({primary_ompl_result.message}); "
                    f"fallback OMPL group '{secondary_ompl_group_name}' succeeded."
                )
                return fallback_result
            fallback_result.message = (
                "; ".join(strategy_failures)
                + f"; primary OMPL group '{primary_ompl_group_name}' failed ({primary_ompl_result.message}); "
                f"fallback OMPL group '{secondary_ompl_group_name}' failed ({fallback_result.message})"
            )
            return fallback_result

        primary_ompl_result.message = (
            "; ".join(strategy_failures)
            + f"; OMPL fallback failed ({primary_ompl_result.message})"
        )
        return primary_ompl_result

    def _plan_retreat_stage(self, start_joint_state: JointState) -> PlannedStage:
        assert self._retreat_pose is not None
        if self._retreat_retrace_insert_first and len(self._last_insert_trajectory.points) > 0:
            retrace = reverse_joint_trajectory(self._last_insert_trajectory)
            if len(retrace.points) > 0:
                final_joint_state = joint_state_from_trajectory_end(start_joint_state, retrace)
                return PlannedStage(
                    stage=PickStage.SAFE_RETREAT,
                    success=True,
                    message="Reused reverse of executed insert trajectory for retreat.",
                    planning_time=0.0,
                    trajectory=retrace,
                    final_joint_state=final_joint_state,
                )

        group_name = self._effective_group_name(PickStage.SAFE_RETREAT, self._safe_retreat_group_name)
        cartesian_result = self._plan_cartesian_stage(
            stage=PickStage.SAFE_RETREAT,
            group_name=group_name,
            pose=self._retreat_pose,
            start_joint_state=start_joint_state,
            max_step=self._retreat_cartesian_max_step,
            min_fraction=self._retreat_cartesian_min_fraction,
            revolute_jump_threshold=self._retreat_cartesian_revolute_jump_threshold,
        )
        if cartesian_result.success:
            return cartesian_result

        if self._retreat_use_cartesian:
            return cartesian_result

        return self._plan_stage(
            stage=PickStage.SAFE_RETREAT,
            group_name=group_name,
            pose=self._retreat_pose,
            start_joint_state=start_joint_state,
            planner_id=self._retreat_planner_id,
        )

    def _trajectory_respects_joint_preferences(
        self,
        result: PlannedStage,
        seed_joint_state: JointState,
        label: str,
    ) -> bool:
        if len(result.trajectory.points) == 0:
            return True

        enforce_base_motion, base_translation_limit, base_theta_limit = self._stage_base_motion_limits(result.stage)
        if enforce_base_motion:
            base_planar_extent = trajectory_base_planar_motion_extent(result.trajectory, seed_joint_state)
            if base_planar_extent > base_translation_limit:
                self.get_logger().warning(
                    f"{label}: rejecting trajectory because base planar stage motion reached "
                    f"{base_planar_extent:.3f} m "
                    f"(limit {base_translation_limit:.3f} m)"
                )
                return False

            base_theta_extent = trajectory_joint_motion_extent(
                result.trajectory,
                seed_joint_state,
                "base_theta",
                revolute_joint_names={"base_theta"},
            )
            if base_theta_extent > base_theta_limit:
                self.get_logger().warning(
                    f"{label}: rejecting trajectory because base yaw stage motion reached "
                    f"{math.degrees(base_theta_extent):.1f} deg "
                    f"(limit {math.degrees(base_theta_limit):.1f} deg)"
                )
                return False

        effective_r1_limit_deg = self._effective_r1_stage_motion_limit_deg(result.stage)
        r1_motion_limit = math.radians(effective_r1_limit_deg)
        r1_motion_extent = trajectory_joint_motion_extent(
            result.trajectory,
            seed_joint_state,
            "r1_joint",
            revolute_joint_names={"r1_joint"},
        )
        if r1_motion_extent > r1_motion_limit:
            start_deg = math.degrees(joint_position(seed_joint_state, "r1_joint", 0.0))
            self.get_logger().warning(
                f"{label}: rejecting trajectory because r1 stage motion reached "
                f"{math.degrees(r1_motion_extent):.1f} deg from start {start_deg:.1f} deg "
                f"(limit {effective_r1_limit_deg:.1f} deg)"
            )
            return False
        return True

    def _prepare_stage_reached(
        self,
        result: PlannedStage,
        seed_joint_state: JointState,
        expected_pose: PoseStamped,
        label: str,
    ) -> bool:
        final_joint_state = result.final_joint_state
        if len(final_joint_state.name) == 0:
            final_joint_state = joint_state_from_trajectory_end(seed_joint_state, result.trajectory)
        else:
            final_joint_state = merge_joint_states(seed_joint_state, final_joint_state)
        achieved_pose = self._call_fk_service(final_joint_state, self._pose_link, label)
        if achieved_pose is None:
            return False
        projection_error, axial_error, orientation_error = prepare_alignment_errors(
            expected_pose.pose,
            achieved_pose.pose,
        )
        self.get_logger().info(
            f"{label}: projection_error={projection_error:.4f}, axial_error={axial_error:.4f}, "
            f"orientation_error={orientation_error:.4f}"
        )
        return (
            projection_error <= self._prepare_projection_tolerance
            and axial_error <= self._prepare_axial_tolerance
            and orientation_error <= self._prepare_orientation_tolerance
        )

    def _final_stage_reached(
        self,
        result: PlannedStage,
        seed_joint_state: JointState,
        expected_pose: PoseStamped,
        label: str,
    ) -> bool:
        final_joint_state = result.final_joint_state
        if len(final_joint_state.name) == 0:
            final_joint_state = joint_state_from_trajectory_end(seed_joint_state, result.trajectory)
        else:
            final_joint_state = merge_joint_states(seed_joint_state, final_joint_state)
        achieved_pose = self._call_fk_service(final_joint_state, self._pose_link, label)
        if achieved_pose is None:
            return False
        position_error, orientation_error = pose_error(expected_pose.pose, achieved_pose.pose)
        self.get_logger().info(
            f"{label}: position_error={position_error:.4f}, orientation_error={orientation_error:.4f}"
        )
        return (
            position_error <= self._final_target_position_tolerance
            and orientation_error <= self._final_target_orientation_tolerance
        )

    def _current_base_pose(self) -> PoseStamped:
        return pose_from_base_joint_state(self._current_joint_state, self._frame_id)

    def _current_prepare_pose_reached(self, expected_pose: PoseStamped, label: str) -> bool:
        achieved_pose = self._call_fk_service(self._current_joint_state, self._pose_link, label)
        if achieved_pose is None:
            return False
        projection_error, axial_error, orientation_error = prepare_alignment_errors(
            expected_pose.pose,
            achieved_pose.pose,
        )
        self.get_logger().info(
            f"{label}: projection_error={projection_error:.4f}, axial_error={axial_error:.4f}, "
            f"orientation_error={orientation_error:.4f}"
        )
        return (
            projection_error <= self._prepare_projection_tolerance
            and axial_error <= self._prepare_axial_tolerance
            and orientation_error <= self._prepare_orientation_tolerance
        )

    def _current_final_pose_reached(self, expected_pose: PoseStamped, label: str) -> bool:
        achieved_pose = self._call_fk_service(self._current_joint_state, self._pose_link, label)
        if achieved_pose is None:
            return False
        position_error, orientation_error = pose_error(expected_pose.pose, achieved_pose.pose)
        self.get_logger().info(
            f"{label}: position_error={position_error:.4f}, orientation_error={orientation_error:.4f}"
        )
        return (
            position_error <= self._final_target_position_tolerance
            and orientation_error <= self._final_target_orientation_tolerance
        )

    def _base_arrival_reached(self) -> bool:
        if self._base_goal_pose is None:
            return False
        current_base_pose = self._current_base_pose()
        position_error, orientation_error = pose_error(self._base_goal_pose.pose, current_base_pose.pose)
        return position_error <= 0.03 and orientation_error <= 0.15

    def _stow_arrival_reached(self) -> bool:
        current_positions = joint_state_positions(self._current_joint_state)
        if not current_positions:
            return False

        stow_joint_state = self._stow_joint_state()
        for joint_name, target_position in zip(stow_joint_state.name, stow_joint_state.position):
            current_position = current_positions.get(joint_name)
            if current_position is None:
                return False
            tolerance = self._stow_raise_tolerance if joint_name == "raise_joint" else self._stow_joint_tolerance
            if abs(current_position - target_position) > tolerance:
                return False
        return True

    def _lift_arrival_reached(self) -> bool:
        if self._lift_target_band_min is None or self._lift_target_band_max is None:
            return False
        current_positions = joint_state_positions(self._current_joint_state)
        current_height = current_positions.get("raise_joint")
        if current_height is None:
            return False
        effective_band_min = self._lift_target_band_min
        if getattr(self, "_discrete_lift_mode", False):
            effective_band_min -= getattr(self, "_lift_arrival_below_band_tolerance", 0.0)
        return effective_band_min <= current_height <= self._lift_target_band_max

    def _build_stow_trajectory(self) -> JointTrajectory:
        stow_joint_state = self._stow_joint_state()
        return self._build_arm_like_trajectory(stow_joint_state, self._stow_motion_duration)

    def _build_prepare_head_trajectory(self) -> JointTrajectory:
        if len(getattr(self, "_prepare_head_positions", ())) != 2:
            return JointTrajectory()
        current_positions = joint_state_positions(self._current_joint_state)
        joint_names = ["h1_joint", "h2_joint"]
        start_positions = [
            current_positions.get("h1_joint", 0.0),
            current_positions.get("h2_joint", -0.5236),
        ]
        end_positions = list(self._prepare_head_positions)
        max_step = max(0.1, float(getattr(self, "_prepare_head_max_step_rad", 0.35)))
        max_delta = max(abs(end - start) for start, end in zip(start_positions, end_positions))
        step_count = max(2, int(math.ceil(max_delta / max_step)) + 1)
        duration_sec = max(
            0.2,
            max(0.1, self._prepare_head_motion_duration),
            0.18 * float(step_count - 1),
        )
        return joint_states_to_interpolated_trajectory(
            joint_names=joint_names,
            start_positions=start_positions,
            end_positions=end_positions,
            duration_sec=duration_sec,
            step_count=step_count,
        )

    def _prepare_head_clearance_reached(self) -> bool:
        if len(getattr(self, "_prepare_head_positions", ())) != 2:
            return True
        current_positions = joint_state_positions(self._current_joint_state)
        if "h1_joint" not in current_positions or "h2_joint" not in current_positions:
            return False
        return (
            abs(current_positions["h1_joint"] - self._prepare_head_positions[0]) <= 0.05
            and abs(current_positions["h2_joint"] - self._prepare_head_positions[1]) <= 0.05
        )

    def _should_use_low_shelf_prepare_seed(self) -> bool:
        if not getattr(self, "_enable_low_shelf_prepare_seed", False):
            return False
        task_target_pose = self._task_target_pose()
        if task_target_pose is None:
            return False
        if len(getattr(self, "_low_shelf_prepare_arm_positions", ())) != len(
            getattr(self, "_low_shelf_prepare_arm_joint_names", ())
        ):
            return False
        return task_target_pose.pose.position.z < self._low_shelf_prepare_seed_target_z_threshold

    def _build_low_shelf_prepare_trajectory(self) -> JointTrajectory:
        if not self._should_use_low_shelf_prepare_seed():
            return JointTrajectory()
        seed_joint_state = JointState()
        seed_joint_state.name = list(self._low_shelf_prepare_arm_joint_names)
        seed_joint_state.position = list(self._low_shelf_prepare_arm_positions)
        return self._build_arm_like_trajectory(
            seed_joint_state,
            max(0.1, self._low_shelf_prepare_motion_duration),
        )

    def _low_shelf_prepare_seed_reached(self) -> bool:
        if not self._should_use_low_shelf_prepare_seed():
            return True
        current_positions = joint_state_positions(self._current_joint_state)
        for joint_name, target_position in zip(
            self._low_shelf_prepare_arm_joint_names,
            self._low_shelf_prepare_arm_positions,
        ):
            current_position = current_positions.get(joint_name)
            if current_position is None or abs(current_position - target_position) > self._stow_joint_tolerance:
                return False
        return True

    def _should_restore_low_shelf_target_lift(self) -> bool:
        if not self._should_use_low_shelf_prepare_seed():
            return False
        if self._target_lift_height is None or self._lift_target_center is None:
            return False
        return self._target_lift_height < (self._lift_target_center - 1e-3)

    def _build_low_shelf_target_lift_trajectory(self) -> JointTrajectory:
        if not self._should_restore_low_shelf_target_lift() or self._target_lift_height is None:
            return JointTrajectory()
        return self._build_lift_trajectory(
            self._target_lift_height,
            max(0.1, self._low_shelf_post_unfold_lift_duration),
        )

    def _low_shelf_target_lift_reached(self) -> bool:
        if not self._should_restore_low_shelf_target_lift() or self._target_lift_height is None:
            return True
        current_height = joint_position(self._current_joint_state, "raise_joint", self._target_lift_height)
        return abs(current_height - self._target_lift_height) <= max(0.02, self._lift_band_half_width)

    def _stow_joint_state(self) -> JointState:
        stow_joint_state = JointState()
        stow_positions = dict(self._stow_state_positions)
        if self._stow_preserve_raise_joint and "raise_joint" in stow_positions:
            current_raise = joint_position(
                self._current_joint_state,
                "raise_joint",
                stow_positions["raise_joint"],
            )
            stow_positions["raise_joint"] = current_raise
        stow_joint_state.name = list(stow_positions.keys())
        stow_joint_state.position = list(stow_positions.values())
        return stow_joint_state

    def _build_arm_like_trajectory(self, target_joint_state: JointState, duration_sec: float) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(target_joint_state.name)

        start_point = self._joint_trajectory_point_for_current(target_joint_state)
        start_point.time_from_start = self._duration_to_msg(0.0)
        end_point = self._joint_trajectory_point_for_target(target_joint_state)
        end_point.time_from_start = self._duration_to_msg(duration_sec)
        trajectory.points = [start_point, end_point]
        return trajectory

    def _joint_trajectory_point_for_current(self, target_joint_state: JointState):
        from trajectory_msgs.msg import JointTrajectoryPoint

        point = JointTrajectoryPoint()
        current_positions = joint_state_positions(self._current_joint_state)
        point.positions = [current_positions.get(name, position) for name, position in zip(target_joint_state.name, target_joint_state.position)]
        return point

    def _joint_trajectory_point_for_target(self, target_joint_state: JointState):
        from trajectory_msgs.msg import JointTrajectoryPoint

        point = JointTrajectoryPoint()
        point.positions = list(target_joint_state.position)
        return point
