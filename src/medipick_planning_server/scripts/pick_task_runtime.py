#!/usr/bin/python3

from __future__ import annotations

import math
from threading import Event
import time
from typing import Optional

from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from pick_task_shared import CandidateDebugSample, PickStage
from pick_task_utils import (
    clone_pose,
    filter_trajectory_joints,
    joint_states_to_interpolated_trajectory,
    joint_state_positions,
    joint_state_from_trajectory_end,
    normalize_revolute_trajectory_continuity,
    seconds_since,
    trajectory_base_planar_motion_extent,
    trajectory_joint_motion_extent,
    trajectory_motion_metrics,
    trajectory_duration,
    trajectory_effective_playback_duration,
)


class PickTaskRuntimeMixin:
    _CONTROLLER_NEGLIGIBLE_MOTION_EPS = 5e-3
    _HEAD_JOINT_NAMES = {"h1_joint", "h2_joint"}
    _WHOLE_BODY_JOINT_NAMES = {"base_x", "base_y", "base_theta", "raise_joint"}
    _WHOLE_BODY_OPERATION_DISCRETE_JOINT_NAMES = {
        "base_x",
        "base_y",
        "base_theta",
        "h1_joint",
        "h2_joint",
        "r1_joint",
        "r2_joint",
        "r3_joint",
        "r4_joint",
        "r5_joint",
        "r6_joint",
        "sucker_joint",
    }
    _ARM_ONLY_DISCRETE_JOINT_NAMES = {
        "h1_joint",
        "h2_joint",
        "r1_joint",
        "r2_joint",
        "r3_joint",
        "r4_joint",
        "r5_joint",
        "r6_joint",
        "sucker_joint",
    }
    _REVOLUTE_CONTROLLER_JOINT_NAMES = {
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
    }
    _WHOLE_BODY_SEGMENTED_LABELS = {
        "plan_to_pre_insert",
        "insert_and_suction",
        "direct_insert_and_suction",
        "safe_retreat",
    }

    def _controller_joint_names_in_order(self) -> Optional[tuple[str, ...]]:
        override_joint_names = tuple(getattr(self, "_controller_joint_names_override", ()))
        if override_joint_names:
            return override_joint_names

        controller_name = self._controller_action_name
        if "mobile_arm_controller" in controller_name:
            return (
                "r1_joint",
                "r2_joint",
                "r3_joint",
                "r4_joint",
                "r5_joint",
                "r6_joint",
            )
        if "base_controller" in controller_name:
            return ("base_x", "base_y", "base_theta")
        if "tool_controller" in controller_name:
            return ("sucker_joint",)
        if "head_controller" in controller_name:
            return ("h1_joint", "h2_joint")
        return None

    def _controller_allowed_joint_names(self) -> Optional[set[str]]:
        ordered_joint_names = self._controller_joint_names_in_order()
        if ordered_joint_names is None:
            return None
        return set(ordered_joint_names)

    def _expand_trajectory_for_controller(
        self,
        trajectory: JointTrajectory,
        ordered_joint_names: tuple[str, ...],
    ) -> JointTrajectory:
        if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
            return trajectory

        if tuple(trajectory.joint_names) == ordered_joint_names:
            return trajectory

        seed_positions = joint_state_positions(self._current_joint_state)
        source_indices = {joint_name: index for index, joint_name in enumerate(trajectory.joint_names)}

        expanded = JointTrajectory()
        expanded.header = trajectory.header
        expanded.joint_names = list(ordered_joint_names)

        for point in trajectory.points:
            new_point = type(point)()
            new_point.positions = []
            has_velocities = len(point.velocities) > 0
            has_accelerations = len(point.accelerations) > 0
            has_effort = len(point.effort) > 0
            if has_velocities:
                new_point.velocities = []
            if has_accelerations:
                new_point.accelerations = []
            if has_effort:
                new_point.effort = []

            for joint_name in ordered_joint_names:
                source_index = source_indices.get(joint_name, -1)
                if source_index >= 0:
                    new_point.positions.append(point.positions[source_index])
                    if has_velocities:
                        new_point.velocities.append(point.velocities[source_index])
                    if has_accelerations:
                        new_point.accelerations.append(point.accelerations[source_index])
                    if has_effort:
                        new_point.effort.append(point.effort[source_index])
                else:
                    new_point.positions.append(seed_positions.get(joint_name, 0.0))
                    if has_velocities:
                        new_point.velocities.append(0.0)
                    if has_accelerations:
                        new_point.accelerations.append(0.0)
                    if has_effort:
                        new_point.effort.append(0.0)
            new_point.time_from_start = point.time_from_start
            expanded.points.append(new_point)
        return expanded

    def _publish_current_goals(self) -> None:
        task_target_pose = self._task_target_pose()
        if self._base_goal_pose is not None:
            self._base_goal_pub.publish(self._base_goal_pose)
        if self._prepare_pose is not None:
            self._prepare_pose_pub.publish(self._prepare_pose)
            self._pre_insert_pose_pub.publish(self._prepare_pose)
        if task_target_pose is not None:
            self._final_pose_pub.publish(task_target_pose)
            self._pick_pose_pub.publish(task_target_pose)
        if self._retreat_pose is not None:
            self._retreat_pose_pub.publish(self._retreat_pose)

    def _publish_stage_path(self) -> None:
        task_target_pose = self._task_target_pose()
        if self._prepare_pose is None or task_target_pose is None or self._retreat_pose is None:
            return
        path = Path()
        path.header.frame_id = task_target_pose.header.frame_id or self._frame_id
        path.poses = [self._prepare_pose, task_target_pose, self._retreat_pose]
        self._stage_path_pub.publish(path)

    def _publish_debug_markers(self) -> None:
        markers = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        now = self.get_clock().now().to_msg()
        frame_id = self._frame_id
        task_target_pose = self._task_target_pose()
        if task_target_pose is not None and task_target_pose.header.frame_id:
            frame_id = task_target_pose.header.frame_id

        stage_text = Marker()
        stage_text.header.frame_id = frame_id
        stage_text.header.stamp = now
        stage_text.ns = "task_stage"
        stage_text.id = 0
        stage_text.type = Marker.TEXT_VIEW_FACING
        stage_text.action = Marker.ADD
        if task_target_pose is not None:
            stage_text.pose.position.x = task_target_pose.pose.position.x - 0.15
            stage_text.pose.position.y = task_target_pose.pose.position.y
            stage_text.pose.position.z = task_target_pose.pose.position.z + 0.18
        else:
            stage_text.pose.position.x = 0.4
            stage_text.pose.position.y = 0.0
            stage_text.pose.position.z = 1.4
        stage_text.pose.orientation.w = 1.0
        stage_text.scale.z = 0.06
        stage_text.color.r = 1.0
        stage_text.color.g = 1.0
        stage_text.color.b = 1.0
        stage_text.color.a = 0.95
        stage_text.text = f"Stage: {self._stage.value}"
        markers.markers.append(stage_text)

        marker_id = 10
        selected_sample: Optional[CandidateDebugSample] = None
        if self._prepare_candidate is not None:
            selected_sample = CandidateDebugSample(
                base_goal_pose=self._prepare_candidate.base_goal_pose,
                lift_height=self._prepare_candidate.lift_height,
                score=self._prepare_candidate.score,
                accepted=True,
                label=f"selected {self._prepare_candidate.score:.2f}",
            )

        for sample in self._candidate_debug_samples:
            color = (0.95, 0.25, 0.2, 0.85)
            scale = 0.05
            if sample.accepted:
                color = (0.95, 0.78, 0.10, 0.90)
            if (
                selected_sample is not None
                and abs(sample.base_goal_pose.pose.position.x - selected_sample.base_goal_pose.pose.position.x) < 1e-4
                and abs(sample.base_goal_pose.pose.position.y - selected_sample.base_goal_pose.pose.position.y) < 1e-4
                and abs(sample.lift_height - selected_sample.lift_height) < 1e-4
            ):
                color = (0.10, 0.95, 0.25, 0.95)
                scale = 0.08

            sphere = Marker()
            sphere.header.frame_id = sample.base_goal_pose.header.frame_id or frame_id
            sphere.header.stamp = now
            sphere.ns = "pre_insert_candidates"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = sample.base_goal_pose.pose.position.x
            sphere.pose.position.y = sample.base_goal_pose.pose.position.y
            sphere.pose.position.z = 0.04
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = scale
            sphere.scale.y = scale
            sphere.scale.z = scale
            sphere.color.r = color[0]
            sphere.color.g = color[1]
            sphere.color.b = color[2]
            sphere.color.a = color[3]
            markers.markers.append(sphere)

            arrow = Marker()
            arrow.header.frame_id = sample.base_goal_pose.header.frame_id or frame_id
            arrow.header.stamp = now
            arrow.ns = "pre_insert_candidate_heading"
            arrow.id = marker_id
            marker_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = clone_pose(sample.base_goal_pose.pose)
            arrow.pose.position.z = 0.08
            arrow.scale.x = 0.16
            arrow.scale.y = 0.02
            arrow.scale.z = 0.02
            arrow.color.r = color[0]
            arrow.color.g = color[1]
            arrow.color.b = color[2]
            arrow.color.a = color[3]
            markers.markers.append(arrow)

            text = Marker()
            text.header.frame_id = sample.base_goal_pose.header.frame_id or frame_id
            text.header.stamp = now
            text.ns = "pre_insert_candidate_text"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = sample.base_goal_pose.pose.position.x
            text.pose.position.y = sample.base_goal_pose.pose.position.y
            text.pose.position.z = 0.16
            text.pose.orientation.w = 1.0
            text.scale.z = 0.035
            text.color.r = color[0]
            text.color.g = color[1]
            text.color.b = color[2]
            text.color.a = 0.95
            text.text = f"{sample.label}\nlift~{sample.lift_height:.2f}"
            markers.markers.append(text)

        self._debug_markers_pub.publish(markers)

    def _publish_trajectory(self, trajectory: JointTrajectory) -> None:
        self._trajectory_pub.publish(trajectory)
        if self._auto_publish_visualization_trajectory:
            self._visual_trajectory_pub.publish(trajectory)

    def _build_controller_trajectory(
        self,
        trajectory: JointTrajectory,
        ordered_joint_names: tuple[str, ...],
    ) -> JointTrajectory:
        filtered = filter_trajectory_joints(
            trajectory,
            allowed_joint_names=set(ordered_joint_names),
        )
        if len(filtered.joint_names) == 0 or len(filtered.points) == 0:
            return JointTrajectory()
        expanded = self._expand_trajectory_for_controller(filtered, ordered_joint_names)
        return normalize_revolute_trajectory_continuity(
            expanded,
            seed_joint_state=self._current_joint_state,
            revolute_joint_names=self._REVOLUTE_CONTROLLER_JOINT_NAMES,
        )

    def _controller_motion_summary(
        self,
        trajectory: JointTrajectory,
        joint_names_of_interest: tuple[str, ...],
    ) -> tuple[float, float, list[str]]:
        total_motion, max_step = trajectory_motion_metrics(trajectory, set(joint_names_of_interest))
        moving_joint_summaries: list[str] = []
        for joint_name in joint_names_of_interest:
            extent = trajectory_joint_motion_extent(
                trajectory,
                self._current_joint_state,
                joint_name,
                revolute_joint_names=self._REVOLUTE_CONTROLLER_JOINT_NAMES,
            )
            if extent > 1e-4:
                moving_joint_summaries.append(f"{joint_name}={extent:.3f}")
        return total_motion, max_step, moving_joint_summaries

    def _send_controller_trajectory(
        self,
        action_client: ActionClient,
        action_name: str,
        trajectory: JointTrajectory,
        label: str,
    ) -> bool:
        goal_handle = None
        for attempt_index in range(self._controller_goal_retry_count):
            attempt_number = attempt_index + 1

            if not action_client.wait_for_server(timeout_sec=3.0):
                if attempt_number < self._controller_goal_retry_count:
                    self.get_logger().warn(
                        f"{label}: controller action '{action_name}' unavailable，"
                        f"第 {attempt_number} 次发送失败，等待 {self._controller_retry_delay:.1f}s 后重试。"
                    )
                    time.sleep(self._controller_retry_delay)
                    continue
                self.get_logger().error(f"{label}: controller action '{action_name}' unavailable.")
                return False

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory

            send_goal_future = action_client.send_goal_async(goal)
            send_done = Event()
            send_goal_future.add_done_callback(lambda _: send_done.set())
            if not send_done.wait(timeout=5.0):
                if attempt_number < self._controller_goal_retry_count:
                    self.get_logger().warn(
                        f"{label}: 第 {attempt_number} 次向 controller '{action_name}' 发送轨迹超时，"
                        f"等待 {self._controller_retry_delay:.1f}s 后重试。"
                    )
                    time.sleep(self._controller_retry_delay)
                    continue
                self.get_logger().error(
                    f"{label}: timed out while sending goal to controller '{action_name}'."
                )
                return False

            goal_handle = send_goal_future.result()
            if goal_handle is not None and goal_handle.accepted:
                break

            if attempt_number < self._controller_goal_retry_count:
                self.get_logger().warn(
                    f"{label}: controller '{action_name}' 暂时拒绝轨迹，第 {attempt_number} 次失败，"
                    f"等待 {self._controller_retry_delay:.1f}s 后重试。"
                )
                time.sleep(self._controller_retry_delay)
                continue

            self.get_logger().error(f"{label}: controller '{action_name}' rejected trajectory.")
            return False

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label}: controller '{action_name}' rejected trajectory.")
            return False

        result_future = goal_handle.get_result_async()
        timeout = max(
            6.0,
            1.5 * trajectory_duration(trajectory) + self._trajectory_execution_timeout_margin,
        )
        result_done = Event()
        result_future.add_done_callback(lambda _: result_done.set())
        if not result_done.wait(timeout=timeout):
            grace_timeout = min(20.0, max(5.0, 0.5 * timeout))
            grace_deadline = time.monotonic() + grace_timeout
            while time.monotonic() < grace_deadline:
                errors = self._joint_goal_errors(trajectory)
                if errors and all(
                    error <= self._joint_settle_tolerance(joint_name)
                    for joint_name, error in errors
                ):
                    self.get_logger().warning(
                        f"{label}: controller '{action_name}' result timed out, but joints settled within tolerance."
                    )
                    return True
                time.sleep(0.1)
            self.get_logger().error(
                f"{label}: timed out while waiting for controller '{action_name}' result."
            )
            return False

        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error(f"{label}: controller '{action_name}' result unavailable.")
            return False

        result = result_wrapper.result
        error_code = int(getattr(result, "error_code", 0))
        if error_code != 0:
            error_string = getattr(result, "error_string", "")
            self.get_logger().error(
                f"{label}: controller '{action_name}' execution failed with "
                f"error_code={error_code}, error='{error_string}'."
            )
            return False
        return True

    def _joint_goal_errors(
        self,
        trajectory: JointTrajectory,
    ) -> list[tuple[str, float]]:
        if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
            return []

        latest_positions = joint_state_positions(self._current_joint_state)
        final_positions = trajectory.points[-1].positions
        errors: list[tuple[str, float]] = []
        for index, joint_name in enumerate(trajectory.joint_names):
            if index >= len(final_positions):
                continue
            actual = latest_positions.get(joint_name)
            if actual is None:
                continue
            error = final_positions[index] - actual
            if joint_name in self._REVOLUTE_CONTROLLER_JOINT_NAMES:
                error = math.atan2(math.sin(error), math.cos(error))
            errors.append((joint_name, abs(error)))
        errors.sort(key=lambda item: item[1], reverse=True)
        return errors

    def _joint_settle_tolerance(self, joint_name: str) -> float:
        if joint_name in {"base_x", "base_y", "raise_joint"}:
            return 0.006
        if joint_name in {"base_theta", "r1_joint", "r2_joint", "r3_joint", "r4_joint", "r5_joint", "r6_joint"}:
            return 0.02
        return 0.03

    def _wait_for_trajectory_settle(
        self,
        trajectory: JointTrajectory,
        label: str,
    ) -> list[tuple[str, float]]:
        if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
            return []

        deadline = time.monotonic() + min(
            2.5,
            max(0.4, 0.35 * trajectory_effective_playback_duration(trajectory)),
        )
        best_errors = self._joint_goal_errors(trajectory)
        while time.monotonic() < deadline:
            errors = self._joint_goal_errors(trajectory)
            if errors:
                best_errors = errors
                if all(error <= self._joint_settle_tolerance(joint_name) for joint_name, error in errors):
                    return []
            time.sleep(0.05)

        if best_errors:
            top_errors = ", ".join(
                f"{joint_name}={error:.4f}" for joint_name, error in best_errors[:4]
            )
            self.get_logger().warn(
                f"{label}: controller reported success but joint settling remained above tolerance: {top_errors}"
            )
        return [
            (joint_name, error)
            for joint_name, error in best_errors
            if error > self._joint_settle_tolerance(joint_name)
        ]

    def _build_settle_correction_trajectory(
        self,
        reference_trajectory: JointTrajectory,
        remaining_errors: list[tuple[str, float]],
        label: str,
        attempt_number: int,
    ) -> Optional[JointTrajectory]:
        if len(reference_trajectory.joint_names) == 0 or len(reference_trajectory.points) == 0:
            return None

        current_positions = joint_state_positions(self._current_joint_state)
        target_positions = {
            joint_name: position
            for joint_name, position in zip(
                reference_trajectory.joint_names,
                reference_trajectory.points[-1].positions,
            )
        }

        correction_joint_names: list[str] = []
        start_positions: list[float] = []
        end_positions: list[float] = []
        max_error = 0.0
        for joint_name, error in remaining_errors:
            if joint_name not in target_positions:
                continue
            current_position = current_positions.get(joint_name)
            if current_position is None:
                continue
            correction_joint_names.append(joint_name)
            start_positions.append(current_position)
            end_positions.append(target_positions[joint_name])
            max_error = max(max_error, error)

        if not correction_joint_names:
            return None

        duration_sec = min(
            max(0.8, float(getattr(self, "_controller_settle_correction_max_duration", 2.5))),
            max(
                float(getattr(self, "_controller_settle_correction_min_duration", 0.9)),
                0.8 + 7.0 * max_error,
            ),
        )
        step_count = max(3, min(8, int(math.ceil(max_error / 0.03)) + 2))

        correction = joint_states_to_interpolated_trajectory(
            joint_names=correction_joint_names,
            start_positions=start_positions,
            end_positions=end_positions,
            duration_sec=duration_sec,
            step_count=step_count,
        )
        correction = normalize_revolute_trajectory_continuity(
            correction,
            seed_joint_state=self._current_joint_state,
            revolute_joint_names=self._REVOLUTE_CONTROLLER_JOINT_NAMES,
        )
        self.get_logger().info(
            f"{label}: correction pass {attempt_number} for joints={correction_joint_names}, "
            f"duration={duration_sec:.2f}s, max_error={max_error:.4f}"
        )
        return correction

    def _controller_requests_for_trajectory(
        self,
        trajectory: JointTrajectory,
        label: str,
    ) -> Optional[list[tuple[str, ActionClient, JointTrajectory]]]:
        requests: list[tuple[str, ActionClient, JointTrajectory]] = []

        primary_action_name = self._controller_action_name
        primary_joint_names = self._controller_joint_names_in_order()
        if primary_joint_names is not None:
            primary_trajectory = self._build_controller_trajectory(trajectory, primary_joint_names)
            primary_total_motion, primary_max_step, primary_moving_joints = self._controller_motion_summary(
                primary_trajectory,
                primary_joint_names,
            )
            if (
                primary_total_motion > self._CONTROLLER_NEGLIGIBLE_MOTION_EPS
                or primary_max_step > self._CONTROLLER_NEGLIGIBLE_MOTION_EPS
            ) and primary_moving_joints:
                self.get_logger().info(
                    f"{label}: primary controller '{primary_action_name}' motion="
                    f"[{', '.join(primary_moving_joints)}], total={primary_total_motion:.3f}, "
                    f"max_step={primary_max_step:.3f}"
                )
                requests.append((primary_action_name, self._trajectory_action, primary_trajectory))

        auxiliary_specs: list[tuple[str, Optional[ActionClient], tuple[str, ...]]] = []
        base_action_name = str(getattr(self, "_base_controller_action_name", "")).strip()
        base_joint_names = tuple(getattr(self, "_base_controller_joint_names_override", ()))
        if base_action_name and base_action_name != primary_action_name and base_joint_names:
            auxiliary_specs.append(
                (
                    base_action_name,
                    getattr(self, "_base_trajectory_action", None),
                    base_joint_names,
                )
            )
        tool_action_name = str(getattr(self, "_tool_controller_action_name", "")).strip()
        if tool_action_name and tool_action_name != primary_action_name:
            auxiliary_specs.append(
                (
                    tool_action_name,
                    getattr(self, "_tool_trajectory_action", None),
                    ("sucker_joint",),
                )
            )
        head_action_name = str(getattr(self, "_head_controller_action_name", "")).strip()
        if head_action_name and head_action_name != primary_action_name:
            auxiliary_specs.append(
                (
                    head_action_name,
                    getattr(self, "_head_trajectory_action", None),
                    ("h1_joint", "h2_joint"),
                )
            )

        for action_name, action_client, joint_names in auxiliary_specs:
            total_motion, max_step, moving_joints = self._controller_motion_summary(trajectory, joint_names)
            if (
                total_motion <= self._CONTROLLER_NEGLIGIBLE_MOTION_EPS
                and max_step <= self._CONTROLLER_NEGLIGIBLE_MOTION_EPS
            ) or not moving_joints:
                continue
            if action_client is None:
                self.get_logger().error(
                    f"{label}: trajectory requires controller '{action_name}' for joints {list(joint_names)}, "
                    "but no action client is configured."
                )
                return None
            controller_trajectory = self._build_controller_trajectory(trajectory, joint_names)
            self.get_logger().info(
                f"{label}: auxiliary controller '{action_name}' motion="
                f"[{', '.join(moving_joints)}], total={total_motion:.3f}, max_step={max_step:.3f}"
            )
            requests.append((action_name, action_client, controller_trajectory))

        if not requests:
            trajectory_total_motion, trajectory_max_step = trajectory_motion_metrics(
                trajectory,
                set(trajectory.joint_names),
            )
            if trajectory_total_motion <= 1e-4 and trajectory_max_step <= 1e-4:
                self.get_logger().info(
                    f"{label}: trajectory already satisfies target state, skipping controller execution."
                )
                return []
            self.get_logger().error(
                f"{label}: no controller trajectory remained after splitting trajectory joints "
                f"{trajectory.joint_names}."
            )
            return None
        return requests

    def _settle_corrections_enabled_for_label(self, label: str) -> bool:
        if ":segment#" in label:
            return False
        return label.split(":", 1)[0] in {
            "plan_to_pre_insert",
            "insert_and_suction",
            "direct_insert_and_suction",
            "safe_retreat",
            "return_to_stow",
        }

    def _maybe_execute_segmented_whole_body_trajectory(
        self,
        trajectory: JointTrajectory,
        label: str,
    ) -> Optional[bool]:
        base_label = label.split(":", 1)[0]
        if base_label not in self._WHOLE_BODY_SEGMENTED_LABELS or ":segment#" in label:
            return None
        if not any(joint_name in self._WHOLE_BODY_JOINT_NAMES for joint_name in trajectory.joint_names):
            return None

        seed_joint_state = self._current_joint_state
        base_planar_motion = trajectory_base_planar_motion_extent(trajectory, seed_joint_state)
        base_yaw_motion = trajectory_joint_motion_extent(
            trajectory,
            seed_joint_state,
            "base_theta",
            revolute_joint_names=self._REVOLUTE_CONTROLLER_JOINT_NAMES,
        )
        arm_joint_names = (
            "h1_joint",
            "h2_joint",
            "r1_joint",
            "r2_joint",
            "r3_joint",
            "r4_joint",
            "r5_joint",
            "r6_joint",
        )
        arm_max_motion = max(
            (
                trajectory_joint_motion_extent(
                    trajectory,
                    seed_joint_state,
                    joint_name,
                    revolute_joint_names=self._REVOLUTE_CONTROLLER_JOINT_NAMES,
                )
                for joint_name in arm_joint_names
                if joint_name in trajectory.joint_names
            ),
            default=0.0,
        )

        if base_label == "plan_to_pre_insert":
            max_base_translation = max(
                0.03,
                float(getattr(self, "_pre_insert_segment_max_base_translation_m", 0.05)),
            )
            max_base_yaw = math.radians(
                max(3.0, float(getattr(self, "_pre_insert_segment_max_base_rotation_deg", 5.0)))
            )
            max_arm_motion = max(
                0.15,
                float(getattr(self, "_pre_insert_segment_max_arm_joint_motion", 0.40)),
            )
            max_segment_count = int(getattr(self, "_pre_insert_segment_max_count", 14))
            min_segment_duration = float(getattr(self, "_pre_insert_segment_min_duration", 1.4))
            max_segment_duration = float(getattr(self, "_pre_insert_segment_max_duration", 3.0))
            step_count = max(4, min(10, int(getattr(self, "_pre_insert_segment_step_count", 6))))
        elif base_label in {"insert_and_suction", "direct_insert_and_suction", "safe_retreat"}:
            max_base_translation = max(
                0.03,
                float(getattr(self, "_insert_segment_max_base_translation_m", 0.05)),
            )
            max_base_yaw = math.radians(
                max(3.0, float(getattr(self, "_insert_segment_max_base_rotation_deg", 5.0)))
            )
            max_arm_motion = max(
                0.15,
                float(getattr(self, "_insert_segment_max_arm_joint_motion", 0.40)),
            )
            max_segment_count = int(getattr(self, "_insert_segment_max_count", 8))
            min_segment_duration = float(getattr(self, "_insert_segment_min_duration", 1.4))
            max_segment_duration = float(getattr(self, "_insert_segment_max_duration", 2.8))
            step_count = max(4, min(8, int(getattr(self, "_insert_segment_step_count", 5))))
        else:
            max_base_translation = max(
                0.05,
                float(getattr(self, "_whole_body_segment_max_base_translation_m", 0.10)),
            )
            max_base_yaw = math.radians(
                max(4.0, float(getattr(self, "_whole_body_segment_max_base_rotation_deg", 8.0)))
            )
            max_arm_motion = max(
                0.20,
                float(getattr(self, "_whole_body_segment_max_arm_joint_motion", 0.85)),
            )
            max_segment_count = int(getattr(self, "_whole_body_segment_max_count", 6))
            min_segment_duration = float(getattr(self, "_whole_body_segment_min_duration", 1.8))
            max_segment_duration = float(getattr(self, "_whole_body_segment_max_duration", 4.0))
            step_count = max(4, min(8, int(getattr(self, "_whole_body_segment_step_count", 5))))

        segment_count = max(
            1,
            int(math.ceil(base_planar_motion / max_base_translation)),
            int(math.ceil(base_yaw_motion / max_base_yaw)),
            int(math.ceil(arm_max_motion / max_arm_motion)),
        )
        segment_count = min(segment_count, max_segment_count)
        if segment_count <= 1:
            return None

        total_duration = max(
            trajectory_effective_playback_duration(trajectory),
            float(segment_count) * min_segment_duration,
        )
        segment_duration = min(
            max(
                min_segment_duration,
                total_duration / float(segment_count),
            ),
            max_segment_duration,
        )

        overall_start_positions = joint_state_positions(seed_joint_state)
        final_joint_state = joint_state_from_trajectory_end(seed_joint_state, trajectory)
        final_positions = joint_state_positions(final_joint_state)

        self.get_logger().info(
            f"{label}: splitting whole-body trajectory into {segment_count} segments "
            f"(base_planar={base_planar_motion:.3f} m, base_yaw={math.degrees(base_yaw_motion):.1f} deg, "
            f"arm_max={arm_max_motion:.3f} rad)."
        )

        for segment_index in range(1, segment_count + 1):
            ratio = float(segment_index) / float(segment_count)
            current_positions = joint_state_positions(self._current_joint_state)
            start_positions: list[float] = []
            end_positions: list[float] = []
            base_lead_fraction = 0.0
            if base_label == "plan_to_pre_insert":
                base_lead_fraction = max(
                    0.0,
                    min(0.6, float(getattr(self, "_pre_insert_segment_base_lead_fraction", 0.35))),
                )
            for joint_name in trajectory.joint_names:
                start_value = current_positions.get(joint_name, overall_start_positions.get(joint_name, 0.0))
                final_value = final_positions.get(joint_name, start_value)
                nominal_start = overall_start_positions.get(joint_name, start_value)
                interpolation_ratio = ratio
                if (
                    base_lead_fraction > 0.0
                    and joint_name
                    in {
                        "r1_joint",
                        "r2_joint",
                        "r3_joint",
                        "r4_joint",
                        "r5_joint",
                        "r6_joint",
                        "sucker_joint",
                    }
                ):
                    interpolation_ratio = max(
                        0.0,
                        min(1.0, (ratio - base_lead_fraction) / max(1e-6, 1.0 - base_lead_fraction)),
                    )
                end_value = nominal_start + (final_value - nominal_start) * interpolation_ratio
                start_positions.append(start_value)
                end_positions.append(end_value)

            segment_trajectory = joint_states_to_interpolated_trajectory(
                joint_names=list(trajectory.joint_names),
                start_positions=start_positions,
                end_positions=end_positions,
                duration_sec=segment_duration,
                step_count=step_count,
            )
            if not self._execute_trajectory_internal(
                segment_trajectory,
                f"{label}:segment#{segment_index}/{segment_count}",
                allow_segmentation=False,
            ):
                return False

        return True

    def _execute_trajectory_internal(
        self,
        trajectory: JointTrajectory,
        label: str,
        *,
        allow_segmentation: bool,
    ) -> bool:
        if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
            self.get_logger().error(f"{label}: empty trajectory.")
            return False

        trajectory, lift_only_completed = self._prepare_discrete_lift_trajectory(trajectory, label)
        if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
            if lift_only_completed:
                self._publish_estimated_achieved_pose(trajectory, label, self._current_joint_state)
                return True
            self.get_logger().error(f"{label}: empty trajectory after discrete lift preparation.")
            return False

        trajectory = self._sanitize_trajectory_for_stage(trajectory, label)
        if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
            self.get_logger().error(f"{label}: empty trajectory after sanitizing.")
            return False

        if allow_segmentation:
            segmented_result = self._maybe_execute_segmented_whole_body_trajectory(trajectory, label)
            if segmented_result is not None:
                if segmented_result:
                    self._publish_estimated_achieved_pose(trajectory, label, self._current_joint_state)
                return segmented_result

        self.get_logger().info(f"{label}: executing joints={trajectory.joint_names}")

        self._publish_trajectory(trajectory)

        if not self._execute_with_controller:
            time.sleep(max(0.1, trajectory_effective_playback_duration(trajectory) + 0.2))
            self._current_joint_state = joint_state_from_trajectory_end(self._current_joint_state, trajectory)
            self._visualization_joint_state_pub.publish(self._current_joint_state)
            self._publish_estimated_achieved_pose(trajectory, label, self._current_joint_state)
            return True

        controller_requests = self._controller_requests_for_trajectory(trajectory, label)
        if controller_requests is None:
            return False
        for action_name, action_client, controller_trajectory in controller_requests:
            controller_label = label if action_name == self._controller_action_name else f"{label}[{action_name}]"
            if not self._send_controller_trajectory(
                action_client,
                action_name,
                controller_trajectory,
                controller_label,
            ):
                return False
            remaining_errors = self._wait_for_trajectory_settle(controller_trajectory, controller_label)
            if self._settle_corrections_enabled_for_label(label):
                correction_attempt = 0
                max_correction_passes = max(0, int(getattr(self, "_controller_settle_correction_passes", 0)))
                while remaining_errors and correction_attempt < max_correction_passes:
                    correction_attempt += 1
                    correction_trajectory = self._build_settle_correction_trajectory(
                        controller_trajectory,
                        remaining_errors,
                        controller_label,
                        correction_attempt,
                    )
                    if correction_trajectory is None:
                        break
                    if not self._send_controller_trajectory(
                        action_client,
                        action_name,
                        correction_trajectory,
                        f"{controller_label}:correction#{correction_attempt}",
                    ):
                        return False
                    remaining_errors = self._wait_for_trajectory_settle(
                        correction_trajectory,
                        f"{controller_label}:correction#{correction_attempt}",
                    )
        time.sleep(0.1)
        self._publish_estimated_achieved_pose(trajectory, label, self._current_joint_state)
        return True

    def _execute_trajectory(self, trajectory: JointTrajectory, label: str) -> bool:
        return self._execute_trajectory_internal(trajectory, label, allow_segmentation=True)

    def _publish_estimated_achieved_pose(
        self,
        trajectory: JointTrajectory,
        label: str,
        estimated_joint_state: Optional[JointState] = None,
    ) -> None:
        if estimated_joint_state is None:
            estimated_joint_state = joint_state_from_trajectory_end(self._current_joint_state, trajectory)
        achieved_pose = self._call_fk_service(estimated_joint_state, self._pose_link, f"{label}_achieved_pose")
        if achieved_pose is None:
            return
        self._achieved_pose_pub.publish(achieved_pose)
        task_target_pose = self._task_target_pose()
        if label == "insert_and_suction" and task_target_pose is not None:
            position_error = math.sqrt(
                (achieved_pose.pose.position.x - task_target_pose.pose.position.x) ** 2
                + (achieved_pose.pose.position.y - task_target_pose.pose.position.y) ** 2
                + (achieved_pose.pose.position.z - task_target_pose.pose.position.z) ** 2
            )
            self.get_logger().info(
                f"{label}: estimated {self._pose_link} position error to target = {position_error:.4f} m"
            )

    def _effective_group_name(self, stage: PickStage, group_name: str) -> str:
        if self._discrete_lift_mode and stage in (
            PickStage.PLAN_TO_PRE_INSERT,
            PickStage.INSERT_AND_SUCTION,
            PickStage.SAFE_RETREAT,
        ):
            return self._discrete_arm_group_name
        return group_name

    def _sanitize_trajectory_for_stage(self, trajectory: JointTrajectory, label: str) -> JointTrajectory:
        result = trajectory
        if self._discrete_lift_mode:
            arm_only_labels = {
                "stow_arm",
                "plan_to_pre_insert",
                "insert_and_suction",
                "direct_insert_and_suction",
                "safe_retreat",
                "return_to_stow",
            }
            contains_whole_body_joints = any(
                joint_name in self._WHOLE_BODY_JOINT_NAMES
                for joint_name in result.joint_names
            )
            if label in arm_only_labels:
                allowed_joint_names = (
                    self._WHOLE_BODY_OPERATION_DISCRETE_JOINT_NAMES
                    if contains_whole_body_joints
                    else self._ARM_ONLY_DISCRETE_JOINT_NAMES
                )
                result = filter_trajectory_joints(
                    result,
                    allowed_joint_names=allowed_joint_names,
                )

            if label in {
                "plan_to_pre_insert",
                "insert_and_suction",
                "direct_insert_and_suction",
                "safe_retreat",
            }:
                non_head_joint_names = set(result.joint_names) - self._HEAD_JOINT_NAMES
                result = filter_trajectory_joints(
                    result,
                    allowed_joint_names=non_head_joint_names,
                )

        return normalize_revolute_trajectory_continuity(
            result,
            seed_joint_state=self._current_joint_state,
            revolute_joint_names=self._REVOLUTE_CONTROLLER_JOINT_NAMES,
        )

    def _prepare_discrete_lift_trajectory(
        self,
        trajectory: JointTrajectory,
        label: str,
    ) -> tuple[JointTrajectory, bool]:
        if (
            not self._discrete_lift_mode
            or len(trajectory.joint_names) == 0
            or len(trajectory.points) == 0
            or "raise_joint" not in trajectory.joint_names
        ):
            return trajectory, False

        raise_joint_index = trajectory.joint_names.index("raise_joint")
        target_height = float(trajectory.points[-1].positions[raise_joint_index])
        current_height = joint_state_positions(self._current_joint_state).get("raise_joint", target_height)
        if not self._drive_external_lift_to_height(target_height, label, current_height):
            return JointTrajectory(), False

        filtered = filter_trajectory_joints(
            trajectory,
            allowed_joint_names=set(trajectory.joint_names) - {"raise_joint"},
        )
        if len(filtered.joint_names) == 0 or len(filtered.points) == 0:
            self.get_logger().info(
                f"{label}: discrete lift mode executed as lift-only motion to {target_height:.4f} m."
            )
            return JointTrajectory(), True

        self.get_logger().info(
            f"{label}: discrete lift mode active, removing raise_joint from controller trajectory "
            f"after driving external lift to {target_height:.4f} m."
        )
        return filtered, False

    def _drive_external_lift_to_height(
        self,
        target_height: float,
        label: str,
        current_height: Optional[float] = None,
    ) -> bool:
        lift_target_pub = getattr(self, "_lift_target_pub", None)
        if lift_target_pub is None:
            self.get_logger().error(f"{label}: no lift target publisher is available in discrete lift mode.")
            return False

        if current_height is None:
            current_height = joint_state_positions(self._current_joint_state).get("raise_joint", target_height)

        lift_tolerance = 0.008
        if abs(target_height - current_height) <= lift_tolerance:
            return True

        self.get_logger().info(
            f"{label}: commanding external lift from {current_height:.4f} m to {target_height:.4f} m."
        )
        lift_target_pub.publish(Float64(data=target_height))

        travel_distance = abs(target_height - current_height)
        timeout_sec = min(
            max(10.0, 4.0 + travel_distance / 0.006),
            max(15.0, self._stage_timeout * 0.5),
        )
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            latest_height = joint_state_positions(self._current_joint_state).get("raise_joint")
            if latest_height is not None and abs(latest_height - target_height) <= lift_tolerance:
                self.get_logger().info(
                    f"{label}: external lift reached {latest_height:.4f} m "
                    f"(target {target_height:.4f} m)."
                )
                return True
            lift_target_pub.publish(Float64(data=target_height))
            time.sleep(0.1)

        latest_height = joint_state_positions(self._current_joint_state).get("raise_joint")
        self.get_logger().error(
            f"{label}: external lift failed to reach target height {target_height:.4f} m "
            f"within {timeout_sec:.1f}s "
            f"(latest={latest_height if latest_height is not None else float('nan'):.4f})."
        )
        return False

    def _has_fresh_target(self) -> bool:
        if self._latest_target_pose is None:
            return False
        age = seconds_since(self.get_clock().now(), self._latest_target_received_at)
        return age <= self._target_pose_timeout

    def _stage_timed_out(self) -> bool:
        return seconds_since(self.get_clock().now(), self._stage_started_at) > self._stage_timeout

    def _transition_to(self, stage: PickStage) -> None:
        if stage == PickStage.ARM_STOW_SAFE:
            self._stow_arrived = False
            self._stow_motion_executed = False
        if stage in (PickStage.ARM_STOW_SAFE, PickStage.LIFT_TO_BAND, PickStage.SELECT_PRE_INSERT):
            self._lift_arrived = False
            self._lift_motion_executed = False
            self._lift_insert_probe_attempted = False
            self._lift_prepare_probe_attempted = False
        self._stage = stage
        self._stage_started_at = self.get_clock().now()
        self.get_logger().info(f"Stage -> {stage.value}")
        self._event(f"Stage -> {stage.value}")

    def _fail(self, message: str) -> None:
        self.get_logger().error(message)
        self._event(message)
        self._transition_to(PickStage.FAILED)

    def _event(self, message: str) -> None:
        self._event_pub.publish(String(data=message))
