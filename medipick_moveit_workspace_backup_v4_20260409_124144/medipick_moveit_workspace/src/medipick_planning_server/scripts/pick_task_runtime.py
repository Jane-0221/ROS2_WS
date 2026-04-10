#!/usr/bin/env python3

from __future__ import annotations

import math
from threading import Event
import time
from typing import Optional

from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from pick_task_shared import CandidateDebugSample, PickStage
from pick_task_utils import (
    clone_pose,
    filter_trajectory_joints,
    joint_state_from_trajectory_end,
    normalize_revolute_trajectory_continuity,
    seconds_since,
    trajectory_duration,
    trajectory_effective_playback_duration,
)


class PickTaskRuntimeMixin:
    def _controller_allowed_joint_names(self) -> Optional[set[str]]:
        controller_name = self._controller_action_name
        if "mobile_arm_controller" in controller_name:
            return {
                "base_x",
                "base_y",
                "base_theta",
                "raise_joint",
                "r1_joint",
                "r2_joint",
                "r3_joint",
                "r4_joint",
                "r5_joint",
                "r6_joint",
            }
        if "tool_controller" in controller_name:
            return {"sucker_joint"}
        if "head_controller" in controller_name:
            return {"h1_joint", "h2_joint"}
        return None

    def _publish_current_goals(self) -> None:
        if self._base_goal_pose is not None:
            self._base_goal_pub.publish(self._base_goal_pose)
        if self._prepare_pose is not None:
            self._prepare_pose_pub.publish(self._prepare_pose)
            self._pre_insert_pose_pub.publish(self._prepare_pose)
        if self._latest_target_pose is not None:
            self._final_pose_pub.publish(self._latest_target_pose)
            self._pick_pose_pub.publish(self._latest_target_pose)
        if self._retreat_pose is not None:
            self._retreat_pose_pub.publish(self._retreat_pose)

    def _publish_stage_path(self) -> None:
        if self._prepare_pose is None or self._latest_target_pose is None or self._retreat_pose is None:
            return
        path = Path()
        path.header.frame_id = self._latest_target_pose.header.frame_id or self._frame_id
        path.poses = [self._prepare_pose, self._latest_target_pose, self._retreat_pose]
        self._stage_path_pub.publish(path)

    def _publish_debug_markers(self) -> None:
        markers = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        now = self.get_clock().now().to_msg()
        frame_id = self._frame_id
        if self._latest_target_pose is not None and self._latest_target_pose.header.frame_id:
            frame_id = self._latest_target_pose.header.frame_id

        stage_text = Marker()
        stage_text.header.frame_id = frame_id
        stage_text.header.stamp = now
        stage_text.ns = "task_stage"
        stage_text.id = 0
        stage_text.type = Marker.TEXT_VIEW_FACING
        stage_text.action = Marker.ADD
        if self._latest_target_pose is not None:
            stage_text.pose.position.x = self._latest_target_pose.pose.position.x - 0.15
            stage_text.pose.position.y = self._latest_target_pose.pose.position.y
            stage_text.pose.position.z = self._latest_target_pose.pose.position.z + 0.18
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

    def _execute_trajectory(self, trajectory: JointTrajectory, label: str) -> bool:
        if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
            self.get_logger().error(f"{label}: empty trajectory.")
            return False

        trajectory = self._sanitize_trajectory_for_stage(trajectory, label)
        if len(trajectory.joint_names) == 0 or len(trajectory.points) == 0:
            self.get_logger().error(f"{label}: empty trajectory after sanitizing.")
            return False

        self.get_logger().info(f"{label}: executing joints={trajectory.joint_names}")

        self._publish_trajectory(trajectory)

        if not self._execute_with_controller:
            time.sleep(max(0.1, trajectory_effective_playback_duration(trajectory) + 0.2))
            self._current_joint_state = joint_state_from_trajectory_end(self._current_joint_state, trajectory)
            self._visualization_joint_state_pub.publish(self._current_joint_state)
            self._publish_estimated_achieved_pose(trajectory, label, self._current_joint_state)
            return True

        if not self._trajectory_action.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                f"{label}: controller action '{self._controller_action_name}' unavailable."
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_goal_future = self._trajectory_action.send_goal_async(goal)
        send_done = Event()
        send_goal_future.add_done_callback(lambda _: send_done.set())
        if not send_done.wait(timeout=5.0):
            self.get_logger().error(f"{label}: timed out while sending goal to controller.")
            return False

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label}: controller rejected trajectory.")
            return False

        result_future = goal_handle.get_result_async()
        timeout = max(
            6.0,
            1.5 * trajectory_duration(trajectory) + self._trajectory_execution_timeout_margin,
        )
        result_done = Event()
        result_future.add_done_callback(lambda _: result_done.set())
        if not result_done.wait(timeout=timeout):
            self.get_logger().error(f"{label}: timed out while waiting for controller result.")
            return False

        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error(f"{label}: controller result unavailable.")
            return False

        result = result_wrapper.result
        error_code = int(getattr(result, "error_code", 0))
        if error_code != 0:
            error_string = getattr(result, "error_string", "")
            self.get_logger().error(
                f"{label}: controller execution failed with error_code={error_code}, error='{error_string}'."
            )
            return False
        self._publish_estimated_achieved_pose(trajectory, label)
        return True

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
        if label == "insert_and_suction" and self._latest_target_pose is not None:
            position_error = math.sqrt(
                (achieved_pose.pose.position.x - self._latest_target_pose.pose.position.x) ** 2
                + (achieved_pose.pose.position.y - self._latest_target_pose.pose.position.y) ** 2
                + (achieved_pose.pose.position.z - self._latest_target_pose.pose.position.z) ** 2
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

        allowed_controller_joints = self._controller_allowed_joint_names()
        if self._execute_with_controller and allowed_controller_joints is not None:
            removed_joint_names = [
                joint_name for joint_name in result.joint_names if joint_name not in allowed_controller_joints
            ]
            if removed_joint_names:
                self.get_logger().info(
                    f"{label}: filtering joints not handled by controller "
                    f"'{self._controller_action_name}': {removed_joint_names}"
                )
                result = filter_trajectory_joints(
                    result,
                    allowed_joint_names=allowed_controller_joints,
                )

        if self._discrete_lift_mode:
            arm_only_labels = {
                "stow_arm",
                "plan_to_pre_insert",
                "insert_and_suction",
                "safe_retreat",
                "return_to_stow",
            }
            if label in arm_only_labels:
                result = filter_trajectory_joints(
                    trajectory,
                    allowed_joint_names={"r1_joint", "r2_joint", "r3_joint", "r4_joint", "r5_joint", "r6_joint"},
                )

        return normalize_revolute_trajectory_continuity(
            result,
            seed_joint_state=self._current_joint_state,
            revolute_joint_names={"r1_joint", "r2_joint", "r3_joint", "r4_joint", "r5_joint", "r6_joint"},
        )

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
