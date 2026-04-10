#!/usr/bin/env python3

from __future__ import annotations

import time
from typing import Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmTrajectoryControllerBridge(Node):
    def __init__(self) -> None:
        super().__init__("medipick_motor_arm_trajectory_controller_bridge")

        self.declare_parameter("action_name", "/mobile_arm_controller/follow_joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("motor_arm_joint_command_topic", "/medipick/hardware/motor_arm_joint_command")
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("goal_tolerance", 0.08)
        self.declare_parameter("goal_timeout", 6.0)

        self._arm_joint_names = (
            "r1_joint",
            "r2_joint",
            "r3_joint",
            "r4_joint",
            "r5_joint",
            "r6_joint",
        )
        self._joint_positions: dict[str, float] = {}
        self._callback_group = ReentrantCallbackGroup()
        self._command_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("motor_arm_joint_command_topic").value),
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            20,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("action_name").value),
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            execute_callback=self._execute_goal,
            callback_group=self._callback_group,
        )
        self.get_logger().info("Motor arm trajectory controller bridge ready.")

    def _on_joint_state(self, msg: JointState) -> None:
        joint_count = min(len(msg.name), len(msg.position))
        for index in range(joint_count):
            self._joint_positions[msg.name[index]] = msg.position[index]

    def _handle_goal(self, goal_request: FollowJointTrajectory.Goal):
        trajectory = goal_request.trajectory
        if not trajectory.joint_names or not trajectory.points:
            self.get_logger().warning("Rejecting empty motor-arm trajectory goal.")
            return GoalResponse.REJECT
        invalid_joint_names = [
            joint_name for joint_name in trajectory.joint_names if joint_name not in self._arm_joint_names
        ]
        if invalid_joint_names:
            self.get_logger().warning(
                f"Rejecting motor-arm trajectory with unsupported joints: {invalid_joint_names}"
            )
            return GoalResponse.REJECT
        if len(set(trajectory.joint_names)) != len(trajectory.joint_names):
            self.get_logger().warning("Rejecting motor-arm trajectory with duplicate joint names.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        missing_joint_names = [joint_name for joint_name in joint_names if joint_name not in self._joint_positions]
        if missing_joint_names:
            goal_handle.abort()
            return self._result(
                FollowJointTrajectory.Result.INVALID_GOAL,
                f"Missing motor-arm joint feedback for: {missing_joint_names}",
            )

        point_times = self._normalized_point_times(trajectory)
        start_positions = [self._joint_positions[joint_name] for joint_name in joint_names]
        publish_period = 1.0 / max(float(self.get_parameter("update_rate").value), 1.0)
        start_time = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(
                    FollowJointTrajectory.Result.SUCCESSFUL,
                    "Motor-arm trajectory canceled.",
                )

            elapsed = time.monotonic() - start_time
            positions = self._interpolate_positions(trajectory, point_times, start_positions, elapsed)
            self._publish_command(joint_names, positions)
            if elapsed >= point_times[-1]:
                break
            time.sleep(publish_period)

        final_positions = list(trajectory.points[-1].positions)
        timeout_deadline = time.monotonic() + float(self.get_parameter("goal_timeout").value)
        while time.monotonic() < timeout_deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(
                    FollowJointTrajectory.Result.SUCCESSFUL,
                    "Motor-arm trajectory canceled.",
                )

            self._publish_command(joint_names, final_positions)
            if self._goal_reached(joint_names, final_positions):
                goal_handle.succeed()
                return self._result(FollowJointTrajectory.Result.SUCCESSFUL, "")
            time.sleep(publish_period)

        goal_handle.abort()
        return self._result(
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
            "Motor-arm joints did not reach the commanded final position in time.",
        )

    def _publish_command(self, joint_names: list[str], positions: list[float]) -> None:
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = joint_names
        command.position = positions
        self._command_publisher.publish(command)

    def _goal_reached(self, joint_names: list[str], target_positions: list[float]) -> bool:
        tolerance = float(self.get_parameter("goal_tolerance").value)
        for joint_name, target_position in zip(joint_names, target_positions):
            current_position = self._joint_positions.get(joint_name)
            if current_position is None:
                return False
            if abs(current_position - target_position) > tolerance:
                return False
        return True

    @staticmethod
    def _result(error_code: int, error_string: str) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        result.error_code = error_code
        result.error_string = error_string
        return result

    @staticmethod
    def _normalized_point_times(trajectory) -> list[float]:
        point_times: list[float] = []
        previous_time = 0.0
        for index, point in enumerate(trajectory.points):
            point_time = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            if point_time <= previous_time:
                point_time = previous_time + (0.1 if index > 0 else 0.1)
            point_times.append(point_time)
            previous_time = point_time
        return point_times

    @staticmethod
    def _interpolate_positions(
        trajectory,
        point_times: list[float],
        start_positions: list[float],
        elapsed: float,
    ) -> list[float]:
        if elapsed <= 0.0:
            return list(start_positions)
        if elapsed >= point_times[-1]:
            return list(trajectory.points[-1].positions)

        previous_time = 0.0
        previous_positions = list(start_positions)
        for point_time, point in zip(point_times, trajectory.points):
            if elapsed <= point_time:
                duration = max(point_time - previous_time, 1e-6)
                ratio = min(max((elapsed - previous_time) / duration, 0.0), 1.0)
                return [
                    start_position + (end_position - start_position) * ratio
                    for start_position, end_position in zip(previous_positions, point.positions)
                ]
            previous_time = point_time
            previous_positions = list(point.positions)

        return list(trajectory.points[-1].positions)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ArmTrajectoryControllerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
