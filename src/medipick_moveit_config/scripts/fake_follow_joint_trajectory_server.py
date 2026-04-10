#!/usr/bin/env python3

from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class FakeFollowJointTrajectoryServer(Node):
    def __init__(self) -> None:
        super().__init__("fake_follow_joint_trajectory_server")

        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("visualization_trajectory_topic", "/medipick/visualization_trajectory")

        self._controller_name = str(self.get_parameter("controller_name").value).strip()
        action_name = f"/{self._controller_name}/follow_joint_trajectory"
        self._trajectory_publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("visualization_trajectory_topic").value),
            10,
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            execute_callback=self._execute_goal,
        )
        self.get_logger().info(f"Fake FollowJointTrajectory server ready on {action_name}")

    def _handle_goal(self, goal_request: FollowJointTrajectory.Goal):
        trajectory = goal_request.trajectory
        if not trajectory.joint_names or not trajectory.points:
            self.get_logger().warning(
                f"{self._controller_name}: rejecting empty trajectory goal."
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, _goal_handle):
        self.get_logger().info(f"{self._controller_name}: cancel request accepted.")
        return CancelResponse.ACCEPT

    def _execute_goal(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self.get_logger().info(
            f"{self._controller_name}: executing trajectory with "
            f"{len(trajectory.points)} points for joints {list(trajectory.joint_names)}"
        )
        self._trajectory_publisher.publish(trajectory)

        final_time = self._trajectory_duration(trajectory)
        end_time = time.monotonic() + final_time
        while time.monotonic() < end_time:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory execution canceled."
                return result

            remaining = max(end_time - time.monotonic(), 0.0)
            sleep_duration = min(0.05, remaining)
            if sleep_duration > 0.0:
                time.sleep(sleep_duration)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ""
        return result

    @staticmethod
    def _trajectory_duration(trajectory: JointTrajectory) -> float:
        if not trajectory.points:
            return 0.0
        last_point = trajectory.points[-1]
        return (
            float(last_point.time_from_start.sec)
            + float(last_point.time_from_start.nanosec) * 1e-9
            + 0.1
        )


def main() -> None:
    rclpy.init()
    node = FakeFollowJointTrajectoryServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
