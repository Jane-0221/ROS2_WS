#!/usr/bin/python3

from __future__ import annotations

import math
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


class PostPickDeliveryManager(Node):
    def __init__(self) -> None:
        super().__init__("medipick_post_pick_delivery_manager")

        self.declare_parameter("stage_topic", "/medipick/task/stage")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("tool_action_name", "/tool_controller/follow_joint_trajectory")
        self.declare_parameter("delivery_goal_frame", "map")
        self.declare_parameter("delivery_goal_x", 2.15)
        self.declare_parameter("delivery_goal_y", 0.0)
        self.declare_parameter("delivery_goal_yaw_deg", 180.0)
        self.declare_parameter("delivery_trigger_stage", "COMPLETED")
        self.declare_parameter("enable_release_after_delivery", True)
        self.declare_parameter("release_joint_name", "sucker_joint")
        self.declare_parameter("release_joint_position", 0.0)
        self.declare_parameter("release_motion_duration_sec", 1.0)
        self.declare_parameter("navigate_server_wait_sec", 1.0)
        self.declare_parameter("tool_server_wait_sec", 0.5)

        self._stage_topic = str(self.get_parameter("stage_topic").value)
        self._navigate_action_name = str(self.get_parameter("navigate_action_name").value)
        self._tool_action_name = str(self.get_parameter("tool_action_name").value)
        self._delivery_goal_frame = str(self.get_parameter("delivery_goal_frame").value)
        self._delivery_goal_x = float(self.get_parameter("delivery_goal_x").value)
        self._delivery_goal_y = float(self.get_parameter("delivery_goal_y").value)
        self._delivery_goal_yaw_deg = float(self.get_parameter("delivery_goal_yaw_deg").value)
        self._delivery_trigger_stage = str(self.get_parameter("delivery_trigger_stage").value).strip().upper()
        self._enable_release_after_delivery = bool(self.get_parameter("enable_release_after_delivery").value)
        self._release_joint_name = str(self.get_parameter("release_joint_name").value).strip()
        self._release_joint_position = float(self.get_parameter("release_joint_position").value)
        self._release_motion_duration_sec = max(
            0.1, float(self.get_parameter("release_motion_duration_sec").value)
        )
        self._navigate_server_wait_sec = max(0.1, float(self.get_parameter("navigate_server_wait_sec").value))
        self._tool_server_wait_sec = max(0.1, float(self.get_parameter("tool_server_wait_sec").value))

        self._stage_sub = self.create_subscription(String, self._stage_topic, self._on_stage, 10)
        self._navigate_action = ActionClient(self, NavigateToPose, self._navigate_action_name)
        self._tool_action = ActionClient(self, FollowJointTrajectory, self._tool_action_name)

        self._delivery_requested = False
        self._delivery_goal_in_flight = False
        self._delivery_completed = False
        self._release_requested = False

        self.get_logger().info(
            "后置放置管理器已启动："
            f" stage_topic={self._stage_topic}, delivery_goal=({self._delivery_goal_x:.2f}, "
            f"{self._delivery_goal_y:.2f}, {self._delivery_goal_yaw_deg:.1f}deg)"
        )

    def _on_stage(self, msg: String) -> None:
        stage = msg.data.strip().upper()
        if stage != self._delivery_trigger_stage:
            return
        if self._delivery_requested or self._delivery_goal_in_flight or self._delivery_completed:
            return
        self._request_delivery_goal()

    def _request_delivery_goal(self) -> None:
        if not self._navigate_action.wait_for_server(timeout_sec=self._navigate_server_wait_sec):
            self.get_logger().warning("放置导航等待 NavigateToPose action server 超时，将继续重试。")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self._delivery_goal_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self._delivery_goal_x
        goal_pose.pose.position.y = self._delivery_goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = quaternion_from_yaw(math.radians(self._delivery_goal_yaw_deg))

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self._delivery_requested = True
        self._delivery_goal_in_flight = True
        future = self._navigate_action.send_goal_async(goal)
        future.add_done_callback(self._on_delivery_goal_response)
        self.get_logger().info(
            f"抓取已完成，开始导航到放置点：x={self._delivery_goal_x:.2f}, "
            f"y={self._delivery_goal_y:.2f}, yaw={self._delivery_goal_yaw_deg:.1f}deg"
        )

    def _on_delivery_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("放置点导航目标被拒绝。")
            self._delivery_goal_in_flight = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_delivery_result)

    def _on_delivery_result(self, future) -> None:
        self._delivery_goal_in_flight = False
        result_wrapper = future.result()
        if result_wrapper is None:
            self.get_logger().error("放置点导航结果不可用。")
            return
        if result_wrapper.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f"放置点导航失败，status={result_wrapper.status}")
            return

        self._delivery_completed = True
        self.get_logger().info("已到达放置点。")
        if self._enable_release_after_delivery:
            self._request_release()

    def _request_release(self) -> None:
        if self._release_requested:
            return
        if not self._tool_action.wait_for_server(timeout_sec=self._tool_server_wait_sec):
            self.get_logger().warning("放置释放等待 tool_controller 超时，跳过释放动作。")
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = [self._release_joint_name]
        point = JointTrajectoryPoint()
        point.positions = [self._release_joint_position]
        total_ns = int(self._release_motion_duration_sec * 1e9)
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._release_requested = True
        future = self._tool_action.send_goal_async(goal)
        future.add_done_callback(self._on_release_goal_response)
        self.get_logger().info(
            f"已到达放置点，发送释放动作：{self._release_joint_name} -> {self._release_joint_position:.3f}"
        )

    def _on_release_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("释放动作目标被拒绝。")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_release_result)

    def _on_release_result(self, future) -> None:
        result_wrapper = future.result()
        if result_wrapper is None:
            self.get_logger().error("释放动作结果不可用。")
            return
        if result_wrapper.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("放置点释放动作完成。")
        else:
            self.get_logger().error(f"放置点释放动作失败，status={result_wrapper.status}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PostPickDeliveryManager()
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
