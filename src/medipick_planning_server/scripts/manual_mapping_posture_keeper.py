#!/usr/bin/python3

from __future__ import annotations

import time
from typing import Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pick_task_shared import DEFAULT_STOW_STATE_POSITIONS


class ManualMappingPostureKeeper(Node):
    def __init__(self) -> None:
        super().__init__("medipick_manual_mapping_posture_keeper")

        stow_state = dict(DEFAULT_STOW_STATE_POSITIONS)
        arm_joint_names = ["r1_joint", "r2_joint", "r3_joint", "r4_joint", "r5_joint", "r6_joint"]
        arm_positions = [float(stow_state[name]) for name in arm_joint_names]

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("lift_topic", "/medipick/task/lift_target_height")
        self.declare_parameter("arm_action_name", "/mobile_arm_controller/follow_joint_trajectory")
        self.declare_parameter("head_action_name", "/head_controller/follow_joint_trajectory")
        self.declare_parameter("arm_joint_names", arm_joint_names)
        self.declare_parameter("arm_positions", arm_positions)
        self.declare_parameter("head_joint_names", ["h1_joint", "h2_joint"])
        self.declare_parameter("head_positions", [0.0, -0.5236])
        self.declare_parameter("hold_head", True)
        self.declare_parameter("lift_height", float(stow_state["raise_joint"]))
        self.declare_parameter("trajectory_duration_sec", 1.5)
        self.declare_parameter("joint_tolerance", 0.04)
        self.declare_parameter("lift_publish_rate_hz", 2.0)
        self.declare_parameter("posture_check_rate_hz", 2.0)
        self.declare_parameter("resend_period_sec", 3.0)

        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._lift_topic = str(self.get_parameter("lift_topic").value)
        self._arm_action_name = str(self.get_parameter("arm_action_name").value)
        self._head_action_name = str(self.get_parameter("head_action_name").value)
        self._arm_joint_names = tuple(str(value) for value in self.get_parameter("arm_joint_names").value)
        self._arm_positions = tuple(float(value) for value in self.get_parameter("arm_positions").value)
        self._head_joint_names = tuple(str(value) for value in self.get_parameter("head_joint_names").value)
        self._head_positions = tuple(float(value) for value in self.get_parameter("head_positions").value)
        self._hold_head = bool(self.get_parameter("hold_head").value)
        self._lift_height = float(self.get_parameter("lift_height").value)
        self._trajectory_duration_sec = max(0.5, float(self.get_parameter("trajectory_duration_sec").value))
        self._joint_tolerance = max(0.005, float(self.get_parameter("joint_tolerance").value))
        lift_publish_rate_hz = max(0.2, float(self.get_parameter("lift_publish_rate_hz").value))
        posture_check_rate_hz = max(0.2, float(self.get_parameter("posture_check_rate_hz").value))
        self._resend_period_sec = max(0.5, float(self.get_parameter("resend_period_sec").value))

        self._current_joint_state = JointState()
        self._last_arm_send_at = 0.0
        self._last_head_send_at = 0.0
        self._arm_goal_in_flight = False
        self._head_goal_in_flight = False
        self._arm_ready_logged = False
        self._head_ready_logged = False

        self.create_subscription(JointState, self._joint_state_topic, self._on_joint_state, 20)
        self._lift_pub = self.create_publisher(Float64, self._lift_topic, 10)
        self._arm_action = ActionClient(self, FollowJointTrajectory, self._arm_action_name)
        self._head_action = ActionClient(self, FollowJointTrajectory, self._head_action_name)
        self.create_timer(1.0 / lift_publish_rate_hz, self._publish_lift_target)
        self.create_timer(1.0 / posture_check_rate_hz, self._enforce_posture)

        self.get_logger().info(
            "手动建图姿态保持器已启动："
            f" lift={self._lift_height:.3f}, "
            f"arm={list(zip(self._arm_joint_names, self._arm_positions))}, "
            f"head_hold={self._hold_head}, "
            f"head={list(zip(self._head_joint_names, self._head_positions))}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._current_joint_state = msg

    def _publish_lift_target(self) -> None:
        self._lift_pub.publish(Float64(data=self._lift_height))

    def _joint_positions_available(self, joint_names: tuple[str, ...]) -> bool:
        return all(name in self._current_joint_state.name for name in joint_names)

    def _joint_position(self, joint_name: str, default: float = 0.0) -> float:
        if joint_name not in self._current_joint_state.name:
            return default
        index = self._current_joint_state.name.index(joint_name)
        if index >= len(self._current_joint_state.position):
            return default
        return float(self._current_joint_state.position[index])

    def _posture_reached(self, joint_names: tuple[str, ...], target_positions: tuple[float, ...]) -> bool:
        if not self._joint_positions_available(joint_names):
            return False
        for joint_name, target in zip(joint_names, target_positions):
            if abs(self._joint_position(joint_name) - target) > self._joint_tolerance:
                return False
        return True

    def _build_trajectory(self, joint_names: tuple[str, ...], target_positions: tuple[float, ...]) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        total_ns = int(self._trajectory_duration_sec * 1e9)
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000
        trajectory.points = [point]
        return trajectory

    def _send_action_goal(
        self,
        action_client: ActionClient,
        joint_names: tuple[str, ...],
        target_positions: tuple[float, ...],
        kind: str,
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._build_trajectory(joint_names, target_positions)
        future = action_client.send_goal_async(goal)
        future.add_done_callback(lambda done: self._on_goal_response(done, kind))

    def _on_goal_response(self, future, kind: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"{kind} 姿态目标发送失败：{exc}")
            if kind == "arm":
                self._arm_goal_in_flight = False
            else:
                self._head_goal_in_flight = False
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(f"{kind} 姿态目标被控制器拒绝。")
            if kind == "arm":
                self._arm_goal_in_flight = False
            else:
                self._head_goal_in_flight = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done: self._on_goal_result(done, kind))

    def _on_goal_result(self, _future, kind: str) -> None:
        if kind == "arm":
            self._arm_goal_in_flight = False
        else:
            self._head_goal_in_flight = False

    def _enforce_posture(self) -> None:
        if len(self._current_joint_state.name) == 0:
            return

        now = time.monotonic()

        if self._arm_action.server_is_ready() and not self._arm_ready_logged:
            self._arm_ready_logged = True
            self.get_logger().info("mobile_arm_controller 已就绪，开始保持收臂姿态。")
        if self._hold_head and self._head_action.server_is_ready() and not self._head_ready_logged:
            self._head_ready_logged = True
            self.get_logger().info("head_controller 已就绪，开始保持建图头部姿态。")
        if not self._hold_head and not self._head_ready_logged:
            self._head_ready_logged = True
            self.get_logger().info("head_controller 头部保持已关闭，允许手动控制 h1/h2。")

        if (
            self._arm_action.server_is_ready()
            and not self._arm_goal_in_flight
            and now - self._last_arm_send_at >= self._resend_period_sec
            and not self._posture_reached(self._arm_joint_names, self._arm_positions)
        ):
            self._arm_goal_in_flight = True
            self._last_arm_send_at = now
            self.get_logger().info("发送收臂姿态目标。")
            self._send_action_goal(self._arm_action, self._arm_joint_names, self._arm_positions, "arm")

        if (
            self._hold_head
            and self._head_action.server_is_ready()
            and not self._head_goal_in_flight
            and now - self._last_head_send_at >= self._resend_period_sec
            and not self._posture_reached(self._head_joint_names, self._head_positions)
        ):
            self._head_goal_in_flight = True
            self._last_head_send_at = now
            self.get_logger().info("发送建图头部姿态目标。")
            self._send_action_goal(self._head_action, self._head_joint_names, self._head_positions, "head")

        if (
            self._head_action.server_is_ready()
            and not self._hold_head
            and self._head_goal_in_flight
        ):
            self._head_goal_in_flight = False


def main() -> None:
    rclpy.init()
    node = ManualMappingPostureKeeper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
