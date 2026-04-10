#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class DefaultJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("default_joint_state_publisher")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("trajectory_topic", "/medipick/visualization_trajectory")
        self.declare_parameter("visualization_joint_state_topic", "/medipick/visualization_joint_state")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_names = [
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
            "sucker_joint",
            "h1_joint",
            "h2_joint",
        ]
        self.default_positions = {
            "base_x": 0.0,
            "base_y": 0.0,
            "base_theta": 0.0,
            "raise_joint": 0.302,
            "r1_joint": -0.681,
            "r2_joint": -1.309,
            "r3_joint": -1.309,
            "r4_joint": -1.571,
            "r5_joint": 0.017,
            "r6_joint": 0.087,
            "sucker_joint": 0.0,
            "h1_joint": -0.424124,
            "h2_joint": 0.432157,
        }
        self.current_positions = dict(self.default_positions)
        self._active_trajectory = None
        self._active_trajectory_start = 0.0

        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter("trajectory_topic").value),
            self.handle_trajectory,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("visualization_joint_state_topic").value),
            self.handle_joint_state_override,
            10,
        )
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self.publish_state)

    def handle_trajectory(self, message: JointTrajectory) -> None:
        if not message.joint_names or not message.points:
            return

        point_times = []
        last_time = 0.0
        for index, point in enumerate(message.points):
            time_from_start = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            if time_from_start <= last_time:
                time_from_start = last_time + (0.15 if index > 0 else 0.15)
            point_times.append(time_from_start)
            last_time = time_from_start

        self._active_trajectory = {
            "joint_names": list(message.joint_names),
            "points": [list(point.positions) for point in message.points],
            "times": point_times,
            "base_positions": dict(self.current_positions),
        }
        self._active_trajectory_start = time.monotonic()

    def handle_joint_state_override(self, message: JointState) -> None:
        self._active_trajectory = None
        joint_count = min(len(message.name), len(message.position))
        for index in range(joint_count):
            joint_name = message.name[index]
            if joint_name in self.current_positions:
                self.current_positions[joint_name] = message.position[index]

    def publish_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        if self._active_trajectory is not None:
            self._update_positions_from_active_trajectory()
        message.position = [self.current_positions[name] for name in self.joint_names]
        self.publisher.publish(message)

    def _update_positions_from_active_trajectory(self) -> None:
        trajectory = self._active_trajectory
        if trajectory is None:
            return

        elapsed = time.monotonic() - self._active_trajectory_start
        point_times = trajectory["times"]
        point_positions = trajectory["points"]
        joint_names = trajectory["joint_names"]

        if elapsed >= point_times[-1]:
            for joint_name, position in zip(joint_names, point_positions[-1]):
                if joint_name in self.current_positions:
                    self.current_positions[joint_name] = position
            self._active_trajectory = None
            return

        previous_time = 0.0
        previous_positions = [
            trajectory["base_positions"].get(joint_name, self.current_positions.get(joint_name, 0.0))
            for joint_name in joint_names
        ]

        for next_time, next_positions in zip(point_times, point_positions):
            if elapsed <= next_time:
                duration = max(next_time - previous_time, 1e-6)
                ratio = min(max((elapsed - previous_time) / duration, 0.0), 1.0)
                for joint_name, start_position, end_position in zip(
                    joint_names,
                    previous_positions,
                    next_positions,
                ):
                    if joint_name in self.current_positions:
                        self.current_positions[joint_name] = (
                            start_position + (end_position - start_position) * ratio
                        )
                return
            previous_time = next_time
            previous_positions = next_positions


def main() -> None:
    rclpy.init()
    node = DefaultJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
