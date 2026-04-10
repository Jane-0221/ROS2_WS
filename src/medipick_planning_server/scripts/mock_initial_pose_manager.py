#!/usr/bin/env python3

from __future__ import annotations

import math
import random
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_FULL_STATE = {
    "base_x": 0.0,
    "base_y": 0.0,
    "base_theta": 0.0,
    "raise_joint": 0.22,
    "r1_joint": 0.10,
    "r2_joint": 1.20,
    "r3_joint": -4.555309,
    "r4_joint": 1.10,
    "r5_joint": 0.45,
    "r6_joint": 0.15,
    "sucker_joint": 0.0,
    "h1_joint": -0.424124,
    "h2_joint": 0.432157,
    "l1_joint": -0.845803,
    "l2_joint": -0.545632,
    "l3_joint": 0.771423,
    "l4_joint": -1.044046,
    "l5_joint": -0.033111,
}


class MockInitialPoseManager(Node):
    def __init__(self) -> None:
        super().__init__("medipick_mock_initial_pose_manager")

        self.declare_parameter("use_ros2_control", False)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("initial_motion_duration", 2.5)
        self.declare_parameter("visualization_joint_state_topic", "/medipick/visualization_joint_state")
        self.declare_parameter("controller_action_name", "/mobile_arm_controller/follow_joint_trajectory")
        self.declare_parameter("target_pose_topic", "/medipick/task/target_pose")
        self.declare_parameter("task_start_service", "/medipick/task/start")
        self.declare_parameter("auto_start_task", True)

        self._use_ros2_control = bool(self.get_parameter("use_ros2_control").value)
        self._initial_motion_duration = float(self.get_parameter("initial_motion_duration").value)
        self._auto_start_task = bool(self.get_parameter("auto_start_task").value)
        self._target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self._task_start_service_name = str(self.get_parameter("task_start_service").value)

        seed = int(self.get_parameter("random_seed").value)
        if seed < 0:
            seed = time.time_ns() & 0xFFFFFFFF
        self._seed = seed
        self._rng = random.Random(seed)

        self._current_joint_state = JointState()
        self._have_target = False
        self._initial_pose_sent = False
        self._initial_pose_done = False
        self._start_requested = False
        self._override_publish_deadline = 0.0

        self._visual_joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("visualization_joint_state_topic").value),
            10,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(PoseStamped, self._target_pose_topic, self._on_target_pose, 10)

        self._start_client = self.create_client(Trigger, self._task_start_service_name)
        self._trajectory_action = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("controller_action_name").value),
        )

        self._sampled_full_state = self._sample_initial_state()
        self._sampled_mobile_arm_state = {name: self._sampled_full_state[name] for name in MOBILE_ARM_JOINTS}
        self.get_logger().info(
            "Random initial pose sampled with seed=%d: base=(%.3f, %.3f, %.3f), raise=%.3f, "
            "arm=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]"
            % (
                self._seed,
                self._sampled_mobile_arm_state["base_x"],
                self._sampled_mobile_arm_state["base_y"],
                self._sampled_mobile_arm_state["base_theta"],
                self._sampled_mobile_arm_state["raise_joint"],
                self._sampled_mobile_arm_state["r1_joint"],
                self._sampled_mobile_arm_state["r2_joint"],
                self._sampled_mobile_arm_state["r3_joint"],
                self._sampled_mobile_arm_state["r4_joint"],
                self._sampled_mobile_arm_state["r5_joint"],
                self._sampled_mobile_arm_state["r6_joint"],
            )
        )

        self.create_timer(0.1, self._tick)

    def _on_joint_state(self, msg: JointState) -> None:
        self._current_joint_state = msg

    def _on_target_pose(self, _msg) -> None:
        self._have_target = True

    def _tick(self) -> None:
        if not self._initial_pose_sent:
            self._dispatch_initial_pose()
            return

        if not self._initial_pose_done:
            if not self._use_ros2_control and time.monotonic() >= self._override_publish_deadline:
                self._initial_pose_done = True
            return

        if time.monotonic() < self._override_publish_deadline:
            self._visual_joint_state_pub.publish(build_joint_state(self._sampled_full_state))

        if not self._auto_start_task or self._start_requested or not self._have_target:
            return

        if not self._start_client.wait_for_service(timeout_sec=0.1):
            return
        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_start_response)
        self._start_requested = True

    def _dispatch_initial_pose(self) -> None:
        self._initial_pose_sent = True
        if not self._use_ros2_control:
            self._override_publish_deadline = time.monotonic() + max(1.0, self._initial_motion_duration)
            self._visual_joint_state_pub.publish(build_joint_state(self._sampled_full_state))
            self.get_logger().info("Published randomized initial joint state override.")
            return

        if not self._trajectory_action.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Initial pose action server unavailable.")
            self._initial_pose_done = True
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = list(MOBILE_ARM_JOINTS)

        start_positions = current_positions_for(MOBILE_ARM_JOINTS, self._current_joint_state, self._sampled_mobile_arm_state)
        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0

        end_point = JointTrajectoryPoint()
        end_point.positions = [self._sampled_mobile_arm_state[name] for name in MOBILE_ARM_JOINTS]
        total_ns = int(max(0.5, self._initial_motion_duration) * 1e9)
        end_point.time_from_start.sec = total_ns // 1_000_000_000
        end_point.time_from_start.nanosec = total_ns % 1_000_000_000
        trajectory.points = [start_point, end_point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._trajectory_action.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_sent)
        self.get_logger().info("Sent randomized initial pose to mobile_arm_controller.")

    def _on_goal_sent(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Randomized initial pose goal was rejected.")
            self._initial_pose_done = True
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        result_wrapper = future.result()
        if result_wrapper is None:
            self.get_logger().error("Randomized initial pose result unavailable.")
        else:
            error_code = int(getattr(result_wrapper.result, "error_code", 0))
            if error_code == 0:
                self.get_logger().info("Randomized initial pose reached.")
            else:
                self.get_logger().error(f"Randomized initial pose failed with error_code={error_code}.")
        self._initial_pose_done = True

    def _on_start_response(self, future) -> None:
        response = future.result()
        if response is None:
            self.get_logger().error("Failed to call /medipick/task/start after initialization.")
            return
        if response.success:
            self.get_logger().info("Task started after randomized initialization.")
        else:
            self.get_logger().warning(f"Task start after initialization was rejected: {response.message}")
            self._start_requested = False

    def _sample_initial_state(self) -> dict[str, float]:
        state = dict(DEFAULT_FULL_STATE)
        state["base_x"] = self._rng.uniform(-0.20, 0.20)
        state["base_y"] = self._rng.uniform(-0.30, 0.30)
        state["base_theta"] = self._rng.uniform(-math.pi, math.pi)
        state["raise_joint"] = self._rng.uniform(0.10, 0.42)
        arm_seed = self._rng.choice(INITIAL_ARM_BRANCH_SEEDS)
        state["r1_joint"] = clamp(arm_seed["r1_joint"] + self._rng.uniform(-0.18, 0.18), -0.60, 0.60)
        state["r2_joint"] = clamp(arm_seed["r2_joint"] + self._rng.uniform(-0.16, 0.16), 0.90, 1.45)
        state["r3_joint"] = normalize_angle(arm_seed["r3_joint"] + self._rng.uniform(-0.22, 0.22))
        state["r4_joint"] = clamp(arm_seed["r4_joint"] + self._rng.uniform(-0.18, 0.18), 0.90, 1.45)
        state["r5_joint"] = clamp(arm_seed["r5_joint"] + self._rng.uniform(-0.25, 0.25), 0.15, 0.95)
        state["r6_joint"] = clamp(arm_seed["r6_joint"] + self._rng.uniform(-0.20, 0.20), -0.15, 0.45)
        return state


MOBILE_ARM_JOINTS = (
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
)

INITIAL_ARM_BRANCH_SEEDS = (
    {
        "r1_joint": 0.05,
        "r2_joint": 1.20,
        "r3_joint": -4.555309,
        "r4_joint": 1.08,
        "r5_joint": 0.38,
        "r6_joint": 0.15,
    },
    {
        "r1_joint": 0.30,
        "r2_joint": 1.10,
        "r3_joint": -4.355309,
        "r4_joint": 1.18,
        "r5_joint": 0.52,
        "r6_joint": 0.18,
    },
    {
        "r1_joint": -0.30,
        "r2_joint": 1.10,
        "r3_joint": -4.755309,
        "r4_joint": 1.18,
        "r5_joint": 0.52,
        "r6_joint": 0.18,
    },
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def build_joint_state(position_by_name: dict[str, float]) -> JointState:
    msg = JointState()
    msg.name = list(position_by_name.keys())
    msg.position = [position_by_name[name] for name in msg.name]
    return msg


def current_positions_for(joint_names, current_joint_state: JointState, fallback_positions: dict[str, float]):
    current_by_name = {name: pos for name, pos in zip(current_joint_state.name, current_joint_state.position)}
    return [current_by_name.get(name, fallback_positions[name]) for name in joint_names]


def main() -> None:
    rclpy.init()
    node = MockInitialPoseManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
