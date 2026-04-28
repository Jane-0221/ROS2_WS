#!/usr/bin/python3

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboPreNavigationManager(Node):
    def __init__(self) -> None:
        super().__init__("medipick_gazebo_pre_navigation_manager")

        self.declare_parameter("target_pose_topic", "/medipick/task/target_pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("task_start_service", "/medipick/task/start")
        self.declare_parameter("base_offset", 0.95)
        self.declare_parameter("base_lateral_offset", 0.0)
        self.declare_parameter("base_yaw_offset_deg", 0.0)
        self.declare_parameter("navigation_route_mode", "central_corridor")
        self.declare_parameter("navigation_corridor_y", 0.0)
        self.declare_parameter("waypoint_position_tolerance", 0.05)
        self.declare_parameter("waypoint_yaw_tolerance_deg", 8.0)
        self.declare_parameter("final_position_tolerance", 0.04)
        self.declare_parameter("final_yaw_tolerance_deg", 6.0)
        self.declare_parameter("kp_linear", 1.2)
        self.declare_parameter("kp_lateral", 1.2)
        self.declare_parameter("kp_angular", 1.6)
        self.declare_parameter("max_linear_speed", 0.22)
        self.declare_parameter("max_lateral_speed", 0.22)
        self.declare_parameter("max_angular_speed", 0.55)
        self.declare_parameter("settle_time", 1.0)
        self.declare_parameter("auto_start_task", True)

        self._target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._task_start_service = str(self.get_parameter("task_start_service").value)
        self._base_offset = float(self.get_parameter("base_offset").value)
        self._base_lateral_offset = float(self.get_parameter("base_lateral_offset").value)
        self._base_yaw_offset = math.radians(float(self.get_parameter("base_yaw_offset_deg").value))
        self._route_mode = str(self.get_parameter("navigation_route_mode").value).strip()
        self._corridor_y = float(self.get_parameter("navigation_corridor_y").value)
        self._waypoint_position_tolerance = max(0.02, float(self.get_parameter("waypoint_position_tolerance").value))
        self._waypoint_yaw_tolerance = math.radians(
            max(2.0, float(self.get_parameter("waypoint_yaw_tolerance_deg").value))
        )
        self._final_position_tolerance = max(0.01, float(self.get_parameter("final_position_tolerance").value))
        self._final_yaw_tolerance = math.radians(max(1.0, float(self.get_parameter("final_yaw_tolerance_deg").value)))
        self._kp_linear = max(0.1, float(self.get_parameter("kp_linear").value))
        self._kp_lateral = max(0.1, float(self.get_parameter("kp_lateral").value))
        self._kp_angular = max(0.1, float(self.get_parameter("kp_angular").value))
        self._max_linear_speed = max(0.05, float(self.get_parameter("max_linear_speed").value))
        self._max_lateral_speed = max(0.05, float(self.get_parameter("max_lateral_speed").value))
        self._max_angular_speed = max(0.05, float(self.get_parameter("max_angular_speed").value))
        self._settle_time = max(0.0, float(self.get_parameter("settle_time").value))
        self._auto_start_task = bool(self.get_parameter("auto_start_task").value)

        self._latest_target_pose: PoseStamped | None = None
        self._latest_odom: Odometry | None = None
        self._route: list[dict[str, float]] = []
        self._route_index = 0
        self._settled_at = 0.0
        self._task_started = False
        self._finished = False
        self._last_state_log = 0.0

        self.create_subscription(PoseStamped, self._target_pose_topic, self._on_target_pose, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 20)
        self._start_client = self.create_client(Trigger, self._task_start_service)
        self.create_timer(0.05, self._on_timer)

        self.get_logger().info(
            "Gazebo 前置导航管理器已启动："
            f" target={self._target_pose_topic}, odom={self._odom_topic}, cmd_vel={self._cmd_vel_topic}"
        )

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self._latest_target_pose = msg
        if not self._route:
            self.get_logger().info(
                "前置导航收到目标药盒位姿，等待里程计后开始规划货架工作区停靠路线。"
            )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _current_state(self) -> dict[str, float] | None:
        if self._latest_odom is None:
            return None
        pose = self._latest_odom.pose.pose
        return {
            "base_x": pose.position.x,
            "base_y": pose.position.y,
            "base_theta": quaternion_to_yaw(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            ),
        }

    def _build_target_state(self, target_pose: PoseStamped) -> dict[str, float]:
        yaw = quaternion_to_yaw(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w,
        )
        lateral_dx = -math.sin(yaw) * self._base_lateral_offset
        lateral_dy = math.cos(yaw) * self._base_lateral_offset
        return {
            "base_x": target_pose.pose.position.x - math.cos(yaw) * self._base_offset + lateral_dx,
            "base_y": target_pose.pose.position.y - math.sin(yaw) * self._base_offset + lateral_dy,
            "base_theta": normalize_angle(yaw + self._base_yaw_offset),
        }

    def _build_route(self, current_state: dict[str, float], target_state: dict[str, float]) -> list[dict[str, float]]:
        if self._route_mode != "central_corridor":
            return [target_state]

        route: list[dict[str, float]] = []
        current_x = current_state["base_x"]
        current_y = current_state["base_y"]
        target_x = target_state["base_x"]
        corridor_y = self._corridor_y

        if abs(current_y - corridor_y) > self._waypoint_position_tolerance:
            route.append(
                {
                    "base_x": current_x,
                    "base_y": corridor_y,
                    "base_theta": math.pi / 2.0 if corridor_y >= current_y else -math.pi / 2.0,
                }
            )
        if abs(target_x - current_x) > self._waypoint_position_tolerance:
            route.append(
                {
                    "base_x": target_x,
                    "base_y": corridor_y,
                    "base_theta": 0.0 if target_x >= current_x else math.pi,
                }
            )
        route.append(dict(target_state))
        return route

    def _publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _request_task_start(self) -> None:
        if self._task_started:
            return
        if not self._start_client.wait_for_service(timeout_sec=0.1):
            return
        future = self._start_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_task_start_response)
        self._task_started = True

    def _on_task_start_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"前置导航完成后调用任务启动失败：{exc}")
            self._task_started = False
            return
        if response is None:
            self._task_started = False
            return
        if response.success:
            self.get_logger().info("前置导航完成，已触发抓取任务启动。")
        else:
            self.get_logger().warning(f"前置导航完成，但任务启动被拒绝：{response.message}")
            self._task_started = False

    def _on_timer(self) -> None:
        current_state = self._current_state()
        if current_state is None or self._latest_target_pose is None:
            now = time.monotonic()
            if now - self._last_state_log > 2.0:
                self._last_state_log = now
                if current_state is None:
                    self.get_logger().info("前置导航仍在等待 /odom。")
                elif self._latest_target_pose is None:
                    self.get_logger().info("前置导航仍在等待目标药盒位姿。")
            return

        if not self._route:
            target_state = self._build_target_state(self._latest_target_pose)
            self._route = self._build_route(current_state, target_state)
            self._route_index = 0
            self.get_logger().info(
                f"前置导航已生成 {len(self._route)} 段目标点，路线模式={self._route_mode}。"
            )

        if self._finished:
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            if self._auto_start_task and (time.monotonic() - self._settled_at) >= self._settle_time:
                self._request_task_start()
            return

        waypoint = self._route[self._route_index]
        error_x = waypoint["base_x"] - current_state["base_x"]
        error_y = waypoint["base_y"] - current_state["base_y"]
        error_theta = normalize_angle(waypoint["base_theta"] - current_state["base_theta"])

        position_tolerance = (
            self._final_position_tolerance
            if self._route_index == len(self._route) - 1
            else self._waypoint_position_tolerance
        )
        yaw_tolerance = (
            self._final_yaw_tolerance
            if self._route_index == len(self._route) - 1
            else self._waypoint_yaw_tolerance
        )
        planar_error = math.hypot(error_x, error_y)
        if planar_error <= position_tolerance and abs(error_theta) <= yaw_tolerance:
            self._publish_cmd_vel(0.0, 0.0, 0.0)
            if self._route_index + 1 < len(self._route):
                self._route_index += 1
                self.get_logger().info(
                    f"前置导航已到达中间路点，切换到下一段：{self._route_index + 1}/{len(self._route)}"
                )
            else:
                self._finished = True
                self._settled_at = time.monotonic()
                self.get_logger().info("前置导航已到达货架工作区，准备启动抓取任务。")
            return

        current_theta = current_state["base_theta"]
        error_forward = math.cos(current_theta) * error_x + math.sin(current_theta) * error_y
        error_lateral = -math.sin(current_theta) * error_x + math.cos(current_theta) * error_y

        cmd_x = max(-self._max_linear_speed, min(self._max_linear_speed, self._kp_linear * error_forward))
        cmd_y = max(-self._max_lateral_speed, min(self._max_lateral_speed, self._kp_lateral * error_lateral))
        cmd_w = max(-self._max_angular_speed, min(self._max_angular_speed, self._kp_angular * error_theta))

        if planar_error < 0.08:
            cmd_x *= 0.5
            cmd_y *= 0.5
        if abs(error_theta) < math.radians(10.0):
            cmd_w *= 0.5

        self._publish_cmd_vel(cmd_x, cmd_y, cmd_w)


def main() -> None:
    rclpy.init()
    node = GazeboPreNavigationManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
