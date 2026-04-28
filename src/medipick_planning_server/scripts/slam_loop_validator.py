#!/usr/bin/python3

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

from rtabmap_msgs.msg import Info as RtabmapInfo


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


class SlamLoopValidator(Node):
    def __init__(self) -> None:
        super().__init__("medipick_slam_loop_validator")

        self.declare_parameter("map_topic", "/rtabmap/grid_prob_map")
        self.declare_parameter("rtabmap_info_topic", "/rtabmap/info")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("min_known_map_cells", 10)
        self.declare_parameter("require_robot_inside_map", True)
        self.declare_parameter("map_ready_margin_m", 0.05)
        self.declare_parameter("wait_after_map_ready_sec", 2.0)
        self.declare_parameter("wait_after_action_server_ready_sec", 2.0)
        self.declare_parameter("wait_after_each_goal_sec", 0.5)
        self.declare_parameter("waypoint_timeout_sec", 40.0)
        self.declare_parameter("goal_retry_backoff_sec", 1.5)
        self.declare_parameter("max_goal_retries", 4)
        self.declare_parameter("continue_on_failure", True)
        self.declare_parameter("mapping_done_topic", "")
        self.declare_parameter("start_at_nearest_waypoint", True)
        self.declare_parameter("preferred_start_abs_y_max", 0.25)
        self.declare_parameter("map_topic_transient_local", False)
        self.declare_parameter(
            "route_xs",
            [0.35, 2.15, 2.15, 2.15, 3.50, 4.85, 4.85, 4.85, 6.55, 6.55, 6.55, 4.85, 3.50, 2.15, 2.15, 2.15, 0.35],
        )
        self.declare_parameter(
            "route_ys",
            [0.0, 0.0, 3.35, 0.0, 0.0, 0.0, 3.35, 0.0, 0.0, -3.35, 0.0, 0.0, 0.0, 0.0, -3.35, 0.0, 0.0],
        )
        self.declare_parameter(
            "route_yaws_deg",
            [180.0, 0.0, 90.0, -90.0, 0.0, 0.0, 90.0, -90.0, 0.0, -90.0, 90.0, 180.0, 180.0, 180.0, -90.0, 90.0, 180.0],
        )

        self._map_topic = str(self.get_parameter("map_topic").value)
        self._rtabmap_info_topic = str(self.get_parameter("rtabmap_info_topic").value).strip()
        self._navigate_action_name = str(self.get_parameter("navigate_action_name").value)
        self._goal_frame = str(self.get_parameter("goal_frame").value).strip()
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value).strip()
        self._min_known_map_cells = max(0, int(self.get_parameter("min_known_map_cells").value))
        self._require_robot_inside_map = bool(self.get_parameter("require_robot_inside_map").value)
        self._map_ready_margin = max(0.0, float(self.get_parameter("map_ready_margin_m").value))
        self._wait_after_map_ready_sec = max(0.0, float(self.get_parameter("wait_after_map_ready_sec").value))
        self._wait_after_action_server_ready_sec = max(
            0.0, float(self.get_parameter("wait_after_action_server_ready_sec").value)
        )
        self._wait_after_each_goal_sec = max(0.0, float(self.get_parameter("wait_after_each_goal_sec").value))
        self._waypoint_timeout_sec = max(1.0, float(self.get_parameter("waypoint_timeout_sec").value))
        self._goal_retry_backoff_sec = max(0.1, float(self.get_parameter("goal_retry_backoff_sec").value))
        self._max_goal_retries = max(0, int(self.get_parameter("max_goal_retries").value))
        self._continue_on_failure = bool(self.get_parameter("continue_on_failure").value)
        self._mapping_done_topic = str(self.get_parameter("mapping_done_topic").value).strip()
        self._start_at_nearest_waypoint = bool(self.get_parameter("start_at_nearest_waypoint").value)
        self._preferred_start_abs_y_max = max(0.0, float(self.get_parameter("preferred_start_abs_y_max").value))
        self._map_topic_transient_local = bool(self.get_parameter("map_topic_transient_local").value)

        route_xs = [float(value) for value in self.get_parameter("route_xs").value]
        route_ys = [float(value) for value in self.get_parameter("route_ys").value]
        route_yaws_deg = [float(value) for value in self.get_parameter("route_yaws_deg").value]
        if len(route_xs) == 0 or len(route_xs) != len(route_ys):
            raise ValueError("route_xs / route_ys 参数必须非空且长度一致")
        if route_yaws_deg and len(route_yaws_deg) != len(route_xs):
            raise ValueError("route_yaws_deg 为空或与 route_xs 长度一致")

        self._route = self._build_route(route_xs, route_ys, route_yaws_deg)

        self._latest_map: Optional[OccupancyGrid] = None
        self._known_map_cells = 0
        self._map_ready = False
        self._map_ready_at = 0.0
        self._goal_index = 0
        self._goal_in_flight = False
        self._goal_handle = None
        self._goal_sent_at = 0.0
        self._cancel_requested = False
        self._completed_goals = 0
        self._failed_goals = 0
        self._last_wait_log = 0.0
        self._next_goal_after = 0.0
        self._shutdown_requested = False
        self._started_at = time.monotonic()
        self._goal_attempts = 0
        self._action_server_ready_at = 0.0
        self._mapping_done = not bool(self._mapping_done_topic)
        self._route_aligned = False

        self._loop_closure_pairs: set[tuple[int, int]] = set()
        self._last_ref_id = 0

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL if self._map_topic_transient_local else DurabilityPolicy.VOLATILE
        )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, map_qos)
        if self._rtabmap_info_topic:
            self.create_subscription(RtabmapInfo, self._rtabmap_info_topic, self._on_rtabmap_info, 20)
        if self._mapping_done_topic:
            done_qos = QoSProfile(depth=1)
            done_qos.reliability = ReliabilityPolicy.RELIABLE
            done_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(Bool, self._mapping_done_topic, self._on_mapping_done, done_qos)

        self._navigate_action = ActionClient(self, NavigateToPose, self._navigate_action_name)
        self.create_timer(0.2, self._on_timer)

        self.get_logger().info(
            "SLAM 回环验证器已启动："
            f" map={self._map_topic}, rtabmap_info={self._rtabmap_info_topic or '<disabled>'}, "
            f"action={self._navigate_action_name}, "
            f"route_points={len(self._route)}, mapping_done_topic={self._mapping_done_topic or '<disabled>'}, "
            f"map_qos_durability={'transient_local' if self._map_topic_transient_local else 'volatile'}"
        )

    def _build_route(
        self, route_xs: list[float], route_ys: list[float], route_yaws_deg: list[float]
    ) -> list[tuple[float, float, float]]:
        route: list[tuple[float, float, float]] = []
        for index, (x_value, y_value) in enumerate(zip(route_xs, route_ys)):
            if route_yaws_deg:
                yaw = math.radians(route_yaws_deg[index])
            elif index + 1 < len(route_xs):
                yaw = math.atan2(route_ys[index + 1] - y_value, route_xs[index + 1] - x_value)
            elif index > 0:
                yaw = math.atan2(y_value - route_ys[index - 1], x_value - route_xs[index - 1])
            else:
                yaw = 0.0
            route.append((x_value, y_value, yaw))
        return route

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        self._known_map_cells = sum(1 for value in msg.data if value >= 0)
        self._maybe_mark_map_ready()

    def _on_rtabmap_info(self, msg: RtabmapInfo) -> None:
        self._last_ref_id = int(msg.ref_id)
        loop_closure_id = int(msg.loop_closure_id)
        if loop_closure_id <= 0:
            return
        pair = (int(msg.ref_id), loop_closure_id)
        if pair in self._loop_closure_pairs:
            return
        self._loop_closure_pairs.add(pair)
        self.get_logger().info(
            f"检测到 RTAB-Map 回环闭合：ref_id={msg.ref_id}, loop_closure_id={loop_closure_id}"
        )

    def _on_mapping_done(self, msg: Bool) -> None:
        if self._mapping_done or not bool(msg.data):
            return
        self._mapping_done = True
        self.get_logger().info("自动建图路线已完成，开始等待地图满足导航条件。")

    def _map_bounds(self) -> tuple[float, float, float, float] | None:
        if self._latest_map is None:
            return None
        origin = self._latest_map.info.origin.position
        resolution = float(self._latest_map.info.resolution)
        width = float(self._latest_map.info.width) * resolution
        height = float(self._latest_map.info.height) * resolution
        return origin.x, origin.x + width, origin.y, origin.y + height

    def _point_inside_map(self, x_value: float, y_value: float, margin: float) -> bool:
        bounds = self._map_bounds()
        if bounds is None:
            return False
        xmin, xmax, ymin, ymax = bounds
        return (xmin + margin) <= x_value <= (xmax - margin) and (ymin + margin) <= y_value <= (ymax - margin)

    def _lookup_robot_pose(self) -> PoseStamped | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._goal_frame,
                self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._goal_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _robot_inside_map(self) -> bool:
        if not self._require_robot_inside_map:
            return True
        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return False
        return self._point_inside_map(robot_pose.pose.position.x, robot_pose.pose.position.y, self._map_ready_margin)

    def _maybe_mark_map_ready(self) -> None:
        if self._map_ready or self._latest_map is None:
            return
        if self._known_map_cells < self._min_known_map_cells or not self._robot_inside_map():
            return
        self._map_ready = True
        self._map_ready_at = time.monotonic()
        bounds = self._map_bounds()
        bounds_text = ""
        if bounds is not None:
            xmin, xmax, ymin, ymax = bounds
            bounds_text = f"，bounds=[{xmin:.2f},{xmax:.2f}] x [{ymin:.2f},{ymax:.2f}]"
        self.get_logger().info(
            f"SLAM 回环验证器已确认地图可用，known_cells={self._known_map_cells}{bounds_text}"
        )

    def _build_goal(self, x_value: float, y_value: float, yaw: float) -> PoseStamped:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._goal_frame
        goal.pose.position.x = x_value
        goal.pose.position.y = y_value
        goal.pose.position.z = 0.0
        goal.pose.orientation = quaternion_from_yaw(yaw)
        return goal

    def _maybe_align_route_to_current_pose(self) -> None:
        if self._route_aligned or not self._start_at_nearest_waypoint or not self._route:
            return
        if self._goal_index != 0 or self._completed_goals > 0 or self._failed_goals > 0:
            self._route_aligned = True
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        robot_x = float(robot_pose.pose.position.x)
        robot_y = float(robot_pose.pose.position.y)
        candidate_indices = [
            index for index, (_, y_value, _) in enumerate(self._route) if abs(y_value) <= self._preferred_start_abs_y_max
        ]
        if not candidate_indices:
            candidate_indices = list(range(len(self._route)))

        nearest_index = min(
            candidate_indices,
            key=lambda index: (self._route[index][0] - robot_x) ** 2 + (self._route[index][1] - robot_y) ** 2,
        )
        nearest_x, nearest_y, nearest_yaw = self._route[nearest_index]
        nearest_distance = math.hypot(nearest_x - robot_x, nearest_y - robot_y)

        if nearest_index != 0:
            self._route = self._route[nearest_index:] + self._route[:nearest_index]

        self._route_aligned = True
        self.get_logger().info(
            "验证路线已按当前机器人位置重排起点："
            f" start_index={nearest_index + 1}/{len(self._route)}, "
            f"robot=({robot_x:.2f}, {robot_y:.2f}), "
            f"waypoint=({nearest_x:.2f}, {nearest_y:.2f}, {math.degrees(nearest_yaw):.1f}deg), "
            f"distance={nearest_distance:.2f}m, "
            f"preferred_abs_y_max={self._preferred_start_abs_y_max:.2f}"
        )

    def _send_goal(self) -> None:
        x_value, y_value, yaw = self._route[self._goal_index]
        goal_pose = self._build_goal(x_value, y_value, yaw)
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self._goal_in_flight = True
        self._goal_sent_at = time.monotonic()
        self._cancel_requested = False
        self._goal_attempts += 1

        self.get_logger().info(
            f"发送验证路点 {self._goal_index + 1}/{len(self._route)}: "
            f"x={x_value:.3f}, y={y_value:.3f}, yaw={math.degrees(yaw):.1f}deg, attempt={self._goal_attempts}"
        )

        future = self._navigate_action.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"发送验证路点失败：{exc}")
            self._mark_goal_failed()
            return

        if goal_handle is None or not goal_handle.accepted:
            if self._goal_attempts <= self._max_goal_retries:
                self._goal_in_flight = False
                self._goal_handle = None
                self._next_goal_after = time.monotonic() + self._goal_retry_backoff_sec
                self.get_logger().warning(
                    f"验证路点被 Nav2 拒绝，稍后重试：{self._goal_index + 1}/{len(self._route)}"
                    f" attempt={self._goal_attempts}/{self._max_goal_retries}"
                )
                return
            self.get_logger().warning("验证路点被 Nav2 拒绝。")
            self._mark_goal_failed()
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._goal_in_flight = False
        self._goal_handle = None
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"获取验证路点结果失败：{exc}")
            self._mark_goal_failed()
            return

        if result is None:
            self.get_logger().warning("验证路点返回空结果。")
            self._mark_goal_failed()
            return

        status = int(result.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._completed_goals += 1
            self._goal_attempts = 0
            self.get_logger().info(f"验证路点完成：{self._goal_index + 1}/{len(self._route)}")
            self._goal_index += 1
            self._next_goal_after = time.monotonic() + self._wait_after_each_goal_sec
            return

        status_name = {
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"status={status}")
        self.get_logger().warning(
            f"验证路点失败：{self._goal_index + 1}/{len(self._route)}，结果={status_name}"
        )
        self._mark_goal_failed()

    def _mark_goal_failed(self) -> None:
        self._goal_in_flight = False
        self._goal_handle = None
        self._goal_attempts = 0
        self._failed_goals += 1
        if self._continue_on_failure:
            self._goal_index += 1
            self._next_goal_after = time.monotonic() + self._wait_after_each_goal_sec
            return
        self._finish()

    def _finish(self) -> None:
        if self._shutdown_requested:
            return
        duration = time.monotonic() - self._started_at
        self.get_logger().info(
            "SLAM 回环验证完成："
            f" completed_goals={self._completed_goals}, failed_goals={self._failed_goals}, "
            f"loop_closures={len(self._loop_closure_pairs)}, last_ref_id={self._last_ref_id}, "
            f"elapsed={duration:.1f}s"
        )
        self._shutdown_requested = True

    def _on_timer(self) -> None:
        now = time.monotonic()

        if self._shutdown_requested:
            rclpy.shutdown()
            return

        if not self._map_ready:
            self._maybe_mark_map_ready()
            if now - self._last_wait_log >= 2.0:
                self._last_wait_log = now
                bounds = self._map_bounds()
                wait_parts = [f"known_cells={self._known_map_cells}"]
                if self._mapping_done_topic:
                    wait_parts.append(f"mapping_done={self._mapping_done}")
                if self._require_robot_inside_map:
                    wait_parts.append(f"robot_inside_map={self._robot_inside_map()}")
                if bounds is not None:
                    xmin, xmax, ymin, ymax = bounds
                    wait_parts.append(f"bounds=[{xmin:.2f},{xmax:.2f}] x [{ymin:.2f},{ymax:.2f}]")
                self.get_logger().info("SLAM 回环验证等待地图准备，" + "，".join(wait_parts) + "。")
            return

        if not self._mapping_done:
            if now - self._last_wait_log >= 2.0:
                self._last_wait_log = now
                self.get_logger().info("SLAM 回环验证等待自动建图路线完成。")
            return

        if now < self._map_ready_at + self._wait_after_map_ready_sec:
            return

        if self._goal_in_flight:
            if (
                not self._cancel_requested
                and self._goal_handle is not None
                and (now - self._goal_sent_at) > self._waypoint_timeout_sec
            ):
                self._cancel_requested = True
                self.get_logger().warning(
                    f"验证路点超时，取消当前目标：{self._goal_index + 1}/{len(self._route)}"
                )
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _: None)
            return

        if self._goal_index >= len(self._route):
            self._finish()
            return

        if now < self._next_goal_after:
            return

        if not self._navigate_action.wait_for_server(timeout_sec=0.1):
            self._action_server_ready_at = 0.0
            if now - self._last_wait_log >= 2.0:
                self._last_wait_log = now
                self.get_logger().info("SLAM 回环验证仍在等待 Nav2 action server。")
            return

        if self._action_server_ready_at == 0.0:
            self._action_server_ready_at = now
            return

        if now < self._action_server_ready_at + self._wait_after_action_server_ready_sec:
            return

        self._maybe_align_route_to_current_pose()
        self._send_goal()


def main() -> None:
    rclpy.init()
    node = SlamLoopValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
