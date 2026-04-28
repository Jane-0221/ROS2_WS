#!/usr/bin/python3

from __future__ import annotations

import math
import time
import heapq
from collections import deque
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from std_msgs.msg import Bool


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("medipick_frontier_explorer")

        self.declare_parameter("map_topic", "/rtabmap/grid_prob_map")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("done_topic", "/medipick/slam_auto_explore_done")
        self.declare_parameter("plan_period_sec", 1.0)
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("map_ready_min_known_cells", 150)
        self.declare_parameter("map_ready_margin_m", 0.05)
        self.declare_parameter("require_robot_inside_map", True)
        self.declare_parameter("bootstrap_head_scan_before_map_ready", True)
        self.declare_parameter("bootstrap_head_scan_period_sec", 6.0)
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("goal_no_progress_timeout_sec", 35.0)
        self.declare_parameter("goal_progress_delta_m", 0.08)
        self.declare_parameter("goal_arrival_grace_distance", 0.20)
        self.declare_parameter("wait_after_goal_sec", 0.8)
        self.declare_parameter("failure_backoff_sec", 2.0)
        self.declare_parameter("frontier_free_threshold", 50)
        self.declare_parameter("frontier_min_cluster_size", 10)
        self.declare_parameter("frontier_min_goal_distance", 0.55)
        self.declare_parameter("frontier_max_goal_distance", 0.0)
        self.declare_parameter("frontier_goal_standoff_distance", 0.0)
        self.declare_parameter("frontier_use_geodesic_distance", True)
        self.declare_parameter("frontier_max_path_distance", 4.0)
        self.declare_parameter("frontier_blacklist_radius", 0.60)
        self.declare_parameter("frontier_blacklist_ttl_sec", 120.0)
        self.declare_parameter("frontier_distance_weight", 1.0)
        self.declare_parameter("frontier_gain_weight", 0.40)
        self.declare_parameter("no_frontier_confirmations", 6)
        self.declare_parameter("status_log_period_sec", 5.0)
        self.declare_parameter("enable_head_scan", True)
        self.declare_parameter("head_trajectory_topic", "/head_controller/joint_trajectory")
        self.declare_parameter("head_scan_h1_sequence_deg", [-30.0, 0.0, 30.0, 0.0])
        self.declare_parameter("head_scan_h2_deg", -30.0)
        self.declare_parameter("head_scan_move_sec", 0.20)
        self.declare_parameter("head_scan_hold_sec", 0.10)

        self._map_topic = str(self.get_parameter("map_topic").value).strip()
        self._navigate_action_name = str(self.get_parameter("navigate_action_name").value).strip()
        self._goal_frame = str(self.get_parameter("goal_frame").value).strip()
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value).strip()
        self._done_topic = str(self.get_parameter("done_topic").value).strip()
        self._plan_period_sec = max(0.2, float(self.get_parameter("plan_period_sec").value))
        self._start_delay_sec = max(0.0, float(self.get_parameter("start_delay_sec").value))
        self._map_ready_min_known_cells = max(0, int(self.get_parameter("map_ready_min_known_cells").value))
        self._map_ready_margin = max(0.0, float(self.get_parameter("map_ready_margin_m").value))
        self._require_robot_inside_map = bool(self.get_parameter("require_robot_inside_map").value)
        self._bootstrap_head_scan_before_map_ready = bool(
            self.get_parameter("bootstrap_head_scan_before_map_ready").value
        )
        self._bootstrap_head_scan_period_sec = max(
            1.0, float(self.get_parameter("bootstrap_head_scan_period_sec").value)
        )
        self._goal_timeout_sec = max(5.0, float(self.get_parameter("goal_timeout_sec").value))
        self._goal_no_progress_timeout_sec = max(
            5.0, float(self.get_parameter("goal_no_progress_timeout_sec").value)
        )
        self._goal_progress_delta_m = max(0.02, float(self.get_parameter("goal_progress_delta_m").value))
        self._goal_arrival_grace_distance = max(
            0.02, float(self.get_parameter("goal_arrival_grace_distance").value)
        )
        self._wait_after_goal_sec = max(0.0, float(self.get_parameter("wait_after_goal_sec").value))
        self._failure_backoff_sec = max(0.1, float(self.get_parameter("failure_backoff_sec").value))
        self._frontier_free_threshold = max(0, int(self.get_parameter("frontier_free_threshold").value))
        self._frontier_min_cluster_size = max(3, int(self.get_parameter("frontier_min_cluster_size").value))
        self._frontier_min_goal_distance = max(0.0, float(self.get_parameter("frontier_min_goal_distance").value))
        self._frontier_max_goal_distance = max(0.0, float(self.get_parameter("frontier_max_goal_distance").value))
        self._frontier_goal_standoff_distance = max(
            0.0, float(self.get_parameter("frontier_goal_standoff_distance").value)
        )
        self._frontier_use_geodesic_distance = bool(
            self.get_parameter("frontier_use_geodesic_distance").value
        )
        self._frontier_max_path_distance = max(0.0, float(self.get_parameter("frontier_max_path_distance").value))
        self._frontier_blacklist_radius = max(0.05, float(self.get_parameter("frontier_blacklist_radius").value))
        self._frontier_blacklist_ttl_sec = max(1.0, float(self.get_parameter("frontier_blacklist_ttl_sec").value))
        self._frontier_distance_weight = max(0.1, float(self.get_parameter("frontier_distance_weight").value))
        self._frontier_gain_weight = max(0.0, float(self.get_parameter("frontier_gain_weight").value))
        self._no_frontier_confirmations = max(1, int(self.get_parameter("no_frontier_confirmations").value))
        self._status_log_period_sec = max(1.0, float(self.get_parameter("status_log_period_sec").value))
        self._enable_head_scan = bool(self.get_parameter("enable_head_scan").value)
        self._head_trajectory_topic = str(self.get_parameter("head_trajectory_topic").value).strip()
        self._head_scan_h1_sequence = tuple(
            math.radians(float(value)) for value in self.get_parameter("head_scan_h1_sequence_deg").value
        )
        self._head_scan_h2 = math.radians(float(self.get_parameter("head_scan_h2_deg").value))
        self._head_scan_move_sec = max(0.10, float(self.get_parameter("head_scan_move_sec").value))
        self._head_scan_hold_sec = max(0.05, float(self.get_parameter("head_scan_hold_sec").value))

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        done_qos = QoSProfile(depth=1)
        done_qos.reliability = ReliabilityPolicy.RELIABLE
        done_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._latest_map: Optional[OccupancyGrid] = None
        self._known_map_cells = 0
        self._map_ready = False
        self._map_ready_at = 0.0
        self._started_at = time.monotonic()
        self._next_goal_after = 0.0
        self._goal_in_flight = False
        self._goal_sent_at = 0.0
        self._goal_last_progress_at = 0.0
        self._goal_best_remaining_distance = math.inf
        self._goal_handle = None
        self._current_goal_xy: Optional[tuple[float, float]] = None
        self._current_goal_size = 0
        self._completed_goals = 0
        self._failed_goals = 0
        self._exploration_done = False
        self._no_frontier_streak = 0
        self._last_status_log = 0.0
        self._last_frontier_count = 0
        self._blacklist: list[tuple[float, float, float]] = []
        self._head_scan_active = False
        self._head_scan_pending = False
        self._head_scan_step_index = 0
        self._head_scan_step_started_at = 0.0
        self._last_bootstrap_head_scan_at = 0.0

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, map_qos)
        self._navigate_action = ActionClient(self, NavigateToPose, self._navigate_action_name)
        self._head_pub = self.create_publisher(JointTrajectory, self._head_trajectory_topic, 10)
        self._done_pub = self.create_publisher(Bool, self._done_topic, done_qos)
        self._publish_done(False)

        self.create_timer(self._plan_period_sec, self._on_timer)

        self.get_logger().info(
            "前沿探索器已启动："
            f" map={self._map_topic}, action={self._navigate_action_name}, done_topic={self._done_topic},"
            f" min_cluster={self._frontier_min_cluster_size}, min_goal_distance={self._frontier_min_goal_distance:.2f},"
            f" geodesic={self._frontier_use_geodesic_distance},"
            f" goal_standoff={self._frontier_goal_standoff_distance:.2f},"
            f" head_scan={self._enable_head_scan}"
        )

    def _publish_done(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self._done_pub.publish(msg)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        self._known_map_cells = sum(1 for value in msg.data if value >= 0)
        self._maybe_mark_map_ready()

    def _lookup_robot_pose(self) -> Optional[tuple[float, float, float]]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._goal_frame,
                self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

        rotation = transform.transform.rotation
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _map_bounds(self) -> Optional[tuple[float, float, float, float]]:
        if self._latest_map is None:
            return None
        info = self._latest_map.info
        xmin = float(info.origin.position.x)
        ymin = float(info.origin.position.y)
        xmax = xmin + float(info.width) * float(info.resolution)
        ymax = ymin + float(info.height) * float(info.resolution)
        return xmin, xmax, ymin, ymax

    def _point_inside_map(self, x_value: float, y_value: float, margin: float) -> bool:
        bounds = self._map_bounds()
        if bounds is None:
            return False
        xmin, xmax, ymin, ymax = bounds
        return (xmin + margin) <= x_value <= (xmax - margin) and (ymin + margin) <= y_value <= (ymax - margin)

    def _maybe_mark_map_ready(self) -> None:
        if self._map_ready or self._latest_map is None:
            return
        if self._known_map_cells < self._map_ready_min_known_cells:
            return
        if self._require_robot_inside_map:
            robot_pose = self._lookup_robot_pose()
            if robot_pose is None:
                return
            if not self._point_inside_map(robot_pose[0], robot_pose[1], self._map_ready_margin):
                return
        self._map_ready = True
        self._map_ready_at = time.monotonic()
        self.get_logger().info(
            f"前沿探索器确认地图可用于探索，known_cells={self._known_map_cells}"
        )

    def _publish_head_target(self, h1_rad: float, h2_rad: float) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = ["h1_joint", "h2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [h1_rad, h2_rad]
        total_ns = int(self._head_scan_move_sec * 1e9)
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000
        trajectory.points = [point]
        self._head_pub.publish(trajectory)

    def _start_head_scan(self, now: float) -> None:
        if not self._enable_head_scan or not self._head_scan_h1_sequence:
            self._head_scan_pending = False
            return
        self._head_scan_active = True
        self._head_scan_pending = False
        self._head_scan_step_index = 0
        self._head_scan_step_started_at = now
        self._publish_head_target(self._head_scan_h1_sequence[0], self._head_scan_h2)
        self.get_logger().info("前沿探索器开始头部补视角扫描。")

    def _advance_head_scan(self, now: float) -> bool:
        if not self._head_scan_active:
            return False
        if now - self._head_scan_step_started_at < self._head_scan_move_sec + self._head_scan_hold_sec:
            return True
        self._head_scan_step_index += 1
        if self._head_scan_step_index >= len(self._head_scan_h1_sequence):
            self._head_scan_active = False
            self.get_logger().info("前沿探索器完成头部补视角扫描。")
            return False
        self._head_scan_step_started_at = now
        self._publish_head_target(self._head_scan_h1_sequence[self._head_scan_step_index], self._head_scan_h2)
        return True

    def _purge_blacklist(self, now: float) -> None:
        if not self._blacklist:
            return
        self._blacklist = [entry for entry in self._blacklist if now - entry[2] <= self._frontier_blacklist_ttl_sec]

    def _is_blacklisted(self, x_value: float, y_value: float) -> bool:
        radius_sq = self._frontier_blacklist_radius * self._frontier_blacklist_radius
        for bx, by, _ in self._blacklist:
            dx = x_value - bx
            dy = y_value - by
            if dx * dx + dy * dy <= radius_sq:
                return True
        return False

    def _add_blacklist(self, x_value: float, y_value: float, now: float) -> None:
        self._blacklist.append((x_value, y_value, now))

    def _world_to_cell(self, x_value: float, y_value: float, origin_x: float, origin_y: float, resolution: float) -> tuple[int, int]:
        cell_x = int(math.floor((x_value - origin_x) / resolution))
        cell_y = int(math.floor((y_value - origin_y) / resolution))
        return cell_x, cell_y

    def _compute_geodesic_distances(
        self,
        robot_x: float,
        robot_y: float,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: list[int],
    ) -> Optional[list[float]]:
        start_x, start_y = self._world_to_cell(robot_x, robot_y, origin_x, origin_y, resolution)
        if start_x < 0 or start_y < 0 or start_x >= width or start_y >= height:
            return None

        start_index = start_y * width + start_x
        start_value = data[start_index]
        if start_value < 0 or start_value > self._frontier_free_threshold:
            return None

        distances = [math.inf] * (width * height)
        distances[start_index] = 0.0
        heap: list[tuple[float, int]] = [(0.0, start_index)]
        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )

        while heap:
            current_cost, current_index = heapq.heappop(heap)
            if current_cost > distances[current_index]:
                continue
            cell_x = current_index % width
            cell_y = current_index // width
            for offset_x, offset_y, step_cost in neighbors:
                next_x = cell_x + offset_x
                next_y = cell_y + offset_y
                if next_x < 0 or next_y < 0 or next_x >= width or next_y >= height:
                    continue
                next_index = next_y * width + next_x
                next_value = data[next_index]
                if next_value < 0 or next_value > self._frontier_free_threshold:
                    continue
                next_cost = current_cost + step_cost * resolution
                if next_cost >= distances[next_index]:
                    continue
                distances[next_index] = next_cost
                heapq.heappush(heap, (next_cost, next_index))

        return distances

    def _project_goal_standoff(
        self,
        frontier_x: float,
        frontier_y: float,
        robot_x: float,
        robot_y: float,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data: list[int],
    ) -> tuple[float, float]:
        standoff_distance = self._frontier_goal_standoff_distance
        if standoff_distance <= 0.0:
            return frontier_x, frontier_y

        delta_x = robot_x - frontier_x
        delta_y = robot_y - frontier_y
        distance = math.hypot(delta_x, delta_y)
        if distance < 1e-6:
            return frontier_x, frontier_y

        direction_x = delta_x / distance
        direction_y = delta_y / distance
        for scale in (1.0, 0.75, 0.50, 0.25, 0.0):
            candidate_x = frontier_x + direction_x * standoff_distance * scale
            candidate_y = frontier_y + direction_y * standoff_distance * scale
            cell_x, cell_y = self._world_to_cell(candidate_x, candidate_y, origin_x, origin_y, resolution)
            if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
                continue
            cell_value = data[cell_y * width + cell_x]
            if 0 <= cell_value <= self._frontier_free_threshold:
                return candidate_x, candidate_y

        return frontier_x, frontier_y

    def _extract_frontier_goal(self, robot_x: float, robot_y: float) -> Optional[tuple[float, float, int, float, float]]:
        if self._latest_map is None:
            return None

        info = self._latest_map.info
        width = int(info.width)
        height = int(info.height)
        resolution = float(info.resolution)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)
        data = list(self._latest_map.data)
        if width < 3 or height < 3:
            self._last_frontier_count = 0
            return None

        frontier_mask = [False] * (width * height)
        frontier_cells: list[int] = []
        geodesic_distances = None
        if self._frontier_use_geodesic_distance:
            geodesic_distances = self._compute_geodesic_distances(
                robot_x,
                robot_y,
                width,
                height,
                resolution,
                origin_x,
                origin_y,
                data,
            )

        for cell_y in range(1, height - 1):
            base = cell_y * width
            for cell_x in range(1, width - 1):
                index = base + cell_x
                value = data[index]
                if value < 0 or value > self._frontier_free_threshold:
                    continue
                has_unknown_neighbor = False
                for offset_y in (-1, 0, 1):
                    for offset_x in (-1, 0, 1):
                        if offset_x == 0 and offset_y == 0:
                            continue
                        neighbor_index = index + offset_x + offset_y * width
                        if data[neighbor_index] < 0:
                            has_unknown_neighbor = True
                            break
                    if has_unknown_neighbor:
                        break
                if not has_unknown_neighbor:
                    continue
                frontier_mask[index] = True
                frontier_cells.append(index)

        self._last_frontier_count = len(frontier_cells)
        if not frontier_cells:
            return None

        visited = [False] * (width * height)
        candidates: list[tuple[float, float, int, float, float]] = []

        for start_index in frontier_cells:
            if visited[start_index]:
                continue
            queue = deque([start_index])
            visited[start_index] = True
            cluster: list[int] = []
            sum_x = 0.0
            sum_y = 0.0

            while queue:
                current = queue.popleft()
                cluster.append(current)
                cx = current % width
                cy = current // width
                sum_x += cx
                sum_y += cy
                for offset_y in (-1, 0, 1):
                    for offset_x in (-1, 0, 1):
                        if offset_x == 0 and offset_y == 0:
                            continue
                        nx = cx + offset_x
                        ny = cy + offset_y
                        if nx <= 0 or ny <= 0 or nx >= width - 1 or ny >= height - 1:
                            continue
                        neighbor = ny * width + nx
                        if visited[neighbor] or not frontier_mask[neighbor]:
                            continue
                        visited[neighbor] = True
                        queue.append(neighbor)

            cluster_size = len(cluster)
            if cluster_size < self._frontier_min_cluster_size:
                continue

            centroid_x = sum_x / cluster_size
            centroid_y = sum_y / cluster_size
            representative = min(
                cluster,
                key=lambda index: ((index % width) - centroid_x) ** 2 + ((index // width) - centroid_y) ** 2,
            )
            goal_cell_x = representative % width
            goal_cell_y = representative // width
            frontier_x = origin_x + (goal_cell_x + 0.5) * resolution
            frontier_y = origin_y + (goal_cell_y + 0.5) * resolution
            goal_x, goal_y = self._project_goal_standoff(
                frontier_x,
                frontier_y,
                robot_x,
                robot_y,
                width,
                height,
                resolution,
                origin_x,
                origin_y,
                data,
            )

            if self._is_blacklisted(goal_x, goal_y):
                continue

            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
            score_distance = distance
            if geodesic_distances is not None:
                path_distance = geodesic_distances[representative]
                if not math.isfinite(path_distance):
                    continue
                score_distance = path_distance
                if self._frontier_max_path_distance > 0.0 and path_distance > self._frontier_max_path_distance:
                    continue
            if distance < self._frontier_min_goal_distance:
                continue
            if self._frontier_max_goal_distance > 0.0 and distance > self._frontier_max_goal_distance:
                continue

            gain_m = cluster_size * resolution
            score = self._frontier_distance_weight * score_distance - self._frontier_gain_weight * gain_m
            candidates.append((goal_x, goal_y, cluster_size, score, score_distance))

        if not candidates:
            return None

        return min(candidates, key=lambda candidate: candidate[3])

    def _build_goal(self, x_value: float, y_value: float, yaw: float) -> PoseStamped:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._goal_frame
        goal.pose.position.x = x_value
        goal.pose.position.y = y_value
        goal.pose.position.z = 0.0
        goal.pose.orientation = quaternion_from_yaw(yaw)
        return goal

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._goal_in_flight = False
            self._goal_handle = None
            self._current_goal_xy = None
            self.get_logger().warn(f"前沿探索目标发送失败：{exc}")
            self._next_goal_after = time.monotonic() + self._failure_backoff_sec
            return

        if not goal_handle.accepted:
            self._goal_in_flight = False
            self._goal_handle = None
            if self._current_goal_xy is not None:
                self.get_logger().warn(
                    f"前沿探索目标被拒绝：goal=({self._current_goal_xy[0]:.2f}, {self._current_goal_xy[1]:.2f})"
                )
            self._current_goal_xy = None
            self._next_goal_after = time.monotonic() + self._failure_backoff_sec
            return

        self._goal_handle = goal_handle
        self._goal_sent_at = time.monotonic()
        self._goal_last_progress_at = self._goal_sent_at
        self._goal_best_remaining_distance = math.inf
        goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        now = time.monotonic()
        goal_xy = self._current_goal_xy
        self._goal_in_flight = False
        self._goal_handle = None
        self._current_goal_xy = None
        self._goal_best_remaining_distance = math.inf
        self._goal_last_progress_at = 0.0

        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._failed_goals += 1
            if goal_xy is not None:
                self._add_blacklist(goal_xy[0], goal_xy[1], now)
            self._next_goal_after = now + self._failure_backoff_sec
            self.get_logger().warn(f"前沿探索目标结果读取失败：{exc}")
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._completed_goals += 1
            self._next_goal_after = now + self._wait_after_goal_sec
            self._head_scan_pending = self._enable_head_scan
            if goal_xy is not None:
                self.get_logger().info(
                    f"前沿探索到达目标：goal=({goal_xy[0]:.2f}, {goal_xy[1]:.2f}), completed={self._completed_goals}"
                )
            return

        self._failed_goals += 1
        if goal_xy is not None:
            self._add_blacklist(goal_xy[0], goal_xy[1], now)
            self.get_logger().warn(
                f"前沿探索目标失败(status={result.status})，已拉黑附近区域：goal=({goal_xy[0]:.2f}, {goal_xy[1]:.2f}), failed={self._failed_goals}"
            )
        else:
            self.get_logger().warn(f"前沿探索目标失败(status={result.status})")
        self._next_goal_after = now + self._failure_backoff_sec

    def _cancel_timed_out_goal(self, now: float) -> None:
        if not self._goal_in_flight or self._goal_handle is None:
            return

        robot_pose = self._lookup_robot_pose()
        remaining_distance = math.inf
        if robot_pose is not None and self._current_goal_xy is not None:
            remaining_distance = math.hypot(
                self._current_goal_xy[0] - robot_pose[0],
                self._current_goal_xy[1] - robot_pose[1],
            )
            if math.isinf(self._goal_best_remaining_distance):
                self._goal_best_remaining_distance = remaining_distance
                self._goal_last_progress_at = now
            elif remaining_distance <= self._goal_best_remaining_distance - self._goal_progress_delta_m:
                self._goal_best_remaining_distance = remaining_distance
                self._goal_last_progress_at = now

        hard_timeout_hit = now >= self._goal_sent_at + self._goal_timeout_sec
        no_progress_timeout_hit = now >= self._goal_last_progress_at + self._goal_no_progress_timeout_sec
        if (
            not hard_timeout_hit
            and math.isfinite(remaining_distance)
            and remaining_distance <= self._goal_arrival_grace_distance
        ):
            return
        if not hard_timeout_hit and not no_progress_timeout_hit:
            return

        goal_xy = self._current_goal_xy
        timeout_reason = "hard_timeout" if hard_timeout_hit else "no_progress_timeout"
        if math.isfinite(remaining_distance):
            self.get_logger().warn(
                "前沿探索目标超时，尝试取消："
                f" reason={timeout_reason}, remaining_distance={remaining_distance:.2f},"
                f" best_remaining_distance={self._goal_best_remaining_distance:.2f}"
            )
        else:
            self.get_logger().warn(f"前沿探索目标超时，尝试取消：reason={timeout_reason}")
        self._goal_handle.cancel_goal_async()
        self._goal_in_flight = False
        self._goal_handle = None
        if goal_xy is not None:
            self._add_blacklist(goal_xy[0], goal_xy[1], now)
        self._current_goal_xy = None
        self._goal_best_remaining_distance = math.inf
        self._goal_last_progress_at = 0.0
        self._next_goal_after = now + self._failure_backoff_sec

    def _send_frontier_goal(self, robot_x: float, robot_y: float, goal_x: float, goal_y: float, cluster_size: int) -> None:
        yaw = math.atan2(goal_y - robot_y, goal_x - robot_x)
        goal_pose = self._build_goal(goal_x, goal_y, yaw)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self._goal_in_flight = True
        self._current_goal_xy = (goal_x, goal_y)
        self._current_goal_size = cluster_size
        self._navigate_action.send_goal_async(goal_msg).add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f"前沿探索发送目标：goal=({goal_x:.2f}, {goal_y:.2f}), cluster_size={cluster_size}, completed={self._completed_goals}, failed={self._failed_goals}"
        )

    def _mark_done(self) -> None:
        if self._exploration_done:
            return
        self._exploration_done = True
        self._publish_done(True)
        self.get_logger().info(
            f"前沿探索结束：completed={self._completed_goals}, failed={self._failed_goals}, known_cells={self._known_map_cells}"
        )

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._purge_blacklist(now)

        if now < self._started_at + self._start_delay_sec:
            return

        if self._exploration_done:
            return

        if self._head_scan_active:
            self._advance_head_scan(now)
            return

        if self._head_scan_pending:
            self._start_head_scan(now)
            if self._head_scan_active:
                return

        if self._goal_in_flight:
            self._cancel_timed_out_goal(now)
            return

        if not self._map_ready:
            if (
                self._bootstrap_head_scan_before_map_ready
                and self._enable_head_scan
                and now >= self._last_bootstrap_head_scan_at + self._bootstrap_head_scan_period_sec
            ):
                self._last_bootstrap_head_scan_at = now
                self._head_scan_pending = True
                self.get_logger().info("前沿探索在地图就绪前执行补视角扫描。")
                self._start_head_scan(now)
                if self._head_scan_active:
                    return
            if now - self._last_status_log >= self._status_log_period_sec:
                self._last_status_log = now
                self.get_logger().info(
                    f"前沿探索等待地图可用：known_cells={self._known_map_cells}, min_required={self._map_ready_min_known_cells}"
                )
            return

        if not self._navigate_action.wait_for_server(timeout_sec=0.05):
            if now - self._last_status_log >= self._status_log_period_sec:
                self._last_status_log = now
                self.get_logger().info("前沿探索等待 NavigateToPose action server 就绪。")
            return

        if now < self._next_goal_after:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            if now - self._last_status_log >= self._status_log_period_sec:
                self._last_status_log = now
                self.get_logger().info("前沿探索等待 map->base_link TF。")
            return

        frontier_goal = self._extract_frontier_goal(robot_pose[0], robot_pose[1])
        if frontier_goal is None:
            self._no_frontier_streak += 1
            if now - self._last_status_log >= self._status_log_period_sec:
                self._last_status_log = now
                self.get_logger().info(
                    f"前沿探索未找到可用前沿：frontier_cells={self._last_frontier_count}, no_frontier_streak={self._no_frontier_streak}/{self._no_frontier_confirmations}"
                )
            if self._no_frontier_streak >= self._no_frontier_confirmations:
                self._mark_done()
            return

        self._no_frontier_streak = 0
        goal_x, goal_y, cluster_size, score, score_distance = frontier_goal
        if now - self._last_status_log >= self._status_log_period_sec:
            self._last_status_log = now
            distance_label = "path_distance" if self._frontier_use_geodesic_distance else "distance"
            self.get_logger().info(
                f"前沿探索候选：goal=({goal_x:.2f}, {goal_y:.2f}), cluster_size={cluster_size}, score={score:.3f}, "
                f"{distance_label}={score_distance:.2f}, frontier_cells={self._last_frontier_count}, known_cells={self._known_map_cells}"
            )
        self._send_frontier_goal(robot_pose[0], robot_pose[1], goal_x, goal_y, cluster_size)


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok() and "Unable to convert call argument" not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
