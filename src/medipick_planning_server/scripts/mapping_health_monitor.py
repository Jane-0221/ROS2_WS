#!/usr/bin/python3

from __future__ import annotations

import math
import sys
import time
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rtabmap_msgs.msg import Info as RtabmapInfo
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class MappingHealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__("medipick_mapping_health_monitor")

        self.declare_parameter("map_topic", "/rtabmap/grid_prob_map")
        self.declare_parameter("info_topic", "/rtabmap/info")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("print_rate_hz", 1.0)
        self.declare_parameter("ref_stale_timeout_sec", 3.0)
        self.declare_parameter("loop_pending_distance_m", 5.0)
        self.declare_parameter("good_known_cells", 150)
        self.declare_parameter("clear_screen", True)

        self._map_topic = str(self.get_parameter("map_topic").value)
        self._info_topic = str(self.get_parameter("info_topic").value)
        self._goal_frame = str(self.get_parameter("goal_frame").value)
        self._robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self._print_rate_hz = max(0.2, float(self.get_parameter("print_rate_hz").value))
        self._ref_stale_timeout_sec = max(0.5, float(self.get_parameter("ref_stale_timeout_sec").value))
        self._loop_pending_distance_m = max(0.0, float(self.get_parameter("loop_pending_distance_m").value))
        self._good_known_cells = max(1, int(self.get_parameter("good_known_cells").value))
        self._clear_screen = bool(self.get_parameter("clear_screen").value)

        self._latest_map: Optional[OccupancyGrid] = None
        self._known_map_cells = 0
        self._latest_info: Optional[RtabmapInfo] = None
        self._stats: dict[str, float] = {}
        self._accepted_loop_pairs: set[tuple[int, int]] = set()
        self._last_ref_id = 0
        self._last_ref_change_at = time.monotonic()
        self._started_at = time.monotonic()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, 10)
        self.create_subscription(RtabmapInfo, self._info_topic, self._on_info, 20)
        self.create_timer(1.0 / self._print_rate_hz, self._print_summary)

        self.get_logger().info(
            "建图健康监视器已启动："
            f" map={self._map_topic}, info={self._info_topic}, frame={self._goal_frame}->{self._robot_base_frame}"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg
        self._known_map_cells = sum(1 for value in msg.data if value >= 0)

    def _on_info(self, msg: RtabmapInfo) -> None:
        self._latest_info = msg
        self._stats = {key: value for key, value in zip(msg.stats_keys, msg.stats_values)}
        ref_id = int(msg.ref_id)
        if ref_id != self._last_ref_id:
            self._last_ref_id = ref_id
            self._last_ref_change_at = time.monotonic()

        loop_closure_id = int(msg.loop_closure_id)
        if loop_closure_id > 0:
            self._accepted_loop_pairs.add((ref_id, loop_closure_id))

    def _map_bounds(self) -> tuple[float, float, float, float] | None:
        if self._latest_map is None:
            return None
        origin = self._latest_map.info.origin.position
        resolution = float(self._latest_map.info.resolution)
        width = float(self._latest_map.info.width) * resolution
        height = float(self._latest_map.info.height) * resolution
        return origin.x, origin.x + width, origin.y, origin.y + height

    def _lookup_robot_pose(self) -> tuple[float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._goal_frame,
                self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return float(transform.transform.translation.x), float(transform.transform.translation.y)

    def _robot_inside_map(self) -> str:
        bounds = self._map_bounds()
        pose = self._lookup_robot_pose()
        if bounds is None or pose is None:
            return "unknown"
        xmin, xmax, ymin, ymax = bounds
        x_value, y_value = pose
        return "yes" if xmin <= x_value <= xmax and ymin <= y_value <= ymax else "no"

    def _status_lines(self) -> list[str]:
        now = time.monotonic()
        elapsed = now - self._started_at
        info = self._latest_info

        if info is None:
            return [
                "SLAM monitor",
                "",
                "status: waiting_for_/rtabmap/info",
                f"elapsed: {elapsed:.1f}s",
            ]

        ref_id = int(info.ref_id)
        loop_closure_id = int(info.loop_closure_id)
        accepted_loops = len(self._accepted_loop_pairs)
        rejected = int(self._stats.get("Loop/RejectedHypothesis", 0.0))
        highest_hypothesis_id = int(self._stats.get("Loop/Highest_hypothesis_id/", 0.0))
        highest_hypothesis_value = float(self._stats.get("Loop/Highest_hypothesis_value/", 0.0))
        visual_matches = int(self._stats.get("Loop/Visual_matches/", 0.0))
        visual_inliers = int(self._stats.get("Loop/Visual_inliers/", 0.0))
        distance_travelled = float(self._stats.get("Memory/Distance_travelled/m", 0.0))
        stale = (now - self._last_ref_change_at) > self._ref_stale_timeout_sec
        slam_active = "yes" if ref_id > 0 and not stale else "no"
        map_quality = "good" if self._known_map_cells >= self._good_known_cells else "building"

        if accepted_loops > 0:
            loop_status = "closed"
        elif distance_travelled < self._loop_pending_distance_m:
            loop_status = "pending"
        elif highest_hypothesis_id > 0 and rejected > 0:
            loop_status = "trying_but_rejected"
        else:
            loop_status = "not_seen_yet"

        map_to_odom = info.odom_cache.map_to_odom
        map_odom_x = float(map_to_odom.translation.x)
        map_odom_y = float(map_to_odom.translation.y)
        map_odom_yaw = math.degrees(
            quaternion_to_yaw(
                map_to_odom.rotation.x,
                map_to_odom.rotation.y,
                map_to_odom.rotation.z,
                map_to_odom.rotation.w,
            )
        )

        bounds = self._map_bounds()
        bounds_text = "unknown"
        if bounds is not None:
            xmin, xmax, ymin, ymax = bounds
            bounds_text = f"x=[{xmin:.2f},{xmax:.2f}] y=[{ymin:.2f},{ymax:.2f}]"

        hint = "keep_mapping"
        if slam_active == "no":
            hint = "check_tracking"
        elif loop_status == "trying_but_rejected":
            hint = "if_already_back_to_old_area_then_loop_is_weak"
        elif loop_status == "pending" and distance_travelled >= self._loop_pending_distance_m:
            hint = "return_to_start_or_old_area_to_test_loop"
        elif accepted_loops > 0:
            hint = "loop_working"

        return [
            "SLAM monitor",
            "",
            f"status: slam_active={slam_active} map={map_quality} loop={loop_status}",
            f"ref_id: {ref_id}  accepted_loops: {accepted_loops}  current_loop_id: {loop_closure_id}",
            f"highest_hypothesis: id={highest_hypothesis_id} value={highest_hypothesis_value:.3f}  rejected={rejected}",
            f"visual: matches={visual_matches} inliers={visual_inliers}",
            f"map: known_cells={self._known_map_cells} bounds={bounds_text} robot_inside_map={self._robot_inside_map()}",
            f"travelled: {distance_travelled:.2f} m  elapsed: {elapsed:.1f} s",
            f"map->odom: x={map_odom_x:.3f} y={map_odom_y:.3f} yaw={map_odom_yaw:.2f} deg",
            "",
            f"hint: {hint}",
            "rule: many rejected loops do not mean failure by themselves; judge after you have really revisited an old area.",
        ]

    def _print_summary(self) -> None:
        lines = self._status_lines()
        text = "\n".join(lines)
        if self._clear_screen:
            sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(text + "\n")
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = MappingHealthMonitor()
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
