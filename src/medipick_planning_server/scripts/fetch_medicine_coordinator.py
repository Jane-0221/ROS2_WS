#!/usr/bin/env python3

from __future__ import annotations

import math
from threading import Event, Lock, Thread
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from medipick_planning_interfaces.srv import (
    FetchMedicine,
    GetMedicineLocation,
    ResolveLocalPick,
)


def normalize_quaternion(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_multiply(
    lhs: tuple[float, float, float, float],
    rhs: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quat_conjugate(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def rotate_vector(
    vector: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    pure = (vector[0], vector[1], vector[2], 0.0)
    rotated = quat_multiply(quat_multiply(quaternion, pure), quat_conjugate(quaternion))
    return (rotated[0], rotated[1], rotated[2])


class FetchMedicineCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("medipick_fetch_medicine_coordinator")

        self.declare_parameter("fetch_service_name", "/medipick/task/fetch_medicine")
        self.declare_parameter("get_medicine_location_service", "/medipick/semantic/get_medicine_location")
        self.declare_parameter("resolve_local_pick_service", "/medipick/vision/resolve_local_pick")
        self.declare_parameter("start_task_service", "/medipick/task/start")
        self.declare_parameter("navigate_action_name", "/navigate_to_pose")
        self.declare_parameter("dock_goal_topic", "/medipick/task/base_goal")
        self.declare_parameter("task_target_pose_topic", "/medipick/task/target_pose")
        self.declare_parameter("output_target_frame", "world")
        self.declare_parameter("odometry_topic", "/odometry/filtered")
        self.declare_parameter("max_record_age_sec", 86400.0)
        self.declare_parameter("min_confidence", 0.6)
        self.declare_parameter("dock_standoff", 0.40)
        self.declare_parameter("require_local_pick_confirmation", True)
        self.declare_parameter("auto_start_pick_task", True)
        self.declare_parameter("service_wait_timeout_sec", 3.0)
        self.declare_parameter("navigate_timeout_sec", 180.0)
        self.declare_parameter("max_position_stddev", 0.10)
        self.declare_parameter("max_yaw_stddev_deg", 5.0)
        self.declare_parameter("transform_timeout_sec", 0.2)

        self._dock_goal_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("dock_goal_topic").value),
            10,
        )
        self._target_pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("task_target_pose_topic").value),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._handle_odometry,
            20,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_odometry: Optional[Odometry] = None
        self._latest_odometry_lock = Lock()
        self._transform_timeout_sec = float(self.get_parameter("transform_timeout_sec").value)

        self._client_node = rclpy.create_node(
            "medipick_fetch_medicine_coordinator_client",
            context=self.context,
        )
        self._client_executor = MultiThreadedExecutor(num_threads=2, context=self.context)
        self._client_executor.add_node(self._client_node)
        self._client_executor_thread = Thread(target=self._client_executor.spin, daemon=True)
        self._client_executor_thread.start()

        self._get_location_client = self._client_node.create_client(
            GetMedicineLocation,
            str(self.get_parameter("get_medicine_location_service").value),
        )
        self._resolve_local_pick_client = self._client_node.create_client(
            ResolveLocalPick,
            str(self.get_parameter("resolve_local_pick_service").value),
        )
        self._start_task_client = self._client_node.create_client(
            Trigger,
            str(self.get_parameter("start_task_service").value),
        )
        self._navigate_client = ActionClient(
            self._client_node,
            NavigateToPose,
            str(self.get_parameter("navigate_action_name").value),
        )

        self.create_service(
            FetchMedicine,
            str(self.get_parameter("fetch_service_name").value),
            self._handle_fetch_medicine,
        )

        self.get_logger().info("Fetch medicine coordinator ready.")

    def _handle_odometry(self, message: Odometry) -> None:
        with self._latest_odometry_lock:
            self._latest_odometry = message

    def _handle_fetch_medicine(
        self,
        request: FetchMedicine.Request,
        response: FetchMedicine.Response,
    ) -> FetchMedicine.Response:
        medicine_id = request.medicine_id.strip()
        if not medicine_id:
            response.success = False
            response.state = "rejected"
            response.message = "medicine_id must not be empty."
            return response

        location = self._call_get_location(medicine_id)
        if location is None or not location.success:
            response.success = False
            response.state = "lookup_failed"
            response.message = getattr(location, "message", f"Medicine '{medicine_id}' not found.")
            return response

        validation_error = self._validate_location(location)
        if validation_error is not None:
            response.success = False
            response.state = "validation_failed"
            response.message = validation_error
            return response

        localization_error = self._validate_localization_quality()
        if localization_error is not None:
            response.success = False
            response.state = "localization_unhealthy"
            response.message = localization_error
            return response

        dock_pose = self._compute_dock_pose(location.shelf_face_pose_map)
        response.dock_pose = dock_pose
        self._dock_goal_pub.publish(dock_pose)

        nav_error = self._navigate_to_pose(dock_pose)
        if nav_error is not None:
            response.success = False
            response.state = "navigation_failed"
            response.message = nav_error
            return response

        local_pick_pose = PoseStamped()
        if bool(self.get_parameter("require_local_pick_confirmation").value):
            local_pick_result = self._call_resolve_local_pick(medicine_id, location.item_pose_map)
            if local_pick_result is None or not local_pick_result.success:
                response.success = False
                response.state = "local_refine_failed"
                response.message = getattr(
                    local_pick_result,
                    "message",
                    "Local pick confirmation service did not return a usable pose.",
                )
                return response

            transformed_target = self._transform_pose(
                local_pick_result.local_pick_pose,
                str(self.get_parameter("output_target_frame").value),
            )
            if transformed_target is None:
                response.success = False
                response.state = "transform_failed"
                response.message = "Failed to transform local pick pose into the planning frame."
                return response
            local_pick_pose = transformed_target
        else:
            transformed_target = self._transform_pose(
                location.item_pose_map,
                str(self.get_parameter("output_target_frame").value),
            )
            if transformed_target is None:
                response.success = False
                response.state = "transform_failed"
                response.message = "Failed to transform medicine pose into the planning frame."
                return response
            local_pick_pose = transformed_target

        self._target_pose_pub.publish(local_pick_pose)
        response.target_pose_for_pick = local_pick_pose

        if bool(self.get_parameter("auto_start_pick_task").value):
            start_response = self._call_trigger(self._start_task_client)
            if start_response is None or not start_response.success:
                response.success = False
                response.state = "pick_start_failed"
                response.message = getattr(start_response, "message", "Pick task start service failed.")
                return response

        response.success = True
        response.state = "pick_started"
        response.message = f"Navigation and local confirmation succeeded for '{medicine_id}'."
        return response

    def _validate_location(self, location: GetMedicineLocation.Response) -> Optional[str]:
        min_confidence = float(self.get_parameter("min_confidence").value)
        if float(location.confidence) < min_confidence:
            return (
                f"Medicine confidence {float(location.confidence):.3f} is below the configured minimum "
                f"{min_confidence:.3f}."
            )

        updated_at = location.updated_at
        if updated_at.sec == 0 and updated_at.nanosec == 0:
            return "Medicine location has no timestamp."

        record_age = self.get_clock().now() - Time.from_msg(updated_at)
        max_record_age_sec = float(self.get_parameter("max_record_age_sec").value)
        if record_age.nanoseconds > max_record_age_sec * 1e9:
            return (
                f"Medicine location is stale ({record_age.nanoseconds / 1e9:.1f}s > {max_record_age_sec:.1f}s)."
            )
        return None

    def _validate_localization_quality(self) -> Optional[str]:
        with self._latest_odometry_lock:
            odometry = self._latest_odometry

        if odometry is None:
            return "No filtered odometry has been received yet."

        covariance = list(odometry.pose.covariance)
        position_stddev = math.sqrt(max(covariance[0], covariance[7], 0.0))
        yaw_stddev = math.sqrt(max(covariance[35], 0.0))
        max_position_stddev = float(self.get_parameter("max_position_stddev").value)
        max_yaw_stddev = math.radians(float(self.get_parameter("max_yaw_stddev_deg").value))

        if position_stddev > max_position_stddev:
            return (
                f"Localization position stddev {position_stddev:.3f}m exceeds {max_position_stddev:.3f}m."
            )
        if yaw_stddev > max_yaw_stddev:
            return f"Localization yaw stddev {math.degrees(yaw_stddev):.2f}deg exceeds the limit."
        return None

    def _compute_dock_pose(self, shelf_face_pose_map: PoseStamped) -> PoseStamped:
        dock_standoff = float(self.get_parameter("dock_standoff").value)
        orientation = normalize_quaternion(
            (
                shelf_face_pose_map.pose.orientation.x,
                shelf_face_pose_map.pose.orientation.y,
                shelf_face_pose_map.pose.orientation.z,
                shelf_face_pose_map.pose.orientation.w,
            )
        )
        normal = rotate_vector((dock_standoff, 0.0, 0.0), orientation)

        dock_pose = PoseStamped()
        dock_pose.header = shelf_face_pose_map.header
        dock_pose.pose.position.x = shelf_face_pose_map.pose.position.x + normal[0]
        dock_pose.pose.position.y = shelf_face_pose_map.pose.position.y + normal[1]
        dock_pose.pose.position.z = shelf_face_pose_map.pose.position.z
        dock_pose.pose.orientation.x = orientation[0]
        dock_pose.pose.orientation.y = orientation[1]
        dock_pose.pose.orientation.z = orientation[2]
        dock_pose.pose.orientation.w = orientation[3]
        return dock_pose

    def _call_get_location(self, medicine_id: str) -> Optional[GetMedicineLocation.Response]:
        request = GetMedicineLocation.Request()
        request.medicine_id = medicine_id
        return self._call_service(self._get_location_client, request)

    def _call_resolve_local_pick(
        self,
        medicine_id: str,
        reference_pose_map: PoseStamped,
    ) -> Optional[ResolveLocalPick.Response]:
        request = ResolveLocalPick.Request()
        request.medicine_id = medicine_id
        request.reference_item_pose_map = reference_pose_map
        return self._call_service(self._resolve_local_pick_client, request)

    def _call_trigger(self, client) -> Optional[Trigger.Response]:
        return self._call_service(client, Trigger.Request())

    def _call_service(self, client, request):
        timeout_sec = float(self.get_parameter("service_wait_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None

        future = client.call_async(request)
        event = Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout=timeout_sec):
            return None
        return future.result()

    def _navigate_to_pose(self, dock_pose: PoseStamped) -> Optional[str]:
        timeout_sec = float(self.get_parameter("service_wait_timeout_sec").value)
        if not self._navigate_client.wait_for_server(timeout_sec=timeout_sec):
            return "NavigateToPose action server is unavailable."

        goal = NavigateToPose.Goal()
        goal.pose = dock_pose
        send_goal_future = self._navigate_client.send_goal_async(goal)
        send_goal_event = Event()
        send_goal_future.add_done_callback(lambda _future: send_goal_event.set())
        if not send_goal_event.wait(timeout=timeout_sec):
            return "Timed out while sending NavigateToPose goal."

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return "NavigateToPose goal was rejected."

        result_future = goal_handle.get_result_async()
        result_event = Event()
        result_future.add_done_callback(lambda _future: result_event.set())
        navigate_timeout_sec = float(self.get_parameter("navigate_timeout_sec").value)
        if not result_event.wait(timeout=navigate_timeout_sec):
            goal_handle.cancel_goal_async()
            return "NavigateToPose timed out."

        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            return f"NavigateToPose failed with status={getattr(result, 'status', 'unknown')}."
        return None

    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        if not target_frame or pose.header.frame_id == target_frame:
            return pose

        try:
            if not self._tf_buffer.can_transform(
                target_frame,
                pose.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.0),
            ):
                self.get_logger().warning(
                    f"Transform from {pose.header.frame_id} to {target_frame} is not available in the TF buffer yet."
                )
                return None
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Failed to transform pose from {pose.header.frame_id} to {target_frame}: {exc}"
            )
            return None

        transform_rotation = normalize_quaternion(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )
        transform_translation = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        pose_rotation = normalize_quaternion(
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            )
        )
        rotated_position = rotate_vector(
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ),
            transform_rotation,
        )

        result = PoseStamped()
        result.header = pose.header
        result.header.frame_id = target_frame
        result.pose.position.x = rotated_position[0] + transform_translation[0]
        result.pose.position.y = rotated_position[1] + transform_translation[1]
        result.pose.position.z = rotated_position[2] + transform_translation[2]
        result_rotation = normalize_quaternion(quat_multiply(transform_rotation, pose_rotation))
        result.pose.orientation.x = result_rotation[0]
        result.pose.orientation.y = result_rotation[1]
        result.pose.orientation.z = result_rotation[2]
        result.pose.orientation.w = result_rotation[3]
        return result

    def destroy_node(self) -> bool:
        self._client_executor.shutdown()
        self._client_node.destroy_node()
        if self._client_executor_thread.is_alive():
            self._client_executor_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FetchMedicineCoordinator()
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
