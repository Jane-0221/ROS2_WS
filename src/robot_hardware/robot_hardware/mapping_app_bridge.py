#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import io
import ipaddress
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response
from geometry_msgs.msg import Twist
from medipick_planning_interfaces.srv import SetBaseControlMode
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from PIL import Image as PilImage
from pydantic import BaseModel
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import uvicorn
from zeroconf import IPVersion, ServiceInfo, Zeroconf


class MappingStartRequest(BaseModel):
    delete_db_on_start: bool = False
    rtabmap_database_path: Optional[str] = None


class ControlModeRequest(BaseModel):
    mode: str


class MappingAppBridge(Node):
    def __init__(self) -> None:
        super().__init__("medipick_mapping_app_bridge")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("compressed_image_topic", "/camera/color/image_raw/compressed")
        self.declare_parameter("raw_image_topic", "/camera/color/image_raw")
        self.declare_parameter("app_cmd_vel_topic", "/medipick/app/cmd_vel")
        self.declare_parameter("active_mode_topic", "/medipick/base/active_mode")
        self.declare_parameter("mode_service", "/medipick/base/set_control_mode")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("mapping_launch_package", "medipick_planning_server")
        self.declare_parameter("mapping_launch_file", "scene_mapping_navigation.launch.py")
        self.declare_parameter("default_database_path", str(Path.home() / ".ros" / "medipick_scene_mapping.db"))
        self.declare_parameter("device_name", "")
        self.declare_parameter("advertise_service", True)
        self.declare_parameter("discovery_service_type", "_medipick._tcp.local.")
        self.declare_parameter("app_cmd_timeout_sec", 0.5)
        self.declare_parameter("app_max_linear_x", 3.0)
        self.declare_parameter("app_max_linear_y", 3.0)
        self.declare_parameter("app_max_angular_z", 3.0)
        self.declare_parameter(
            "mapping_log_path",
            str(Path.home() / ".ros" / "mapping_app_bridge_scene_mapping.log"),
        )

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._default_database_path = str(Path(str(self.get_parameter("default_database_path").value)).expanduser())
        self._hostname = socket.gethostname().strip() or "medipick-host"
        raw_device_name = str(self.get_parameter("device_name").value).strip()
        self._device_name = raw_device_name or f"MediPick-{self._hostname}"
        self._advertise_service = bool(self.get_parameter("advertise_service").value)
        self._discovery_service_type = str(self.get_parameter("discovery_service_type").value).strip() or "_medipick._tcp.local."
        self._mapping_launch_package = str(self.get_parameter("mapping_launch_package").value)
        self._mapping_launch_file = str(self.get_parameter("mapping_launch_file").value)
        self._mapping_log_path = Path(str(self.get_parameter("mapping_log_path").value)).expanduser()
        self._app_cmd_timeout_sec = float(self.get_parameter("app_cmd_timeout_sec").value)
        self._app_max_linear_x = float(self.get_parameter("app_max_linear_x").value)
        self._app_max_linear_y = float(self.get_parameter("app_max_linear_y").value)
        self._app_max_angular_z = float(self.get_parameter("app_max_angular_z").value)

        self._state_lock = threading.Lock()
        self._event_loop = None
        self._ws_clients: set[WebSocket] = set()

        self._active_mode = "unknown"
        self._map_png: Optional[bytes] = None
        self._map_headers: dict[str, str] = {}
        self._map_version = 0
        self._video_jpeg: Optional[bytes] = None
        self._video_version = 0
        self._robot_pose = None
        self._last_video_stamp = 0.0
        self._last_app_cmd_time = 0.0
        self._last_app_cmd = (0.0, 0.0, 0.0)

        self._mapping_process: Optional[subprocess.Popen] = None
        self._mapping_log_handle = None
        self._zeroconf: Optional[Zeroconf] = None
        self._service_info: Optional[ServiceInfo] = None

        self._app_cmd_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("app_cmd_vel_topic").value),
            20,
        )
        self._mode_client = self.create_client(
            SetBaseControlMode,
            str(self.get_parameter("mode_service").value),
        )

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._handle_map,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._handle_odometry,
            20,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("compressed_image_topic").value),
            self._handle_compressed_image,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("raw_image_topic").value),
            self._handle_raw_image,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("active_mode_topic").value),
            self._handle_active_mode,
            10,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self.create_timer(0.2, self._refresh_runtime_state)
        self.create_timer(0.1, self._enforce_app_cmd_timeout)
        self.create_timer(0.25, self._schedule_status_push)

        self.get_logger().info(f"Mapping app bridge ready at http://{self.host}:{self.port}")

    def build_fastapi_app(self) -> FastAPI:
        app = FastAPI(title="MediPick Mapping App Bridge", version="0.1.0")

        @app.on_event("startup")
        async def _startup() -> None:
            self._event_loop = asyncio.get_running_loop()
            await asyncio.to_thread(self.start_service_advertisement)

        @app.on_event("shutdown")
        async def _shutdown() -> None:
            await asyncio.to_thread(self.stop_service_advertisement)
            self.stop_mapping_process()
            self.publish_zero_app_cmd()

        @app.get("/health")
        async def health() -> JSONResponse:
            return JSONResponse(
                {
                    "ok": True,
                    "mapping_running": self.mapping_running,
                    "active_mode": self._active_mode,
                    "device_name": self._device_name,
                    "host_ip": self._detect_host_ip(),
                }
            )

        @app.get("/api/session")
        async def session() -> JSONResponse:
            return JSONResponse(self.build_status_payload())

        @app.post("/api/mapping/start")
        async def start_mapping(request: MappingStartRequest) -> JSONResponse:
            result = self.start_mapping_process(request)
            status_code = 200 if result["success"] else 500
            return JSONResponse(result, status_code=status_code)

        @app.post("/api/mapping/stop")
        async def stop_mapping() -> JSONResponse:
            result = self.stop_mapping_process()
            status_code = 200 if result["success"] else 500
            return JSONResponse(result, status_code=status_code)

        @app.post("/api/control/mode")
        async def set_control_mode(request: ControlModeRequest) -> JSONResponse:
            result = self.set_base_control_mode(request.mode)
            status_code = 200 if result["success"] else 400
            return JSONResponse(result, status_code=status_code)

        @app.post("/api/control/estop")
        async def estop() -> JSONResponse:
            self.publish_zero_app_cmd()
            return JSONResponse({"success": True, "message": "Emergency stop published."})

        @app.get("/api/map/latest")
        async def get_map() -> Response:
            with self._state_lock:
                map_png = self._map_png
                headers = dict(self._map_headers)
            if map_png is None:
                raise HTTPException(status_code=404, detail="Map not available yet.")
            return Response(content=map_png, media_type="image/png", headers=headers)

        @app.get("/api/video/latest.jpg")
        async def get_video() -> Response:
            with self._state_lock:
                video = self._video_jpeg
                version = self._video_version
            if video is None:
                raise HTTPException(status_code=404, detail="Video frame not available yet.")
            return Response(
                content=video,
                media_type="image/jpeg",
                headers={"X-Video-Version": str(version)},
            )

        @app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket) -> None:
            await websocket.accept()
            with self._state_lock:
                self._ws_clients.add(websocket)
            await websocket.send_text(json.dumps({"type": "status", **self.build_status_payload()}))
            try:
                while True:
                    message = await websocket.receive_text()
                    await self._handle_ws_message(websocket, message)
            except WebSocketDisconnect:
                pass
            finally:
                with self._state_lock:
                    self._ws_clients.discard(websocket)

        return app

    @property
    def mapping_running(self) -> bool:
        return self._mapping_process is not None and self._mapping_process.poll() is None

    def build_status_payload(self) -> dict:
        with self._state_lock:
            pose = self._robot_pose
            map_version = self._map_version
            video_version = self._video_version
            map_available = self._map_png is not None
            video_available = self._video_jpeg is not None
            active_mode = self._active_mode

        return {
            "available_modes": ["nav", "stm32", "app"],
            "active_mode": active_mode,
            "mapping_running": self.mapping_running,
            "device_name": self._device_name,
            "map_available": map_available,
            "map_version": map_version,
            "video_available": video_available,
            "video_version": video_version,
            "robot_pose": pose,
            "database_path": self._default_database_path,
            "host_ip": self._detect_host_ip(),
            "bridge_port": self.port,
            "timestamp": time.time(),
        }

    def start_mapping_process(self, request: MappingStartRequest) -> dict:
        if self.mapping_running:
            return {"success": True, "message": "Mapping is already running."}

        database_path = str(Path(request.rtabmap_database_path or self._default_database_path).expanduser())
        ros2_executable = shutil.which("ros2") or "/opt/ros/humble/bin/ros2"
        command = [
            ros2_executable,
            "launch",
            self._mapping_launch_package,
            self._mapping_launch_file,
            "localization:=false",
            "start_nav2:=false",
            "start_rtabmap_viz:=false",
            "start_stm32:=false",
            "base_control_mode:=app",
            f"delete_db_on_start:={'true' if request.delete_db_on_start else 'false'}",
            f"rtabmap_database_path:={database_path}",
        ]

        self._mapping_log_path.parent.mkdir(parents=True, exist_ok=True)
        self._mapping_log_handle = self._mapping_log_path.open("ab")
        self.publish_zero_app_cmd()
        try:
            self._mapping_process = subprocess.Popen(
                command,
                stdout=self._mapping_log_handle,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                env=os.environ.copy(),
            )
        except Exception as exc:
            self._mapping_process = None
            if self._mapping_log_handle is not None:
                self._mapping_log_handle.close()
                self._mapping_log_handle = None
            return {"success": False, "message": f"Failed to start mapping launch: {exc}"}

        self._default_database_path = database_path
        self.get_logger().info("Started scene mapping launch for app control.")
        return {
            "success": True,
            "message": "Mapping launch started.",
            "database_path": database_path,
            "pid": self._mapping_process.pid,
        }

    def stop_mapping_process(self) -> dict:
        self.publish_zero_app_cmd()
        process = self._mapping_process
        if process is None:
            return {"success": True, "message": "Mapping is not running."}

        success = True
        message = "Mapping launch stopped."
        try:
            pgid = os.getpgid(process.pid)
            os.killpg(pgid, signal.SIGTERM)
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if process.poll() is not None:
                    break
                time.sleep(0.1)
            if process.poll() is None:
                os.killpg(pgid, signal.SIGKILL)
                process.wait(timeout=5.0)
        except Exception as exc:
            success = False
            message = f"Failed to stop mapping launch cleanly: {exc}"
        finally:
            self._mapping_process = None
            if self._mapping_log_handle is not None:
                self._mapping_log_handle.close()
                self._mapping_log_handle = None
        return {"success": success, "message": message}

    def set_base_control_mode(self, mode: str) -> dict:
        mode = mode.strip().lower()
        if mode not in {"nav", "stm32", "app"}:
            return {
                "success": False,
                "message": "Unsupported mode. Expected one of nav/stm32/app.",
                "active_mode": self._active_mode,
            }

        if not self._mode_client.wait_for_service(timeout_sec=3.0):
            return {
                "success": False,
                "message": "Base control mode service is unavailable.",
                "active_mode": self._active_mode,
            }

        request = SetBaseControlMode.Request()
        request.mode = mode
        future = self._mode_client.call_async(request)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.05)

        if not future.done():
            return {
                "success": False,
                "message": "Timed out waiting for base control mode service.",
                "active_mode": self._active_mode,
            }

        try:
            response = future.result()
        except Exception as exc:
            return {
                "success": False,
                "message": f"Base control mode service failed: {exc}",
                "active_mode": self._active_mode,
            }

        if response.success:
            with self._state_lock:
                self._active_mode = response.active_mode
        return {
            "success": bool(response.success),
            "message": str(response.message),
            "active_mode": str(response.active_mode),
        }

    def publish_app_cmd(self, vx: float, vy: float, wz: float) -> None:
        command = Twist()
        command.linear.x = max(-self._app_max_linear_x, min(self._app_max_linear_x, float(vx)))
        command.linear.y = max(-self._app_max_linear_y, min(self._app_max_linear_y, float(vy)))
        command.angular.z = max(-self._app_max_angular_z, min(self._app_max_angular_z, float(wz)))
        self._app_cmd_publisher.publish(command)
        self._last_app_cmd_time = time.monotonic()
        self._last_app_cmd = (command.linear.x, command.linear.y, command.angular.z)

    def publish_zero_app_cmd(self) -> None:
        self.publish_app_cmd(0.0, 0.0, 0.0)

    def _handle_map(self, message: OccupancyGrid) -> None:
        map_image = np.asarray(message.data, dtype=np.int16).reshape((message.info.height, message.info.width))
        map_pixels = np.full_like(map_image, 205, dtype=np.uint8)
        map_pixels[map_image == 0] = 255
        map_pixels[map_image > 0] = 0
        map_pixels = np.flipud(map_pixels)

        image = PilImage.fromarray(map_pixels, mode="L")
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        buffer.seek(0)

        with self._state_lock:
            self._map_png = buffer.getvalue()
            self._map_version += 1
            self._map_headers = {
                "X-Map-Resolution": str(message.info.resolution),
                "X-Map-Origin-X": str(message.info.origin.position.x),
                "X-Map-Origin-Y": str(message.info.origin.position.y),
                "X-Map-Origin-Z": str(message.info.origin.position.z),
                "X-Map-Width": str(message.info.width),
                "X-Map-Height": str(message.info.height),
                "X-Map-Frame-Id": str(message.header.frame_id),
                "X-Map-Version": str(self._map_version),
            }

    def _handle_odometry(self, _message: Odometry) -> None:
        # Pose overlay uses TF in map frame; odometry subscription keeps bridge state aligned
        return

    def _handle_compressed_image(self, message: CompressedImage) -> None:
        with self._state_lock:
            self._video_jpeg = bytes(message.data)
            self._video_version += 1
            self._last_video_stamp = time.monotonic()

    def _handle_raw_image(self, message: Image) -> None:
        with self._state_lock:
            if time.monotonic() - self._last_video_stamp < 1.0 and self._video_jpeg is not None:
                return

        jpeg = self._jpeg_from_raw_image(message)
        if jpeg is None:
            return

        with self._state_lock:
            self._video_jpeg = jpeg
            self._video_version += 1
            self._last_video_stamp = time.monotonic()

    def _handle_active_mode(self, message: String) -> None:
        with self._state_lock:
            self._active_mode = message.data

    def _refresh_runtime_state(self) -> None:
        if self._mapping_process is not None and self._mapping_process.poll() is not None:
            self.get_logger().warn("Scene mapping launch exited.")
            self._mapping_process = None
            if self._mapping_log_handle is not None:
                self._mapping_log_handle.close()
                self._mapping_log_handle = None

        try:
            transform = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            pose = {
                "frame_id": self._map_frame,
                "x": translation.x,
                "y": translation.y,
                "z": translation.z,
                "yaw": yaw,
            }
            with self._state_lock:
                self._robot_pose = pose
        except TransformException:
            pass

    def _enforce_app_cmd_timeout(self) -> None:
        if self._active_mode != "app":
            return

        if time.monotonic() - self._last_app_cmd_time <= self._app_cmd_timeout_sec:
            return

        if any(abs(value) > 1e-4 for value in self._last_app_cmd):
            self.publish_zero_app_cmd()

    def _schedule_status_push(self) -> None:
        if self._event_loop is None:
            return

        with self._state_lock:
            if not self._ws_clients:
                return

        asyncio.run_coroutine_threadsafe(self._broadcast_status(), self._event_loop)

    async def _broadcast_status(self) -> None:
        payload = json.dumps({"type": "status", **self.build_status_payload()})
        stale_clients = []
        with self._state_lock:
            clients = list(self._ws_clients)

        for websocket in clients:
            try:
                await websocket.send_text(payload)
            except Exception:
                stale_clients.append(websocket)

        if stale_clients:
            with self._state_lock:
                for websocket in stale_clients:
                    self._ws_clients.discard(websocket)

    async def _handle_ws_message(self, websocket: WebSocket, message: str) -> None:
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            await websocket.send_text(json.dumps({"type": "error", "message": "Invalid JSON payload."}))
            return

        message_type = str(payload.get("type", "")).strip().lower()
        if message_type == "cmd_vel":
            self.publish_app_cmd(
                float(payload.get("vx", 0.0)),
                float(payload.get("vy", 0.0)),
                float(payload.get("wz", 0.0)),
            )
            return

        if message_type == "ping":
            await websocket.send_text(json.dumps({"type": "status", **self.build_status_payload()}))
            return

        await websocket.send_text(json.dumps({"type": "error", "message": f"Unsupported message type '{message_type}'."}))

    def _jpeg_from_raw_image(self, message: Image) -> Optional[bytes]:
        try:
            data = np.frombuffer(message.data, dtype=np.uint8)
            encoding = message.encoding.lower()

            if encoding in {"rgb8", "bgr8"}:
                pixels = data.reshape((message.height, message.step // 3, 3))[:, : message.width, :]
                if encoding == "bgr8":
                    pixels = pixels[:, :, ::-1]
                image = PilImage.fromarray(pixels, mode="RGB")
            elif encoding in {"rgba8", "bgra8"}:
                pixels = data.reshape((message.height, message.step // 4, 4))[:, : message.width, :]
                if encoding == "bgra8":
                    pixels = pixels[:, :, [2, 1, 0, 3]]
                image = PilImage.fromarray(pixels, mode="RGBA").convert("RGB")
            elif encoding == "mono8":
                pixels = data.reshape((message.height, message.step))[:, : message.width]
                image = PilImage.fromarray(pixels, mode="L").convert("RGB")
            else:
                return None

            buffer = io.BytesIO()
            image.save(buffer, format="JPEG", quality=75)
            return buffer.getvalue()
        except Exception:
            return None

    def start_service_advertisement(self) -> None:
        if not self._advertise_service or self._zeroconf is not None:
            return

        addresses = self._detect_host_ipv4_addresses()
        if not addresses:
            self.get_logger().warn("No non-loopback IPv4 address found; skipping mDNS advertisement.")
            return

        service_type = self._normalized_service_type()
        service_label = self._sanitize_service_label(self._device_name)
        server_name = f"{self._sanitize_service_label(self._hostname)}.local."
        properties = {
            b"device_name": self._device_name.encode("utf-8"),
            b"http_path": b"/health",
            b"ws_path": b"/ws",
        }

        service_info = ServiceInfo(
            type_=service_type,
            name=f"{service_label}.{service_type}",
            addresses=[socket.inet_aton(address) for address in addresses],
            port=self.port,
            properties=properties,
            server=server_name,
        )

        try:
            zeroconf = Zeroconf(ip_version=IPVersion.V4Only)
            zeroconf.register_service(service_info, allow_name_change=True)
            self._zeroconf = zeroconf
            self._service_info = service_info
            self.get_logger().info(
                "Advertising mapping bridge via mDNS as '%s' on %s:%s"
                % (service_info.name, addresses[0], self.port)
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to advertise mDNS service: {exc!r}")
            if self._zeroconf is not None:
                self._zeroconf.close()
            self._zeroconf = None
            self._service_info = None

    def stop_service_advertisement(self) -> None:
        zeroconf = self._zeroconf
        service_info = self._service_info
        self._zeroconf = None
        self._service_info = None

        if zeroconf is None:
            return

        try:
            if service_info is not None:
                zeroconf.unregister_service(service_info)
        except Exception:
            pass
        finally:
            zeroconf.close()

    def _normalized_service_type(self) -> str:
        service_type = self._discovery_service_type.strip()
        if not service_type.endswith(".local."):
            service_type = f"{service_type.rstrip('.')}.local."
        return service_type

    @staticmethod
    def _sanitize_service_label(label: str) -> str:
        sanitized = "".join(character if character.isalnum() or character in "-_ " else "-" for character in label)
        sanitized = sanitized.strip().strip(".")
        return (sanitized or "MediPick-Bridge")[:63]

    @classmethod
    def _detect_host_ipv4_addresses(cls) -> list[str]:
        private_addresses: list[str] = []
        fallback_addresses: list[str] = []

        commands = [
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            ["hostname", "-I"],
        ]
        for command in commands:
            try:
                output = subprocess.check_output(command, text=True, stderr=subprocess.DEVNULL).strip()
            except Exception:
                continue

            if command[0] == "ip":
                for line in output.splitlines():
                    parts = line.split()
                    if len(parts) < 4:
                        continue
                    address = parts[3].split("/", 1)[0].strip()
                    cls._append_candidate_address(address, private_addresses, fallback_addresses)
            else:
                for candidate in output.split():
                    cls._append_candidate_address(candidate, private_addresses, fallback_addresses)

        if not private_addresses and not fallback_addresses:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.connect(("8.8.8.8", 80))
                    address = sock.getsockname()[0]
                    cls._append_candidate_address(address, private_addresses, fallback_addresses)
            except Exception:
                pass

        addresses = private_addresses or fallback_addresses
        unique_addresses: list[str] = []
        seen = set()
        for address in addresses:
            if address in seen:
                continue
            seen.add(address)
            unique_addresses.append(address)
        return unique_addresses

    @staticmethod
    def _is_valid_ipv4_for_lan(address: str) -> bool:
        if not address or address.startswith("127.") or address.startswith("169.254."):
            return False
        try:
            ip = ipaddress.IPv4Address(address)
        except OSError:
            return False
        except ipaddress.AddressValueError:
            return False

        if ip in ipaddress.IPv4Network("198.18.0.0/15"):
            return False
        return True

    @classmethod
    def _append_candidate_address(
        cls,
        address: str,
        private_addresses: list[str],
        fallback_addresses: list[str],
    ) -> None:
        if not cls._is_valid_ipv4_for_lan(address):
            return
        try:
            ip = ipaddress.IPv4Address(address)
        except ipaddress.AddressValueError:
            return

        if ip.is_private:
            private_addresses.append(address)
        else:
            fallback_addresses.append(address)

    @staticmethod
    def _detect_host_ip() -> str:
        addresses = MappingAppBridge._detect_host_ipv4_addresses()
        return addresses[0] if addresses else "127.0.0.1"


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MappingAppBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = node.build_fastapi_app()
    server = uvicorn.Server(
        uvicorn.Config(
            app,
            host=node.host,
            port=node.port,
            log_level="info",
        )
    )

    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_service_advertisement()
        node.stop_mapping_process()
        node.publish_zero_app_cmd()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
