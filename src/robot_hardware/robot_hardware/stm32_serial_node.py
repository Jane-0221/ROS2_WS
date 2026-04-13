#!/usr/bin/env python3

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray
from std_msgs.msg import String

from robot_hardware.medipick_hardware_calibration import (
    REQUIRED_ARM_JOINTS,
    HardwareCalibration,
    load_hardware_calibration,
)


FRAME_HEADER = b"\xAA\x55"
FRAME_TAIL = b"\xEE\xFF"
UP_FRAME_TYPE = 0x01
UP_DATA_LEN = 56
UP_FLOAT_COUNT = 14
UP_FRAME_SIZE = 64
DN_FRAME_TYPE = 0x02
DN_DATA_LEN = 51


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF
    return crc


@dataclass
class UpData:
    air_path_state: float = 0.0
    suck_state: float = 0.0
    head_motor_angle_1: float = 0.0
    head_motor_angle_2: float = 0.0
    arm_motor_angles: list[float] = field(default_factory=lambda: [0.0] * 6)
    lift_height: float = 0.0
    chassis_vx: float = 0.0
    chassis_vy: float = 0.0
    chassis_yaw: float = 0.0


@dataclass
class DnData:
    motor_arm_joint_targets: list[float] = field(default_factory=lambda: [0.0] * 12)
    pump_state: int = 0
    target_lift_height: float = 0.0


class STM32SerialNode(Node):
    def __init__(self) -> None:
        super().__init__("stm32_serial_node")

        self.declare_parameter("serial_port", "/dev/ttySTM32")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("arm_joint_states_topic", "/medipick/hardware/arm_joint_states")
        self.declare_parameter(
            "motor_arm_joint_command_topic",
            "/medipick/hardware/motor_arm_joint_command",
        )
        self.declare_parameter(
            "legacy_arm_joint_command_topic",
            "",
        )
        self.declare_parameter(
            "motor_arm_joint_feedback_topic",
            "/medipick/hardware/motor_arm_joint_feedback",
        )
        self.declare_parameter(
            "stm32_base_cmd_vel_topic",
            "/medipick/hardware/stm32_base_cmd_vel",
        )
        self.declare_parameter(
            "parsed_summary_topic",
            "/medipick/hardware/stm32_parsed_summary",
        )
        self.declare_parameter(
            "parsed_chassis_summary_topic",
            "/medipick/hardware/stm32_chassis_summary",
        )
        self.declare_parameter("stm32_base_linear_scale", 1.0)
        self.declare_parameter("stm32_base_angular_scale", 1.0)
        self.declare_parameter("lift_height_topic", "/medipick/hardware/raw_lift_height_mm")
        self.declare_parameter("raw_suction_topic", "/medipick/hardware/raw_suction_state")
        self.declare_parameter("legacy_servo_control_topic", "/stm32/servo_control")
        self.declare_parameter("enable_legacy_servo_control_topic", False)
        self.declare_parameter("publish_full_joint_states", True)
        self.declare_parameter("publish_base_joints", True)
        self.declare_parameter("log_received_up_frames", False)
        self.declare_parameter("send_period_sec", 0.05)
        self.declare_parameter("recv_period_sec", 0.01)

        calibration_file = str(self.get_parameter("calibration_file").value).strip()
        self._calibration = load_hardware_calibration(calibration_file)
        self._validate_calibration(self._calibration)

        serial_port = str(self.get_parameter("serial_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                timeout=0.01,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self.get_logger().info(f"Opened STM32 serial port {serial_port} @ {baudrate}.")
        except Exception as exc:  # pragma: no cover - hardware dependent
            raise RuntimeError(f"Failed to open STM32 serial port {serial_port}: {exc}") from exc

        self.up_data = UpData()
        self.dn_data = DnData()
        self._recv_buffer = bytearray()

        self._publish_full_joint_states = bool(self.get_parameter("publish_full_joint_states").value)
        self._publish_base_joints = bool(self.get_parameter("publish_base_joints").value)
        self._stm32_base_linear_scale = float(self.get_parameter("stm32_base_linear_scale").value)
        self._stm32_base_angular_scale = float(self.get_parameter("stm32_base_angular_scale").value)

        self._joint_state_publisher = None
        if self._publish_full_joint_states:
            self._joint_state_publisher = self.create_publisher(
                JointState,
                str(self.get_parameter("joint_states_topic").value),
                20,
            )
        self._arm_joint_state_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("arm_joint_states_topic").value),
            20,
        )
        self._lift_height_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("lift_height_topic").value),
            20,
        )
        self._raw_suction_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("raw_suction_topic").value),
            20,
        )
        self._motor_arm_feedback_publisher = self.create_publisher(
            JointState,
            str(self.get_parameter("motor_arm_joint_feedback_topic").value),
            20,
        )
        self._base_cmd_vel_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("stm32_base_cmd_vel_topic").value),
            20,
        )
        self._parsed_summary_publisher = self.create_publisher(
            String,
            str(self.get_parameter("parsed_summary_topic").value),
            20,
        )
        self._parsed_chassis_summary_publisher = self.create_publisher(
            String,
            str(self.get_parameter("parsed_chassis_summary_topic").value),
            20,
        )

        self.create_subscription(Float32, "stm32/target_height", self.height_callback, 10)
        self.create_subscription(Bool, "stm32/pump_control", self.pump_callback, 10)
        self.create_subscription(
            JointState,
            str(self.get_parameter("motor_arm_joint_command_topic").value),
            self.motor_arm_joint_command_callback,
            10,
        )
        legacy_joint_command_topic = str(self.get_parameter("legacy_arm_joint_command_topic").value)
        primary_joint_command_topic = str(self.get_parameter("motor_arm_joint_command_topic").value)
        if legacy_joint_command_topic and legacy_joint_command_topic != primary_joint_command_topic:
            self.create_subscription(
                JointState,
                legacy_joint_command_topic,
                self.motor_arm_joint_command_callback,
                10,
            )
        if bool(self.get_parameter("enable_legacy_servo_control_topic").value):
            self.create_subscription(
                Float32MultiArray,
                str(self.get_parameter("legacy_servo_control_topic").value),
                self.legacy_servo_callback,
                10,
            )

        self.create_timer(float(self.get_parameter("send_period_sec").value), self.send_down_frame)
        self.create_timer(float(self.get_parameter("recv_period_sec").value), self.recv_up_frame)

    @staticmethod
    def _validate_calibration(calibration: HardwareCalibration) -> None:
        for joint_name in REQUIRED_ARM_JOINTS:
            mapping = calibration.arm_joint_mappings[joint_name]
            if mapping.command_index < 0 or mapping.command_index >= 12:
                raise ValueError(
                    f"Joint '{joint_name}' command_index={mapping.command_index} is outside 0..11."
                )
            if mapping.feedback_index < 4 or mapping.feedback_index >= 10:
                raise ValueError(
                    f"Joint '{joint_name}' feedback_index={mapping.feedback_index} is outside 4..9 for the current STM32 frame."
                )

    def pack_down_frame(self) -> bytes:
        frame = bytearray()
        frame.extend(FRAME_HEADER)
        frame.append(DN_FRAME_TYPE)
        frame.append(DN_DATA_LEN)

        for angle in self.dn_data.motor_arm_joint_targets:
            frame.extend(struct.pack("<f", angle))
        frame.append(self.dn_data.pump_state)
        height_int = int(self.dn_data.target_lift_height * 10.0)
        frame.extend(struct.pack("<H", height_int))

        crc = crc16_ccitt(frame[2:])
        frame.extend(struct.pack(">H", crc))
        frame.extend(FRAME_TAIL)
        return bytes(frame)

    def send_down_frame(self) -> None:
        if not self.ser or not self.ser.is_open:
            return
        try:
            self.ser.write(self.pack_down_frame())
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().error(f"Failed to send STM32 frame: {exc}")

    def recv_up_frame(self) -> None:
        if not self.ser or not self.ser.is_open:
            return
        try:
            if self.ser.in_waiting:
                self._recv_buffer.extend(self.ser.read(self.ser.in_waiting))
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.get_logger().error(f"Failed to read STM32 data: {exc}")
            return

        while len(self._recv_buffer) >= UP_FRAME_SIZE:
            header_index = self._recv_buffer.find(FRAME_HEADER)
            if header_index < 0:
                self._recv_buffer.clear()
                return
            if header_index > 0:
                del self._recv_buffer[:header_index]
            if len(self._recv_buffer) < UP_FRAME_SIZE:
                return

            frame = bytes(self._recv_buffer[:UP_FRAME_SIZE])
            if frame[2] != UP_FRAME_TYPE or frame[3] != UP_DATA_LEN:
                del self._recv_buffer[0]
                continue
            if frame[-2:] != FRAME_TAIL:
                del self._recv_buffer[0]
                continue

            crc_data = frame[2:-4]
            crc_calc = crc16_ccitt(crc_data)
            crc_recv = struct.unpack("<H", frame[-4:-2])[0]
            if crc_calc != crc_recv:
                del self._recv_buffer[0]
                continue

            del self._recv_buffer[:UP_FRAME_SIZE]
            self.up_data = self.unpack_up_frame(frame)
            if bool(self.get_parameter("log_received_up_frames").value):
                self.get_logger().info(self.format_up_data(self.up_data))
            self.publish_feedback()

    @staticmethod
    def unpack_up_frame(frame: bytes) -> UpData:
        if len(frame) != UP_FRAME_SIZE:
            raise ValueError(f"STM32 up-frame must be exactly {UP_FRAME_SIZE} bytes, got {len(frame)}.")
        if frame[2] != UP_FRAME_TYPE:
            raise ValueError(f"STM32 up-frame type must be 0x{UP_FRAME_TYPE:02X}, got 0x{frame[2]:02X}.")
        if frame[3] != UP_DATA_LEN:
            raise ValueError(f"STM32 up-frame data_len must be {UP_DATA_LEN}, got {frame[3]}.")

        values = struct.unpack("<14f", frame[4:4 + UP_DATA_LEN])
        return UpData(
            air_path_state=values[0],
            suck_state=values[1],
            head_motor_angle_1=values[2],
            head_motor_angle_2=values[3],
            arm_motor_angles=list(values[4:10]),
            lift_height=values[10],
            chassis_vx=values[11],
            chassis_vy=values[12],
            chassis_yaw=values[13],
        )

    @staticmethod
    def format_up_data(up_data: UpData) -> str:
        arm_values = ", ".join(f"{value:.4f}" for value in up_data.arm_motor_angles)
        return (
            "STM32 up-frame: "
            f"air_path_state={up_data.air_path_state:.4f}, "
            f"suck_state={up_data.suck_state:.4f}, "
            f"head_motor_angle_1={up_data.head_motor_angle_1:.4f}, "
            f"head_motor_angle_2={up_data.head_motor_angle_2:.4f}, "
            f"arm_motor_angles=[{arm_values}], "
            f"lift_height={up_data.lift_height:.4f}, "
            f"chassis_vx={up_data.chassis_vx:.4f}, "
            f"chassis_vy={up_data.chassis_vy:.4f}, "
            f"chassis_yaw={up_data.chassis_yaw:.4f}"
        )

    def publish_feedback(self) -> None:
        lift_height_mm = float(self.up_data.lift_height)
        self._lift_height_publisher.publish(Float32(data=lift_height_mm))
        self._raw_suction_publisher.publish(Bool(data=bool(self.up_data.suck_state >= 0.5)))

        scaled_chassis_vx = self.up_data.chassis_vx * self._stm32_base_linear_scale
        scaled_chassis_vy = self.up_data.chassis_vy * self._stm32_base_linear_scale
        scaled_chassis_yaw = self.up_data.chassis_yaw * self._stm32_base_angular_scale

        base_cmd = Twist()
        base_cmd.linear.x = scaled_chassis_vx
        base_cmd.linear.y = scaled_chassis_vy
        base_cmd.angular.z = scaled_chassis_yaw
        self._base_cmd_vel_publisher.publish(base_cmd)
        self._parsed_summary_publisher.publish(String(data=self.format_up_data(self.up_data)))
        self._parsed_chassis_summary_publisher.publish(
            String(
                data=(
                    f"raw: vx={self.up_data.chassis_vx:.4f} m/s, "
                    f"vy={self.up_data.chassis_vy:.4f} m/s, "
                    f"yaw={self.up_data.chassis_yaw:.4f} rad/s | "
                    f"scaled: vx={scaled_chassis_vx:.4f} m/s, "
                    f"vy={scaled_chassis_vy:.4f} m/s, "
                    f"yaw={scaled_chassis_yaw:.4f} rad/s"
                )
            )
        )

        arm_joint_state = JointState()
        arm_joint_state.header.stamp = self.get_clock().now().to_msg()
        ordered_arm_joint_names = [
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

        joint_positions = dict(self._calibration.base_joints)
        joint_positions["raise_joint"] = self._calibration.lift.mm_to_joint(lift_height_mm)
        for joint_name, mapping in self._calibration.arm_joint_mappings.items():
            arm_index = mapping.feedback_index - 4
            raw_feedback = self.up_data.arm_motor_angles[arm_index]
            joint_positions[joint_name] = mapping.feedback_to_joint(raw_feedback)
        joint_positions.update(self._calibration.fixed_joints)

        arm_joint_state.name = ordered_arm_joint_names
        arm_joint_state.position = [joint_positions.get(joint_name, 0.0) for joint_name in ordered_arm_joint_names]
        self._arm_joint_state_publisher.publish(arm_joint_state)

        if self._joint_state_publisher is not None:
            joint_state = JointState()
            joint_state.header = arm_joint_state.header
            ordered_joint_names = list(ordered_arm_joint_names)
            if self._publish_base_joints:
                ordered_joint_names = ["base_x", "base_y", "base_theta"] + ordered_joint_names
            joint_state.name = ordered_joint_names
            joint_state.position = [joint_positions.get(joint_name, 0.0) for joint_name in ordered_joint_names]
            self._joint_state_publisher.publish(joint_state)

        motor_arm_feedback = JointState()
        motor_arm_feedback.header = arm_joint_state.header
        motor_arm_feedback.name = list(self._calibration.arm_joint_mappings.keys())
        motor_arm_feedback.position = [joint_positions[joint_name] for joint_name in motor_arm_feedback.name]
        self._motor_arm_feedback_publisher.publish(motor_arm_feedback)

    def height_callback(self, msg: Float32) -> None:
        self.dn_data.target_lift_height = float(msg.data)

    def pump_callback(self, msg: Bool) -> None:
        self.dn_data.pump_state = 1 if msg.data else 0

    def legacy_servo_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().error("Legacy servo control payload must be [channel_id, command_value].")
            return
        servo_id = int(msg.data[0])
        if servo_id < 0 or servo_id >= len(self.dn_data.motor_arm_joint_targets):
            self.get_logger().error(f"Motor-arm channel id {servo_id} is outside 0..11.")
            return
        self.dn_data.motor_arm_joint_targets[servo_id] = float(msg.data[1])

    def motor_arm_joint_command_callback(self, msg: JointState) -> None:
        joint_count = min(len(msg.name), len(msg.position))
        for index in range(joint_count):
            joint_name = msg.name[index]
            mapping = self._calibration.arm_joint_mappings.get(joint_name)
            if mapping is None:
                continue
            self.dn_data.motor_arm_joint_targets[mapping.command_index] = mapping.joint_to_command(
                msg.position[index]
            )

    def destroy_node(self) -> bool:
        if getattr(self, "ser", None) is not None and self.ser.is_open:
            self.ser.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = STM32SerialNode()
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
