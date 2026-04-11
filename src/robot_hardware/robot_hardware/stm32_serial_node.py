#!/usr/bin/env python3

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray

from robot_hardware.medipick_hardware_calibration import (
    REQUIRED_ARM_JOINTS,
    HardwareCalibration,
    load_hardware_calibration,
)


FRAME_HEADER = b"\xAA\x55"
FRAME_TAIL = b"\xEE\xFF"
UP_FRAME_SIZE = 34
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
    air_path_state: int = 0
    suck_state: int = 0
    head_motor_angle: float = 0.0
    motor_arm_joint_feedback: list[float] = field(default_factory=lambda: [0.0] * 11)
    lift_height: float = 0.0


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
        self.declare_parameter("lift_height_topic", "/medipick/hardware/raw_lift_height_mm")
        self.declare_parameter("raw_suction_topic", "/medipick/hardware/raw_suction_state")
        self.declare_parameter("legacy_servo_control_topic", "/stm32/servo_control")
        self.declare_parameter("enable_legacy_servo_control_topic", False)
        self.declare_parameter("publish_full_joint_states", True)
        self.declare_parameter("publish_base_joints", True)
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
            if mapping.feedback_index < 0 or mapping.feedback_index >= 11:
                raise ValueError(
                    f"Joint '{joint_name}' feedback_index={mapping.feedback_index} is outside 0..10."
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
            self.publish_feedback()

    @staticmethod
    def unpack_up_frame(frame: bytes) -> UpData:
        up_data = UpData()
        offset = 4
        up_data.air_path_state = frame[offset]
        offset += 1
        up_data.suck_state = frame[offset]
        offset += 1
        head_angle_int = struct.unpack("<h", frame[offset:offset + 2])[0]
        up_data.head_motor_angle = head_angle_int / 10.0
        offset += 2
        for index in range(11):
            angle_int = struct.unpack("<h", frame[offset:offset + 2])[0]
            up_data.motor_arm_joint_feedback[index] = angle_int / 10.0
            offset += 2
        height_int = struct.unpack("<H", frame[offset:offset + 2])[0]
        up_data.lift_height = height_int / 10.0
        return up_data

    def publish_feedback(self) -> None:
        lift_height_mm = float(self.up_data.lift_height)
        self._lift_height_publisher.publish(Float32(data=lift_height_mm))
        self._raw_suction_publisher.publish(Bool(data=bool(self.up_data.suck_state)))

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
            raw_feedback = self.up_data.motor_arm_joint_feedback[mapping.feedback_index]
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
