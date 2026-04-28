#!/usr/bin/env python3

import math
import os
import struct
import time

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Float32MultiArray

# -------------------------- 达锂BMS RS485协议常量（V1.2） --------------------------
DEFAULT_BATTERY_PORT = "/dev/ttyBattery"
DEFAULT_BAUDRATE = 9600
BYTESIZE = serial.EIGHTBITS
STOPBITS = serial.STOPBITS_ONE
PARITY = serial.PARITY_NONE

FRAME_HEADER = 0xA5
PC_ADDR = 0x40
BMS_ADDR = 0x01
DATA_ID_SOC = 0x90
DATA_LEN_FIX = 0x08
FRAME_SEND_LEN = 13
FRAME_RECV_LEN = 13

VOLT_COEFF = 0.1
CURR_COEFF = 0.1
CURR_OFFSET = 30000
SOC_COEFF = 0.1


class BmsSocReaderNode(Node):
    def __init__(self):
        super().__init__("bms_soc_reader_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("port", DEFAULT_BATTERY_PORT),
                ("baudrate", DEFAULT_BAUDRATE),
                ("poll_period_sec", 1.0),
                ("reconnect_period_sec", 3.0),
                ("frame_id", "base_link"),
                ("soc_topic", "bms/battery_soc"),
                ("raw_data_topic", "bms/battery_data"),
                ("battery_state_topic", "battery"),
                ("status_current_threshold_a", 0.2),
                ("exclusive_port", True),
            ],
        )

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.poll_period_sec = float(self.get_parameter("poll_period_sec").value)
        self.reconnect_period_sec = float(self.get_parameter("reconnect_period_sec").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.status_current_threshold_a = float(self.get_parameter("status_current_threshold_a").value)
        self.exclusive_port = bool(self.get_parameter("exclusive_port").value)

        self.soc_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("soc_topic").value),
            10,
        )
        self.bms_data_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("raw_data_topic").value),
            10,
        )
        self.battery_state_pub = self.create_publisher(
            BatteryState,
            str(self.get_parameter("battery_state_topic").value),
            10,
        )

        self.ser = None
        self._last_open_attempt_monotonic = 0.0

        self.battery_soc = 0.0
        self.total_volt = 0.0
        self.battery_curr = 0.0

        self.query_timer = self.create_timer(self.poll_period_sec, self.bms_query_task)
        self._open_serial(initial=True)

    def calc_checksum(self, data_list: list[int]) -> int:
        return sum(data_list) & 0xFF

    def _describe_port(self) -> str:
        real_path = os.path.realpath(self.port)
        if real_path and real_path != self.port:
            return f"{self.port} -> {real_path}"
        return self.port

    def _open_serial(self, initial: bool = False) -> bool:
        if self.ser and self.ser.is_open:
            return True

        now = time.monotonic()
        if not initial and now - self._last_open_attempt_monotonic < self.reconnect_period_sec:
            return False
        self._last_open_attempt_monotonic = now

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=BYTESIZE,
                parity=PARITY,
                stopbits=STOPBITS,
                timeout=0.2,
                write_timeout=0.2,
                exclusive=self.exclusive_port,
            )
            self.get_logger().info(
                f"RS485串口 {self._describe_port()} 打开成功，波特率 {self.baudrate}，exclusive={self.exclusive_port}"
            )
            return True
        except Exception as exc:
            log_fn = self.get_logger().error if initial else self.get_logger().warn
            log_fn(
                f"RS485串口 {self._describe_port()} 打开失败: {exc}"
            )
            self.ser = None
            return False

    def _close_serial(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def send_bms_query_cmd(self) -> bool:
        if not self.ser or not self.ser.is_open:
            return False

        try:
            send_frame = [0x00] * FRAME_SEND_LEN
            send_frame[0] = FRAME_HEADER
            send_frame[1] = PC_ADDR
            send_frame[2] = DATA_ID_SOC
            send_frame[3] = DATA_LEN_FIX
            send_frame[4:12] = [0x00] * 8
            send_frame[12] = self.calc_checksum(send_frame[0:12])

            self.ser.reset_input_buffer()
            self.ser.write(bytes(send_frame))
            self.ser.flush()
            return True
        except Exception as exc:
            self.get_logger().error(
                f"发送BMS指令失败 ({self._describe_port()}): {exc}"
            )
            self._close_serial()
            return False

    def recv_and_parse_bms_frame(self) -> bool:
        if not self.ser or not self.ser.is_open:
            return False

        try:
            recv_data = self.ser.read(FRAME_RECV_LEN)
            if len(recv_data) < FRAME_RECV_LEN:
                return False

            header_idx = recv_data.find(bytes([FRAME_HEADER]))
            if header_idx == -1:
                self.get_logger().warn("未找到BMS响应帧头，已丢弃本次数据")
                return False

            recv_frame = recv_data[header_idx:header_idx + FRAME_RECV_LEN]
            if len(recv_frame) != FRAME_RECV_LEN:
                return False

            if recv_frame[1] != BMS_ADDR or recv_frame[2] != DATA_ID_SOC:
                self.get_logger().warn("BMS地址或数据ID不匹配，已丢弃本次数据")
                return False

            recv_checksum = recv_frame[-1]
            calc_checksum = self.calc_checksum(recv_frame[0:-1])
            if recv_checksum != calc_checksum:
                self.get_logger().warn(
                    f"BMS校验和失败: 接收 {recv_checksum:02X}，计算 {calc_checksum:02X}"
                )
                return False

            frame_data = recv_frame[4:12]
            self.total_volt = struct.unpack(">H", frame_data[0:2])[0] * VOLT_COEFF
            curr_raw = struct.unpack(">H", frame_data[4:6])[0]
            self.battery_curr = (curr_raw - CURR_OFFSET) * CURR_COEFF
            self.battery_soc = struct.unpack(">H", frame_data[6:8])[0] * SOC_COEFF
            self.battery_soc = max(0.0, min(100.0, self.battery_soc))

            self.get_logger().info(
                "BMS数据解析成功 | 总压: %.1fV | 电流: %.1fA | SOC: %.1f%%"
                % (self.total_volt, self.battery_curr, self.battery_soc)
            )
            return True
        except Exception as exc:
            self.get_logger().error(
                f"接收/解析BMS帧失败 ({self._describe_port()}): {exc}"
            )
            self._close_serial()
            return False

    def _battery_status_from_current(self) -> int:
        if self.battery_curr > self.status_current_threshold_a:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if self.battery_curr < -self.status_current_threshold_a:
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

    def publish_bms_data(self) -> None:
        soc_msg = Float32()
        soc_msg.data = self.battery_soc
        self.soc_pub.publish(soc_msg)

        bms_data_msg = Float32MultiArray()
        bms_data_msg.data = [self.total_volt, self.battery_curr, self.battery_soc]
        self.bms_data_pub.publish(bms_data_msg)

        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = self.frame_id
        battery_msg.voltage = float(self.total_volt)
        battery_msg.current = float(self.battery_curr)
        battery_msg.charge = math.nan
        battery_msg.capacity = math.nan
        battery_msg.design_capacity = math.nan
        battery_msg.percentage = float(self.battery_soc / 100.0)
        battery_msg.power_supply_status = self._battery_status_from_current()
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery_msg.present = True
        self.battery_state_pub.publish(battery_msg)

    def bms_query_task(self) -> None:
        if not self._open_serial():
            return
        if not self.send_bms_query_cmd():
            return
        if not self.recv_and_parse_bms_frame():
            return
        self.publish_bms_data()

    def destroy_node(self):
        self._close_serial()
        self.get_logger().info("RS485串口已关闭")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BmsSocReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
