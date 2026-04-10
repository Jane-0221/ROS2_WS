#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64

from robot_hardware.medipick_hardware_calibration import load_hardware_calibration


class LiftCommandAdapter(Node):
    def __init__(self) -> None:
        super().__init__("medipick_lift_command_adapter")

        self.declare_parameter("calibration_file", "")
        self.declare_parameter("input_topic", "/medipick/task/lift_target_height")
        self.declare_parameter("output_topic", "stm32/target_height")
        self.declare_parameter("diagnostic_topic", "/medipick/hardware/target_lift_height_mm")

        calibration_file = str(self.get_parameter("calibration_file").value).strip()
        self._calibration = load_hardware_calibration(calibration_file)
        self._height_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self._diagnostic_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("diagnostic_topic").value),
            10,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter("input_topic").value),
            self._on_lift_target,
            10,
        )
        self.get_logger().info("Lift command adapter ready.")

    def _on_lift_target(self, msg: Float64) -> None:
        target_height_mm = self._calibration.lift.joint_to_mm(msg.data)
        height_message = Float32(data=float(target_height_mm))
        self._height_publisher.publish(height_message)
        self._diagnostic_publisher.publish(height_message)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LiftCommandAdapter()
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
