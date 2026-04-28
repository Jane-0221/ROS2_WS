#!/usr/bin/python3

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Simple5AssetJointStatePublisher(Node):
    DEFAULT_POSITIONS = {
        "h2_joint": -0.5236,
    }

    def __init__(self) -> None:
        super().__init__("simple5_asset_joint_state_publisher")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("robot_description", "")
        self.declare_parameter("joint_state_topic", "/simple5_asset_preview/joint_states")

        robot_description = str(self.get_parameter("robot_description").value)
        if not robot_description.strip():
            raise RuntimeError("robot_description 为空，无法发布原始 simple5 关节状态。")

        self.joint_names = self._extract_movable_joint_names(robot_description)
        joint_state_topic = str(self.get_parameter("joint_state_topic").value).strip() or "/simple5_asset_preview/joint_states"
        self.publisher = self.create_publisher(JointState, joint_state_topic, 10)
        publish_rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_joint_state)

        self.get_logger().info(
            f"原始 simple5 预览关节发布器已启动，关节数: {len(self.joint_names)}，话题: {joint_state_topic}"
        )

    @staticmethod
    def _extract_movable_joint_names(robot_description: str) -> list[str]:
        root = ET.fromstring(robot_description)
        joint_names: list[str] = []
        for joint in root.findall("joint"):
            joint_type = joint.attrib.get("type", "").strip()
            joint_name = joint.attrib.get("name", "").strip()
            if not joint_name:
                continue
            if joint_type in ("fixed", "floating", "planar"):
                continue
            joint_names.append(joint_name)
        return joint_names

    def publish_joint_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.joint_names)
        message.position = [self.DEFAULT_POSITIONS.get(joint_name, 0.0) for joint_name in self.joint_names]
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = Simple5AssetJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
