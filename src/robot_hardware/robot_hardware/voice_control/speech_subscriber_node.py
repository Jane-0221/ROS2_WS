#!/usr/bin/env python3
"""
简单的语音识别结果订阅节点。

用于保留 FunASR 旧链路中的调试观察能力。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechSubscriberNode(Node):
    """订阅并打印语音识别结果。"""

    def __init__(self):
        super().__init__('speech_subscriber_node')
        self.subscription = self.create_subscription(
            String,
            '/speech_recognition/text',
            self.speech_callback,
            10,
        )

        self.get_logger().info('语音识别结果订阅节点已启动')
        self.get_logger().info('订阅话题: /speech_recognition/text')

    def speech_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'收到语音识别结果: "{text}"')


def main(args=None):
    rclpy.init(args=args)
    node = SpeechSubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
