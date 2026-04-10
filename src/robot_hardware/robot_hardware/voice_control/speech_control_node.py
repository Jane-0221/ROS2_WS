#!/usr/bin/env python3
"""
语音控制底盘运动节点

订阅语音识别结果，根据识别到的关键词控制底盘运动
支持命令：前进、停止、后退，以0.5m/s速度运动
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class SpeechControlNode(Node):
    """语音控制底盘运动节点"""
    
    def __init__(self):
        super().__init__('speech_control_node')
        
        # 参数配置
        self.declare_parameter('linear_speed', 0.2)  # 线速度 m/s
        self.declare_parameter('angular_speed', 0.2) # 角速度 rad/s
        self.declare_parameter('move_duration', 2.0) # 运动持续时间 秒
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.move_duration = self.get_parameter('move_duration').value
        
        # 创建订阅者 - 订阅语音识别结果
        self.speech_sub = self.create_subscription(
            String,
            '/speech_recognition/text',
            self.speech_callback,
            10
        )
        
        # 创建发布者 - 发布底盘控制命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # 控制状态
        self.is_moving = False
        self.current_command = None
        self.last_command_time = time.time()
        
        # 关键词映射
        self.command_keywords = {
            '前进': 'forward',
            '停止': 'stop', 
            '后退': 'backward',
            '左转': 'left',
            '右转': 'right'
        }
        
        # 创建定时器，持续发送运动命令避免超时
        self.command_timer = self.create_timer(0.1, self.publish_command)  # 10Hz
        
        self.get_logger().info('语音控制节点已启动')
        self.get_logger().info(f'线速度: {self.linear_speed} m/s')
        self.get_logger().info(f'角速度: {self.angular_speed} rad/s')
        self.get_logger().info('支持命令: 前进, 停止, 后退, 左转, 右转')
        self.get_logger().info('运动命令将持续发送，避免底盘超时停止')
    
    def publish_command(self):
        """持续发布当前运动命令"""
        if self.current_command and self.current_command != 'stop':
            twist_msg = Twist()
            
            if self.current_command == 'forward':
                twist_msg.linear.x = self.linear_speed
            elif self.current_command == 'backward':
                twist_msg.linear.x = -self.linear_speed
            elif self.current_command == 'left':
                twist_msg.angular.z = self.angular_speed
            elif self.current_command == 'right':
                twist_msg.angular.z = -self.angular_speed
            
            self.cmd_vel_pub.publish(twist_msg)
    
    def speech_callback(self, msg):
        """处理语音识别结果"""
        text = msg.data.strip()
        
        if not text:
            return
        
        self.get_logger().info(f'收到语音识别: "{text}"')
        
        # 检查是否包含关键词
        command = None
        for keyword, cmd in self.command_keywords.items():
            if keyword in text:
                command = cmd
                self.get_logger().info(f'检测到命令: {keyword} -> {cmd}')
                break
        
        if command:
            self.execute_command(command)
        else:
            self.get_logger().info(f'未识别到有效命令，文本: "{text}"')
    
    def execute_command(self, command):
        """执行运动命令"""
        # 更新当前命令状态
        self.current_command = command
        self.last_command_time = time.time()
        
        if command == 'forward':
            # 前进 - 持续前进直到收到停止命令
            self.get_logger().info(f'执行前进命令，持续前进速度: {self.linear_speed} m/s')
            self.get_logger().info('说"停止"来停止前进')
            
        elif command == 'backward':
            # 后退 - 持续后退直到收到停止命令
            self.get_logger().info(f'执行后退命令，持续后退速度: {self.linear_speed} m/s')
            self.get_logger().info('说"停止"来停止后退')
            
        elif command == 'stop':
            # 停止
            self.get_logger().info('执行停止命令，已停止所有运动')
            
        elif command == 'left':
            # 左转 - 持续左转直到收到停止命令
            self.get_logger().info(f'执行左转命令，持续左转角速度: {self.angular_speed} rad/s')
            self.get_logger().info('说"停止"来停止左转')
            
        elif command == 'right':
            # 右转 - 持续右转直到收到停止命令
            self.get_logger().info(f'执行右转命令，持续右转角速度: {self.angular_speed} rad/s')
            self.get_logger().info('说"停止"来停止右转')
        
        # 命令会通过定时器持续发布，避免底盘超时

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SpeechControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行错误: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()