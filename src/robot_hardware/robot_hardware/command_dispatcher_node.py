#!/usr/bin/env python3
"""
指令分发节点

订阅控制指令，解析并发送到对应的执行节点：
- 气泵控制 → stm32/pump_control
- 底盘控制 → cmd_vel
- 高度控制 → stm32/target_height
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Float32MultiArray
from geometry_msgs.msg import Twist
import json
import math


class CommandDispatcherNode(Node):
    """指令分发节点"""
    
    def __init__(self):
        super().__init__('command_dispatcher_node')
        
        # 创建订阅者 - 订阅控制指令
        self.command_sub = self.create_subscription(
            String,
            '/robot/commands',
            self.command_callback,
            10
        )
        
        # 创建发布者 - 气泵控制
        self.pump_pub = self.create_publisher(
            Bool,
            '/stm32/pump_control',
            10
        )
        
        # 创建发布者 - 底盘控制
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # 创建发布者 - 高度控制
        self.height_pub = self.create_publisher(
            Float32,
            '/stm32/target_height',
            10
        )
        
        # 创建发布者 - 舵机角度控制（弧度制）
        self.servo_pub = self.create_publisher(
            Float32MultiArray,
            '/stm32/servo_control',
            10
        )
        
        # 移动命令定时器
        self.is_moving = False
        self.current_direction = None
        self.linear_speed = 0.2
        self.angular_speed = 0.2
        self.move_timer = self.create_timer(0.1, self.publish_move_command)
        
        self.get_logger().info('指令分发节点已启动')
        self.get_logger().info(f'订阅话题: /robot/commands')
        self.get_logger().info(f'发布话题: /stm32/pump_control, cmd_vel, /stm32/target_height, /stm32/servo_control')
    
    def command_callback(self, msg):
        """处理控制指令"""
        try:
            command = json.loads(msg.data)
            action = command.get('action', '')
            
            self.get_logger().info(f'收到指令: {command}')
            
            if action == 'pump':
                # 气泵控制
                value = command.get('value', 0)
                self.control_pump(value)
                
            elif action == 'move':
                # 移动控制
                direction = command.get('direction', 'stop')
                self.control_move(direction)
                
            elif action == 'height':
                # 高度控制
                value = command.get('value', 0)
                self.control_height(value)
                
            elif action == 'lift':
                # 升降杆控制
                direction = command.get('direction', 'stop')
                self.control_lift(direction)
                
            elif action == 'servo':
                # 舵机角度控制
                servo_id = command.get('id', 0)
                angle_rad = command.get('angle', 0.0)  # 角度（弧度）
                self.get_logger().info(f'收到舵机控制指令: ID={servo_id}, 角度={angle_rad}弧度')
                self.control_servo(servo_id, angle_rad)
                
            else:
                self.get_logger().warning(f'未知指令: {action}')
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析失败: {e}')
        except Exception as e:
            self.get_logger().error(f'指令处理错误: {e}')
    
    def control_pump(self, value):
        """控制气泵"""
        msg = Bool()
        msg.data = bool(value)
        self.pump_pub.publish(msg)
        
        state = "打开" if value else "关闭"
        self.get_logger().info(f'气泵已{state}')
    
    def control_move(self, direction):
        """控制移动"""
        self.current_direction = direction
        self.is_moving = (direction != 'stop')
        
        if direction == 'stop':
            self.get_logger().info('停止移动')
        else:
            self.get_logger().info(f'开始移动: {direction}')
    
    def publish_move_command(self):
        """发布移动命令"""
        if not self.is_moving or not self.current_direction:
            return
        
        twist = Twist()
        
        if self.current_direction == 'forward':
            twist.linear.x = self.linear_speed
        elif self.current_direction == 'backward':
            twist.linear.x = -self.linear_speed
        elif self.current_direction == 'left':
            twist.angular.z = self.angular_speed
        elif self.current_direction == 'right':
            twist.angular.z = -self.angular_speed
        
        self.cmd_vel_pub.publish(twist)
    
    def control_height(self, value):
        """控制高度"""
        msg = Float32()
        msg.data = float(value)
        self.height_pub.publish(msg)
        
        self.get_logger().info(f'设置高度: {value}mm')
    
    def control_lift(self, direction):
        """控制升降杆"""
        # 简单实现：根据方向增加或减少高度
        current_height = 500  # 假设当前高度
        
        if direction == 'up':
            new_height = min(current_height + 50, 1000)
        elif direction == 'down':
            new_height = max(current_height - 50, 0)
        else:
            return
        
        self.control_height(new_height)
        self.get_logger().info(f'升降杆{direction}: {new_height}mm')
    
    def control_servo(self, servo_id, angle_rad):
        """控制舵机角度（弧度制）"""
        # 检查舵机ID范围
        if servo_id < 0 or servo_id >= 12:
            self.get_logger().error(f'舵机ID超出范围: {servo_id}，有效范围: 0-11')
            return
        
        # 发布舵机控制消息
        msg = Float32MultiArray()
        msg.data = [float(servo_id), angle_rad]
        self.servo_pub.publish(msg)
        
        # 弧度转角度用于日志
        angle_deg = math.degrees(angle_rad)
        self.get_logger().info(f'舵机{servo_id}设置角度: {angle_rad:.3f}弧度 ({angle_deg:.1f}°)')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CommandDispatcherNode()
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
