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
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist
import json


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
        
        # 移动命令定时器
        self.is_moving = False
        self.current_direction = None
        self.linear_speed = 0.2
        self.angular_speed = 0.2
        self.move_timer = self.create_timer(0.1, self.publish_move_command)
        
        self.get_logger().info('指令分发节点已启动')
        self.get_logger().info(f'订阅话题: /robot/commands')
        self.get_logger().info(f'发布话题: /stm32/pump_control, cmd_vel, /stm32/target_height')
    
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
