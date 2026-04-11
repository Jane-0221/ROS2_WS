#!/usr/bin/env python3
"""
Wheeltec底盘控制节点
基于wheeltec_normalized.py的归一化速度控制器
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool
import threading
import time

# 导入底盘控制库
try:
    from wheeltec_normalized import MethodC_Controller, ControlProtocol
    from wheeltec_protocol import WheeltecBaseProtocol
except ImportError:
    # 如果在ROS2环境中无法直接导入，使用相对导入
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from wheeltec_normalized import MethodC_Controller, ControlProtocol
    from wheeltec_protocol import WheeltecBaseProtocol


class WheeltecChassisNode(Node):
    """Wheeltec底盘控制ROS2节点"""
    
    def __init__(self):
        super().__init__('wheeltec_chassis_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('protocol', 'serial'),  # 通信协议: serial/can
                ('robot_type', 'diff'),  # 底盘类型: diff/ackermann/omni
                ('cmd_vel_topic', 'cmd_vel'),
                ('port', '/dev/ttyACM0'),  # 串口设备
                ('baudrate', 115200),     # 波特率
                ('max_vx', 500.0),        # 最大线速度 mm/s
                ('max_vy', 500.0),        # 最大侧向速度 mm/s
                ('max_vz', 1.0),          # 最大角速度 rad/s
                ('publish_odom', False),   # 是否发布里程计
                ('publish_battery', True), # 是否发布电池状态
                ('cmd_timeout', 0.5),     # 命令超时时间(秒)
                ('linear_x_sign', -1.0),
                ('linear_y_sign', 1.0),
                ('angular_z_sign', 1.0),
            ]
        )
        
        # 获取参数
        protocol_str = self.get_parameter('protocol').value
        robot_type = self.get_parameter('robot_type').value
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        max_speeds = (
            self.get_parameter('max_vx').value,
            self.get_parameter('max_vy').value,
            self.get_parameter('max_vz').value
        )
        
        # 转换协议类型
        protocol = ControlProtocol.SERIAL if protocol_str == 'serial' else ControlProtocol.CAN
        
        # 初始化底盘控制器
        connection_params = {
            'port': port,
            'baudrate': baudrate
        }
        
        self.controller = MethodC_Controller(
            protocol=protocol,
            robot_type=robot_type,
            max_speeds=max_speeds,
            **connection_params
        )
        
        # 创建订阅者
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self.cmd_vel_callback,
            10
        )
        
        # 创建发布者
        if self.get_parameter('publish_odom').value:
            self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        if self.get_parameter('publish_battery').value:
            self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        
        # 创建服务
        self.enable_service = self.create_service(
            SetBool,
            'enable_motors',
            self.enable_motors_callback
        )
        
        # 控制状态
        self.motors_enabled = True  # 默认使能电机
        self.last_cmd_time = None
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        
        # 启动定时器
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.get_logger().info('Wheeltec底盘节点初始化完成')
        
        # 连接底盘
        if self.controller.connect():
            self.get_logger().info('底盘连接成功')
        else:
            self.get_logger().error('底盘连接失败')
    
    def cmd_vel_callback(self, msg):
        """处理速度命令"""
        if not self.motors_enabled:
            self.get_logger().warn('电机未使能，忽略速度命令')
            return
        
        # 更新最后命令时间
        self.last_cmd_time = self.get_clock().now()
        
        linear_x_sign = float(self.get_parameter('linear_x_sign').value)
        linear_y_sign = float(self.get_parameter('linear_y_sign').value)
        angular_z_sign = float(self.get_parameter('angular_z_sign').value)

        # 将Twist消息转换为归一化速度
        # 注意：ROS2的Twist使用m/s和rad/s，需要转换为mm/s
        kx = linear_x_sign * msg.linear.x * 1000.0 / self.controller.max_speeds[0]
        ky = linear_y_sign * msg.linear.y * 1000.0 / self.controller.max_speeds[1]
        kz = angular_z_sign * msg.angular.z / self.controller.max_speeds[2]

        # 限制在[-1, 1]范围内
        kx = max(-1.0, min(1.0, kx))
        ky = max(-1.0, min(1.0, ky))
        kz = max(-1.0, min(1.0, kz))
        
        # 执行动作
        self.controller.execute_action(kx, ky, kz)
        
        self.get_logger().debug(f'执行速度命令: kx={kx:.3f}, ky={ky:.3f}, kz={kz:.3f}')
    
    def enable_motors_callback(self, request, response):
        """处理电机使能服务请求"""
        self.motors_enabled = request.data
        
        if self.motors_enabled:
            self.get_logger().info('电机已使能')
            response.success = True
            response.message = '电机使能成功'
        else:
            # 停止底盘
            self.controller.stop()
            self.get_logger().info('电机已禁用')
            response.success = True
            response.message = '电机禁用成功'
        
        return response
    
    def timer_callback(self):
        """定时器回调函数"""
        # 检查命令超时
        if (self.last_cmd_time is not None and 
            self.motors_enabled and
            (self.get_clock().now() - self.last_cmd_time).nanoseconds > self.cmd_timeout * 1e9):
            self.controller.stop()
            self.last_cmd_time = None
            self.get_logger().warn('速度命令超时，已停止底盘')
        
        # 发布里程计数据（需要从底盘读取）
        if hasattr(self, 'odom_pub'):
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # 这里需要根据实际硬件实现里程计读取
            # 暂时使用零值
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.w = 1.0
            
            self.odom_pub.publish(odom_msg)
        
        # 发布电池状态（需要从底盘读取）
        if hasattr(self, 'battery_pub'):
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.voltage = 12.0  # 默认值，需要从硬件读取
            battery_msg.percentage = 0.8  # 默认值，需要从硬件读取
            
            self.battery_pub.publish(battery_msg)
    
    def destroy_node(self):
        """节点销毁时停止底盘"""
        self.controller.stop()
        self.controller.base_protocol.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = WheeltecChassisNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
