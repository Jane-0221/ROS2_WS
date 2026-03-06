#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class BmsHeightControllerNode(Node):
    def __init__(self):
        super().__init__("bms_height_controller_node")
        
        # 订阅BMS数据（电流、电压、SOC）
        self.bms_data_sub = self.create_subscription(
            Float32MultiArray, 
            "bms/battery_data", 
            self.bms_data_callback, 
            10
        )
        
        # 发布高度设置到STM32节点（通过服务或话题）
        # 这里我们创建一个服务客户端来设置STM32的高度
        self.height_setter = self.create_publisher(Float32, "stm32/target_height", 10)
        
        # 初始化变量
        self.current_current = 0.0  # 当前电流值
        self.target_height = 300.0  # 默认高度
        self.last_height = 300.0    # 上次设置的高度（用于避免重复设置）
        
        # 电流阈值配置
        self.current_threshold = -1.5  # 电流阈值：-1.5A
        
        self.get_logger().info("BMS智能高度控制器节点已启动")
        self.get_logger().info(f"电流阈值设置为: {self.current_threshold}A")

    def bms_data_callback(self, msg):
        """BMS数据回调函数 - 根据电流值智能设置高度"""
        if len(msg.data) >= 3:
            # msg.data格式: [总压V, 电流A, SOC%]
            voltage = msg.data[0]
            current = msg.data[1]
            soc = msg.data[2]
            
            self.current_current = current
            
            # 根据电流值智能设置高度
            new_height = self.calculate_target_height(current)
            
            # 只有当高度发生变化时才发布新设置
            if abs(new_height - self.last_height) > 0.1:
                self.target_height = new_height
                self.last_height = new_height
                self.publish_height_setpoint()
                
                # 记录决策信息
                self.get_logger().info(
                    f"电流: {current:.2f}A | "
                    f"阈值: {self.current_threshold}A | "
                    f"设置高度: {new_height}mm"
                )

    def calculate_target_height(self, current):
        """根据电流值计算目标高度"""
        if current > self.current_threshold:
            # 电流大于阈值（放电较小），设置较低高度300mm
            return 300.0
        else:
            # 电流小于等于阈值（放电较大），设置较高高度600mm
            return 600.0

    def publish_height_setpoint(self):
        """发布高度设置点到STM32"""
        height_msg = Float32()
        height_msg.data = self.target_height
        self.height_setter.publish(height_msg)
        self.get_logger().debug(f"发布高度设置点: {self.target_height}mm")

def main(args=None):
    rclpy.init(args=args)
    node = BmsHeightControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()