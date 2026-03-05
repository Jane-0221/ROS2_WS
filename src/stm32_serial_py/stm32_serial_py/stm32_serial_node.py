#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import time
from typing import List

# -------------------------- 协议常量定义（与STM32完全一致） --------------------------
FRAME_HEADER = b'\xAA\x55'  # 帧头
FRAME_TAIL = b'\xEE\xFF'    # 帧尾
UP_FRAME_TYPE = 0x01        # 上行帧（STM32→主机）
DN_FRAME_TYPE = 0x02        # 下行帧（主机→STM32）
UP_DATA_LEN = 28            # 上行数据域字节数
DN_DATA_LEN = 27            # 下行数据域字节数
SERIAL_PORT = "/dev/ttyUSB0"# USB转TTL设备名（根据实际修改）
BAUDRATE = 115200           # 波特率

# -------------------------- CRC16-CCITT校验实现 --------------------------
def crc16_ccitt(data: bytes) -> int:
    """
    计算CRC16-CCITT校验值（多项式0x1021，初始值0xFFFF）
    :param data: 待校验的字节数据
    :return: 16位校验值
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
        crc &= 0xFFFF  # 保持16位
    return crc

# -------------------------- 数据结构体定义 --------------------------
class UpData:
    """上行数据（STM32→主机）"""
    def __init__(self):
        self.air_path_state: int = 0    # 气路状态：0=关，1=开
        self.suck_state: int = 0        # 吸附状态：0=未吸住，1=吸住
        self.head_motor_angle: float = 0.0  # 头部电机角度（°）
        self.arm_servo_angles: List[float] = [0.0]*11  # 11路机械臂/舵机角度（°）
        self.lift_height: float = 0.0   # 升降杆高度（mm）

class DnData:
    """下行数据（主机→STM32）"""
    def __init__(self):
        self.target_angles: List[float] = [0.0]*12  # 12路电机/舵机目标角度（°）
        self.pump_state: int = 0        # 气泵状态：0=关，1=开
        self.target_lift_height: float = 0.0  # 升降杆目标高度（mm）

# -------------------------- ROS2串口节点 --------------------------
class STM32SerialNode(Node):
    def __init__(self):
        super().__init__("stm32_serial_node")
        
        # 1. 初始化串口
        self.ser = None
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                timeout=0.01,  # 非阻塞读取
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f"串口 {SERIAL_PORT} 打开成功！")
        except Exception as e:
            self.get_logger().error(f"串口打开失败：{str(e)}")
            return
        
        # 2. 初始化数据缓存
        self.up_data = UpData()
        self.dn_data = DnData()
        # 示例：设置默认下行指令
        self.dn_data.target_angles[0] = 45.0  # 第1路电机目标45°
        self.dn_data.pump_state = 1           # 气泵开启
        self.dn_data.target_lift_height = 333.0  # 升降杆333mm
      
        # 3. 创建定时器：定时发送下行帧（100ms）
        self.send_timer = self.create_timer(0.1, self.send_down_frame)
        # 4. 创建定时器：定时接收上行帧（10ms）
        self.recv_timer = self.create_timer(0.01, self.recv_up_frame)

    def pack_down_frame(self) -> bytes:
        """打包下行帧（主机→STM32）"""
        # 初始化帧缓冲区
        frame = bytearray()
        # 1. 帧头
        frame.extend(FRAME_HEADER)
        # 2. 帧类型
        frame.append(DN_FRAME_TYPE)
        # 3. 数据长度
        frame.append(DN_DATA_LEN)
        
        # 4. 数据域：转换为协议规定的格式（0.1°/0.1mm，小端）
        # 12路目标角度（转int16，0.1°单位）
        for angle in self.dn_data.target_angles:
            angle_int = int(angle * 10)
            frame.extend(struct.pack('<h', angle_int))  # <h：小端int16
        # 气泵状态
        frame.append(self.dn_data.pump_state)
        # 升降杆目标高度（转uint16，0.1mm单位）
        height_int = int(self.dn_data.target_lift_height * 10)
        frame.extend(struct.pack('<H', height_int))  # <H：小端uint16
        
        # 5. CRC16校验（校验范围：帧类型+数据长度+数据域）
        crc_data = frame[2:]  # 跳过帧头，取后续数据
        crc = crc16_ccitt(crc_data)
        frame.extend(struct.pack('<H', crc))  # 小端存储CRC
        
        # 6. 帧尾
        frame.extend(FRAME_TAIL)
        
        return bytes(frame)

    def send_down_frame(self):
        """发送下行帧"""
        if not self.ser or not self.ser.is_open:
            return
        try:
            frame = self.pack_down_frame()
            self.ser.write(frame)
            # self.get_logger().debug(f"发送下行帧：{frame.hex()}")
        except Exception as e:
            self.get_logger().error(f"发送帧失败：{str(e)}")

    def unpack_up_frame(self, frame: bytes) -> UpData:
        """解析上行帧（STM32→主机）"""
        up_data = UpData()
        # 跳过帧头(2)+帧类型(1)+数据长度(1)，从第4字节开始解析
        offset = 4
        
        # 气路状态
        up_data.air_path_state = frame[offset]
        offset += 1
        # 吸附状态
        up_data.suck_state = frame[offset]
        offset += 1
        # 头部电机角度（int16，0.1°→转°）
        head_angle_int = struct.unpack('<h', frame[offset:offset+2])[0]
        up_data.head_motor_angle = head_angle_int / 10.0
        offset += 2
        # 11路机械臂/舵机角度
        for i in range(11):
            angle_int = struct.unpack('<h', frame[offset:offset+2])[0]
            up_data.arm_servo_angles[i] = angle_int / 10.0
            offset += 2
        # 升降杆高度（uint16，0.1mm→转mm）
        height_int = struct.unpack('<H', frame[offset:offset+2])[0]
        up_data.lift_height = height_int / 10.0
        
        return up_data

    def recv_up_frame(self):
        """接收并解析上行帧"""
        if not self.ser or not self.ser.is_open:
            return
        try:
            # 读取串口所有可用数据
            recv_data = self.ser.read(self.ser.in_waiting)
            if len(recv_data) < 34:  # 上行帧总长度34字节（2+1+1+28+2+2）
                return
            
            # 查找帧头位置（处理粘包）
            header_idx = recv_data.find(FRAME_HEADER)
            if header_idx == -1:
                return
            # 截取完整帧（从帧头开始取34字节）
            frame = recv_data[header_idx:header_idx+34]
            if len(frame) != 34:
                return
            
            # 校验帧尾
            if frame[-2:] != FRAME_TAIL:
                self.get_logger().warn("帧尾校验失败")
                return
            
            # 校验CRC16
            crc_data = frame[2:-4]  # 帧类型+数据长度+数据域（跳过帧头、CRC、帧尾）
            crc_calc = crc16_ccitt(crc_data)
            crc_recv = struct.unpack('<H', frame[-4:-2])[0]  # 提取接收的CRC
            if crc_calc != crc_recv:
                self.get_logger().warn(f"CRC校验失败：计算值={crc_calc:04X}，接收值={crc_recv:04X}")
                return
            
            # 解析数据
            self.up_data = self.unpack_up_frame(frame)
            # 打印解析结果（可替换为ROS2话题发布）
            self.get_logger().info(
                f"气路状态：{self.up_data.air_path_state} | 吸附状态：{self.up_data.suck_state} | "
                f"头部角度：{self.up_data.head_motor_angle:.1f}° | 升降高度：{self.up_data.lift_height:.1f}mm"
            )
        except Exception as e:
            self.get_logger().error(f"接收/解析帧失败：{str(e)}")

    def destroy_node(self):
        """节点销毁时关闭串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = STM32SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()