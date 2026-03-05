#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import Float32, Float32MultiArray

# -------------------------- 达锂BMS RS485协议常量（V1.2） --------------------------
# 串口配置
BATTERY_PORT = "/dev/ttyBattery"   # USB转RS485设备名，根据实际修改（如/dev/ttyACM0）
BAUDRATE = 9600               # 协议固定9600波特率
BYTESIZE = serial.EIGHTBITS   # 8位数据位
STOPBITS = serial.STOPBITS_ONE# 1位停止位
PARITY = serial.PARITY_NONE   # 无校验

# 协议帧格式定义
FRAME_HEADER = 0xA5           # 固定帧头
PC_ADDR = 0x40                # 上位机（PC/ROS）通信地址
BMS_ADDR = 0x01               # BMS从机地址
DATA_ID_SOC = 0x90            # 总压/电流/SOC 数据ID（核心）
DATA_LEN_FIX = 0x08           # 协议固定数据长度8字节
FRAME_SEND_LEN = 13           # 上位机发送帧总长度：1(头)+1(地址)+1(ID)+1(长度)+8(数据)+1(校验)
FRAME_RECV_LEN = 13           # BMS响应帧总长度：与发送帧一致

# 数据解析系数
VOLT_COEFF = 0.1              # 总压系数：0.1V/单位
CURR_COEFF = 0.1              # 电流系数：0.1A/单位
CURR_OFFSET = 30000           # 电流偏移量：协议固定30000
SOC_COEFF = 0.1               # SOC系数：0.1%/单位

class BmsSocReaderNode(Node):
    def __init__(self):
        super().__init__("bms_soc_reader_node")
        
        # 1. 初始化串口（RS485）
        self.ser = None
        try:
            self.ser = serial.Serial(
                port=BATTERY_PORT,
                baudrate=BAUDRATE,
                bytesize=BYTESIZE,
                parity=PARITY,
                stopbits=STOPBITS,
                timeout=0.1,  # 读取超时0.1s，避免阻塞
                write_timeout=0.1
            )
            self.get_logger().info(f"RS485串口 {BATTERY_PORT} 打开成功！波特率：{BAUDRATE}")
        except Exception as e:
            self.get_logger().error(f"RS485串口打开失败：{str(e)}")
            return
        
        # 2. ROS2话题发布：核心发布SOC，附带发布总压/电流（多数据）
        self.soc_pub = self.create_publisher(Float32, "bms/battery_soc", 10)  # 电池SOC(%)
        self.bms_data_pub = self.create_publisher(Float32MultiArray, "bms/battery_data", 10)  # [总压V, 电流A, SOC%]
        
        # 3. 定时查询BMS数据：1秒1次（可根据需求调整，建议≥500ms）
        self.query_timer = self.create_timer(1.0, self.bms_query_task)
        
        # 初始化数据缓存
        self.battery_soc = 0.0    # 电池剩余电量(%)
        self.total_volt = 0.0     # 电池总压(V)
        self.battery_curr = 0.0   # 电池电流(A)：充电为正，放电为负

    # 校验和计算：协议规定-前面所有数据之和，只取低8字节（模256）
    def calc_checksum(self, data_list: list) -> int:
        sum_val = sum(data_list)
        return sum_val & 0xFF  # 取低8位

    # 打包并发送BMS查询指令（读取SOC/总压/电流）
    def send_bms_query_cmd(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            # 构造发送帧：帧头+上位机地址+数据ID+数据长度+8字节预留数据(全0)+校验和
            send_frame = [0x00] * FRAME_SEND_LEN
            send_frame[0] = FRAME_HEADER    # 帧头0xA5
            send_frame[1] = PC_ADDR         # 上位机地址0x40
            send_frame[2] = DATA_ID_SOC     # 读取SOC的Data ID 0x90
            send_frame[3] = DATA_LEN_FIX    # 数据长度0x08
            send_frame[4:12] = [0x00]*8     # 8字节预留数据，协议要求全0
            # 计算校验和：帧头到预留数据的所有字节
            checksum = self.calc_checksum(send_frame[0:12])
            send_frame[12] = checksum       # 最后1字节为校验和

            # 发送帧（转字节串）
            self.ser.write(bytes(send_frame))
            # self.get_logger().debug(f"发送BMS查询帧：{bytes(send_frame).hex(' ')}")
        except Exception as e:
            self.get_logger().error(f"发送BMS指令失败：{str(e)}")

    # 接收并解析BMS响应帧
    def recv_and_parse_bms_frame(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            # 读取串口所有可用数据
            recv_data = self.ser.read(self.ser.in_waiting)
            if len(recv_data) < FRAME_RECV_LEN:
                return  # 数据不足，直接返回

            # 查找帧头（处理粘包，从帧头开始截取完整帧）
            header_idx = recv_data.find(bytes([FRAME_HEADER]))
            if header_idx == -1:
                self.get_logger().warn("未找到BMS响应帧头，丢弃数据")
                return
            # 截取完整的13字节响应帧
            recv_frame = recv_data[header_idx:header_idx+FRAME_RECV_LEN]
            if len(recv_frame) != FRAME_RECV_LEN:
                return

            # 帧合法性校验1：地址+数据ID校验（必须是BMS地址+SOC的Data ID）
            if recv_frame[1] != BMS_ADDR or recv_frame[2] != DATA_ID_SOC:
                self.get_logger().warn("BMS地址/数据ID不匹配，丢弃帧")
                return
            # 帧合法性校验2：校验和校验
            recv_checksum = recv_frame[-1]
            calc_checksum = self.calc_checksum(recv_frame[0:-1])
            if recv_checksum != calc_checksum:
                self.get_logger().warn(f"BMS校验和失败：接收{recv_checksum:02X}，计算{calc_checksum:02X}")
                return

            # 解析核心数据：协议规定高位在前（大端模式）
            frame_data = recv_frame[4:12]  # 提取8字节数据内容
            # 1. 电池总压：Byte0~Byte1 拼接，0.1V/单位
            self.total_volt = struct.unpack('>H', frame_data[0:2])[0] * VOLT_COEFF
            # 2. 电池电流：Byte4~Byte5 拼接，30000偏移，0.1A/单位，充电正/放电负
            curr_raw = struct.unpack('>H', frame_data[4:6])[0]
            self.battery_curr = (curr_raw - CURR_OFFSET) * CURR_COEFF
            # 3. 电池SOC：Byte6~Byte7 拼接，0.1%/单位（核心读取目标）
            self.battery_soc = struct.unpack('>H', frame_data[6:8])[0] * SOC_COEFF

            # 数据范围过滤（防止异常值）
            self.battery_soc = max(0.0, min(100.0, self.battery_soc))  # SOC限定0~100%
            self.get_logger().info(f"BMS数据解析成功 | 总压：{self.total_volt:.1f}V | 电流：{self.battery_curr:.1f}A | SOC：{self.battery_soc:.1f}%")

        except Exception as e:
            self.get_logger().error(f"接收/解析BMS帧失败：{str(e)}")

    # 定时任务：发送查询指令 → 接收解析数据 → 发布ROS2话题
    def bms_query_task(self):
        self.send_bms_query_cmd()    # 发送查询指令
        self.recv_and_parse_bms_frame()  # 接收并解析响应
        self.publish_bms_data()      # 发布解析后的数据

    # 发布ROS2话题：单独发布SOC + 批量发布总压/电流/SOC
    def publish_bms_data(self):
        # 发布SOC（Float32）
        soc_msg = Float32()
        soc_msg.data = self.battery_soc
        self.soc_pub.publish(soc_msg)

        # 发布总压/电流/SOC（Float32MultiArray）
        bms_data_msg = Float32MultiArray()
        bms_data_msg.data = [self.total_volt, self.battery_curr, self.battery_soc]
        self.bms_data_pub.publish(bms_data_msg)

    # 节点销毁：关闭RS485串口
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("RS485串口已关闭")
        super().destroy_node()

# 主函数
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