#!/usr/bin/env python3
"""
WHEELTEC机器人控制协议实现
基于《STM32运动底盘开发手册》V1.0
- 第5.1节：串口控制协议
- 第5.2节：CAN控制协议
"""

import serial
import can
import struct
import time
import logging
from typing import Optional, Tuple
from enum import Enum

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("WheeltecProtocol")


INT16_MIN = -32768
INT16_MAX = 32767

class ControlProtocol(Enum):
    """控制协议枚举"""
    SERIAL = "serial"  # 串口协议 (文档5.1节)
    CAN = "can"        # CAN协议 (文档5.2节)

class WheeltecBaseProtocol:
    """
    WHEELTEC底盘基础协议层
    实现文档中定义的串口/CAN通信协议
    """
    
    def __init__(
        self,
        protocol: ControlProtocol = ControlProtocol.SERIAL,
        robot_type: str = "diff",
        **connection_params
    ):
        """
        初始化基础协议层
        
        Args:
            protocol: 控制协议 (SERIAL 或 CAN)
            robot_type: 底盘类型 ('diff':差速, 'ackermann':阿克曼, 'omni':全向)
            **connection_params: 连接参数
                - 串口: port, baudrate=115200, timeout=1.0
                - CAN: interface='pcan', channel=1, bitrate=1000000
        """
        self.protocol = protocol
        self.robot_type = robot_type
        self.connection_params = connection_params
        
        # 通信对象
        self.serial_port = None
        self.can_bus = None
        
        # 协议常量
        self.SERIAL_FRAME_HEADER = 0x7B
        self.SERIAL_FRAME_FOOTER = 0x7D
        self.SERIAL_RESERVED = 0x00
        self.CAN_FRAME_ID = 0x181
        
        logger.info(f"初始化基础协议: {protocol.value}协议, {robot_type}底盘")
    
    def connect(self) -> bool:
        """建立通信连接"""
        try:
            if self.protocol == ControlProtocol.SERIAL:
                return self._connect_serial()
            elif self.protocol == ControlProtocol.CAN:
                return self._connect_can()
            else:
                logger.error(f"不支持的控制协议: {self.protocol}")
                return False
        except Exception as e:
            logger.error(f"连接失败: {e}")
            return False
    
    def _connect_serial(self) -> bool:
        """连接串口 (文档5.1节: 115200, 8N1)"""
        port = self.connection_params.get('port', 'COM3')
        baudrate = self.connection_params.get('baudrate', 115200)
        timeout = self.connection_params.get('timeout', 1.0)
        
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            
            if self.serial_port.is_open:
                logger.info(f"串口连接成功: {port}, 波特率{baudrate}")
                return True
            return False
            
        except Exception as e:
            logger.error(f"串口连接失败: {e}")
            return False
    
    def _connect_can(self) -> bool:
        """连接CAN总线 (文档5.2节: 1Mbps)"""
        interface = self.connection_params.get('interface', 'pcan')
        channel = self.connection_params.get('channel', 1)
        bitrate = self.connection_params.get('bitrate', 1000000)
        
        try:
            # 处理PCAN的特殊通道格式
            if interface == 'pcan':
                channel = f'PCAN_USBBUS{channel}' if channel > 0 else 'PCAN_USBBUS1'
            
            self.can_bus = can.interface.Bus(
                interface=interface,
                channel=channel,
                bitrate=bitrate
            )
            
            logger.info(f"CAN连接成功: {interface}, 通道{channel}, 波特率{bitrate}")
            return True
            
        except Exception as e:
            logger.error(f"CAN连接失败: {e}")
            if "PCANBasic" in str(e):
                logger.error("请安装PCAN-Basic驱动: https://www.peak-system.com/PCAN-Basic.239.0.html")
            return False
    
    def _pack_serial_data(self, vx_mmps: float, vy_mmps: float, vz_radps: float) -> bytes:
        """
        打包串口数据帧 (文档5.1节表5-1)
        11字节格式: [0x7B, 预留, 预留, vx_high, vx_low, vy_high, vy_low, vz_high, vz_low, 校验, 0x7D]
        Z轴角速度放大1000倍
        """
        # 转换为整数
        vx_int = self._saturate_int16(vx_mmps, "vx_mmps")
        vy_int = self._saturate_int16(vy_mmps, "vy_mmps")
        vz_int = self._saturate_int16(vz_radps * 1000, "vz_x1000")  # 关键: 放大1000倍
        
        # 构建前9字节
        first_9_bytes = struct.pack(
            '>BBBhhh',
            self.SERIAL_FRAME_HEADER,
            self.SERIAL_RESERVED,
            self.SERIAL_RESERVED,
            vx_int,
            vy_int,
            vz_int
        )
        
        # 计算BCC校验 (前9字节异或)
        bcc = 0
        for i in range(9):
            bcc ^= first_9_bytes[i]
        
        # 完整11字节帧
        full_frame = first_9_bytes + struct.pack('BB', bcc, self.SERIAL_FRAME_FOOTER)
        
        return full_frame
    
    def _pack_can_data(self, vx_mmps: float, vy_mmps: float, vz_radps: float) -> bytes:
        """
        打包CAN数据帧 (文档5.2节表5-2)
        8字节格式: [vx_high, vx_low, vy_high, vy_low, vz_high, vz_low, 预留, 预留]
        Z轴角速度放大1000倍
        """
        # 转换为整数
        vx_int = self._saturate_int16(vx_mmps, "vx_mmps")
        vy_int = self._saturate_int16(vy_mmps, "vy_mmps")
        vz_int = self._saturate_int16(vz_radps * 1000, "vz_x1000")  # 关键: 放大1000倍
        
        # 8字节数据帧
        can_data = struct.pack('>hhhBB', vx_int, vy_int, vz_int, 0x00, 0x00)
        
        return can_data
    
    def send_velocity(self, vx_mmps: float, vy_mmps: float, vz_radps: float) -> bool:
        """
        发送三轴速度指令 (核心方法)
        
        Args:
            vx_mmps: X轴速度 (mm/s)
            vy_mmps: Y轴速度 (mm/s)
            vz_radps: Z轴角速度 (rad/s)
            
        Returns:
            bool: 发送成功返回True
        """
        # 根据底盘类型调整 (文档注意事项)
        if self.robot_type in ['diff', 'ackermann']:
            vy_mmps = 0.0  # 差速/阿克曼底盘不支持Y轴移动
        
        try:
            if self.protocol == ControlProtocol.SERIAL:
                if not self.serial_port or not self.serial_port.is_open:
                    logger.error("串口未连接")
                    return False
                
                data = self._pack_serial_data(vx_mmps, vy_mmps, vz_radps)
                bytes_written = self.serial_port.write(data)
                
                if bytes_written == len(data):
                    logger.debug(f"串口发送: vx={vx_mmps:.1f}, vy={vy_mmps:.1f}, vz={vz_radps:.3f}")
                    return True
                return False
                
            elif self.protocol == ControlProtocol.CAN:
                if not self.can_bus:
                    logger.error("CAN总线未连接")
                    return False
                
                data = self._pack_can_data(vx_mmps, vy_mmps, vz_radps)
                msg = can.Message(
                    arbitration_id=self.CAN_FRAME_ID,
                    data=data,
                    is_extended_id=False
                )
                
                self.can_bus.send(msg)
                logger.debug(f"CAN发送: vx={vx_mmps:.1f}, vy={vy_mmps:.1f}, vz={vz_radps:.3f}")
                return True
                
        except Exception as e:
            logger.error(f"发送速度指令失败: {e}")
            return False
        
        return False
    
    def stop(self) -> bool:
        """发送停止指令"""
        return self.send_velocity(0, 0, 0)

    @staticmethod
    def _saturate_int16(value: float, field_name: str) -> int:
        int_value = int(value)
        if int_value < INT16_MIN:
            logger.warning(f"{field_name}={int_value} below int16 range, saturating to {INT16_MIN}")
            return INT16_MIN
        if int_value > INT16_MAX:
            logger.warning(f"{field_name}={int_value} above int16 range, saturating to {INT16_MAX}")
            return INT16_MAX
        return int_value
    
    def close(self):
        """关闭连接"""
        if self.protocol == ControlProtocol.SERIAL and self.serial_port:
            self.serial_port.close()
            logger.info("串口连接已关闭")
        elif self.protocol == ControlProtocol.CAN and self.can_bus:
            self.can_bus.shutdown()
            logger.info("CAN连接已关闭")
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
