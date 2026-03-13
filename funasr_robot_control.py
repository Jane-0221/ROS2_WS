#!/usr/bin/env python3
"""
FunASR语音控制机器人 - 最终版本

基于已验证成功的FunASR版本，集成机器人控制功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pyaudio
import wave
import tempfile
import os
import time
import numpy as np

try:
    from funasr import AutoModel
    FUNASR_AVAILABLE = True
except ImportError:
    FUNASR_AVAILABLE = False

class FunASRRobotControl(Node):
    """FunASR语音控制机器人"""
    
    def __init__(self):
        super().__init__('funasr_robot_control')
        
        # 创建速度发布者
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # FunASR模型
        self.model = None
        
        # 控制参数
        self.device_index = 7  # USB设备索引
        self.sample_rate = 48000  # 已验证的采样率
        self.recording_duration = 3  # 录音时长
        
        # 初始化
        self.init_funasr()
        
        if self.model:
            self.start_voice_control()
            print("✅ FunASR语音控制机器人启动成功")
    
    def init_funasr(self):
        """初始化FunASR"""
        if not FUNASR_AVAILABLE:
            print("❌ FunASR未安装")
            return
        
        try:
            self.model = AutoModel(
                model="iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch",
                model_revision="v2.0.4"
            )
            print("✅ FunASR模型加载成功")
        except Exception as e:
            print(f"❌ FunASR初始化失败: {e}")
    
    # 其他方法保持不变...
    # [录音、识别、控制等方法]

def main():
    rclpy.init()
    node = FunASRRobotControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("👋 程序已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
