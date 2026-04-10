#!/usr/bin/env python3
"""
简化版中文语音识别节点

使用sounddevice和vosk进行离线中文语音识别
"""

import rclpy
from rclpy.node import Node
import numpy as np
import threading
import queue
import json
import time

try:
    import sounddevice as sd
    SOUNDDEVICE_AVAILABLE = True
except ImportError:
    SOUNDDEVICE_AVAILABLE = False
    print("警告: sounddevice 库未安装")

try:
    import vosk
    VOSK_AVAILABLE = True
except ImportError:
    VOSK_AVAILABLE = False
    print("警告: vosk 库未安装")


class SimpleSpeechRecognitionNode(Node):
    """简化版语音识别节点"""
    
    def __init__(self):
        super().__init__('simple_speech_recognition_node')
        
        # 参数配置
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 4000)
        self.declare_parameter('model_path', '/path/to/vosk-model')
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.model_path = self.get_parameter('model_path').value
        
        # 音频队列
        self.audio_queue = queue.Queue()
        
        # 识别器
        self.model = None
        self.recognizer = None
        
        # 控制标志
        self.is_running = True
        
        # 初始化识别器
        self.init_recognizer()
        
        # 启动音频流
        self.start_audio_stream()
        
        self.get_logger().info('简化版语音识别节点已启动')
    
    def init_recognizer(self):
        """初始化Vosk识别器"""
        if not VOSK_AVAILABLE:
            self.get_logger().error('vosk 库未安装')
            return
        
        try:
            # 加载模型（需要先下载中文模型）
            # 可以从 https://alphacephei.com/vosk/models 下载
            self.model = vosk.Model(self.model_path)
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)
            self.get_logger().info('Vosk识别器初始化成功')
        except Exception as e:
            self.get_logger().error(f'初始化Vosk识别器失败: {e}')
    
    def audio_callback(self, indata, frames, time, status):
        """音频回调函数"""
        if status:
            self.get_logger().warn(f'音频状态: {status}')
        
        # 将音频数据放入队列
        self.audio_queue.put(bytes(indata))
    
    def start_audio_stream(self):
        """启动音频流"""
        if not SOUNDDEVICE_AVAILABLE:
            self.get_logger().error('sounddevice 库未安装')
            return
        
        try:
            # 启动音频输入流
            self.stream = sd.InputStream(
                samplerate=self.sample_rate,
                blocksize=self.chunk_size,
                channels=1,
                dtype='int16',
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info('音频流已启动')
            
            # 启动识别线程
            self.recognition_thread = threading.Thread(target=self.recognition_loop)
            self.recognition_thread.daemon = True
            self.recognition_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'启动音频流失败: {e}')
    
    def recognition_loop(self):
        """识别循环"""
        while self.is_running:
            try:
                # 从队列获取音频数据
                if not self.audio_queue.empty():
                    data = self.audio_queue.get()
                    
                    if self.recognizer.AcceptWaveform(data):
                        # 获取最终识别结果
                        result = json.loads(self.recognizer.Result())
                        text = result.get('text', '')
                        
                        if text.strip():
                            self.get_logger().info(f'识别结果: {text}')
                            print(f"\n=== 语音识别结果 ===")
                            print(f"时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
                            print(f"内容: {text}")
                            print("=" * 30)
                    else:
                        # 获取部分识别结果
                        partial_result = json.loads(self.recognizer.PartialResult())
                        partial_text = partial_result.get('partial', '')
                        
                        if partial_text.strip():
                            print(f"识别中: {partial_text}", end='\r')
                            
            except Exception as e:
                self.get_logger().error(f'识别错误: {e}')
                time.sleep(0.1)
    
    def destroy_node(self):
        """清理资源"""
        self.is_running = False
        
        if hasattr(self, 'stream') and self.stream:
            self.stream.stop()
            self.stream.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 检查必要的库
    if not SOUNDDEVICE_AVAILABLE:
        print("错误: 需要安装 sounddevice 库")
        print("安装命令: pip install sounddevice")
        return
    
    if not VOSK_AVAILABLE:
        print("错误: 需要安装 vosk 库")
        print("安装命令: pip install vosk")
        return
    
    node = SimpleSpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()