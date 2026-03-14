#!/usr/bin/env python3
"""
千问实时语音识别节点

使用qwen3-asr-flash-realtime进行实时语音识别
将识别结果发布到 /speech/text 话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import json
import time
import pyaudio
import numpy as np
import threading
import os

try:
    import dashscope
    from dashscope.audio.qwen_omni import OmniRealtimeConversation, OmniRealtimeCallback, TranscriptionParams, MultiModality
    DASHSCOPE_AVAILABLE = True
except ImportError:
    DASHSCOPE_AVAILABLE = False
    print("❌ dashscope未安装，请运行: pip install dashscope")


class QwenASRCallback(OmniRealtimeCallback):
    """ASR回调处理类"""
    
    def __init__(self, node):
        self.node = node
        self.conversation = None
        self.final_text = ""
        self.text_lock = threading.Lock()
    
    def set_conversation(self, conv):
        self.conversation = conv
    
    def on_open(self):
        self.node.get_logger().info('千问ASR连接已打开')
    
    def on_close(self, code, msg):
        self.node.get_logger().info(f'千问ASR连接已关闭: code={code}, msg={msg}')
    
    def on_event(self, response):
        try:
            response_type = response.get('type', '')
            
            if response_type == 'session.created':
                session_id = response.get('session', {}).get('id', 'unknown')
                self.node.get_logger().info(f'ASR会话创建成功: {session_id}')
            
            # 最终识别结果
            elif response_type == 'conversation.item.input_audio_transcription.completed':
                transcript = response.get('transcript', '')
                if transcript:
                    with self.text_lock:
                        self.final_text = transcript
                    self.node.get_logger().info(f'识别到文本: {transcript}')
                    # 发布识别结果
                    self.node.publish_text(transcript)
            
            # 实时中间结果
            elif response_type == 'conversation.item.input_audio_transcription.text':
                stash_text = response.get('stash', '')
                if stash_text:
                    self.node.get_logger().debug(f'中间识别: {stash_text}')
            
            # 语音开始/结束
            elif response_type == 'input_audio_buffer.speech_started':
                self.node.get_logger().debug('检测到语音开始')
            
            elif response_type == 'input_audio_buffer.speech_stopped':
                self.node.get_logger().debug('检测到语音结束')
        
        except Exception as e:
            self.node.get_logger().error(f'ASR回调处理错误: {e}')


class QwenASRNode(Node):
    """千问实时语音识别节点"""
    
    def __init__(self):
        super().__init__('qwen_asr_node')
        
        # 参数配置
        self.declare_parameter('api_key', os.environ.get('DASHSCOPE_API_KEY', 'sk-2cab9b4a77914400b0f504817b8fc0ae'))
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device_index', 0)
        
        self.api_key = self.get_parameter('api_key').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.device_index = self.get_parameter('device_index').value
        
        # 设置API Key
        if self.api_key:
            dashscope.api_key = self.api_key
        
        # 创建发布者
        self.speech_publisher = self.create_publisher(
            String,
            '/speech/text',
            10
        )
        
        # 音频录制相关 - 必须在return之前初始化
        self.is_recording = False
        self.audio_thread = None
        self.p = None
        
        # 初始化ASR
        self.conversation = None  # 确保在return之前初始化
        self.callback = None
        
        if not DASHSCOPE_AVAILABLE:
            self.get_logger().error('dashscope未安装，节点将退出')
            return
        
        # 创建回调
        self.callback = QwenASRCallback(self)
        
        # 创建对话会话
        try:
            self.conversation = OmniRealtimeConversation(
                model='qwen3-asr-flash-realtime',
                url='wss://dashscope.aliyuncs.com/api-ws/v1/realtime',
                callback=self.callback
            )
            self.callback.set_conversation(self.conversation)
            self.get_logger().info('千问ASR会话创建成功')
        except Exception as e:
            self.get_logger().error(f'创建ASR会话失败: {e}')
            return
        
        # 检测音频设备
        self.detect_audio_device()
        
        # 启动识别
        self.start_asr()
        
        self.get_logger().info('千问语音识别节点已启动')
        self.get_logger().info(f'发布话题: /speech/text')
    
    def detect_audio_device(self):
        """检测音频设备"""
        self.p = pyaudio.PyAudio()
        
        try:
            self.get_logger().info('正在检测音频设备...')
            
            # 查找USB音频设备
            usb_device_index = None
            for i in range(self.p.get_device_count()):
                device_info = self.p.get_device_info_by_index(i)
                if device_info['maxInputChannels'] > 0:
                    self.get_logger().info(
                        f'设备 [{i}]: {device_info["name"]} - '
                        f'采样率: {device_info.get("defaultSampleRate", "未知")}Hz'
                    )
                    if "USB" in device_info['name']:
                        usb_device_index = i
                        self.get_logger().info(f'找到USB设备: 索引 {i}')
            
            # 使用USB设备或默认设备
            self.device_index = usb_device_index if usb_device_index is not None else 0
            
            # 获取设备支持的采样率
            device_info = self.p.get_device_info_by_index(self.device_index)
            self.sample_rate = int(device_info.get('defaultSampleRate', 16000))
            
            self.get_logger().info(f'使用设备: 索引 {self.device_index}, 采样率: {self.sample_rate}Hz')
            
        except Exception as e:
            self.get_logger().error(f'音频设备检测失败: {e}')
        finally:
            self.p.terminate()
    
    def start_asr(self):
        """启动ASR识别"""
        try:
            # 连接
            self.conversation.connect()
            self.get_logger().info('ASR连接中...')
            
            # 配置会话
            transcription_params = TranscriptionParams(
                language='zh',
                sample_rate=self.sample_rate,
                input_audio_format='pcm'
            )
            
            self.conversation.update_session(
                output_modalities=[MultiModality.TEXT],
                enable_turn_detection=True,
                turn_detection_type='server_vad',
                turn_detection_threshold=0.0,
                turn_detection_silence_duration_ms=400,
                enable_input_audio_transcription=True,
                transcription_params=transcription_params
            )
            
            self.get_logger().info('ASR会话配置完成，开始录音...')
            
            # 启动录音线程
            self.is_recording = True
            self.audio_thread = threading.Thread(target=self.record_and_send_audio)
            self.audio_thread.daemon = True
            self.audio_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'启动ASR失败: {e}')
    
    def record_and_send_audio(self):
        """录制音频并发送到ASR"""
        p = pyaudio.PyAudio()
        
        try:
            # 打开音频流
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=1024,
                input_device_index=self.device_index
            )
            
            self.get_logger().info('开始录音并识别...')
            
            while self.is_recording:
                try:
                    # 录制音频 (每次100ms)
                    audio_data = stream.read(1024)
                    
                    # 转换为base64并发送
                    audio_b64 = base64.b64encode(audio_data).decode('utf-8')
                    self.conversation.append_audio(audio_b64)
                    
                except Exception as e:
                    self.get_logger().error(f'录音/发送失败: {e}')
                    break
            
        except Exception as e:
            self.get_logger().error(f'录音线程错误: {e}')
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()
    
    def publish_text(self, text):
        """发布识别文本"""
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)
        
        # 打印识别结果
        print("\n" + "=" * 60)
        print(f"🎯 千问ASR识别结果")
        print(f"⏰ 时间: {time.strftime('%H:%M:%S')}")
        print(f"📝 内容: {text}")
        print("=" * 60)
    
    def destroy_node(self):
        """销毁节点"""
        self.is_recording = False
        if self.audio_thread:
            self.audio_thread.join(timeout=2)
        if self.conversation:
            try:
                self.conversation.close()
            except:
                pass
        self.get_logger().info('语音识别节点已停止')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = QwenASRNode()
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
