#!/usr/bin/env python3
"""
千问实时语音识别节点

使用qwen3-asr-flash-realtime进行实时语音识别
将识别结果发布到 /speech/text 话题
"""

import sys
import os

# 重要：在导入其他模块之前先添加用户Python包路径
_user_python_path = "/home/jszn/.local/lib/python3.10/site-packages"
if _user_python_path not in sys.path:
    sys.path.insert(0, _user_python_path)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import json
import time
import pyaudio
import numpy as np
import threading

# 全局变量存储dashscope导入状态
DASHSCOPE_AVAILABLE = False
dashscope = None
OmniRealtimeConversation = None
OmniRealtimeCallback = None
TranscriptionParams = None
MultiModality = None


def _try_import_dashscope():
    """尝试动态导入dashscope"""
    global DASHSCOPE_AVAILABLE, dashscope, OmniRealtimeConversation, OmniRealtimeCallback, TranscriptionParams, MultiModality
    
    if DASHSCOPE_AVAILABLE:
        return True
    
    try:
        import dashscope as _ds
        dashscope = _ds
        from dashscope.audio.qwen_omni import (
            OmniRealtimeConversation as _Conv,
            OmniRealtimeCallback as _Cb,
            MultiModality as _Mm
        )
        from dashscope.audio.qwen_omni.omni_realtime import TranscriptionParams as _Tp
        OmniRealtimeConversation = _Conv
        OmniRealtimeCallback = _Cb
        TranscriptionParams = _Tp
        MultiModality = _Mm
        DASHSCOPE_AVAILABLE = True
        return True
    except ImportError as e:
        print(f"❌ 导入dashscope失败: {e}")
        return False


# 回调类将在运行时动态创建
QwenASRCallback = None


class QwenASRNode(Node):
    """千问实时语音识别节点"""
    
    def __init__(self):
        super().__init__('qwen_asr_node')
        
        # 尝试导入dashscope
        if not _try_import_dashscope():
            self.get_logger().error('dashscope未安装，节点将退出')
            return
        
        # 动态创建回调类（因为OmniRealtimeCallback需要从dashscope导入）
        global QwenASRCallback
        if QwenASRCallback is None:
            QwenASRCallback = type('QwenASRCallback', (OmniRealtimeCallback,), {
                '__init__': self._callback_init,
                'on_open': self._callback_on_open,
                'on_close': self._callback_on_close,
                'on_event': self._callback_on_event,
            })
        
        # 参数配置 - 直接从环境变量获取，如果没有则使用默认值
        api_key = os.environ.get('DASHSCOPE_API_KEY')
        if not api_key:
            api_key = 'sk-2cab9b4a77914400b0f504817b8fc0ae'
        os.environ['DASHSCOPE_API_KEY'] = api_key
        
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('device_index', 0)
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.device_index = self.get_parameter('device_index').value
        
        # 设置API Key
        if api_key and dashscope:
            dashscope.api_key = api_key
        
        # 创建发布者
        self.speech_publisher = self.create_publisher(
            String,
            '/speech/text',
            10
        )
        
        # 音频录制相关
        self.is_recording = False
        self.audio_thread = None
        self.p = None
        
        # 初始化ASR
        self.conversation = None
        self.callback = None
        
        # 回调数据
        self.final_text = ""
        self.text_lock = threading.Lock()
        
        # 创建回调
        self.callback = QwenASRCallback(self)
        
        # 创建对话会话
        try:
            self.conversation = OmniRealtimeConversation(
                model='qwen3-asr-flash-realtime',
                url='wss://dashscope.aliyuncs.com/api-ws/v1/realtime',
                callback=self.callback
            )
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
    
    def _callback_init(self, node):
        self.node = node
        self.conversation = None
        self.final_text = ""
        self.text_lock = threading.Lock()
    
    def _callback_on_open(self):
        self.node.get_logger().info('千问ASR连接已打开')
    
    def _callback_on_close(self, code, msg):
        self.node.get_logger().info(f'千问ASR连接已关闭: code={code}, msg={msg}')
    
    def _callback_on_event(self, response):
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
    
    def detect_audio_device(self):
        """检测音频设备"""
        self.p = pyaudio.PyAudio()
        
        try:
            self.get_logger().info('正在检测音频设备...')
            
            # 查找可用的输入设备
            available_input_devices = []
            default_input_device = self.p.get_default_input_device_info()
            
            for i in range(self.p.get_device_count()):
                device_info = self.p.get_device_info_by_index(i)
                max_input_channels = device_info['maxInputChannels']
                
                if max_input_channels > 0:
                    self.get_logger().info(
                        f'设备 [{i}]: {device_info["name"]} - '
                        f'输入通道数: {max_input_channels}, '
                        f'采样率: {device_info.get("defaultSampleRate", "未知")}Hz'
                    )
                    available_input_devices.append({
                        'index': i,
                        'name': device_info['name'],
                        'channels': max_input_channels,
                        'sample_rate': int(device_info.get('defaultSampleRate', 16000))
                    })
                    
                    if "USB" in device_info['name']:
                        self.get_logger().info(f'找到USB音频设备: 索引 {i}')
            
            # 优先选择USB设备，否则选择默认输入设备
            selected_device = None
            for dev in available_input_devices:
                if "USB" in dev['name']:
                    selected_device = dev
                    break
            
            if selected_device is None and available_input_devices:
                # 尝试使用默认设备
                for dev in available_input_devices:
                    if dev['index'] == default_input_device['index']:
                        selected_device = dev
                        break
                # 如果默认设备不在列表中，使用第一个可用设备
                if selected_device is None:
                    selected_device = available_input_devices[0]
            
            if selected_device is None:
                self.get_logger().error('未找到可用的音频输入设备！')
                return
            
            self.device_index = selected_device['index']
            self.sample_rate = selected_device['sample_rate']
            self.max_channels = selected_device['channels']
            
            self.get_logger().info(
                f'使用设备: 索引 {self.device_index}, '
                f'名称: {selected_device["name"]}, '
                f'采样率: {self.sample_rate}Hz, '
                f'最大通道数: {self.max_channels}'
            )
            
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
            
            # 优化 VAD 参数以降低延迟
            self.conversation.update_session(
                output_modalities=[MultiModality.TEXT],
                enable_turn_detection=True,
                turn_detection_type='server_vad',
                turn_detection_threshold=0.0,
                turn_detection_silence_duration_ms=200,  # 减少静音检测时间，更快结束识别
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
        stream = None
        
        try:
            # 获取设备信息
            device_info = p.get_device_info_by_index(self.device_index)
            max_channels = int(device_info['maxInputChannels'])
            
            # 确定使用的通道数：优先使用单声道，但如果设备不支持则使用设备支持的通道数
            channels = 1 if max_channels >= 1 else max_channels
            if channels < 1:
                self.get_logger().error(f'设备不支持音频输入: 通道数={max_channels}')
                return
            
            self.get_logger().info(f'音频配置: 通道数={channels}, 采样率={self.sample_rate}')
            
            # 计算合适的缓冲区大小：约 30ms 的音频数据
            # 16000Hz * 0.03s = 480, 44100Hz * 0.03s = 1323
            chunk_size = int(self.sample_rate * 0.03)  # 约 30ms
            # 确保 chunk_size 是 2 的幂次方，便于处理
            chunk_size = max(512, min(1024, chunk_size))
            
            # 打开音频流 - 使用较小的缓冲区降低延迟
            stream = p.open(
                format=pyaudio.paInt16,
                channels=channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=chunk_size,
                input_device_index=self.device_index
            )
            
            self.get_logger().info(f'开始录音并识别... (缓冲区: {chunk_size} 样本, 约 {chunk_size/self.sample_rate*1000:.1f}ms)')
            
            while self.is_recording:
                try:
                    # 录制音频，忽略溢出错误
                    audio_data = stream.read(chunk_size, exception_on_overflow=False)
                    
                    # 如果是多声道，转换为单声道（取左声道或平均）
                    if channels > 1:
                        audio_array = np.frombuffer(audio_data, dtype=np.int16)
                        # 重塑为 (样本数, 通道数)
                        audio_array = audio_array.reshape(-1, channels)
                        # 取所有通道的平均值转换为单声道
                        audio_mono = np.mean(audio_array, axis=1).astype(np.int16)
                        audio_data = audio_mono.tobytes()
                    
                    # 转换为base64并发送
                    audio_b64 = base64.b64encode(audio_data).decode('utf-8')
                    self.conversation.append_audio(audio_b64)
                    
                except IOError as e:
                    # 音频溢出警告，继续录音
                    if e.errno == -9981:  # Input overflowed
                        self.get_logger().warn('音频缓冲区溢出，继续录音...')
                        continue
                    else:
                        raise
                except Exception as e:
                    self.get_logger().error(f'录音/发送失败: {e}')
                    break
            
        except Exception as e:
            self.get_logger().error(f'录音线程错误: {e}')
        finally:
            if stream is not None:
                try:
                    stream.stop_stream()
                    stream.close()
                except:
                    pass
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