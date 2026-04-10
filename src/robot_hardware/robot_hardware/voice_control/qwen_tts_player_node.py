#!/usr/bin/env python3
"""
千问TTS语音播放节点

订阅需要语音合成的文本，使用qwen3-tts-flash-realtime合成语音并播放到喇叭
"""

# 重要：在导入pyaudio之前定义此宏，解决Python 3.10+兼容性问题
import sys
sys.setrecursionlimit(2000)

import os
# 必须在导入pyaudio之前设置
os.environ['PY_SSIZE_T_CLEAN'] = '1'

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import threading
import time

try:
    import dashscope
    from dashscope.audio.qwen_tts_realtime import QwenTtsRealtime, QwenTtsRealtimeCallback, AudioFormat
    DASHSCOPE_AVAILABLE = True
except ImportError:
    DASHSCOPE_AVAILABLE = False
    print("❌ dashscope未安装，请运行: pip install dashscope")

try:
    import pyaudio
    PYAUDIO_AVAILABLE = True
except ImportError:
    PYAUDIO_AVAILABLE = False
    print("❌ pyaudio未安装")


class TTSCallback(QwenTtsRealtimeCallback):
    """TTS回调处理类"""
    
    def __init__(self, node):
        self.node = node
        self.tts = None
        self.audio_data = []
        self.lock = threading.Lock()
    
    def set_tts(self, tts):
        self.tts = tts
    
    def on_open(self) -> None:
        self.node.get_logger().info('TTS连接已打开')
    
    def on_close(self, close_status_code, close_msg) -> None:
        self.node.get_logger().info(f'TTS连接已关闭: {close_status_code}, {close_msg}')
    
    def on_event(self, response: str) -> None:
        try:
            response_type = response.get('type', '')
            
            if response_type == 'session.created':
                session_id = response.get('session', {}).get('id', 'unknown')
                self.node.get_logger().info(f'TTS会话创建成功: {session_id}')
            
            # 音频数据
            elif response_type == 'response.audio.delta':
                audio_b64 = response.get('delta', '')
                if audio_b64:
                    audio_bytes = base64.b64decode(audio_b64)
                    with self.lock:
                        self.audio_data.append(audio_bytes)
            
            # 完成
            elif response_type == 'response.done':
                response_id = response.get('response_id', 'unknown')
                self.node.get_logger().info(f'TTS响应完成: {response_id}')
                # 播放音频
                self.play_audio()
        
        except Exception as e:
            self.node.get_logger().error(f'TTS回调处理错误: {e}')
    
    def play_audio(self):
        """播放合成的音频"""
        if not PYAUDIO_AVAILABLE:
            self.node.get_logger().error('pyaudio未安装，无法播放')
            return
        
        with self.lock:
            if not self.audio_data:
                self.node.get_logger().warning('没有音频数据')
                return
            
            # 合并音频数据
            combined_audio = b''.join(self.audio_data)
            self.audio_data = []  # 清空
        
        try:
            # 创建PyAudio对象
            p = pyaudio.PyAudio()
            
            # 打开音频流 (24000Hz, 16位, 单声道)
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=24000,
                output=True
            )
            
            # 播放音频
            stream.write(combined_audio)
            
            # 等待播放完成
            time.sleep(0.1)
            
            # 关闭流
            stream.stop_stream()
            stream.close()
            p.terminate()
            
            self.node.get_logger().info('语音播放完成')
        
        except Exception as e:
            self.node.get_logger().error(f'音频播放失败: {e}')


class QwenTTSPlayerNode(Node):
    """千问TTS播放节点"""
    
    def __init__(self):
        super().__init__('qwen_tts_player_node')
        
        # 参数配置
        self.declare_parameter('api_key', os.environ.get('DASHSCOPE_API_KEY', 'sk-2cab9b4a77914400b0f504817b8fc0ae'))
        self.declare_parameter('voice', 'Cherry')  # 音色
        
        self.api_key = self.get_parameter('api_key').value
        self.voice = self.get_parameter('voice').value
        
        # 设置API Key
        if self.api_key:
            dashscope.api_key = self.api_key
        
        # 创建订阅者 - 订阅TTS文本
        self.tts_sub = self.create_subscription(
            String,
            '/robot/tts_text',
            self.tts_callback,
            10
        )
        
        # 初始化TTS
        if not DASHSCOPE_AVAILABLE:
            self.get_logger().error('dashscope未安装，节点将退出')
            return
        
        if not PYAUDIO_AVAILABLE:
            self.get_logger().error('pyaudio未安装，无法播放语音')
            return
        
        # 创建回调
        self.callback = TTSCallback(self)
        
        # TTS会话状态
        self.tts_session = None
        self.is_connected = False
        self.pending_text = None
        self.lock = threading.Lock()
        
        # ===== 防重复播放机制 =====
        self.last_tts_text = ""
        self.last_tts_time = 0
        self.tts_dedup_window = 5.0  # 5秒内相同文本不重复播放
        self.playing_lock = threading.Lock()  # 播放锁，防止并发处理
        
        # 启动TTS连接
        self.connect_tts()
        
        self.get_logger().info('千问TTS播放节点已启动')
        self.get_logger().info(f'音色: {self.voice}')
        self.get_logger().info(f'订阅话题: /robot/tts_text')
    
    def connect_tts(self):
        """连接TTS服务"""
        try:
            # 创建TTS会话
            self.tts_session = QwenTtsRealtime(
                model='qwen3-tts-flash-realtime',
                callback=self.callback,
                url='wss://dashscope.aliyuncs.com/api-ws/v1/realtime'
            )
            self.callback.set_tts(self.tts_session)
            
            # 连接
            self.tts_session.connect()
            self.is_connected = True
            self.get_logger().info('TTS连接成功')
            
            # 配置会话
            self.tts_session.update_session(
                voice=self.voice,
                response_format=AudioFormat.PCM_24000HZ_MONO_16BIT,
                mode='server_commit'
            )
            
            self.get_logger().info('TTS会话配置完成')
        
        except Exception as e:
            self.get_logger().error(f'TTS连接失败: {e}')
            self.is_connected = False
    
    def tts_callback(self, msg):
        """处理TTS文本"""
        text = msg.data.strip()
        
        if not text:
            return
        
        # ===== 防重复播放检查 =====
        current_time = time.time()
        
        # 使用锁防止并发处理
        if not self.playing_lock.acquire(blocking=False):
            self.get_logger().debug(f'正在处理其他TTS请求，跳过: "{text}"')
            return
        
        try:
            # 检查是否在时间窗口内重复
            if text == self.last_tts_text and (current_time - self.last_tts_time) < self.tts_dedup_window:
                self.get_logger().info(f'重复TTS文本，跳过: "{text}"')
                return
            
            # 更新最后处理的文本和时间
            self.last_tts_text = text
            self.last_tts_time = current_time
            
            self.get_logger().info(f'收到TTS文本: "{text}"')
            
            # 合成并播放语音
            self.synthesize_and_play(text)
        
        finally:
            self.playing_lock.release()
    
    def synthesize_and_play(self, text):
        """合成语音并播放"""
        if not self.is_connected or not self.tts_session:
            self.get_logger().warning('TTS未连接，尝试重新连接')
            self.connect_tts()
            
            if not self.is_connected:
                self.get_logger().error('TTS连接失败，无法合成语音')
                return
        
        try:
            # 发送文本进行合成
            self.callback.audio_data = []  # 清空之前的音频数据
            self.tts_session.append_text(text)
            self.get_logger().info(f'已发送文本到TTS: {text}')
            
        except Exception as e:
            self.get_logger().error(f'TTS合成失败: {e}')
            # 尝试重连
            self.is_connected = False
            self.connect_tts()
    
    def destroy_node(self):
        """销毁节点"""
        if self.tts_session:
            try:
                self.tts_session.finish()
                self.tts_session.close()
            except:
                pass
        self.get_logger().info('TTS播放节点已停止')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = QwenTTSPlayerNode()
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