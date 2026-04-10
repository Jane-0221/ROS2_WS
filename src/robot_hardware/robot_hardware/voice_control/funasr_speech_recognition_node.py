#!/usr/bin/env python3
"""
FunASR实时中文语音识别ROS2节点

基于已测试成功的real_time_speech_recognition_pyaudio.py
添加ROS2话题发布功能，将识别结果发布到/speech_recognition/text话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import pyaudio
import wave
import tempfile
import os
import numpy as np

try:
    from funasr import AutoModel
    FUNASR_AVAILABLE = True
except ImportError:
    FUNASR_AVAILABLE = False
    print("❌ FunASR未安装")

class FunASRSpeechRecognitionNode(Node):
    """FunASR语音识别节点"""
    
    def __init__(self):
        super().__init__('funasr_speech_recognition_node')
        
        # 参数配置
        self.declare_parameter('sample_rate', 16000)  # 采样率
        self.declare_parameter('record_duration', 3)  # 录音时长（秒）
        self.declare_parameter('device_index', 0)     # 音频设备索引
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.record_duration = self.get_parameter('record_duration').value
        self.device_index = self.get_parameter('device_index').value
        
        # 创建发布者
        self.speech_publisher = self.create_publisher(
            String, 
            '/speech_recognition/text', 
            10
        )
        
        # 控制标志
        self.is_running = True
        self.recognition_count = 0
        
        # 初始化FunASR模型
        self.model = self.init_funasr_model()
        if not self.model:
            self.get_logger().error('FunASR模型初始化失败，节点将退出')
            return
        
        # 检测音频设备
        self.device_index, self.sample_rate = self.detect_audio_device()
        
        self.get_logger().info(f'FunASR语音识别节点已启动')
        self.get_logger().info(f'设备索引: {self.device_index}, 采样率: {self.sample_rate}Hz')
        self.get_logger().info(f'录音时长: {self.record_duration}秒')
        self.get_logger().info(f'发布话题: /speech_recognition/text')
        
        # 启动识别循环
        self.recognition_loop()
    
    def init_funasr_model(self):
        """初始化FunASR模型"""
        if not FUNASR_AVAILABLE:
            self.get_logger().error('FunASR未安装，请安装: pip install funasr')
            return None
        
        try:
            self.get_logger().info('正在加载FunASR中文识别模型...')
            model = AutoModel(
                model="iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch",
                model_revision="v2.0.4"
            )
            self.get_logger().info('FunASR模型加载成功')
            return model
        except Exception as e:
            self.get_logger().error(f'FunASR模型加载失败: {e}')
            return None
    
    def detect_audio_device(self):
        """检测音频设备和采样率"""
        p = pyaudio.PyAudio()
        
        try:
            self.get_logger().info('正在检测音频设备...')
            
            # 查找USB音频设备
            usb_device_index = None
            for i in range(p.get_device_count()):
                device_info = p.get_device_info_by_index(i)
                if device_info['maxInputChannels'] > 0:
                    self.get_logger().info(
                        f'设备 [{i}]: {device_info["name"]} - '
                        f'采样率: {device_info.get("defaultSampleRate", "未知")}Hz'
                    )
                    if "USB" in device_info['name']:
                        usb_device_index = i
                        self.get_logger().info(f'找到USB设备: 索引 {i}')
            
            # 使用USB设备或默认设备
            device_index = usb_device_index if usb_device_index is not None else 0
            
            # 获取设备支持的采样率
            device_info = p.get_device_info_by_index(device_index)
            sample_rate = int(device_info.get('defaultSampleRate', 16000))
            
            return device_index, sample_rate
            
        except Exception as e:
            self.get_logger().error(f'音频设备检测失败: {e}')
            return 0, 16000
        finally:
            p.terminate()
    
    def record_audio_pyaudio(self):
        """使用pyaudio录制音频"""
        p = pyaudio.PyAudio()
        
        try:
            # 计算采样点数
            frames = int(self.sample_rate * self.record_duration)
            
            self.get_logger().info('开始录音...', throttle_duration_sec=5)
            
            # 打开音频流
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=1024,
                input_device_index=self.device_index
            )
            
            # 录制音频
            frames_data = []
            for i in range(0, int(self.sample_rate / 1024 * self.record_duration)):
                data = stream.read(1024)
                frames_data.append(data)
            
            # 关闭流
            stream.stop_stream()
            stream.close()
            
            self.get_logger().info('录音完成', throttle_duration_sec=5)
            
            # 转换为numpy数组
            audio_data = b''.join(frames_data)
            audio_array = np.frombuffer(audio_data, dtype=np.int16)
            
            # 转换为float32格式
            audio_float = audio_array.astype(np.float32) / 32767.0
            
            return audio_float
            
        except Exception as e:
            self.get_logger().error(f'录音失败: {e}')
            return None
        finally:
            p.terminate()
    
    def save_audio_to_tempfile(self, audio_data):
        """保存音频到临时文件"""
        try:
            # 创建临时文件
            temp_file = tempfile.NamedTemporaryFile(suffix='.wav', delete=False)
            
            # 转换为16位PCM格式
            audio_int16 = (audio_data * 32767).astype(np.int16)
            
            # 保存为WAV文件
            with wave.open(temp_file.name, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)  # 16位 = 2字节
                wf.setframerate(self.sample_rate)
                wf.writeframes(audio_int16.tobytes())
            
            return temp_file.name
            
        except Exception as e:
            self.get_logger().error(f'保存音频文件失败: {e}')
            return None
    
    def recognize_speech_funasr(self, audio_file):
        """使用FunASR识别语音"""
        if not self.model:
            return None
        
        try:
            self.get_logger().info('FunASR识别中...', throttle_duration_sec=5)
            result = self.model.generate(input=audio_file, batch_size=1)
            
            if result and len(result) > 0:
                text = result[0].get("text", "").strip()
                return text
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'FunASR识别失败: {e}')
            return None
    
    def publish_speech_text(self, text):
        """发布识别到的文本"""
        msg = String()
        msg.data = text
        self.speech_publisher.publish(msg)
        
        # 打印识别结果
        self.recognition_count += 1
        print("\n" + "=" * 60)
        print(f"🎯 FunASR识别结果 #{self.recognition_count}")
        print(f"⏰ 时间: {time.strftime('%H:%M:%S')}")
        print(f"📝 内容: {text}")
        print(f"🔧 引擎: FunASR（完全离线）")
        print(f"🔊 采样率: {self.sample_rate}Hz")
        print("=" * 60)
        
        # 同时记录到ROS2日志
        self.get_logger().info(f'识别到语音: "{text}"')
    
    def recognition_loop(self):
        """识别循环"""
        self.get_logger().info('开始实时语音识别')
        self.get_logger().info('对着麦克风说话，程序会自动识别并发布结果')
        
        while self.is_running and self.model:
            try:
                self.get_logger().info('正在监听语音输入...', throttle_duration_sec=10)
                
                # 录制音频
                audio_data = self.record_audio_pyaudio()
                
                if audio_data is not None:
                    # 保存为临时文件
                    audio_file = self.save_audio_to_tempfile(audio_data)
                    
                    if audio_file:
                        # 使用FunASR识别
                        text = self.recognize_speech_funasr(audio_file)
                        
                        # 清理临时文件
                        try:
                            os.unlink(audio_file)
                        except:
                            pass
                        
                        if text and len(text) > 0:
                            # 发布识别结果
                            self.publish_speech_text(text)
                        else:
                            self.get_logger().info('未检测到有效语音')
                    else:
                        self.get_logger().error('音频文件保存失败')
                else:
                    self.get_logger().error('录音失败')
                
                # 间隔一下
                time.sleep(1)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'识别过程错误: {e}')
                time.sleep(2)
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.is_running = False
        self.get_logger().info('语音识别节点已停止')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FunASRSpeechRecognitionNode()
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
    main()#!/usr/bin/env python3
"""
FunASR实时中文语音识别ROS2节点

基于已测试成功的real_time_speech_recognition_pyaudio.py
添加ROS2话题发布功能，将识别结果发布到/speech_recognition/text话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import pyaudio
import wave
import tempfile
import os
import numpy as np

try:
    from funasr import AutoModel
    FUNASR_AVAILABLE = True
except ImportError:
    FUNASR_AVAILABLE = False
    print("❌ FunASR未安装")