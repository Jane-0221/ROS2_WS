#!/usr/bin/env python3
"""
中文语音识别节点

使用已安装的SpeechRecognition库进行实时中文语音识别
支持在线和离线两种模式
"""

import rclpy
from rclpy.node import Node
import threading
import time
import speech_recognition as sr


class SpeechRecognitionNode(Node):
    """语音识别节点"""
    
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # 参数配置
        self.declare_parameter('recognition_mode', 'online')  # online/offline
        self.declare_parameter('listen_timeout', 5)  # 监听超时时间
        self.declare_parameter('phrase_time_limit', 10)  # 短语时间限制
        self.declare_parameter('energy_threshold', 300)  # 能量阈值
        
        self.recognition_mode = self.get_parameter('recognition_mode').value
        self.listen_timeout = self.get_parameter('listen_timeout').value
        self.phrase_time_limit = self.get_parameter('phrase_time_limit').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        
        # 语音识别器
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # 调整识别器参数
        self.recognizer.energy_threshold = self.energy_threshold
        self.recognizer.dynamic_energy_threshold = True
        
        # 控制标志
        self.is_running = True
        
        # 初始化麦克风
        self.init_microphone()
        
        # 启动识别线程
        self.start_recognition()
        
        self.get_logger().info(f'语音识别节点已启动 - 模式: {self.recognition_mode}')
    
    def init_microphone(self):
        """初始化麦克风"""
        try:
            # 调整环境噪音
            self.get_logger().info('正在调整环境噪音，请保持安静...')
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.get_logger().info('环境噪音调整完成')
            
        except Exception as e:
            self.get_logger().error(f'初始化麦克风失败: {e}')
    
    def recognize_speech_online(self, audio_data):
        """在线语音识别（需要网络）"""
        try:
            text = self.recognizer.recognize_google(audio_data, language='zh-CN')
            return text
        except sr.UnknownValueError:
            return "无法识别语音"
        except sr.RequestError as e:
            return f"在线识别服务错误: {e}"
    
    def recognize_speech_offline(self, audio_data):
        """离线语音识别"""
        try:
            # 尝试使用pocketsphinx（需要额外安装）
            text = self.recognizer.recognize_sphinx(audio_data, language='zh-CN')
            return text
        except sr.UnknownValueError:
            return "无法识别语音"
        except Exception as e:
            return f"离线识别错误: {e}"
    
    def recognition_loop(self):
        """识别循环"""
        recognition_count = 0
        
        while self.is_running:
            try:
                self.get_logger().info('正在监听语音输入...')
                
                # 监听麦克风
                with self.microphone as source:
                    audio = self.recognizer.listen(
                        source, 
                        timeout=self.listen_timeout,
                        phrase_time_limit=self.phrase_time_limit
                    )
                
                recognition_count += 1
                self.get_logger().info(f'检测到语音输入 #{recognition_count}')
                
                # 根据模式选择识别方法
                if self.recognition_mode == 'online':
                    result = self.recognize_speech_online(audio)
                else:
                    result = self.recognize_speech_offline(audio)
                
                # 处理识别结果
                if result and result != "无法识别语音" and not result.startswith("在线识别服务错误"):
                    self.get_logger().info(f'识别结果: {result}')
                    
                    # 格式化输出
                    print(f"\n{'='*50}")
                    print(f"🎤 语音识别结果 #{recognition_count}")
                    print(f"⏰ 时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
                    print(f"📝 内容: {result}")
                    print(f"🌐 模式: {self.recognition_mode}")
                    print(f"{'='*50}")
                else:
                    self.get_logger().warn(f'识别失败: {result}')
                    print(f"❌ 识别失败: {result}")
                
            except sr.WaitTimeoutError:
                # 监听超时是正常的
                continue
            except Exception as e:
                self.get_logger().error(f'识别循环错误: {e}')
                time.sleep(1)
    
    def start_recognition(self):
        """启动语音识别"""
        # 启动识别线程
        recognition_thread = threading.Thread(target=self.recognition_loop)
        recognition_thread.daemon = True
        recognition_thread.start()
        
        self.get_logger().info('语音识别线程已启动')
    
    def destroy_node(self):
        """清理资源"""
        self.is_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 检查SpeechRecognition是否可用
    try:
        import speech_recognition as sr
        print("✅ SpeechRecognition库可用")
    except ImportError:
        print("❌ SpeechRecognition库未安装")
        return
    
    # 检查pyaudio是否可用
    try:
        import pyaudio
        print("✅ PyAudio库可用")
    except ImportError:
        print("❌ PyAudio库未安装")
        return
    
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 语音识别节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
中文语音识别节点

使用已安装的SpeechRecognition库进行实时中文语音识别
支持在线和离线两种模式
"""

import rclpy
from rclpy.node import Node
import threading
import time
import speech_recognition as sr


class SpeechRecognitionNode(Node):
    """语音识别节点"""
    
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # 参数配置
        self.declare_parameter('recognition_mode', 'online')  # online/offline
        self.declare_parameter('listen_timeout', 5)  # 监听超时时间
        self.declare_parameter('phrase_time_limit', 10)  # 短语时间限制
        self.declare_parameter('energy_threshold', 300)  # 能量阈值
        
        self.recognition_mode = self.get_parameter('recognition_mode').value
        self.listen_timeout = self.get_parameter('listen_timeout').value
        self.phrase_time_limit = self.get_parameter('phrase_time_limit').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        
        # 语音识别器
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # 调整识别器参数
        self.recognizer.energy_threshold = self.energy_threshold
        self.recognizer.dynamic_energy_threshold = True
        
        # 控制标志
        self.is_running = True
        
        # 初始化麦克风
        self.init_microphone()
        
        # 启动识别线程
        self.start_recognition()
        
        self.get_logger().info(f'语音识别节点已启动 - 模式: {self.recognition_mode}')
    
    def init_microphone(self):
        """初始化麦克风"""
        try:
            # 调整环境噪音
            self.get_logger().info('正在调整环境噪音，请保持安静...')
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=2)
            self.get_logger().info('环境噪音调整完成')
            
        except Exception as e:
            self.get_logger().error(f'初始化麦克风失败: {e}')
    
    def recognize_speech_online(self, audio_data):
        """在线语音识别（需要网络）"""
        try:
            text = self.recognizer.recognize_google(audio_data, language='zh-CN')
            return text
        except sr.UnknownValueError:
            return "无法识别语音"
        except sr.RequestError as e:
            return f"在线识别服务错误: {e}"
    
    def recognize_speech_offline(self, audio_data):
        """离线语音识别"""
        try:
            # 尝试使用pocketsphinx（需要额外安装）
            text = self.recognizer.recognize_sphinx(audio_data, language='zh-CN')
            return text
        except sr.UnknownValueError:
            return "无法识别语音"
        except Exception as e:
            return f"离线识别错误: {e}"
    
    def recognition_loop(self):
        """识别循环"""
        recognition_count = 0
        
        while self.is_running:
            try:
                self.get_logger().info('正在监听语音输入...')
                
                # 监听麦克风
                with self.microphone as source:
                    audio = self.recognizer.listen(
                        source, 
                        timeout=self.listen_timeout,
                        phrase_time_limit=self.phrase_time_limit
                    )
                
                recognition_count += 1
                self.get_logger().info(f'检测到语音输入 #{recognition_count}')
                
                # 根据模式选择识别方法
                if self.recognition_mode == 'online':
                    result = self.recognize_speech_online(audio)
                else:
                    result = self.recognize_speech_offline(audio)
                
                # 处理识别结果
                if result and result != "无法识别语音" and not result.startswith("在线识别服务错误"):
                    self.get_logger().info(f'识别结果: {result}')
                    
                    # 格式化输出
                    print(f"\n{'='*50}")
                    print(f"🎤 语音识别结果 #{recognition_count}")
                    print(f"⏰ 时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
                    print(f"📝 内容: {result}")
                    print(f"🌐 模式: {self.recognition_mode}")
                    print(f"{'='*50}")
                else:
                    self.get_logger().warn(f'识别失败: {result}')
                    print(f"❌ 识别失败: {result}")
                
            except sr.WaitTimeoutError:
                # 监听超时是正常的
                continue
            except Exception as e:
                self.get_logger().error(f'识别循环错误: {e}')
                time.sleep(1)
    
    def start_recognition(self):
        """启动语音识别"""
        # 启动识别线程
        recognition_thread = threading.Thread(target=self.recognition_loop)
        recognition_thread.daemon = True
        recognition_thread.start()
        
        self.get_logger().info('语音识别线程已启动')
    
    def destroy_node(self):
        """清理资源"""
        self.is_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    # 检查SpeechRecognition是否可用
    try:
        import speech_recognition as sr
        print("✅ SpeechRecognition库可用")
    except ImportError:
        print("❌ SpeechRecognition库未安装")
        return
    
    # 检查pyaudio是否可用
    try:
        import pyaudio
        print("✅ PyAudio库可用")
    except ImportError:
        print("❌ PyAudio库未安装")
        return
    
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 语音识别节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()