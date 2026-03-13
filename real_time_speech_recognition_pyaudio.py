#!/usr/bin/env python3
"""
实时语音识别脚本 - pyaudio版本

使用pyaudio替代sounddevice，解决采样率问题
使用FunASR进行高质量中文语音识别
"""

import speech_recognition as sr
import time
import threading
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

def init_funasr_model():
    """初始化FunASR模型"""
    if not FUNASR_AVAILABLE:
        print("❌ FunASR未安装")
        return None
    
    try:
        print("🔧 加载FunASR中文识别模型...")
        model = AutoModel(
            model="iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch",
            model_revision="v2.0.4"
        )
        print("✅ FunASR模型加载成功")
        return model
    except Exception as e:
        print(f"❌ FunASR模型加载失败: {e}")
        return None

def detect_audio_device():
    """检测音频设备和采样率"""
    p = pyaudio.PyAudio()
    
    print("🔊 检测音频设备...")
    
    # 查找USB音频设备
    usb_device_index = None
    for i in range(p.get_device_count()):
        device_info = p.get_device_info_by_index(i)
        if device_info['maxInputChannels'] > 0:
            print(f"  [{i}] {device_info['name']} - 采样率: {device_info.get('defaultSampleRate', '未知')}Hz")
            if "USB" in device_info['name']:
                usb_device_index = i
                print(f"🎯 找到USB设备: 索引 {i}")
    
    # 使用USB设备或默认设备
    device_index = usb_device_index if usb_device_index is not None else 0
    
    # 获取设备支持的采样率
    device_info = p.get_device_info_by_index(device_index)
    sample_rate = int(device_info.get('defaultSampleRate', 44100))
    
    print(f"🔧 使用设备索引: {device_index}")
    print(f"🔧 使用采样率: {sample_rate}Hz")
    
    p.terminate()
    return device_index, sample_rate

def record_audio_pyaudio(device_index, sample_rate, duration=3):
    """使用pyaudio录制音频"""
    p = pyaudio.PyAudio()
    
    try:
        # 计算采样点数
        frames = int(sample_rate * duration)
        
        print("🎤 开始录音...")
        
        # 打开音频流
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=sample_rate,
            input=True,
            frames_per_buffer=1024,
            input_device_index=device_index
        )
        
        # 录制音频
        frames_data = []
        for i in range(0, int(sample_rate / 1024 * duration)):
            data = stream.read(1024)
            frames_data.append(data)
        
        # 关闭流
        stream.stop_stream()
        stream.close()
        
        print("✅ 录音完成")
        
        # 转换为numpy数组
        audio_data = b''.join(frames_data)
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        # 转换为float32格式
        audio_float = audio_array.astype(np.float32) / 32767.0
        
        return audio_float
        
    except Exception as e:
        print(f"❌ 录音失败: {e}")
        return None
    finally:
        p.terminate()

def save_audio_to_tempfile(audio_data, sample_rate=44100):
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
            wf.setframerate(sample_rate)
            wf.writeframes(audio_int16.tobytes())
        
        return temp_file.name
        
    except Exception as e:
        print(f"❌ 保存音频文件失败: {e}")
        return None

def recognize_speech_funasr(model, audio_file):
    """使用FunASR识别语音"""
    if not model:
        return None
    
    try:
        print("🔧 FunASR识别中...")
        result = model.generate(input=audio_file, batch_size=1)
        
        if result and len(result) > 0:
            text = result[0].get("text", "").strip()
            return text
        
        return None
        
    except Exception as e:
        print(f"❌ FunASR识别失败: {e}")
        return None

def real_time_speech_recognition():
    """实时语音识别主函数 - pyaudio版本"""
    
    print("🎤 实时语音识别程序启动 - pyaudio版本")
    print("=" * 50)
    
    # 初始化FunASR模型
    model = init_funasr_model()
    if not model:
        print("❌ FunASR初始化失败，无法继续")
        return
    
    # 检测音频设备
    device_index, sample_rate = detect_audio_device()
    
    print("✅ FunASR语音识别引擎就绪")
    print("=" * 50)
    
    print("\n🎯 开始实时语音识别")
    print("💡 对着麦克风说话，程序会自动识别并打印结果")
    print("⏹️  按 Ctrl+C 停止程序")
    print("-" * 50)
    
    recognition_count = 0
    
    try:
        while True:
            try:
                print("\n👂 正在监听...（请说中文）")
                
                # 使用pyaudio录制音频
                audio_data = record_audio_pyaudio(device_index, sample_rate, duration=3)
                
                if audio_data is not None:
                    recognition_count += 1
                    print(f"✅ 检测到语音输入 #{recognition_count}")
                    
                    # 保存为临时文件
                    audio_file = save_audio_to_tempfile(audio_data, sample_rate)
                    
                    if audio_file:
                        # 使用FunASR识别
                        text = recognize_speech_funasr(model, audio_file)
                        
                        # 清理临时文件
                        try:
                            os.unlink(audio_file)
                        except:
                            pass
                        
                        if text:
                            # 用print打印识别结果
                            print("\n" + "=" * 60)
                            print(f"🎯 FunASR识别结果 #{recognition_count}")
                            print(f"⏰ 时间: {time.strftime('%H:%M:%S')}")
                            print(f"📝 内容: {text}")
                            print(f"🔧 引擎: FunASR（完全离线）")
                            print(f"🔊 采样率: {sample_rate}Hz")
                            print("=" * 60)
                        else:
                            print("❌ FunASR识别失败")
                    else:
                        print("❌ 音频文件保存失败")
                else:
                    print("❌ 录音失败")
                
                # 间隔一下
                time.sleep(1)
                
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print(f"❌ 识别过程错误: {e}")
                time.sleep(2)
                
    except KeyboardInterrupt:
        print("\n\n👋 程序已停止")
        print(f"总计识别次数: {recognition_count}")
    except Exception as e:
        print(f"❌ 程序运行错误: {e}")

def main():
    print("选择识别模式:")
    print("1. 完整模式（推荐）")
    print("2. 简化模式")
    
    choice = input("请输入选择 (1/2, 默认1): ").strip()
    
    if choice == "2":
        # 简化模式（如果需要）
        pass
    else:
        real_time_speech_recognition()


if __name__ == "__main__":
    main()