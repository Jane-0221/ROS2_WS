#!/usr/bin/env python3
"""
意图理解节点

订阅语音识别结果，调用千问LLM理解用户意图
发布控制指令到 /robot/commands
发布语音回复文本到 /robot/tts_text

特性：
- 支持上下文对话，能记住历史对话记录
- 防止重复回复相同内容
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
import json
import re
import time
import threading

# 全局变量
DASHSCOPE_AVAILABLE = False
dashscope = None
Generation = None


def _try_import_dashscope():
    """尝试动态导入dashscope"""
    global DASHSCOPE_AVAILABLE, dashscope, Generation
    
    if DASHSCOPE_AVAILABLE:
        return True
    
    try:
        import dashscope as _ds
        dashscope = _ds
        from dashscope import Generation as _Gen
        Generation = _Gen
        DASHSCOPE_AVAILABLE = True
        return True
    except ImportError as e:
        print(f"❌ 导入dashscope失败: {e}")
        return False


# 系统提示词
SYSTEM_PROMPT = """你是一个智能机器人助手，既能控制机器人硬件，也能和用户进行自然对话。

## 控制指令
当用户想要控制机器人硬件时，输出JSON格式的指令：
1. 打开气泵: {"action": "pump", "value": 1, "reply": "气泵已打开"}
2. 关闭气泵: {"action": "pump", "value": 0, "reply": "气泵已关闭"}
3. 前进: {"action": "move", "direction": "forward", "reply": "好的，正在前进"}
4. 后退: {"action": "move", "direction": "backward", "reply": "好的，正在后退"}
5. 停止: {"action": "move", "direction": "stop", "reply": "已停止"}
6. 左转: {"action": "move", "direction": "left", "reply": "好的，向左转"}
7. 右转: {"action": "move", "direction": "right", "reply": "好的，向右转"}
8. 设置升降杆高度: {"action": "height", "value": 数值(0-1000mm), "reply": "已将高度设置为XX毫米"}
9. 升降杆上升: {"action": "lift", "direction": "up", "reply": "升降杆正在上升"}
10. 升降杆下降: {"action": "lift", "direction": "down", "reply": "升降杆正在下降"}

## 闲聊对话
当用户只是在和你聊天、问问题（如问天气、星座运势、闲聊等），而不是控制机器人时，输出：
{"action": "chat", "reply": "你的回复内容"}

## 重要规则
- 每次只输出一个JSON对象，不要输出多个
- 严格按照JSON格式输出，不要有其他内容
- 只有明确是控制机器人的指令才输出控制动作
- 其他情况都用 "chat" 类型回复
- 回复要简洁，控制在50字以内
- 不要追问或继续对话，只回复当前问题"""


class IntentUnderstandingNode(Node):
    """意图理解节点"""
    
    def __init__(self):
        super().__init__('intent_understanding_node')
        
        # 尝试导入dashscope
        if not _try_import_dashscope():
            self.get_logger().error('dashscope未安装，节点将退出')
            return
        
        # 参数配置 - 直接从环境变量获取
        api_key = os.environ.get('DASHSCOPE_API_KEY')
        if not api_key:
            api_key = 'sk-2cab9b4a77914400b0f504817b8fc0ae'
        os.environ['DASHSCOPE_API_KEY'] = api_key
        
        # 设置API Key
        dashscope.api_key = api_key
        
        self.declare_parameter('llm_model', 'qwen-turbo')  # 使用更经济的模型
        self.llm_model = self.get_parameter('llm_model').value
        
        # ===== 上下文对话管理 =====
        # 对话历史记录：保存用户和助手的对话
        self.conversation_history = []
        # 最大历史记录数（保留最近N轮对话）
        self.max_history = 10
        
        # ===== 防重复回复机制 =====
        # 锁：防止并发处理多个请求
        self.processing_lock = threading.Lock()
        # 正在处理的标志
        self.is_processing = False
        # 最近处理的文本和时间（用于去重）
        self.last_processed_text = ""
        self.last_processed_time = 0
        # 去重时间窗口（秒）
        self.dedup_window = 3.0
        
        # ===== TTS 防重复发送 =====
        self.last_tts_reply = ""
        self.last_tts_time = 0
        self.tts_cooldown = 3.0  # TTS 发送冷却时间
        
        # 创建订阅者 - 订阅语音识别结果
        self.speech_sub = self.create_subscription(
            String,
            '/speech/text',
            self.speech_callback,
            10
        )
        
        # 创建发布者 - 发布控制指令
        self.command_publisher = self.create_publisher(
            String,
            '/robot/commands',
            10
        )
        
        # 创建发布者 - 发布TTS文本
        self.tts_publisher = self.create_publisher(
            String,
            '/robot/tts_text',
            10
        )
        
        self.get_logger().info('意图理解节点已启动')
        self.get_logger().info(f'LLM模型: {self.llm_model}')
        self.get_logger().info(f'订阅话题: /speech/text')
        self.get_logger().info(f'发布话题: /robot/commands, /robot/tts_text')
        self.get_logger().info(f'上下文历史: 最近{self.max_history}轮对话')
    
    def speech_callback(self, msg):
        """处理语音识别结果"""
        text = msg.data.strip()
        
        if not text:
            return
        
        # ===== 防重复处理 =====
        current_time = time.time()
        
        # 检查是否在处理中
        if not self.processing_lock.acquire(blocking=False):
            self.get_logger().debug(f'正在处理其他请求，跳过: "{text}"')
            return
        
        try:
            # 检查是否是重复的文本（在时间窗口内）
            if text == self.last_processed_text and (current_time - self.last_processed_time) < self.dedup_window:
                self.get_logger().debug(f'重复文本，跳过: "{text}"')
                return
            
            # 更新最后处理的文本和时间
            self.last_processed_text = text
            self.last_processed_time = current_time
            
            self.get_logger().info(f'收到语音识别结果: "{text}"')
            
            # 调用LLM理解意图
            self.understand_intent(text)
        
        finally:
            self.processing_lock.release()
    
    def understand_intent(self, user_text):
        """调用LLM理解用户意图"""
        if not DASHSCOPE_AVAILABLE:
            self.get_logger().error('dashscope未安装')
            return
        
        try:
            # 构建消息：系统提示 + 历史对话 + 当前用户输入
            messages = [{'role': 'system', 'content': SYSTEM_PROMPT}]
            
            # 添加历史对话记录
            messages.extend(self.conversation_history)
            
            # 添加当前用户输入
            messages.append({'role': 'user', 'content': user_text})
            
            self.get_logger().debug(f'对话历史长度: {len(self.conversation_history)}')
            
            # 调用LLM
            response = Generation.call(
                api_key=dashscope.api_key,
                model=self.llm_model,
                messages=messages,
                result_format='message'
            )
            
            if response.status_code == 200:
                llm_response = response.output.choices[0].message.content
                self.get_logger().info(f'LLM回复: {llm_response}')
                
                # 解析LLM响应并处理
                result = self.parse_and_publish(llm_response, user_text)
                
                # 如果成功处理，保存到对话历史
                if result:
                    self.save_to_history(user_text, result)
            else:
                self.get_logger().error(f'LLM调用失败: {response.status_code}, {response.message}')
                self.publish_tts("抱歉，处理失败")
        
        except Exception as e:
            self.get_logger().error(f'意图理解错误: {e}')
            self.publish_tts("抱歉，处理失败")
    
    def save_to_history(self, user_text, assistant_reply):
        """保存对话到历史记录"""
        # 添加用户消息
        self.conversation_history.append({'role': 'user', 'content': user_text})
        # 添加助手回复
        self.conversation_history.append({'role': 'assistant', 'content': assistant_reply})
        
        # 保持历史记录在限制内
        while len(self.conversation_history) > self.max_history * 2:
            self.conversation_history.pop(0)  # 移除最旧的一条
        
        self.get_logger().debug(f'对话历史已保存，当前长度: {len(self.conversation_history)}')
    
    def parse_and_publish(self, llm_response, user_text):
        """解析LLM响应并发布，返回助手回复文本"""
        try:
            # 查找所有可能的 JSON 对象
            # 只取第一个完整的 JSON 对象
            json_str = None
            
            # 方法1: 查找第一个 { 和匹配的 }
            brace_count = 0
            start_idx = None
            for i, char in enumerate(llm_response):
                if char == '{':
                    if start_idx is None:
                        start_idx = i
                    brace_count += 1
                elif char == '}':
                    brace_count -= 1
                    if brace_count == 0 and start_idx is not None:
                        json_str = llm_response[start_idx:i+1]
                        break
            
            if json_str:
                self.get_logger().debug(f'提取的JSON: {json_str}')
                
                try:
                    command = json.loads(json_str)
                except json.JSONDecodeError as e:
                    self.get_logger().error(f'JSON解析失败: {e}, 原始: {json_str}')
                    result = self.handle_simple_command(llm_response)
                    return result
                
                action = command.get('action', 'unknown')
                reply = command.get('reply', '好的')
                
                # 根据动作类型处理
                if action == 'chat':
                    self.get_logger().info(f'闲聊模式，回复: {reply}')
                elif action == 'unknown':
                    self.get_logger().info(f'未知指令，回复: {reply}')
                else:
                    self.publish_command(command)
                    self.get_logger().info(f'执行动作: {action}')
                
                # 发布TTS文本
                self.publish_tts(reply)
                
                return reply
                
            else:
                # 没有JSON，尝试简单匹配
                result = self.handle_simple_command(llm_response)
                return result
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析失败: {e}')
            self.publish_tts("抱歉，没听清楚")
            return "抱歉，没听清楚"
        except Exception as e:
            self.get_logger().error(f'解析错误: {e}')
            self.publish_tts("抱歉，处理失败")
            return "抱歉，处理失败"
    
    def handle_simple_command(self, text):
        """处理简单命令匹配"""
        text_lower = text.lower()
        
        # 简单匹配
        if '打开气泵' in text or '气泵开' in text:
            self.publish_command({'action': 'pump', 'value': 1})
            self.publish_tts('气泵已打开')
            return '气泵已打开'
        elif '关闭气泵' in text or '气泵关' in text:
            self.publish_command({'action': 'pump', 'value': 0})
            self.publish_tts('气泵已关闭')
            return '气泵已关闭'
        elif '前进' in text:
            self.publish_command({'action': 'move', 'direction': 'forward'})
            self.publish_tts('好的，正在前进')
            return '好的，正在前进'
        elif '后退' in text or '后退' in text:
            self.publish_command({'action': 'move', 'direction': 'backward'})
            self.publish_tts('好的，正在后退')
            return '好的，正在后退'
        elif '停止' in text:
            self.publish_command({'action': 'move', 'direction': 'stop'})
            self.publish_tts('已停止')
            return '已停止'
        elif '左转' in text:
            self.publish_command({'action': 'move', 'direction': 'left'})
            self.publish_tts('好的，向左转')
            return '好的，向左转'
        elif '右转' in text:
            self.publish_command({'action': 'move', 'direction': 'right'})
            self.publish_tts('好的，向右转')
            return '好的，向右转'
        else:
            self.publish_tts('抱歉，我没听清楚，请再说一遍')
            return '抱歉，我没听清楚，请再说一遍'
    
    def publish_command(self, command):
        """发布控制指令"""
        msg = String()
        msg.data = json.dumps(command)
        self.command_publisher.publish(msg)
    
    def publish_tts(self, text):
        """发布TTS文本（带防重复机制）"""
        current_time = time.time()
        
        # 检查是否在冷却时间内发送了相同的回复
        if text == self.last_tts_reply and (current_time - self.last_tts_time) < self.tts_cooldown:
            self.get_logger().debug(f'重复TTS回复，跳过: "{text}"')
            return
        
        # 更新记录
        self.last_tts_reply = text
        self.last_tts_time = current_time
        
        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IntentUnderstandingNode()
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