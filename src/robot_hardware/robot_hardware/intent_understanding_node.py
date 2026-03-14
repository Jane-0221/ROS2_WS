#!/usr/bin/env python3
"""
意图理解节点

订阅语音识别结果，调用千问LLM理解用户意图
发布控制指令到 /robot/commands
发布语音回复文本到 /robot/tts_text
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re
import os

try:
    import dashscope
    from dashscope import Generation
    DASHSCOPE_AVAILABLE = True
except ImportError:
    DASHSCOPE_AVAILABLE = False
    print("❌ dashscope未安装")


# 系统提示词
SYSTEM_PROMPT = """你是一个机器人控制助手。用户会说各种指令，你需要理解意图并输出JSON格式的指令。

支持的控制指令:
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

请严格按照JSON格式输出，不要有其他内容。如果用户指令不明确或无法理解，请输出: {"action": "unknown", "reply": "抱歉，我没听清楚，请再说一遍"}

用户指令: """


class IntentUnderstandingNode(Node):
    """意图理解节点"""
    
    def __init__(self):
        super().__init__('intent_understanding_node')
        
        # 参数配置
        self.declare_parameter('api_key', os.environ.get('DASHSCOPE_API_KEY', 'sk-2cab9b4a77914400b0f504817b8fc0ae'))
        self.declare_parameter('llm_model', 'qwen-plus')
        
        self.api_key = self.get_parameter('api_key').value
        self.llm_model = self.get_parameter('llm_model').value
        
        # 设置API Key
        if self.api_key:
            dashscope.api_key = self.api_key
        
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
    
    def speech_callback(self, msg):
        """处理语音识别结果"""
        text = msg.data.strip()
        
        if not text:
            return
        
        self.get_logger().info(f'收到语音识别结果: "{text}"')
        
        # 调用LLM理解意图
        self.understand_intent(text)
    
    def understand_intent(self, user_text):
        """调用LLM理解用户意图"""
        if not DASHSCOPE_AVAILABLE:
            self.get_logger().error('dashscope未安装')
            return
        
        try:
            # 构建消息
            messages = [
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': user_text}
            ]
            
            # 调用LLM
            response = Generation.call(
                api_key=self.api_key,
                model=self.llm_model,
                messages=messages,
                result_format='message'
            )
            
            if response.status_code == 200:
                llm_response = response.output.choices[0].message.content
                self.get_logger().info(f'LLM回复: {llm_response}')
                
                # 解析LLM响应
                self.parse_and_publish(llm_response)
            else:
                self.get_logger().error(f'LLM调用失败: {response.status_code}, {response.message}')
                # 发送错误回复
                self.publish_tts("抱歉，处理失败")
        
        except Exception as e:
            self.get_logger().error(f'意图理解错误: {e}')
            self.publish_tts("抱歉，处理失败")
    
    def parse_and_publish(self, llm_response):
        """解析LLM响应并发布"""
        try:
            # 尝试提取JSON
            json_match = re.search(r'\{[^}]+\}', llm_response, re.DOTALL)
            
            if json_match:
                json_str = json_match.group(0)
                command = json.loads(json_str)
                
                action = command.get('action', 'unknown')
                reply = command.get('reply', '好的')
                
                # 发布控制指令
                self.publish_command(command)
                
                # 发布TTS文本
                self.publish_tts(reply)
                
                self.get_logger().info(f'执行动作: {action}, 回复: {reply}')
            else:
                # 没有JSON，尝试简单匹配
                self.handle_simple_command(llm_response)
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析失败: {e}')
            self.publish_tts("抱歉，没听清楚")
        except Exception as e:
            self.get_logger().error(f'解析错误: {e}')
            self.publish_tts("抱歉，处理失败")
    
    def handle_simple_command(self, text):
        """处理简单命令匹配"""
        text_lower = text.lower()
        
        # 简单匹配
        if '打开气泵' in text or '气泵开' in text:
            self.publish_command({'action': 'pump', 'value': 1})
            self.publish_tts('气泵已打开')
        elif '关闭气泵' in text or '气泵关' in text:
            self.publish_command({'action': 'pump', 'value': 0})
            self.publish_tts('气泵已关闭')
        elif '前进' in text:
            self.publish_command({'action': 'move', 'direction': 'forward'})
            self.publish_tts('好的，正在前进')
        elif '后退' in text or '后退' in text:
            self.publish_command({'action': 'move', 'direction': 'backward'})
            self.publish_tts('好的，正在后退')
        elif '停止' in text:
            self.publish_command({'action': 'move', 'direction': 'stop'})
            self.publish_tts('已停止')
        elif '左转' in text:
            self.publish_command({'action': 'move', 'direction': 'left'})
            self.publish_tts('好的，向左转')
        elif '右转' in text:
            self.publish_command({'action': 'move', 'direction': 'right'})
            self.publish_tts('好的，向右转')
        else:
            self.publish_tts('抱歉，我没听清楚，请再说一遍')
    
    def publish_command(self, command):
        """发布控制指令"""
        msg = String()
        msg.data = json.dumps(command)
        self.command_publisher.publish(msg)
    
    def publish_tts(self, text):
        """发布TTS文本"""
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
