语音控制代码已独立收拢到这个目录。

当前主链路:
- `qwen_asr_node.py`: 实时语音识别，发布到 `/speech/text`
- `intent_understanding_node.py`: LLM 意图理解，发布到 `/robot/commands`
- `command_dispatcher_node.py`: 把控制指令转发到底盘、STM32、高度和舵机话题
- `qwen_tts_player_node.py`: 播放 `/robot/tts_text`

保留的历史实现:
- `funasr_speech_recognition_node.py`
- `simple_speech_recognition_node.py`
- `speech_recognition_node.py`
- `speech_control_node.py`
- `speech_subscriber_node.py`

兼容性:
- `setup.py` 继续保留原来的 `ros2 run` 可执行名
- `launch/` 根目录保留了同名包装 launch，实际实现位于 `launch/voice_control/`
