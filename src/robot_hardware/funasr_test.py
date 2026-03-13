from funasr import AutoModel

# 加载中文离线识别模型（首次运行自动下载，后续完全离线）
model = AutoModel(
    model="iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch",
    model_revision="v2.0.4"  # 指定稳定版本，避免兼容性问题
)

# 执行离线识别（输入音频文件路径）
res = model.generate(input="test.wav", batch_size=1)

# 打印识别结果（提取中文文本）
print("离线识别结果：", res[0]["text"])
