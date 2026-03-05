#!/bin/bash
# CH341设备检查脚本 - 用于udev规则中的PROGRAM检查

# 检查是否已经有ttySTM32设备存在
if [ -L "/dev/ttySTM32" ]; then
    # 如果符号链接已存在，检查是否指向有效的CH341设备
    real_device=$(readlink -f "/dev/ttySTM32")
    if [ -e "$real_device" ]; then
        # 检查设备是否为CH341
        vendor_id=$(udevadm info -q property -n "$real_device" | grep "ID_VENDOR_ID" | cut -d'=' -f2)
        product_id=$(udevadm info -q property -n "$real_device" | grep "ID_MODEL_ID" | cut -d'=' -f2)
        
        if [ "$vendor_id" = "1a86" ] && [ "$product_id" = "5523" ]; then
            # 已经有有效的CH341设备，拒绝新的映射
            exit 1
        fi
    fi
fi

# 允许创建新的映射
exit 0