# USB设备自动识别和配置指南

本文档描述了如何自动识别CH340和CH341设备并分配固定设备名称。

## 文件清单

### 核心文件
- `src/robot_hardware/robot_hardware/usb_scanner.py` - USB设备扫描工具
- `src/robot_hardware/robot_hardware/usb_auto_config.sh` - 自动配置脚本
- `src/robot_hardware/robot_hardware/udev_ch341_check.sh` - CH341设备检查脚本
- `src/robot_hardware/robot_hardware/udev_ch340_check.sh` - CH340设备检查脚本

### 配置文件
- `etc/udev/rules.d/99-serial-permissions.rules` - udev规则文件
- `etc/systemd/system/usb-auto-config.service` - systemd服务文件

## 设备识别规则

### CH340 (电池电量设备)
- Vendor ID: `1a86`
- Product ID: `7523`
- 分配设备名: `/dev/ttyBattery`

### CH341 (STM32通讯设备)
- Vendor ID: `1a86`
- Product ID: `5523`
- 分配设备名: `/dev/ttySTM32`

## 安装步骤

### 1. 设置文件权限
```bash
cd /home/jszn/robot_ros2_ws
chmod +x src/robot_hardware/robot_hardware/*.sh
chmod +x src/robot_hardware/robot_hardware/*.py
```

### 2. 安装udev规则
```bash
sudo cp etc/udev/rules.d/99-serial-permissions.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. 安装systemd服务（可选）
```bash
sudo cp etc/systemd/system/usb-auto-config.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable usb-auto-config.service
sudo systemctl start usb-auto-config.service
```

## 使用方法

### 手动运行扫描工具
```bash
cd /home/jszn/robot_ros2_ws/src/robot_hardware/robot_hardware
python3 usb_scanner.py
```

### 运行自动配置脚本
```bash
cd /home/jszn/robot_ros2_ws/src/robot_hardware/robot_hardware
./usb_auto_config.sh
```

### 检查设备状态
```bash
# 查看所有USB设备
ls -la /dev/ttyUSB*

# 查看符号链接
ls -la /dev/ttySTM32 /dev/ttyBattery

# 查看设备详细信息
udevadm info -q property -n /dev/ttySTM32
udevadm info -q property -n /dev/ttyBattery
```

## 高级配置

### 基于物理端口位置的固定映射（推荐）

1. 查看设备物理路径：
```bash
udevadm info -q property -n /dev/ttyUSB0 | grep ID_PATH
```

2. 修改udev规则，取消注释并修改以下行：
```
# 基于物理端口位置的设备识别
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="5523", ENV{ID_PATH}=="你的设备路径", SYMLINK+="ttySTM32", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ENV{ID_PATH}=="你的设备路径", SYMLINK+="ttyBattery", MODE="0666", GROUP="dialout"
```

### 基于设备序列号的识别

1. 查看设备序列号：
```bash
lsusb -v | grep -A 10 -B 10 "1a86"
```

2. 修改udev规则，取消注释并修改以下行：
```
# 基于设备序列号的识别
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="5523", ATTRS{serial}=="你的序列号", SYMLINK+="ttySTM32", MODE="0666", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{serial}=="你的序列号", SYMLINK+="ttyBattery", MODE="0666", GROUP="dialout"
```

## 故障排除

### 设备未识别
1. 检查设备是否插入：`lsusb | grep 1a86`
2. 检查udev规则是否加载：`sudo udevadm test /sys/class/tty/ttyUSB0`
3. 检查权限：确保用户属于`dialout`组

### 符号链接未创建
1. 检查脚本权限：`ls -la src/robot_hardware/robot_hardware/*.sh`
2. 手动运行扫描工具查看详细错误信息

### 服务启动失败
1. 检查服务状态：`sudo systemctl status usb-auto-config`
2. 查看服务日志：`sudo journalctl -u usb-auto-config`

## 更新日志

- 初始版本：支持CH340/CH341设备自动识别和配置
- 支持多种识别方式：设备ID、物理端口、序列号
- 提供systemd服务支持
- 完整的故障排除指南