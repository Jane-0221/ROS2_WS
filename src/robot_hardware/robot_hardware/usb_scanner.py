#!/usr/bin/env python3
"""
USB设备扫描工具
自动识别CH340和CH341设备并分配固定设备名称
"""

import os
import subprocess
import re
import time
from typing import Dict, List, Optional

class USBDeviceScanner:
    """USB设备扫描器"""
    
    def __init__(self):
        # CH340和CH341的设备信息
        self.ch340_info = {
            'vendor_id': '1a86',
            'product_id': '7523',
            'name': 'CH340',
            'device_name': 'ttyBattery'
        }
        
        self.ch341_info = {
            'vendor_id': '1a86', 
            'product_id': '5523',
            'name': 'CH341',
            'device_name': 'ttySTM32'
        }
    
    def scan_usb_devices(self) -> List[Dict]:
        """扫描所有USB转串口设备"""
        devices = []
        
        # 检查/dev目录下的ttyUSB设备
        for i in range(10):  # 检查ttyUSB0到ttyUSB9
            device_path = f"/dev/ttyUSB{i}"
            if os.path.exists(device_path):
                device_info = self.get_device_info(device_path, i)
                if device_info:
                    devices.append(device_info)
        
        return devices
    
    def get_device_info(self, device_path: str, index: int) -> Optional[Dict]:
        """获取单个设备详细信息"""
        try:
            # 使用udevadm获取设备信息
            cmd = f"udevadm info -q property -n {device_path}"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            if result.returncode != 0:
                return None
            
            device_info = {'device_path': device_path, 'index': index}
            
            # 解析udev信息
            for line in result.stdout.split('\n'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    
                    if key == 'ID_VENDOR_ID':
                        device_info['vendor_id'] = value.lower()
                    elif key == 'ID_MODEL_ID':
                        device_info['product_id'] = value.lower()
                    elif key == 'ID_VENDOR_FROM_DATABASE':
                        device_info['vendor_name'] = value
                    elif key == 'ID_MODEL_FROM_DATABASE':
                        device_info['product_name'] = value
                    elif key == 'ID_SERIAL_SHORT':
                        device_info['serial'] = value
                    elif key == 'ID_PATH':
                        device_info['path'] = value
            
            # 识别设备类型
            device_info['type'] = self.identify_device_type(device_info)
            
            return device_info
            
        except Exception as e:
            print(f"获取设备 {device_path} 信息失败: {e}")
            return None
    
    def identify_device_type(self, device_info: Dict) -> str:
        """识别设备类型"""
        vendor_id = device_info.get('vendor_id', '')
        product_id = device_info.get('product_id', '')
        
        if vendor_id == self.ch340_info['vendor_id'] and product_id == self.ch340_info['product_id']:
            return 'CH340'
        elif vendor_id == self.ch341_info['vendor_id'] and product_id == self.ch341_info['product_id']:
            return 'CH341'
        else:
            return 'Unknown'
    
    def create_symlinks(self, devices: List[Dict]) -> bool:
        """创建符号链接"""
        success = True
        
        for device in devices:
            device_type = device.get('type')
            device_path = device.get('device_path')
            
            if device_type == 'CH340':
                target_link = f"/dev/{self.ch340_info['device_name']}"
            elif device_type == 'CH341':
                target_link = f"/dev/{self.ch341_info['device_name']}"
            else:
                continue
            
            try:
                # 使用sudo权限创建符号链接
                import subprocess
                
                # 删除已存在的符号链接
                if os.path.exists(target_link) or os.path.islink(target_link):
                    subprocess.run(['sudo', 'rm', '-f', target_link], check=True)
                
                # 创建新的符号链接
                subprocess.run(['sudo', 'ln', '-sf', device_path, target_link], check=True)
                print(f"创建符号链接: {device_path} -> {target_link}")
                
            except subprocess.CalledProcessError as e:
                print(f"创建符号链接失败 {device_path} -> {target_link}: {e}")
                print("提示: 请使用sudo权限运行此脚本")
                success = False
            except Exception as e:
                print(f"创建符号链接失败 {device_path} -> {target_link}: {e}")
                success = False
        
        return success
    
    def print_device_info(self, devices: List[Dict]):
        """打印设备信息"""
        print("\n=== USB设备扫描结果 ===")
        
        ch340_found = False
        ch341_found = False
        
        for device in devices:
            device_type = device.get('type', 'Unknown')
            print(f"\n设备: {device['device_path']}")
            print(f"  类型: {device_type}")
            print(f"  Vendor ID: {device.get('vendor_id', 'N/A')}")
            print(f"  Product ID: {device.get('product_id', 'N/A')}")
            print(f"  厂商: {device.get('vendor_name', 'N/A')}")
            print(f"  产品: {device.get('product_name', 'N/A')}")
            print(f"  序列号: {device.get('serial', 'N/A')}")
            
            if device_type == 'CH340':
                ch340_found = True
                print(f"  分配名称: {self.ch340_info['device_name']}")
            elif device_type == 'CH341':
                ch341_found = True
                print(f"  分配名称: {self.ch341_info['device_name']}")
        
        print("\n=== 设备状态 ===")
        print(f"CH340 (电池电量): {'找到' if ch340_found else '未找到'}")
        print(f"CH341 (STM32通讯): {'找到' if ch341_found else '未找到'}")

def main():
    """主函数"""
    scanner = USBDeviceScanner()
    
    print("开始扫描USB设备...")
    devices = scanner.scan_usb_devices()
    
    if not devices:
        print("未找到任何USB转串口设备")
        return
    
    scanner.print_device_info(devices)
    
    # 创建符号链接
    print("\n创建符号链接...")
    if scanner.create_symlinks(devices):
        print("符号链接创建完成")
    else:
        print("符号链接创建过程中出现错误")

if __name__ == "__main__":
    main()#!/usr/bin/env python3
"""
USB设备扫描工具
自动识别CH340和CH341设备并分配固定设备名称
"""

import os
import subprocess
import re
import time
from typing import Dict, List, Optional

class USBDeviceScanner:
    """USB设备扫描器"""
    
    def __init__(self):
        # CH340和CH341的设备信息
        self.ch340_info = {
            'vendor_id': '1a86',
            'product_id': '7523',
            'name': 'CH340',
            'device_name': 'ttyBattery'
        }
        
        self.ch341_info = {
            'vendor_id': '1a86', 
            'product_id': '5523',
            'name': 'CH341',
            'device_name': 'ttySTM32'
        }
    
    def scan_usb_devices(self) -> List[Dict]:
        """扫描所有USB转串口设备"""
        devices = []
        
        # 检查/dev目录下的ttyUSB设备
        for i in range(10):  # 检查ttyUSB0到ttyUSB9
            device_path = f"/dev/ttyUSB{i}"
            if os.path.exists(device_path):
                device_info = self.get_device_info(device_path, i)
                if device_info:
                    devices.append(device_info)
        
        return devices
    
    def get_device_info(self, device_path: str, index: int) -> Optional[Dict]:
        """获取单个设备详细信息"""
        try:
            # 使用udevadm获取设备信息
            cmd = f"udevadm info -q property -n {device_path}"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            
            if result.returncode != 0:
                return None
            
            device_info = {'device_path': device_path, 'index': index}
            
            # 解析udev信息
            for line in result.stdout.split('\n'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    
                    if key == 'ID_VENDOR_ID':
                        device_info['vendor_id'] = value.lower()
                    elif key == 'ID_MODEL_ID':
                        device_info['product_id'] = value.lower()
                    elif key == 'ID_VENDOR_FROM_DATABASE':
                        device_info['vendor_name'] = value
                    elif key == 'ID_MODEL_FROM_DATABASE':
                        device_info['product_name'] = value
                    elif key == 'ID_SERIAL_SHORT':
                        device_info['serial'] = value
                    elif key == 'ID_PATH':
                        device_info['path'] = value
            
            # 识别设备类型
            device_info['type'] = self.identify_device_type(device_info)
            
            return device_info
            
        except Exception as e:
            print(f"获取设备 {device_path} 信息失败: {e}")
            return None
    
    def identify_device_type(self, device_info: Dict) -> str:
        """识别设备类型"""
        vendor_id = device_info.get('vendor_id', '')
        product_id = device_info.get('product_id', '')
        
        if vendor_id == self.ch340_info['vendor_id'] and product_id == self.ch340_info['product_id']:
            return 'CH340'
        elif vendor_id == self.ch341_info['vendor_id'] and product_id == self.ch341_info['product_id']:
            return 'CH341'
        else:
            return 'Unknown'
    
    def create_symlinks(self, devices: List[Dict]) -> bool:
        """创建符号链接"""
        success = True
        
        for device in devices:
            device_type = device.get('type')
            device_path = device.get('device_path')
            
            if device_type == 'CH340':
                target_link = f"/dev/{self.ch340_info['device_name']}"
            elif device_type == 'CH341':
                target_link = f"/dev/{self.ch341_info['device_name']}"
            else:
                continue
            
            try:
                # 删除已存在的符号链接
                if os.path.exists(target_link) or os.path.islink(target_link):
                    os.unlink(target_link)
                
                # 创建新的符号链接
                os.symlink(device_path, target_link)
                print(f"创建符号链接: {device_path} -> {target_link}")
                
            except Exception as e:
                print(f"创建符号链接失败 {device_path} -> {target_link}: {e}")
                success = False
        
        return success
    
    def print_device_info(self, devices: List[Dict]):
        """打印设备信息"""
        print("\n=== USB设备扫描结果 ===")
        
        ch340_found = False
        ch341_found = False
        
        for device in devices:
            device_type = device.get('type', 'Unknown')
            print(f"\n设备: {device['device_path']}")
            print(f"  类型: {device_type}")
            print(f"  Vendor ID: {device.get('vendor_id', 'N/A')}")
            print(f"  Product ID: {device.get('product_id', 'N/A')}")
            print(f"  厂商: {device.get('vendor_name', 'N/A')}")
            print(f"  产品: {device.get('product_name', 'N/A')}")
            print(f"  序列号: {device.get('serial', 'N/A')}")
            
            if device_type == 'CH340':
                ch340_found = True
                print(f"  分配名称: {self.ch340_info['device_name']}")
            elif device_type == 'CH341':
                ch341_found = True
                print(f"  分配名称: {self.ch341_info['device_name']}")
        
        print("\n=== 设备状态 ===")
        print(f"CH340 (电池电量): {'找到' if ch340_found else '未找到'}")
        print(f"CH341 (STM32通讯): {'找到' if ch341_found else '未找到'}")

def main():
    """主函数"""
    scanner = USBDeviceScanner()
    
    print("开始扫描USB设备...")
    devices = scanner.scan_usb_devices()
    
    if not devices:
        print("未找到任何USB转串口设备")
        return
    
    scanner.print_device_info(devices)
    
    # 创建符号链接
    print("\n创建符号链接...")
    if scanner.create_symlinks(devices):
        print("符号链接创建完成")
    else:
        print("符号链接创建过程中出现错误")

if __name__ == "__main__":
    main()