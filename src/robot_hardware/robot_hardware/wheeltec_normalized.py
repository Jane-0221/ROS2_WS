#!/usr/bin/env python3
"""
WHEELTEC机器人 - 方法C：归一化三轴速度比例控制器
为强化学习提供的连续动作空间接口
文件名: wheeltec_normalized.py
依赖: wheeltec_protocol.py (基础协议层)
"""

import time
import logging
from typing import Tuple, Optional
from wheeltec_protocol import WheeltecBaseProtocol, ControlProtocol

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("WheeltecNormalized")

class MethodC_Controller:
    """
    方法C: 归一化三轴速度比例控制器
    为强化学习提供的连续动作空间接口
    输出范围: kx, ky, kz ∈ [-1.0, 1.0]
    """

    def __init__(
        self,
        protocol: ControlProtocol = ControlProtocol.SERIAL,
        robot_type: str = "diff",
        max_speeds: Tuple[float, float, float] = (500.0, 500.0, 1.0),
        **connection_params
    ):
        """
        初始化归一化速度控制器

        Args:
            protocol: 控制协议 (SERIAL 或 CAN)
            robot_type: 底盘类型 ('diff':差速, 'ackermann':阿克曼, 'omni':全向)
            max_speeds: 最大速度 (vx_max_mmps, vy_max_mmps, vz_max_radps)
            **connection_params: 连接参数
                - 串口: port, baudrate=115200, timeout=1.0
                - CAN: interface='pcan', channel=1, bitrate=1000000
        """
        self.robot_type = robot_type
        self.max_speeds = max_speeds  # (vx_max, vy_max, vz_max)

        # 初始化底层协议
        self.base_protocol = WheeltecBaseProtocol(
            protocol=protocol,
            robot_type=robot_type,
            **connection_params
        )

        # 当前速度比例
        self.current_kx = 0.0
        self.current_ky = 0.0
        self.current_kz = 0.0

        logger.info(f"初始化归一化速度控制器: {robot_type}底盘")
        logger.info(f"最大速度: vx={max_speeds[0]}mm/s, vy={max_speeds[1]}mm/s, vz={max_speeds[2]}rad/s")

    def connect(self) -> bool:
        """连接到底盘"""
        return self.base_protocol.connect()

    def execute_action(
        self,
        kx: float,
        ky: float = 0.0,
        kz: float = 0.0,
        duration: float = 0.0
    ) -> bool:
        """
        执行归一化速度动作 (核心API)

        Args:
            kx: X轴速度比例 [-1.0, 1.0]，正数前进，负数后退
            ky: Y轴速度比例 [-1.0, 1.0]，正数左移，负数右移
            kz: Z轴角速度比例 [-1.0, 1.0]，正数逆时针，负数顺时针
            duration: 持续时间(秒)，0表示持续执行直到调用stop()

        Returns:
            bool: 执行成功返回True
        """
        # 限制比例在[-1, 1]范围内
        kx = max(-1.0, min(1.0, kx))
        ky = max(-1.0, min(1.0, ky))
        kz = max(-1.0, min(1.0, kz))

        # 保存当前速度比例
        self.current_kx = kx
        self.current_ky = ky
        self.current_kz = kz

        # 转换为实际速度
        vx = kx * self.max_speeds[0]
        vy = ky * self.max_speeds[1]
        vz = kz * self.max_speeds[2]

        # 根据底盘类型调整
        if self.robot_type in ['diff', 'ackermann']:
            if vy != 0:
                logger.debug(f"{self.robot_type}底盘不支持Y轴移动，Y轴分量被忽略")
            vy = 0.0

        # 发送速度指令
        success = self.base_protocol.send_velocity(vx, vy, vz)

        if success:
            logger.info(f"执行动作: kx={kx:.2f}, ky={ky:.2f}, kz={kz:.2f}")
            logger.info(f"实际速度: vx={vx:.1f}mm/s, vy={vy:.1f}mm/s, vz={vz:.3f}rad/s")

            # 如果有持续时间，等待后停止
            if duration > 0:
                time.sleep(duration)
                self.stop()

        return success

    def execute_discrete_for_rl(self, action_idx: int, action_duration: float = 0.2) -> bool:
        """
        为离散动作RL算法提供的便捷接口
        将动作索引映射到速度比例

        Args:
            action_idx: 动作索引 (0-10)
                0: 停止
                1: 前进 (kx=1.0)
                2: 后退 (kx=-1.0)
                3: 左移 (ky=1.0)
                4: 右移 (ky=-1.0)
                5: 左前 (kx=0.7, ky=0.7)
                6: 右前 (kx=0.7, ky=-0.7)
                7: 左后 (kx=-0.7, ky=0.7)
                8: 右后 (kx=-0.7, ky=-0.7)
                9: 原地左转 (kz=1.0)
                10: 原地右转 (kz=-1.0)
            action_duration: 动作持续时间

        Returns:
            bool: 执行成功返回True
        """
        # 动作映射表
        action_mapping = [
            (0.0, 0.0, 0.0),    # 0: 停止
            (1.0, 0.0, 0.0),    # 1: 前进
            (-1.0, 0.0, 0.0),   # 2: 后退
            (0.0, 1.0, 0.0),    # 3: 左移
            (0.0, -1.0, 0.0),   # 4: 右移
            (0.7, 0.7, 0.0),    # 5: 左前
            (0.7, -0.7, 0.0),   # 6: 右前
            (-0.7, 0.7, 0.0),   # 7: 左后
            (-0.7, -0.7, 0.0),  # 8: 右后
            (0.0, 0.0, 1.0),    # 9: 原地左转
            (0.0, 0.0, -1.0),   # 10: 原地右转
        ]

        if action_idx < 0 or action_idx >= len(action_mapping):
            logger.error(f"无效的动作索引: {action_idx}，有效范围: 0-{len(action_mapping)-1}")
            return False

        kx, ky, kz = action_mapping[action_idx]
        return self.execute_action(kx, ky, kz, action_duration)

    def get_current_action(self) -> Tuple[float, float, float]:
        """获取当前执行的动作比例"""
        return (self.current_kx, self.current_ky, self.current_kz)

    def stop(self) -> bool:
        """停止"""
        self.current_kx = 0.0
        self.current_ky = 0.0
        self.current_kz = 0.0
        return self.base_protocol.stop()

    def close(self):
        """关闭连接"""
        self.base_protocol.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def test_method_c():
    """测试方法C: 归一化速度比例控制"""
    print("测试方法C: 归一化三轴速度比例控制")
    print("=" * 50)

    # 创建控制器
    controller = MethodC_Controller(
        protocol=ControlProtocol.SERIAL,
        robot_type="diff",
        max_speeds=(400.0, 400.0, 0.8),  # 最大速度
        port="COM3"  # 根据实际情况修改
    )

    with controller:
        print("连接成功，开始归一化速度测试...")

        # 测试1: 前进 (50%速度)
        print("\n1. 前进 (kx=0.5) 持续1秒")
        controller.execute_action(kx=0.5, duration=1.0)
        time.sleep(0.3)

        # 测试2: 后退并右转
        print("2. 后退并右转 (kx=-0.3, kz=-0.6) 持续1.5秒")
        controller.execute_action(kx=-0.3, kz=-0.6, duration=1.5)
        time.sleep(0.3)

        # 测试3: 原地左转
        print("3. 原地左转 (kz=0.8) 持续1秒")
        controller.execute_action(kz=0.8, duration=1.0)
        time.sleep(0.3)

        # 测试4: 斜向移动 (全向底盘有效)
        if controller.robot_type == 'omni':
            print("4. 右前移动 (kx=0.6, ky=-0.6) 持续1秒")
            controller.execute_action(kx=0.6, ky=-0.6, duration=1.0)
            time.sleep(0.3)

        # 测试5: RL离散动作接口
        print("5. 测试RL离散动作接口")
        print("  动作1: 前进")
        controller.execute_discrete_for_rl(1, 0.5)  # 前进0.5秒
        time.sleep(0.3)

        print("  动作9: 原地左转")
        controller.execute_discrete_for_rl(9, 0.5)  # 原地左转0.5秒
        time.sleep(0.3)

        print("  动作2: 后退")
        controller.execute_discrete_for_rl(2, 0.5)  # 后退0.5秒

        # 获取当前动作
        kx, ky, kz = controller.get_current_action()
        print(f"\n当前动作比例: kx={kx:.2f}, ky={ky:.2f}, kz={kz:.2f}")

        print("\n✅ 归一化速度测试完成!")


def quick_demo():
    """快速演示: 前进3cm -> 后退3cm -> 停止"""
    print("快速演示: 前进3cm -> 后退3cm -> 停止")
    print("=" * 40)

    # 创建控制器
    controller = MethodC_Controller(
        protocol=ControlProtocol.SERIAL,
        robot_type="diff",
        max_speeds=(300.0, 300.0, 0.6),
        port="COM3"
    )

    with controller:
        print("连接成功，开始演示...")

        # 前进3cm: 距离30mm, 速度150mm/s, 时间0.2秒
        print("\n前进3cm (kx=0.5, 持续0.2秒)")
        controller.execute_action(kx=0.5, duration=0.2)
        time.sleep(0.3)

        # 后退3cm
        print("后退3cm (kx=-0.5, 持续0.2秒)")
        controller.execute_action(kx=-0.5, duration=0.2)
        time.sleep(0.3)

        # 停止
        print("停止")
        controller.stop()

        print("\n✅ 演示完成!")


if __name__ == "__main__":
    """主程序入口"""
    print("WHEELTEC 归一化速度控制器测试程序")
    print("=" * 50)
    print("选择测试模式:")
    print("1. 完整测试 (方法C所有功能)")
    print("2. 快速演示 (前进3cm -> 后退3cm)")
    print("3. 自定义测试")

    try:
        choice = input("\n请输入选择 (1-3, 默认1): ").strip()

        if choice == "" or choice == "1":
            test_method_c()
        elif choice == "2":
            quick_demo()
        elif choice == "3":
            # 自定义测试
            print("\n自定义测试模式")
            print("请修改代码中的参数进行测试")
        else:
            print("无效选择，运行完整测试")
            test_method_c()

    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    except Exception as e:
        print(f"程序运行错误: {e}")