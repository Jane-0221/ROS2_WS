# MediPick 安卓建图遥控台

## 1. 启动机器人桥接服务

```bash
cd /home/jszn/robot_ros2_ws
bash run_mapping_console_bridge.sh
```

默认会在 `0.0.0.0:8000` 提供接口。
同时会通过 mDNS 广播成 `MediPick Mapping Bridge` 服务，安卓端可直接扫描设备名。

## 2. 安卓 App 连接地址

- HTTP: `http://<电脑IP>:8000`
- WebSocket: `ws://<电脑IP>:8000/ws`

如果局域网广播正常，App 会直接列出电脑设备名，不需要手动填 IP。手动地址输入只作为兜底。

## 3. 常用接口

- `GET /health`
- `GET /api/session`
- `POST /api/mapping/start`
- `POST /api/mapping/stop`
- `POST /api/control/mode`
- `POST /api/control/estop`
- `GET /api/map/latest`
- `GET /api/video/latest.jpg`

## 4. 映射控制模式

- `nav`: Nav2 `/cmd_vel`
- `stm32`: STM32 `/medipick/hardware/stm32_base_cmd_vel`
- `app`: 安卓 App `/medipick/app/cmd_vel`

底盘统一消费 `/medipick/base/cmd_vel_muxed`。
