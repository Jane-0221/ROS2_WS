# MediPick 场景建图与导航流程

这套工作区当前更接近下面这条链路：

- `Orbbec RGB-D 相机 + IMU`
- `RTAB-Map` 负责视觉建图/重定位
- `robot_localization EKF` 负责融合视觉里程计和 IMU
- `Nav2` 直接使用 RTAB-Map 发布的 `/map`

所以它不是经典的“2D 激光 SLAM + AMCL”路线。对这个仓库来说，更直接的流程是：

1. 手动遥控机器人绕场建图
2. RTAB-Map 把场景存进数据库
3. 下次启动时加载同一个数据库做定位
4. Nav2 在 RTAB-Map 当前位置和 `/map` 上做导航

## 两种底盘控制模式

- `base_control_mode:=stm32`
  - 适合遥控器命令先到 STM32，再由 STM32 发布底盘速度
  - `wheeltec_chassis_node` 会订阅 `/medipick/hardware/stm32_base_cmd_vel`
- `base_control_mode:=nav`
  - 适合在 ROS 主机上直接跑 `joy_node + teleop_twist_joy`
  - `wheeltec_chassis_node` 会直接订阅 `/cmd_vel`

## 1. 编译并加载环境

```bash
cd /home/jszn/robot_ros2_ws
colcon build --packages-select medipick_planning_server robot_hardware --symlink-install
source install/setup.bash
```

## 2. 建图

如果要从零开始建一张新图，先删旧数据库，或者换一个新的数据库路径：

```bash
rm -f ~/.ros/medipick_scene_mapping.db
```

启动轻量建图链路：

```bash
ros2 launch medipick_planning_server scene_mapping_navigation.launch.py \
  localization:=false \
  start_nav2:=false \
  start_rtabmap_viz:=true \
  delete_db_on_start:=true \
  base_control_mode:=stm32
```

如果你不是用 STM32 遥控，而是 USB 手柄直接接工控机，开第二个终端：

```bash
source /home/jszn/robot_ros2_ws/install/setup.bash
ros2 launch robot_hardware joystick_teleop.launch.py \
  joy_config:=ps3-holonomic \
  cmd_vel_topic:=/cmd_vel
```

如果你的手柄不是 `ps3-holonomic` 映射，就把 `joy_config` 或 `config_filepath` 改掉。

建图时建议：

- 先原地慢慢转一圈，让 RTAB-Map 建立初始特征
- 再沿可通行区域绕场走一圈
- 最好形成回环，不要只走一条单程线
- 速度放慢，避免纯纹理少、快速转弯和强反光区域导致丢跟踪

## 3. 建图时检查

至少确认下面几个话题是活的：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /odometry/filtered --once
ros2 topic echo /map --once
```

如果 `/map` 没有，通常先看：

- 深度图和相机内参有没有出来
- `/camera/gyro_accel/sample` 有没有数据
- `base_link -> camera_link` 静态 TF 是否正确

## 4. 可选：导出 2D 栅格地图

RTAB-Map 的核心地图是数据库文件；如果还想导出 Nav2 常见的 `.yaml + .pgm` 静态地图：

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/medipick_room --ros-args -r map:=/map
```

## 5. 基于已有场景做定位和导航

重新启动，但这次切到定位模式，并使用同一个数据库：

```bash
ros2 launch medipick_planning_server scene_mapping_navigation.launch.py \
  localization:=true \
  start_nav2:=true \
  start_rtabmap_viz:=false \
  delete_db_on_start:=false \
  base_control_mode:=nav \
  rtabmap_database_path:=~/.ros/medipick_scene_mapping.db
```

如果还是走 STM32 遥控链路，就把 `base_control_mode:=nav` 改成 `base_control_mode:=stm32`。

`localization:=true` 的含义是：

- RTAB-Map 读取已有数据库
- 不再继续增量建新图
- 用已有场景做重定位
- Nav2 直接用 RTAB-Map 维护的 `/map` 和 `map -> world` 变换

## 6. 这个仓库里“保存地图后导航”的准确说法

更准确地说，不是：

- “随便拿遥控器转一圈，就自动变成 AMCL 导航”

而是：

- “用遥控器驱动底盘做 RTAB-Map 视觉建图”
- “把场景保存在 RTAB-Map 数据库里”
- “下次加载同一个数据库做视觉重定位”
- “Nav2 在 RTAB-Map 提供的位置和地图上导航”

## 7. 常见问题

- `delete_db_on_start:=true` 只适合重新建图，不适合定位模式
- 如果 `base_control_mode` 设错，底盘会收不到速度
- 如果画面正常但地图漂，优先检查相机安装高度、俯仰角、IMU 数据和行驶速度
- 如果场景变化太大，视觉重定位会变差
