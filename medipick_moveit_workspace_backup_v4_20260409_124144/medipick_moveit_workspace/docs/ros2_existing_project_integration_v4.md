# 将当前版本并入现存 ROS 2 项目的指导

## 目的

这份文档面向一个很具体的目标：

- 你已经有自己的 ROS 2 工程
- 你希望把当前这套 Medipick V4 任务规划链融合进去
- 你需要的是“怎么并、保留什么、替换什么、先验证什么”

这份文档比 [ros2_integration_guide.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/current_system/ros2_integration_guide.md) 更偏落地执行；后者仍然建议保留作为现状参考。

## 当前版本包含什么

当前可运行链路不是一个脚本，而是四层叠起来的系统：

1. 机器人模型与 MoveIt 语义
2. 规划服务层
3. 任务状态机层
4. mock 感知、实验和可视化辅助层

对应最小必需包：

- `medipick_simple3_description`
- `medipick_moveit_config`
- `medipick_planning_interfaces`
- `medipick_planning_server`

如果你想把“当前算法能力”完整带走，这四个包要整体带走，不建议只抽一个 `pick_task_manager.py`。

## 推荐的融合方式

### 方式 A：整包接入

这是最稳的方式，推荐优先采用。

做法：

1. 将以下 4 个包复制到你的目标工作区 `src/`：
   - `src/medipick_simple3_description`
   - `src/medipick_moveit_config`
   - `src/medipick_planning_interfaces`
   - `src/medipick_planning_server`
2. 保留包名和目录结构不变。
3. 在目标工作区执行 `rosdep install` 和 `colcon build`。
4. 再用你自己的 launch 去包裹它们。

这种方式的优点：

- 依赖边界清晰
- 和当前版本最一致
- 后续调试时能直接对照本工作区文档和代码

### 方式 B：并入已有机器人 / MoveIt 工程

如果你已经有自己的机器人描述包和 MoveIt config，也可以这样做：

1. 保留你自己的描述包和 MoveIt config。
2. 将 `medipick_planning_interfaces` 与 `medipick_planning_server` 并入你的工程。
3. 把当前系统依赖的“语义”映射到你的机器人上。

这里最关键的是“语义一致”，不是“文件名一致”。

## 并入前必须先判断的一件事

当前任务链假设的是这种机器人语义：

- 底盘可平面移动：`base_x` / `base_y` / `base_theta`
- 升降轴：`raise_joint`
- 主机械臂：`r1_joint` 到 `r6_joint`
- 末端工具：`sucker_link`
- 当前任务是吸盘式取物，不是夹爪抓取

如果你的目标工程满足的是同类语义，那么可以直接融合当前算法。

如果你的目标工程不满足，比如：

- 没有可规划的底盘自由度
- 升降不是 `raise_joint` 语义
- 末端不是吸盘
- `pose_link / ik_pose_link / tool_reference_link` 关系完全不同

那就不是简单合并，而是“以当前项目为参考重新适配”。

## 哪些文件必须原样保留或严格对齐

### 模型与 MoveIt 语义层

这些定义了机器人运动学和规划组，必须一起看：

- [simple3_moveit.urdf](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_simple3_description/urdf/simple3_moveit.urdf)
- [medipick.srdf](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_moveit_config/config/medipick.srdf)
- [kinematics.yaml](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_moveit_config/config/kinematics.yaml)
- [ompl_planning.yaml](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_moveit_config/config/ompl_planning.yaml)
- [joint_limits.yaml](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_moveit_config/config/joint_limits.yaml)

你至少要保证这些语义一致：

- `pose_link = sucker_link`
- `ik_pose_link = r6_link`
- `tool_reference_link = sucker_link`
- 存在 `arm`、`mobile_arm`、`tool` 等关键规划组
- `raise_joint = 0` 的几何语义正确

### 规划服务层

- [planning_server.yaml](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/config/planning_server.yaml)
- [planning_server.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/planning_server.py)

这层负责把外部传进来的末端目标，正确换算成内部 IK / FK 语义。

如果这里和模型不一致，后果通常不是“完全报错”，而是更危险的“规划出来但位姿偏了”。

### 任务状态机层

- [pick_task_manager.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_manager.py)
- [pick_task_flow.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_flow.py)
- [pick_task_services.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_services.py)
- [pick_task_runtime.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_runtime.py)
- [pick_task_shared.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_shared.py)
- [pick_task_utils.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_utils.py)

这一层不要只抽其中一个文件；它们已经拆成互相依赖的主链结构了。

## 当前系统依赖的 ROS 2 接口

### 任务输入

最关键的输入只有一个：

- `/medipick/task/target_pose`

如果你有自己的感知系统，最简单的融合方式就是：

- 让你的感知系统算出目标位姿
- 桥接或直接发布到 `/medipick/task/target_pose`

### 任务输出

这几个 topic 最有用：

- `/medipick/task/stage`
- `/medipick/task/event`
- `/medipick/task/base_goal`
- `/medipick/task/pre_insert_pose`
- `/medipick/task/pick_pose`
- `/medipick/task/retreat_pose`
- `/medipick/task/achieved_pose`

### 任务服务

- `/medipick/task/start`
- `/medipick/task/reset`
- `/medipick/task/mark_base_arrived`
- `/medipick/task/mark_lift_arrived`

### MoveIt / planning 相关服务

任务链内部依赖：

- `/medipick_planning_server/plan_to_pose`
- `/compute_fk`
- `/compute_ik`
- `/check_state_validity`
- `/compute_cartesian_path`

也就是说，你的目标系统最终必须有：

- `move_group`
- `planning_server.py`

这两层一起在线。

## 控制器与执行侧必须对齐什么

当前默认控制器语义是：

- `mobile_arm_controller`
- `tool_controller`
- `head_controller`

其中对主任务最关键的是：

- `/mobile_arm_controller/follow_joint_trajectory`

如果你的控制器 joint 列表或名字不同，必须同步改这几处：

- `moveit_controllers.yaml`
- `ros2_controllers.yaml`
- [pick_task_runtime.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_runtime.py) 中的 controller joint 过滤逻辑

如果不改，常见现象就是：

- MoveIt 能规划
- 轨迹执行时 controller 拒绝
- 日志报 “Incoming joint xxx doesn't match the controller's joints”

## 建议的融合步骤

### 第一步：只合模型和 MoveIt

目标：

- `robot_state_publisher` 正常
- `move_group` 正常
- RViz MotionPlanning 可用

这一步先不要接任务状态机。

建议验证：

1. 能启动 RViz
2. `arm` 组可以手工 `Plan / Execute`
3. `mobile_arm` 组可以手工 `Plan / Execute`

### 第二步：接入 planning server

目标：

- `/medipick_planning_server/plan_to_pose` 正常工作
- 目标 pose 和工具偏移换算正确

建议验证：

1. 手工构造一个目标 pose
2. 调 `plan_to_pose`
3. 检查末端最终到位误差和姿态是否符合预期

### 第三步：接入任务状态机

目标：

- `/medipick/task/target_pose` 能驱动完整主流程

建议验证：

1. 只给一个静态目标 pose
2. 看是否能从 `ACQUIRE_TARGET` 走到 `COMPLETED`
3. 观察是否会卡在：
   - `ARM_STOW_SAFE`
   - `LIFT_TO_BAND`
   - `PLAN_TO_PRE_INSERT`
   - `INSERT_AND_SUCTION`

### 第四步：再替换 mock 组件

当前这些 mock 组件可以先保留，方便并入时做回归：

- `mock_vision_publisher.py`
- `mock_pick_path_publisher.py`
- `default_joint_state_publisher.py`
- 随机场景与实验脚本

等主链稳定后，再换成你的真实感知、真实控制和真实系统 launch。

## 推荐的最小融合目录结构

你的目标工作区建议至少有：

```text
src/
  your_robot_description/
  your_moveit_config/
  medipick_planning_interfaces/
  medipick_planning_server/
  medipick_simple3_description/        # 如果直接沿用当前模型
  medipick_moveit_config/              # 如果直接沿用当前 MoveIt 配置
```

如果你不沿用当前模型和 MoveIt 配置，那你自己的 `description` / `moveit_config` 就必须提供与当前任务链等价的语义。

## 推荐的 launch 融合方式

最简单的融合思路是：保留三层 launch 分工。

1. 机器人与 MoveIt 基础层
   - robot description
   - `robot_state_publisher`
   - `move_group`
   - controllers

2. 规划服务层
   - `planning_server.py`

3. 任务层
   - `pick_task_manager.py`
   - 你的感知桥接节点

不要把所有逻辑都硬塞进一个超长 launch，后面会很难排查。

## 一个实际可执行的迁移顺序

### 路线 1：先复制当前工程，再慢慢替换

这是最稳的路线。

1. 先把当前版本完整复制进你的工作区。
2. 确保当前演示仍然能跑。
3. 先替换目标 pose 来源。
4. 再替换模型。
5. 再替换控制器。
6. 最后替换场景 / 点云 / 实机接口。

优点是每一步都可回退。

### 路线 2：以你的工程为主，挑功能接入

这是更自由但风险更高的路线。

1. 保留你的模型和 MoveIt。
2. 只引入 `planning_interfaces + planning_server + task manager`。
3. 手工对齐所有 joint、group、tool offset、controller 名字。

这条路线更容易出现“系统启动了，但行为不对”的隐性问题。

## 当前版本的主流程

当前主流程是：

`ACQUIRE_TARGET -> ARM_STOW_SAFE -> BASE_ENTER_WORKSPACE -> LIFT_TO_BAND -> SELECT_PRE_INSERT -> PLAN_TO_PRE_INSERT -> INSERT_AND_SUCTION -> SAFE_RETREAT -> return_to_stow`

如果你把它并入现存项目，最容易出问题的是这三段：

### `ARM_STOW_SAFE`

要求：

- 你的机器人能有一组安全收臂姿态
- 底盘运动前，这组姿态必须可靠到位

### `PLAN_TO_PRE_INSERT`

要求：

- `mobile_arm` 或 `arm` 至少有一套局部可行解
- 当前系统会限制底盘在这个阶段的运动，不允许它大幅重构

### `INSERT_AND_SUCTION`

要求：

- 末端到目标已经足够接近
- 插入阶段优先走局部前插，而不是重新做全局 OMPL

## 你最需要替换的通常不是算法，而是这些接口

真实工程里，通常优先替换的是：

- 目标 pose 来源
- 底盘到位反馈
- 升降到位反馈
- 真实 controller
- 真正的场景 / 碰撞环境

通常不需要第一时间改的是：

- 任务状态机主阶段定义
- `pre-insert` / insert / retreat 的基本策略

## 最小验证清单

并入后至少按这个顺序验：

1. 模型方向正确
2. `raise_joint = 0` 语义正确
3. `sucker_link` / `r6_link` 偏移正确
4. `arm` / `mobile_arm` 规划组正确
5. `plan_to_pose` 对工具目标换算正确
6. `target_pose` 输入后能进 `ARM_STOW_SAFE`
7. 能走到 `PLAN_TO_PRE_INSERT`
8. 能成功 `INSERT_AND_SUCTION`
9. 能 `SAFE_RETREAT`
10. 最终能 `COMPLETED`

## 并入后最常见的问题

### 1. 规划能出，执行被 controller 拒绝

通常是 joint 列表不一致。

### 2. 末端看着到了，但实际偏了

通常是：

- `pose_link`
- `ik_pose_link`
- `tool_reference_link`
- `tool_to_ik_offset_*`

这组参数没对齐。

### 3. `PLAN_TO_PRE_INSERT` 老失败

通常是：

- `mobile_arm` 组定义不对
- 底盘 / 升降自由度没正确接进 MoveIt
- 起始站位与收臂姿态不适配

### 4. 插入阶段动作很奇怪

通常是：

- 目标工程里没有保留“局部前插优先”的策略
- 让 `mobile_arm` 在插入阶段重新做高自由度全局 OMPL

## 推荐先看哪些文档

建议按这个顺序一起看：

1. [docs/current_system/flow_and_runtime.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/current_system/flow_and_runtime.md)
2. [docs/current_system/technical_details.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/current_system/technical_details.md)
3. [docs/current_system/architecture_and_files.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/current_system/architecture_and_files.md)
4. [docs/current_system/ros2_integration_guide.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/current_system/ros2_integration_guide.md)

## 结论

如果你要的是“尽快把当前能力并入现有 ROS 2 项目”，最推荐的做法是：

- 先整包带走四个核心包
- 保留当前模型 / MoveIt / planning server / task manager 的语义
- 先替换输入输出接口，不急着第一天就重写内部算法

这样成本最低，也最不容易把已经验证过的主链打散。
