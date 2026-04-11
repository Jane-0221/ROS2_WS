# 并入 ROS 2 项目的集成说明

## 1. 集成目标

这一版系统不是单个脚本，而是一整条 ROS 2 / MoveIt 工作链：

- 机器人模型
- MoveIt 配置
- 自定义规划接口
- 任务状态机
- mock 感知与调试脚本

如果要合并到另一个 ROS 2 项目，最稳的思路不是“只拷一个 `pick_task_manager.py`”，而是按层接入。

## 2. 最小必需包

如果你要把当前系统完整带走，最少需要这 4 个包：

| 包 | 作用 |
| --- | --- |
| `medipick_simple3_description` | 运行时机器人模型 |
| `medipick_moveit_config` | MoveIt 语义、规划器、控制器配置 |
| `medipick_planning_interfaces` | `PlanToPose` 等自定义 service |
| `medipick_planning_server` | 规划服务封装与任务状态机 |

## 3. 哪些文件必须保持一致

### 模型与语义

- `src/medipick_simple3_description/urdf/simple3_moveit.urdf`
- `src/medipick_moveit_config/config/medipick.srdf`
- `src/medipick_moveit_config/config/kinematics.yaml`
- `src/medipick_moveit_config/config/ompl_planning.yaml`
- `src/medipick_moveit_config/config/joint_limits.yaml`

这些定义了：

- link / joint 关系
- MoveIt group
- IK link / tool link 语义
- OMPL projection 与规划器行为

### 规划服务层

- `src/medipick_planning_server/config/planning_server.yaml`
- `src/medipick_planning_server/scripts/planning_server.py`

这些定义了：

- `pose_link`
- `ik_pose_link`
- `tool_reference_link`
- `tool_to_ik_offset_*`

如果这些和模型不一致，外部传入的末端 pose 会在内部换算错位。

### 任务管理器层

- `src/medipick_planning_server/scripts/pick_task_manager.py`
- `src/medipick_planning_server/scripts/pick_task_flow.py`
- `src/medipick_planning_server/scripts/pick_task_services.py`
- `src/medipick_planning_server/scripts/pick_task_runtime.py`
- `src/medipick_planning_server/scripts/pick_task_shared.py`
- `src/medipick_planning_server/scripts/pick_task_utils.py`

这些文件共同实现了 V3 主链，不应只抽其中一个文件单独搬。

## 4. 建议的合并方式

### 方式 A：整包带走

最稳，也最推荐。

做法：

1. 把上面 4 个包整体复制到你的目标工作区 `src/`
2. 保留当前相对目录结构
3. 在目标工作区重新 `colcon build`
4. 用你自己的 launch 再去包这一层

优点：

- 最不容易丢依赖
- 版本边界清晰
- 出问题时可直接对照当前工作区排查

### 方式 B：并入已有 MoveIt 工程

如果你已经有自己的机器人描述包和 MoveIt config，可以这样接：

1. 先让你的描述包和 MoveIt config 提供与当前系统等价的语义：
   - `pose_link`
   - `ik_pose_link`
   - `tool_reference_link`
   - `mobile_arm` / `arm` / `tool` 这类组语义
2. 再把 `medipick_planning_interfaces` 和 `medipick_planning_server` 并进去
3. 最后把 launch 和参数接上

这种方式的关键不是文件名一致，而是“语义一致”。

## 5. 并入已有 ROS 2 项目时必须对齐的接口

### 5.1 关节与控制器语义

当前任务链默认依赖这些 joint：

- `base_x`
- `base_y`
- `base_theta`
- `raise_joint`
- `r1_joint`
- `r2_joint`
- `r3_joint`
- `r4_joint`
- `r5_joint`
- `r6_joint`
- `sucker_joint`

控制器侧最关键的是：

- `/mobile_arm_controller/follow_joint_trajectory`

如果你的控制器 joint 列表不一样，至少要同步修改：

- `ros2_controllers.yaml`
- `moveit_controllers.yaml`
- `pick_task_runtime.py` 里的 controller joint 过滤逻辑

### 5.2 任务输入输出接口

当前主任务最关键的输入 topic：

- `/medipick/task/target_pose`

当前主任务常用输出：

- `/medipick/task/stage`
- `/medipick/task/event`
- `/medipick/task/base_goal`
- `/medipick/task/pre_insert_pose`
- `/medipick/task/pick_pose`
- `/medipick/task/retreat_pose`

如果你要并入已有感知系统，最常见的做法是：

- 保留任务管理器的内部输出不变
- 只把你的感知结果桥接到 `/medipick/task/target_pose`

### 5.3 规划服务接口

当前任务链内部调用：

- `/medipick_planning_server/plan_to_pose`
- `/compute_fk`
- `/compute_ik`
- `/check_state_validity`
- `/compute_cartesian_path`

也就是说，目标工程里必须有：

- `move_group`
- `planning_server.py`

这两层同时存在。

## 6. 合并步骤建议

### 第一步：只合模型和 MoveIt

先确认下面两件事：

1. `rviz2 + move_group` 能启动
2. `arm` / `mobile_arm` / `whole_body` 规划组能正常用

### 第二步：再合规划服务

确认：

- `/medipick_planning_server/plan_to_pose` 能调用
- 工具偏移换算正确

### 第三步：最后合任务状态机

确认：

- `/medipick/task/target_pose` 能正常驱动
- 状态机能从 `ACQUIRE_TARGET` 走到 `COMPLETED`

## 7. 合并后优先验证什么

最推荐的验证顺序是：

1. 模型和 TF 是否正确
2. MoveIt group 是否正确
3. `plan_to_pose` 是否正确
4. `ACQUIRE_TARGET` 是否能稳定刷新任务几何
5. `PLAN_TO_PRE_INSERT` 是否能走 seeded IK 分支

## 8. 不建议直接照搬的部分

这些内容更适合作为参考，不一定要原样搬过去：

- `mock_vision_publisher.py`
- `mock_pick_path_publisher.py`
- `default_joint_state_publisher.py`
- 根目录随机实验脚本

如果你的目标工程已经有：

- 真正的相机/点云
- 真正的底盘和升降控制
- 自己的启动系统

那么这些 mock 组件可以只留作调试工具。

## 9. 最小集成检查清单

- 模型坐标系前向/上向语义一致
- `raise_joint=0` 语义一致
- `sucker_link` / `r6_link` 关系一致
- `tool_to_ik_offset_*` 已同步
- `mobile_arm` 组包含底盘、升降、右臂
- `/compute_fk` 和 `/compute_ik` 正常返回
- `/medipick_planning_server/plan_to_pose` 正常工作
- `/medipick/task/target_pose` 输入后状态机能进入 `ARM_STOW_SAFE`

## 10. 这版适合作为哪种基线

当前 V3 更适合被当成：

- 一个“能运行、能调试、能做随机实验”的任务规划基线

还不适合直接当成：

- 实机量产前的最终产品版本

因为它现在仍然保留了不少工程上明确可继续优化的点，比如：

- 启动/场景同步屏障
- `pre-insert` 自适应候选
- `r1` 限制的前置代价化
- 站位与柜体几何的自适应
