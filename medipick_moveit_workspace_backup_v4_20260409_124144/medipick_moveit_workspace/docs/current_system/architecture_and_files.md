# 文件架构与代码入口

## 1. 工作区整体结构

当前工作区主要由四个 ROS 包和若干根目录脚本组成：

| 路径 | 角色 |
| --- | --- |
| `src/medipick_simple3_description` | 机器人模型、mesh、URDF、世界文件 |
| `src/medipick_moveit_config` | MoveIt 配置、SRDF、规划器、控制器、RViz |
| `src/medipick_planning_interfaces` | 自定义 service 接口定义 |
| `src/medipick_planning_server` | 规划服务封装、任务管理器、mock 感知与可视化 |
| `scripts/` | 常用启动脚本与环境脚本 |
| `docs/` | 设计文档、迁移记录、当前系统说明 |
| `assets/` | 外部导出的原始机器人模型与资源 |

## 2. 包级职责

### 2.1 `medipick_simple3_description`

这个包名历史上沿用 `simple3`，但当前内容已经是：

- simple4 结构与几何

主要内容：

- `urdf/simple3_moveit.urdf`
- `meshes/`
- `worlds/medipick_test.world.sdf`

它回答的问题是：

- 机器人长什么样
- link/joint 关系是什么
- 碰撞与视觉模型是什么

### 2.2 `medipick_moveit_config`

这是 MoveIt 配置包，主要负责：

- 规划组定义
- 关节限位
- 运动学插件
- OMPL 配置
- ros2_control 控制器
- MoveIt RViz 配置

关键文件：

| 文件 | 作用 |
| --- | --- |
| `config/medipick.srdf` | MoveIt 语义模型与规划组 |
| `config/kinematics.yaml` | IK 配置 |
| `config/ompl_planning.yaml` | OMPL 规划器配置 |
| `config/joint_limits.yaml` | 关节约束 |
| `config/ros2_controllers.yaml` | ros2_control 控制器 |
| `config/moveit_controllers.yaml` | MoveIt 控制器映射 |
| `scripts/default_joint_state_publisher.py` | 无控制器模式下的 joint state 模拟 |

### 2.3 `medipick_planning_interfaces`

这是接口包，职责很单一：

- 提供任务和规划相关的 service 定义

当前主要接口：

- `PlanToPose.srv`
- `UpsertCollisionBox.srv`
- `RemoveCollisionObject.srv`

### 2.4 `medipick_planning_server`

这是当前项目的主逻辑包，主要包含两大块：

1. 规划服务封装
2. 任务状态机

此外还带了若干 mock 节点与调试辅助节点。

## 3. 当前主路径文件图

如果只看当前主链，最重要的文件可以按下面方式理解。

### 模型与 MoveIt 层

| 文件 | 当前角色 |
| --- | --- |
| `src/medipick_simple3_description/urdf/simple3_moveit.urdf` | 当前运行时机器人模型 |
| `src/medipick_moveit_config/config/medipick.srdf` | 规划组与语义约定 |
| `src/medipick_moveit_config/config/kinematics.yaml` | KDL IK 配置 |
| `src/medipick_moveit_config/config/ompl_planning.yaml` | OMPL 规划器与 projection |
| `src/medipick_planning_server/config/planning_server.yaml` | 规划服务默认参数 |

### 规划与任务层

| 文件 | 当前角色 |
| --- | --- |
| `src/medipick_planning_server/scripts/planning_server.py` | MoveIt 服务封装层 |
| `src/medipick_planning_server/scripts/pick_task_manager.py` | 任务状态机外壳、参数、tick 调度 |
| `src/medipick_planning_server/scripts/pick_task_flow.py` | 任务几何、候选选择、阶段规划逻辑 |
| `src/medipick_planning_server/scripts/pick_task_services.py` | IK/FK/Cartesian/PlanToPose 等服务调用封装 |
| `src/medipick_planning_server/scripts/pick_task_runtime.py` | topic 发布、trajectory 执行、debug marker |
| `src/medipick_planning_server/scripts/pick_task_shared.py` | 枚举、dataclass、默认模板姿态 |
| `src/medipick_planning_server/scripts/pick_task_utils.py` | 轨迹、角度、pose、joint state 工具函数 |

### Mock 与调试层

| 文件 | 当前角色 |
| --- | --- |
| `src/medipick_planning_server/scripts/mock_vision_publisher.py` | mock 货架与目标生成 |
| `src/medipick_planning_server/scripts/mock_initial_pose_manager.py` | 初始姿态随机化 |
| `src/medipick_planning_server/scripts/mock_pick_path_publisher.py` | 额外调试轨迹发布 |
| `src/medipick_planning_server/scripts/plan_to_pose_client.py` | 手工调 planning server |

## 4. `pick_task_manager.py` 的拆分边界

当前任务管理器已经不是早期那种所有逻辑全塞在一个文件里的形式，而是按职责拆分成了几个 mixin。

### `pick_task_manager.py`

负责：

- ROS Node 初始化
- 参数声明与读取
- topic / service / action client 创建
- 状态机阶段切换
- 各阶段 `tick` 调度

### `pick_task_flow.py`

负责：

- 任务几何刷新
- `pre-insert` pose 生成
- `base_goal` / `retreat_pose` / 高度带计算
- 插入与撤出阶段的规划策略
- 轨迹偏好检查

### `pick_task_services.py`

负责：

- 调 `/compute_ik`
- 调 `/compute_fk`
- 调 `/check_state_validity`
- 调 `/compute_cartesian_path`
- 调 `/medipick_planning_server/plan_to_pose`
- `seeded IK + joint interpolation` 的预备阶段规划

### `pick_task_runtime.py`

负责：

- 发布阶段、goal、trajectory、marker
- 执行 controller action
- 估计执行后的 achieved pose
- 轨迹的可视化辅助

### `pick_task_shared.py`

负责：

- `PickStage`
- `PlannedStage`
- `PrepareCandidate`
- 默认 stow 模板

### `pick_task_utils.py`

负责：

- joint state 与 trajectory 的转换
- revolute 连续角处理
- 轨迹运动量统计
- pose / quaternion 小工具

## 5. 启动入口的当前建议

### 推荐入口

| 脚本 | 适用场景 |
| --- | --- |
| `scripts/run_moveit_visual_demo.sh` | 当前主链自动演示 |
| `scripts/run_moveit_plan_execute_debug.sh` | RViz 手工规划调试 |
| `scripts/build.sh` | 常用构建 |

### 直接 launch 入口

| 文件 | 作用 |
| --- | --- |
| `src/medipick_planning_server/launch/moveit_mock_demo.launch.py` | 当前主 launch |
| `src/medipick_planning_server/launch/pick_task_manager.launch.py` | 单独起任务管理器 |
| `src/medipick_moveit_config/launch/demo.launch.py` | MoveIt config 侧入口 |

当前最值得优先记住的是：

- `moveit_mock_demo.launch.py`

因为它已经把：

- 模型
- MoveIt
- planning server
- mock vision
- task manager
- RViz

这条主链都串起来了。

## 6. 哪些文件是“当前主路径”，哪些只是辅助

### 当前主路径文件

这些文件是理解当前系统最值得优先看的：

- `src/medipick_simple3_description/urdf/simple3_moveit.urdf`
- `src/medipick_moveit_config/config/medipick.srdf`
- `src/medipick_moveit_config/config/kinematics.yaml`
- `src/medipick_moveit_config/config/ompl_planning.yaml`
- `src/medipick_planning_server/config/planning_server.yaml`
- `src/medipick_planning_server/launch/moveit_mock_demo.launch.py`
- `src/medipick_planning_server/scripts/planning_server.py`
- `src/medipick_planning_server/scripts/pick_task_manager.py`
- `src/medipick_planning_server/scripts/pick_task_flow.py`
- `src/medipick_planning_server/scripts/pick_task_services.py`
- `src/medipick_planning_server/scripts/pick_task_runtime.py`
- `scripts/run_moveit_visual_demo.sh`

### 辅助与历史遗留文件

这些文件可能还有用，但不是当前主链理解的第一优先级：

- `scripts/run_moveit_mock_progress.sh`
- `scripts/run_task_chain_demo.sh`
- `scripts/run_moveit_mock_progress_rrtstar.sh`
- `src/medipick_moveit_config/scripts/controller_parameter_seed.py`
- `src/medipick_moveit_config/scripts/fake_follow_joint_trajectory_server.py`
- `src/medipick_planning_server/launch/planning_demo.launch.py`
- `src/medipick_planning_server/rviz/pose_goal_obstacle_demo.rviz`
- `src/medipick_planning_server/rviz/randomized_pick_demo.rviz`

## 7. 模型资产与运行时模型的关系

当前需要明确区分两层：

### 资产层

- `assets/simple4_urdf/urdf/simple4_urdf.urdf`

这是外部导出的原始模型资产，适合做：

- 坐标检查
- 导出核对
- 与 CAD 导出结果比对

### 运行时层

- `src/medipick_simple3_description/urdf/simple3_moveit.urdf`

这是项目真正拿来跑 MoveIt、控制器和任务状态机的模型。

如果要让项目真正换模型，最终应该改的是运行时层，而不是只改资产层。
