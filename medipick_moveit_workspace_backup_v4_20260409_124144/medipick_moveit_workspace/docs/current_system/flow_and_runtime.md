# 主流程与运行时关系

## 1. 当前状态机主流程

当前任务管理器采用定时 `tick` 驱动的离散状态机，阶段定义在：

- `src/medipick_planning_server/scripts/pick_task_shared.py`

调度与执行主循环在：

- `src/medipick_planning_server/scripts/pick_task_manager.py`

当前主阶段如下。

| 阶段 | 作用 | 关键输入 | 关键输出 | 成功判据 |
| --- | --- | --- | --- | --- |
| `ACQUIRE_TARGET` | 从最新目标位姿刷新任务几何 | `/medipick/task/target_pose`、当前 `joint_states` | `base_goal`、`pre_insert_pose`、`retreat_pose`、目标高度带 | 已成功计算几何参数 |
| `ARM_STOW_SAFE` | 先收臂，保证底盘运动安全 | 当前关节状态、`transit_stow_v0` 模板 | 收臂轨迹 | 关节状态进入 stow 容差 |
| `BASE_ENTER_WORKSPACE` | 底盘进入可工作区域 | `base_goal_pose` | 底盘轨迹 | 底盘执行完成且被接受为到位 |
| `LIFT_TO_BAND` | 升降进入目标高度带 | `lift_target_center`、高度带上下界 | 升降轨迹或升降指令 | `raise_joint` 进入高度带 |
| `SELECT_PRE_INSERT` | 选择柜口外 `pre-insert` 位姿 | `T_pick`、柜口入口平面 | 一小队 `PrepareCandidate` | 当前配置下能生成至少一个柜口外候选 |
| `PLAN_TO_PRE_INSERT` | 到达选中的 `pre-insert` | 当前 joint state、选中 pose、小候选队列 | 预备轨迹 | 规划成功、`r1` 运动幅度合理、FK 到位 |
| `INSERT_AND_SUCTION` | 从 `pre-insert` 插入到目标吸取位姿 | 当前 joint state、目标 pose | 插入轨迹 | 规划成功、末端到位、`r1` 幅度合理 |
| `SAFE_RETREAT` | 抽出末端并离柜 | 当前 joint state、`retreat_pose` | 撤出轨迹、回 stow 轨迹 | 撤出成功，可选回 stow 成功 |
| `COMPLETED` | 成功结束 | 无 | 事件 | 任务闭环完成 |
| `FAILED` | 失败结束 | 无 | 错误事件 | 任一阶段失败 |

## 2. 阶段详细语义

### `ACQUIRE_TARGET`

当前实现会在这里统一刷新任务几何：

- 计算底盘目标位姿
- 计算目标高度中心和高度带
- 计算默认 `pre-insert` 位姿
- 计算撤退位姿

并且当前已经额外加入了进入几何计算前的轻量屏障：

- 目标 pose 必须在短时间内保持稳定
- `ACQUIRE_TARGET` 自身要经过一个短暂 scene settle 窗口
- `plan_to_pose / compute_fk / compute_ik / check_state_validity / compute_cartesian_path` 必须已就绪

这样做是为了减少随机实验里那类：

- 刚启动就调 `/compute_fk`
- 结果被 scene/TF 同步问题拖住
- 阶段长时间卡在 `ACQUIRE_TARGET`

核心逻辑在：

- `pick_task_flow.py::_refresh_task_geometry`
- `pick_task_manager.py::_tick_observe`

### `ARM_STOW_SAFE`

这是当前 V3 的硬规则：

- 底盘进入工作区前，手臂必须先收回
- 这个阶段只服务于底盘安全通行
- 它不承担抓取准备和预对齐职责

当前默认模板姿态叫：

- `transit_stow_v0`

定义在：

- `pick_task_shared.py`

### `BASE_ENTER_WORKSPACE`

这个阶段只做粗定位，不追求末端精对齐：

- 底盘轨迹由简单的关节轨迹生成
- 当前演示里可以自动接受“底盘已到位”
- 更真实的系统可以改成外部确认或更宽容差检查

### `LIFT_TO_BAND`

这是 V3 的一个关键改动：

- 升降不是追单点，而是追一个高度带
- 只要 `raise_joint` 进入 `[band_min, band_max]`，阶段就算完成

这样做是因为当前升降机构更像：

- 单自由度粗调机构
- 依赖实时回读闭环
- 不适合被当成精细插入阶段里的自由规划关节

### `SELECT_PRE_INSERT`

这里的目标不是重规划，而是选出一个柜口外的末端预备位姿。

当前版本为了提速，已经被收成“小候选 pose 选择”模式：

- 默认 `pre_insert_select_pose_only = true`
- 默认 `pre_insert_candidate_count = 3`
- `SELECT_PRE_INSERT` 只负责快速筛出 2-3 个柜口外 pose
- 不在这个阶段做重型 MoveIt 重规划

这也是你在 RViz 里看到黄色箭头的来源之一。当前更接近：

- 先确定一个很小的柜口外候选队列
- 再把真正的规划放到下一阶段

### `PLAN_TO_PRE_INSERT`

这是当前主流程里最关键的稳定性改进点。

现在的优先顺序是：

1. 先尝试 `seeded IK + 短 joint-space 插值轨迹`
2. 先对 seeded IK 终点做 `r1` 阶段位移前置筛选
3. 对这条插值轨迹做状态有效性采样检查
4. 再做 `r1` 阶段运动幅度检查
5. 再做 FK 到位检查
6. 如果失败，再 fallback 到传统 pose-goal 规划
7. 如果当前候选还是失败，再自动切换到队列里的下一个 `pre-insert` 候选

这样做的原因是：

- 直接对 `pre-insert` 做 pose-goal 规划时，MoveIt 有时会选到一个不合理的 IK 分支
- 这会导致 `r1_joint` 为了到达同一个末端 pose 绕很大的圈
- 用“当前状态附近的 IK”做 seed，更容易得到连续、自然的解
- 一个柜口外 pose 不通，不代表旁边 2-3 cm 的 pose 也一定不通

### `INSERT_AND_SUCTION`

当前插入阶段的逻辑是：

1. 先尝试短距离 Cartesian path
2. 如果 Cartesian 成功，直接使用
3. 如果 Cartesian 失败，并且 `final_use_cartesian = false`，先尝试 `insert_group_name` 的 OMPL pose-goal 规划
4. 如果主插入组失败，再尝试 `insert_fallback_group_name`，当前默认回退到 `mobile_arm`

这使得插入阶段既保留了：

- 柜内短程直推的优势

又保留了：

- 在 Cartesian 不足时仍能 fallback 的能力

### `SAFE_RETREAT`

撤出阶段与插入阶段思路类似：

1. 优先 Cartesian 撤退
2. 如需要可 fallback 到普通规划
3. 撤出后默认执行 `return_to_stow`

当前已经不再强制绕回旧的固定 `complete pose`。

## 3. 当前 `pre-insert` 的实际实现状态

当前 `pre-insert` 还不是最终“全候选评估版”，而是一个偏工程化的稳定版本。

现状是：

- 候选参数生成能力还保留
- 默认只保留一个很小的候选队列
- `SELECT_PRE_INSERT` 阶段只选 pose，不做重型重评估
- `PLAN_TO_PRE_INSERT` 阶段才真正规划
- 预备位姿必须位于柜口外

这套做法的优点是：

- 快
- 好调
- 行为可解释

缺点是：

- 还没有把 `pre-insert` 候选搜索完全做成鲁棒的局部搜索器

## 4. 运行时通信关系

### 4.1 主要节点

| 节点 | 作用 |
| --- | --- |
| `move_group` | MoveIt 核心规划与场景服务 |
| `medipick_planning_server` | 对 MoveIt 服务做二次封装，提供 `PlanToPose` 等接口 |
| `medipick_pick_task_manager` | V3 状态机与任务调度 |
| `medipick_mock_vision_publisher` | mock 货架与目标发布 |
| `default_joint_state_publisher` | 无控制器模式下的 joint state 模拟器 |
| `robot_state_publisher` | 从 URDF 发布 TF |
| `rviz2` | 可视化与手工调试 |

### 4.2 任务管理器发布的主要 topic

| Topic | 类型 | 作用 |
| --- | --- | --- |
| `/medipick/task/stage` | `std_msgs/String` | 当前阶段 |
| `/medipick/task/event` | `std_msgs/String` | 阶段事件与状态变化 |
| `/medipick/task/base_goal` | `PoseStamped` | 底盘目标位姿 |
| `/medipick/task/lift_target_height` | `Float64` | 升降目标高度 |
| `/medipick/task/prepare_pose` | `PoseStamped` | 历史命名保留的预备 pose |
| `/medipick/task/pre_insert_pose` | `PoseStamped` | 当前柜口外预备 pose |
| `/medipick/task/final_pose` | `PoseStamped` | 历史命名保留的最终 pose |
| `/medipick/task/pick_pose` | `PoseStamped` | 目标吸取位姿 |
| `/medipick/task/retreat_pose` | `PoseStamped` | 撤出 pose |
| `/medipick/task/achieved_pose` | `PoseStamped` | 根据 FK 估计的执行到位 pose |
| `/medipick/task/stage_path` | `nav_msgs/Path` | `pre_insert -> pick -> retreat` 路径 |
| `/medipick/task/stow_joint_state` | `JointState` | 收臂模板 |
| `/medipick/task/planned_trajectory` | `JointTrajectory` | 当前阶段规划结果 |
| `/medipick/visualization_trajectory` | `JointTrajectory` | 纯可视化轨迹 |
| `/medipick/task/debug_markers` | `MarkerArray` | 候选点、黄色箭头、阶段文字等 debug marker |

### 4.3 任务管理器订阅的主要 topic

| Topic | 类型 | 作用 |
| --- | --- | --- |
| `/medipick/task/target_pose` | `PoseStamped` | 目标药盒吸取位姿 |
| `/joint_states` | `JointState` | 当前机器人状态 |

### 4.4 任务管理器提供的 service

| Service | 作用 |
| --- | --- |
| `/medipick/task/start` | 启动任务 |
| `/medipick/task/reset` | 重置任务状态机 |
| `/medipick/task/mark_base_arrived` | 人工确认底盘到位 |
| `/medipick/task/mark_lift_arrived` | 人工确认升降到位 |

### 4.5 任务管理器调用的外部接口

| 接口 | 类型 | 作用 |
| --- | --- | --- |
| `/medipick_planning_server/plan_to_pose` | service | 普通 pose-goal 规划 |
| `/compute_fk` | service | FK 校验 |
| `/compute_ik` | service | IK 求解 |
| `/check_state_validity` | service | 碰撞与合法性检查 |
| `/compute_cartesian_path` | service | 短距离 Cartesian 插入/撤出 |
| `/mobile_arm_controller/follow_joint_trajectory` | action | 控制器执行轨迹 |

## 5. 关键运行模式

### 无控制器可视化模式

在 `run_moveit_visual_demo.sh` 里默认：

- `use_ros2_control = false`

这时系统通过：

- `default_joint_state_publisher.py`

来模拟 `/joint_states`，并通过：

- `/medipick/visualization_trajectory`

播放轨迹。

### 控制器执行模式

当：

- `use_ros2_control = true`

时，任务管理器会通过轨迹 action 真实发送控制目标。

### 离散升降模式

当：

- `discrete_lift_mode = true`

时，任务管理器不会直接对升降做普通轨迹执行，而是依赖：

- `lift_target_topic`
- 实时高度回读
- “进入高度带后完成”的判据

## 6. 当前最重要的工程结论

当前系统已经验证出几个关键结论：

1. `pre-insert` 位姿应放在柜口外，而不是柜内。
2. `SELECT_PRE_INSERT` 不需要一开始就做重型搜索，轻量选择 pose 反而更稳。
3. `PLAN_TO_PRE_INSERT` 直接做 pose-goal 容易选到坏 IK 分支。
4. 对 `pre-insert`，优先使用“seeded IK + 短 joint 轨迹”明显更自然。
5. 对 `r1_joint` 的限制应是“相对阶段起点的最大偏移”，不是绝对角度，也不是整条轨迹 `max-min`。
