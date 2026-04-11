# 技术细节与关键约束

## 1. 机器人模型与坐标约定

### 1.1 运行时实际使用的模型

项目运行时实际使用的 URDF 是：

- `src/medipick_simple3_description/urdf/simple3_moveit.urdf`

虽然包名还叫 `medipick_simple3_description`，但当前内容已经切换为：

- `simple4_urdf` 结构与几何

对应的原始导出资产在：

- `assets/simple4_urdf/urdf/simple4_urdf.urdf`

### 1.2 坐标修正

新版 simple4 导出模型原始语义曾经存在：

- 机器人朝 `-Y`
- 头朝 `-Z`

当前项目在运行时使用的模型里已经通过固定关节修正为项目约定：

- `+X` 为前向
- `+Z` 为上向

关键固定关节是：

- `base_roll_mount`
- `base_heading_mount`

### 1.3 `raise_joint` 语义

当前约定：

- `raise_joint = 0` 表示升降最低位置
- 当前项目中的升降值本质上是相对高度偏移
- 在模型中，`raise_joint` 是 prismatic joint

当前运行时模型里它的定义已经对齐为：

- 原点：`xyz = 0 0 -0.939`
- 轴向：`axis = 0 0 -1`

这套定义是围绕当前 simple4 模型修正过的，不应再按旧 simple3 语义理解。

### 1.4 末端与 IK link

当前主要 link 约定：

| 角色 | 当前值 |
| --- | --- |
| pose link | `sucker_link` |
| tool reference link | `sucker_link` |
| IK pose link | `r6_link` |

也就是说：

- 外部任务目标、FK 校验、末端可视化主要面向 `sucker_link`
- 真正给 IK 和 MoveIt 规划时，内部会把目标转换成 `r6_link`

### 1.5 工具偏移

当前工具偏移来自：

- `tool_to_ik_offset_x = 0.1112`
- `tool_to_ik_offset_y = -0.0235`
- `tool_to_ik_offset_z = 0.0`

这组值定义的是：

- `sucker_link -> r6_link`

的工具偏移在世界坐标中的展开方式。

## 2. MoveIt 规划组

当前 SRDF 主要规划组如下。

| 规划组 | Base | Tip | 用途 |
| --- | --- | --- | --- |
| `car` | `world` | `base_link` | 底盘 |
| `arm` | `base_link` | `r6_link` | 右臂 + 升降基于基体 |
| `arm_no_lift` | `raise_link` | `r6_link` | 不含升降的手臂链 |
| `mobile_arm` | `world` | `r6_link` | 车 + 升降 + 右臂 |
| `whole_body` | `world` | `sucker_link` | 车 + 升降 + 右臂 + 吸盘 joint |
| `tool` | `r6_link` | `sucker_link` | 工具链 |

当前主任务里常用的是：

- `pre_insert_group_name = mobile_arm`
- `insert_group_name = arm`
- `insert_fallback_group_name = mobile_arm`
- `safe_retreat_group_name = arm`

也就是：

- 到柜口外预备位姿时允许车臂联合
- 真正插入和撤出时默认以手臂为主

## 3. 规划后端与接口

### 3.1 `planning_server.py` 的作用

`src/medipick_planning_server/scripts/planning_server.py` 不是另一个独立规划器，它更像是：

- 对 `move_group` 的二次封装层

它负责：

- 接收 `PlanToPose` 请求
- 选择后端
- 规范化 pose
- 根据 `pose_link` 与工具偏移把目标转换成真正规划 link
- 调用 `/plan_kinematic_path`
- 返回 `JointTrajectory` 与最终 joint state

### 3.2 当前服务接口

自定义接口包：

- `src/medipick_planning_interfaces`

主要服务：

- `PlanToPose.srv`
- `UpsertCollisionBox.srv`
- `RemoveCollisionObject.srv`

其中最关键的是 `PlanToPose`：

- 输入：group、pose_link、target_pose、起始 joint state、规划时限等
- 输出：success、planning_time、joint_trajectory、final_joint_state 等

### 3.3 后端模式

`planning_server.yaml` 中当前默认：

- `backend = auto`

实际会优先使用：

- `move_group` 后端

如果将来需要纯 mock 场景，可切到 mock backend，但当前主链不是靠它工作的。

## 4. OMPL 与 IK 配置

### 4.1 IK 配置

当前 `kinematics.yaml` 中所有主组默认使用：

- `kdl_kinematics_plugin/KDLKinematicsPlugin`

主要差异是：

- 搜索分辨率
- timeout
- attempts

当前较重要的组：

- `arm`
- `mobile_arm`
- `whole_body`

### 4.2 OMPL 规划器

当前主要启用：

- `RRTConnect`
- `RRTstar`
- `PRM`

其中当前主线实际常见的是：

- `RRTConnectkConfigDefault`
- `RRTstarkConfigDefault`

### 4.3 投影配置

为了减少规划过度偏向 `r1_joint`，当前 OMPL projection 做过调整：

| 组 | projection_evaluator |
| --- | --- |
| `arm` | `joints(raise_joint,r2_joint)` |
| `arm_no_lift` | `joints(r2_joint,r4_joint)` |
| `car` | `joints(base_x,base_y)` |
| `mobile_arm` | `joints(base_x,base_y,raise_joint)` |
| `whole_body` | `joints(base_x,base_y,raise_joint,sucker_joint)` |

## 5. 当前 V3 关键参数与行为

### 5.1 `pre-insert` 当前默认行为

当前不是“全量候选搜索版”，而是“轻量小候选队列版”：

| 参数 | 当前默认值 | 含义 |
| --- | --- | --- |
| `pre_insert_first_candidate_only` | `false` | 不再强制只取第一组 |
| `pre_insert_select_pose_only` | `true` | `SELECT_PRE_INSERT` 只选 pose，不做重规划 |
| `pre_insert_candidate_count` | `3` | 默认保留 3 个轻量候选 |
| `candidate_max_evaluations` | `1` | 重型候选评估路径仍被压到 1 |
| `candidate_shortlist_size` | `1` | shortlist 只保留一个 |
| `candidate_planning_time` | `1.0` | pose fallback 的时间预算 |
| `candidate_num_planning_attempts` | `1` | pose fallback 的尝试次数 |

这是一个明显偏“提稳、提速”的工程配置，不是最终搜索器形态。

### 5.2 `PLAN_TO_PRE_INSERT` 当前优先逻辑

当前优先顺序：

1. `seeded IK`
2. 先对 seeded IK 终点做 `r1` 阶段位移前置筛选
3. 构造短 joint 插值轨迹
4. 逐点 state validity 检查
5. `r1` 阶段幅度检查
6. FK 到位检查
7. 不通过时才 fallback 到普通 pose 规划
8. 当前候选失败时，自动切换到同一批次里排在后面的备选 `pre-insert`

这意味着：

- `SELECT_PRE_INSERT` 负责快速筛出一小批柜口外 pose
- `PLAN_TO_PRE_INSERT` 负责真正验证“哪个候选能走通”

### 5.3 `ACQUIRE_TARGET` 当前轻量屏障

当前进入任务几何刷新前，会先等三类条件：

| 条件 | 当前参数 |
| --- | --- |
| 目标 pose 稳定时间 | `acquire_target_stable_age = 0.4` |
| 场景 settle 时间 | `acquire_scene_settle_time = 0.6` |
| service 探测等待 | `acquire_service_probe_timeout = 0.05` |
| `ACQUIRE_TARGET` 内部 FK 超时 | `acquire_fk_timeout = 1.0` |

目的不是把 `ACQUIRE_TARGET` 变慢，而是减少随机实验里：

- `move_group` 和 planning scene 刚起来就立刻调 FK
- `/compute_fk` 卡住
- 阶段在 `ACQUIRE_TARGET` 假死

### 5.4 `r1_joint` 限制的当前语义

这是当前实现里很重要的一点：

- 不是看 `r1_joint` 的绝对角度有没有超过 `90 deg`
- 也不是看整条轨迹的 `max-min`
- 而是看**相对当前阶段起点，`r1_joint` 最多偏离了多少**

当前参数：

| 参数 | 默认值 |
| --- | --- |
| `r1_stage_motion_soft_penalty_start_deg` | `70.0` |
| `r1_stage_motion_limit_deg` | `100.0` |

含义：

- 超过 `70 deg` 开始软惩罚
- 超过 `100 deg` 直接拒绝该阶段轨迹

## 6. 插入阶段的当前 fallback 逻辑

当前 `INSERT_AND_SUCTION` 已经不是单一路径：

1. 先尝试短距离 Cartesian 插入
2. Cartesian 不足时，先尝试 `insert_group_name`，当前默认是 `arm`
3. 如果主插入组失败，再尝试 `insert_fallback_group_name`，当前默认是 `mobile_arm`

这条 fallback 的作用是：

- 柜口附近优先保持“短直推”
- 如果手臂单独插不进去，允许车+升降+臂一起做小范围补偿

### 5.5 收臂模板

当前安全收臂模板为：

| Joint | 值 |
| --- | --- |
| `raise_joint` | `0.302` |
| `r1_joint` | `-0.681` |
| `r2_joint` | `-1.309` |
| `r3_joint` | `-1.309` |
| `r4_joint` | `-1.571` |
| `r5_joint` | `0.017` |
| `r6_joint` | `0.087` |

这个模板在代码中名称是：

- `TRANSIT_STOW_V0_STATE_POSITIONS`

### 5.5 撤退后的行为

当前默认：

- `return_to_stow_after_retreat = true`

也就是：

- 抽出末端后，不再规划回旧的 `complete point`
- 而是直接收回到 stow 姿态

## 6. 当前脚本与 launch 的默认选择

### 主演示脚本

`scripts/run_moveit_visual_demo.sh` 当前默认特点：

- `rviz = true`
- `use_ros2_control = false`
- `run_task_manager = true`
- `task_auto_start_on_target = true`
- `task_auto_accept_base_arrival = true`
- `task_auto_accept_lift_arrival = true`
- `pre_insert_group_name = mobile_arm`
- `insert_group_name = arm`
- `safe_retreat_group_name = arm`
- `pre_insert_offset = 0.06`
- `base_standoff = 0.68`
- `base_lateral_offset = 0.18`

这套参数比较适合当前 mock 演示。

### 手工调试脚本

`scripts/run_moveit_plan_execute_debug.sh` 更适合：

- 手动切规划组
- 在 RViz 里测试 pose 能否规划
- 单独观察模型、碰撞和 IK 情况

## 7. 当前仍然保留但要小心理解的历史命名

虽然主流程已经切到了 V3 语义，但代码里仍有一些旧命名保留，这是为了兼容渐进重构。

比较典型的包括：

- `prepare_*`
- `final_*`
- `_prepare_pose`
- `PrepareCandidate`

这些名字在当前代码里通常可以按下面理解：

| 历史命名 | 当前更贴切的理解 |
| --- | --- |
| `prepare_pose` | `pre_insert_pose` |
| `prepare_group_name` | 历史预备阶段用组，当前更多是兼容入口 |
| `final_pose` | `pick_pose` |
| `final_group_name` | 历史最终到位组，当前更多是兼容入口 |

换句话说，当前语义已经以 `pre_insert / insert / retreat` 为主，但代码命名仍在过渡期。
