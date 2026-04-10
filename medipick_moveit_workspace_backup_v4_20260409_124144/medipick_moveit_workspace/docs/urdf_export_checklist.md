# URDF 导出检查表

这份清单用于导出新版 URDF 时，对照当前工程的运动学语义，尽量保证新版模型可以继续复用现有的 `prepare / final_approach` 算法框架。

核心原则不是“外形完全一样”，而是：

- joint 名字一致
- 父子关系一致
- joint type 一致
- joint axis 方向语义一致
- `r6_link -> sucker` 的工具定义一致


## 1. 总体约定

### 1.1 机器人基本字段

| 项目 | 当前值 | 要求 | 是否允许改 |
|---|---|---|---|
| `robot name` | `simple3` | 最好保持一致 | 可改 |
| 主链起点 | `world` | 必须存在 | 不建议改 |
| 末端工具 link | `sucker` | 供任务目标和 FK 校验使用 | 不建议改 |
| IK 参考 link | `r6_link` | MoveIt 规划实际参考 link | 不建议改 |
| 工具组链路 | `r6_link -> sucker` | 必须能单独表达工具偏移 | 不建议改 |

### 1.2 主链拓扑

当前工程依赖的主链如下：

```text
world
└── base_x_link
    └── base_y_link
        └── base_theta_link
            └── base_link
                └── raise_link
                    └── r1_link
                        └── r2_link
                            └── r3_link
                                └── r4_link
                                    └── r5_link
                                        └── r6_link
                                            └── sucker
```

新版导出时，这条主链的名字、顺序、父子关系尽量都保持不变。


## 2. 主链 joint 模板

下表是当前主链 joint 的标准模板。

| joint | type | parent | child | axis | 是否允许改 |
|---|---|---|---|---|---|
| `base_x` | `prismatic` | `world` | `base_x_link` | `1 0 0` | 不建议改 |
| `base_y` | `prismatic` | `base_x_link` | `base_y_link` | `0 1 0` | 不建议改 |
| `base_theta` | `revolute` | `base_y_link` | `base_theta_link` | `0 0 1` | 不建议改 |
| `base_mount` | `fixed` | `base_theta_link` | `base_link` | fixed | 不建议改 |
| `raise_joint` | `prismatic` | `base_link` | `raise_link` | `0 0 1` | 不建议改 |
| `r1_joint` | `revolute` | `raise_link` | `r1_link` | `1 0 0` | 不建议改 |
| `r2_joint` | `revolute` | `r1_link` | `r2_link` | `0 1 0` | 不建议改 |
| `r3_joint` | `revolute` | `r2_link` | `r3_link` | `1 0 0` | 不建议改 |
| `r4_joint` | `revolute` | `r3_link` | `r4_link` | `0 1 0` | 不建议改 |
| `r5_joint` | `revolute` | `r4_link` | `r5_link` | `1 0 0` | 不建议改 |
| `r6_joint` | `revolute` | `r5_link` | `r6_link` | `0 -1 0` | 不建议改 |
| `sucker_joint` | `revolute` | `r6_link` | `sucker` | `1 0 0` | 可改，但要同步参数 |

### 2.1 为什么这些不能乱改

- `pick_task_manager.py` 默认把工具前向当成 `sucker` 坐标系的 `+X`
- `planning_server.yaml` 里保存了 `sucker` 相对 `r6_link` 的工具偏移
- MoveIt 组定义默认以 `r6_link` 为 arm 链的 tip link

所以如果 joint 数量虽然没变，但 axis 或父子关系改了，现有算法通常不能只靠调参继续工作。


## 3. 关键 origin 模板

下面几个 joint 的 `origin` 特别重要。

| joint | origin xyz | origin rpy | 备注 |
|---|---|---|---|
| `base_mount` | `0 0 0` | `0 0 -1.5707963267948966` | 底盘坐标语义很关键 |
| `raise_joint` | `0 0 0.791000000000037` | `0 0 0` | 升降零位参考 |
| `sucker_joint` | `0.116348988405816 0.0237500000077588 0` | `0 0 0` | 必须与工具偏移参数一致 |

### 3.1 最关键的一项

当前 `r6_link -> sucker` 的偏移是：

```text
xyz = 0.116348988405816 0.0237500000077588 0
rpy = 0 0 0
```

如果新版模型里这个偏移变了，必须同步修改规划参数，否则 RViz 里看起来会像：

- `prepare` 没有投影对齐
- `final_approach` 命中了 `r6_link`，但吸盘没命中目标


## 4. SRDF 组定义模板

当前工程里的 MoveIt 分组语义如下。

| group | chain |
|---|---|
| `car` | `world -> base_link` |
| `arm` | `base_link -> r6_link` |
| `arm_no_lift` | `raise_link -> r6_link` |
| `mobile_arm` | `world -> r6_link` |
| `tool` | `r6_link -> sucker` |

### 4.1 末端执行器定义

| 字段 | 当前值 |
|---|---|
| `name` | `suction_tool` |
| `parent_link` | `r6_link` |
| `group` | `tool` |
| `parent_group` | `arm` |

如果你换了新版模型，但这几个组定义的 base/tip link 变了，那么：

- MoveIt 配置要一起改
- 现有 planning group 名字也要跟着改
- `prepare / final_approach` 逻辑里的 group 假设会失效


## 5. 附属链命名约定

如果你还希望保留头部、左侧链路、可视化和控制接口，下面这些名字也尽量保留。

### 5.1 头部链

| joint | type | parent | child | axis |
|---|---|---|---|---|
| `h1_joint` | `revolute` | `raise_link` | `h1_link` | `0 0 -1` |
| `h2_joint` | `revolute` | `h1_link` | `h2_link` | `1 0 0` |
| `h2_motor` | `fixed` | `h1_link` | `h2_motor` | fixed |

### 5.2 左侧链

| joint | type | parent | child | axis |
|---|---|---|---|---|
| `l1_joint` | `revolute` | `raise_link` | `l1_link` | `-1 0 0` |
| `l2_joint` | `revolute` | `l1_link` | `l2_link` | `0 1 0` |
| `l3_joint` | `revolute` | `l2_link` | `l3_link` | `-1 0 0` |
| `l4_joint` | `revolute` | `l3_link` | `l4_link` | `0 0 1` |
| `l5_joint` | `revolute` | `l4_link` | `l5_link` | `-1 0 0` |
| `l2_motor` | `fixed` | `l1_link` | `l2_motor` | fixed |
| `l3_motor` | `fixed` | `l2_link` | `l3_motor` | fixed |
| `l4_motor` | `fixed` | `l3_link` | `l4_motor` | fixed |
| `l5_motor` | `fixed` | `l4_link` | `l5_motor` | fixed |
| `l6_motor` | `fixed` | `l5_link` | `l6_motor` | fixed |

### 5.3 右臂电机装饰链

| joint | type | parent | child |
|---|---|---|---|
| `r2_motor` | `fixed` | `r1_link` | `r2_motor` |
| `r3_motor` | `fixed` | `r2_link` | `r3_motor` |
| `r4_motor` | `fixed` | `r3_link` | `r4_motor` |
| `r5_motor` | `fixed` | `r4_link` | `r5_motor` |
| `r6_motor` | `fixed` | `r5_link` | `r6_motor` |


## 6. 导出后必须同步检查的配置

### 6.1 规划服务器参数

文件：

- `/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/config/planning_server.yaml`

关键字段：

```yaml
default_pose_link: sucker
ik_pose_link: r6_link
tool_reference_link: sucker
tool_to_ik_offset_x: 0.116348988405816
tool_to_ik_offset_y: 0.0237500000077588
tool_to_ik_offset_z: 0.0
```

### 6.2 必须一起检查的文件

| 文件 | 要检查什么 |
|---|---|
| `src/medipick_simple3_description/urdf/simple3_moveit.urdf` | joint 名字、父子关系、axis、origin |
| `src/medipick_moveit_config/config/medipick.srdf` | group chain、end effector |
| `src/medipick_planning_server/config/planning_server.yaml` | `default_pose_link`、`ik_pose_link`、`tool_reference_link`、`tool_to_ik_offset_*` |
| `src/medipick_moveit_config/config/joint_limits.yaml` | 新 joint limit |
| 自碰撞矩阵 | 新几何下重新生成 |


## 7. 哪些可以变，哪些不能只靠调参补回来

### 7.1 可以变

- mesh 外形
- 连杆长度
- 惯量参数
- 碰撞体
- 关节限位
- `r6_link -> sucker` 的偏移

这些变化通常可以通过：

- 更新 URDF
- 更新 SRDF
- 更新 tool offset 参数
- 重调 `stow / prepare` 参数

来适配。

### 7.2 不能指望只靠调参补回来的

- joint 名字改掉
- 父子关系改掉
- joint axis 正方向翻转
- `sucker` 前向不再是工具坐标 `+X`
- `arm` 的 tip link 不再是 `r6_link`

这些一旦变化，通常意味着：

- `prepare_pose` 的“沿工具轴回退”定义会错
- 横向投影误差的验收会错
- IK/FK 目标 link 语义会错


## 8. 导出后的 5 步验收

1. `check_urdf` 能通过。
2. RViz 里零位姿看起来和旧模型的运动学语义一致。
3. 单独转每个 joint，确认正方向没反。
4. 重新量 `r6_link -> sucker` 偏移，并同步到 `planning_server.yaml`。
5. MoveIt 中 `arm / arm_no_lift / mobile_arm / tool` 都能正常识别。


## 9. 最后的建议

最稳妥的导出策略是：

- 保留旧的运动学语义
- 只替换新版的几何和必要尺寸

也就是：

- joint 名字不动
- 父子关系不动
- axis 不动
- base/tip link 定义不动
- 只更新 mesh、碰撞体、长度、偏移和限位

这样现有这套 `prepare / final_approach` 框架才最容易迁过去继续工作。
