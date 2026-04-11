# 当前场景与 RL/仿真接入说明

这份文档是给“另一个开发者或另一个 AI”直接接手用的。

目标不是解释所有历史细节，而是把下面几件事说清楚：

- 当前系统里正在验证的是一个什么场景
- 强化学习如果要接进来，训练的到底是哪一段动作
- 初始化环境时，机器人、货架、目标点应该怎么设置
- 哪些部分已经有现成代码，哪些部分应该继续替换或扩展


## 1. 当前正在验证的任务是什么

当前版本验证的是一个简化但贴近真实项目的任务：

- 机器人正面对着一个多层货架
- 目标不再依赖真实目标检测，而是直接给一个 `target_pose`
- `target_pose` 位于货架两个隔板之间
- 吸盘姿态要求水平，且吸取方向从货架开口指向背板
- 升降杆是离散执行机构，不参与连续联合轨迹
- 右臂负责最终的预备、前伸吸取、回撤

当前任务不是导航问题，也不是完整 SLAM 问题。

当前主链只验证：

1. 根据目标位姿，求一个合适的底盘站位和升降杆高度
2. 计算一个动态 `prepare_pose`
3. 再完成最终只用右臂的 `FINAL_APPROACH`
4. 吸取后 `RETREAT`


## 2. 当前状态机的真实含义

目前新版逻辑按下面这些阶段理解最准确：

1. `OBSERVE`
   输入目标位姿、环境点云、当前机器人状态，计算候选解

2. `STOW_ARM`
   先让升降杆到安全高度，再让右臂收拢

3. `BASE_APPROACH`
   底盘去到候选站位

4. `PREPARE_POSE`
   先让升降杆到目标附近高度，再让右臂到预备姿态

5. `FINAL_APPROACH`
   从预备姿态出发，尽量只用右臂完成最后吸取动作

6. `RETREAT`
   退出货架

其中要特别注意：

- `raise_joint` 不能再被当成连续联合规划轴
- 现在真正值得训练或优化的核心，是 `PREPARE_POSE -> FINAL_APPROACH`


## 3. 当前仿真/测试场景是什么

当前 mock 场景由 [mock_vision_publisher.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/mock_vision_publisher.py) 生成。

场景组成是：

- 一个多层货架
- 若干层板
- 左右侧板
- 背板
- 可选杂物点云
- 一个直接生成的 `target_pose`

当前默认理解下的货架坐标关系是：

- `x` 方向：从机器人朝向货架深处
- `y` 方向：货架横向
- `z` 方向：竖直向上

当前目标位姿的定义是：

- 位置在两个隔板之间的中间高度
- `x` 位于货架开口往里一点的位置
- `y` 在货格横向允许范围内随机
- 姿态是单位四元数

也就是说当前的吸取方向默认就是局部 `+X`。

这意味着：

- 红色目标箭头应该从货架开口指向背板
- `prepare_pose` 应该与这个姿态一致
- `final_approach` 的本质应该是沿局部 `+X` 再前进一小段


## 4. 强化学习应该训练什么动作

不建议直接训练整条任务，也不建议一开始训练：

- 底盘
- 升降杆
- 手臂

全部一起输出。

更合理的训练对象是：

- **只训练最终靠近吸取的局部动作**
- 或者训练“动态预备姿态修正 + 最后前伸”这一小段

也就是训练下面这类能力：

1. 在底盘已经大致到位、升降杆已经大致对齐高度时
2. 根据当前末端和目标位姿的相对关系
3. 修正右臂姿态，使：
   - 吸盘与目标同高度
   - 吸盘姿态与目标姿态一致
   - 吸盘在垂直于吸取方向的平面上投影与目标重合
4. 然后沿目标法向前伸吸取

一句话说：

- RL 更适合训练 `PREPARE_ARM residual + FINAL_APPROACH`
- 不适合一上来训练 `base + lift + arm` 全链


## 5. 如果做 RL，建议把动作定义成什么

优先级从高到低：

### 方案 A：末端残差动作

动作输出：

- 末端局部 `x/y/z` 小位移残差
- 末端姿态小角度残差

优点：

- 更接近“让吸盘对齐并前伸”的任务本质
- 训练目标更稳定

缺点：

- 需要底层 IK 或控制映射


### 方案 B：右臂关节小残差

动作输出：

- `r1..r6` 的小增量

但范围必须很小，例如：

- `r1`: `±5~8 deg`
- `r2`: `±8~12 deg`
- `r3`: `±8~10 deg`
- `r4`: `±8~12 deg`
- `r5`: `±5~8 deg`
- `r6`: `±5~8 deg`

并且需要加入强约束：

- 惩罚 `r2/r3/r6` 大翻转
- 惩罚自碰撞
- 惩罚多余路径长度


### 方案 C：只训练前伸距离

如果前面的 `prepare_pose` 已经足够稳定，最简单的是：

- RL 只学最后前伸距离或前伸速度

这适合最保守的第一版。


## 6. 当前最推荐的训练目标

如果现在就要交给别人开始做 RL，我建议先这样定义：

### 训练段

- 输入：`prepare_pose` 附近状态
- 输出：右臂残差或末端残差
- 目标：完成 `FINAL_APPROACH`

### 训练成功标准

- 末端吸盘与目标同高度
- 吸盘姿态与目标姿态误差小
- 沿目标法向向前推进
- 不自碰撞
- 不碰货架
- 不出现 `r2/r3/r6` 无意义翻转

### 不建议现在训练的部分

- 底盘导航
- 升降杆整段控制
- 完整六阶段一体化策略


## 7. 初始化环境时，货架该怎么设置

如果要复现当前项目逻辑，初始化环境时至少要给出下面这些参数。

### 货架几何参数

这些参数在 [mock_vision_publisher.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/mock_vision_publisher.py) 里已经有现成实现：

- `shelf_levels`
- `shelf_center_x`
- `shelf_center_y`
- `shelf_depth`
- `shelf_width`
- `shelf_bottom_z`
- `shelf_level_gap`
- `shelf_board_thickness`
- `shelf_side_thickness`
- `shelf_back_thickness`

### 当前默认场景理解

一个典型有效场景可以理解为：

- 货架中心在机器人前方
- 货架有 5 层左右
- 隔层间距约 `0.24 m`
- 背板在货架最深处
- 目标位姿在某两个隔层之间

### 点云环境设置

如果不是用真实视觉，而是用整体点云直接喂给 MoveIt，则点云至少要表达：

- 货架左右侧板
- 背板
- 层板
- 可选杂物

如果 RL 只训练最终接近段，环境里最少需要保留：

- 层板
- 背板
- 两侧边界

因为这些才真正决定最后一段的可行性和碰撞风险。


## 8. 目标点和目标位姿如何设置

这是最关键的部分。

### 目标位置

目标点不是药盒中心，而更像“吸盘最终应该到达的吸取点”。

当前规则是：

- `z` 在两个隔板之间的中间
- `y` 在货格横向允许范围内
- `x` 在货架开口向里一定深度的位置

也就是：

- 不是贴在背板上
- 也不是在货架外面
- 是“已经在货格内、但仍可前伸到达”的位置

### 目标姿态

当前默认目标姿态应满足：

- 吸盘水平
- 吸取方向垂直于货架背板
- 朝向从货架开口指向背板

换句话说：

- 目标局部 `+X` 是前伸吸取方向

### 如果交给别人实现

请直接告诉对方：

- `target_pose` 不是“药盒几何中心”
- `target_pose` 是“末端最终吸取姿态”

这点非常重要，否则整个后续 `prepare_pose` 和 `final_approach` 都会偏掉。


## 9. 初始化机器人姿态时应该怎么做

这部分可以参考 [mock_initial_pose_manager.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/mock_initial_pose_manager.py)。

### 当前随机初始化思路

当前已经有一个用于随机初始化的 manager，会随机采样：

- `base_x`
- `base_y`
- `base_theta`
- `raise_joint`
- `r1..r6`

### 当前默认范围

当前代码里的典型范围是：

- `base_x`: `[-0.20, 0.20]`
- `base_y`: `[-0.30, 0.30]`
- `base_theta`: `[-pi, pi]`
- `raise_joint`: `[0.10, 0.42]`
- `r1_joint`: `[-0.35, 0.30]`
- `r2_joint`: `[0.95, 1.80]`
- `r3_joint`: `[-2.60, -1.30]`
- `r4_joint`: `[0.80, 1.90]`
- `r5_joint`: `[0.20, 1.20]`
- `r6_joint`: `[-0.40, 0.60]`

### 但如果是给 RL 初始化，不建议完全照抄

更合理的做法是分两类初始化：

#### 类型 1：任务级初始化

适合完整六阶段验证：

- 机器人大致面向货架
- 底盘已经在货架前方合理范围
- 升降杆在可工作区
- 手臂在收拢或半收拢状态

#### 类型 2：局部技能初始化

适合训练 `PREPARE -> FINAL_APPROACH`：

- 底盘已经基本对齐
- 升降杆已经基本对齐高度
- 右臂在一组 prepare 模板附近

这类初始化更适合强化学习训练最终局部动作。


## 10. 如何根据目标位姿初始化机器人

如果不是随机乱采样，而是根据目标生成初始状态，建议这样做。

### 步骤 1：根据目标位姿确定底盘目标站位

使用目标位姿的吸取方向：

- 从 `target_pose.orientation` 提取局部 `+X`
- 沿反方向退一个 `standoff`
- 可加少量横向偏移
- 得到底盘站位

### 步骤 2：根据目标高度估计升降杆高度

规则可以简单写成：

- `raise_joint = f(target_z)`

并限制在：

- `prepare_raise_min <= raise_joint <= prepare_raise_max`

### 步骤 3：根据当前任务选 arm 初始化模板

从一组 prepare arm 模板中选一个最合适的：

- `r2/r4` 保持弯曲
- `r3` 用于调前臂 roll 朝向
- `r6` 用于末端细调

### 步骤 4：如果是 RL 训练

在这个模板附近再加小扰动，而不是全空间随机。


## 11. 建议的 RL 观测量

交给别人做时，可以直接提供下面这份观测定义。

### 机器人状态

- `base_x, base_y, base_theta`
- `raise_joint`
- `r1..r6`
- 当前末端 pose

### 目标相关

- `target_pose`
- 末端到目标的相对位姿
- 目标法向

### 环境相关

- 货架层板高度
- 背板位置
- 左右边界位置
- 或局部点云 / 局部占据栅格

### 派生量

- 末端与目标的高度差
- 末端与目标在法向垂直平面上的投影误差
- 末端与目标沿法向的距离
- 当前弯曲度指标
- 自碰撞距离 / 最近障碍距离


## 12. 建议的 reward

如果要训练最终吸取段，reward 至少包括：

- 末端位置误差惩罚
- 末端姿态误差惩罚
- 高度差惩罚
- 投影误差惩罚
- 自碰撞惩罚
- 与货架碰撞惩罚
- `r2/r3/r6` 大幅翻转惩罚
- 动作总量惩罚
- 成功到达目标给予终奖励

建议特别强调：

- 不要让“到达目标但姿态乱翻”也拿高分
- 不要让“绕很大一圈才到达目标”也拿高分


## 13. 当前代码里已经有哪些现成入口

### 任务状态机

- [pick_task_manager.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_manager.py)

### 点云和目标 pose mock

- [mock_vision_publisher.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/mock_vision_publisher.py)

### 随机初始化

- [mock_initial_pose_manager.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/mock_initial_pose_manager.py)

### 进展可视化

- [run_moveit_mock_progress.sh](/home/lzoolloozl/projects/medipick_moveit_workspace/scripts/run_moveit_mock_progress.sh)


## 14. 最重要的交接说明

如果你把这份文档交给别人，请务必强调下面这几点：

1. 当前项目不是在训练完整导航系统  
2. 当前最重要的是 `PREPARE_POSE -> FINAL_APPROACH`  
3. 升降杆是离散执行机构，不应作为连续联合规划轴  
4. `target_pose` 表示最终吸取位姿，不是药盒几何中心  
5. RL 更适合训练局部技能，而不是一次性输出全部自由度  
6. 初始化不要全空间乱采样，应围绕可行的 prepare 模板分布采样  
7. reward 必须强惩罚自碰撞、关节翻转和冗余动作


## 15. 当前已知问题

这部分也要告诉接手的人，避免误判。

- `FINAL_APPROACH` 目前仍然可能出现多余动作或规划失败
- 用不同 planner 时，行为差异很大
- `RRT*` 不一定更好，有时会直接失败
- `prepare_pose` 的动态候选虽然已经能选出来，但还在继续收敛
- 仍有 `shape_mask` 相关 warning，没有完全清干净

所以当前版本的正确定位是：

- 已经具备“可继续做 RL / 几何候选优化 / 局部技能训练”的基础
- 但还不是最终稳定可部署版本


## 16. 给接手者的直接任务建议

如果是另一个开发者或另一个 AI，建议直接按这个顺序继续：

1. 固化当前场景和观测定义
2. 先训练或优化 `PREPARE -> FINAL_APPROACH`
3. 强化惩罚自碰撞和多余关节翻转
4. 不要先碰导航和完整 SLAM
5. 等局部吸取技能稳定后，再往完整任务拼接
