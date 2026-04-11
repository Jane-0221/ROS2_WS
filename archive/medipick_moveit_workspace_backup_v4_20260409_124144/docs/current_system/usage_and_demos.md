# 使用方法与可视化演示

## 1. 最常用的两个入口

### 自动主演示

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_moveit_visual_demo.sh
```

这条命令会：

- 启动当前运行时机器人模型
- 启动 MoveIt 与 RViz
- 启动 mock 货架、目标 pose、pick path 发布
- 启动当前 V3 任务管理器
- 自动跑完整主链

适合：

- 看当前算法的整体效果
- 观察每个阶段的时序
- 回归验证“这版还能不能完整跑通”

### 手工 MoveIt 调试

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_moveit_plan_execute_debug.sh
```

这条命令更适合：

- 在 RViz 里手工拖 goal
- 验证单个规划组能不能规划
- 点 `Plan` / `Execute` 看轨迹是否合理

### 按实验记录复现场景并在 RViz 中观察

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_experiment_case_rviz.sh docs/experiment_records/20260409_102237_random_cabinet_experiment/case_003
```

这条命令会：

- 从已有实验记录里的 `params.json` 读取柜体和目标参数
- 打开 RViz
- 加载对应场景
- 启动 task manager，但默认先不自动执行

适合：

- 先静态看某个失败/成功场景到底长什么样
- 在真正开始规划前，先观察柜体、目标和机器人站位
- 对照 `run.log` 分析为什么会失败

## 2. RViz 里怎么看

### 2.1 自动主演示时重点看什么

当前任务管理器会持续发布这些可视化：

- `/medipick/task/pre_insert_pose`
- `/medipick/task/pick_pose`
- `/medipick/task/retreat_pose`
- `/medipick/task/stage_path`
- `/medipick/task/debug_markers`
- `/medipick/visualization_trajectory`

通常你会看到：

- 黄色箭头或 marker：当前 `pre-insert` 位姿
- 路径线：`pre-insert -> pick -> retreat`
- 文字 marker：当前任务阶段
- 轨迹播放：当前阶段规划出来的关节轨迹

### 2.2 这一版主流程在 RViz 里的典型表现

当前 V3 主链是：

`ACQUIRE_TARGET -> ARM_STOW_SAFE -> BASE_ENTER_WORKSPACE -> LIFT_TO_BAND -> SELECT_PRE_INSERT -> PLAN_TO_PRE_INSERT -> INSERT_AND_SUCTION -> SAFE_RETREAT -> return_to_stow`

正常情况下你会看到：

1. 先收臂
2. 底盘靠近货柜
3. 升降进入目标高度带
4. 柜口外出现 `pre-insert` 预备位姿
5. 机器人到达 `pre-insert`
6. 短程进入柜内吸取
7. 退出并回到 stow

### 2.3 怎么看当前阶段

可以直接看 RViz marker，也可以开一个终端：

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
source install/setup.bash
ros2 topic echo /medipick/task/stage
```

如果想看阶段事件：

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
source install/setup.bash
ros2 topic echo /medipick/task/event
```

### 2.4 看完场景后，如何手动开始规划

如果你是用这种“先只打开场景，不自动执行”的方式启动的：

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_experiment_case_rviz.sh docs/experiment_records/20260409_102237_random_cabinet_experiment/case_003
```

看完 RViz 里的场景后，再开一个新终端执行：

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
source install/setup.bash
ros2 service call /medipick/task/start std_srvs/srv/Trigger "{}"
```

这会让 task manager 从当前场景正式开始整条规划流程。

## 3. 手工 `Plan / Execute` 怎么做

运行：

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_moveit_plan_execute_debug.sh
```

然后在 RViz 的 `MotionPlanning` 面板里：

1. 选择 `Planning Group`
2. 在 3D 视图里拖动 goal marker
3. 点 `Plan`
4. 轨迹合理再点 `Execute`

当前常用规划组：

- `arm`
  - 只测手臂插入/退出
- `mobile_arm`
  - 测底盘 + 升降 + 手臂联合规划
- `whole_body`
  - 连吸盘 joint 一起纳入

## 4. 推荐的调试顺序

### 先看整体流程

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_moveit_visual_demo.sh
```

### 看特定实验场景

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_experiment_case_rviz.sh docs/experiment_records/20260409_102237_random_cabinet_experiment/case_003
```

### 再看单阶段规划

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_moveit_plan_execute_debug.sh
```

### 最后跑随机实验

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_random_cabinet_experiment.sh 8 seed_start:=4001 case_timeout:=60 record_video:=false
```

## 5. 随机实验怎么用

### 跑一批随机柜体

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_random_cabinet_experiment.sh 8 \
  seed_start:=4001 \
  case_timeout:=60 \
  record_video:=false \
  base_standoff:=0.58 \
  lift_end_effector_target_offset:=0.02 \
  candidate_retry_planning_time:=4.0 \
  candidate_retry_num_planning_attempts:=3
```

### 看实验记录

实验会生成到：

- `docs/experiment_records/<timestamp>_random_cabinet_experiment/`

重点看：

- `README.md`
- `summary.csv`
- `analysis.md`
- 每个 case 下的 `README.md`
- 每个 case 下的 `run.log`

## 6. 可直接复制的场景命令

### 6.1 历史 hard case `5004`

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_scene_5004_rviz.sh
```

这个场景是历史上“已经到达 `pre-insert`，但插入阶段策略不够局部化”的典型例子。

### 6.2 `7203`：`SELECT_PRE_INSERT` 底盘平移超 `0.30m` 的场景

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_experiment_case_rviz.sh docs/experiment_records/20260409_102237_random_cabinet_experiment/case_003
```

这个场景最适合看“为什么 `pre-insert` 候选在选择阶段就被整体拒掉”。

### 6.3 `7302`：插入阶段局部 `mobile_arm` 微调略超限制的场景

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_experiment_case_rviz.sh docs/experiment_records/20260409_103207_random_cabinet_experiment/case_002
```

这个场景适合看“局部插入时底盘是否需要极小微调”。

### 6.4 `5004` 旧失败版本记录

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_experiment_case_rviz.sh docs/experiment_records/20260409_100702_random_cabinet_experiment/case_001
```

这个场景适合对照看“底盘微调限制从 `0.04m` 放宽到 `0.07m` 前后”的差异。

### 6.5 打开场景后手动开始任务

如果上面的命令是“只打开场景，不自动执行”的场景复现入口，看完后用这条开始任务：

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
source install/setup.bash
ros2 service call /medipick/task/start std_srvs/srv/Trigger "{}"
```

## 7. 常见问题

### 为什么 RViz 里看起来不动

先确认：

- `/medipick/task/stage` 有没有变化
- `run.log` 里有没有卡在 `ACQUIRE_TARGET`
- 是否是 `rviz:=false` 模式在跑实验脚本

### 为什么自动演示能跑，手工 `Plan / Execute` 不一定能跑

因为当前自动主链用了任务状态机里的：

- 柜口外 `pre-insert`
- seeded IK 优先
- `r1` 阶段运动限幅
- 插入/撤退阶段的额外约束

而手工 `Plan / Execute` 更接近“直接让 MoveIt 自己找一条轨迹”。

### 为什么随机实验里的前几个阶段显得慢

当前统计里常见慢阶段是：

- `ARM_STOW_SAFE`
- `BASE_ENTER_WORKSPACE`
- `LIFT_TO_BAND`

这是因为当前系统仍然偏向“先建立安全进柜前置条件”，而不是一开始就让手臂自由贴近目标。
