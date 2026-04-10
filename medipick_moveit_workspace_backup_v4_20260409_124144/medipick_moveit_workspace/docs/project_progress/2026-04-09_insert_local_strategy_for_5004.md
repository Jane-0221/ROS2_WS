# 2026-04-09 Insert Local Strategy For 5004

## Problem

`5004` 是一个典型 hard case。现象不是“目标不可达”，而是：

- 机器人已经到达合理的 `pre-insert` / `prepare` 柜外位姿
- 末端与目标之间只差一小段前插
- 但旧的 `INSERT_AND_SUCTION` fallback 会落到过于自由的 `mobile_arm` OMPL 规划
- 这类轨迹虽然能把末端送到终点，却经常伴随不合理的大范围整机重构，尤其是 `r1_joint` 大角度摆动

更直接的失败例子见：

- 历史失败场景记录：
  [20260409_005009_random_cabinet_experiment/case_001/README.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_005009_random_cabinet_experiment/case_001/README.md)
- 场景参数：
  [20260409_005009_random_cabinet_experiment/case_001/params.json](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_005009_random_cabinet_experiment/case_001/params.json)

## Diagnosis

这类失败的本质不是 `prepare` 点选错，而是插入阶段用了不合适的规划策略。

当系统已经处于“末端方向对、距离近、主要只差沿工具轴前插”的局部情形时，继续允许自由 OMPL 在 `mobile_arm` 的高自由度空间里搜索，容易出现：

- 末端到位但路径不合理
- 为了达到终点，底盘、升降和大臂被一起重新构形
- `r1_joint` 等关节运动幅度异常

因此，插入阶段应当优先视为“局部前插问题”，而不是“重新做一次全局姿态搜索问题”。

## Change Strategy

这次改动按三层局部化思路落地：

1. 插入阶段增加“局部窗口”判断

- 用当前关节状态做 FK
- 计算当前末端与目标之间的：
  - 总位置误差
  - 沿目标工具轴的轴向间隙
  - 法平面横向误差
  - 姿态误差
- 当这些量足够小，就认为进入了“局部前插窗口”

2. 在局部窗口内，优先只用局部策略

- `arm` Cartesian 直推
- `arm` 的 seeded IK + 小关节插值
- 受限的 `mobile_arm` seeded IK + 小关节插值

3. 只有不在局部窗口内，才允许再回到自由 OMPL fallback

- 这样可以避免已经接近目标时，系统又重新去做一条“大范围整机规划”

## Code Changes

主要改动文件：

- [pick_task_flow.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_flow.py)
- [pick_task_services.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_services.py)
- [pick_task_manager.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_manager.py)
- [moveit_mock_demo.launch.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/launch/moveit_mock_demo.launch.py)
- [run_moveit_visual_demo.sh](/home/lzoolloozl/projects/medipick_moveit_workspace/scripts/run_moveit_visual_demo.sh)
- [run_scene_5004_rviz.sh](/home/lzoolloozl/projects/medipick_moveit_workspace/scripts/run_scene_5004_rviz.sh)

具体包括：

- 新增插入阶段局部窗口判断
- 新增 `arm` 的 local seeded IK 小轨迹分支
- 新增受限 `mobile_arm` 的 local seeded IK 小轨迹分支
- 给插入阶段增加独立的底盘微动限制
- 当处于局部窗口时，默认拒绝自由 OMPL fallback
- 将这些参数透传到 launch 和演示脚本

## Intermediate Experiments

### Step 1

第一次实现后，`5004` 仍失败，但失败模式已经明显变好：

- `mobile_arm` 的自由 OMPL 不再主导插入阶段
- 失败点变成：
  - `arm` local seeded IK 失败
  - `mobile_arm` local seeded IK 因底盘平移限制过紧被拒绝

对应记录：

- [20260409_100702_random_cabinet_experiment/case_001/run.log](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_100702_random_cabinet_experiment/case_001/run.log)

关键观察：

- 插入阶段确实进入了局部窗口
- `mobile_arm` local seeded IK 想要的底盘平移只有约 `5~6 cm`
- 但当时默认上限只有 `4 cm`

这说明方向是对的，只是局部底盘微调阈值设置过紧。

### Step 2

随后把插入阶段的底盘平移限制从 `0.04 m` 调整为 `0.07 m`，保持“只允许很小微调”的原则，但放宽到足以覆盖 `5004` 这类真实需求。

## Result

`5004` 在当前版本下已经成功完成整条流程：

- 记录目录：
  [20260409_100833_random_cabinet_experiment](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_100833_random_cabinet_experiment)
- 单 case 日志：
  [20260409_100833_random_cabinet_experiment/case_001/run.log](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_100833_random_cabinet_experiment/case_001/run.log)

关键证据：

- 插入阶段进入了 `insert_local_window`
- `arm` local seeded IK 仍然可能失败
- 但受限 `mobile_arm` 局部策略成功补上了这类 tiny reconfiguration
- 最终执行后进入 `SAFE_RETREAT`
- 整体任务到达 `COMPLETED`

## Regression

新的随机回归批次已经完成：

- [20260409_101028_random_cabinet_experiment](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_101028_random_cabinet_experiment)
- 汇总：
  [20260409_101028_random_cabinet_experiment/README.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_101028_random_cabinet_experiment/README.md)
- 失败分析：
  [20260409_101028_random_cabinet_experiment/analysis.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_101028_random_cabinet_experiment/analysis.md)

结果：

- `5/5` 成功
- 成功率 `100%`
- 这说明本次“局部插入优先 + tiny mobile base adjustment”并不是只修活了 `5004` 单点，而是在一小批新随机场景上也保持了稳定

这批的意义是确认：这次改动没有把插入阶段重新放回“大范围自由 OMPL”，而是把插入语义稳定收成了更符合任务本质的局部策略。

## Next

如果后续随机回归里仍暴露问题，下一步优先考虑：

1. 把 `pre_insert_select_arm_insert_ik` 的标签继续结构化，分清“纯 arm 不可达”和“局部 mobile_arm 可微调可达”
2. 给 `insert` 局部窗口引入更细的“是否必须允许 tiny base shift”判断，而不是只靠固定阈值
3. 针对 `base_link` 与 `r2_link` 的起始碰撞频繁问题，继续优化 `BASE_ENTER_WORKSPACE` 和 `LIFT_TO_BAND` 后的起始几何裕量

## Later Stress Testing Notes

### Fair batch: 20260409_103536

继续做了一批新的随机压力测试：

- [20260409_103536_random_cabinet_experiment](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_103536_random_cabinet_experiment)
- 分析：
  [20260409_103536_random_cabinet_experiment/analysis.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_103536_random_cabinet_experiment/analysis.md)

测试口径：

- seed 段：`7401-7405`
- `case_timeout = 60s`

结果：

- `5/5` 成功
- 成功率 `100%`

这批可以认为是目前比较公平的一次普适性回归，因为：

- 启动、服务就绪、场景同步有足够时间完成
- 每个 case 仍然是全新随机柜型
- 没有再出现插入阶段掉回“大范围自由 OMPL”导致的失败

### Fast batch: 20260409_103207

也做过一批更激进的快批：

- [20260409_103207_random_cabinet_experiment](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_103207_random_cabinet_experiment)

测试口径：

- seed 段：`7301-7310`
- `case_timeout = 45s`

这批不适合作为“算法普适性”的主指标。原因是：

- 已经观测到一部分 case 停在 `ACQUIRE_TARGET`
- 这些失败更像启动、服务准备和场景同步开销，而不是几何或规划策略本身失败

所以这批更适合反映系统级启动效率，不适合直接拿来评价抓取策略。

### Aborted long-timeout batch: 20260409_102237

还尝试过一批 `120s` 单 case 超时的长批次：

- [20260409_102237_random_cabinet_experiment](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/experiment_records/20260409_102237_random_cabinet_experiment)

这批后来被中止，原因不是算法崩了，而是实验脚本本身会一直挂到超时回收，导致单 case 即使已经完成任务，也不会立刻退出，批测效率太低。

不过中途观察到一个值得保留的新失败模式：

- `SELECT_PRE_INSERT` 由于所有候选都需要超过 `0.30m` 的底盘平移，被提前整体拒绝

这说明后续仍可以继续改进：

- `BASE_ENTER_WORKSPACE` 对更偏、更远的柜位仍然偏保守
- `pre-insert` 选择阶段对底盘平移阈值比较敏感
