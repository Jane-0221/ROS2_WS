# 2026-04-09 Mock Visualization Playback Sync

## Problem

在 `run_experiment_case_rviz.sh` 的 mock 可视化模式下，任务日志显示流程成功完成，但在 RViz 中：

- `PLAN_TO_PRE_INSERT` / `prepare`
- `INSERT_AND_SUCTION`

这两个阶段看起来不像连续动作，更像是“前一段刚开始播，后一段就接管了”。

用户提供的典型现象：

- 日志已经到 `Stage -> COMPLETED`
- `INSERT_AND_SUCTION` 的 Cartesian path 也显示 `followed 100%`
- 但视觉上 `prepare` 和“向前吸取”的动作不明显或像跳过去

## Diagnosis

问题不在规划成功与否，而在 mock 执行的“等待时长”和 RViz 关节动画的“实际播放时长”不一致。

当前系统里有两条相关链路：

1. 任务管理器 mock 执行

- 文件：
  [pick_task_runtime.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_runtime.py)
- 在 `execute_with_controller = false` 时，任务管理器会：
  - 发布一条 `JointTrajectory`
  - 按 `trajectory_duration(...) + margin` 睡眠
  - 然后认为这条轨迹已经执行完

2. RViz 联动的默认关节回放

- 文件：
  [default_joint_state_publisher.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_moveit_config/scripts/default_joint_state_publisher.py)
- 它在收到 `/medipick/visualization_trajectory` 后，会自己播放轨迹
- 对于 `time_from_start = 0` 或非递增的轨迹点，会自动补成每段 `0.15s`

这意味着：

- 任务管理器可能只等了一个很短的“原始轨迹时长”
- 但 RViz 实际上还在按补时后的节奏播动画
- 下一阶段轨迹又已经发出来了
- 结果就会让阶段切换看起来不连续

这在 Cartesian path 尤其明显，因为很多点的时间戳原本很短，甚至接近零。

## Change Strategy

这次改两件事：

1. 让 mock 执行等待时长和 `default_joint_state_publisher.py` 的回放时长一致

- 新增一个“有效播放时长”估计函数
- 采用和 joint state publisher 一样的补时规则：
  - 若时间戳不递增，则按 `0.15s` 递补

2. mock 执行结束后，主动发布一次可视化 joint state 终点覆盖

- 将估计到达的 joint state 发到 `/medipick/visualization_joint_state`
- 保证下一段动画从正确终点继续，而不是残留在中途

## Code Changes

涉及文件：

- [pick_task_utils.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_utils.py)
- [pick_task_runtime.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_runtime.py)
- [pick_task_manager.py](/home/lzoolloozl/projects/medipick_moveit_workspace/src/medipick_planning_server/scripts/pick_task_manager.py)

具体包括：

- 在 `pick_task_utils.py` 中新增：
  - `trajectory_effective_playback_duration(...)`
- 在 `pick_task_runtime.py` 的 mock 执行路径中：
  - 用 `trajectory_effective_playback_duration(...)` 替代原来的 `trajectory_duration(...)`
  - mock 执行结束后发布 `/medipick/visualization_joint_state`
- 在 `pick_task_manager.py` 中新增：
  - `/medipick/visualization_joint_state` publisher

## Validation

已完成的验证：

- `python3 -m py_compile`
- `colcon build --packages-select medipick_planning_server medipick_moveit_config --symlink-install`

当前还没有单独保存一份“修复后重新回放 case_003”的新实验记录目录；这条修复主要是可视化回放链路修复，不改变规划成功性。

## Expected Result

修复后，在 mock RViz 回放中：

- `PLAN_TO_PRE_INSERT` 应该更容易看到完整的移动过程
- `INSERT_AND_SUCTION` 的前插动作不应再被下一阶段快速覆盖
- `SAFE_RETREAT` 应该在前一段动画播完后再开始

如果后续仍然觉得视觉上过快，下一步可以继续做：

1. 给 mock 可视化单独加一个 `visualization_speed_scale`
2. 对关键阶段（如 `prepare`、`insert`）额外加 stage-end hold
3. 将阶段轨迹保存为逐帧 joint state，而不是靠 `JointTrajectory` 回放
