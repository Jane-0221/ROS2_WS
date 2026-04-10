# Phase 1 重构边界

这份文档只回答一个问题：当前仓库里，哪些入口属于主链，哪些入口先视为 legacy。

## 当前主链

- `scripts/run_moveit_plan_execute_debug.sh`
  - 手工 MoveIt 调试入口
- `scripts/run_moveit_mock_progress.sh`
  - 当前 mock 场景下的主验证入口
- `scripts/run_task_chain_demo.sh`
  - 当前任务链演示入口
- `src/medipick_planning_server/scripts/planning_server.py`
  - 主规划服务
- `src/medipick_planning_server/scripts/pick_task_manager.py`
  - 主任务状态机

## 当前先降级为 legacy 的入口

- `src/medipick_planning_server/scripts/randomized_pick_demo.py`
- `src/medipick_planning_server/scripts/pose_goal_obstacle_demo.py`
- `src/medipick_planning_server/scripts/static_shelf_scene.py`
- `src/medipick_planning_server/launch/randomized_pick_demo.launch.py`
- `src/medipick_planning_server/launch/pose_goal_obstacle_demo.launch.py`
- `src/medipick_planning_server/launch/static_shelf_scene.launch.py`
- `src/medipick_moveit_config/scripts/fake_follow_joint_trajectory_server.py`
- `src/medipick_moveit_config/scripts/controller_parameter_seed.py`

这些文件本轮不删除，但不再作为“主流程设计依据”。

## Phase 1 已做的事

- 把 `pick_task_manager.py` 的默认模板、数据结构和公共工具拆到独立模块
- 让 `pick_task_manager.launch.py` 的默认 group 与当前主链策略一致
- 保留旧文件，但把重构目标聚焦到主链，而不是继续给实验入口叠补丁

## Phase 2 建议

- 继续拆 `pick_task_manager.py`
  - 状态机
  - prepare 规划
  - final/retreat 规划
  - 执行与可视化
- 合并重复 launch 逻辑
- 把 legacy 入口从安装/文档主路径中进一步下线
