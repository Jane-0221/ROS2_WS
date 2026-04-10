# Phase 4 清理收口

这轮把主链之外的旧入口真正下线了。

## 当前保留的主入口

- `scripts/run_moveit_visual_demo.sh`
  - 当前唯一推荐的可视化任务链演示脚本
- `scripts/run_moveit_plan_execute_debug.sh`
  - 手工 MoveIt `Plan & Execute` 调试入口
- `src/medipick_planning_server/launch/moveit_mock_demo.launch.py`
  - 主 launch
- `src/medipick_planning_server/scripts/planning_server.py`
  - 主规划服务
- `src/medipick_planning_server/scripts/pick_task_manager.py`
  - 主任务状态机

## 已下线的 legacy 入口

- `randomized_pick_demo`
- `pose_goal_obstacle_demo`
- `static_shelf_scene`
- `controller_parameter_seed`
- `fake_follow_joint_trajectory_server`

这些入口不再参与安装，也不再作为推荐使用路径。

## 兼容性处理

- `scripts/run_moveit_demo.sh`
- `scripts/run_moveit_mock_demo.sh`
- `scripts/run_task_chain_demo.sh`
- `scripts/run_pick_task_manager.sh`

这些旧脚本现在全部转发到 `scripts/run_moveit_visual_demo.sh`，避免再记多套入口。
