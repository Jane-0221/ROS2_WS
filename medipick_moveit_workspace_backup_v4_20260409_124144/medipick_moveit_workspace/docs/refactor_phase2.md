# Phase 2 重构结果

这轮继续遵循一个原则：先拆结构，不先改任务行为。

## 本轮拆分

- `src/medipick_planning_server/scripts/pick_task_services.py`
  - 承接 `pick_task_manager.py` 中的 MoveIt 规划/IK/FK/状态有效性/Cartesian 调用
  - 也承接和这些调用紧邻的几何辅助方法，例如 base goal、prepare 回退距离、retreat pose
- `src/medipick_planning_server/scripts/pick_task_runtime.py`
  - 承接执行、事件发布、轨迹过滤、调试 marker、阶段切换

## 主文件现在负责什么

`src/medipick_planning_server/scripts/pick_task_manager.py` 现在主要保留：

- ROS 参数和 publisher/client/action 初始化
- 目标回调、状态回调、服务回调
- 主状态机推进
- prepare/final/retreat 的高层决策

## 这样拆的目的

- 把“状态机决策”与“服务调用细节”分开
- 把“运行时执行/可视化”与“规划候选搜索”分开
- 为下一轮继续拆 `prepare` 和 `final/retreat` 铺路

## 下一轮建议

- Phase 3 继续把 `prepare` 候选搜索和 `final/retreat` 规划从主类里拆成独立模块
- 然后再合并重复 launch，并进一步下线 legacy demo 入口
