# Medipick 当前系统说明

## 目的

这个目录描述的是**当前代码已经实现并正在使用的主链路**，重点回答三类问题：

- 项目现在的主流程到底是什么
- 关键技术约束和实现细节是什么
- 代码和文件应该从哪里看起

这里的内容和 `docs/v3.md` 的关系是：

- `docs/v3.md` 更偏设计方案和重构原则
- `docs/current_system/` 更偏**当前落地实现的现状说明**

如果两者有差异，以当前代码和本目录文档为准。

## 建议阅读顺序

1. [flow_and_runtime.md](./flow_and_runtime.md)
2. [technical_details.md](./technical_details.md)
3. [architecture_and_files.md](./architecture_and_files.md)
4. [usage_and_demos.md](./usage_and_demos.md)
5. [ros2_integration_guide.md](./ros2_integration_guide.md)
6. [version_explainer_v3.md](./version_explainer_v3.md)

## 当前主入口

### 主演示

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_moveit_visual_demo.sh
```

用途：

- 启动 MoveIt 可视化
- 启动 mock 目标发布
- 启动当前主任务链
- 用默认参数自动跑一遍 V3 主流程

### 手工 MoveIt 调试

```bash
cd /home/lzoolloozl/projects/medipick_moveit_workspace
bash scripts/run_moveit_plan_execute_debug.sh
```

用途：

- 打开 RViz 的 MotionPlanning 面板
- 手工选择规划组、拖动 goal、点 `Plan` / `Execute`

更完整的使用说明见：

- [usage_and_demos.md](./usage_and_demos.md)

## 当前主链一句话总结

当前系统的主链是：

`ACQUIRE_TARGET -> ARM_STOW_SAFE -> BASE_ENTER_WORKSPACE -> LIFT_TO_BAND -> SELECT_PRE_INSERT -> PLAN_TO_PRE_INSERT -> INSERT_AND_SUCTION -> SAFE_RETREAT -> return_to_stow`

它的核心思想是：

- 底盘负责进入可工作区域
- 升降负责进入目标高度带
- `pre-insert` 位姿放在柜口外
- 进入柜体前先到柜口外预备位姿
- `PLAN_TO_PRE_INSERT` 优先使用“当前状态附近的 IK + 短 joint 轨迹”
- 柜内插入和撤出尽量短、直、受限

## 这套说明覆盖哪些代码

当前主路径主要覆盖下面这些文件：

- `src/medipick_simple3_description/urdf/simple3_moveit.urdf`
- `src/medipick_moveit_config/config/medipick.srdf`
- `src/medipick_moveit_config/config/kinematics.yaml`
- `src/medipick_moveit_config/config/ompl_planning.yaml`
- `src/medipick_planning_server/config/planning_server.yaml`
- `src/medipick_planning_server/scripts/planning_server.py`
- `src/medipick_planning_server/scripts/pick_task_manager.py`
- `src/medipick_planning_server/scripts/pick_task_flow.py`
- `src/medipick_planning_server/scripts/pick_task_services.py`
- `src/medipick_planning_server/scripts/pick_task_runtime.py`
- `src/medipick_planning_server/scripts/pick_task_shared.py`
- `src/medipick_planning_server/scripts/pick_task_utils.py`

## 新增文档说明

### 使用与演示

- [usage_and_demos.md](./usage_and_demos.md)

回答：

- 怎么启动当前主演示
- 怎么看 RViz 里的关键可视化
- 怎么做手工 `Plan / Execute`
- 怎么跑随机实验和看实验记录

### 合并到别的 ROS 2 项目

- [ros2_integration_guide.md](./ros2_integration_guide.md)
- [../ros2_existing_project_integration_v4.md](/home/lzoolloozl/projects/medipick_moveit_workspace/docs/ros2_existing_project_integration_v4.md)

回答：

- 哪些包必须带走
- 最小可运行接入方式是什么
- 如果并到已有 MoveIt/控制器工程，需要保留哪些语义和接口

### v3 版本解释

- [version_explainer_v3.md](./version_explainer_v3.md)

回答：

- 这一版和早期版本相比到底改了什么
- 为什么会从旧 `prepare/final` 逻辑收敛到现在的 V3 主链
- 当前版本的优势、局限和进入 V4 的原因是什么

## 历史文档说明

下面这些文档仍然有参考价值，但它们更多反映讨论过程，不一定与当前实现完全一致：

- `docs/1.md`
- `docs/8.md`
- `docs/v3初始.md`
- `docs/refactor_phase1.md`
- `docs/refactor_phase2.md`
- `docs/refactor_phase3.md`
- `docs/refactor_phase4_cleanup.md`

如果需要快速理解当前系统，请优先看本目录。
