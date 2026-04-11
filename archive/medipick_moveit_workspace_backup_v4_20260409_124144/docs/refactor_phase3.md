# Phase 3 完全重构

这轮不再继续兼容旧主流程，而是把任务链直接收成一条新主路径。

## 新主链

- `OBSERVE`
  - 只刷新目标相关几何
  - 不再做大规模 candidate 搜索
- `STOW_ARM`
  - 只收臂到安全姿态
- `BASE_APPROACH`
  - 只做底盘对位
- `LIFT_ALIGN`
  - 只做高度对位
- `FINAL_APPROACH`
  - 先尝试 direct pick
  - 失败后才进入 `PREPARE_POSE`
- `PREPARE_POSE`
  - 只做柜外粗对齐候选搜索
  - 每个候选必须通过“能到 prepare + 能从 prepare 再到 final”两段验证
- `RETREAT`
  - 抓取后退出

## 主动砍掉的旧复杂性

- `OBSERVE` 里不再预缓存 `final_result`
- 不再保留多套 `prepare` fallback 并存
- 不再保留多套 `final_approach` fallback 并存
- `PREPARE_POSE` 候选搜索收成小规模局部搜索

## 当前代码结构

- `pick_task_manager.py`
  - 薄状态机
- `pick_task_flow.py`
  - 新主链的高层规划与候选逻辑
- `pick_task_services.py`
  - MoveIt/FK/IK/Cartesian 调用
- `pick_task_runtime.py`
  - 执行、事件、调试可视化

## 当前默认策略

- 柜外粗对齐
- 车 + 升降 + 手臂联合进柜
- 保留吸盘碰撞
- 直捡优先，失败后才做柜外 prepare
