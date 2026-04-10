# Failure Analysis: 20260408_210657_random_cabinet_experiment

## Summary

- Total cases: `8`
- Successful cases: `4`
- Failed cases: `4`
- Success ratio: `50.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `1.239s`
- `ARM_STOW_SAFE`: `1.958s`
- `BASE_ENTER_WORKSPACE`: `2.712s`
- `INSERT_AND_SUCTION`: `0.645s`
- `LIFT_TO_BAND`: `1.585s`
- `PLAN_TO_PRE_INSERT`: `1.990s`
- `SAFE_RETREAT`: `2.745s`
- `SELECT_PRE_INSERT`: `0.005s`

## Slow Stage Frequency

- `BASE_ENTER_WORKSPACE`: `6` cases exceeded `2.0s`
- `ARM_STOW_SAFE`: `5` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `4` cases exceeded `2.0s`
- `SAFE_RETREAT`: `3` cases exceeded `2.0s`
- `ACQUIRE_TARGET`: `2` cases exceeded `2.0s`
- `PLAN_TO_PRE_INSERT`: `2` cases exceeded `2.0s`

## Failure Types

- `startup_fk_or_scene_sync`: `2` cases
- `pre_insert_r1_motion_limit`: `1` cases
- `pre_insert_invalid_motion`: `1` cases

## Failure Details

### case_001

- Seed: `4001`
- Final stage: `ACQUIRE_TARGET`
- Failure type: `startup_fk_or_scene_sync`
- Failure message: `N/A`
- Shelf size (depth,width): `(0.240, 0.806)`
- Shelf center: `(0.888, -0.000)`
- Shelf bottom / level gap: `(0.555, 0.279)`
- Total parsed stage time: `0.000s`
- Detailed case: [README.md](./case_001/README.md)
- Suggested improvements:
- 在 `ACQUIRE_TARGET` 前增加对 `/compute_fk`、`/check_state_validity` 等 MoveIt service 的 readiness 检查，不要只靠节点启动完成就立即进主流程。
- 把 mock 货架/点云发布后的短暂稳定窗口显式化，例如在首帧目标 pose 和点云都到齐后再允许 `ACQUIRE_TARGET` 调 FK。
- 优先处理反复出现的 `shape_mask: Missing transform for shape mesh`，这更像是场景物体 frame 或发布时间早于 TF 稳定的同步问题。

### case_003

- Seed: `4003`
- Final stage: `ACQUIRE_TARGET`
- Failure type: `startup_fk_or_scene_sync`
- Failure message: `N/A`
- Shelf size (depth,width): `(0.199, 0.857)`
- Shelf center: `(0.952, 0.086)`
- Shelf bottom / level gap: `(0.539, 0.246)`
- Total parsed stage time: `0.000s`
- Detailed case: [README.md](./case_003/README.md)
- Suggested improvements:
- 在 `ACQUIRE_TARGET` 前增加对 `/compute_fk`、`/check_state_validity` 等 MoveIt service 的 readiness 检查，不要只靠节点启动完成就立即进主流程。
- 把 mock 货架/点云发布后的短暂稳定窗口显式化，例如在首帧目标 pose 和点云都到齐后再允许 `ACQUIRE_TARGET` 调 FK。
- 优先处理反复出现的 `shape_mask: Missing transform for shape mesh`，这更像是场景物体 frame 或发布时间早于 TF 稳定的同步问题。

### case_007

- Seed: `4007`
- Final stage: `FAILED`
- Failure type: `pre_insert_r1_motion_limit`
- Failure message: `Pre-insert trajectory violates the R1 stage-motion limit.`
- Shelf size (depth,width): `(0.202, 0.675)`
- Shelf center: `(0.968, 0.070)`
- Shelf bottom / level gap: `(0.475, 0.299)`
- Total parsed stage time: `9.847s`
- Detailed case: [README.md](./case_007/README.md)
- Suggested improvements:
- 把 `PLAN_TO_PRE_INSERT` 的 seeded IK 终点筛选也接入 `r1` 阶段位移代价，提前避开接近 100 度阈值的解，而不是规划完再拒绝。
- 对这类柜型给 `pre-insert` 加一个更偏向当前 `r1` 半空间的小候选偏置，减少 `mobile_arm` 为了够到柜口而反向大摆。
- 必要时把 `r1` 限制从硬阈值拆成两级：90 度以上重罚，100 度以上拒绝。

### case_008

- Seed: `4008`
- Final stage: `FAILED`
- Failure type: `pre_insert_invalid_motion`
- Failure message: `Pre-insert planning failed: MoveIt failed to produce a valid trajectory (INVALID_MOTION_PLAN, code=-2).; retry failed: MoveIt failed to produce a valid trajectory (FAILURE, code=99999).`
- Shelf size (depth,width): `(0.227, 0.822)`
- Shelf center: `(0.807, -0.010)`
- Shelf bottom / level gap: `(0.516, 0.232)`
- Total parsed stage time: `11.515s`
- Detailed case: [README.md](./case_008/README.md)
- Suggested improvements:
- 给 `PLAN_TO_PRE_INSERT` 增加按目标高度和柜口几何自适应的站位/升降预调，而不是固定常数。
- 把 `pre-insert` 从单 pose 再扩成少量 2-3 个候选，但仍保持 seeded IK 优先，兼顾速度和兜底能力。

## Suggested Next Actions

- 优先做柜深/层高驱动的 `base_standoff` 与 `pre_insert_offset` 自适应，而不是继续用全局常数。
- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。