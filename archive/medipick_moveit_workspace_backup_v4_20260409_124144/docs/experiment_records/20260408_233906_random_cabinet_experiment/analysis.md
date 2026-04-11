# Failure Analysis: 20260408_233906_random_cabinet_experiment

## Summary

- Total cases: `10`
- Successful cases: `7`
- Failed cases: `3`
- Success ratio: `70.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `0.890s`
- `ARM_STOW_SAFE`: `2.645s`
- `BASE_ENTER_WORKSPACE`: `2.464s`
- `INSERT_AND_SUCTION`: `0.638s`
- `LIFT_TO_BAND`: `1.990s`
- `PLAN_TO_PRE_INSERT`: `5.339s`
- `SAFE_RETREAT`: `2.767s`
- `SELECT_PRE_INSERT`: `0.057s`

## Slow Stage Frequency

- `ARM_STOW_SAFE`: `10` cases exceeded `2.0s`
- `BASE_ENTER_WORKSPACE`: `9` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `9` cases exceeded `2.0s`
- `SAFE_RETREAT`: `7` cases exceeded `2.0s`
- `PLAN_TO_PRE_INSERT`: `4` cases exceeded `2.0s`

## Failure Types

- `pre_insert_timeout`: `2` cases
- `pre_insert_r1_motion_limit`: `1` cases

## Failure Details

### case_003

- Seed: `5003`
- Final stage: `FAILED`
- Failure type: `pre_insert_r1_motion_limit`
- Failure message: `Retreat trajectory violates the R1 reasonable-angle limit.`
- Shelf size (depth,width): `(0.237, 0.841)`
- Shelf center: `(0.896, 0.018)`
- Shelf bottom / level gap: `(0.507, 0.238)`
- Total parsed stage time: `20.049s`
- Detailed case: [README.md](./case_003/README.md)
- Suggested improvements:
- 把 `PLAN_TO_PRE_INSERT` 的 seeded IK 终点筛选也接入 `r1` 阶段位移代价，提前避开接近 100 度阈值的解，而不是规划完再拒绝。
- 对这类柜型给 `pre-insert` 加一个更偏向当前 `r1` 半空间的小候选偏置，减少 `mobile_arm` 为了够到柜口而反向大摆。
- 必要时把 `r1` 限制从硬阈值拆成两级：90 度以上重罚，100 度以上拒绝。

### case_004

- Seed: `5004`
- Final stage: `FAILED`
- Failure type: `pre_insert_timeout`
- Failure message: `Pre-insert planning failed: MoveIt failed to produce a valid trajectory (TIMED_OUT, code=-6).; retry failed: MoveIt failed to produce a valid trajectory (INVALID_MOTION_PLAN, code=-2).`
- Shelf size (depth,width): `(0.227, 0.885)`
- Shelf center: `(0.905, -0.003)`
- Shelf bottom / level gap: `(0.411, 0.262)`
- Total parsed stage time: `26.117s`
- Detailed case: [README.md](./case_004/README.md)
- Suggested improvements:
- 让 `PLAN_TO_PRE_INSERT` 对深柜或窄柜自适应增大 `candidate_retry_planning_time` 和 `candidate_retry_num_planning_attempts`，而不是固定 1 次快速尝试。
- 按柜深动态增大 `pre_insert_offset` 和 `base_standoff`，减少手臂在柜口外就需要大折叠的情况。

### case_008

- Seed: `5008`
- Final stage: `FAILED`
- Failure type: `pre_insert_timeout`
- Failure message: `Pre-insert planning failed: MoveIt failed to produce a valid trajectory (INVALID_MOTION_PLAN, code=-2).; retry failed: MoveIt failed to produce a valid trajectory (FAILURE, code=99999).`
- Shelf size (depth,width): `(0.282, 0.778)`
- Shelf center: `(0.843, 0.090)`
- Shelf bottom / level gap: `(0.402, 0.234)`
- Total parsed stage time: `20.070s`
- Detailed case: [README.md](./case_008/README.md)
- Suggested improvements:
- 让 `PLAN_TO_PRE_INSERT` 对深柜或窄柜自适应增大 `candidate_retry_planning_time` 和 `candidate_retry_num_planning_attempts`，而不是固定 1 次快速尝试。
- 按柜深动态增大 `pre_insert_offset` 和 `base_standoff`，减少手臂在柜口外就需要大折叠的情况。

## Suggested Next Actions

- 优先做柜深/层高驱动的 `base_standoff` 与 `pre_insert_offset` 自适应，而不是继续用全局常数。
- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。