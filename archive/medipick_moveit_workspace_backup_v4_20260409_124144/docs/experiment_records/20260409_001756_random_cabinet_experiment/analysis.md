# Failure Analysis: 20260409_001756_random_cabinet_experiment

## Summary

- Total cases: `1`
- Successful cases: `0`
- Failed cases: `1`
- Success ratio: `0.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `0.659s`
- `ARM_STOW_SAFE`: `2.307s`
- `BASE_ENTER_WORKSPACE`: `2.715s`
- `INSERT_AND_SUCTION`: `10.525s`
- `LIFT_TO_BAND`: `2.219s`
- `PLAN_TO_PRE_INSERT`: `1.720s`
- `SELECT_PRE_INSERT`: `0.052s`

## Slow Stage Frequency

- `INSERT_AND_SUCTION`: `2` cases exceeded `2.0s`
- `ARM_STOW_SAFE`: `1` cases exceeded `2.0s`
- `BASE_ENTER_WORKSPACE`: `1` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `1` cases exceeded `2.0s`
- `PLAN_TO_PRE_INSERT`: `1` cases exceeded `2.0s`

## Failure Types

- `pre_insert_r1_motion_limit`: `1` cases

## Failure Details

### case_001

- Seed: `5003`
- Final stage: `INSERT_AND_SUCTION`
- Failure type: `pre_insert_r1_motion_limit`
- Failure message: `N/A`
- Shelf size (depth,width): `(0.237, 0.841)`
- Shelf center: `(0.896, 0.018)`
- Shelf bottom / level gap: `(0.507, 0.238)`
- Total parsed stage time: `34.263s`
- Detailed case: [README.md](./case_001/README.md)
- Suggested improvements:
- 把 `PLAN_TO_PRE_INSERT` 的 seeded IK 终点筛选也接入 `r1` 阶段位移代价，提前避开接近 100 度阈值的解，而不是规划完再拒绝。
- 对这类柜型给 `pre-insert` 加一个更偏向当前 `r1` 半空间的小候选偏置，减少 `mobile_arm` 为了够到柜口而反向大摆。
- 必要时把 `r1` 限制从硬阈值拆成两级：90 度以上重罚，100 度以上拒绝。

## Suggested Next Actions

- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。