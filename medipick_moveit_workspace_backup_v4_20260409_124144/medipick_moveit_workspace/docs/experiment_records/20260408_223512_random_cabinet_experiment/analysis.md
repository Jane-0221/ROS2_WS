# Failure Analysis: 20260408_223512_random_cabinet_experiment

## Summary

- Total cases: `2`
- Successful cases: `0`
- Failed cases: `2`
- Success ratio: `0.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `0.651s`
- `ARM_STOW_SAFE`: `2.301s`
- `BASE_ENTER_WORKSPACE`: `2.712s`
- `INSERT_AND_SUCTION`: `5.401s`
- `LIFT_TO_BAND`: `2.210s`
- `PLAN_TO_PRE_INSERT`: `4.184s`
- `SELECT_PRE_INSERT`: `0.023s`

## Slow Stage Frequency

- `INSERT_AND_SUCTION`: `4` cases exceeded `2.0s`
- `ARM_STOW_SAFE`: `2` cases exceeded `2.0s`
- `BASE_ENTER_WORKSPACE`: `2` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `2` cases exceeded `2.0s`
- `PLAN_TO_PRE_INSERT`: `2` cases exceeded `2.0s`

## Failure Types

- `pre_insert_r1_motion_limit`: `1` cases
- `unclassified`: `1` cases

## Failure Details

### case_001

- Seed: `4007`
- Final stage: `FAILED`
- Failure type: `pre_insert_r1_motion_limit`
- Failure message: `Pre-insert trajectory violates the R1 stage-motion limit.`
- Shelf size (depth,width): `(0.202, 0.675)`
- Shelf center: `(0.968, 0.070)`
- Shelf bottom / level gap: `(0.475, 0.299)`
- Total parsed stage time: `11.943s`
- Detailed case: [README.md](./case_001/README.md)
- Suggested improvements:
- 把 `PLAN_TO_PRE_INSERT` 的 seeded IK 终点筛选也接入 `r1` 阶段位移代价，提前避开接近 100 度阈值的解，而不是规划完再拒绝。
- 对这类柜型给 `pre-insert` 加一个更偏向当前 `r1` 半空间的小候选偏置，减少 `mobile_arm` 为了够到柜口而反向大摆。
- 必要时把 `r1` 限制从硬阈值拆成两级：90 度以上重罚，100 度以上拒绝。

### case_002

- Seed: `4008`
- Final stage: `INSERT_AND_SUCTION`
- Failure type: `unclassified`
- Failure message: `N/A`
- Shelf size (depth,width): `(0.227, 0.822)`
- Shelf center: `(0.807, -0.010)`
- Shelf bottom / level gap: `(0.516, 0.232)`
- Total parsed stage time: `50.648s`
- Detailed case: [README.md](./case_002/README.md)
- Suggested improvements:
- 需要补充更细的失败分类规则，当前日志还不足以自动定位根因。

## Suggested Next Actions

- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。