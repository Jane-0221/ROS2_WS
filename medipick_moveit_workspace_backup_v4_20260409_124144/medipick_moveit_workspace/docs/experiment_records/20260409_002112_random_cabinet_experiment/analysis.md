# Failure Analysis: 20260409_002112_random_cabinet_experiment

## Summary

- Total cases: `1`
- Successful cases: `0`
- Failed cases: `1`
- Success ratio: `0.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `0.671s`
- `ARM_STOW_SAFE`: `2.303s`
- `BASE_ENTER_WORKSPACE`: `2.712s`
- `LIFT_TO_BAND`: `2.219s`
- `PLAN_TO_PRE_INSERT`: `20.509s`
- `SELECT_PRE_INSERT`: `0.388s`

## Slow Stage Frequency

- `ARM_STOW_SAFE`: `1` cases exceeded `2.0s`
- `BASE_ENTER_WORKSPACE`: `1` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `1` cases exceeded `2.0s`
- `PLAN_TO_PRE_INSERT`: `1` cases exceeded `2.0s`

## Failure Types

- `pre_insert_timeout`: `1` cases

## Failure Details

### case_001

- Seed: `5004`
- Final stage: `FAILED`
- Failure type: `pre_insert_timeout`
- Failure message: `Pre-insert planning failed: arm: MoveIt failed to produce a valid trajectory (TIMED_OUT, code=-6).; retry failed: MoveIt failed to produce a valid trajectory (INVALID_MOTION_PLAN, code=-2).; mobile_arm: MoveIt failed to produce a valid trajectory (TIMED_OUT, code=-6).; retry failed: MoveIt failed to produce a valid trajectory (INVALID_MOTION_PLAN, code=-2).`
- Shelf size (depth,width): `(0.227, 0.885)`
- Shelf center: `(0.905, -0.003)`
- Shelf bottom / level gap: `(0.411, 0.262)`
- Total parsed stage time: `28.802s`
- Detailed case: [README.md](./case_001/README.md)
- Suggested improvements:
- 让 `PLAN_TO_PRE_INSERT` 对深柜或窄柜自适应增大 `candidate_retry_planning_time` 和 `candidate_retry_num_planning_attempts`，而不是固定 1 次快速尝试。
- 按柜深动态增大 `pre_insert_offset` 和 `base_standoff`，减少手臂在柜口外就需要大折叠的情况。

## Suggested Next Actions

- 优先做柜深/层高驱动的 `base_standoff` 与 `pre_insert_offset` 自适应，而不是继续用全局常数。
- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。