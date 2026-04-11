# Failure Analysis: 20260409_100833_random_cabinet_experiment

## Summary

- Total cases: `1`
- Successful cases: `1`
- Failed cases: `0`
- Success ratio: `100.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `0.667s`
- `ARM_STOW_SAFE`: `2.308s`
- `BASE_ENTER_WORKSPACE`: `2.717s`
- `INSERT_AND_SUCTION`: `1.688s`
- `LIFT_TO_BAND`: `2.218s`
- `PLAN_TO_PRE_INSERT`: `6.542s`
- `SAFE_RETREAT`: `3.622s`
- `SELECT_PRE_INSERT`: `0.429s`

## Slow Stage Frequency

- `ARM_STOW_SAFE`: `1` cases exceeded `2.0s`
- `BASE_ENTER_WORKSPACE`: `1` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `1` cases exceeded `2.0s`
- `PLAN_TO_PRE_INSERT`: `1` cases exceeded `2.0s`
- `SAFE_RETREAT`: `1` cases exceeded `2.0s`

## Failure Types

- No failures in this experiment batch.

## Failure Details

- No failures to analyze.
## Suggested Next Actions

- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。
- 当前这批未出现失败，可以继续扩大随机范围：增加 `clutter_count`、扩大 `shelf_depth` 和 `target_lateral_span`。