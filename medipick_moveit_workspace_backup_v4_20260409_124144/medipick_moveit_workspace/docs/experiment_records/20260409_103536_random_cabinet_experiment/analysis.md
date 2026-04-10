# Failure Analysis: 20260409_103536_random_cabinet_experiment

## Summary

- Total cases: `5`
- Successful cases: `5`
- Failed cases: `0`
- Success ratio: `100.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `2.251s`
- `ARM_STOW_SAFE`: `2.303s`
- `BASE_ENTER_WORKSPACE`: `2.453s`
- `INSERT_AND_SUCTION`: `0.662s`
- `LIFT_TO_BAND`: `2.218s`
- `PLAN_TO_PRE_INSERT`: `1.450s`
- `SAFE_RETREAT`: `2.676s`
- `SELECT_PRE_INSERT`: `0.402s`

## Slow Stage Frequency

- `ARM_STOW_SAFE`: `5` cases exceeded `2.0s`
- `BASE_ENTER_WORKSPACE`: `5` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `5` cases exceeded `2.0s`
- `SAFE_RETREAT`: `5` cases exceeded `2.0s`
- `ACQUIRE_TARGET`: `2` cases exceeded `2.0s`

## Failure Types

- No failures in this experiment batch.

## Failure Details

- No failures to analyze.
## Suggested Next Actions

- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。
- 当前这批未出现失败，可以继续扩大随机范围：增加 `clutter_count`、扩大 `shelf_depth` 和 `target_lateral_span`。