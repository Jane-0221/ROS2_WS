# Failure Analysis: 20260409_002646_random_cabinet_experiment

## Summary

- Total cases: `1`
- Successful cases: `0`
- Failed cases: `1`
- Success ratio: `0.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `1.684s`
- `ARM_STOW_SAFE`: `2.214s`
- `BASE_ENTER_WORKSPACE`: `2.711s`
- `INSERT_AND_SUCTION`: `10.455s`
- `LIFT_TO_BAND`: `2.212s`
- `PLAN_TO_PRE_INSERT`: `1.599s`
- `SELECT_PRE_INSERT`: `0.388s`

## Slow Stage Frequency

- `INSERT_AND_SUCTION`: `4` cases exceeded `2.0s`
- `ARM_STOW_SAFE`: `1` cases exceeded `2.0s`
- `BASE_ENTER_WORKSPACE`: `1` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `1` cases exceeded `2.0s`
- `PLAN_TO_PRE_INSERT`: `1` cases exceeded `2.0s`

## Failure Types

- `lift_alignment`: `1` cases

## Failure Details

### case_001

- Seed: `5008`
- Final stage: `INSERT_AND_SUCTION`
- Failure type: `lift_alignment`
- Failure message: `lift_reference_tool_height: FK request timed out.`
- Shelf size (depth,width): `(0.282, 0.778)`
- Shelf center: `(0.843, 0.090)`
- Shelf bottom / level gap: `(0.402, 0.234)`
- Total parsed stage time: `60.576s`
- Detailed case: [README.md](./case_001/README.md)
- Suggested improvements:
- 继续强化末端高度对齐逻辑，让 `raise_joint` 目标基于当前收臂 FK 高度和目标位姿动态反推。
- 对不同层高把 `lift_band_half_width` 变成比例或分段参数，避免高层/低层误差感受不一致。

## Suggested Next Actions

- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。