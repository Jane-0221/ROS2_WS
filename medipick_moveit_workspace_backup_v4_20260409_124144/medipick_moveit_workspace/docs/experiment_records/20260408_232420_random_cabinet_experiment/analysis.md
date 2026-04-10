# Failure Analysis: 20260408_232420_random_cabinet_experiment

## Summary

- Total cases: `1`
- Successful cases: `0`
- Failed cases: `1`
- Success ratio: `0.0%`

## Average Stage Duration

- `ACQUIRE_TARGET`: `0.627s`
- `ARM_STOW_SAFE`: `2.305s`
- `BASE_ENTER_WORKSPACE`: `0.000s`
- `LIFT_TO_BAND`: `2.603s`
- `SELECT_PRE_INSERT`: `0.330s`

## Slow Stage Frequency

- `ARM_STOW_SAFE`: `1` cases exceeded `2.0s`
- `LIFT_TO_BAND`: `1` cases exceeded `2.0s`

## Failure Types

- `generic_failed`: `1` cases

## Failure Details

### case_001

- Seed: `4007`
- Final stage: `FAILED`
- Failure type: `generic_failed`
- Failure message: `No feasible outside-cabinet pre-insert candidate found.`
- Shelf size (depth,width): `(0.202, 0.675)`
- Shelf center: `(0.968, 0.070)`
- Shelf bottom / level gap: `(0.475, 0.299)`
- Total parsed stage time: `5.865s`
- Detailed case: [README.md](./case_001/README.md)
- Suggested improvements:
- 为失败 case 自动导出更细的分类字段，例如 `ik_failed`、`collision_blocked`、`stage_timeout`，避免后续只能人工读日志。
- 在 case README 里追加失败时的最后 3 条 manager error，便于快速复盘。

## Suggested Next Actions

- 检查收臂模板和底盘到位判定，必要时把 transit stow 再收紧，缩短前置阶段耗时。