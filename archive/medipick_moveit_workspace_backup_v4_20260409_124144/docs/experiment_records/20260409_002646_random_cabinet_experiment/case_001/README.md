# case_001

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `INSERT_AND_SUCTION`
- Failure message: `lift_reference_tool_height: FK request timed out.`

## Parameters

- Seed: `5008`
- Shelf levels: `4`
- Target gap index: `0`
- Target level: `2`
- Shelf center: `(0.843, 0.090)`
- Shelf size (depth,width): `(0.282, 0.778)`
- Shelf bottom / level gap: `(0.402, 0.234)`
- Shelf board / side / back thickness: `(0.026, 0.052, 0.036)`
- Target box size: `(0.080, 0.101, 0.054)`
- Target pose: `(0.722, -0.001, 0.519)`

## Stage Durations

- `ACQUIRE_TARGET`: 1.684s
- `ARM_STOW_SAFE`: 2.214s
- `BASE_ENTER_WORKSPACE`: 2.711s
- `LIFT_TO_BAND`: 2.212s
- `SELECT_PRE_INSERT`: 0.390s
- `PLAN_TO_PRE_INSERT`: 2.187s
- `INSERT_AND_SUCTION`: 10.512s
- `SELECT_PRE_INSERT`: 0.398s
- `PLAN_TO_PRE_INSERT`: 1.449s
- `INSERT_AND_SUCTION`: 10.476s
- `SELECT_PRE_INSERT`: 0.380s
- `PLAN_TO_PRE_INSERT`: 1.451s
- `INSERT_AND_SUCTION`: 10.348s
- `SELECT_PRE_INSERT`: 0.382s
- `PLAN_TO_PRE_INSERT`: 1.458s
- `INSERT_AND_SUCTION`: 10.484s
- `SELECT_PRE_INSERT`: 0.390s
- `PLAN_TO_PRE_INSERT`: 1.449s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
