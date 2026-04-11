# case_001

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `FAILED`
- Failure message: `No feasible outside-cabinet pre-insert candidate found.`

## Parameters

- Seed: `5004`
- Shelf levels: `6`
- Target gap index: `0`
- Target level: `3`
- Shelf center: `(0.905, -0.003)`
- Shelf size (depth,width): `(0.227, 0.885)`
- Shelf bottom / level gap: `(0.411, 0.262)`
- Shelf board / side / back thickness: `(0.026, 0.033, 0.024)`
- Target box size: `(0.105, 0.111, 0.094)`
- Target pose: `(0.804, -0.029, 0.542)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.657s
- `ARM_STOW_SAFE`: 2.306s
- `BASE_ENTER_WORKSPACE`: 2.723s
- `LIFT_TO_BAND`: 2.211s
- `SELECT_PRE_INSERT`: 0.507s
- `PLAN_TO_PRE_INSERT`: 7.299s
- `INSERT_AND_SUCTION`: 10.576s
- `PLAN_TO_PRE_INSERT`: 1.451s
- `INSERT_AND_SUCTION`: 10.406s
- `PLAN_TO_PRE_INSERT`: 1.440s
- `INSERT_AND_SUCTION`: 10.686s
- `PLAN_TO_PRE_INSERT`: 1.441s
- `INSERT_AND_SUCTION`: 10.429s
- `SELECT_PRE_INSERT`: 0.510s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
