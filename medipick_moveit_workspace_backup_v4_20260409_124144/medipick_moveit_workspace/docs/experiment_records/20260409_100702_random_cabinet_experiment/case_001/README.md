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

- `ACQUIRE_TARGET`: 0.702s
- `ARM_STOW_SAFE`: 2.300s
- `BASE_ENTER_WORKSPACE`: 2.715s
- `LIFT_TO_BAND`: 2.215s
- `SELECT_PRE_INSERT`: 0.410s
- `PLAN_TO_PRE_INSERT`: 5.967s
- `INSERT_AND_SUCTION`: 0.255s
- `PLAN_TO_PRE_INSERT`: 1.525s
- `INSERT_AND_SUCTION`: 0.250s
- `PLAN_TO_PRE_INSERT`: 1.459s
- `INSERT_AND_SUCTION`: 0.248s
- `SELECT_PRE_INSERT`: 0.402s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
