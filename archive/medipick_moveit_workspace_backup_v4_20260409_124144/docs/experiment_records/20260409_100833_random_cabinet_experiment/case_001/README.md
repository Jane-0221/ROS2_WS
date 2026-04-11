# case_001

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

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

- `ACQUIRE_TARGET`: 0.667s
- `ARM_STOW_SAFE`: 2.308s
- `BASE_ENTER_WORKSPACE`: 2.717s
- `LIFT_TO_BAND`: 2.218s
- `SELECT_PRE_INSERT`: 0.429s
- `PLAN_TO_PRE_INSERT`: 6.542s
- `INSERT_AND_SUCTION`: 1.688s
- `SAFE_RETREAT`: 3.622s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
