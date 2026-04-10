# case_001

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

## Parameters

- Seed: `7001`
- Shelf levels: `4`
- Target gap index: `2`
- Target level: `2`
- Shelf center: `(0.951, 0.046)`
- Shelf size (depth,width): `(0.311, 0.820)`
- Shelf bottom / level gap: `(0.425, 0.231)`
- Shelf board / side / back thickness: `(0.025, 0.036, 0.035)`
- Target box size: `(0.068, 0.106, 0.070)`
- Target pose: `(0.829, -0.020, 1.002)`

## Stage Durations

- `ACQUIRE_TARGET`: 1.643s
- `ARM_STOW_SAFE`: 2.792s
- `BASE_ENTER_WORKSPACE`: 2.716s
- `LIFT_TO_BAND`: 2.212s
- `SELECT_PRE_INSERT`: 0.383s
- `PLAN_TO_PRE_INSERT`: 1.514s
- `INSERT_AND_SUCTION`: 0.689s
- `SAFE_RETREAT`: 2.886s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
