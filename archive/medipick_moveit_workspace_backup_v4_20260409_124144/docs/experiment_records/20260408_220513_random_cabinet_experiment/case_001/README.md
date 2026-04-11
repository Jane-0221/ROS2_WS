# case_001

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

## Parameters

- Seed: `4001`
- Shelf levels: `7`
- Target gap index: `1`
- Target level: `3`
- Shelf center: `(0.888, -0.000)`
- Shelf size (depth,width): `(0.240, 0.806)`
- Shelf bottom / level gap: `(0.555, 0.279)`
- Shelf board / side / back thickness: `(0.039, 0.058, 0.037)`
- Target box size: `(0.077, 0.134, 0.087)`
- Target pose: `(0.788, -0.008, 0.974)`

## Stage Durations

- `ACQUIRE_TARGET`: 1.648s
- `ARM_STOW_SAFE`: 5.828s
- `BASE_ENTER_WORKSPACE`: 2.716s
- `LIFT_TO_BAND`: 2.216s
- `SELECT_PRE_INSERT`: 0.004s
- `PLAN_TO_PRE_INSERT`: 1.541s
- `INSERT_AND_SUCTION`: 0.645s
- `SAFE_RETREAT`: 3.259s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
