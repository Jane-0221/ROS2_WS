# case_001

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

## Parameters

- Seed: `7101`
- Shelf levels: `6`
- Target gap index: `3`
- Target level: `3`
- Shelf center: `(0.889, -0.044)`
- Shelf size (depth,width): `(0.317, 0.588)`
- Shelf bottom / level gap: `(0.411, 0.299)`
- Shelf board / side / back thickness: `(0.036, 0.045, 0.041)`
- Target box size: `(0.060, 0.085, 0.094)`
- Target pose: `(0.751, -0.074, 1.458)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.609s
- `ARM_STOW_SAFE`: 2.326s
- `BASE_ENTER_WORKSPACE`: 2.714s
- `LIFT_TO_BAND`: 2.211s
- `SELECT_PRE_INSERT`: 0.364s
- `PLAN_TO_PRE_INSERT`: 1.533s
- `INSERT_AND_SUCTION`: 0.666s
- `SAFE_RETREAT`: 2.865s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
