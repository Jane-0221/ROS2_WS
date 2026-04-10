# case_001

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `INSERT_AND_SUCTION`

## Parameters

- Seed: `5003`
- Shelf levels: `5`
- Target gap index: `0`
- Target level: `2`
- Shelf center: `(0.896, 0.018)`
- Shelf size (depth,width): `(0.237, 0.841)`
- Shelf bottom / level gap: `(0.507, 0.238)`
- Shelf board / side / back thickness: `(0.032, 0.046, 0.026)`
- Target box size: `(0.079, 0.157, 0.073)`
- Target pose: `(0.802, -0.012, 0.626)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.659s
- `ARM_STOW_SAFE`: 2.307s
- `BASE_ENTER_WORKSPACE`: 2.715s
- `LIFT_TO_BAND`: 2.219s
- `SELECT_PRE_INSERT`: 0.050s
- `PLAN_TO_PRE_INSERT`: 2.236s
- `INSERT_AND_SUCTION`: 10.509s
- `SELECT_PRE_INSERT`: 0.049s
- `PLAN_TO_PRE_INSERT`: 1.463s
- `INSERT_AND_SUCTION`: 10.540s
- `SELECT_PRE_INSERT`: 0.055s
- `PLAN_TO_PRE_INSERT`: 1.460s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
