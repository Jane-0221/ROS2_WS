# case_001

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

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

- `ACQUIRE_TARGET`: 2.716s
- `ARM_STOW_SAFE`: 0.000s
- `BASE_ENTER_WORKSPACE`: 3.059s
- `LIFT_TO_BAND`: 2.213s
- `SELECT_PRE_INSERT`: 0.374s
- `PLAN_TO_PRE_INSERT`: 2.241s
- `INSERT_AND_SUCTION`: 10.715s
- `SELECT_PRE_INSERT`: 0.375s
- `PLAN_TO_PRE_INSERT`: 1.447s
- `INSERT_AND_SUCTION`: 23.257s
- `SAFE_RETREAT`: 17.487s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
