# case_005

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

## Parameters

- Seed: `1005`
- Shelf levels: `7`
- Target gap index: `2`
- Target level: `3`
- Shelf center: `(0.874, 0.106)`
- Shelf size (depth,width): `(0.236, 0.848)`
- Shelf bottom / level gap: `(0.515, 0.223)`
- Shelf board / side / back thickness: `(0.035, 0.035, 0.024)`
- Target box size: `(0.052, 0.134, 0.088)`
- Target pose: `(0.779, 0.105, 1.073)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.479s
- `ARM_STOW_SAFE`: 2.309s
- `BASE_ENTER_WORKSPACE`: 2.713s
- `LIFT_TO_BAND`: 2.213s
- `SELECT_PRE_INSERT`: 0.003s
- `PLAN_TO_PRE_INSERT`: 1.587s
- `INSERT_AND_SUCTION`: 0.646s
- `SAFE_RETREAT`: 3.278s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
