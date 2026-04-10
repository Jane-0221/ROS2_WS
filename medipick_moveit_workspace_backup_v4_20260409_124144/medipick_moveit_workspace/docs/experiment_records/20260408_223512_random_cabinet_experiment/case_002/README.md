# case_002

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `INSERT_AND_SUCTION`

## Parameters

- Seed: `4008`
- Shelf levels: `4`
- Target gap index: `0`
- Target level: `2`
- Shelf center: `(0.807, -0.010)`
- Shelf size (depth,width): `(0.227, 0.822)`
- Shelf bottom / level gap: `(0.516, 0.232)`
- Shelf board / side / back thickness: `(0.044, 0.047, 0.032)`
- Target box size: `(0.104, 0.099, 0.063)`
- Target pose: `(0.716, -0.008, 0.632)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.609s
- `ARM_STOW_SAFE`: 2.303s
- `BASE_ENTER_WORKSPACE`: 2.713s
- `LIFT_TO_BAND`: 2.213s
- `SELECT_PRE_INSERT`: 0.022s
- `PLAN_TO_PRE_INSERT`: 15.148s
- `INSERT_AND_SUCTION`: 5.392s
- `SELECT_PRE_INSERT`: 0.026s
- `PLAN_TO_PRE_INSERT`: 1.453s
- `INSERT_AND_SUCTION`: 5.461s
- `SELECT_PRE_INSERT`: 0.023s
- `PLAN_TO_PRE_INSERT`: 1.495s
- `INSERT_AND_SUCTION`: 5.394s
- `SELECT_PRE_INSERT`: 0.022s
- `PLAN_TO_PRE_INSERT`: 1.474s
- `INSERT_AND_SUCTION`: 5.355s
- `SELECT_PRE_INSERT`: 0.022s
- `PLAN_TO_PRE_INSERT`: 1.524s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
