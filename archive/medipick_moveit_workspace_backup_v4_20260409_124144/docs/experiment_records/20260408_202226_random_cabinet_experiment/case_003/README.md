# case_003

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

## Parameters

- Seed: `1003`
- Shelf levels: `7`
- Target gap index: `5`
- Target level: `3`
- Shelf center: `(0.883, 0.009)`
- Shelf size (depth,width): `(0.211, 0.770)`
- Shelf bottom / level gap: `(0.473, 0.266)`
- Shelf board / side / back thickness: `(0.024, 0.045, 0.035)`
- Target box size: `(0.080, 0.134, 0.082)`
- Target pose: `(0.796, 0.009, 1.936)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.269s
- `ARM_STOW_SAFE`: 2.303s
- `BASE_ENTER_WORKSPACE`: 1.019s
- `LIFT_TO_BAND`: 2.213s
- `SELECT_PRE_INSERT`: 0.005s
- `PLAN_TO_PRE_INSERT`: 2.117s
- `INSERT_AND_SUCTION`: 0.648s
- `SAFE_RETREAT`: 3.226s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
