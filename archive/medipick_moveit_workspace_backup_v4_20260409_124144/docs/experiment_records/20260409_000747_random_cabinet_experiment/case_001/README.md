# case_001

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

## Parameters

- Seed: `6001`
- Shelf levels: `6`
- Target gap index: `1`
- Target level: `3`
- Shelf center: `(0.831, 0.035)`
- Shelf size (depth,width): `(0.307, 0.832)`
- Shelf bottom / level gap: `(0.519, 0.264)`
- Shelf board / side / back thickness: `(0.044, 0.035, 0.030)`
- Target box size: `(0.095, 0.149, 0.079)`
- Target pose: `(0.712, 0.077, 0.915)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.683s
- `ARM_STOW_SAFE`: 2.291s
- `BASE_ENTER_WORKSPACE`: 2.714s
- `LIFT_TO_BAND`: 2.214s
- `SELECT_PRE_INSERT`: 0.047s
- `PLAN_TO_PRE_INSERT`: 1.532s
- `INSERT_AND_SUCTION`: 0.685s
- `SAFE_RETREAT`: 3.285s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
