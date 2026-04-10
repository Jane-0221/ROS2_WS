# case_004

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `FAILED`
- Failure message: `Pre-insert planning failed: MoveIt failed to produce a valid trajectory (TIMED_OUT, code=-6).; retry failed: MoveIt failed to produce a valid trajectory (INVALID_MOTION_PLAN, code=-2).`

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

- `ACQUIRE_TARGET`: 0.000s
- `ARM_STOW_SAFE`: 5.810s
- `BASE_ENTER_WORKSPACE`: 2.717s
- `LIFT_TO_BAND`: 2.215s
- `SELECT_PRE_INSERT`: 0.023s
- `PLAN_TO_PRE_INSERT`: 15.351s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
