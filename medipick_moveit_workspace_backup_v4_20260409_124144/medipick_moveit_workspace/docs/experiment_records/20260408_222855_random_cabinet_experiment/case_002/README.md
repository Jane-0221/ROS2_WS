# case_002

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `FAILED`
- Failure message: `Pre-insert planning failed: MoveIt failed to produce a valid trajectory (TIMED_OUT, code=-6).; retry failed: MoveIt failed to produce a valid trajectory (FAILURE, code=99999).`

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

- `ACQUIRE_TARGET`: 0.874s
- `ARM_STOW_SAFE`: 2.305s
- `BASE_ENTER_WORKSPACE`: 2.712s
- `LIFT_TO_BAND`: 2.209s
- `SELECT_PRE_INSERT`: 0.024s
- `PLAN_TO_PRE_INSERT`: 5.110s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
