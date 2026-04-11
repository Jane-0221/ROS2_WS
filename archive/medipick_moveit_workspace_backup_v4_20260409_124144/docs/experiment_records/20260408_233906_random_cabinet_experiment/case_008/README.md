# case_008

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `FAILED`
- Failure message: `Pre-insert planning failed: MoveIt failed to produce a valid trajectory (INVALID_MOTION_PLAN, code=-2).; retry failed: MoveIt failed to produce a valid trajectory (FAILURE, code=99999).`

## Parameters

- Seed: `5008`
- Shelf levels: `4`
- Target gap index: `0`
- Target level: `2`
- Shelf center: `(0.843, 0.090)`
- Shelf size (depth,width): `(0.282, 0.778)`
- Shelf bottom / level gap: `(0.402, 0.234)`
- Shelf board / side / back thickness: `(0.026, 0.052, 0.036)`
- Target box size: `(0.080, 0.101, 0.054)`
- Target pose: `(0.722, -0.001, 0.519)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.644s
- `ARM_STOW_SAFE`: 2.305s
- `BASE_ENTER_WORKSPACE`: 0.216s
- `LIFT_TO_BAND`: 2.209s
- `SELECT_PRE_INSERT`: 0.022s
- `PLAN_TO_PRE_INSERT`: 14.674s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
