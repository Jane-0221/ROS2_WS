# case_007

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `FAILED`
- Failure message: `Pre-insert trajectory violates the R1 stage-motion limit.`

## Parameters

- Seed: `4007`
- Shelf levels: `5`
- Target gap index: `0`
- Target level: `2`
- Shelf center: `(0.968, 0.070)`
- Shelf size (depth,width): `(0.202, 0.675)`
- Shelf bottom / level gap: `(0.475, 0.299)`
- Shelf board / side / back thickness: `(0.027, 0.034, 0.031)`
- Target box size: `(0.064, 0.152, 0.055)`
- Target pose: `(0.883, 0.034, 0.625)`

## Stage Durations

- `ACQUIRE_TARGET`: 2.019s
- `ARM_STOW_SAFE`: 2.306s
- `BASE_ENTER_WORKSPACE`: 2.711s
- `LIFT_TO_BAND`: 2.210s
- `SELECT_PRE_INSERT`: 0.005s
- `PLAN_TO_PRE_INSERT`: 0.596s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
