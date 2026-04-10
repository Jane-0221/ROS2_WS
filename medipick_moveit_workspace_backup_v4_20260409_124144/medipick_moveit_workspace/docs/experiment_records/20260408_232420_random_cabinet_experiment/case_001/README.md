# case_001

![scene](./scene.svg)

## Result

- Success: `False`
- Final stage: `FAILED`
- Failure message: `No feasible outside-cabinet pre-insert candidate found.`

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

- `ACQUIRE_TARGET`: 0.627s
- `ARM_STOW_SAFE`: 2.305s
- `BASE_ENTER_WORKSPACE`: 0.000s
- `LIFT_TO_BAND`: 2.603s
- `SELECT_PRE_INSERT`: 0.330s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
