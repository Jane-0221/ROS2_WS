# case_001

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`

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

- `ACQUIRE_TARGET`: 0.695s
- `ARM_STOW_SAFE`: 2.302s
- `BASE_ENTER_WORKSPACE`: 2.721s
- `LIFT_TO_BAND`: 2.211s
- `SELECT_PRE_INSERT`: 0.024s
- `PLAN_TO_PRE_INSERT`: 13.824s
- `INSERT_AND_SUCTION`: 0.701s
- `SAFE_RETREAT`: 3.271s

## Video

- No video metadata was generated for this case.

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
