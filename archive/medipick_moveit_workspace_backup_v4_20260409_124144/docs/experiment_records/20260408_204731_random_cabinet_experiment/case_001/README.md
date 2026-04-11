# case_001

![scene](./scene.svg)

## Result

- Success: `True`
- Final stage: `COMPLETED`
- Video recorded: `True`

## Parameters

- Seed: `1001`
- Shelf levels: `4`
- Target gap index: `1`
- Target level: `2`
- Shelf center: `(0.970, 0.013)`
- Shelf size (depth,width): `(0.208, 0.839)`
- Shelf bottom / level gap: `(0.468, 0.208)`
- Shelf board / side / back thickness: `(0.039, 0.047, 0.043)`
- Target box size: `(0.052, 0.156, 0.085)`
- Target pose: `(0.882, 0.066, 0.780)`

## Stage Durations

- `ACQUIRE_TARGET`: 0.040s
- `ARM_STOW_SAFE`: 2.308s
- `BASE_ENTER_WORKSPACE`: 2.713s
- `LIFT_TO_BAND`: 2.217s
- `SELECT_PRE_INSERT`: 0.005s
- `PLAN_TO_PRE_INSERT`: 1.601s
- `INSERT_AND_SUCTION`: 0.630s
- `SAFE_RETREAT`: 3.274s

## Video

- Record success: `True`
- Reason: `stop_file`
- Frames: `163`
- Duration: `81.500s`
- Stored size: `400x287`
- Video file: [rviz_capture.avi](./rviz_capture.avi)
- Recorder log: [video.log](./video.log)

## Files

- `scene.svg`: cabinet image
- `params.json`: generated cabinet parameters
- `result.json`: parsed experiment result
- `run.log`: raw ROS/MoveIt log
- `rviz_capture.avi`: recorded RViz video
- `video.json`: recorder metadata
- `video.log`: recorder stdout/stderr
