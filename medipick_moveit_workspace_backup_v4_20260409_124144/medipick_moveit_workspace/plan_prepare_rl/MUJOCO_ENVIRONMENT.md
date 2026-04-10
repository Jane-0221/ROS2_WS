# MuJoCo Environment Design

## 1. Environment Objective

The MuJoCo environment should simulate only the `PLAN_TO_PRE_INSERT` stage.

At reset, the environment already knows:

- the randomized cabinet geometry
- the selected target medicine pose
- the selected `pre-insert` goal pose
- the robot stage-start state

The agent's job is:

- move from the stage-start state to the selected `pre-insert` pose

## 2. Environment State Components

### Robot

Use the current simple4-based robot model:

- mobile base
- lift
- right arm
- suction tool

### Scene

Use:

- shelf frame
- left and right side walls
- lower and upper shelf planes for the chosen gap
- back panel
- optional target box mesh or box primitive

For V1, clutter can be disabled.

## 3. MuJoCo Modeling Recommendation

### Recommended actuation style

Use position or velocity servo targets, not raw torques, for the first version.

Reason:

- the real system is not being controlled by learned torques
- the current ROS stack is already operating at a higher planning/control abstraction
- position-target RL is easier to stabilize for this task

### Lift actuator

Implement the lift as:

- velocity-like up/down/stop command

or:

- continuous action internally quantized to `{-1, 0, +1}`

### Base actuator

If base is enabled:

- do not give the agent unrestricted continuous base motion
- use small bounded residual actions and simulate tracking noise

## 4. Reset Distribution

The environment reset must match the real stage start.

### Recommended reset pipeline

For each randomized cabinet:

1. sample cabinet geometry
2. sample target pose
3. compute current upstream target values:
   - base goal
   - lift target center
   - selected `pre-insert` pose
4. initialize robot near the expected stage-start state

### Best reset source

The best reset source is not hand-made random arm postures.
It should come from one of:

- actual recorded stage-start states from the real pipeline
- or a deterministic simulator implementation of the upstream stages

### Reset noise

Add small reset noise:

- base position noise
- base yaw noise
- lift height noise
- arm joint noise

This avoids overfitting to a single perfect start state.

## 5. Goal Pose Construction in the RL Environment

The RL environment should use the same `pre-insert` goal definition as the current system:

- selected from the current geometric selection logic
- must remain outside cabinet
- uses the tool pose, not only raw joint angles

This is important.
Do not define a different success target for RL.

## 6. Recommended Observation Implementation

Use a dict observation internally, even if the final RL library needs a flat vector.

Recommended dict fields:

- `q`
- `qd`
- `tool_pos_error`
- `tool_ori_error`
- `tool_twist`
- `base_residual`
- `r1_residual`
- `cabinet_clearance`
- `goal_pose_in_stage_frame`
- `prev_action`

Then provide:

- a flattened vector wrapper
- and optionally a debug decoder

## 7. Collision Modeling

Use contact-based penalties and termination.

At minimum distinguish:

- self collision
- tool vs cabinet collision
- arm link vs cabinet collision
- base vs cabinet collision

For this stage, tool contact with the cabinet should usually also be penalized,
because the goal is still outside the cabinet.

## 8. Cabinet Geometry Randomization

Match the current random experiment ranges.

Current project range summary:

- shelf width: about `0.58` to `0.92`
- shelf depth: about `0.18` to `0.32`
- shelf center x: about `0.80` to `0.98`
- shelf center y: about `-0.12` to `0.12`
- shelf bottom z: about `0.40` to `0.58`
- shelf level gap: about `0.19` to `0.30`
- board thickness: about `0.022` to `0.045`
- side thickness: about `0.030` to `0.060`
- back thickness: about `0.020` to `0.050`

## 9. Training / Validation Split

Do not evaluate only on the same sampled cabinets used in training.

Use:

- train seeds
- validation seeds
- held-out stress-test seeds

Also keep a list of known hard scenes like:

- current `4007`
- current `5004`
- current `5008`

## 10. Domain Randomization

To improve transfer:

- joint damping randomization
- action delay randomization
- observation noise
- base tracking noise
- lift tracking noise
- contact/friction small variation

Do not randomize everything from day one.
Add these in curriculum order.

## 11. Recommended Evaluation Metrics

For each checkpoint, report:

- stage success rate
- mean episode length
- mean final tool pose error
- collision rate
- base residual exceed rate
- `r1` exceed rate
- mean cabinet clearance at success

## 12. Suggested Environment Variants

### `PrepareFixedBase-v1`

- no base action
- no clutter
- shaped reward

### `PrepareResidualBase-v1`

- bounded base action
- base residual constraints active

### `PrepareResidualBaseNoise-v1`

- bounded base action
- observation and execution noise active

### `PrepareResidualBaseClutter-v1`

- clutter enabled
- full randomization

## 13. Practical Note

If MuJoCo conversion of the full robot is slow or messy, start with:

- simplified collision geometry
- box/capsule approximations for links

For this stage, accurate collision volume matters more than mesh visual quality.
