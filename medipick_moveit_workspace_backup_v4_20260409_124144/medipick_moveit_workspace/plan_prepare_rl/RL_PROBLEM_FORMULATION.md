# RL Problem Formulation

## 1. Main Recommendation

Treat `PLAN_TO_PRE_INSERT` as a **goal-conditioned constrained motion policy**.

The policy should not choose the medicine target and should not choose the whole task sequence.
For the first version, it should solve only:

- current stage start state
- selected `pre-insert` goal pose
- cabinet geometry
- move to the goal safely and with reasonable body posture

## 2. Replace Which Part First

Do **not** replace `SELECT_PRE_INSERT` and `PLAN_TO_PRE_INSERT` at the same time.

Recommended first replacement:

- keep the current geometric `pre-insert` candidate selection
- replace only the motion generation from start state to selected candidate

So the RL policy input includes:

- start state
- chosen `pre-insert` pose

and the policy output is:

- a sequence of control actions that reaches that pose

## 3. Recommended Variants

### Variant A: Fixed-base prepare policy

Action controls:

- `raise_joint`
- `r1_joint`
- `r2_joint`
- `r3_joint`
- `r4_joint`
- `r5_joint`
- `r6_joint`

Base is frozen during the stage.

This is the cleanest first baseline.

### Variant B: Residual-base prepare policy

Action controls:

- small residual `base_x`
- small residual `base_y`
- small residual `base_theta`
- `raise_joint`
- `r1_joint`
- `r2_joint`
- `r3_joint`
- `r4_joint`
- `r5_joint`
- `r6_joint`

This matches current real-world behavior better, but should be introduced only after Variant A.

## 4. Why Variant A First

The current observed bottleneck is:

- `mobile_arm` sometimes reaches the correct end-effector pose with a bad whole-body solution

That means the first RL baseline should not give the base too much freedom.

If Variant A cannot solve enough cases, then Variant B can add bounded residual base motion.

## 5. Observation Design

### 5.1 Policy should observe goal-relative state, not only raw joint state

The observation should include:

#### Robot configuration

- current controlled joint positions
- current controlled joint velocities

#### Goal-relative end-effector state

- end-effector position error to selected `pre-insert` pose
- end-effector orientation error to selected `pre-insert` pose
- end-effector linear velocity
- end-effector angular velocity

#### Cabinet-relative geometry

- signed distance to cabinet entrance plane
- signed distance to left side wall
- signed distance to right side wall
- signed distance to lower shelf plane
- signed distance to upper shelf plane
- signed distance to back panel plane

#### Stage-relative posture constraints

- base planar displacement from stage start
- base yaw displacement from stage start
- `r1` displacement from stage start

#### Action history

- previous action

### 5.2 Recommended final observation blocks

Recommended blocks for Variant A:

1. joint positions
2. joint velocities
3. tool pose error to goal
4. tool velocity
5. cabinet clearance features
6. `r1` relative displacement
7. previous action

Recommended blocks for Variant B:

add:

8. base displacement from stage start
9. base yaw displacement from stage start

## 6. Observation Coordinates

Use a stable local frame.

Recommended:

- origin: base frame at stage start
- represent goal and current tool pose in this frame

This keeps the problem stationary and easier to learn.

## 7. Action Design

### Variant A

Use continuous residual target increments:

- `d_raise`
- `d_r1`
- `d_r2`
- `d_r3`
- `d_r4`
- `d_r5`
- `d_r6`

These are not torques.
They are small target changes applied to position-controlled joints.

### Variant B

Use:

- `d_base_x`
- `d_base_y`
- `d_base_theta`
- `d_raise`
- `d_r1`
- `d_r2`
- `d_r3`
- `d_r4`
- `d_r5`
- `d_r6`

with very small per-step limits.

## 8. Realism Constraints

### Lift realism

The real lift is not an ideal continuous target joint.
It behaves more like:

- up
- down
- stop

So in MuJoCo, either:

- discretize the lift action into 3 commands, or
- use a continuous action and quantize it internally to `{-1, 0, +1}`

### Base realism

The real base is not very precise.
So for Variant B:

- include action delay
- include execution noise
- include small odometry bias/noise

## 9. Goal Definition

Success should not mean only:

- end-effector reached the goal pose

It should also require:

- goal is reached outside cabinet
- no collision
- posture remains reasonable
- base residual motion stays within bound
- `r1` residual motion stays within bound
- final velocity is small enough

## 10. Episode Horizon

Recommended starting horizon:

- 60 to 100 control steps

Recommended control rate:

- 5 to 10 Hz high-level control

This is long enough for a short prepare motion,
but short enough that the policy learns urgency.

## 11. Recommended Algorithms

### Primary recommendation

- `SAC`

Why:

- continuous action
- good sample efficiency
- robust for MuJoCo control
- works well with shaped rewards

### Secondary baseline

- `PPO`

Why:

- very standard
- easier comparison baseline

### Not recommended as first step

- diffusion policy
- offline RL
- pure imitation learning from current logs only

Those need better data coverage than the project currently records.

## 12. Recommended Network

Start simple:

- actor: MLP, 3 layers, width 256 or 512
- critic: twin Q networks, same size
- activation: ReLU or SiLU
- action squashing: tanh

If observation becomes highly structured, later consider:

- separate encoders for robot state and scene geometry

## 13. Curriculum Recommendation

### Stage 1

- fixed base
- no clutter
- no sensor noise
- easy cabinet sizes

### Stage 2

- same as Stage 1, but full cabinet randomization

### Stage 3

- add start-state noise
- add actuation lag/noise

### Stage 4

- enable residual base action
- keep base residual tightly bounded

### Stage 5

- add clutter and more realistic perception noise

## 14. What the RL Policy Should Not Do in V1

Do not let the first version:

- choose the cabinet
- choose the target object
- choose the final grasp pose
- choose among many `pre-insert` candidates
- solve the final insert stage

That would make debugging much harder.

## 15. Success Criterion for the RL Project

The RL project is successful if it can beat the current bottleneck on:

- `PLAN_TO_PRE_INSERT` success rate
- average planning/execution time for the stage
- reduced bad whole-body postures
- fewer failures caused by excessive base motion or excessive `r1`
