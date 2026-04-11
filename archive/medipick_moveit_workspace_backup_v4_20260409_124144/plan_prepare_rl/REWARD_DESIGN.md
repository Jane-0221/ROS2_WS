# Reward Design

## 1. Design Goal

The reward should teach the policy to:

- reach the selected `pre-insert` pose
- remain outside the cabinet
- avoid collisions
- keep body posture reasonable
- minimize unnecessary base motion
- minimize unnecessary `r1` swing

## 2. Success Definition

Success should require all of the following:

- tool position error below threshold
- tool orientation error below threshold
- tool still outside the cabinet entrance plane
- no collision
- low terminal velocity
- base residual motion within allowed bound
- `r1` residual motion within allowed bound

### Suggested thresholds

- tool position error <= `0.02 m`
- tool orientation error <= `0.10 rad`
- outside margin >= `0.01 m`
- base planar residual <= `0.20 m` for residual-base RL, or `0` for fixed-base RL
- base yaw residual <= `20 deg` for residual-base RL
- `r1` residual <= `90 deg` soft safe zone, `100 deg` hard reject

## 3. Recommended Reward Form

Use a dense potential-based reward with sparse terminal bonuses.

Recommended structure:

`reward = progress_reward + shaping_terms - penalties + terminal_bonus`

## 4. Potential Function

Define a potential:

`Phi(s) = w_pos * d_pos + w_ori * d_ori + w_out * outside_violation + w_base * d_base + w_r1 * d_r1`

Where:

- `d_pos`: end-effector position error to selected `pre-insert`
- `d_ori`: orientation error
- `outside_violation`: amount by which the tool entered the forbidden cabinet side
- `d_base`: base residual motion from stage start
- `d_r1`: `r1` residual motion from stage start

Then use:

`progress_reward = alpha * (Phi(s_t) - Phi(s_t+1))`

This encourages monotonic progress.

## 5. Dense Reward Terms

### 5.1 End-effector position progress

Reward improvement in:

- Euclidean tool position error to goal

This is the most important dense term.

### 5.2 End-effector orientation progress

Reward reduction in:

- tool orientation error to goal

Use a moderate weight, smaller than position.

### 5.3 Outside-cabinet safety term

Heavy penalty if the tool or arm crosses too far into the cabinet before success.

This stage should stop outside.

### 5.4 Base restraint term

Penalize:

- base planar displacement from stage start
- base yaw displacement from stage start

This is essential for this project because the current real bottleneck is bad whole-body posture despite valid tool pose.

### 5.5 `r1` restraint term

Penalize:

- absolute `r1` displacement from stage start

This should start softly before the hard limit.

### 5.6 Smoothness term

Penalize:

- squared action magnitude
- squared action change

This reduces oscillation.

## 6. Sparse Terminal Rewards

### Success bonus

Give a strong positive terminal bonus:

- example: `+50` to `+100`

### Collision penalty

Give a strong negative terminal penalty:

- example: `-50`

### Constraint violation penalty

If:

- base residual exceeds hard bound
- or `r1` exceeds hard bound

terminate with strong negative penalty.

## 7. Suggested Reward Terms Table

| Term | Symbol | Suggested sign |
| --- | --- | --- |
| Position progress | `r_pos` | positive |
| Orientation progress | `r_ori` | positive |
| Outside-cabinet violation | `p_outside` | negative |
| Collision | `p_collision` | negative |
| Base planar residual | `p_base_xy` | negative |
| Base yaw residual | `p_base_yaw` | negative |
| `r1` residual | `p_r1` | negative |
| Action magnitude | `p_action` | negative |
| Action jerk | `p_action_delta` | negative |
| Time penalty | `p_time` | negative |
| Success bonus | `b_success` | positive |

## 8. Suggested Initial Weights

These are only starting points.

- `w_pos = 8.0`
- `w_ori = 2.0`
- `w_outside = 12.0`
- `w_base_xy = 4.0`
- `w_base_yaw = 2.0`
- `w_r1 = 3.0`
- `w_action = 0.01`
- `w_action_delta = 0.02`
- `w_time = 0.02`
- `b_success = 80.0`
- `p_collision = -80.0`
- `p_hard_constraint = -60.0`

## 9. Hard vs Soft Constraints

Use both.

### Soft constraints

Start penalizing before hard failure:

- base planar residual above `0.10 m`
- base yaw residual above `10 deg`
- `r1` residual above `70 deg`

### Hard constraints

Terminate if:

- base planar residual > configured limit
- base yaw residual > configured limit
- `r1` residual > configured limit
- collision

## 10. Recommended Reward Stages

### Stage A: easy shaping

Use only:

- position progress
- orientation progress
- collision penalty
- success bonus

### Stage B: add posture discipline

Add:

- base penalties
- `r1` penalties

### Stage C: full realistic reward

Add:

- outside-cabinet penalties
- smoothness terms
- time penalty

## 11. Why Progress Reward Matters

If you only use sparse success/failure:

- training will be slow
- policy may discover awkward postures that happen to succeed

Progress reward keeps learning aligned with the real geometric objective.

## 12. Why Posture Penalties Matter Here

For this project, end-effector-only success is not enough.

The current failure cases show that the robot can:

- reach the correct point
- but use an unreasonable base or arm posture

So the reward must encode:

- how the robot gets there

not only:

- whether the tool gets there

## 13. Recommended Ablation Order

When training:

1. train without base penalties
2. add base penalties
3. add `r1` penalties
4. compare success and posture metrics

This will tell you whether the RL policy really solves the same bottleneck as the current MoveIt pipeline.
