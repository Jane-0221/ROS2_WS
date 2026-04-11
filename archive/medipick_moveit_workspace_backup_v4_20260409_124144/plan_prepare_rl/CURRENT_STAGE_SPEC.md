# Current Stage Specification

## 1. Scope

This document describes the current real implementation of the `PLAN_TO_PRE_INSERT` stage.

It is based on the current repository behavior, not on an older conceptual document.

The relevant source files in the main project are:

- `src/medipick_planning_server/scripts/pick_task_manager.py`
- `src/medipick_planning_server/scripts/pick_task_flow.py`
- `src/medipick_planning_server/scripts/pick_task_services.py`
- `src/medipick_planning_server/scripts/pick_task_utils.py`

## 2. Stage Boundary

### Upstream stages

Before `PLAN_TO_PRE_INSERT`, the system already completed:

1. `ACQUIRE_TARGET`
2. `ARM_STOW_SAFE`
3. `BASE_ENTER_WORKSPACE`
4. `LIFT_TO_BAND`
5. `SELECT_PRE_INSERT`

### Downstream stages

If `PLAN_TO_PRE_INSERT` succeeds, the system enters:

- `INSERT_AND_SUCTION`

If it fails, it either:

- switches to the next queued `pre-insert` candidate, or
- goes to `FAILED`

## 3. What This Stage Is Actually Solving

This stage does **not** choose the target medicine pose.
It does **not** choose the cabinet.
It does **not** do the final insert.

It solves exactly this:

- given the current robot state after base and lift alignment
- and given one selected `pre-insert` pose outside the cabinet
- compute and execute a trajectory that moves the robot from the current state to that pose

## 4. Start State

The stage starts from the current measured joint state after:

- arm has been stowed for safe base motion
- base has moved into the workspace
- lift has entered the target height band

The real start state is therefore not a single fixed pose.
It is a distribution produced by the upstream pipeline.

### Stage-start robot configuration components

The current state typically includes:

- `base_x`
- `base_y`
- `base_theta`
- `raise_joint`
- `r1_joint`
- `r2_joint`
- `r3_joint`
- `r4_joint`
- `r5_joint`
- `r6_joint`
- plus auxiliary joints that may still exist in the message

### Important upstream facts

- `ARM_STOW_SAFE` uses a stow template called `transit_stow_v0`
- `BASE_ENTER_WORKSPACE` already performs a coarse base move
- `LIFT_TO_BAND` aligns height coarsely, not perfectly

### Current stow template values

The current stow template in the main project is approximately:

- `raise_joint = 0.302`
- `r1_joint = -0.681 rad` (`-39.0 deg`)
- `r2_joint = -1.309 rad` (`-75.0 deg`)
- `r3_joint = -1.309 rad` (`-75.0 deg`)
- `r4_joint = -1.571 rad` (`-90.0 deg`)
- `r5_joint = 0.017 rad` (`1.0 deg`)
- `r6_joint = 0.087 rad` (`5.0 deg`)

This means the policy should not assume:

- perfect base placement
- perfect lift alignment
- perfect zero-velocity initial conditions

## 5. Goal State

The goal is the selected `pre-insert` pose.

This is a pose for the task pose link:

- current pose link: `sucker_link`

### How the current code builds the goal pose

The goal is derived from the final pick pose `T_pick`:

1. move backward along tool approach axis by `retreat_distance`
2. optionally add small lateral offset
3. optionally add small vertical offset
4. optionally add small yaw perturbation

In the current code:

- the tool approach axis is the local `+X` axis of the target pose
- the selected `pre-insert` must remain outside the cabinet entrance plane

### Outside-cabinet condition

Current acceptance rule:

- `pre_insert_pose.x <= cabinet_entry_x - pre_insert_outside_margin`

with:

- `cabinet_entry_x = shelf_center_x - shelf_depth / 2`

## 6. Current Inputs

At the beginning of `PLAN_TO_PRE_INSERT`, the current implementation depends on:

### Robot state input

- full current joint state

### Goal input

- selected `pre_insert_pose`

### Scene input

- shelf center
- shelf depth
- cabinet entrance plane

### Constraint input

- `r1_stage_motion_limit_deg`
- `pre_insert_limit_base_motion`
- `pre_insert_base_translation_limit_m`
- `pre_insert_base_rotation_limit_deg`
- pose reach tolerances:
  - `prepare_projection_tolerance`
  - `prepare_axial_tolerance`
  - `prepare_orientation_tolerance`

### Planner input

- planning group, usually `mobile_arm`
- candidate planner id, currently usually `RRTConnectkConfigDefault`
- planning time budget
- retry time budget

## 7. Current Default Parameter Values

Important current defaults in the main project are:

- `pre_insert_group_name = mobile_arm`
- `insert_group_name = arm`
- `insert_fallback_group_name = mobile_arm`
- `pre_insert_select_pose_only = true`
- `pre_insert_candidate_count = 3`
- `candidate_planner_id = RRTConnectkConfigDefault`
- `candidate_planning_time = 1.0`
- `candidate_num_planning_attempts = 1`
- `candidate_retry_on_failure = true`
- `candidate_retry_planning_time = 3.0`
- `candidate_retry_num_planning_attempts = 2`
- `prepare_projection_tolerance = 0.04`
- `prepare_axial_tolerance = 0.05`
- `prepare_orientation_tolerance = 0.16`
- `pre_insert_limit_base_motion = true`
- `pre_insert_base_translation_limit_m = 0.5`
- `pre_insert_base_rotation_limit_deg = 40.0`
- `r1_stage_motion_limit_deg = 100.0`

In recent random experiments, these launch overrides were often used:

- `base_standoff = 0.58`
- `lift_end_effector_target_offset = 0.02`
- `candidate_retry_planning_time = 4.0`
- `candidate_retry_num_planning_attempts = 3`

## 8. Current Outputs

If the stage succeeds, it produces:

- one accepted trajectory from stage-start state to the selected `pre-insert` pose
- one final joint state
- one executed motion

If it fails, it produces:

- a structured failure reason in logs
- optional fallback to next queued candidate

## 9. Current Internal Logic

The current logic is:

1. Take selected candidate from `SELECT_PRE_INSERT`
2. Try `seeded IK + short joint interpolation`
3. If seeded branch fails, fall back to pose-goal planning
4. If first pose-goal attempt fails, retry with larger time budget
5. Reject trajectories that violate base motion or `r1` motion preferences
6. Reject trajectories whose FK final pose does not tightly match the selected `pre-insert`
7. Execute accepted trajectory

## 10. Current Candidate Queue Logic

The current project no longer uses only one `pre-insert` candidate.
It now keeps a small queue.

Current candidate generation logic:

1. compute a base retreat distance from cabinet geometry
2. generate a few nearby candidate specs:
   - primary centered/biased retreat
   - zero-bias retreat
   - small lateral and yaw-biased retreat
3. keep only candidates that:
   - are outside the cabinet
   - have an IK solution
   - pass state validity
   - satisfy the current base-motion and `r1` preference filters
4. sort them by a cheap score
5. try them one by one in `PLAN_TO_PRE_INSERT`

This matters for RL because it suggests two possible scopes:

- replace only the motion generation for one selected candidate
- or later replace candidate ranking as a second model

## 11. Current Acceptance Conditions

### Pose accuracy

The final FK pose must satisfy:

- projection error <= `prepare_projection_tolerance`
- axial error <= `prepare_axial_tolerance`
- orientation error <= `prepare_orientation_tolerance`

### Joint preference constraints

The current stage rejects trajectories if:

- `r1` stage motion exceeds the limit

Current default:

- soft concern starts near `70 deg`
- hard reject at `100 deg`

### Base motion constraints

For `PLAN_TO_PRE_INSERT`, the stage now also rejects trajectories if:

- base planar motion exceeds `0.5 m`
- or base yaw motion exceeds `40 deg`

These are stage-relative constraints from the stage start state.

## 12. Current Failure Modes

The main current failure modes are:

### A. Bad whole-body path to a valid end-effector pose

The end-effector can reach the selected `pre-insert` pose,
but the robot gets there with:

- excessive base motion
- excessive base rotation
- excessive `r1` motion

This is the exact bottleneck that motivated the RL design in this folder.

### B. Seeded IK interpolation invalid

The seeded IK end state exists,
but a simple interpolation from start to goal collides or becomes invalid.

### C. Pose-goal planning timeout or invalid plan

MoveIt can fail to produce a valid plan within the budget.

### D. Final FK reach check failure

A trajectory can finish but not actually satisfy the intended `pre-insert` pose tightly enough.

## 13. Current Observation Space, If Viewed As an RL Problem

The code itself is not an RL environment,
but if we reinterpret the stage as an RL problem, the true state contains:

- current full robot configuration
- current end-effector pose
- selected `pre-insert` pose
- cabinet geometry
- cabinet entrance plane
- relative base displacement from stage start
- relative `r1` displacement from stage start

This is the minimum information needed to reproduce the same decision problem.

## 14. Current Action Space, If Viewed As an RL Problem

The current code does not expose an RL action.
Instead it calls:

- seeded IK interpolation
- or MoveIt pose-goal planning

If we convert this stage to RL, the action should represent:

- a residual motion policy that moves from stage-start state to the selected `pre-insert` pose

The recommended design is given in `RL_PROBLEM_FORMULATION.md`.
