# PLAN_TO_PRE_INSERT RL Package

## Purpose

This folder is a self-contained design package for replacing or augmenting the current
`PLAN_TO_PRE_INSERT` stage with a MuJoCo-based reinforcement learning policy.

It is written so that another Codex session can work by looking only at this folder.

## Recommended Reading Order

1. `CURRENT_STAGE_SPEC.md`
2. `RL_PROBLEM_FORMULATION.md`
3. `MUJOCO_ENVIRONMENT.md`
4. `REWARD_DESIGN.md`
5. `TRAINING_PLAN.md`
6. `DATA_LOGGING_AND_DATASET.md`

## Folder Structure

- `CURRENT_STAGE_SPEC.md`
  - Exact description of the current `PLAN_TO_PRE_INSERT` stage in the ROS 2 / MoveIt stack.
- `RL_PROBLEM_FORMULATION.md`
  - Recommended RL task definition, action/observation choices, and scope boundaries.
- `MUJOCO_ENVIRONMENT.md`
  - How to build the MuJoCo environment and what must be randomized.
- `REWARD_DESIGN.md`
  - Detailed reward, termination, and curriculum proposal.
- `TRAINING_PLAN.md`
  - Step-by-step implementation and training plan.
- `DATA_LOGGING_AND_DATASET.md`
  - What to log now so the same pipeline can later support imitation learning or offline RL.
- `configs/`
  - YAML files for task, reward, model, and curriculum defaults.
- `python/`
  - Small helper modules for geometry, observations, constraints, and reward logic.
- `schemas/`
  - JSON schema-like specifications for episode and step recording.

## Core Recommendation

Do not start with end-to-end whole-task RL.

Start with a narrow task:

- input: current robot state at the beginning of `PLAN_TO_PRE_INSERT`
- goal: reach the selected `pre-insert` pose
- constraints: stay outside the cabinet, avoid collisions, keep `r1` motion reasonable,
  and keep base residual motion small

In other words:

- the candidate selection stage stays outside the RL policy for the first version
- the RL policy only solves the motion problem from stage start state to the already-selected
  `pre-insert` goal

## Recommended First Baseline

Use a goal-conditioned continuous control policy in MuJoCo with position-control targets:

- algorithm: `SAC`
- policy family: MLP actor-critic
- first action space: `raise + r1..r6`, with base frozen
- second action space: add small residual base motion if needed

## Why This Scope

The current bottleneck is not generic full-task manipulation.
It is specifically:

- `PLAN_TO_PRE_INSERT` sometimes chooses or executes a motion that reaches the end-effector pose
  but uses an unreasonable whole-body posture
- especially too much base motion or too much `r1` motion

That makes this stage a good candidate for a constrained RL sub-problem.
