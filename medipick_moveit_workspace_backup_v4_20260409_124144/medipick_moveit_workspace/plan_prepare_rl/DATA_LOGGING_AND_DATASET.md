# Data Logging And Dataset Plan

## 1. Why This Matters

Even if the first solution is RL, you should still record structured data.

That data can later support:

- imitation learning
- candidate ranking
- offline RL
- failure prediction

## 2. What To Record Per Episode

### Scene

- cabinet geometry
- target box geometry
- target pose
- optional clutter geometry

### Stage-start state

- full joint positions
- full joint velocities
- end-effector pose
- base pose
- selected `pre-insert` goal pose

### Rollout

- observation at every step
- action at every step
- reward terms at every step
- contact events
- termination reason

### Outcome

- success
- total return
- final pose error
- final base residual
- final `r1` residual

## 3. What To Record From Current ROS Pipeline

From the existing real project, the most valuable fields are:

- stage-start joint state for `PLAN_TO_PRE_INSERT`
- selected `pre-insert` candidate queue
- chosen candidate index
- full planned trajectory
- final executed trajectory
- final measured joint state
- exact failure reason if failed

## 4. Minimal Dataset To Start With

The first useful dataset can be:

- `scene.json`
- `stage_start.json`
- `goal.json`
- `episode.json`
- `steps.npz`

## 5. Labels For Supervised Learning

Even before RL, you can learn:

- whether a candidate will succeed
- whether `arm` or `mobile_arm` should be used
- what `base_standoff` should be
- what `pre_insert_offset` should be

For that you need labels like:

- `success`
- `failure_type`
- `selected_group`
- `planning_time`
- `final_pose_error`

## 6. Data Quality Requirements

Do not only record success.

You must record:

- success episodes
- failure episodes
- near-success episodes
- which constraints were violated

## 7. MuJoCo Dataset Format Recommendation

Recommended files:

- `episodes/episode_000001/scene.json`
- `episodes/episode_000001/task.json`
- `episodes/episode_000001/rollout.npz`
- `episodes/episode_000001/result.json`

Where:

- `rollout.npz` contains `obs`, `actions`, `rewards`, `done`, `info`

## 8. Use Of The Dataset

### Short term

Use it to:

- debug reward
- debug curriculum
- analyze failure clusters

### Medium term

Use it to train:

- candidate ranking models
- parameter regressors
- success predictors

### Long term

Use it for:

- imitation initialization
- offline RL

## 9. Practical Recommendation

Implement logging from day one.

It is much easier to discard fields later than to realize later that critical information was never saved.
