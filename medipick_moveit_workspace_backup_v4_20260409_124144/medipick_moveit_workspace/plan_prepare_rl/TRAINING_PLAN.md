# Training Plan

## 1. Project Goal

Build an RL policy that solves the current `PLAN_TO_PRE_INSERT` stage better than the existing bottleneck cases.

The policy should first be evaluated in MuJoCo only.

## 2. Milestones

### Milestone 0: reference extraction

Goal:

- reproduce current stage behavior in a standalone simulator problem definition

Deliverables:

- stage-start state definition
- selected goal pose definition
- success and failure conditions

### Milestone 1: fixed-base baseline

Goal:

- train a policy with base frozen

Deliverables:

- `PrepareFixedBase-v1`
- first success curves on easy cabinets

### Milestone 2: randomized cabinet generalization

Goal:

- train on the same cabinet randomization range as the current project

Deliverables:

- held-out evaluation on random seeds
- success rate and failure breakdown

### Milestone 3: residual-base policy

Goal:

- allow small base residual motion

Deliverables:

- `PrepareResidualBase-v1`
- comparison against fixed-base baseline

### Milestone 4: sim realism

Goal:

- add execution noise and observation noise

Deliverables:

- robustness curves
- policy comparison before and after randomization

## 3. Suggested Implementation Order

1. implement the environment with hand-coded reset and reward
2. verify reward and termination with scripted policies
3. train fixed-base SAC
4. run evaluation on hard scenes
5. enable residual base action
6. add domain randomization
7. connect real recorded stage starts if available

## 4. Hard-Scene Evaluation Set

Maintain a fixed evaluation set that always includes:

- seed `4007`
- seed `5004`
- seed `5008`

These are important because they expose the known bottlenecks.

## 5. Success Metrics

Primary metrics:

- success rate
- mean final tool pose error
- mean base residual motion
- mean `r1` residual motion
- collision rate

Secondary metrics:

- episode length
- action smoothness
- cabinet clearance margin

## 6. Comparison Baselines

Always compare RL against:

### Baseline A

Current seeded IK + MoveIt stage logic

### Baseline B

Current stage with `pre_insert_group_name = arm`

### Baseline C

Current stage with `mobile_arm` and base residual limits

## 7. Data Needed During Training

For every episode log:

- seed
- cabinet parameters
- start state
- selected `pre-insert` goal
- action sequence
- reward sequence
- termination reason
- success/failure
- final state

## 8. Recommended Training Strategy

### Phase 1

Use easy cabinets:

- moderate depth
- moderate width
- center targets

### Phase 2

Use full current randomization range.

### Phase 3

Oversample hard cases:

- lateral targets
- shallow and deep shelves
- targets near current failure seeds

## 9. Failure Analysis Loop

After each training round:

1. find failed evaluation seeds
2. classify failure
3. decide whether the issue is:
   - poor reward
   - poor observation
   - poor action limits
   - environment mismatch
4. update only one factor at a time

## 10. Recommended First Experiments

### Experiment 1

- fixed base
- no clutter
- no noise
- SAC

### Experiment 2

- fixed base
- full cabinet randomization

### Experiment 3

- residual base
- tight base residual limits

### Experiment 4

- residual base
- observation noise and actuation lag

## 11. What Not To Optimize Too Early

Do not spend early time on:

- large transformer policies
- image-based policies
- diffusion models
- offline RL

First prove that the narrow MuJoCo control problem is learnable.

## 12. Decision Point After Baseline

After fixed-base baseline:

### If fixed-base works well

Then:

- keep RL stage base-free
- use upstream deterministic base placement

### If fixed-base fails often

Then:

- enable residual base action
- but keep strong residual constraints

## 13. Sim-to-Real Strategy

Do not send the learned policy directly to the real robot first.

Use staged deployment:

1. MuJoCo only
2. replay evaluation on recorded starts
3. shadow mode in ROS
4. limited execution on safe cases

## 14. Immediate Next Coding Steps

1. implement environment reset
2. implement observation builder
3. implement reward function
4. implement scripted sanity-check controller
5. verify success condition on a handful of seeds
6. train SAC baseline
