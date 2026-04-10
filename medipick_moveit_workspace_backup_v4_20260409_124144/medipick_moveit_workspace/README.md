# MediPick MoveIt Workspace

This is a sibling workspace created next to the original custom planner:

`/home/lzoolloozl/projects/medipick_moveit_workspace`

It does not modify the existing `medipick_planner_workspace`. The goal here is
to try MoveIt on the same robot model with the smallest practical scope.

## Scope

- Uses MoveIt for both the mobile base chain and the right arm chain
- Models `base_x / base_y / base_theta` directly in the MoveIt-specific URDF
- Keeps `raise_joint + r1_joint ... r6_joint` in the arm planning group
- Adds a `mobile_arm` whole-body planning group from `world` to `r6_link`
- Keeps `sucker_joint` as a separate tool chain attached to the arm tip
- Does not import the current point-cloud obstacle pipeline yet

This means the new workspace is good for:

- checking whether MoveIt can solve mobile-base + arm joint-space planning
- checking whether MoveIt can solve arm pose goals
- exposing a real planning service that forwards requests into `move_group`
- injecting collision boxes into the MoveIt planning scene for obstacle avoidance
- viewing trajectories in RViz
- comparing MoveIt planning behavior against the current staged planner
- doing planning preview before wiring a real controller

It is still not a drop-in replacement for the current planner node because the
current point-cloud obstacle pipeline is not bridged into MoveIt yet.

## Layout

- `src/medipick_simple3_description`: ROS 2 description package with a MoveIt-safe URDF copy
- `src/medipick_moveit_config`: MoveIt config package, launch files, and RViz config
- `src/medipick_planning_interfaces`: planning service interfaces
- `src/medipick_planning_server`: planning service, fake perception, and RViz demo launch files
- `scripts`: one-click build and launch scripts

## Build

```bash
bash scripts/build.sh
```

## One-Click Scripts

```bash
bash scripts/build.sh
bash scripts/run_planning_server.sh
bash scripts/run_moveit_demo.sh
bash scripts/run_moveit_mock_demo.sh
bash scripts/run_mock_perception.sh
```

Script summary:

- `scripts/build.sh`: build the whole workspace with the correct conda + ROS environment
- `scripts/run_planning_server.sh [backend]`: start the headless planning stack, including `move_group` and the planning service
- `scripts/run_mock_perception.sh`: robot + fake point cloud + target pose + path in RViz
- `scripts/run_moveit_demo.sh`: plain MoveIt demo for this robot
- `scripts/run_moveit_mock_demo.sh`: MoveIt + fake perception + path overlay in one RViz session

Examples:

```bash
bash scripts/run_planning_server.sh
bash scripts/run_planning_server.sh mock
bash scripts/run_planning_server.sh rviz:=true
bash scripts/run_moveit_demo.sh rviz:=false
```

If you already have a separate `move_group` process running, you can still run
only the planning node with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run medipick_planning_server planning_server.py --ros-args -p backend:=auto
```

## Required system packages

This workspace assumes ROS 2 Humble plus MoveIt 2 are installed.

## Next steps

If you want this to replace the current planner flow later, the next integration
steps are:

1. Add a planning scene bridge from the current point-cloud obstacle pipeline into MoveIt collision geometry.
2. Decide whether grasp pose requests should target `arm` only or `mobile_arm`.
3. Add a wrapper node that converts your current grasp pose input into `PlanToPose` plus planning-scene updates.
