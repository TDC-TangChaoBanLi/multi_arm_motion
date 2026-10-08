# multi_arm_motion

Reusable MoveIt planning and execution components for multi-arm robots.

## Current baseline

`space_sim_stomp_planner` contains the existing MoveIt 2.12.4 STOMP plugin,
migrated from `Dual_UR_Demo/src/control/task_control/space_sim_stomp_planner`.
The ROS package name, plugin identifiers and parameters are preserved so that
existing consumers continue to work. See the package README for its current
changes relative to upstream and its remaining task-specific constraint.

Dynamic obstacle prediction, MPC execution and Hybrid Planning integration are
not implemented in this baseline.

Satellite assembly actions, docking and simulation task state remain in the
consumer project's `space_sim` package.

## Build and test

Place this repository in a ROS 2 Jazzy workspace, install its package dependencies,
and run from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to space_sim_stomp_planner
colcon test --packages-select space_sim_stomp_planner
colcon test-result --verbose
```

When moving an already-built package, add `--cmake-clean-cache` to the first build
to replace CMake's cached source directory.

## Licensing

The existing planner declares BSD-3-Clause licensing. Upstream copyright and
license notices are retained in the source files.
