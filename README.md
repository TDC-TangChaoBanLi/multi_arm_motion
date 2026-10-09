# multi_arm_motion

Reusable MoveIt planning and execution components for multi-arm robots.

## Packages

| Package | Responsibility |
| --- | --- |
| `multi_arm_motion_interfaces` | Concurrent ExecuteMotion action and versioned motion predictions |
| `multi_arm_motion_core` | Prediction validation/interpolation, in-process collision distances |
| `dynamic_stomp_planner` | MoveIt STOMP plugin with optional future-motion collision costs |
| `multi_arm_mpc` | Receding horizon solver and Hybrid Planning local solver plugin |
| `multi_arm_hybrid_planning` | Concurrent goal coordinator, Hybrid plugins, checked command routing and launch |
| `space_sim_stomp_planner` | Compatibility dependency; the original plugin ID remains available |

The baseline was migrated from
`Dual_UR_Demo/src/control/task_control/space_sim_stomp_planner` (MoveIt 2.12.4).
Existing consumers may keep using `space_sim_stomp/StompPlanner`; future-motion
costs are disabled for this compatibility entry by default.

Satellite assembly actions, docking and simulation task state remain in the
consumer project's `space_sim` package.

## Build and test

This implementation targets ROS 2 Jazzy / MoveIt 2.12.4. Install
`ros-jazzy-moveit-hybrid-planning` with the other ROS dependencies. If that package
is unavailable, `scripts/build_hybrid_dependency.sh WORKSPACE_ROOT` builds the
exact upstream 2.12.4 component in the workspace without administrator access.

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash # if Hybrid Planning was built in this workspace
colcon build --symlink-install --packages-up-to multi_arm_hybrid_planning
source install/setup.bash
colcon test --packages-select multi_arm_motion_core dynamic_stomp_planner multi_arm_mpc multi_arm_hybrid_planning
colcon test-result --verbose
```

## Status and integration

The default runtime accepts independent goals on disjoint groups and drives one
aggregate MPC loop. Every controller update includes a checked braking tail.
See [architecture, API and limits](docs/architecture.md) for deployment details.
See [verification results and reproduction](docs/validation.md) for the unit,
fault-injection and articulated dual-arm MuJoCo runs.
Execution is disabled by default. Satellite actions retain their existing backend.

Software-only smoke test, using an unused ROS domain:

```bash
ROS_DOMAIN_ID=94 ROS_STATIC_PEERS= ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
  python3 lib/multi_arm_motion/examples/concurrent_smoke.py
```

The repository path in this example assumes it is checked out as the
`lib/multi_arm_motion` submodule. The mock driver tests message integration only.

Launch the generic concurrent runtime:

```bash
ros2 launch multi_arm_hybrid_planning hybrid.launch.py robot_config:=/absolute/robot.yaml
```

Supply descriptions, an aggregate `group_name`, joint limits/kinematics and
`mpc.enable_execution: true` in that ROS parameter file. Start `command_router`
with disjoint controller joint lists and topics. Submit
`multi_arm_motion_interfaces/action/ExecuteMotion` goals to `/execute_motion`.
Use `/stop_motion`, `/resume_motion` and `/reset_motion_predictions` for execution
lifecycle control. `legacy_runtime:=true` selects the upstream single-goal manager.

## Licensing

The existing planner declares BSD-3-Clause licensing. Upstream copyright and
license notices are retained in the source files.
