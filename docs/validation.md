# Verification record — 2026-10-08 to 2026-10-09

Environment: WSL, ROS 2 Jazzy, MoveIt 2.12.4. The missing Hybrid Planning
dependency was built from upstream tag 2.12.4, commit
`1ade0e9dcf50dbbbc3a984b995c786cf12736235`, using the supplied bootstrap script.
No changes were committed automatically.

## Build and unit tests

All six packages build. The four implementation packages pass 38 GoogleTest
cases (15 core, 7 STOMP, 14 MPC and 2 plugin tests), with no errors, failures
or skipped cases. Suite wrappers are not counted as additional test cases.

Coverage includes prediction expiry/versioning, interpolation and continuous
joint branch cuts, future peer collisions, ACM and attachments, nearest-point
gradients, conservative distance bounds, partial orientation constraints,
STOMP update/endpoints, MPC acceleration limits and validated stopping tails.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --base-paths lib/multi_arm_motion
colcon test --packages-select multi_arm_motion_core dynamic_stomp_planner multi_arm_mpc multi_arm_hybrid_planning
colcon test-result --verbose
```

## Concurrent software fault test

`examples/concurrent_smoke.py` uses independent prismatic groups and a cubic
trajectory controller mock, including the JTC default immediate replacement and
feedback-to-future-first-point interpolation. The latest fixed-seed run passed
all ten original checks in the original replacement mode. The extended eleven
checks passed with continuous
desired-state interpolation, 40 ms actuator lag, 0.0002 joint tolerance and
0.0005 stopped-velocity tolerance (`/tmp/multi_arm_concurrent_6s9x1s02`), and
again with optional independent progress (`/tmp/multi_arm_concurrent_5ktbc_kx`):

1. Concurrent independent goals and overlapping ownership rejection.
2. Supplied reference and disconnected-start rejection.
3. Supplied seed with temporal STOMP optimization.
4. Safe world-geometry update while a goal executes.
5. Cancel one goal while the other finishes.
6. Unreachable goal reports failure.
7. Stop latch and explicit resume.
8. Goal deadline reports `TIMED_OUT`.
9. Stale feedback aborts while retaining the checked braking tail.
10. Killing the producer leaves the controller executing its final braking tail.
11. A timed reference with identical start/end executes its intermediate motion;
    completion cannot precede its committed end time, even with command lead.

```bash
ROS_DOMAIN_ID=94 ROS_STATIC_PEERS= ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
  python3 lib/multi_arm_motion/examples/concurrent_smoke.py
```

Add `MPC_CONTINUOUS_DESIRED_TEST=1` for the lagged continuous-controller variant.

## Articulated dual-arm MuJoCo tests

The consumer's actual dual UR model, joint trajectory controllers, MoveIt scene
and MPC runtime completed both forward and reverse concurrent tests. Each test
commands independent opposite-sign 0.04 rad wrist changes. These are execution
integration tests, not six-mirror assembly tests.

| Measurement | Forward | Reverse |
| --- | ---: | ---: |
| Successful action results | 2/2 | 2/2 |
| Maximum final joint error (rad) | 0.0092274 | 0.0095227 |
| Joint-state samples with both arms moving | 1510 | 1552 |
| Matching controller command start stamps | 40 | 40 |
| Measured-state collision checks passed | 41/41 | 41/41 |

The configured execution tolerance was 0.01 rad. Collision checks use the
MoveIt scene's configured ACM; they are sampled checks, not a proof of continuous
physical clearance or an exhaustive MuJoCo contact audit.

Use an unused ROS domain in **every terminal**, after sourcing the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=95
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_STATIC_PEERS=
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

Start the consumer's simulator/controllers, scene provider and runtime in three
separate terminals with those same environment variables:

```bash
ros2 launch launch_controller start_traditional_controllers.launch.py \
  env:=space_sim_mujoco initial_ur_controller:=joint_trajectory_controller \
  mujoco_headless:=true launch_rviz:=false controller_spawner_timeout:=60
```

```bash
ros2 launch space_sim_moveit_config demo.launch.py launch_rviz:=false
```

```bash
ros2 launch space_sim_moveit_config hybrid_motion.launch.py enable_execution:=true
```

Once both controllers and the scene are available, run in a fourth terminal:

```bash
python3 src/control/task_control/space_sim/test/test_hybrid_motion.py \
  --output /tmp/multi_arm_forward.json
python3 src/control/task_control/space_sim/test/test_hybrid_motion.py \
  --offset -0.04 --output /tmp/multi_arm_reverse.json
```

The test records joint positions/velocities, controller timestamps, action
results and measured-state checks. Workspace artifacts for the recorded runs are
in `tmp/multi_arm_motion_validation_20261008/` in the consumer repository.

## Defects found and corrected during verification

- Continuous joint normalization changed the controller coordinate branch:
  preserve unwrapped coordinates through optimization and interpolation.
- The inherited STOMP filter treated update deltas as absolute positions and
  moved endpoints: smooth deltas, bound resulting positions and fix endpoints.
- Repeated full mesh checks missed command deadlines: compute closest-point
  gradients once and use a conservative geometric bound for validation reuse.
- Unchanged scene/transform heartbeats invalidated commands: compare geometry
  and transform values rather than message timestamps alone.
- A budget-exhausted warm start reused stale velocity: recenter on feedback and
  the current reference each cycle; a regression test verifies convergence.
- Long global validation blocked execution: validate outside the execution lock
  and recheck scene/prediction versions when committing.
- Delayed action result requests expired an already completed peer result:
  request both result futures immediately in the integration client.
- Preserve the original request constraint metadata in the STOMP cost so the
  normal-axis constraint name reaches optimization as well as final validation.

## Scope

The concurrent software smoke test now includes an eleven-check suite. In
addition to the existing concurrency and fault checks, a timed reference with
identical start and end configurations must execute its intermediate motion
before reporting success. The suite passed with continuous desired commands,
and separately with optional independent reference progress enabled. These are
software tests, not evidence that adaptive progress has passed physical assembly.

The new runtime is enabled explicitly. Satellite assembly actions can select
`motion_backend:=hybrid`; their contact references and free-space seeds then use
this runtime. A retained-state continuation has physically assembled all six
mirrors and passed independent final pose/attachment checks. A subsequent fresh
initialization run (43) also completed all six and both arms' return home, with
measured concurrency and sampled physics contacts. Fresh alternate-order runs
45 (S1 then S0), 46 (S2 then S3) and 47 (S2 then S3 without later sequence gates)
also passed final pose/attachment checks and returned both arms home. A physical
payload-release injection aborted the task and stopped both arms. Detailed runs
are tracked in the consumer's `docs/assembly_hybrid_validation_20261008.md`.
After tightening STOMP collision sampling, run 50 again completed the baseline
six-mirror flow from initialization with both arms returning home: 635 sampled
valid states, 3858 samples with both arms moving, and 6.18 micrometre maximum
final position error. Run 48 failed and run 49 was a retained-state diagnostic
continuation; neither is counted as a fresh complete pass.
Run 51 passed the S1-then-S0 flow with later sequence gates removed, including
both returns home: 897 sampled valid states, 2124 samples with both arms moving,
6.182 micrometre maximum final position error, and 8910 sampled contact snapshots
consistent with the consumer's interface timing/adjacency checks. The largest
sampled mirror-normal world-z magnitude during storage exits was 0.06965 against
the 0.1 limit. These observations are not a continuous collision-free proof.
No hardware or floating/planar-joint
deployment has been validated. Forecasts model robot joints, not moving world
objects or uncertainty tubes. Sampling, tracking tolerances and deadline checks
are documented in [architecture.md](architecture.md).
