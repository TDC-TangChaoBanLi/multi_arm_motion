# Dynamic STOMP plugin

`dynamic_stomp/DynamicStompPlanner` adds time-sampled future peer-motion costs.
The legacy `space_sim_stomp/StompPlanner` entry is retained with dynamic mode
disabled by default. See [integration contracts and limitations](../docs/architecture.md).
The generic geometric extension is named `link_normal_horizontal nx ny nz tolerance`;
the original constraint spelling below remains supported for compatibility.

Based on MoveIt 2.12.4, commit `1ade0e9dcf50dbbbc3a984b995c786cf12736235`,
`moveit_planners/stomp`. Upstream BSD license notices are retained in each source.

Changes:

- Future peer trajectories are sampled in absolute ROS time, including stationary
  segments of the planned arm. The coordinator supplies temporal context directly;
  standalone planner users use the prediction topic and required-source settings.
- `minimum_clearance` (metres, default zero) rejects trajectories that are
  collision-free but too close to other geometry. Set it to the executor's
  clearance so the global path and local controller enforce the same margin.
  Supplied seeds with small positive clearance deficits receive bounded distance
  gradient corrections at interior waypoints before optimization, for at most
  10% of the planning budget (capped at 1 s). Endpoints and the temporal grid are
  preserved. This seed is never executed without full STOMP and consumer checks.
  Static scenes without peer predictions use joint displacement for collision
  subdivision (0.005 in joint-coordinate norm); peer forecasts additionally
  require 10 ms temporal subdivision. This finite sampling cannot certify all
  geometry between samples; dense consumer validation remains required.
  Intended contact pairs must be authorized explicitly in the scene ACM.
- `clearance_buffer` (metres, default 0.0005) adds optimization reserve above a
  positive `minimum_clearance`. Dense consumer validation still enforces the
  original minimum; the buffer reduces boundary failures between sampled points.
- Update filtering smooths increments, enforces bounds on the resulting absolute
  coordinates, and fixes both endpoints. Continuous coordinates are kept unwrapped.
  Temporal requests additionally penalize velocity violations and project updates
  onto per-segment velocity bounds while retaining the fixed time grid.
  Once an iterate satisfies geometry, task and velocity checks, update filtering
  backtracks an infeasible proposal, retaining the feasible iterate if necessary.
  This does not replace the consumer's independent dense trajectory validation.
- Complete joint goals use their requested positions directly. `random_seed`
  controls rollout reproducibility; `-1` retains the thread RNG state.

- Satisfied MoveIt constraints produce zero violation cost. Upstream used the
  nonzero distance even for satisfied orientation constraints, preventing free rotations.
- Collision and constraint callbacks own their RobotState per planning request.
  Static states in upstream callbacks were shared between arms and retained old attachments.
- Path constraints named `space_sim_mirror_vertical nx ny nz tolerance` use the
  first orientation constraint's link, and constrain `abs((R_link * normal_in_link).z())`.
  The orientation message itself has all angular tolerances open. Both horizontal
  normal azimuth and rotation around the mirror normal are free.

STOMP still uses randomized rollouts. No OMPL/RRT fallback is included.
This constraint does not prescribe the relative progress of translation and rotation.
