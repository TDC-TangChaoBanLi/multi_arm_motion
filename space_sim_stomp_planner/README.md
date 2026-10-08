# space_sim STOMP plugin

Based on MoveIt 2.12.4, commit `1ade0e9dcf50dbbbc3a984b995c786cf12736235`,
`moveit_planners/stomp`. Upstream BSD license notices are retained in each source.

Changes:

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
