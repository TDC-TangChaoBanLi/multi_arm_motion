#!/usr/bin/env bash
# Build the exact Hybrid Planning version used by this workspace without sudo.
set -euo pipefail
motion_workspace="${1:?Usage: build_hybrid_dependency.sh WORKSPACE_ROOT [SOURCE_DIRECTORY]}"
motion_dependency_source="${2:-/tmp/multi_arm_motion_moveit2}"
if [ ! -d "$motion_dependency_source/.git" ]; then
  git clone --depth 1 --branch 2.12.4 --filter=blob:none --sparse \
    https://github.com/moveit/moveit2.git "$motion_dependency_source"
fi
motion_revision="$(git -C "$motion_dependency_source" rev-parse HEAD)"
if [ "$motion_revision" != "1ade0e9dcf50dbbbc3a984b995c786cf12736235" ]; then
  echo "Unexpected MoveIt revision: $motion_revision" >&2
  exit 1
fi
git -C "$motion_dependency_source" sparse-checkout set moveit_ros/hybrid_planning
source /opt/ros/jazzy/setup.bash
cd "$motion_workspace"
colcon build --base-paths "$motion_dependency_source/moveit_ros/hybrid_planning" \
  --packages-select moveit_hybrid_planning --cmake-args -DBUILD_TESTING=OFF
