#pragma once
#include <multi_arm_mpc/solver.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace multi_arm_motion
{
// Synchronized constant acceleration stop. Each joint uses at most amax;
// all joints stop at T=max(|v|/amax). No instantaneous zero-velocity command.
// The complete braking sweep must pass bounds, clearance and task constraints.
trajectory_msgs::msg::JointTrajectory brakingTrajectory(
    const std::vector<std::string>& names, const Eigen::VectorXd& q,
    const Eigen::VectorXd& v, const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper, const Eigen::VectorXd& amax,
    double sample_dt, double clearance, const DistanceOracle& distance,
    const StatePredicate& allowed = {});
}
