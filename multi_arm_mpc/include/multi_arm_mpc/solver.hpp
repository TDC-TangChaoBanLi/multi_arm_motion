#pragma once
#include <Eigen/Core>
#include <functional>
#include <string>
#include <vector>
namespace multi_arm_motion
{
struct MpcConfig
{
  int horizon{10}, iterations{15}, stop_step{2};
  double dt{0.05}, budget_s{0.035}, clearance{0.025}, influence_distance{0.08};
  double tracking_weight{10}, velocity_weight{0.05}, acceleration_weight{0.1}, obstacle_weight{1000};
  double task_weight{100000};
  double terminal_weight{100000};
  double finite_difference{1e-4}, validation_joint_step{0.01}, validation_time_step{0.01};
  void validate() const;
};
struct MpcResult
{
  double optimization_seconds{0}, validation_seconds{0};
  bool feasible{false}, deadline_reached{false}, braking_only{false};
  std::string reason;
  std::vector<int> paused_joints;
  Eigen::MatrixXd positions, velocities;
  double min_distance{0};
};
// Distance oracle uses q and future time in seconds. Negative/zero means collision.
using DistanceOracle = std::function<double(const Eigen::VectorXd&, double)>;
using StatePredicate = std::function<bool(const Eigen::VectorXd&, double)>;
using DistanceGradientOracle=std::function<std::pair<double,Eigen::VectorXd>(const Eigen::VectorXd&,double)>;
using TaskCostOracle=std::function<std::pair<double,Eigen::VectorXd>(const Eigen::VectorXd&,double,bool)>;
using StopPredicate=std::function<bool(const Eigen::VectorXd&,const Eigen::VectorXd&,double)>;
class MpcSolver
{
public:
  explicit MpcSolver(MpcConfig config = {});
  MpcResult solve(const Eigen::VectorXd& q, const Eigen::VectorXd& velocity,
      const Eigen::MatrixXd& reference, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
      const Eigen::VectorXd& vmax, const Eigen::VectorXd& amax,
      const DistanceOracle& distance, const StatePredicate& allowed = {},const DistanceGradientOracle& distance_gradient = {},
      const DistanceOracle& validation_distance = {},const TaskCostOracle& task_cost = {},const StopPredicate& can_stop = {},
      const std::vector<std::vector<int>>& independent_groups = {});
  void reset() { warm_.resize(0,0); }
private:
  MpcConfig config_;
  Eigen::MatrixXd warm_;
};
}
