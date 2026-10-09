#pragma once
#include <stomp_moveit/cost_functions.hpp>
#include <multi_arm_motion_core/prediction.hpp>
#include <multi_arm_motion_core/distance.hpp>

namespace stomp_moveit::costs
{
inline CostFn dynamicCollisionCost(const planning_scene::PlanningSceneConstPtr& scene,
    const moveit::core::JointModelGroup* group,
    const std::vector<multi_arm_motion::Prediction>& predictions,
    const rclcpp::Time& start, double dt, double clearance = 0.0)
{
  if (!std::isfinite(clearance) || clearance < 0.0)
    throw std::invalid_argument("invalid STOMP minimum clearance");
  auto cache = std::make_shared<multi_arm_motion::ShapeBoundsCache>();
  return [=, state = scene->getCurrentState(), bounds = std::shared_ptr<multi_arm_motion::DistanceBound>{}](const Eigen::MatrixXd& values,
      Eigen::VectorXd& cost, bool& valid) mutable {
    cost.setZero(values.cols()); valid = true;
    const auto& names = group->getVariableNames();
    for (Eigen::Index i = 0; i < values.cols(); ++i)
    {
      const Eigen::Index next = std::min(i + 1, values.cols() - 1);
      const double gap = (values.col(next) - values.col(i)).norm();
      // With no peer forecast the scene is static: joint displacement sets
      // geometric resolution. A slow time law must not duplicate thousands of
      // identical static checks. Moving peers still require temporal sampling,
      // including when this arm remains stationary.
      // Thin payload edges can cross the requested clearance between samples
      // spaced 0.025 rad/m apart. Keep static sampling displacement based, but
      // tighten its resolution; moving-peer sampling matches the consumer's
      // 10 ms commit sweep. Dense consumer validation remains mandatory.
      const int temporal_checks = predictions.empty() ? 0 : int(std::ceil(dt / 0.01));
      const int count = next == i ? 1 : std::max(2, std::max(int(std::ceil(gap / 0.005)), temporal_checks));
      for (int k = 0; k < count; ++k)
      {
        const double fraction = double(k) / count;
        const Eigen::VectorXd q = values.col(i) * (1 - fraction) + values.col(next) * fraction;
        try
        {
          multi_arm_motion::applyPredictions(state, predictions, names,
              start + rclcpp::Duration::from_seconds((double(i) + fraction) * dt));
          state.setJointGroupPositions(group, q); state.update();
          if (clearance > 0.0)
          {
            if (!bounds) bounds = std::make_shared<multi_arm_motion::DistanceBound>(
                *scene, state, group->getName(), clearance, cache.get());
            const double distance = bounds->distance(state);
            if (std::isnan(distance) || distance < clearance)
            {
              cost[i] += std::isfinite(distance) ? 1.0 + (clearance-distance)/clearance : 1.0;
              valid = false;
            }
          }
          else if (scene->isStateColliding(state, group->getName())) { cost[i] += 1; valid = false; }
        }
        catch (const std::exception&) { cost[i] += 1; valid = false; }
      }
    }
    return true;
  };
}
}
