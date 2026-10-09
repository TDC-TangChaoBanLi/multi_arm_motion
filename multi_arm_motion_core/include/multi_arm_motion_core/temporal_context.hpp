#pragma once
#include <multi_arm_motion_core/prediction.hpp>
namespace multi_arm_motion
{
// Optional planner extension: carries the clock and forecast snapshot alongside
// MotionPlanResponse, which has no field for either of these.
class TemporalContext
{
public:
  virtual ~TemporalContext() = default;
  virtual void setTemporalContext(std::vector<Prediction>,const rclcpp::Time&) = 0;
  virtual void setTemporalStep(double) = 0;
  virtual rclcpp::Time temporalOrigin() const = 0;
  virtual std::vector<Prediction> temporalPredictions() const = 0;
};
}
