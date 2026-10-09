#pragma once
#include <multi_arm_motion_interfaces/msg/motion_prediction.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace multi_arm_motion
{
using Prediction = multi_arm_motion_interfaces::msg::MotionPrediction;
// Invalid/expired predictions are errors, never silently treated as static obstacles.
void validatePrediction(const Prediction& p);
std::vector<double> samplePrediction(const Prediction& p, const rclcpp::Time& at);
class PredictionStore
{
public:
  bool update(const Prediction& prediction, std::string& error);
  std::vector<Prediction> snapshot() const;
  uint64_t revision() const {std::lock_guard<std::mutex> lock(mutex_);return revision_;}
  void clear() {std::lock_guard<std::mutex> lock(mutex_);predictions_.clear();++revision_;}
private:
  mutable std::mutex mutex_;
  std::map<std::string, Prediction> predictions_;
  uint64_t revision_{0};
};
void applyPredictions(moveit::core::RobotState& state, const std::vector<Prediction>& predictions,
                      const std::vector<std::string>& controlled, const rclcpp::Time& at);
}  // namespace multi_arm_motion
