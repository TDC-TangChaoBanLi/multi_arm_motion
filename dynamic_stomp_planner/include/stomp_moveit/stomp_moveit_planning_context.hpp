/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** @file
 * @author Henning Kayser
 * @brief Planning Context implementation for STOMP
 **/

#pragma once

#include <moveit/planning_interface/planning_interface.hpp>
#include <multi_arm_motion_core/prediction.hpp>
#include <multi_arm_motion_core/temporal_context.hpp>
#include <mutex>
#include <atomic>

#include <dynamic_stomp_planner/stomp_moveit_parameters.hpp>

// Forward declaration
namespace stomp
{
class Stomp;
}

namespace stomp_moveit
{
class StompPlanningContext : public planning_interface::PlanningContext, public multi_arm_motion::TemporalContext
{
public:
  StompPlanningContext(const std::string& name, const std::string& group_name, const stomp_moveit::Params& params);

  void solve(planning_interface::MotionPlanResponse& res) override;

  void solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;

  void clear() override;

  void setPathPublisher(const std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>& path_publisher);
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> getPathPublisher();

  void setPredictionContext(std::vector<multi_arm_motion::Prediction> predictions,
                            const rclcpp::Time& start, double step)
  {
    if(!std::isfinite(step)||step<=0)throw std::invalid_argument("invalid temporal grid");
    for(const auto& prediction:predictions)multi_arm_motion::validatePrediction(prediction);
    predictions_ = std::move(predictions); prediction_start_ = start; prediction_step_ = step;
  }
  const auto& predictions() const { return predictions_; }
  const auto& predictionStart() const { return prediction_start_; }
  double predictionStep() const { return prediction_step_; }
  double minimumClearance() const { return minimum_clearance_; }
  void setMinimumClearance(double value) { minimum_clearance_ = value; }
  double clearanceBuffer() const { return clearance_buffer_; }
  void setClearanceBuffer(double value) { clearance_buffer_ = value; }
  void setTemporalContext(std::vector<multi_arm_motion::Prediction> predictions,const rclcpp::Time& start) override
  { setPredictionContext(std::move(predictions),start,params_.delta_t); }
  void setTemporalStep(double step) override
  {
    if(!std::isfinite(step)||step<=0)throw std::invalid_argument("invalid temporal step");
    prediction_step_=step;
  }
  rclcpp::Time temporalOrigin() const override { return prediction_start_; }
  std::vector<multi_arm_motion::Prediction> temporalPredictions() const override { return predictions_; }
  void requireTemporalContext() { temporal_required_=true; }

private:
  const stomp_moveit::Params params_;
  std::vector<multi_arm_motion::Prediction> predictions_;
  rclcpp::Time prediction_start_{0, 0, RCL_ROS_TIME};
  double prediction_step_{0.0};
  double minimum_clearance_{0.0};
  double clearance_buffer_{0.0005};
  bool temporal_required_{false};
  std::shared_ptr<stomp::Stomp> stomp_;
  std::mutex stomp_mutex_;
  std::atomic<bool> canceled_{false};
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> path_publisher_;
};
}  // namespace stomp_moveit
