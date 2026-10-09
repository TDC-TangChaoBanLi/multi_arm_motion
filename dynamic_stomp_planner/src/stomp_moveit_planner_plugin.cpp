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
 * @brief Planning Plugin implementation for STOMP
 */

#include <class_loader/class_loader.hpp>
#include <stomp_moveit/stomp_moveit_planning_context.hpp>
#include <moveit/utils/logger.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace stomp_moveit
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.stomp.planner_manager");
}
}  // namespace

using namespace planning_interface;

class StompPlannerManager : public PlannerManager
{
public:
  StompPlannerManager() = default;
  ~StompPlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    robot_model_ = model;
    node_ = node;
    parameter_namespace_ = parameter_namespace;
    param_listener_ = std::make_shared<stomp_moveit::ParamListener>(node, parameter_namespace);

    const auto prefix = parameter_namespace.empty() ? "" : parameter_namespace + ".";
    auto parameter = [&](const std::string& name, const auto& value) {
      if (!node->has_parameter(prefix + name)) node->declare_parameter(prefix + name, value);
      return node->get_parameter(prefix + name);
    };
    dynamic_ = parameter("dynamic_obstacles", false).as_bool();
    external_context_ = parameter("external_temporal_context",false).as_bool();
    start_delay_ = parameter("prediction_start_delay", 15.0).as_double();
    minimum_clearance_ = parameter("minimum_clearance", 0.0).as_double();
    if (!std::isfinite(minimum_clearance_) || minimum_clearance_ < 0.0) return false;
    clearance_buffer_ = parameter("clearance_buffer", 0.0005).as_double();
    if (!std::isfinite(clearance_buffer_) || clearance_buffer_ < 0.0) return false;
    required_sources_ = parameter("required_prediction_sources", std::vector<std::string>{}).as_string_array();
    const auto topic = parameter("prediction_topic", std::string("/motion_predictions")).as_string();
    if (dynamic_&&!external_context_)
    {
      if (!std::isfinite(start_delay_) || start_delay_ < 0) return false;
      prediction_sub_ = node->create_subscription<multi_arm_motion::Prediction>(topic, 10,
          [this](multi_arm_motion::Prediction::ConstSharedPtr msg) {
            std::string error;
            if (!predictions_.update(*msg, error))
              RCLCPP_WARN(node_->get_logger(), "Rejected motion prediction: %s", error.c_str());
          });
    }

    return true;
  }

  std::string getDescription() const override
  {
    return "STOMP";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs = { "STOMP" };
  }

  PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                        const MotionPlanRequest& req,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    if (!canServiceRequest(req))
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return nullptr;
    }

    const auto params = param_listener_->get_params();

    std::shared_ptr<StompPlanningContext> planning_context =
        std::make_shared<StompPlanningContext>("STOMP", req.group_name, params);
    planning_context->setPlanningScene(planning_scene);
    planning_context->setMotionPlanRequest(req);
    planning_context->setMinimumClearance(minimum_clearance_);
    planning_context->setClearanceBuffer(clearance_buffer_);
    if(external_context_)planning_context->requireTemporalContext();
    if (dynamic_&&!external_context_)
    {
      const auto predictions = predictions_.snapshot();
      const auto start = node_->now() + rclcpp::Duration::from_seconds(start_delay_);
      try
      {
        for (const auto& source : required_sources_)
          if (std::none_of(predictions.begin(), predictions.end(), [&](const auto& p) { return p.source == source; }))
            throw std::runtime_error("missing required prediction: " + source);
        for (const auto& prediction : predictions)
        {
          const auto& controlled=robot_model_->getJointModelGroup(req.group_name)->getVariableNames();
          if(std::all_of(prediction.trajectory.joint_names.begin(),prediction.trajectory.joint_names.end(),
              [&](const auto& name){return std::find(controlled.begin(),controlled.end(),name)!=controlled.end();}))continue;
          // Uncertainty tubes require a separate robust collision model. Reject
          // them rather than silently presenting point predictions as robust.
          if (prediction.position_uncertainty != 0) throw std::runtime_error("nonzero prediction uncertainty unsupported");
          multi_arm_motion::samplePrediction(prediction,
              start + rclcpp::Duration::from_seconds((params.num_timesteps - 1) * params.delta_t));
        }
        planning_context->setPredictionContext(predictions, start, params.delta_t);
        RCLCPP_INFO(node_->get_logger(), "Dynamic STOMP temporal origin %.9f, dt %.4f (planning only)",
                    start.seconds(), params.delta_t);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(node_->get_logger(), "Dynamic STOMP rejected request: %s", e.what());
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        return nullptr;
      }
    }

    // Only create path publisher if topic parameter has been set
    if (!params.path_marker_topic.empty())
    {
      auto path_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>(params.path_marker_topic,
                                                                                          rclcpp::SystemDefaultsQoS());
      planning_context->setPathPublisher(path_publisher);
    }

    return planning_context;
  }

  bool canServiceRequest(const MotionPlanRequest& req) const override
  {
    if (req.goal_constraints.empty())
    {
      RCLCPP_ERROR(getLogger(), "Invalid goal constraints");
      return false;
    }

    if (req.group_name.empty() || !robot_model_->hasJointModelGroup(req.group_name))
    {
      RCLCPP_ERROR(getLogger(), "Invalid joint group '%s'", req.group_name.c_str());
      return false;
    }
    return true;
  }

  void setPlannerConfigurations(const PlannerConfigurationMap& /*pcs*/) override
  {
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  std::shared_ptr<stomp_moveit::ParamListener> param_listener_;
  bool dynamic_{false};
  bool external_context_{false};
  double start_delay_{15.0};
  double minimum_clearance_{0.0};
  double clearance_buffer_{0.0005};
  std::vector<std::string> required_sources_;
  mutable multi_arm_motion::PredictionStore predictions_;
  rclcpp::Subscription<multi_arm_motion::Prediction>::SharedPtr prediction_sub_;
};

class DynamicStompPlannerManager : public StompPlannerManager
{
public:
  bool initialize(const moveit::core::RobotModelConstPtr& model,const rclcpp::Node::SharedPtr& node,
      const std::string& ns) override
  {
    const std::string key=(ns.empty()?"":ns+".")+"dynamic_obstacles";
    if(!node->has_parameter(key))node->declare_parameter(key,true);
    return StompPlannerManager::initialize(model,node,ns);
  }
  std::string getDescription() const override {return "Dynamic STOMP";}
};

}  // namespace stomp_moveit

CLASS_LOADER_REGISTER_CLASS(stomp_moveit::StompPlannerManager, planning_interface::PlannerManager)
CLASS_LOADER_REGISTER_CLASS(stomp_moveit::DynamicStompPlannerManager, planning_interface::PlannerManager)
