#include <moveit/local_planner/trajectory_operator_interface.hpp>
#include <multi_arm_motion_core/trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <map>
#include <mutex>
#include <set>
#include <cmath>

namespace multi_arm_motion
{
class IndependentProgress : public moveit::hybrid_planning::TrajectoryOperatorInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node,const moveit::core::RobotModelConstPtr& model,
                  const std::string& group) override
  {
    model_=model;group_=model_->getJointModelGroup(group);
    auto parameter=[&](const std::string& name,const auto& value){
      if(!node->has_parameter(name))node->declare_parameter(name,value);return node->get_parameter(name);};
    dt_=parameter("mpc.dt",0.05).as_double();horizon_=parameter("mpc.horizon",10).as_int();
    tolerance_=parameter("reference.joint_tolerance",0.01).as_double();
    velocity_tolerance_=parameter("reference.velocity_tolerance",0.02).as_double();
    preview_=parameter("reference.preview_horizon",20.0).as_double();
    return group_&&horizon_>=2&&std::isfinite(dt_)&&dt_>0&&std::isfinite(tolerance_)&&tolerance_>0&&
      std::isfinite(velocity_tolerance_)&&velocity_tolerance_>0&&std::isfinite(preview_)&&preview_>=dt_*horizon_;
  }
  bool reset() override {std::lock_guard<std::mutex> lock(mutex_);references_.clear();return true;}
  moveit_msgs::action::LocalPlanner::Feedback addTrajectorySegment(const robot_trajectory::RobotTrajectory& tr) override
  {
    std::lock_guard<std::mutex> lock(mutex_);moveit_msgs::action::LocalPlanner::Feedback feedback;
    if(tr.empty()||!tr.getGroup()) {feedback.feedback="invalid reference";return feedback;}
    const auto& controlled=group_->getVariableNames();
    for(const auto& name:tr.getGroup()->getVariableNames())
    {
      if(std::find(controlled.begin(),controlled.end(),name)==controlled.end())
      {feedback.feedback="reference outside aggregate group";return feedback;}
      for(const auto& entry:references_)
        if(entry.first!=tr.getGroupName())
        {
          const auto& other=entry.second.trajectory->getGroup()->getVariableNames();
          if(std::find(other.begin(),other.end(),name)!=other.end())
          {feedback.feedback="overlapping reference ownership";return feedback;}
        }
    }
    for(size_t i=1;i<tr.getWayPointCount();++i)
      if(!std::isfinite(tr.getWayPointDurationFromPrevious(i))||tr.getWayPointDurationFromPrevious(i)<=0)
      {feedback.feedback="reference has no valid time law";return feedback;}
    references_[tr.getGroupName()]={std::make_shared<robot_trajectory::RobotTrajectory>(tr),0};
    return feedback;
  }
  moveit_msgs::action::LocalPlanner::Feedback getLocalTrajectory(const moveit::core::RobotState& current,
      robot_trajectory::RobotTrajectory& local) override
  {
    std::lock_guard<std::mutex> lock(mutex_);moveit_msgs::action::LocalPlanner::Feedback feedback;local.clear();
    if(references_.empty()) {feedback.feedback="missing reference";return feedback;}
    for(auto& entry:references_)
    {
      auto& ref=entry.second;
      while(ref.index+1<ref.trajectory->getWayPointCount()&&
          close(current,ref.trajectory->getWayPoint(ref.index),ref.trajectory->getGroup()))++ref.index;
    }
    local.addSuffixWayPoint(current,0.0);
    for(int k=1;k<=std::max(horizon_,int(std::ceil(preview_/dt_)));++k)
    {
      auto target=current;
      for(const auto& entry:references_)
      {
        const auto& ref=entry.second;auto sample=std::make_shared<moveit::core::RobotState>(current);
        const double start=ref.trajectory->getWayPointDurationFromStart(ref.index);
        const double t=start+(k-1)*dt_;
        sampleTrajectory(*ref.trajectory,t,*sample);
        const auto* g=ref.trajectory->getGroup();std::vector<double> q;sample->copyJointGroupPositions(g,q);
        target.setJointGroupPositions(g,q);
      }
      target.update();local.addSuffixWayPoint(target,dt_);
    }
    return feedback;
  }
  double getTrajectoryProgress(const moveit::core::RobotState& current) override
  {
    std::lock_guard<std::mutex> lock(mutex_);if(references_.empty())return 0;
    for(const auto& entry:references_)
    {
      const auto& tr=*entry.second.trajectory;
      if(!close(current,tr.getLastWayPoint(),tr.getGroup()))return 0;
      for(const auto& name:tr.getGroup()->getVariableNames())
        if(std::abs(current.getVariableVelocity(name))>velocity_tolerance_)return 0;
    }
    return 1;
  }
private:
  bool close(const moveit::core::RobotState& a,const moveit::core::RobotState& b,
      const moveit::core::JointModelGroup* group) const
  {
    for(const auto& name:group->getVariableNames())
      if(std::abs(a.getVariablePosition(name)-b.getVariablePosition(name))>tolerance_)return false;
    return true;
  }
  struct Reference {robot_trajectory::RobotTrajectoryPtr trajectory;size_t index;};
  std::map<std::string,Reference> references_;std::mutex mutex_;
  moveit::core::RobotModelConstPtr model_;const moveit::core::JointModelGroup* group_{nullptr};
  int horizon_{10};double dt_{0.05},tolerance_{0.01},velocity_tolerance_{0.02},preview_{20};
};
}
PLUGINLIB_EXPORT_CLASS(multi_arm_motion::IndependentProgress,moveit::hybrid_planning::TrajectoryOperatorInterface)
