#include <moveit/global_planner/global_planner_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <mutex>

namespace multi_arm_motion
{
class StompGlobal : public moveit::hybrid_planning::GlobalPlannerInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node) override
  {
    node_=node;
    auto parameter=[&](const std::string& name,const auto& value){
      if(!node_->has_parameter(name))node_->declare_parameter(name,value);return node_->get_parameter(name);};
    const auto scene_topic=parameter("monitored_planning_scene_topic",std::string("/monitored_planning_scene")).as_string();
    const auto joints=parameter("joint_states_topic",std::string("/joint_states")).as_string();
    monitor_=std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_,"robot_description","global_scene");
    if(!monitor_->getPlanningScene())return false;
    monitor_->startSceneMonitor(scene_topic);monitor_->startStateMonitor(joints);
    loader_=std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
        "moveit_core","planning_interface::PlannerManager");
    const auto plugin=parameter("stomp_plugin",std::string("dynamic_stomp/DynamicStompPlanner")).as_string();
    if(plugin!="dynamic_stomp/DynamicStompPlanner"&&plugin!="space_sim_stomp/StompPlanner")
    {RCLCPP_ERROR(node_->get_logger(),"Only STOMP is permitted by this global component");return false;}
    manager_=loader_->createSharedInstance(plugin);
    return manager_->initialize(monitor_->getRobotModel(),node_,"dynamic_stomp");
  }
  bool reset() noexcept override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(active_)active_->terminate();return true;
  }
  moveit_msgs::msg::MotionPlanResponse plan(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> handle) override
  {
    moveit_msgs::msg::MotionPlanResponse response;
    response.error_code.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    const auto& sequence=handle->get_goal()->motion_sequence;
    // Do not silently drop later requests, start states or path constraints.
    if(sequence.items.size()!=1 || sequence.items[0].blend_radius!=0)
    {RCLCPP_ERROR(node_->get_logger(),"This global plugin accepts exactly one unblended motion request");return response;}
    if(!monitor_->getStateMonitor()->haveCompleteState(rclcpp::Duration::from_seconds(0.2)))
    {response.error_code.val=moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE;return response;}
    planning_scene::PlanningScenePtr scene;
    {planning_scene_monitor::LockedPlanningSceneRO locked(monitor_);scene=locked->diff();scene->decoupleParent();}
    const auto& request=sequence.items[0].req;
    auto context=manager_->getPlanningContext(scene,request,response.error_code);
    if(!context)return response;
    {std::lock_guard<std::mutex> lock(mutex_);active_=context;}
    planning_interface::MotionPlanResponse result;
    context->solve(result);
    {std::lock_guard<std::mutex> lock(mutex_);active_.reset();}
    response.error_code=result.error_code;response.planning_time=result.planning_time;response.group_name=request.group_name;
    if(result.error_code.val!=moveit_msgs::msg::MoveItErrorCodes::SUCCESS||!result.trajectory)return response;
    // The dynamic planner produces a fixed temporal grid. Reject invalid speed
    // rather than silently retiming it against different future obstacle poses.
    const auto* group=scene->getRobotModel()->getJointModelGroup(request.group_name);
    for(size_t i=1;i<result.trajectory->getWayPointCount();++i)
    {
      const double dt=result.trajectory->getWayPointDurationFromPrevious(i);
      if(dt<=0){response.error_code.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;return response;}
      for(const auto& name:group->getVariableNames())
      {
        const auto& bounds=scene->getRobotModel()->getVariableBounds(name);
        const double speed=std::abs(result.trajectory->getWayPoint(i).getVariablePosition(name)-
            result.trajectory->getWayPoint(i-1).getVariablePosition(name))/dt;
        if(bounds.velocity_bounded_&&speed>bounds.max_velocity_*request.max_velocity_scaling_factor)
        {response.error_code.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;return response;}
      }
    }
    moveit::core::robotStateToRobotStateMsg(result.trajectory->getFirstWayPoint(),response.trajectory_start);
    result.trajectory->getRobotTrajectoryMsg(response.trajectory);return response;
  }
private:
  rclcpp::Node::SharedPtr node_;planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> loader_;
  planning_interface::PlannerManagerPtr manager_;planning_interface::PlanningContextPtr active_;std::mutex mutex_;
};
}
PLUGINLIB_EXPORT_CLASS(multi_arm_motion::StompGlobal,moveit::hybrid_planning::GlobalPlannerInterface)
