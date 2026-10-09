#include <multi_arm_mpc/solver.hpp>
#include <multi_arm_mpc/braking.hpp>
#include <multi_arm_motion_core/distance.hpp>
#include <multi_arm_motion_core/constraints.hpp>
#include <multi_arm_motion_core/prediction.hpp>
#include <multi_arm_motion_core/trajectory.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.hpp>
#include <moveit/local_planner/feedback_types.hpp>
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace multi_arm_motion
{
class DistanceMpc : public moveit::hybrid_planning::LocalConstraintSolverInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node,
      const planning_scene_monitor::PlanningSceneMonitorPtr& monitor, const std::string& group) override
  {
    node_=node;monitor_=monitor;group_=group;
    monitor_->getStateMonitor()->enableCopyDynamics(true);
    auto parameter=[&](const std::string& name,const auto& value) {
      if(!node_->has_parameter(name)) node_->declare_parameter(name,value);
      return node_->get_parameter(name);
    };
    config_.horizon=parameter("mpc.horizon",10).as_int();
    config_.dt=parameter("mpc.dt",0.05).as_double();
    config_.budget_s=parameter("mpc.optimization_budget",0.035).as_double();
    config_.clearance=parameter("mpc.clearance",0.025).as_double();
    config_.influence_distance=parameter("mpc.influence_distance",0.08).as_double();
    config_.iterations=parameter("mpc.iterations",15).as_int();
    config_.validation_joint_step=parameter("mpc.validation_joint_step",.01).as_double();
    config_.validation_time_step=parameter("mpc.validation_time_step",.01).as_double();
    enabled_=parameter("mpc.enable_execution",false).as_bool();
    age_=parameter("mpc.max_state_age",0.2).as_double();
    acceleration_=parameter("mpc.default_acceleration",0.5).as_double();
    source_=parameter("mpc.prediction_source",std::string("local_mpc")).as_string();
    const auto output=parameter("mpc.command_topic",std::string("mpc_joint_commands")).as_string();
    const auto topic=parameter("mpc.prediction_topic",std::string("/motion_predictions")).as_string();
    group_model_=monitor_->getRobotModel()->getJointModelGroup(group_);
    if(!group_model_ || group_model_->getVariableCount()!=group_model_->getActiveJointModels().size() ||
       !std::isfinite(age_) || age_<=0 || !std::isfinite(acceleration_) || acceleration_<=0) return false;
    try {solver_=std::make_unique<MpcSolver>(config_);} catch(const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(),"MPC configuration: %s",e.what());return false;}
    names_=group_model_->getVariableNames();
    const int n=names_.size();lower_.resize(n);upper_.resize(n);vmax_.resize(n);amax_.resize(n);
    for(int j=0;j<n;++j)
    {
      const auto& b=monitor_->getRobotModel()->getVariableBounds(names_[j]);
      if(!b.velocity_bounded_) return false;
      const double guard=parameter("mpc.continuous_coordinate_guard",10000.0).as_double();
      if(!std::isfinite(guard)||guard<=0)return false;
      lower_[j]=b.position_bounded_?b.min_position_:-guard;
      upper_[j]=b.position_bounded_?b.max_position_:guard;
      vmax_[j]=std::min(std::abs(b.min_velocity_),std::abs(b.max_velocity_));
      amax_[j]=b.acceleration_bounded_ ? std::min(std::abs(b.min_acceleration_),std::abs(b.max_acceleration_)) : acceleration_;
    }
    command_pub_=node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(output,1);
    prediction_pub_=node_->create_publisher<Prediction>(topic,10);
    prediction_sub_=node_->create_subscription<Prediction>(topic,10,[this](Prediction::ConstSharedPtr p){
      if(p->source==source_)return;std::string error;
      if(!predictions_.update(*p,error)) RCLCPP_WARN(node_->get_logger(),"Prediction rejected: %s",error.c_str());
    });
    stop_service_=node_->create_service<std_srvs::srv::Trigger>("mpc_stop",[this](
        std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response){
      stopped_=true;response->success=reset();response->message=response->success?
          "Execution latched stopped; validated braking requested":"Stop latched; new braking unavailable, previous controller tail retained";
    });
    return true;
  }
  bool reset() override
  {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    if(solver_)solver_->reset();
    if(enabled_&&monitor_&&command_pub_)
    {
      moveit::core::RobotState state(monitor_->getRobotModel());
      {planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);state=scene->getCurrentState();}
      return publishBrake(state);
    }
    return true;
  }
  moveit_msgs::action::LocalPlanner::Feedback solve(const robot_trajectory::RobotTrajectory& reference,
      const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> goal,
      trajectory_msgs::msg::JointTrajectory& solution) override
  {
    std::lock_guard<std::mutex> lock(solver_mutex_);
    const auto started=std::chrono::steady_clock::now();
    moveit_msgs::action::LocalPlanner::Feedback feedback;solution=trajectory_msgs::msg::JointTrajectory();
    if(!enabled_ || stopped_) {feedback.feedback="MPC execution disabled or stopped";return feedback;}
    planning_scene::PlanningScenePtr scene;
    {planning_scene_monitor::LockedPlanningSceneRO locked(monitor_);scene=locked->diff();scene->decoupleParent();}
    auto state=scene->getCurrentState();const auto now=node_->now();
    try
    {
      if(!monitor_->getStateMonitor()->haveCompleteState(rclcpp::Duration::from_seconds(age_)))
        throw std::runtime_error("joint state incomplete or stale");
      if(reference.empty() || reference.getGroupName()!=group_) throw std::runtime_error("empty/wrong group reference");
      std::vector<double> qv,vv;state.copyJointGroupPositions(group_model_,qv);
      state.copyJointGroupVelocities(group_model_,vv);
      const Eigen::VectorXd q=Eigen::Map<Eigen::VectorXd>(qv.data(),qv.size());
      const Eigen::VectorXd velocity=Eigen::Map<Eigen::VectorXd>(vv.data(),vv.size());
      auto sampleAt=[&](double t,moveit::core::RobotStatePtr& sample) {
        sampleTrajectory(reference,t,*sample);
      };
      Eigen::MatrixXd target(names_.size(),config_.horizon);
      for(int k=0;k<config_.horizon;++k)
      {
        auto sample=std::make_shared<moveit::core::RobotState>(state);
        sampleAt((k+1)*config_.dt,sample);
        std::vector<double> values;sample->copyJointGroupPositions(group_model_,values);
        target.col(k)=Eigen::Map<Eigen::VectorXd>(values.data(),values.size());
      }
      const auto predictions=predictions_.snapshot();
      for(const auto& p:predictions) if(p.position_uncertainty!=0)
        throw std::runtime_error("uncertainty tube is not supported by this MPC backend");
      auto setState=[&](const Eigen::VectorXd& x,double t) {
        applyPredictions(state,predictions,names_,now+rclcpp::Duration::from_seconds(t));
        state.setJointGroupPositions(group_model_,x);state.update();
      };
      DistanceOracle distance=[&](const Eigen::VectorXd& x,double t) {
        setState(x,t);return minimumDistance(*scene,state,group_,config_.influence_distance+.01);
      };
      DistanceGradientOracle gradient=[&](const Eigen::VectorXd& x,double t) {
        setState(x,t);return minimumDistanceGradient(*scene,state,group_,config_.influence_distance+.01);
      };
      PathConstraints constraints(*scene);
      for(const auto& constraint:goal->local_constraints) constraints.add(constraint);
      StatePredicate allowed=[&](const Eigen::VectorXd& x,double t) {
        setState(x,t);return constraints.satisfied(state);
      };
      const auto result=solver_->solve(q,velocity,target,lower_,upper_,vmax_,amax_,distance,allowed,gradient);
      if(!result.feasible)throw std::runtime_error(result.reason);
      // A two-point short segment is consumed by a dedicated command router/JTC.
      solution.joint_names=names_;
      trajectory_msgs::msg::JointTrajectoryPoint first,next;
      first.positions=qv;first.velocities=vv;
      const Eigen::VectorXd qnext=result.positions.col(0),vnext=result.velocities.col(0);
      next.positions.assign(qnext.data(),qnext.data()+qnext.size());
      next.velocities.assign(vnext.data(),vnext.data()+vnext.size());
      next.time_from_start=rclcpp::Duration::from_seconds(config_.dt);
      solution.points={first,next};
      auto brake=brakingTrajectory(names_,qnext,vnext,lower_,upper_,amax_,.005,config_.clearance,
          [&](const Eigen::VectorXd& x,double t){return distance(x,t+config_.dt);},
          [&](const Eigen::VectorXd& x,double t){return allowed(x,t+config_.dt);});
      for(size_t i=1;i<brake.points.size();++i)
      {auto point=brake.points[i];point.time_from_start=rclcpp::Duration(point.time_from_start)+rclcpp::Duration::from_seconds(config_.dt);solution.points.push_back(point);}
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>config_.dt)
        throw std::runtime_error("MPC validation exceeded control period; late command rejected");
      solution.header.stamp=node_->now();command_pub_->publish(solution);
      Prediction prediction;prediction.source=source_;prediction.version=++version_;
      prediction.trajectory.joint_names=names_;prediction.trajectory.header.stamp=now;
      prediction.trajectory.points.push_back(first);
      for(int k=0;k<config_.horizon;++k)
      {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        const Eigen::VectorXd x=result.positions.col(k),v=result.velocities.col(k);
        point.positions.assign(x.data(),x.data()+x.size());point.velocities.assign(v.data(),v.data()+v.size());
        point.time_from_start=rclcpp::Duration::from_seconds((k+1)*config_.dt);
        prediction.trajectory.points.push_back(point);
      }
      // Long forecast follows the nominal reference after the optimized prefix.
      // It is a point forecast, not a reservation guaranteeing future execution.
      const double optimized_duration=config_.horizon*config_.dt;
      const int tail=static_cast<int>(std::floor(reference.getDuration()/config_.dt));
      auto anchor=std::make_shared<moveit::core::RobotState>(state);
      sampleAt(optimized_duration,anchor);
      std::vector<double> nominal_anchor;anchor->copyJointGroupPositions(group_model_,nominal_anchor);
      const Eigen::VectorXd correction=result.positions.col(config_.horizon-1)-
          Eigen::Map<Eigen::VectorXd>(nominal_anchor.data(),nominal_anchor.size());
      for(int k=config_.horizon+1;k<=tail;++k)
      {
        auto sample=std::make_shared<moveit::core::RobotState>(state);
        sampleAt(k*config_.dt,sample);
        std::vector<double> values;sample->copyJointGroupPositions(group_model_,values);
        const double blend=std::max(0.0,1.0-(k*config_.dt-optimized_duration));
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for(size_t j=0;j<values.size();++j)values[j]+=blend*correction[j];
        point.positions=values;point.time_from_start=rclcpp::Duration::from_seconds(k*config_.dt);
        prediction.trajectory.points.push_back(point);
      }
      prediction.valid_until=now+rclcpp::Duration::from_seconds(std::max(optimized_duration,tail*config_.dt));
      prediction_pub_->publish(prediction);
    }
    catch(const std::exception& e)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,"MPC stopped: %s",e.what());
      // Stock LocalPlanner does not publish local_solution when feedback is set.
      // CUSTOM output requests bounded braking before requesting replan.
      moveit::core::RobotState current(monitor_->getRobotModel());
      {planning_scene_monitor::LockedPlanningSceneRO locked(monitor_);current=locked->getCurrentState();}
      publishBrake(current);
      feedback.feedback=moveit::hybrid_planning::toString(moveit::hybrid_planning::LocalFeedbackEnum::COLLISION_AHEAD);
    }
    return feedback;
  }
private:
  bool publishBrake(moveit::core::RobotState state)
  {
    try
    {
      if(!monitor_->getStateMonitor()->haveCompleteState(rclcpp::Duration::from_seconds(age_)))return false;
      std::vector<double> q,v;state.copyJointGroupPositions(group_model_,q);state.copyJointGroupVelocities(group_model_,v);
      const Eigen::VectorXd position=Eigen::Map<Eigen::VectorXd>(q.data(),q.size()),velocity=Eigen::Map<Eigen::VectorXd>(v.data(),v.size());
      const auto at=node_->now();const auto predictions=predictions_.snapshot();
      planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
      auto brake=brakingTrajectory(names_,position,velocity,lower_,upper_,amax_,.005,config_.clearance,
          [&](const Eigen::VectorXd& x,double t){
            applyPredictions(state,predictions,names_,at+rclcpp::Duration::from_seconds(t));
            state.setJointGroupPositions(group_model_,x);state.update();
            return minimumDistance(*scene.operator->(),state,group_,config_.influence_distance+.01);
          });
      brake.header.stamp=node_->now();command_pub_->publish(brake);return true;
    }
    catch(const std::exception& e)
    {RCLCPP_ERROR(node_->get_logger(),"Braking rejected; previous checked tail retained: %s",e.what());return false;}
  }
  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  const moveit::core::JointModelGroup* group_model_{nullptr};
  std::string group_,source_;std::vector<std::string> names_;MpcConfig config_;
  double age_{0.2},acceleration_{0.5};bool enabled_{false};std::atomic<bool> stopped_{false};uint64_t version_{0};
  std::mutex solver_mutex_;
  Eigen::VectorXd lower_,upper_,vmax_,amax_;std::unique_ptr<MpcSolver> solver_;PredictionStore predictions_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_pub_;
  rclcpp::Publisher<Prediction>::SharedPtr prediction_pub_;
  rclcpp::Subscription<Prediction>::SharedPtr prediction_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};
}
PLUGINLIB_EXPORT_CLASS(multi_arm_motion::DistanceMpc,moveit::hybrid_planning::LocalConstraintSolverInterface)
