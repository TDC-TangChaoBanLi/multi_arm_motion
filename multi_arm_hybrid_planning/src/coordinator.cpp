#include <multi_arm_motion_interfaces/action/execute_motion.hpp>
#include <multi_arm_motion_core/temporal_context.hpp>
#include <multi_arm_motion_core/trajectory.hpp>
#include <multi_arm_motion_core/distance.hpp>
#include <multi_arm_motion_core/constraints.hpp>
#include <multi_arm_motion_core/command.hpp>
#include <multi_arm_mpc/solver.hpp>
#include <multi_arm_mpc/braking.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <future>
#include <geometric_shapes/shape_operations.h>
#include <set>

namespace multi_arm_motion
{
using Action=multi_arm_motion_interfaces::action::ExecuteMotion;
using Handle=rclcpp_action::ServerGoalHandle<Action>;
using Codes=moveit_msgs::msg::MoveItErrorCodes;

// One aggregate controller owner, independently scheduled group action goals.
// Global optimization runs outside the execution lock. Every commit and every
// MPC horizon is checked against current geometry and peer motion.
class Coordinator : public rclcpp::Node
{
  struct CommandDeadlineMiss : std::runtime_error { using std::runtime_error::runtime_error; };
public:
  Coordinator():Node("multi_arm_coordinator",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)){}
  template<class T>T param(const std::string& key,T value)
  {if(!has_parameter(key))declare_parameter(key,value);return get_parameter(key).get_value<T>();}
  void initialize()
  {
    group_name_=param<std::string>("group_name","");
    enabled_=param<bool>("mpc.enable_execution",false);
    config_.dt=param<double>("mpc.dt",.05);config_.horizon=param<int>("mpc.horizon",10);
    config_.budget_s=param<double>("mpc.optimization_budget",.025);
    config_.clearance=param<double>("mpc.clearance",.025);
    config_.influence_distance=param<double>("mpc.influence_distance",.08);
    config_.iterations=param<int>("mpc.iterations",15);
    config_.task_weight=param<double>("mpc.task_weight",100000.);
    config_.terminal_weight=param<double>("mpc.terminal_weight",100000.);
    config_.tracking_weight=param<double>("mpc.tracking_weight",10.);
    config_.validation_joint_step=param<double>("mpc.validation_joint_step",.01);
    config_.validation_time_step=param<double>("mpc.validation_time_step",.01);
    max_age_=param<double>("mpc.max_state_age",.2);
    lead_=param<double>("execution.command_lead",config_.dt*.8);
    goal_timeout_=param<double>("execution.goal_timeout",60.0);
    tolerance_=param<double>("execution.joint_tolerance",.01);
    start_tolerance_=param<double>("execution.start_tolerance",.005);
    if(!std::isfinite(start_tolerance_)||start_tolerance_<=0)throw std::invalid_argument("invalid start tolerance");
    stopped_velocity_=param<double>("execution.stopped_velocity",.002);
    if(!std::isfinite(stopped_velocity_)||stopped_velocity_<=0)throw std::invalid_argument("invalid stopped velocity");
    tracking_limit_=param<double>("execution.tracking_limit",.25);
    reference_lag_tolerance_=param<double>("execution.reference_lag_tolerance",0.);
    if(!std::isfinite(reference_lag_tolerance_)||reference_lag_tolerance_<0)
      throw std::invalid_argument("invalid reference lag tolerance");
    continuous_desired_=param<bool>("execution.continuous_desired_commands",false);
    max_deadline_misses_=param<int>("execution.max_consecutive_deadline_misses",3);
    if(max_deadline_misses_<0)throw std::invalid_argument("negative deadline retry limit");
    commit_budget_=param<double>("execution.commit_validation_budget",2.0);
    if(!std::isfinite(commit_budget_)||commit_budget_<=0||commit_budget_>30)
      throw std::invalid_argument("invalid commit validation budget");
    scene_service_=param<std::string>("planning_scene_service","");
    command_steps_=param<int>("execution.command_steps",2);
    config_.stop_step=command_steps_;
    seed_duration_scale_=param<double>("execution.seed_duration_scale",1.3);
    reference_duration_scale_=param<double>("execution.reference_duration_scale",1.0);
    if(!std::isfinite(reference_duration_scale_)||reference_duration_scale_<1||reference_duration_scale_>10)
      throw std::invalid_argument("invalid reference duration scale");
    if(!std::isfinite(seed_duration_scale_)||seed_duration_scale_<1||seed_duration_scale_>10)
      throw std::invalid_argument("invalid seed duration scale");
    if(command_steps_<1||command_steps_>config_.horizon)throw std::invalid_argument("command_steps outside MPC horizon");
    required_sources_=param<std::vector<std::string>>("dynamic_stomp.required_prediction_sources",{});
    if(lead_<=.001||lead_>=config_.dt||max_age_<=0||goal_timeout_<=0||tolerance_<=0||tracking_limit_<=0)
      throw std::invalid_argument("invalid execution configuration");
    monitor_=std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(),"robot_description","coordinator_scene");
    if(!monitor_->getPlanningScene())throw std::runtime_error("robot model unavailable");
    model_=monitor_->getRobotModel();group_=model_->getJointModelGroup(group_name_);
    // Mesh bounding-sphere construction is independent of robot motion and may
    // be expensive. Warm immutable geometry before accepting execution goals.
    for(const auto* link:model_->getLinkModelsWithCollisionGeometry())
      for(const auto& shape:link->getShapes())shape_bounds_.get(shape);
    if(!group_||group_->getVariableCount()!=group_->getActiveJointModels().size())
      throw std::runtime_error("aggregate group must contain bounded single DOF joints");
    names_=group_->getVariableNames();const int n=names_.size();
    lower_.resize(n);upper_.resize(n);vmax_.resize(n);amax_.resize(n);
    for(int j=0;j<n;++j)
    {
      const auto& b=model_->getVariableBounds(names_[j]);
      if(!b.velocity_bounded_)throw std::runtime_error("explicit velocity bounds required");
      // Finite numerical domain for unwrapped continuous coordinates; this is
      // not a physical position limit. Multi-DOF groups were rejected above.
      const double guard=param<double>("mpc.continuous_coordinate_guard",10000.0);
      if(!std::isfinite(guard)||guard<=0)throw std::runtime_error("invalid continuous coordinate guard");
      lower_[j]=b.position_bounded_?b.min_position_:-guard;
      upper_[j]=b.position_bounded_?b.max_position_:guard;
      vmax_[j]=std::min(std::abs(b.min_velocity_),std::abs(b.max_velocity_));
      amax_[j]=b.acceleration_bounded_?std::min(std::abs(b.min_acceleration_),std::abs(b.max_acceleration_)):
          param<double>("mpc.default_acceleration",.5);
      if(vmax_[j]<=0||amax_[j]<=0)throw std::runtime_error("invalid joint dynamic limits");
    }
    monitor_->startSceneMonitor(param<std::string>("monitored_planning_scene_topic","/monitored_planning_scene"));
    monitor_->startStateMonitor(param<std::string>("joint_states_topic","/joint_states"));
    monitor_->getStateMonitor()->enableCopyDynamics(true);
    monitor_->addUpdateCallback([this](planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update){
      if(update & planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY)++scene_version_;
    });
    loader_=std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>("moveit_core","planning_interface::PlannerManager");
    manager_=loader_->createSharedInstance("dynamic_stomp/DynamicStompPlanner");
    // The coordinator supplies immutable forecasts directly through the planner
    // extension; its short published forecast must not enter a second cache.
    if(!param<bool>("dynamic_stomp.external_temporal_context",true))
      throw std::runtime_error("coordinator requires external_temporal_context=true");
    if(!manager_->initialize(model_,shared_from_this(),"dynamic_stomp"))throw std::runtime_error("STOMP initialization failed");
    solver_=std::make_unique<MpcSolver>(config_);
    command_=create_publisher<trajectory_msgs::msg::JointTrajectory>(param<std::string>("mpc.command_topic","mpc_joint_commands"),1);
    source_=param<std::string>("mpc.prediction_source",get_fully_qualified_name());
    if(source_.empty())throw std::invalid_argument("prediction source must not be empty");
    const auto prediction_topic=param<std::string>("mpc.prediction_topic","/motion_predictions");
    forecast_=create_publisher<Prediction>(prediction_topic,10);
    prediction_sub_=create_subscription<Prediction>(prediction_topic,10,[this](Prediction::ConstSharedPtr p){
      if(p->source==source_)return;std::string error;
      if(!predictions_.update(*p,error))RCLCPP_WARN(get_logger(),"Forecast rejected: %s",error.c_str());
    });
    server_=rclcpp_action::create_server<Action>(shared_from_this(),"execute_motion",
      [this](const rclcpp_action::GoalUUID& id,std::shared_ptr<const Action::Goal> goal){return reserve(id,*goal);},
      [](std::shared_ptr<Handle>){return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<Handle> handle){accept(handle);});
    stop_=create_service<std_srvs::srv::Trigger>("stop_motion",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
      std::lock_guard<std::mutex> lock(mutex_);stopped_=true;brake_requested_=true;
      response->success=true;response->message="Stop latched; validated braking requested";
    });
    resume_=create_service<std_srvs::srv::Trigger>("resume_motion",[this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
      std::lock_guard<std::mutex> lock(mutex_);
      if(!goals_.empty()||!fresh()){response->success=false;response->message="Goals must terminate and fresh stopped joint feedback is required";return;}
      auto state=snapshot()->getCurrentState();std::vector<double> v;state.copyJointGroupVelocities(group_,v);
      if(std::any_of(v.begin(),v.end(),[](double x){return std::abs(x)>.02;}))
      {response->success=false;response->message="Robot is still moving";return;}
      stopped_=false;brake_requested_=false;last_command_=trajectory_msgs::msg::JointTrajectory();solver_->reset();
      response->success=true;response->message="Execution resumed from fresh stopped feedback";
    });
    reset_predictions_=create_service<std_srvs::srv::Trigger>("reset_motion_predictions",[this](
        std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
      std::lock_guard<std::mutex> lock(mutex_);
      if(!stopped_||!goals_.empty())
      {response->success=false;response->message="Stop execution and terminate goals before resetting forecast versions";return;}
      predictions_.clear();response->success=true;response->message="External forecast cache cleared; republish required predictions before resume";
    });
    timer_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_=create_wall_timer(std::chrono::duration<double>(config_.dt),[this]{tick();},timer_group_);
    RCLCPP_INFO(get_logger(),"Concurrent ExecuteMotion ready for %s (%zu joints), execution %s",group_name_.c_str(),names_.size(),enabled_?"enabled":"disabled");
  }
  ~Coordinator() override {for(auto& job:jobs_)if(job.valid())job.wait();}
  void shutdown()
  {
    if(timer_)timer_->cancel();
    {
      std::lock_guard<std::mutex> lock(mutex_);shutting_down_=true;
      for(const auto& goal:goals_)if(goal->context)goal->context->terminate();
    }
    for(auto& job:jobs_)if(job.valid())job.wait();jobs_.clear();goals_.clear();
    // Plugin managers and PSM retain their node. Release them explicitly before
    // the node leaves scope rather than leaving a shared_ptr ownership cycle.
    manager_.reset();loader_.reset();
    if(monitor_){monitor_->stopStateMonitor();monitor_->stopSceneMonitor();}
    group_=nullptr;model_.reset();monitor_.reset();
  }
private:
  struct Goal
  {
    rclcpp_action::GoalUUID id;std::shared_ptr<Handle> handle;
    planning_interface::PlanningContextPtr context;
    robot_trajectory::RobotTrajectoryPtr path;
    rclcpp::Time origin{0,0,RCL_ROS_TIME};
    std::chrono::steady_clock::time_point deadline;
    std::string group;double progress_floor{0};bool planning{true};
  };
  bool fresh() const {return monitor_->getStateMonitor()->haveCompleteState(rclcpp::Duration::from_seconds(max_age_));}
  planning_scene::PlanningScenePtr snapshot() const
  {
    const auto measured=monitor_->getStateMonitor()->getCurrentState();
    planning_scene_monitor::LockedPlanningSceneRO locked(monitor_);
    auto scene=locked->diff();scene->decoupleParent();
    auto& state=scene->getCurrentStateNonConst();
    state.setVariablePositions(measured->getVariablePositions());
    if(measured->hasVelocities())state.setVariableVelocities(measured->getVariableVelocities());
    state.update();return scene;
  }
  bool sameTransforms(const planning_scene::PlanningScene& snapshot) const
  {
    planning_scene_monitor::LockedPlanningSceneRO current(monitor_);
    const auto& a=snapshot.getTransforms().getAllTransforms();
    const auto& b=current->getTransforms().getAllTransforms();
    if(a.size()!=b.size())return false;
    for(const auto& transform:a)
    {
      const auto it=b.find(transform.first);
      if(it==b.end()||!transform.second.matrix().isApprox(it->second.matrix(),1e-10))return false;
    }
    return true;
  }
  bool sameGeometry(const planning_scene::PlanningScene& snapshot) const
  {
    planning_scene_monitor::LockedPlanningSceneRO current(monitor_);
    if(snapshot.getCollisionEnv()->getLinkPadding()!=current->getCollisionEnv()->getLinkPadding()||
       snapshot.getCollisionEnv()->getLinkScale()!=current->getCollisionEnv()->getLinkScale())return false;
    moveit_msgs::msg::AllowedCollisionMatrix a,b;
    snapshot.getAllowedCollisionMatrix().getMessage(a);current->getAllowedCollisionMatrix().getMessage(b);
    if(a!=b)return false;
    auto shapesEqual=[](const auto& a,const auto& b){
      if(a.size()!=b.size())return false;
      for(size_t i=0;i<a.size();++i)if(a[i]!=b[i])
      {
        shapes::ShapeMsg ma,mb;
        if(!shapes::constructMsgFromShape(a[i].get(),ma)||!shapes::constructMsgFromShape(b[i].get(),mb)||ma!=mb)return false;
      }
      return true;
    };
    auto transformsEqual=[](const auto& a,const auto& b){
      if(a.size()!=b.size())return false;
      for(size_t i=0;i<a.size();++i)if(!a[i].matrix().isApprox(b[i].matrix(),1e-10))return false;
      return true;
    };
    const auto ids=snapshot.getWorld()->getObjectIds();
    if(ids!=current->getWorld()->getObjectIds())return false;
    for(const auto& id:ids)
    {
      const auto left=snapshot.getWorld()->getObject(id),right=current->getWorld()->getObject(id);
      if(!left->pose_.matrix().isApprox(right->pose_.matrix(),1e-10)||!shapesEqual(left->shapes_,right->shapes_)||
         !transformsEqual(left->shape_poses_,right->shape_poses_))return false;
    }
    std::vector<const moveit::core::AttachedBody*> attached;snapshot.getCurrentState().getAttachedBodies(attached);
    std::vector<const moveit::core::AttachedBody*> live;current->getCurrentState().getAttachedBodies(live);
    if(attached.size()!=live.size())return false;
    for(const auto* body:attached)
    {
      const auto* other=current->getCurrentState().getAttachedBody(body->getName());
      if(!other||body->getAttachedLinkName()!=other->getAttachedLinkName()||body->getTouchLinks()!=other->getTouchLinks()||
         !shapesEqual(body->getShapes(),other->getShapes())||
         !transformsEqual(body->getShapePosesInLinkFrame(),other->getShapePosesInLinkFrame()))return false;
    }
    return true;
  }
  rclcpp_action::GoalResponse reserve(const rclcpp_action::GoalUUID& id,const Action::Goal& goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);const auto& request=goal.request;
    const auto* group=model_->getJointModelGroup(request.group_name);
    if(!enabled_||stopped_||!fresh()||!group||request.goal_constraints.empty()||
       !std::isfinite(goal.timeout)||goal.timeout<0||!std::isfinite(request.allowed_planning_time)||
       request.allowed_planning_time<=0||request.allowed_planning_time>30||
       request.max_velocity_scaling_factor<=0||request.max_velocity_scaling_factor>1||
       request.max_acceleration_scaling_factor<=0||request.max_acceleration_scaling_factor>1)
      return rclcpp_action::GoalResponse::REJECT;
    for(const auto& name:group->getVariableNames())
    {
      if(std::find(names_.begin(),names_.end(),name)==names_.end())return rclcpp_action::GoalResponse::REJECT;
      for(const auto& other:goals_)
      {
        const auto& peer=model_->getJointModelGroup(other->group)->getVariableNames();
        if(std::find(peer.begin(),peer.end(),name)!=peer.end())return rclcpp_action::GoalResponse::REJECT;
      }
    }
    auto entry=std::make_shared<Goal>();entry->id=id;entry->group=request.group_name;
    entry->deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(goal.timeout>0?goal.timeout:goal_timeout_));
    goals_.push_back(entry);return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  void accept(const std::shared_ptr<Handle>& handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it=std::find_if(goals_.begin(),goals_.end(),[&](const auto& g){return g->id==handle->get_goal_id();});
    if(it==goals_.end())return;auto goal=*it;goal->handle=handle;
    jobs_.erase(std::remove_if(jobs_.begin(),jobs_.end(),[](auto& job){
      return job.wait_for(std::chrono::seconds(0))==std::future_status::ready;}),jobs_.end());
    jobs_.push_back(std::async(std::launch::async,[this,goal]{plan(goal);}));
  }
  void finish(const std::shared_ptr<Goal>& goal,int code,const std::string& message)
  {
    if(goal->path)++schedule_version_;
    if(goal->path&&code!=Codes::SUCCESS&&goal->handle)
      stopping_constraints_[goal->group]=goal->handle->get_goal()->request.path_constraints;
    if(goal->context)goal->context->terminate();
    if(goal->handle&&goal->handle->is_active())
    {
      auto result=std::make_shared<Action::Result>();result->error_code.val=code;result->message=message;
      if(goal->handle->is_canceling())goal->handle->canceled(result);
      else if(code==Codes::SUCCESS)goal->handle->succeed(result);else goal->handle->abort(result);
    }
    goals_.erase(std::remove(goals_.begin(),goals_.end(),goal),goals_.end());
    RCLCPP_INFO(get_logger(),"%s: %s (code %d)",goal->group.c_str(),message.c_str(),code);
  }
  std::vector<Prediction> forecasts(const rclcpp::Time& start,double duration) const
  {
    auto result=predictions_.snapshot();
    for(const auto& goal:goals_)if(goal->path)
    {
      Prediction p;p.source="goal/"+goal->group;p.version=1;p.trajectory.header.stamp=start;
      p.trajectory.joint_names=goal->path->getGroup()->getVariableNames();
      for(double t=0;t<=duration+1e-8;t+=config_.dt)
      {
        auto state=goal->path->getFirstWayPoint();
        sampleTrajectory(*goal->path,std::max(0.0,(start-goal->origin).seconds()+t),state);
        trajectory_msgs::msg::JointTrajectoryPoint point;state.copyJointGroupPositions(goal->path->getGroup(),point.positions);
        point.time_from_start=rclcpp::Duration::from_seconds(t);p.trajectory.points.push_back(point);
      }
      p.valid_until=start+rclcpp::Duration::from_seconds(duration+config_.dt);result.push_back(p);
    }
    requireForecasts(result);return result;
  }
  void requireForecasts(const std::vector<Prediction>& forecasts) const
  {
    for(const auto& source:required_sources_)
      if(std::none_of(forecasts.begin(),forecasts.end(),[&](const auto& p){return p.source==source;}))
        throw std::runtime_error("missing required motion prediction: "+source);
  }
  void plan(const std::shared_ptr<Goal>& goal)
  {
    try
    {
      // A successful authoritative scene mutation need not have reached the
      // monitored-scene subscriber yet. Synchronize outside the execution lock
      // before planning a stage that depends on new contact/attachment state.
      if(!scene_service_.empty()&&!monitor_->requestPlanningSceneState(scene_service_))
        throw std::runtime_error("authoritative planning scene synchronization failed: "+scene_service_);
      planning_scene::PlanningScenePtr scene;planning_interface::MotionPlanRequest request;
      planning_interface::PlanningContextPtr context;
      robot_trajectory::RobotTrajectoryPtr reference;
      double reference_step=0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if(shutting_down_||!rclcpp::ok()||!goal->handle->is_active())return;
        scene=snapshot();request=goal->handle->get_goal()->request;
        // Actual feedback is authoritative. Reject explicit stale start states
        // instead of executing a disconnected plan.
        auto start=scene->getCurrentState();
        if(!request.start_state.joint_state.name.empty())
        {
          auto specified=start;moveit::core::robotStateMsgToRobotState(request.start_state,specified);
          for(const auto& name:model_->getJointModelGroup(goal->group)->getVariableNames())
            if(std::abs(specified.getVariablePosition(name)-start.getVariablePosition(name))>start_tolerance_)
              throw std::runtime_error("requested start state differs from feedback");
        }
        moveit::core::robotStateToRobotStateMsg(start,request.start_state);
        goal->origin=now()+rclcpp::Duration::from_seconds(request.allowed_planning_time+lead_+.1);
        const auto& supplied=goal->handle->get_goal()->reference;
        if(!supplied.points.empty())
        {
          Prediction check;check.source="request/reference";check.version=1;check.trajectory=supplied;
          check.trajectory.header.stamp=goal->origin;
          const double duration=rclcpp::Duration(supplied.points.back().time_from_start).seconds();
          if(supplied.points.size()<2||duration<=0||duration>300)
            throw std::runtime_error("reference must have at least two samples and duration in (0,300]");
          check.valid_until=goal->origin+rclcpp::Duration::from_seconds(duration);
          validatePrediction(check);
          auto provided=supplied.joint_names,expected=model_->getJointModelGroup(goal->group)->getVariableNames();
          std::sort(provided.begin(),provided.end());std::sort(expected.begin(),expected.end());
          if(provided!=expected)throw std::runtime_error("reference joints must exactly match the controlled group");
          reference=std::make_shared<robot_trajectory::RobotTrajectory>(model_,goal->group);
          auto timed=supplied;
          if(!goal->handle->get_goal()->optimize_reference)
          {
            if(duration*reference_duration_scale_>300)throw std::runtime_error("reference exceeds maximum horizon after timing scale");
            for(auto& point:timed.points)
            {
              point.time_from_start=rclcpp::Duration::from_seconds(rclcpp::Duration(point.time_from_start).seconds()*reference_duration_scale_);
              for(auto& v:point.velocities)v/=reference_duration_scale_;
              for(auto& a:point.accelerations)a/=(reference_duration_scale_*reference_duration_scale_);
            }
          }
          reference->setRobotTrajectoryMsg(start,timed);
          if(goal->handle->get_goal()->optimize_reference)
          {
            const int steps=std::max(3,param<int>("dynamic_stomp.num_timesteps",40));
            // Choose the optimization grid before forecasting peers. Preserve
            // this grid through commit and execution; do not retime afterwards.
            if(duration*seed_duration_scale_>300)throw std::runtime_error("optimized reference exceeds maximum horizon");
            reference_step=duration*seed_duration_scale_/(steps-1);
            request.trajectory_constraints.constraints.clear();
            for(int i=0;i<steps;++i)
            {
              auto sample=start;sampleTrajectory(*reference,duration*i/(steps-1),sample);
              moveit_msgs::msg::Constraints seed;
              for(const auto& name:model_->getJointModelGroup(goal->group)->getVariableNames())
              {moveit_msgs::msg::JointConstraint c;c.joint_name=name;c.position=sample.getVariablePosition(name);seed.joint_constraints.push_back(c);}
              request.trajectory_constraints.constraints.push_back(seed);
            }
          }
        }
        if(!reference||goal->handle->get_goal()->optimize_reference)
        {
        moveit_msgs::msg::MoveItErrorCodes error;
        goal->context=manager_->getPlanningContext(scene,request,error);
        if(!goal->context)throw std::runtime_error("STOMP rejected planning request, code "+std::to_string(error.val));
        auto* temporal=dynamic_cast<TemporalContext*>(goal->context.get());
        if(!temporal)throw std::runtime_error("planner does not expose temporal context");
        auto peers=forecasts(goal->origin,std::max(30.0,reference?reference->getDuration()*seed_duration_scale_+config_.dt:30.0));auto validation=start;
        applyPredictions(validation,peers,model_->getJointModelGroup(goal->group)->getVariableNames(),goal->origin);
        temporal->setTemporalContext(std::move(peers),goal->origin);
        if(reference_step>0)temporal->setTemporalStep(reference_step);
        context=goal->context;
        }
      }
      planning_interface::MotionPlanResponse result;
      if(context)context->solve(result);
      else {result.trajectory=reference;result.error_code.val=Codes::SUCCESS;}
      std::string global_failure_detail;
      if(result.error_code.val!=Codes::SUCCESS&&reference)
      {
        // Diagnose the exact supplied seed, outside the MPC lock. These are
        // geometric observations, not a claim that optimization is impossible.
        auto sample=scene->getCurrentState();
        double minimum=std::numeric_limits<double>::infinity(),minimum_t=0;
        std::string closest;
        for(int i=0;i<40;++i)
        {
          const double t=reference->getDuration()*i/39.;sampleTrajectory(*reference,t,sample);
          collision_detection::DistanceRequest query;query.group_name=goal->group;
          query.enableGroup(model_);query.acm=&scene->getAllowedCollisionMatrix();
          query.type=collision_detection::DistanceRequestTypes::SINGLE;query.distance_threshold=.05;
          collision_detection::DistanceResult self,world;
          scene->getCollisionEnv()->distanceSelf(query,self,sample);
          scene->getCollisionEnv()->distanceRobot(query,world,sample);
          const auto& pair=self.minimum_distance.distance<world.minimum_distance.distance?
              self.minimum_distance:world.minimum_distance;
          if(pair.distance<minimum)
          {minimum=pair.distance;minimum_t=t;closest=pair.link_names[0]+" / "+pair.link_names[1];}
          if(i==0||i==39)global_failure_detail+=std::string(i==0?"; seed start clearance=":"; seed goal clearance=")+std::to_string(pair.distance)+" m";
        }
        global_failure_detail+="; sampled seed minimum="+std::to_string(minimum)+" m at t="+
            std::to_string(minimum_t)+" s, pair="+closest;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      if(shutting_down_||!rclcpp::ok()||!goal->handle->is_active())return;
      goal->context.reset();
      if(goal->handle->is_canceling()){finish(goal,Codes::PREEMPTED,"Canceled during global planning");return;}
      if(result.error_code.val!=Codes::SUCCESS||!result.trajectory)
      {finish(goal,result.error_code.val,"STOMP optimization failed; no planner fallback"+global_failure_detail);return;}
      // Preserve the optimized relative time law. If optimization consumes the
      // original lead, reserve time for a full sweep at a new absolute origin.
      // No shifted result is accepted without checking fresh peer forecasts.
      if(now()+rclcpp::Duration::from_seconds(lead_+commit_budget_)>goal->origin)
        goal->origin=now()+rclcpp::Duration::from_seconds(lead_+commit_budget_);
      // Commit against fresh geometry and all latest forecasts, not the planning
      // snapshot. Changed forecasts are accepted only after a complete recheck.
      for(int attempt=0;;++attempt)
      {
      const auto geometry_version=scene_version_.load(),schedule_version=schedule_version_,forecast_version=predictions_.revision();
      auto current=snapshot();auto state=current->getCurrentState();
      if(!fresh())throw std::runtime_error("joint feedback stale at commit");
      auto& first=*result.trajectory->getWayPointPtr(0);
      for(const auto& name:model_->getJointModelGroup(goal->group)->getVariableNames())
      {
        const double delta=state.getVariablePosition(name)-first.getVariablePosition(name);
        if(std::abs(delta)>start_tolerance_)
          throw std::runtime_error("controlled group moved during optimization: "+name+" delta="+
              std::to_string(delta)+" rad/m, tolerance="+std::to_string(start_tolerance_));
        // Reconcile bounded servo settling at the start, then recheck the
        // entire changed segment, constraints and temporal velocity law below.
        first.setVariablePosition(name,state.getVariablePosition(name));
      }
      first.update();
      const auto peers=forecasts(goal->origin,result.trajectory->getDuration()+config_.dt);
      const auto validation_started=std::chrono::steady_clock::now();
      // Collision checking a long global result must never block the MPC loop.
      lock.unlock();
      PathConstraints constraints(*current);constraints.add(request.path_constraints);
      const auto* group=model_->getJointModelGroup(goal->group);
      const int checks=std::max(1,int(std::ceil(result.trajectory->getDuration()/.01)));
      const double required_clearance=context ?
          get_parameter("dynamic_stomp.minimum_clearance").as_double() : config_.clearance;
      std::unique_ptr<DistanceBound> clearance_bounds;
      for(int step=0;step<=checks;++step)
      {
        const double t=result.trajectory->getDuration()*step/checks;
        sampleTrajectory(*result.trajectory,t,state);
        applyPredictions(state,peers,group->getVariableNames(),goal->origin+rclcpp::Duration::from_seconds(t));
        if(!state.satisfiesBounds(group))
          throw std::runtime_error("commit joint bound violation at trajectory t="+std::to_string(t));
        if(!constraints.satisfied(state))
          throw std::runtime_error("commit task constraint violation at trajectory t="+std::to_string(t)+": "+constraints.describeViolation(state));
        if(current->isStateColliding(state,goal->group))
        {
          collision_detection::CollisionRequest query;query.group_name=goal->group;
          query.contacts=true;query.max_contacts=5;
          collision_detection::CollisionResult collision;current->checkCollision(query,collision,state);
          std::string pairs;
          for(const auto& entry:collision.contacts)pairs+=" "+entry.first.first+" / "+entry.first.second+";";
          throw std::runtime_error("commit collision at trajectory t="+std::to_string(t)+":"+pairs);
        }
        if(required_clearance>0)
        {
          if(!clearance_bounds)clearance_bounds=std::make_unique<DistanceBound>(*current,state,goal->group,required_clearance,&shape_bounds_);
          const double distance=clearance_bounds->distance(state);
          if(std::isnan(distance)||distance<required_clearance)
            throw std::runtime_error("commit clearance violation at trajectory t="+std::to_string(t)+
                ": distance="+std::to_string(distance)+" m, required="+std::to_string(required_clearance)+" m");
        }
      }
      bool goal_satisfied=false;
      for(const auto& constraint:request.goal_constraints)
      {PathConstraints goal_constraints(*current);goal_constraints.add(constraint);goal_satisfied|=goal_constraints.satisfied(state);}
      if(!goal_satisfied)
      {
        std::ostringstream detail;detail<<"optimized endpoint violates goal:";
        for(const auto& c:request.goal_constraints.front().joint_constraints)
          detail<<" "<<c.joint_name<<"="<<state.getVariablePosition(c.joint_name)<<" requested="<<c.position;
        RCLCPP_DEBUG(get_logger(),"%s",detail.str().c_str());
        throw std::runtime_error("optimized endpoint does not satisfy requested goal constraints");
      }
      for(size_t i=1;i<result.trajectory->getWayPointCount();++i)
      {
        const double dt=result.trajectory->getWayPointDurationFromPrevious(i);
        if(!std::isfinite(dt)||dt<=0)throw std::runtime_error("STOMP returned invalid time law");
        for(const auto& name:group->getVariableNames())
        {
          const auto& b=model_->getVariableBounds(name);
          const double speed=std::abs(result.trajectory->getWayPoint(i).getVariablePosition(name)-
              result.trajectory->getWayPoint(i-1).getVariablePosition(name))/dt;
          if(speed>b.max_velocity_*request.max_velocity_scaling_factor+1e-6)
            throw std::runtime_error("STOMP fixed temporal grid exceeds requested velocity limit");
        }
      }
      lock.lock();
      if(!goal->handle->is_active())return;
      if(goal->handle->is_canceling()){finish(goal,Codes::PREEMPTED,"Canceled during commit validation");return;}
      if(schedule_version!=schedule_version_||(geometry_version!=scene_version_&&!sameGeometry(*current))||
          forecast_version!=predictions_.revision()||!sameTransforms(*current))
      {
        if(attempt<2)continue;
        throw std::runtime_error("scene, peer schedule or prediction version kept changing during commit validation");
      }
      if(now()+rclcpp::Duration::from_seconds(lead_)>=goal->origin)
      {
        if(attempt>=3)throw std::runtime_error("commit validation repeatedly missed temporal execution origin");
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-validation_started).count();
        goal->origin=now()+rclcpp::Duration::from_seconds(lead_+std::max(commit_budget_,2*elapsed));
        RCLCPP_WARN(get_logger(),"%s: commit origin deferred; rechecking complete trajectory and fresh peer forecasts",goal->group.c_str());
        continue;
      }
      goal->path=result.trajectory;goal->planning=false;++schedule_version_;
      RCLCPP_INFO(get_logger(),"%s %s committed; start %.6f, duration %.3f, latest forecasts revalidated",goal->group.c_str(),context?"STOMP":"reference",goal->origin.seconds(),goal->path->getDuration());
      break;
      }
    }
    catch(const std::exception& e)
    {std::lock_guard<std::mutex> lock(mutex_);if(!shutting_down_&&rclcpp::ok()&&goal->handle->is_active())finish(goal,Codes::PLANNING_FAILED,e.what());}
  }
  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!enabled_)return;
    const auto started=std::chrono::steady_clock::now();auto scene=snapshot();auto state=scene->getCurrentState();
    std::vector<double> qv,vv;state.copyJointGroupPositions(group_,qv);state.copyJointGroupVelocities(group_,vv);
    Eigen::VectorXd q=Eigen::Map<Eigen::VectorXd>(qv.data(),qv.size()),v=Eigen::Map<Eigen::VectorXd>(vv.data(),vv.size());
    const auto at=now()+rclcpp::Duration::from_seconds(lead_);
    auto entries=goals_;
    for(const auto& goal:entries)if(goal->handle)
    {
      if(goal->handle->is_canceling()||started>goal->deadline||stopped_)
      {if(stopped_)brake_requested_=true;finish(goal,started>goal->deadline?Codes::TIMED_OUT:Codes::PREEMPTED,
          stopped_?"Stop latched":started>goal->deadline?"Goal deadline exceeded":"Goal canceled");}
    }
    if(goals_.empty()&&!brake_requested_)return;
    if(!fresh())
    {
      // Never synthesize a new command from stale feedback. The controller
      // already holds a validated braking tail from the last accepted command.
      stopped_=true;brake_requested_=false;
      auto failed=goals_;for(const auto& goal:failed)if(goal->handle)finish(goal,Codes::ROBOT_STATE_STALE,"Joint feedback stale; last controller braking tail retained");
      return;
    }
    if(goals_.empty()&&!brake_requested_)return;
    try
    {
      if(!state.satisfiesBounds(group_)||(v.array().abs()>vmax_.array()+1e-6).any())
        throw std::runtime_error("measured joint state outside physical position or velocity bounds");
      const auto scene_version=scene_version_.load();
      const auto prediction_version=predictions_.revision();
      const auto peers=predictions_.snapshot();
      Eigen::VectorXd tracking_offset=Eigen::VectorXd::Zero(q.size());
      requireForecasts(peers);
      for(const auto& p:peers)if(p.position_uncertainty!=0)throw std::runtime_error("uncertainty margin unsupported; forecast rejected");
      auto set=[&](const Eigen::VectorXd& x,double t){
        applyPredictions(state,peers,names_,at+rclcpp::Duration::from_seconds(t));
        state.setJointGroupPositions(group_,Eigen::VectorXd(x+tracking_offset));state.update();};
      size_t queries=0;double query_time=0;std::map<std::vector<double>,double> distance_cache;
      std::string clearance_detail;
      const auto describe_clearance=[&](){
        if(!clearance_detail.empty())return;
        collision_detection::DistanceRequest query;query.group_name=group_name_;
        query.enableGroup(model_);query.acm=&scene->getAllowedCollisionMatrix();
        query.type=collision_detection::DistanceRequestTypes::SINGLE;
        query.distance_threshold=config_.clearance+.001;
        collision_detection::DistanceResult self,world;
        scene->getCollisionEnv()->distanceSelf(query,self,state);
        scene->getCollisionEnv()->distanceRobot(query,world,state);
        const auto& pair=self.minimum_distance.distance<world.minimum_distance.distance?
            self.minimum_distance:world.minimum_distance;
        clearance_detail="; nearest pair="+pair.link_names[0]+" / "+pair.link_names[1];
      };
      DistanceOracle distance=[&](const Eigen::VectorXd& x,double t){
        std::vector<double> key(x.data(),x.data()+x.size());if(!peers.empty())key.push_back(t);
        const auto found=distance_cache.find(key);if(found!=distance_cache.end())return found->second;
        const auto begin=std::chrono::steady_clock::now();set(x,t);
        const double value=minimumDistance(*scene,state,group_name_,config_.influence_distance+.01);
        if(value<config_.clearance)describe_clearance();
        query_time+=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();++queries;
        distance_cache.emplace(std::move(key),value);return value;
      };
      DistanceGradientOracle gradient=[&](const Eigen::VectorXd& x,double t){
        const auto begin=std::chrono::steady_clock::now();set(x,t);
        auto result=minimumDistanceGradient(*scene,state,group_name_,config_.influence_distance+.01);
        if(result.first<config_.clearance)describe_clearance();
        query_time+=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();++queries;return result;};
      std::unique_ptr<DistanceBound> bounds;
      DistanceOracle validation=[&](const Eigen::VectorXd& x,double t){
        set(x,t);if(!bounds)bounds=std::make_unique<DistanceBound>(*scene,state,group_name_,config_.clearance,&shape_bounds_);
        const double value=bounds->distance(state);
        if(value<config_.clearance)
        {
          collision_detection::DistanceRequest query;query.group_name=group_name_;
          query.enableGroup(model_);query.acm=&scene->getAllowedCollisionMatrix();
          query.type=collision_detection::DistanceRequestTypes::SINGLE;
          query.distance_threshold=config_.clearance+.001;
          collision_detection::DistanceResult self,world;
          scene->getCollisionEnv()->distanceSelf(query,self,state);
          scene->getCollisionEnv()->distanceRobot(query,world,state);
          const auto& pair=self.minimum_distance.distance<world.minimum_distance.distance?
              self.minimum_distance:world.minimum_distance;
          clearance_detail="; nearest pair="+pair.link_names[0]+" / "+pair.link_names[1];
          std::ostringstream joints;joints<<"; predicted joints=";
          for(size_t j=0;j<names_.size();++j)joints<<names_[j]<<"="<<x[j]<<" ";
          RCLCPP_DEBUG(get_logger(),"Clearance failure%s%s",clearance_detail.c_str(),joints.str().c_str());
        }
        return value;
      };
      PathConstraints constraints(*scene);
      for(auto it=stopping_constraints_.begin();it!=stopping_constraints_.end();)
      {
        const auto* group=model_->getJointModelGroup(it->first);bool settled=true;
        for(const auto& name:group->getVariableNames())settled&=std::abs(state.getVariableVelocity(name))<.001;
        if(settled)it=stopping_constraints_.erase(it);
        else {constraints.add(it->second);++it;}
      }
      if(!last_command_.points.empty())
      {
        const auto measured_expected=sampleCommand(last_command_,now());
        const auto next_expected=sampleCommand(last_command_,at);
        for(size_t j=0;j<names_.size();++j)
        {
          const double deviation=q[j]-measured_expected.first[j];
          if(std::abs(deviation)>tracking_limit_)throw std::runtime_error("controller deviated from the committed command: "+names_[j]);
          if(continuous_desired_)tracking_offset[j]=deviation;
          q[j]=next_expected.first[j]+(continuous_desired_?0.:deviation);v[j]=next_expected.second[j];
        }
        qv.assign(q.data(),q.data()+q.size());vv.assign(v.data(),v.data()+v.size());
      }
      Eigen::MatrixXd target= q.replicate(1,config_.horizon);bool executing=false;
      auto vmax=vmax_,amax=amax_;
      auto active=goals_;
      for(const auto& goal:active)if(goal->path)
      {
        const auto* group=goal->path->getGroup();double error=0;
        if(reference_lag_tolerance_>0&&at>goal->origin)
        {
          const double elapsed=(at-goal->origin).seconds();
          Eigen::VectorXd predicted(group->getVariableCount());
          auto nominal=state;sampleTrajectory(*goal->path,elapsed,nominal);
          double lag=0;size_t index=0;
          for(const auto& name:group->getVariableNames())
          {
            const auto j=std::find(names_.begin(),names_.end(),name)-names_.begin();
            predicted[index++]=q[j]+tracking_offset[j];
            lag=std::max(lag,std::abs(nominal.getVariablePosition(name)-q[j]-tracking_offset[j]));
          }
          const auto projection=projectTrajectoryTime(*goal->path,predicted,goal->progress_floor,
              std::min(elapsed,goal->path->getDuration()));
          goal->progress_floor=std::max(goal->progress_floor,projection.time);
          const double delay=elapsed-projection.time-config_.dt;
          if(lag>reference_lag_tolerance_&&delay>config_.dt*.25)
          {
            goal->origin=goal->origin+rclcpp::Duration::from_seconds(delay);++schedule_version_;
            RCLCPP_DEBUG(get_logger(),"%s reference progress slowed by %.6f s, lag %.6f; peers continue independently",
                goal->group.c_str(),delay,lag);
          }
        }
        for(const auto& name:group->getVariableNames())error=std::max(error,std::abs(state.getVariablePosition(name)-goal->path->getLastWayPoint().getVariablePosition(name)));
        bool settled=true;for(const auto& name:group->getVariableNames())settled&=std::abs(state.getVariableVelocity(name))<stopped_velocity_;
        if(now()>=goal->origin+rclcpp::Duration::from_seconds(goal->path->getDuration())&&error<tolerance_&&settled)
        {finish(goal,Codes::SUCCESS,"Goal reached and stopped");continue;}
        Action::Feedback feedback;feedback.phase="MPC execution";feedback.joint_error=error;goal->handle->publish_feedback(std::make_shared<Action::Feedback>(feedback));
        if(at<goal->origin)
        {
          // A committed future contact path must retain its exact starting
          // pose while waiting for its origin, rather than holding a drifting
          // sequence of instantaneous positions from the warm start.
          constraints.add(goal->handle->get_goal()->request.path_constraints);
          executing=true;
          for(const auto& name:group->getVariableNames())
          {
            const auto j=std::find(names_.begin(),names_.end(),name)-names_.begin();
            target.row(j).setConstant(goal->path->getFirstWayPoint().getVariablePosition(name));
            vmax[j]*=goal->handle->get_goal()->request.max_velocity_scaling_factor;
            amax[j]*=goal->handle->get_goal()->request.max_acceleration_scaling_factor;
          }
          continue;
        }
        auto nominal=state;sampleTrajectory(*goal->path,(at-goal->origin).seconds(),nominal);
        for(const auto& name:group->getVariableNames())
          if(std::abs(nominal.getVariablePosition(name)-state.getVariablePosition(name))>tracking_limit_)
            throw std::runtime_error("reference tracking error exceeds configured limit for "+name+
                ": nominal="+std::to_string(nominal.getVariablePosition(name))+", measured="+
                std::to_string(state.getVariablePosition(name)));
        constraints.add(goal->handle->get_goal()->request.path_constraints);
        executing=true;
        for(const auto& name:group->getVariableNames())
        {
          const auto j=std::find(names_.begin(),names_.end(),name)-names_.begin();
          vmax[j]*=goal->handle->get_goal()->request.max_velocity_scaling_factor;
          amax[j]*=goal->handle->get_goal()->request.max_acceleration_scaling_factor;
        }
        for(int k=0;k<config_.horizon;++k)
        {
          auto sample=state;sampleTrajectory(*goal->path,(at-goal->origin).seconds()+(k+1)*config_.dt,sample);
          for(const auto& name:group->getVariableNames())
          {const auto j=std::find(names_.begin(),names_.end(),name)-names_.begin();target(j,k)=sample.getVariablePosition(name);}
        }
      }
      std::string constraint_detail;
      StatePredicate allowed=[&](const Eigen::VectorXd& x,double t){
        set(x,t);const bool valid=state.satisfiesBounds(group_)&&constraints.satisfied(state);
        if(!valid)constraint_detail=constraints.describeViolation(state);return valid;};
      if(constraints.empty()&&!continuous_desired_)allowed=StatePredicate{};
      TaskCostOracle task_cost;
      if(!constraints.empty())task_cost=[&](const Eigen::VectorXd& x,double t,bool derivative){
        set(x,t);const double value=constraints.trackingCost(state);
        Eigen::VectorXd g=Eigen::VectorXd::Zero(x.size());
        if(derivative)for(Eigen::Index j=0;j<x.size();++j)
        {
          auto plus=x,minus=x;plus[j]+=config_.finite_difference;minus[j]-=config_.finite_difference;
          set(plus,t);const double a=constraints.trackingCost(state);
          set(minus,t);g[j]=(a-constraints.trackingCost(state))/(2*config_.finite_difference);
        }
        return std::make_pair(value,g);
      };
      if(brake_requested_)
      {
        auto brake=brakingTrajectory(names_,q,v,lower_,upper_,amax_,.005,config_.clearance,validation,allowed);
        if(now()>=at)throw std::runtime_error("braking validation missed scheduled controller start; previous tail retained");
        brake.header.stamp=at;command_->publish(brake);last_command_=brake;brake_requested_=false;solver_->reset();
        RCLCPP_WARN(get_logger(),"Validated acceleration bounded braking command published");return;
      }
      if(!executing)return;
      std::string stop_failure;
      trajectory_msgs::msg::JointTrajectory checked_brake;
      Eigen::VectorXd checked_brake_q,checked_brake_v;
      double checked_brake_offset=-1;
      StopPredicate can_stop=[&](const Eigen::VectorXd& x,const Eigen::VectorXd& speed,double offset){
        try
        {
          checked_brake=brakingTrajectory(names_,x,speed,lower_,upper_,amax,.005,config_.clearance,
              [&](const Eigen::VectorXd& position,double t){return validation(position,t+offset);},
              [&](const Eigen::VectorXd& position,double t){return !allowed||allowed(position,t+offset);});
          checked_brake_q=x;checked_brake_v=speed;checked_brake_offset=offset;
          return true;
        }
        catch(const std::exception& error)
        {stop_failure=std::string(error.what())+clearance_detail+
            (constraint_detail.empty()?"":"; "+constraint_detail);return false;}
      };
      std::vector<std::vector<int>> independent_groups;
      for(const auto& goal:goals_)if(goal->path)
      {
        std::vector<int> joints;
        for(const auto& name:goal->path->getGroup()->getVariableNames())
          joints.push_back(std::find(names_.begin(),names_.end(),name)-names_.begin());
        independent_groups.push_back(std::move(joints));
      }
      auto result=solver_->solve(q,v,target,lower_,upper_,vmax,amax,distance,allowed,gradient,validation,task_cost,can_stop,independent_groups);
      if(!result.feasible)throw std::runtime_error(result.reason+
          (result.reason.find("task constraint")!=std::string::npos&&!constraint_detail.empty()?"; "+constraint_detail:"")+
          (result.reason.find("clearance")!=std::string::npos?clearance_detail:"")+
          (result.reason.find("braking tail")!=std::string::npos?"; "+stop_failure:""));
      // Validation must finish within one control period, including geometry
      // checks. Late results never reach the controller.
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>lead_-.001)
        throw CommandDeadlineMiss("MPC validation missed control deadline: elapsed "+
          std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count())+" s, "+
          std::to_string(queries)+" distance queries / "+std::to_string(query_time)+" s, optimization "+
          std::to_string(result.optimization_seconds)+" s, sweep validation "+std::to_string(result.validation_seconds)+" s");
      // Append a validated braking tail to EVERY command. Lost updates, process
      // death and missed deadlines therefore leave a bounded stop in the JTC.
      const double prefix_duration=command_steps_*config_.dt;
      if(checked_brake.points.empty()||checked_brake_q.size()!=q.size()||
          (checked_brake_q-result.positions.col(command_steps_-1)).cwiseAbs().maxCoeff()>1e-9||
          (checked_brake_v-result.velocities.col(command_steps_-1)).cwiseAbs().maxCoeff()>1e-9||
          std::abs(checked_brake_offset-prefix_duration)>1e-9)
        throw std::logic_error("MPC result has no matching checked stopping tail");
      auto brake=std::move(checked_brake);
      if((scene_version!=scene_version_&&!sameGeometry(*scene))||!sameTransforms(*scene))
      {
        // Attach/detach and contact permissions can change while another group
        // moves. A revision invalidates the check, not necessarily the motion.
        // Recheck the complete candidate and its stop against fresh geometry.
        const auto updated_version=scene_version_.load();
        auto updated=snapshot();auto checked=updated->getCurrentState();
        PathConstraints updated_constraints(*updated);
        for(const auto& entry:stopping_constraints_)updated_constraints.add(entry.second);
        for(const auto& goal:goals_)if(goal->path)
          updated_constraints.add(goal->handle->get_goal()->request.path_constraints);
        std::unique_ptr<DistanceBound> updated_bounds;
        auto set_updated=[&](const Eigen::VectorXd& x,double t){
          applyPredictions(checked,peers,names_,at+rclcpp::Duration::from_seconds(t));
          checked.setJointGroupPositions(group_,Eigen::VectorXd(x+tracking_offset));checked.update();
        };
        DistanceOracle updated_distance=[&](const Eigen::VectorXd& x,double t){
          set_updated(x,t);
          if(!updated_bounds)updated_bounds=std::make_unique<DistanceBound>(*updated,checked,group_name_,config_.clearance,&shape_bounds_);
          return updated_bounds->distance(checked);
        };
        StatePredicate updated_allowed=[&](const Eigen::VectorXd& x,double t){
          set_updated(x,t);return checked.satisfiesBounds(group_)&&updated_constraints.satisfied(checked);
        };
        Eigen::VectorXd previous=q,previous_speed=v;
        for(int k=0;k<config_.horizon;++k)
        {
          const int checks=std::max(1,std::max(int(std::ceil(config_.dt/config_.validation_time_step)),
              int(std::ceil((result.positions.col(k)-previous).cwiseAbs().maxCoeff()/config_.validation_joint_step))));
          for(int i=0;i<=checks;++i)
          {
            const double a=double(i)/checks,t=(k+a)*config_.dt;
            const Eigen::VectorXd x=previous+config_.dt*(a*previous_speed+.5*a*a*(result.velocities.col(k)-previous_speed));
            if(updated_distance(x,t)<config_.clearance||!updated_allowed(x,t))
              throw std::runtime_error("updated scene invalidates MPC candidate: clearance or task constraint");
            if(now()>=at)throw CommandDeadlineMiss("updated scene revalidation missed command deadline");
          }
          previous=result.positions.col(k);previous_speed=result.velocities.col(k);
        }
        brake=brakingTrajectory(names_,result.positions.col(command_steps_-1),result.velocities.col(command_steps_-1),lower_,upper_,amax,.005,
            config_.clearance,[&](const Eigen::VectorXd& x,double t){return updated_distance(x,t+prefix_duration);},
            [&](const Eigen::VectorXd& x,double t){return updated_allowed(x,t+prefix_duration);});
        if((updated_version!=scene_version_&&!sameGeometry(*updated))||!sameTransforms(*updated))
          throw std::runtime_error("planning scene kept changing during MPC revalidation");
        RCLCPP_DEBUG(get_logger(),"Scene transition accepted after fresh MPC horizon and braking revalidation");
      }
      if(prediction_version!=predictions_.revision())throw std::runtime_error("external prediction revision changed during MPC validation");
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
      if(elapsed>lead_-.001)
        throw CommandDeadlineMiss("braking validation exceeded command deadline: elapsed "+std::to_string(elapsed)+
            " s, "+std::to_string(queries)+" optimization distance queries / "+std::to_string(query_time)+" s");
      trajectory_msgs::msg::JointTrajectory command;command.header.stamp=at;command.joint_names=names_;
      trajectory_msgs::msg::JointTrajectoryPoint first;first.positions=qv;first.velocities=vv;command.points={first};
      // Overlap checked prefixes so normal timer/transport jitter cannot enter
      // the previous command's braking tail on every update.
      for(int k=0;k<command_steps_;++k)
      {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        const Eigen::VectorXd x=result.positions.col(k),speed=result.velocities.col(k);
        point.positions.assign(x.data(),x.data()+x.size());point.velocities.assign(speed.data(),speed.data()+speed.size());
        point.time_from_start=rclcpp::Duration::from_seconds((k+1)*config_.dt);command.points.push_back(point);
      }
      for(size_t i=1;i<brake.points.size();++i)
      {auto point=brake.points[i];point.time_from_start=rclcpp::Duration(point.time_from_start)+rclcpp::Duration::from_seconds(prefix_duration);command.points.push_back(point);}
      // Both routed controllers receive the same absolute start instant.
      command.header.stamp=at;command_->publish(command);last_command_=command;
      consecutive_deadline_misses_=0;
      if(result.braking_only)
      {
        std::string paused_groups;
        for(const auto& goal:goals_)if(goal->path)
        {
          bool paused=false;
          for(const auto& name:goal->path->getGroup()->getVariableNames())
          {
            const int joint=std::find(names_.begin(),names_.end(),name)-names_.begin();
            paused|=std::find(result.paused_joints.begin(),result.paused_joints.end(),joint)!=result.paused_joints.end();
          }
          if(paused){goal->origin+=rclcpp::Duration::from_seconds(config_.dt);paused_groups+=goal->group+" ";}
        }
        ++schedule_version_;
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"Local MPC braking fallback; paused groups=[%s]: %s%s; constraint=%s; stop=%s; optimization %.4f s, sweep %.4f s",paused_groups.c_str(),result.reason.c_str(),clearance_detail.c_str(),constraint_detail.c_str(),stop_failure.c_str(),result.optimization_seconds,result.validation_seconds);
      }
      Prediction prediction;prediction.source=source_;prediction.version=++version_;prediction.trajectory=command;
      prediction.trajectory.points.resize(command_steps_+1);
      for(int k=command_steps_;k<config_.horizon;++k)
      {
        trajectory_msgs::msg::JointTrajectoryPoint point;const Eigen::VectorXd x=result.positions.col(k);
        point.positions.assign(x.data(),x.data()+x.size());point.time_from_start=rclcpp::Duration::from_seconds((k+1)*config_.dt);
        prediction.trajectory.points.push_back(point);
      }
      if(continuous_desired_)for(auto& point:prediction.trajectory.points)
        for(size_t j=0;j<names_.size();++j)point.positions[j]+=tracking_offset[j];
      prediction.valid_until=rclcpp::Time(command.header.stamp)+rclcpp::Duration::from_seconds(config_.dt*config_.horizon);forecast_->publish(prediction);
    }
    catch(const std::exception& e)
    {
      if(dynamic_cast<const CommandDeadlineMiss*>(&e) && !last_command_.points.empty() &&
          !brake_requested_ && ++consecutive_deadline_misses_<=max_deadline_misses_)
      {
        // No late result is published. The controller keeps the previous fully
        // validated command and stop. Retry from fresh feedback next cycle;
        // bounded retries never extend an action's wall-clock deadline.
        for(const auto& goal:goals_)if(goal->path)goal->origin+=rclcpp::Duration::from_seconds(config_.dt);
        ++schedule_version_;solver_->reset();
        RCLCPP_WARN(get_logger(),"MPC deadline retry %d/%d; previous checked stopping tail retained, no new command published: %s",
          consecutive_deadline_misses_,max_deadline_misses_,e.what());
        return;
      }
      stopped_=true;
      if(brake_requested_)
      {
        // Do not repeatedly replace a checked tail with an unvalidated stop.
        brake_requested_=false;last_command_=trajectory_msgs::msg::JointTrajectory();
        RCLCPP_ERROR(get_logger(),"New braking sweep unavailable; retained controller tail: %s",e.what());
      }
      else brake_requested_=true;
      auto failed=goals_;for(const auto& goal:failed)if(goal->handle)finish(goal,Codes::CONTROL_FAILED,e.what());
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"Execution latched stopped: %s",e.what());
    }
  }
  std::mutex mutex_;std::vector<std::shared_ptr<Goal>> goals_;std::vector<std::future<void>> jobs_;
  std::map<std::string,moveit_msgs::msg::Constraints> stopping_constraints_;
  std::string group_name_,source_,scene_service_;std::vector<std::string> names_,required_sources_;bool enabled_{false},stopped_{false},brake_requested_{false},shutting_down_{false},continuous_desired_{false};
  double max_age_,lead_,goal_timeout_,tolerance_,tracking_limit_,reference_lag_tolerance_;uint64_t version_{0},schedule_version_{0};std::atomic<uint64_t> scene_version_{0};MpcConfig config_;
  int command_steps_{2};
  int max_deadline_misses_{3},consecutive_deadline_misses_{0};
  double seed_duration_scale_{1.3};
  double reference_duration_scale_{1.0};
  double stopped_velocity_{.002};
  double start_tolerance_{.005};
  double commit_budget_{2.0};
  Eigen::VectorXd lower_,upper_,vmax_,amax_;moveit::core::RobotModelConstPtr model_;const moveit::core::JointModelGroup* group_{};
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;PredictionStore predictions_;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> loader_;planning_interface::PlannerManagerPtr manager_;
  std::unique_ptr<MpcSolver> solver_;rclcpp_action::Server<Action>::SharedPtr server_;
  ShapeBoundsCache shape_bounds_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_;rclcpp::Publisher<Prediction>::SharedPtr forecast_;
  rclcpp::Subscription<Prediction>::SharedPtr prediction_sub_;rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_,resume_,reset_predictions_;
  trajectory_msgs::msg::JointTrajectory last_command_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  try
  {
    auto node=std::make_shared<multi_arm_motion::Coordinator>();node->initialize();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),4);executor.add_node(node);executor.spin();node->shutdown();
  }
  catch(const std::exception& e){RCLCPP_FATAL(rclcpp::get_logger("multi_arm_coordinator"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
