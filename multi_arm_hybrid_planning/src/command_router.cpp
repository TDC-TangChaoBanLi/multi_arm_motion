#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <set>
#include <cmath>
#include <algorithm>
#include <deque>
// Routes a single MPC command into disjoint controller joint sets. Controllers
// must be exclusively owned by this execution stack while it is enabled.
class CommandRouter : public rclcpp::Node
{
public:
  CommandRouter():Node("multi_arm_command_router")
  {
    const auto controllers=declare_parameter<std::vector<std::string>>("controllers",std::vector<std::string>{});
    if(controllers.empty())throw std::invalid_argument("controllers must be configured");
    std::set<std::string> assigned;
    for(const auto& controller:controllers)
    {
      Route route;route.names=declare_parameter<std::vector<std::string>>(controller+".joints",std::vector<std::string>{});
      const auto topic=declare_parameter<std::string>(controller+".topic","");
      if(route.names.empty()||topic.empty())throw std::invalid_argument("controller joints/topic missing");
      for(const auto& name:route.names)
        if(name.empty()||!assigned.insert(name).second)throw std::invalid_argument("overlapping/empty controller joint");
      route.publisher=create_publisher<trajectory_msgs::msg::JointTrajectory>(topic,1);routes_.push_back(route);
    }
    const auto input=declare_parameter<std::string>("input_topic","mpc_joint_commands");
    require_tail_=declare_parameter<bool>("require_stopping_tail",true);
    max_age_=declare_parameter<double>("max_command_age",.1);
    max_future_=declare_parameter<double>("max_future_start",.5);
    dispatch_lead_=declare_parameter<double>("dispatch_lead",.005);
    if(!std::isfinite(dispatch_lead_)||dispatch_lead_<0||dispatch_lead_>=max_future_)
      throw std::invalid_argument("dispatch_lead outside command clock window");
    if(max_age_<=0||max_future_<=0)throw std::invalid_argument("invalid command clock limits");
    subscription_=create_subscription<trajectory_msgs::msg::JointTrajectory>(input,1,
      [this,assigned](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr command) {
        try
        {
          const rclcpp::Time stamp(command->header.stamp,get_clock()->get_clock_type());
          const double age=(now()-stamp).seconds();
          if(stamp.nanoseconds()==0||age>max_age_||age < -max_future_||stamp.nanoseconds()<=last_stamp_)
            throw std::runtime_error("stale, replayed or unsynchronized controller start stamp");
          std::set<std::string> names(command->joint_names.begin(),command->joint_names.end());
          if(names!=assigned||names.size()!=command->joint_names.size()||command->points.empty())
            throw std::runtime_error("command joint set does not match controller ownership");
          double previous=-1;
          for(const auto& p:command->points)
          {
            const double t=p.time_from_start.sec+p.time_from_start.nanosec*1e-9;
            if(t<0||t<=previous||p.positions.size()!=names.size()||
                (!p.velocities.empty()&&p.velocities.size()!=names.size())||
                (!p.accelerations.empty()&&p.accelerations.size()!=names.size())||!p.effort.empty())
              throw std::runtime_error("invalid command samples");
            for(const auto* values:{&p.positions,&p.velocities,&p.accelerations})
              for(double x:*values)if(!std::isfinite(x))throw std::runtime_error("nonfinite command");
            previous=t;
          }
          if(require_tail_)
          {
            const auto& tail=command->points.back();
            if(command->points.size()<2||tail.velocities.size()!=names.size()||
               std::any_of(tail.velocities.begin(),tail.velocities.end(),[](double x){return std::abs(x)>1e-8;}))
              throw std::runtime_error("command is missing a zero velocity braking tail");
          }
          std::vector<trajectory_msgs::msg::JointTrajectory> outputs;
          for(const auto& route:routes_)
          {
            trajectory_msgs::msg::JointTrajectory out;out.header=command->header;out.joint_names=route.names;
            for(const auto& p:command->points)
            {
              trajectory_msgs::msg::JointTrajectoryPoint point;point.time_from_start=p.time_from_start;
              for(const auto& name:route.names)
              {
                const auto j=std::find(command->joint_names.begin(),command->joint_names.end(),name)-command->joint_names.begin();
                point.positions.push_back(p.positions[j]);
                if(!p.velocities.empty())point.velocities.push_back(p.velocities[j]);
                if(!p.accelerations.empty())point.accelerations.push_back(p.accelerations[j]);
              }
              out.points.push_back(point);
            }
            outputs.push_back(out);
          }
          for(const auto& route:routes_)if(route.publisher->get_subscription_count()==0)
            throw std::runtime_error("controller command subscriber unavailable");
          pending_.push_back({stamp,std::move(outputs)});
          last_stamp_=stamp.nanoseconds();
        }
        catch(const std::exception& e){RCLCPP_ERROR(get_logger(),"Rejected MPC command: %s",e.what());}
      });
    // JTC replaces a trajectory on receipt and interpolates from feedback to a
    // future first sample. Publishing far ahead would repeatedly discard most
    // of the running prefix. Keep the timestamp, dispatch close to that start.
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this]{
      while(!pending_.empty()&&(now()-pending_.front().stamp).seconds()>=-dispatch_lead_)
      {
        auto next=std::move(pending_.front());pending_.pop_front();
        if((now()-next.stamp).seconds()>max_age_)
        {RCLCPP_ERROR(get_logger(),"Queued command missed its dispatch window; retained controller braking tail");continue;}
        if(std::any_of(routes_.begin(),routes_.end(),[](const auto& route){return route.publisher->get_subscription_count()==0;}))
        {RCLCPP_ERROR(get_logger(),"Controller subscriber disappeared before dispatch; retained controller braking tail");continue;}
        for(size_t i=0;i<routes_.size();++i)routes_[i].publisher->publish(next.outputs[i]);
      }
    });
  }
private:
  struct Route {std::vector<std::string> names;rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;};
  std::vector<Route> routes_;rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  bool require_tail_{true};double max_age_{.1},max_future_{.5};int64_t last_stamp_{-1};
  struct Pending {rclcpp::Time stamp;std::vector<trajectory_msgs::msg::JointTrajectory> outputs;};
  std::deque<Pending> pending_;rclcpp::TimerBase::SharedPtr timer_;double dispatch_lead_{.005};
};
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<CommandRouter>());}
  catch(const std::exception& e){RCLCPP_FATAL(rclcpp::get_logger("command_router"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
