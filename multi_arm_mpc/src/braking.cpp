#include <multi_arm_mpc/braking.hpp>
#include <rclcpp/duration.hpp>
#include <cmath>
#include <stdexcept>

namespace multi_arm_motion
{
trajectory_msgs::msg::JointTrajectory brakingTrajectory(
    const std::vector<std::string>& names,const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper,const Eigen::VectorXd& amax,
    double dt,double clearance,const DistanceOracle& distance,const StatePredicate& allowed)
{
  const auto n=q.size();
  if(!n||names.size()!=static_cast<size_t>(n)||v.size()!=n||lower.size()!=n||upper.size()!=n||amax.size()!=n||
     !q.allFinite()||!v.allFinite()||!lower.allFinite()||!upper.allFinite()||!amax.allFinite()||
     (amax.array()<=0).any()||!(dt>0)||!std::isfinite(dt)||!std::isfinite(clearance)||clearance<0||!distance)
    throw std::invalid_argument("invalid braking input");
  const double duration=std::max(dt,(v.cwiseAbs().array()/amax.array()).maxCoeff());
  const int count=std::max(1,static_cast<int>(std::ceil(duration/dt)));
  trajectory_msgs::msg::JointTrajectory trajectory;trajectory.joint_names=names;
  for(int i=0;i<=count;++i)
  {
    const double t=duration*i/count;
    const Eigen::VectorXd x=q+v*(t-0.5*t*t/duration),velocity=v*(1-t/duration);
    if((x.array()<lower.array()).any()||(x.array()>upper.array()).any())
      throw std::runtime_error("braking joint bound violation at t="+std::to_string(t)+" s");
    const double separation=distance(x,t);
    if(std::isnan(separation)||separation<clearance)
      throw std::runtime_error("braking clearance violation at t="+std::to_string(t)+" s: distance="+
          std::to_string(separation)+" m, required="+std::to_string(clearance)+" m");
    if(allowed&&!allowed(x,t))
      throw std::runtime_error("braking task constraint violation at t="+std::to_string(t)+" s");
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(x.data(),x.data()+n);point.velocities.assign(velocity.data(),velocity.data()+n);
    const Eigen::VectorXd acceleration=-v/duration;
    point.accelerations.assign(acceleration.data(),acceleration.data()+n);
    point.time_from_start=rclcpp::Duration::from_seconds(t);trajectory.points.push_back(point);
  }
  // The end acceleration is zero; intermediate points describe a quadratic
  // position segment using positions/velocities, matching cubic JTC interpolation.
  for(auto& p:trajectory.points)p.accelerations.clear();
  return trajectory;
}
}
