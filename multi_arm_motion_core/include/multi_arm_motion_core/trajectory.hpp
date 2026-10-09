#pragma once
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>
namespace multi_arm_motion
{
struct TrajectoryProjection { double time; double distance; };
// Nearest point on the requested portion of an unwrapped joint trajectory.
// Used for independent reference progress; never changes its geometric path.
inline TrajectoryProjection projectTrajectoryTime(const robot_trajectory::RobotTrajectory& trajectory,
    const Eigen::VectorXd& positions,double first,double last)
{
  const auto* group=trajectory.getGroup();
  if(trajectory.empty()||!group||positions.size()!=static_cast<Eigen::Index>(group->getVariableCount())||
      !positions.allFinite()||!std::isfinite(first)||!std::isfinite(last)||first<0||last<first)
    throw std::invalid_argument("invalid trajectory projection");
  last=std::min(last,trajectory.getDuration());first=std::min(first,last);
  TrajectoryProjection best{first,std::numeric_limits<double>::infinity()};
  for(size_t i=1;i<trajectory.getWayPointCount();++i)
  {
    const double a=trajectory.getWayPointDurationFromStart(i-1),b=trajectory.getWayPointDurationFromStart(i);
    if(b<first||a>last||b<=a)continue;
    Eigen::VectorXd x,y;trajectory.getWayPoint(i-1).copyJointGroupPositions(group,x);
    trajectory.getWayPoint(i).copyJointGroupPositions(group,y);
    const Eigen::VectorXd delta=y-x;
    const double lower=std::clamp((first-a)/(b-a),0.,1.),upper=std::clamp((last-a)/(b-a),0.,1.);
    const double fraction=delta.squaredNorm()>1e-16 ?
        std::clamp((positions-x).dot(delta)/delta.squaredNorm(),lower,upper) : upper;
    const double distance=(positions-x-fraction*delta).norm(),time=a+fraction*(b-a);
    if(distance<best.distance-1e-10||(std::abs(distance-best.distance)<=1e-10&&time>best.time))best={time,distance};
  }
  if(!std::isfinite(best.distance))
  {
    Eigen::VectorXd q;trajectory.getFirstWayPoint().copyJointGroupPositions(group,q);
    best.distance=(positions-q).norm();
  }
  return best;
}
// Handle exact internal waypoints explicitly: accumulated durations can differ
// from k*dt by rounding, producing interpolation ratios slightly above one.
inline void sampleTrajectory(const robot_trajectory::RobotTrajectory& trajectory, double t,
                             moveit::core::RobotState& output)
{
  if(trajectory.empty()||!std::isfinite(t))throw std::invalid_argument("invalid trajectory sample");
  if(t<=0){output=trajectory.getFirstWayPoint();return;}
  if(t>=trajectory.getDuration()-1e-9){output=trajectory.getLastWayPoint();return;}
  size_t lower=0,upper=trajectory.getWayPointCount()-1;
  while(upper-lower>1)
  {
    const size_t mid=(lower+upper)/2;
    if(trajectory.getWayPointDurationFromStart(mid)<t)lower=mid;else upper=mid;
  }
  const double a=trajectory.getWayPointDurationFromStart(lower),b=trajectory.getWayPointDurationFromStart(upper);
  if(std::abs(t-a)<1e-9){output=trajectory.getWayPoint(lower);return;}
  if(std::abs(t-b)<1e-9){output=trajectory.getWayPoint(upper);return;}
  const double fraction=std::clamp((t-a)/(b-a),0.0,1.0);
  trajectory.getWayPoint(lower).interpolate(trajectory.getWayPoint(upper),fraction,output);
  for(const auto* joint:trajectory.getRobotModel()->getActiveJointModels())
    if(joint->getVariableCount()==1&&!joint->getVariableBounds().front().position_bounded_)
    {
      const double x=*trajectory.getWayPoint(lower).getJointPositions(joint);
      const double y=*trajectory.getWayPoint(upper).getJointPositions(joint);
      const double value=x+fraction*(y-x);output.setJointPositions(joint,&value);
    }
  output.update();
}
}
