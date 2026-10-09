#pragma once
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <utility>
namespace multi_arm_motion
{
// JTC position+velocity messages use cubic Hermite interpolation. Predicting
// this polynomial prevents resetting each replacement segment to old feedback.
inline std::pair<std::vector<double>,std::vector<double>> sampleCommand(
    const trajectory_msgs::msg::JointTrajectory& command,const rclcpp::Time& at)
{
  if(command.points.empty())throw std::invalid_argument("empty controller command");
  const double t=(at-rclcpp::Time(command.header.stamp,at.get_clock_type())).seconds();
  const auto& points=command.points;
  auto seconds=[](const auto& point){return rclcpp::Duration(point.time_from_start).seconds();};
  if(t<=seconds(points.front()))return {points.front().positions,points.front().velocities};
  if(t>=seconds(points.back()))return {points.back().positions,points.back().velocities};
  const auto b=std::upper_bound(points.begin(),points.end(),t,[&](double x,const auto& p){return x<seconds(p);});
  const auto a=std::prev(b);const double dt=seconds(*b)-seconds(*a),u=(t-seconds(*a))/dt;
  if(a->velocities.size()!=a->positions.size()||b->velocities.size()!=a->positions.size())
    throw std::invalid_argument("controller polynomial requires positions and velocities");
  auto q=a->positions,v=a->velocities;
  for(size_t j=0;j<q.size();++j)
  {
    q[j]=(2*u*u*u-3*u*u+1)*a->positions[j]+(u*u*u-2*u*u+u)*dt*a->velocities[j]+
         (-2*u*u*u+3*u*u)*b->positions[j]+(u*u*u-u*u)*dt*b->velocities[j];
    v[j]=(6*u*u-6*u)/dt*a->positions[j]+(3*u*u-4*u+1)*a->velocities[j]+
         (-6*u*u+6*u)/dt*b->positions[j]+(3*u*u-2*u)*b->velocities[j];
  }
  return {q,v};
}
}
