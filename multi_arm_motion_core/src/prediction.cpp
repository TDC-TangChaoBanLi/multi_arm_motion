#include <multi_arm_motion_core/prediction.hpp>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace multi_arm_motion
{
namespace
{
double seconds(const builtin_interfaces::msg::Duration& t)
{ return double(t.sec) + double(t.nanosec) * 1e-9; }
}
void validatePrediction(const Prediction& p)
{
  const auto& tr = p.trajectory;
  if (p.source.empty() || tr.joint_names.empty() || tr.points.empty() ||
      !std::isfinite(p.position_uncertainty) || p.position_uncertainty < 0)
    throw std::invalid_argument("invalid prediction metadata");
  std::set<std::string> names;
  for (const auto& name : tr.joint_names)
    if (name.empty() || !names.insert(name).second) throw std::invalid_argument("duplicate/empty joint");
  double previous = -1;
  for (const auto& point : tr.points)
  {
    const double t = seconds(point.time_from_start);
    if (point.time_from_start.nanosec >= 1000000000 || t < 0 || t <= previous ||
        point.positions.size() != names.size() ||
        (!point.velocities.empty()&&point.velocities.size()!=names.size())) throw std::invalid_argument("invalid prediction samples");
    for (double q : point.positions)
      if (!std::isfinite(q)) throw std::invalid_argument("nonfinite prediction");
    for(double v:point.velocities)if(!std::isfinite(v))throw std::invalid_argument("nonfinite prediction velocity");
    previous = t;
  }
  if (seconds(tr.points.front().time_from_start) != 0)
    throw std::invalid_argument("prediction must include its initial state at t=0");
  if (rclcpp::Time(p.valid_until) < rclcpp::Time(tr.header.stamp) + rclcpp::Duration::from_seconds(previous))
    throw std::invalid_argument("validity ends before prediction horizon");
}
std::vector<double> samplePrediction(const Prediction& p, const rclcpp::Time& at)
{
  if(p.trajectory.points.empty()||p.trajectory.joint_names.empty())throw std::invalid_argument("empty motion prediction");
  const double t = (at - rclcpp::Time(p.trajectory.header.stamp, at.get_clock_type())).seconds();
  if (t < 0 || at > rclcpp::Time(p.valid_until, at.get_clock_type()))
    throw std::runtime_error("prediction outside validity interval: " + p.source);
  const auto& points = p.trajectory.points;
  if (t >= seconds(points.back().time_from_start)) return points.back().positions;
  const auto end = std::upper_bound(points.begin(),points.end(),t,
      [](double value,const auto& point){return value < seconds(point.time_from_start);});
  const auto begin = std::prev(end);
  const double a = (t-seconds(begin->time_from_start)) /
      (seconds(end->time_from_start)-seconds(begin->time_from_start));
  auto q=begin->positions;
  const double dt=seconds(end->time_from_start)-seconds(begin->time_from_start);
  for(size_t j=0;j<q.size();++j)
    if(begin->velocities.size()==q.size()&&end->velocities.size()==q.size())
      q[j]=(2*a*a*a-3*a*a+1)*begin->positions[j]+(a*a*a-2*a*a+a)*dt*begin->velocities[j]+
          (-2*a*a*a+3*a*a)*end->positions[j]+(a*a*a-a*a)*dt*end->velocities[j];
    else q[j]+=a*(end->positions[j]-q[j]);
  return q;
}
bool PredictionStore::update(const Prediction& p, std::string& error)
{
  try { validatePrediction(p); }
  catch (const std::exception& e) { error = e.what(); return false; }
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = predictions_.find(p.source);
  if (it != predictions_.end() && p.version <= it->second.version)
  { error = "prediction version is not increasing"; return false; }
  predictions_[p.source] = p;
  ++revision_;
  return true;
}
std::vector<Prediction> PredictionStore::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Prediction> out;
  for (const auto& entry : predictions_) out.push_back(entry.second);
  return out;
}
void applyPredictions(moveit::core::RobotState& state, const std::vector<Prediction>& predictions,
                      const std::vector<std::string>& controlled, const rclcpp::Time& at)
{
  std::set<std::string> assigned;
  for (const auto& prediction : predictions)
  {
    bool needed = false;
    for (const auto& name : prediction.trajectory.joint_names)
      needed |= std::find(controlled.begin(), controlled.end(), name) == controlled.end();
    if (!needed) continue;
    if(prediction.position_uncertainty!=0)throw std::runtime_error("nonzero prediction uncertainty unsupported: "+prediction.source);
    const auto values = samplePrediction(prediction, at);
    for (size_t i = 0; i < values.size(); ++i)
    {
      const auto& name = prediction.trajectory.joint_names[i];
      if (std::find(controlled.begin(), controlled.end(), name) != controlled.end()) continue;
      const auto& known = state.getRobotModel()->getVariableNames();
      if (std::find(known.begin(), known.end(), name) == known.end() || !assigned.insert(name).second)
        throw std::runtime_error("unknown or multiply owned prediction joint: " + name);
      const auto& bounds=state.getRobotModel()->getVariableBounds(name);
      if(bounds.position_bounded_&&(values[i]<bounds.min_position_||values[i]>bounds.max_position_))
        throw std::runtime_error("prediction exceeds joint bounds: "+name);
      state.setVariablePosition(name, values[i]);
    }
  }
  state.update();
}
}
