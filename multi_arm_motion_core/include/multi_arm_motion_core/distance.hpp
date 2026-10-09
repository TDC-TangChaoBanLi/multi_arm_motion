#pragma once
#include <moveit/planning_scene/planning_scene.hpp>
#include <limits>
#include <mutex>
#include <map>
namespace multi_arm_motion
{
// Includes ACM, world objects, self collision and attached bodies; metres.
double minimumDistance(const planning_scene::PlanningScene& scene, const moveit::core::RobotState& state,
                       const std::string& group = "",double threshold=std::numeric_limits<double>::max());
// One closest-point query, then FK point derivatives. Works for aggregate groups
// and attached payloads without requiring a serial-chain group Jacobian.
std::pair<double,Eigen::VectorXd> minimumDistanceGradient(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& state,const std::string& group,double threshold);
// Collision shapes are immutable. Cache their bounding spheres across control
// cycles; scene padding, transforms and attachments are still evaluated live.
class ShapeBoundsCache
{
public:
  std::pair<Eigen::Vector3d,double> get(const shapes::ShapeConstPtr&);
private:
  struct Entry {std::weak_ptr<const shapes::Shape> shape;Eigen::Vector3d center;double radius;};
  std::mutex mutex_;
  std::map<const shapes::Shape*,Entry> entries_;
};
// Conservative distance lower bound between exact queries. Every robot shape,
// including payloads, is enclosed about its parent link origin. Relative point
// displacement bounds how much the minimum separation can decrease.
class DistanceBound
{
public:
  DistanceBound(const planning_scene::PlanningScene&,const moveit::core::RobotState&,
      std::string group,double clearance,ShapeBoundsCache* cache=nullptr);
  double distance(const moveit::core::RobotState&);
private:
  const planning_scene::PlanningScene& scene_;
  std::string group_;double clearance_,anchor_distance_;
  struct LinkBound {const moveit::core::LinkModel* link;double radius;Eigen::Isometry3d anchor;};
  std::vector<LinkBound> links_;
  struct PairBound {
    const moveit::core::LinkModel* first;const moveit::core::LinkModel* second;
    double first_radius,second_radius,distance;Eigen::Isometry3d relative;
  };
  std::vector<PairBound> pairs_;
  bool pair_bounds_{false};
  void anchor(const moveit::core::RobotState&);
};
}
