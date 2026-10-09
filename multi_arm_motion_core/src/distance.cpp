#include <multi_arm_motion_core/distance.hpp>
#include <algorithm>
#include <cmath>
#include <geometric_shapes/shape_operations.h>
namespace multi_arm_motion
{
double minimumDistance(const planning_scene::PlanningScene& scene, const moveit::core::RobotState& state,
                       const std::string& group,double threshold)
{
  collision_detection::DistanceRequest request;
  request.distance_threshold=threshold;
  request.type=collision_detection::DistanceRequestTypes::SINGLE;
  request.group_name = group;
  request.acm = &scene.getAllowedCollisionMatrix();
  request.enableGroup(scene.getRobotModel());
  collision_detection::DistanceResult self, world;
  scene.getCollisionEnv()->distanceSelf(request, self, state);
  scene.getCollisionEnv()->distanceRobot(request, world, state);
  return std::max(0.0,std::min(self.minimum_distance.distance, world.minimum_distance.distance));
}
std::pair<double,Eigen::VectorXd> minimumDistanceGradient(const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& state,const std::string& group,double threshold)
{
  const auto* joint_group=scene.getRobotModel()->getJointModelGroup(group);
  if(!joint_group)throw std::invalid_argument("unknown distance gradient group");
  const auto& names=joint_group->getVariableNames();Eigen::VectorXd gradient=Eigen::VectorXd::Zero(names.size());
  collision_detection::DistanceRequest request;request.group_name=group;request.acm=&scene.getAllowedCollisionMatrix();
  request.enableGroup(scene.getRobotModel());request.enable_nearest_points=true;request.distance_threshold=threshold;
  request.type=collision_detection::DistanceRequestTypes::SINGLE;
  collision_detection::DistanceResult self,world;
  scene.getCollisionEnv()->distanceSelf(request,self,state);scene.getCollisionEnv()->distanceRobot(request,world,state);
  const auto& closest=self.minimum_distance.distance<world.minimum_distance.distance?self.minimum_distance:world.minimum_distance;
  const double d=std::max(0.0,closest.distance);
  if(d<=0||d>=threshold||!std::isfinite(d))return {d,gradient};
  std::string links[2];Eigen::Vector3d local[2];
  for(int side=0;side<2;++side)
  {
    if(closest.body_types[side]==collision_detection::BodyType::ROBOT_LINK)links[side]=closest.link_names[side];
    else if(closest.body_types[side]==collision_detection::BodyType::ROBOT_ATTACHED)
    {
      const auto* attached=state.getAttachedBody(closest.link_names[side]);
      if(!attached)throw std::runtime_error("closest attached body missing from robot state");
      links[side]=attached->getAttachedLinkName();
    }
    if(!links[side].empty())local[side]=state.getGlobalLinkTransform(links[side]).inverse()*closest.nearest_points[side];
  }
  const Eigen::Vector3d normal=(closest.nearest_points[0]-closest.nearest_points[1]).normalized();
  if(!normal.allFinite())return {d,gradient};
  auto plus=state,minus=state;constexpr double epsilon=1e-6;
  for(size_t j=0;j<names.size();++j)
  {
    const double q=state.getVariablePosition(names[j]);
    plus.setVariablePosition(names[j],q+epsilon);minus.setVariablePosition(names[j],q-epsilon);plus.update();minus.update();
    Eigen::Vector3d derivative=Eigen::Vector3d::Zero();
    for(int side=0;side<2;++side)if(!links[side].empty())
      derivative+=(side==0?1.0:-1.0)*((plus.getGlobalLinkTransform(links[side])*local[side])-
          (minus.getGlobalLinkTransform(links[side])*local[side]))/(2*epsilon);
    gradient[j]=normal.dot(derivative);
    plus.setVariablePosition(names[j],q);minus.setVariablePosition(names[j],q);
  }
  return {d,gradient};
}
std::pair<Eigen::Vector3d,double> ShapeBoundsCache::get(const shapes::ShapeConstPtr& shape)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it=entries_.find(shape.get());
    if(it!=entries_.end()&&!it->second.shape.expired())return {it->second.center,it->second.radius};
  }
  Eigen::Vector3d center;double radius;shapes::computeShapeBoundingSphere(shape.get(),center,radius);
  std::lock_guard<std::mutex> lock(mutex_);
  entries_[shape.get()]={shape,center,radius};return {center,radius};
}
DistanceBound::DistanceBound(const planning_scene::PlanningScene& scene,const moveit::core::RobotState& state,
    std::string group,double clearance,ShapeBoundsCache* cache):scene_(scene),group_(std::move(group)),clearance_(clearance)
{
  std::map<const moveit::core::LinkModel*,double> radii;
  auto include=[&](const moveit::core::LinkModel* link,const shapes::ShapeConstPtr& shape,const Eigen::Isometry3d& local){
    if(shape->type==shapes::PLANE||shape->type==shapes::OCTREE)
      throw std::runtime_error("robot plane/octree shape cannot use finite motion bounds");
    Eigen::Vector3d center;double radius;
    if(cache){const auto value=cache->get(shape);center=value.first;radius=value.second;}
    else shapes::computeShapeBoundingSphere(shape.get(),center,radius);
    if(!center.allFinite()||!std::isfinite(radius)||radius<0)throw std::runtime_error("unbounded collision shape cannot use distance cache");
    const double padding=std::abs(scene.getCollisionEnv()->getLinkPadding(link->getName()));
    const double scale=std::max(1.0,scene.getCollisionEnv()->getLinkScale(link->getName()));
    const double bound=((local*center).norm()+radius)*scale+padding;
    radii[link]=std::max(radii[link],bound);
  };
  for(const auto* link:scene.getRobotModel()->getLinkModelsWithCollisionGeometry())
    for(size_t i=0;i<link->getShapes().size();++i)include(link,link->getShapes()[i],link->getCollisionOriginTransforms()[i]);
  std::vector<const moveit::core::AttachedBody*> attached;state.getAttachedBodies(attached);
  for(const auto* body:attached)
    for(size_t i=0;i<body->getShapes().size();++i)
      include(body->getAttachedLink(),body->getShapes()[i],state.getGlobalLinkTransform(body->getAttachedLink()).inverse()*body->getGlobalCollisionBodyTransforms()[i]);
  for(const auto& entry:radii)links_.push_back({entry.first,entry.second,Eigen::Isometry3d::Identity()});
  anchor(state);
}
void DistanceBound::anchor(const moveit::core::RobotState& state)
{
  for(auto& link:links_)link.anchor=state.getGlobalLinkTransform(link.link);
  const double threshold=clearance_+.02;
  collision_detection::DistanceRequest request;request.group_name=group_;request.enableGroup(scene_.getRobotModel());
  request.acm=&scene_.getAllowedCollisionMatrix();request.distance_threshold=threshold;
  request.type=collision_detection::DistanceRequestTypes::SINGLE;
  collision_detection::DistanceResult self,world;
  scene_.getCollisionEnv()->distanceSelf(request,self,state);
  scene_.getCollisionEnv()->distanceRobot(request,world,state);
  anchor_distance_=std::max(0.0,std::min(threshold,std::min(self.minimum_distance.distance,world.minimum_distance.distance)));
  pairs_.clear();
  auto transform=[&](const moveit::core::LinkModel* link)->Eigen::Isometry3d {
    return link?state.getGlobalLinkTransform(link):Eigen::Isometry3d::Identity();};
  for(const auto* result:{&self,&world})for(const auto& entry:result->distances)for(const auto& pair:entry.second)
  {
    const moveit::core::LinkModel* link[2]={nullptr,nullptr};double radius[2]={0,0};
    for(int side=0;side<2;++side)
    {
      if(pair.body_types[side]==collision_detection::BodyType::ROBOT_LINK)
        link[side]=state.getRobotModel()->getLinkModel(pair.link_names[side]);
      else if(pair.body_types[side]==collision_detection::BodyType::ROBOT_ATTACHED)
      {
        const auto* body=state.getAttachedBody(pair.link_names[side]);
        if(!body)throw std::runtime_error("distance cache attached body unavailable");
        link[side]=body->getAttachedLink();
      }
      if(link[side])
      {
        const auto found=std::find_if(links_.begin(),links_.end(),[&](const auto& b){return b.link==link[side];});
        if(found==links_.end())throw std::runtime_error("distance cache shape bound unavailable");
        radius[side]=found->radius;
      }
    }
    // For a world object, use the robot side's world displacement. Its own
    // enclosing radius is unknown, so disable the opposite-frame bound.
    if(!link[0])radius[0]=std::numeric_limits<double>::infinity();
    if(!link[1])radius[1]=std::numeric_limits<double>::infinity();
    pairs_.push_back({link[0],link[1],radius[0],radius[1],std::max(0.0,pair.distance),
        transform(link[0]).inverse()*transform(link[1])});
  }
  // Fall back conservatively if a collision backend omits per-pair records.
  pair_bounds_=!pairs_.empty()||anchor_distance_>=threshold;
}
double DistanceBound::distance(const moveit::core::RobotState& state)
{
  double movement=0;
  for(const auto& link:links_)
  {
    const auto& transform=state.getGlobalLinkTransform(link.link);
    const double angle=Eigen::AngleAxisd(link.anchor.linear().transpose()*transform.linear()).angle();
    movement=std::max(movement,(transform.translation()-link.anchor.translation()).norm()+2*link.radius*std::sin(.5*angle));
  }
  double lower=(pair_bounds_?clearance_+.02:anchor_distance_)-2*movement-1e-9;
  if(pair_bounds_)for(const auto& pair:pairs_)
  {
    const Eigen::Isometry3d first=pair.first?state.getGlobalLinkTransform(pair.first):Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d second=pair.second?state.getGlobalLinkTransform(pair.second):Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d relative=first.inverse()*second;
    auto displacement=[](const Eigen::Isometry3d& before,const Eigen::Isometry3d& after,double radius){
      if(!std::isfinite(radius))return std::numeric_limits<double>::infinity();
      const double angle=Eigen::AngleAxisd(before.linear().transpose()*after.linear()).angle();
      return (before.translation()-after.translation()).norm()+2*radius*std::sin(.5*angle);};
    const double change=std::min(displacement(pair.relative,relative,pair.second_radius),
        displacement(pair.relative.inverse(),relative.inverse(),pair.first_radius));
    lower=std::min(lower,pair.distance-change-1e-9);
  }
  if(lower>=clearance_)return lower;
  anchor(state);return anchor_distance_;
}
}
