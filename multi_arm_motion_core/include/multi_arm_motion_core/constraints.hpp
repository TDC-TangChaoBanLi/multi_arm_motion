#pragma once
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematic_constraints/kinematic_constraint.hpp>
#include <sstream>
#include <stdexcept>
#include <limits>

namespace multi_arm_motion
{
// The same geometric acceptance rule is used at global commit, MPC horizon
// validation and braking. Normal-horizontal leaves rotation about the normal free.
class PathConstraints
{
public:
  explicit PathConstraints(const planning_scene::PlanningScene& scene):scene_(scene),standard_(scene.getRobotModel()){}
  void add(moveit_msgs::msg::Constraints message)
  {
    const std::string generic="link_normal_horizontal ",legacy="space_sim_mirror_vertical ";
    const auto marker=message.name.rfind(generic,0)==0?generic:legacy;
    if(message.name.rfind(marker,0)==0)
    {
      Normal constraint;std::istringstream input(message.name.substr(marker.size()));
      if(!(input>>constraint.normal.x()>>constraint.normal.y()>>constraint.normal.z()>>constraint.tolerance)||
         !constraint.normal.allFinite()||constraint.normal.norm()<1e-8||
         !std::isfinite(constraint.tolerance)||constraint.tolerance<=0||constraint.tolerance>=1||
         message.orientation_constraints.size()!=1)
        throw std::invalid_argument("invalid link-normal constraint");
      constraint.normal.normalize();constraint.link=message.orientation_constraints.front().link_name;
      if(!scene_.getRobotModel()->getLinkModel(constraint.link))throw std::invalid_argument("unknown constrained link");
      normals_.push_back(constraint);message.orientation_constraints.clear();
    }
    if(!message.joint_constraints.empty()||!message.position_constraints.empty()||
       !message.orientation_constraints.empty()||!message.visibility_constraints.empty())
      if(!standard_.add(message,scene_.getTransforms()))throw std::invalid_argument("invalid MoveIt path constraint");
  }
  bool satisfied(const moveit::core::RobotState& state) const
  {
    if(!standard_.decide(state).satisfied)return false;
    for(const auto& normal:normals_)
      if(std::abs((state.getGlobalLinkTransform(normal.link).linear()*normal.normal).z())>normal.tolerance)return false;
    return true;
  }
  bool empty() const {return standard_.empty()&&normals_.empty();}
  // Prefer the centre of a cylindrical path corridor while preserving motion
  // along its axis. This provides a smooth optimization signal before a hard
  // tolerance is reached. A normal-horizontal constraint penalizes only excess
  // tilt and leaves normal-axis rotation unpenalized. Hard acceptance still uses satisfied().
  double trackingCost(const moveit::core::RobotState& state) const
  {
    double cost=0;
    for(const auto& c:standard_.getPositionConstraints())
    {
      const Eigen::Vector3d offset(c.target_point_offset.x,c.target_point_offset.y,c.target_point_offset.z);
      const Eigen::Vector3d point=scene_.getFrameTransform(state,c.header.frame_id).inverse()*
          (state.getGlobalLinkTransform(c.link_name)*offset);
      // A union of regions is an alternative, not multiple simultaneous targets.
      double best=std::numeric_limits<double>::infinity();
      for(size_t i=0;i<c.constraint_region.primitives.size();++i)
      {
        const auto& shape=c.constraint_region.primitives[i];
        if(shape.type!=shape_msgs::msg::SolidPrimitive::CYLINDER||shape.dimensions.size()!=2||
            i>=c.constraint_region.primitive_poses.size())continue;
        const auto& p=c.constraint_region.primitive_poses[i];
        const Eigen::Quaterniond r(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
        const Eigen::Vector3d local=r.inverse()*(point-Eigen::Vector3d(p.position.x,p.position.y,p.position.z));
        const double axial=std::max(0.,std::abs(local.z())-.5*shape.dimensions[0]);
        best=std::min(best,local.head<2>().squaredNorm()+axial*axial);
      }
      if(std::isfinite(best))cost+=best;
    }
    for(const auto& normal:normals_)
    {
      const double tilt=std::abs((state.getGlobalLinkTransform(normal.link).linear()*normal.normal).z());
      const double excess=std::max(0.,tilt-normal.tolerance);
      cost+=.01*excess*excess;
    }
    return cost;
  }
  std::string describeViolation(const moveit::core::RobotState& state) const
  {
    std::ostringstream detail;
    std::vector<kinematic_constraints::ConstraintEvaluationResult> results;
    if(!standard_.decide(state,results).satisfied)
    {
      for(size_t i=0;i<results.size();++i)
        if(!results[i].satisfied)detail<<"MoveIt constraint #"<<i<<" measured distance="<<results[i].distance<<"; ";
      standard_.print(detail);
      for(const auto& c:standard_.getPositionConstraints())
      {
        const Eigen::Vector3d offset(c.target_point_offset.x,c.target_point_offset.y,c.target_point_offset.z);
        const Eigen::Vector3d point=state.getGlobalLinkTransform(c.link_name)*offset;
        for(size_t i=0;i<c.constraint_region.primitives.size();++i)
        {
          const auto& shape=c.constraint_region.primitives[i];
          if(shape.type!=shape_msgs::msg::SolidPrimitive::CYLINDER||shape.dimensions.size()!=2||
              i>=c.constraint_region.primitive_poses.size())continue;
          const auto& pose=c.constraint_region.primitive_poses[i];
          const Eigen::Quaterniond rotation(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
          const Eigen::Vector3d local=rotation.inverse()*(scene_.getFrameTransform(state,c.header.frame_id).inverse()*point-
              Eigen::Vector3d(pose.position.x,pose.position.y,pose.position.z));
          detail<<"cylinder link="<<c.link_name<<" radial="<<local.head<2>().norm()<<" / "<<shape.dimensions[1]
              <<" m, axial="<<local.z()<<" / +/-"<<shape.dimensions[0]*.5<<" m; ";
        }
      }
    }
    for(const auto& normal:normals_)
    {
      const double tilt=std::abs((state.getGlobalLinkTransform(normal.link).linear()*normal.normal).z());
      if(tilt>normal.tolerance)detail<<"normal constraint link="<<normal.link<<" measured="<<tilt<<" tolerance="<<normal.tolerance<<"; ";
    }
    return detail.str();
  }
private:
  struct Normal {Eigen::Vector3d normal;double tolerance;std::string link;};
  const planning_scene::PlanningScene& scene_;kinematic_constraints::KinematicConstraintSet standard_;std::vector<Normal> normals_;
};
}
