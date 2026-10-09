#pragma once
#include <multi_arm_motion_core/distance.hpp>
#include <multi_arm_motion_core/prediction.hpp>
#include <chrono>
#include <algorithm>

namespace stomp_moveit
{
// Repair small positive clearance deficits in an optimization seed. This is
// only an initial guess: STOMP and the consumer must still validate the entire
// path. Colliding states with no reliable distance gradient are left to STOMP.
inline bool conditionSeedClearance(const planning_scene::PlanningScene& scene,
    const moveit::core::JointModelGroup* group,
    const std::vector<multi_arm_motion::Prediction>& predictions,
    const rclcpp::Time& origin, double dt, double clearance, Eigen::MatrixXd& seed,
    std::chrono::steady_clock::time_point deadline)
{
  if(clearance<=0||seed.cols()<3)return false;
  auto state=scene.getCurrentState();
  const auto& names=group->getVariableNames();
  if(seed.rows()!=static_cast<Eigen::Index>(names.size()))return false;
  const double desired=clearance*1.25;
  bool changed=false;
  for(int pass=0;pass<3;++pass)
    for(Eigen::Index i=0;i+1<seed.cols();++i)
    {
      const int count=std::max(2,int(std::ceil((seed.col(i+1)-seed.col(i)).norm()/.025)));
      for(int k=0;k<count;++k)
      {
      if(std::chrono::steady_clock::now()>=deadline)return changed;
      const double fraction=double(k)/count;
      const double left=i==0?0.:1-fraction,right=i+2==seed.cols()?0.:fraction;
      const double freedom=left*left+right*right;
      if(freedom<1e-12)continue;
      multi_arm_motion::applyPredictions(state,predictions,names,
          origin+rclcpp::Duration::from_seconds((i+fraction)*dt));
      state.setJointGroupPositions(group,Eigen::VectorXd((1-fraction)*seed.col(i)+fraction*seed.col(i+1)));state.update();
      const auto [distance,gradient]=multi_arm_motion::minimumDistanceGradient(scene,state,group->getName(),desired);
      if(!std::isfinite(distance)||distance<=0||distance>=desired||
          !gradient.allFinite()||gradient.squaredNorm()<1e-12)continue;
      Eigen::VectorXd update=gradient*((desired-distance)/(gradient.squaredNorm()*freedom));
      const double max_change=update.cwiseAbs().maxCoeff();
      if(max_change>.05)update*=.05/max_change;
      for(int trial=0;trial<6;++trial)
      {
        if(std::chrono::steady_clock::now()>=deadline)return changed;
        Eigen::VectorXd a=seed.col(i)+left*update,b=seed.col(i+1)+right*update;
        for(Eigen::Index j=0;j<a.size();++j)
        {
          const auto& bounds=scene.getRobotModel()->getVariableBounds(names[j]);
          if(bounds.position_bounded_)
          {
            a[j]=std::clamp(a[j],bounds.min_position_,bounds.max_position_);
            b[j]=std::clamp(b[j],bounds.min_position_,bounds.max_position_);
          }
        }
        state.setJointGroupPositions(group,Eigen::VectorXd((1-fraction)*a+fraction*b));state.update();
        const double next=multi_arm_motion::minimumDistance(scene,state,group->getName(),desired);
        if(std::isfinite(next)&&next>distance+1e-7)
        {
          seed.col(i)=a;seed.col(i+1)=b;changed=true;break;
        }
        // An infinite thresholded result means all pairs exceed desired.
        if(std::isinf(next)&&next>0)
        {
          seed.col(i)=a;seed.col(i+1)=b;changed=true;break;
        }
        update*=.5;
      }
      }
    }
  return changed;
}
}
