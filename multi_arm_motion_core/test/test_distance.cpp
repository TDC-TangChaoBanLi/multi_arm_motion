#include <gtest/gtest.h>
#include <multi_arm_motion_core/distance.hpp>
#include <geometric_shapes/shapes.h>
#include <multi_arm_motion_core/prediction.hpp>
#include <multi_arm_motion_core/trajectory.hpp>
#include <multi_arm_motion_core/command.hpp>
#include <multi_arm_motion_core/constraints.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

TEST(ShapeBoundsCache, ReusesImmutableGeometryAndHandlesNewShapes)
{
  multi_arm_motion::ShapeBoundsCache cache;
  shapes::ShapeConstPtr first=std::make_shared<shapes::Sphere>(.1);
  const auto a=cache.get(first),again=cache.get(first);
  EXPECT_TRUE(a.first.isApprox(again.first));
  EXPECT_DOUBLE_EQ(a.second,.1);
  EXPECT_DOUBLE_EQ(a.second,again.second);
  first.reset();
  shapes::ShapeConstPtr second=std::make_shared<shapes::Sphere>(.3);
  EXPECT_DOUBLE_EQ(cache.get(second).second,.3);
}

planning_scene::PlanningScenePtr scene()
{
  const std::string xml=R"(<robot name="sliders"><link name="base"/>
    <link name="first"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
    <link name="second"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
    <joint name="one" type="prismatic"><parent link="base"/><child link="first"/>
      <axis xyz="1 0 0"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    <joint name="two" type="prismatic"><parent link="base"/><child link="second"/>
      <origin xyz="0.4 0 0"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    </robot>)";
  auto urdf=urdf::parseURDF(xml);auto srdf=std::make_shared<srdf::Model>();
  srdf->initString(*urdf,R"(<robot name="sliders"><group name="first_group"><joint name="one"/></group>
    <group name="both"><joint name="one"/><joint name="two"/></group></robot>)");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto out=std::make_shared<planning_scene::PlanningScene>(model);
  out->getCurrentStateNonConst().setToDefaultValues();out->getCurrentStateNonConst().update();return out;
}
TEST(Distance, IncludesOtherArmAndHonorsAcm)
{
  const auto s=scene();auto state=s->getCurrentState();
  EXPECT_NEAR(multi_arm_motion::minimumDistance(*s,state,"first_group"),0.2,1e-5);
  state.setVariablePosition("two",-0.3);state.update();
  EXPECT_EQ(multi_arm_motion::minimumDistance(*s,state,"first_group"),0);
  s->getAllowedCollisionMatrixNonConst().setEntry("first","second",true);
  EXPECT_GT(multi_arm_motion::minimumDistance(*s,state,"first_group"),1);
}
TEST(Distance, IncludesAttachedGeometry)
{
  const auto s=scene();auto state=s->getCurrentState();
  auto pose=Eigen::Isometry3d::Identity();pose.translation().x()=0.25;
  state.attachBody("payload",pose,{std::make_shared<shapes::Sphere>(0.1)},
      {Eigen::Isometry3d::Identity()},std::set<std::string>{"first"},"first");state.update();
  EXPECT_EQ(multi_arm_motion::minimumDistance(*s,state,"first_group"),0);
}
TEST(Distance, ClosestPointGradientMatchesPerturbedGeometryForAggregateGroup)
{
  const auto s=scene();auto state=s->getCurrentState();
  const auto result=multi_arm_motion::minimumDistanceGradient(*s,state,"both",.3);
  EXPECT_NEAR(result.first,.2,1e-5);ASSERT_EQ(result.second.size(),2);
  EXPECT_NEAR(result.second[0],-1,1e-5);EXPECT_NEAR(result.second[1],1,1e-5);
  for(int j=0;j<2;++j)
  {
    const auto& name=s->getRobotModel()->getJointModelGroup("both")->getVariableNames()[j];
    auto plus=state,minus=state;plus.setVariablePosition(name,1e-5);minus.setVariablePosition(name,-1e-5);plus.update();minus.update();
    const double numeric=(multi_arm_motion::minimumDistance(*s,plus,"both")-multi_arm_motion::minimumDistance(*s,minus,"both"))/2e-5;
    EXPECT_NEAR(result.second[j],numeric,1e-5);
  }
}
TEST(Distance, ClosestPointGradientIncludesPayloadAttachment)
{
  auto s=scene();auto state=s->getCurrentState();
  auto pose=Eigen::Isometry3d::Identity();pose.translation().x()=.15;
  state.attachBody("payload",pose,{std::make_shared<shapes::Sphere>(.05)},
      {Eigen::Isometry3d::Identity()},std::set<std::string>{"first"},"first");state.update();
  const auto result=multi_arm_motion::minimumDistanceGradient(*s,state,"both",.3);
  EXPECT_NEAR(result.first,.1,1e-5);
  EXPECT_NEAR(result.second[0],-1,1e-5);EXPECT_NEAR(result.second[1],1,1e-5);
}
TEST(Distance, MotionBoundNeverOverestimatesActualSeparation)
{
  auto s=scene();auto state=s->getCurrentState();
  multi_arm_motion::DistanceBound bound(*s,state,"both",.025);
  for(double q:{.01,.02,.05,.1,.15,.2,.3,-.1,-.5})
  {
    state.setVariablePosition("one",q);state.update();
    const double conservative=bound.distance(state),actual=multi_arm_motion::minimumDistance(*s,state,"both");
    EXPECT_LE(conservative,actual+1e-8);
    if(actual<.025)EXPECT_LT(conservative,.025);
  }
}
TEST(Distance, PairBoundsHandleCommonMotionAndAttachedWorldApproach)
{
  auto s=scene();auto state=s->getCurrentState();
  state.setVariablePosition("two",-.194);state.update();
  multi_arm_motion::ShapeBoundsCache cache;
  multi_arm_motion::DistanceBound bound(*s,state,"both",.005,&cache);
  for(double shift:{.001,.003,.007,.02,.05,.1})
  {
    state.setVariablePosition("one",shift);state.setVariablePosition("two",shift-.194);state.update();
    const double conservative=bound.distance(state),actual=multi_arm_motion::minimumDistance(*s,state,"both");
    EXPECT_LE(conservative,actual+1e-8);EXPECT_GE(conservative,.005);
  }
  auto payload_pose=Eigen::Isometry3d::Identity();payload_pose.translation().x()=.15;
  payload_pose.translation().y()=.3;
  state.attachBody("payload",payload_pose,{std::make_shared<shapes::Sphere>(.03)},
      {Eigen::Isometry3d::Identity()},std::set<std::string>{"first"},"first");state.update();
  auto world_pose=Eigen::Isometry3d::Identity();world_pose.translation().x()=.6;
  world_pose.translation().y()=.3;
  s->getWorldNonConst()->addToObject("obstacle",std::make_shared<shapes::Sphere>(.04),world_pose);
  multi_arm_motion::DistanceBound attached(*s,state,"both",.005,&cache);
  for(double shift:{.12,.16,.2,.25,.3,.4})
  {
    state.setVariablePosition("one",shift);state.setVariablePosition("two",shift-.194);state.update();
    const double conservative=attached.distance(state),actual=multi_arm_motion::minimumDistance(*s,state,"both");
    EXPECT_LE(conservative,actual+1e-8);
    if(actual<.005)EXPECT_LT(conservative,.005);
  }
}
TEST(Distance, FuturePredictionChangesCollisionState)
{
  const auto s=scene();auto state=s->getCurrentState();
  multi_arm_motion::Prediction p;p.source="second_controller";p.version=1;
  p.trajectory.header.stamp.sec=10;p.valid_until.sec=11;p.trajectory.joint_names={"two"};
  trajectory_msgs::msg::JointTrajectoryPoint a,b;a.positions={0};b.positions={-0.3};b.time_from_start.sec=1;
  p.trajectory.points={a,b};multi_arm_motion::validatePrediction(p);
  multi_arm_motion::applyPredictions(state,{p},{"one"},rclcpp::Time(10,0,RCL_ROS_TIME));
  EXPECT_GT(multi_arm_motion::minimumDistance(*s,state,"first_group"),0);
  multi_arm_motion::applyPredictions(state,{p},{"one"},rclcpp::Time(11,0,RCL_ROS_TIME));
  EXPECT_EQ(multi_arm_motion::minimumDistance(*s,state,"first_group"),0);
  EXPECT_THROW(multi_arm_motion::applyPredictions(state,{p,p},{"one"},rclcpp::Time(10,0,RCL_ROS_TIME)),std::runtime_error);
}
TEST(TrajectorySampling, HandlesFloatingPointAtInternalWaypoint)
{
  const auto s=scene();auto state=s->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(s->getRobotModel(),"both");
  trajectory.addSuffixWayPoint(state,0);
  for(int i=1;i<=5;++i)
  {state.setVariablePosition("one",i*0.1);state.update();trajectory.addSuffixWayPoint(state,0.1);}
  multi_arm_motion::sampleTrajectory(trajectory,0.3+1e-15,state);
  EXPECT_NEAR(state.getVariablePosition("one"),0.3,1e-12);
  multi_arm_motion::sampleTrajectory(trajectory,0.35,state);
  EXPECT_NEAR(state.getVariablePosition("one"),0.35,1e-12);
}
TEST(TrajectorySampling, ContinuousCoordinatesStayUnwrapped)
{
  auto urdf=urdf::parseURDF(R"(<robot name="continuous"><link name="base"/><link name="tip"/>
    <joint name="turn" type="continuous"><parent link="base"/><child link="tip"/>
    <axis xyz="0 0 1"/><limit velocity="1" effort="1"/></joint></robot>)");
  auto srdf=std::make_shared<srdf::Model>();
  srdf->initString(*urdf,R"(<robot name="continuous"><group name="arm"><joint name="turn"/></group></robot>)");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);moveit::core::RobotState state(model);
  state.setToDefaultValues();state.setVariablePosition("turn",3.1);state.update();
  robot_trajectory::RobotTrajectory trajectory(model,"arm");trajectory.addSuffixWayPoint(state,0);
  state.setVariablePosition("turn",3.3);state.update();trajectory.addSuffixWayPoint(state,1);
  multi_arm_motion::sampleTrajectory(trajectory,.5,state);
  EXPECT_NEAR(state.getVariablePosition("turn"),3.2,1e-10);
}
TEST(TrajectorySampling, ProjectionPreservesProgressAcrossRepeatedPositions)
{
  const auto s=scene();auto state=s->getCurrentState();
  robot_trajectory::RobotTrajectory trajectory(s->getRobotModel(),"both");
  trajectory.addSuffixWayPoint(state,0);
  state.setVariablePosition("one",1);state.update();trajectory.addSuffixWayPoint(state,1);
  state.setVariablePosition("one",0);state.update();trajectory.addSuffixWayPoint(state,1);
  Eigen::VectorXd observed(2);observed<<.2,0;
  const auto early=multi_arm_motion::projectTrajectoryTime(trajectory,observed,0,1);
  EXPECT_NEAR(early.time,.2,1e-10);EXPECT_NEAR(early.distance,0,1e-10);
  const auto late=multi_arm_motion::projectTrajectoryTime(trajectory,observed,1,2);
  EXPECT_NEAR(late.time,1.8,1e-10);EXPECT_NEAR(late.distance,0,1e-10);
  EXPECT_THROW(multi_arm_motion::projectTrajectoryTime(trajectory,observed,2,1),std::invalid_argument);
}
TEST(CommandSampling, MatchesConstantAccelerationControllerPolynomial)
{
  trajectory_msgs::msg::JointTrajectory command;command.header.stamp.sec=10;command.joint_names={"joint"};
  trajectory_msgs::msg::JointTrajectoryPoint first,last;first.positions={0};first.velocities={0};
  last.positions={.5};last.velocities={1};last.time_from_start.sec=1;command.points={first,last};
  const auto sample=multi_arm_motion::sampleCommand(command,rclcpp::Time(10,500000000,RCL_ROS_TIME));
  EXPECT_NEAR(sample.first[0],.125,1e-10);EXPECT_NEAR(sample.second[0],.5,1e-10);
}
TEST(PathConstraint, NormalHorizontalLeavesInPlaneRotationFree)
{
  auto s=scene();moveit_msgs::msg::Constraints message;
  message.name="link_normal_horizontal 1 0 0 0.05";
  moveit_msgs::msg::OrientationConstraint orientation;orientation.link_name="first";message.orientation_constraints={orientation};
  multi_arm_motion::PathConstraints horizontal(*s);horizontal.add(message);
  EXPECT_TRUE(horizontal.satisfied(s->getCurrentState()));
  message.name="link_normal_horizontal 0 0 1 0.05";
  multi_arm_motion::PathConstraints vertical(*s);vertical.add(message);
  EXPECT_FALSE(vertical.satisfied(s->getCurrentState()));
  message.orientation_constraints.front().link_name="missing";
  EXPECT_THROW(vertical.add(message),std::invalid_argument);
}
