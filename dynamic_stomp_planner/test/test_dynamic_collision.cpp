#include <gtest/gtest.h>
#include <stomp_moveit/dynamic_collision_cost.hpp>
#include <stomp_moveit/filter_functions.hpp>
#include <stomp_moveit/seed_conditioning.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

TEST(DynamicStomp, DetectsThinObstacleBetweenFormerStaticSamples)
{
  auto urdf=urdf::parseURDF(R"(<robot name="thin"><link name="base"/>
    <link name="tip"><collision><geometry><sphere radius="0.001"/></geometry></collision></link>
    <joint name="slide" type="prismatic"><parent link="base"/><child link="tip"/>
      <axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint></robot>)");
  auto srdf=std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf->initString(*urdf,
      R"(<robot name="thin"><group name="arm"><joint name="slide"/></group></robot>)"));
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  scene->getCurrentStateNonConst().setToDefaultValues();scene->getCurrentStateNonConst().update();
  scene->getWorldNonConst()->addToObject("thin_obstacle",shapes::ShapeConstPtr(new shapes::Sphere(.001)),
      Eigen::Translation3d(.005,0.,0.)*Eigen::Quaterniond::Identity());
  Eigen::MatrixXd values(1,2);values << 0., .05;
  Eigen::VectorXd cost;bool valid=true;
  auto check=stomp_moveit::costs::dynamicCollisionCost(scene,model->getJointModelGroup("arm"),{},
      rclcpp::Time(10,0,RCL_ROS_TIME),100.,.001);
  ASSERT_TRUE(check(values,cost,valid));EXPECT_FALSE(valid);EXPECT_GT(cost.sum(),0.);
}

TEST(DynamicStomp, UpdateFilterPreservesEndpointsAndBoundsAbsoluteCoordinates)
{
  auto urdf=urdf::parseURDF(R"(<robot name="joint"><link name="base"/><link name="tip"/>
    <joint name="j" type="prismatic"><parent link="base"/><child link="tip"/>
    <limit lower="1" upper="2" velocity="1" effort="1"/></joint></robot>)");
  auto srdf=std::make_shared<srdf::Model>();srdf->initString(*urdf,
      R"(<robot name="joint"><group name="arm"><joint name="j"/></group></robot>)");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  const auto filter=stomp_moveit::filters::smoothBoundedUpdates(model->getJointModelGroup("arm"),10);
  const auto parameters=Eigen::MatrixXd::Constant(1,10,1.9).eval();
  Eigen::MatrixXd updates=Eigen::MatrixXd::Zero(1,10);
  ASSERT_TRUE(filter(parameters,updates));EXPECT_NEAR(updates.norm(),0,1e-10);
  updates.setConstant(.5);ASSERT_TRUE(filter(parameters,updates));
  EXPECT_DOUBLE_EQ(updates(0,0),0);EXPECT_DOUBLE_EQ(updates(0,9),0);
  EXPECT_LE((parameters+updates).maxCoeff(),2.0000001);
  EXPECT_GE((parameters+updates).minCoeff(),.9999999);
  const auto temporal=stomp_moveit::filters::temporalBoundedUpdates(
      model->getJointModelGroup("arm"),10,Eigen::VectorXd::Constant(1,.01));
  updates.setConstant(.5);ASSERT_TRUE(temporal(parameters,updates));
  const auto projected=(parameters+updates).eval();
  EXPECT_DOUBLE_EQ(projected(0,0),1.9);EXPECT_DOUBLE_EQ(projected(0,9),1.9);
  for(int i=1;i<10;++i)EXPECT_LE(std::abs(projected(0,i)-projected(0,i-1)),.010000001);
  Eigen::MatrixXd unreachable=parameters;unreachable(0,9)=1.;updates.setZero();
  EXPECT_FALSE(temporal(unreachable,updates));
  const stomp_moveit::CostFn narrow_cost=[](const Eigen::MatrixXd& values,Eigen::VectorXd& costs,bool& valid){
    costs=Eigen::VectorXd::Zero(values.cols());valid=values.maxCoeff()<=1.905;return true;};
  const auto preserving=stomp_moveit::filters::preserveFeasibleUpdates(temporal,narrow_cost);
  updates.setConstant(.5);ASSERT_TRUE(preserving(parameters,updates));
  EXPECT_LE((parameters+updates).maxCoeff(),1.905);
  EXPECT_DOUBLE_EQ(updates(0,0),0);EXPECT_DOUBLE_EQ(updates(0,9),0);
}

TEST(DynamicStomp, DetectsOtherArmCrossingBetweenStationaryWaypoints)
{
  auto urdf=urdf::parseURDF(R"(<robot name="sliders"><link name="base"/>
    <link name="first"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
    <link name="second"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
    <joint name="one" type="prismatic"><parent link="base"/><child link="first"/>
      <axis xyz="1 0 0"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    <joint name="two" type="prismatic"><parent link="base"/><child link="second"/>
      <origin xyz="0.4 0 0"/><axis xyz="1 0 0"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    </robot>)");
  auto srdf=std::make_shared<srdf::Model>();
  srdf->initString(*urdf,R"(<robot name="sliders"><group name="first_group"><joint name="one"/></group></robot>)");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  auto scene=std::make_shared<planning_scene::PlanningScene>(model);
  scene->getCurrentStateNonConst().setToDefaultValues();scene->getCurrentStateNonConst().update();
  const auto* group=model->getJointModelGroup("first_group");
  multi_arm_motion::Prediction p;p.source="peer";p.version=1;p.valid_until.sec=11;
  p.trajectory.header.stamp.sec=10;p.trajectory.joint_names={"two"};
  trajectory_msgs::msg::JointTrajectoryPoint a,b;a.positions={0};b.positions={-0.8};b.time_from_start.sec=1;
  p.trajectory.points={a,b};multi_arm_motion::validatePrediction(p);
  Eigen::MatrixXd values=Eigen::MatrixXd::Zero(1,2);Eigen::VectorXd cost;bool valid;
  auto static_cost=stomp_moveit::costs::getCollisionCostFunction(scene,group,1.0);
  ASSERT_TRUE(static_cost(values,cost,valid));EXPECT_TRUE(valid);
  // There is a 0.2 m gap: collision-free, but invalid for a requested 0.25 m margin.
  auto margin_cost=stomp_moveit::costs::dynamicCollisionCost(
      scene,group,{},rclcpp::Time(10,0,RCL_ROS_TIME),1,.25);
  ASSERT_TRUE(margin_cost(values,cost,valid));EXPECT_FALSE(valid);EXPECT_GT(cost.sum(),0);
  margin_cost=stomp_moveit::costs::dynamicCollisionCost(
      scene,group,{},rclcpp::Time(10,0,RCL_ROS_TIME),1,.15);
  ASSERT_TRUE(margin_cost(values,cost,valid));EXPECT_TRUE(valid);
  // Static geometry must still be checked between robot waypoints, even with
  // an arbitrarily slow time law and no peer forecast.
  Eigen::MatrixXd crossing(1,2);crossing << 0., .8;
  auto static_slow=stomp_moveit::costs::dynamicCollisionCost(
      scene,group,{},rclcpp::Time(10,0,RCL_ROS_TIME),100);
  ASSERT_TRUE(static_slow(crossing,cost,valid));EXPECT_FALSE(valid);
  // A positive but insufficient gap can condition the seed without moving
  // endpoints. The complete cost check remains required afterwards.
  Eigen::MatrixXd narrow(1,3);narrow << 0., .18, 0.;
  ASSERT_TRUE(stomp_moveit::conditionSeedClearance(*scene,group,{},
      rclcpp::Time(10,0,RCL_ROS_TIME),1,.05,narrow,
      std::chrono::steady_clock::now()+std::chrono::seconds(1)));
  EXPECT_DOUBLE_EQ(narrow(0,0),0);EXPECT_DOUBLE_EQ(narrow(0,2),0);
  EXPECT_LT(narrow(0,1),.15);
  auto conditioned=stomp_moveit::costs::dynamicCollisionCost(
      scene,group,{},rclcpp::Time(10,0,RCL_ROS_TIME),1,.05);
  ASSERT_TRUE(conditioned(narrow,cost,valid));EXPECT_TRUE(valid);
  auto dynamic_cost=stomp_moveit::costs::dynamicCollisionCost(scene,group,{p},rclcpp::Time(10,0,RCL_ROS_TIME),1);
  ASSERT_TRUE(dynamic_cost(values,cost,valid));EXPECT_FALSE(valid);EXPECT_GT(cost.sum(),0);
  // Expired predictions must invalidate the candidate rather than disappear.
  dynamic_cost=stomp_moveit::costs::dynamicCollisionCost(scene,group,{p},rclcpp::Time(12,0,RCL_ROS_TIME),1);
  ASSERT_TRUE(dynamic_cost(values,cost,valid));EXPECT_FALSE(valid);
}
