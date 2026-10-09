#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.hpp>
#include <moveit/local_planner/trajectory_operator_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

TEST(Plugins, PlannerAndMpcAreDiscoverable)
{
  pluginlib::ClassLoader<planning_interface::PlannerManager> planners("moveit_core","planning_interface::PlannerManager");
  EXPECT_TRUE(planners.isClassAvailable("space_sim_stomp/StompPlanner"));
  ASSERT_TRUE(planners.isClassAvailable("dynamic_stomp/DynamicStompPlanner"));
  EXPECT_NE(planners.createSharedInstance("dynamic_stomp/DynamicStompPlanner"),nullptr);
  pluginlib::ClassLoader<moveit::hybrid_planning::LocalConstraintSolverInterface> solvers(
      "moveit_hybrid_planning","moveit::hybrid_planning::LocalConstraintSolverInterface");
  ASSERT_TRUE(solvers.isClassAvailable("multi_arm_mpc/DistanceMpc"));
  EXPECT_NE(solvers.createSharedInstance("multi_arm_mpc/DistanceMpc"),nullptr);
}
TEST(Plugins, ReferencesProgressIndependentlyAndRequireMeasuredGoalArrival)
{
  if(!rclcpp::ok())rclcpp::init(0,nullptr);
  auto node=std::make_shared<rclcpp::Node>("reference_operator_test");
  auto urdf=urdf::parseURDF(R"(<robot name="pair"><link name="base"/><link name="a"/><link name="b"/>
    <joint name="j1" type="prismatic"><parent link="base"/><child link="a"/><axis xyz="1 0 0"/>
      <limit lower="-2" upper="2" velocity="1" effort="1"/></joint>
    <joint name="j2" type="prismatic"><parent link="base"/><child link="b"/><axis xyz="0 1 0"/>
      <limit lower="-2" upper="2" velocity="1" effort="1"/></joint></robot>)");
  auto srdf=std::make_shared<srdf::Model>();
  srdf->initString(*urdf,R"(<robot name="pair"><group name="a"><joint name="j1"/></group>
    <group name="b"><joint name="j2"/></group><group name="all"><joint name="j1"/><joint name="j2"/></group></robot>)");
  auto model=std::make_shared<moveit::core::RobotModel>(urdf,srdf);
  pluginlib::ClassLoader<moveit::hybrid_planning::TrajectoryOperatorInterface> operators(
      "moveit_hybrid_planning","moveit::hybrid_planning::TrajectoryOperatorInterface");
  auto op=operators.createSharedInstance("multi_arm_hybrid_planning/IndependentProgress");
  ASSERT_TRUE(op->initialize(node,model,"all"));
  moveit::core::RobotState current(model);current.setToDefaultValues();current.update();
  robot_trajectory::RobotTrajectory a(model,"a"),b(model,"b"),local(model,"all");
  a.addSuffixWayPoint(current,0);b.addSuffixWayPoint(current,0);
  auto end=current;end.setVariablePosition("j1",1);end.update();a.addSuffixWayPoint(end,2);
  end=current;end.setVariablePosition("j2",-1);end.update();b.addSuffixWayPoint(end,2);
  EXPECT_TRUE(op->addTrajectorySegment(a).feedback.empty());EXPECT_TRUE(op->addTrajectorySegment(b).feedback.empty());
  EXPECT_DOUBLE_EQ(op->getTrajectoryProgress(current),0);
  EXPECT_TRUE(op->getLocalTrajectory(current,local).feedback.empty());ASSERT_GT(local.getWayPointCount(),1u);
  EXPECT_GT(local.getWayPoint(1).getVariablePosition("j1"),0);
  EXPECT_LT(local.getWayPoint(1).getVariablePosition("j2"),0);
  current.setVariablePosition("j1",1);current.update();
  EXPECT_DOUBLE_EQ(op->getTrajectoryProgress(current),0);
  current.setVariablePosition("j2",-1);current.update();current.zeroVelocities();
  EXPECT_DOUBLE_EQ(op->getTrajectoryProgress(current),1);
  op->reset();EXPECT_DOUBLE_EQ(op->getTrajectoryProgress(current),0);
}
