#include <gtest/gtest.h>
#include <multi_arm_motion_core/prediction.hpp>
using namespace multi_arm_motion;
Prediction example()
{
  Prediction p; p.source = "robot"; p.version = 1;
  p.trajectory.header.stamp.sec = 10; p.valid_until.sec = 12;
  p.trajectory.joint_names = {"j1"};
  trajectory_msgs::msg::JointTrajectoryPoint a, b;
  a.positions = {0}; b.positions = {2}; b.time_from_start.sec = 1;
  p.trajectory.points = {a,b}; return p;
}
TEST(Prediction, InterpolatesAndHoldsOnlyInsideValidity)
{
  auto p = example(); validatePrediction(p);
  EXPECT_DOUBLE_EQ(samplePrediction(p, rclcpp::Time(10,500000000,RCL_ROS_TIME))[0], 1);
  EXPECT_DOUBLE_EQ(samplePrediction(p, rclcpp::Time(12,0,RCL_ROS_TIME))[0], 2);
  EXPECT_THROW(samplePrediction(p, rclcpp::Time(13,0,RCL_ROS_TIME)), std::runtime_error);
  EXPECT_THROW(samplePrediction(p, rclcpp::Time(9,0,RCL_ROS_TIME)), std::runtime_error);
}
TEST(Prediction, RejectsStaleVersionAndMalformedSamples)
{
  PredictionStore store; std::string error; auto p = example();
  EXPECT_TRUE(store.update(p,error)); EXPECT_FALSE(store.update(p,error));
  p.version++; p.trajectory.points.back().positions = {NAN};
  EXPECT_FALSE(store.update(p,error)); EXPECT_EQ(store.snapshot()[0].version, 1u);
  p = example(); p.trajectory.points.back().time_from_start.sec = 0;
  EXPECT_THROW(validatePrediction(p),std::invalid_argument);
}
