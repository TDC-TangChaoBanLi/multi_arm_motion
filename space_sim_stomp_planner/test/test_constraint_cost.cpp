#include <gtest/gtest.h>
#include <stomp_moveit/cost_functions.hpp>

TEST(StompConstraintCost, AllowedOrientationDeviationHasNoViolationCost)
{
  EXPECT_DOUBLE_EQ(stomp_moveit::costs::constraintViolationCost(
    kinematic_constraints::ConstraintEvaluationResult(true, 0.8), 1.0), 0.0);
  EXPECT_DOUBLE_EQ(stomp_moveit::costs::constraintViolationCost(
    kinematic_constraints::ConstraintEvaluationResult(false, 0.2), 2.0), 0.4);
}

TEST(StompConstraintCost, VerticalMirrorAllowsNormalSpinAndHorizontalAzimuth)
{
  const Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
  for (double spin : {-3.0, -1.0, 0.0, 1.0, 3.0})
    for (double azimuth : {-2.0, -0.5, 0.0, 0.5, 2.0})
    {
      const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(spin, normal)).toRotationMatrix();
      EXPECT_DOUBLE_EQ(stomp_moveit::costs::verticalMirrorViolationCost(rotation, normal, 0.1), 0.0);
    }
}

TEST(StompConstraintCost, ExcessiveMirrorTiltIsInvalid)
{
  const auto cost = stomp_moveit::costs::verticalMirrorViolationCost(
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix(), Eigen::Vector3d::UnitY(), 0.1);
  EXPECT_GT(cost, 0.0);
}

TEST(StompConstraintCost, ViolationInsideTrajectorySegmentIsDetected)
{
  auto cost = stomp_moveit::costs::getCostFunctionFromStateValidator(
    [](const Eigen::VectorXd& q) { return (q[0] > 0.4 && q[0] < 0.6) ? 1.0 : 0.0; }, 0.05);
  Eigen::MatrixXd path(1, 2);
  path << 0.0, 1.0;
  Eigen::VectorXd costs(2);
  bool valid = true;
  ASSERT_TRUE(cost(path, costs, valid));
  EXPECT_FALSE(valid);
  EXPECT_GT(costs.sum(), 0.0);
}
