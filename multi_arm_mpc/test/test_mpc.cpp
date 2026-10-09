#include <gtest/gtest.h>
#include <multi_arm_mpc/solver.hpp>
#include <multi_arm_mpc/braking.hpp>
#include <rclcpp/duration.hpp>
using namespace multi_arm_motion;
TEST(Mpc, BrakesBlockedGroupWhileIndependentPeerProgresses)
{
  MpcConfig config;config.horizon=5;config.dt=.1;config.budget_s=1.;
  config.clearance=.05;config.influence_distance=.08;
  MpcSolver solver(config);
  const Eigen::VectorXd zero=Eigen::VectorXd::Zero(2),one=Eigen::VectorXd::Ones(2);
  const auto result=solver.solve(zero,zero,Eigen::MatrixXd::Constant(2,5,.5),
      -one,one,one,one,[](const Eigen::VectorXd& q,double){return .1-q[0];},
      {},{},{},{},[](const auto&,const auto&,double){return true;},{{0},{1}});
  ASSERT_TRUE(result.feasible)<<result.reason;
  ASSERT_TRUE(result.braking_only);
  EXPECT_EQ(result.paused_joints,std::vector<int>({0}));
  EXPECT_LE(result.positions.row(0).maxCoeff(),.05+1e-9);
  EXPECT_GT(result.positions(1,4),.01);
}
TEST(Braking, PreservesMeasuredVelocityAndBoundsAcceleration)
{
  Eigen::VectorXd q=Eigen::VectorXd::Zero(3),v(3),limits=Eigen::VectorXd::Ones(3);v<<.4,-.2,0;
  auto trajectory=brakingTrajectory({"a","b","c"},q,v,-limits,limits,limits,.005,.025,
      [](const auto&,double){return 1.;});
  ASSERT_GT(trajectory.points.size(),2u);
  EXPECT_DOUBLE_EQ(trajectory.points.front().velocities[0],.4);
  EXPECT_NEAR(trajectory.points.back().positions[0],.08,1e-8);
  EXPECT_DOUBLE_EQ(trajectory.points.back().velocities[0],0);
  for(size_t i=1;i<trajectory.points.size();++i)
  {
    double dt=(rclcpp::Duration(trajectory.points[i].time_from_start)-rclcpp::Duration(trajectory.points[i-1].time_from_start)).seconds();
    for(int j=0;j<3;++j)EXPECT_LE(std::abs(trajectory.points[i].velocities[j]-trajectory.points[i-1].velocities[j])/dt,1.00001);
  }
}
TEST(Braking, RejectsCollisionAlongStoppingSweep)
{
  Eigen::VectorXd q=Eigen::VectorXd::Zero(1),v=Eigen::VectorXd::Constant(1,.4),one=Eigen::VectorXd::Ones(1);
  EXPECT_THROW(brakingTrajectory({"a"},q,v,-one,one,one,.005,.025,
      [](const Eigen::VectorXd& x,double){return .07-x[0];}),std::runtime_error);
}
TEST(Mpc, SupportsArbitraryDimensionAndLimitsAcceleration)
{
  for(int n:{1,3,12,18})
  {
    MpcConfig c;c.budget_s=1;MpcSolver solver(c);
    auto zero=Eigen::VectorXd::Zero(n).eval(),one=Eigen::VectorXd::Ones(n).eval();
    auto result=solver.solve(zero,zero,Eigen::MatrixXd::Constant(n,c.horizon,0.5),-one,one,one,one,
       [](const auto&,double){return 1.0;});
    ASSERT_TRUE(result.feasible)<<result.reason;
    EXPECT_LE(result.velocities.col(0).cwiseAbs().maxCoeff(),c.dt+1e-8);
    EXPECT_GT(result.positions.col(0).minCoeff(),0);
    for(int k=1;k<c.horizon;++k)
      EXPECT_LE((result.velocities.col(k)-result.velocities.col(k-1)).cwiseAbs().maxCoeff(),c.dt+1e-8);
  }
}
TEST(Mpc, KeepsFeasibleTaskPathWhenObstacleCostPrefersLeavingIt)
{
  MpcConfig config;config.horizon=5;config.budget_s=1.;config.iterations=30;
  MpcSolver solver(config);
  Eigen::VectorXd zero=Eigen::VectorXd::Zero(1),one=Eigen::VectorXd::Ones(1);
  const auto result=solver.solve(zero,zero,Eigen::MatrixXd::Zero(1,config.horizon),-one,one,one,one,
      [](const Eigen::VectorXd& q,double){return .03+q[0];},
      [](const Eigen::VectorXd& q,double){return std::abs(q[0])<=.0025;},
      [](const Eigen::VectorXd& q,double){return std::make_pair(.03+q[0],Eigen::VectorXd::Ones(1).eval());});
  ASSERT_TRUE(result.feasible)<<result.reason;
  EXPECT_LE(result.positions.cwiseAbs().maxCoeff(),.0025);
}
TEST(Mpc, SmoothTaskCostKeepsMotionAwayFromCorridorBoundary)
{
  MpcConfig config;config.horizon=5;config.budget_s=1.;config.iterations=30;
  config.terminal_weight=config.tracking_weight;
  Eigen::VectorXd zero=Eigen::VectorXd::Zero(1),one=Eigen::VectorXd::Ones(1);
  MpcSolver solver(config);
  const auto result=solver.solve(zero,zero,Eigen::MatrixXd::Zero(1,config.horizon),-one,one,one,one,
      [](const Eigen::VectorXd& q,double){return .03+q[0];},
      [](const Eigen::VectorXd& q,double){return std::abs(q[0])<=.0025;},
      [](const Eigen::VectorXd& q,double){return std::make_pair(.03+q[0],Eigen::VectorXd::Ones(1).eval());},{},
      [](const Eigen::VectorXd& q,double,bool){return std::make_pair(q.squaredNorm(),(2*q).eval());});
  ASSERT_TRUE(result.feasible)<<result.reason;
  EXPECT_LT(result.positions.cwiseAbs().maxCoeff(),.001);
}
TEST(Mpc, ReachesSafeEndpointDespiteSoftObstacleRepulsion)
{
  MpcConfig config;config.clearance=.005;config.influence_distance=.03;config.budget_s=.01;
  MpcSolver solver(config);
  Eigen::VectorXd q=Eigen::VectorXd::Zero(1),v=q,one=Eigen::VectorXd::Ones(1);
  const Eigen::MatrixXd target=Eigen::MatrixXd::Constant(1,config.horizon,.01);
  for(int i=0;i<120;++i)
  {
    const auto result=solver.solve(q,v,target,-one,one,one,one,
        [](const Eigen::VectorXd& x,double){return .03-x[0];});
    ASSERT_TRUE(result.feasible)<<result.reason;
    q=result.positions.col(0);v=result.velocities.col(0);
  }
  EXPECT_NEAR(q[0],.01,1e-4);
  EXPECT_NEAR(v[0],0.,1e-4);
}
TEST(Mpc, PreservesCheckedStoppingOptionBeforeIssuingPrefix)
{
  MpcConfig config;config.budget_s=.1;
  MpcSolver solver(config);
  Eigen::VectorXd q=Eigen::VectorXd::Zero(1),v=Eigen::VectorXd::Constant(1,.3),one=Eigen::VectorXd::Ones(1);
  StopPredicate stop=[](const Eigen::VectorXd& x,const Eigen::VectorXd& speed,double){
    return x[0]+.5*speed[0]*std::abs(speed[0])<=.06;
  };
  const auto result=solver.solve(q,v,Eigen::MatrixXd::Constant(1,config.horizon,.5),-one,one,one,one,
      [](const auto&,double){return 1.;},{},{},{},{},stop);
  ASSERT_TRUE(result.feasible)<<result.reason;
  EXPECT_TRUE(stop(result.positions.col(config.stop_step-1),result.velocities.col(config.stop_step-1),0.));
}
TEST(Mpc, RejectsBrakingFallbackThatIsAlsoUnsafe)
{
  MpcConfig config;config.budget_s=.1;
  MpcSolver solver(config);
  Eigen::VectorXd q=Eigen::VectorXd::Zero(1),v=Eigen::VectorXd::Constant(1,.5),one=Eigen::VectorXd::Ones(1);
  const auto result=solver.solve(q,v,Eigen::MatrixXd::Constant(1,config.horizon,.5),-one,one,one,one,
      [](const auto&,double){return 1.;},{},{},{},{},
      [](const Eigen::VectorXd& x,const Eigen::VectorXd& speed,double){return x[0]+.5*speed[0]*std::abs(speed[0])<=.06;});
  EXPECT_FALSE(result.feasible);
}
TEST(Mpc, RejectsCollisionAndTaskConstraintViolation)
{
  MpcConfig c;c.budget_s=1;MpcSolver solver(c);
  Eigen::VectorXd q=Eigen::VectorXd::Zero(1),one=Eigen::VectorXd::Ones(1);
  auto ref=Eigen::MatrixXd::Ones(1,c.horizon).eval();
  EXPECT_FALSE(solver.solve(q,q,ref,-one,one,one,one,[](const auto&,double){return 0.;}).feasible);
  EXPECT_FALSE(solver.solve(q,q,ref,-one,one,one,one,[](const auto&,double){return 1.;},
      [](const auto&,double){return false;}).feasible);
}
TEST(Mpc, RejectsInvalidInput)
{
  EXPECT_THROW(MpcSolver(MpcConfig{0}),std::invalid_argument);
}
TEST(Mpc, ChangesMotionForApproachingObstacle)
{
  MpcConfig c;c.budget_s=2;c.iterations=40;
  Eigen::VectorXd q=Eigen::VectorXd::Zero(1),one=Eigen::VectorXd::Ones(1);
  auto reference=Eigen::MatrixXd::Constant(1,c.horizon,0.5).eval();
  MpcSolver free_solver(c),avoid_solver(c);
  auto free=free_solver.solve(q,q,reference,-one,one,one,one,[](const auto&,double){return 1.;});
  auto avoid=avoid_solver.solve(q,q,reference,-one,one,one,one,
      [](const Eigen::VectorXd& x,double t){return 0.1-0.02*t-x[0];});
  ASSERT_TRUE(free.feasible);ASSERT_TRUE(avoid.feasible)<<avoid.reason;
  EXPECT_LT(avoid.positions(0,c.horizon-1),free.positions(0,c.horizon-1));
  EXPECT_GE(avoid.min_distance,c.clearance);
}
TEST(Mpc, UsesClosestPointGradientOracle)
{
  MpcConfig c;c.budget_s=2;c.iterations=40;
  Eigen::VectorXd q=Eigen::VectorXd::Zero(1),one=Eigen::VectorXd::Ones(1);
  const auto reference=Eigen::MatrixXd::Constant(1,c.horizon,.5).eval();
  DistanceOracle distance=[](const Eigen::VectorXd& x,double t){return .1-.02*t-x[0];};
  MpcSolver finite(c),provided(c);int queries=0;
  const auto a=finite.solve(q,q,reference,-one,one,one,one,distance);
  const auto b=provided.solve(q,q,reference,-one,one,one,one,distance,{},
      [&](const Eigen::VectorXd& x,double t){++queries;return std::make_pair(distance(x,t),-one);});
  ASSERT_TRUE(a.feasible);ASSERT_TRUE(b.feasible);EXPECT_GT(queries,0);
  EXPECT_LT((a.positions-b.positions).norm(),1e-6);
}
TEST(Mpc, ExhaustedOptimizationBudgetStillRecentersWarmStartAndStopsAtReference)
{
  MpcConfig c;c.budget_s=1e-9;MpcSolver solver(c);
  Eigen::VectorXd q=Eigen::VectorXd::Zero(1),v=q,one=Eigen::VectorXd::Ones(1);
  const auto reference=Eigen::MatrixXd::Constant(1,c.horizon,.05).eval();
  for(int step=0;step<200;++step)
  {
    const auto result=solver.solve(q,v,reference,-one,one,one,one,[](const auto&,double){return 1.;});
    ASSERT_TRUE(result.feasible)<<result.reason;
    q=result.positions.col(0);v=result.velocities.col(0);
  }
  EXPECT_NEAR(q[0],.05,1e-4);EXPECT_NEAR(v[0],0,1e-4);
}
