#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "teb_local_planner/g2o_types/vertex_steering_angle.h"
#include "teb_local_planner/g2o_types/edge_steering.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/teb_config.h"

using teb_local_planner::PoseSE2;
using teb_local_planner::TebConfig;
using teb_local_planner::VertexPose;
using teb_local_planner::VertexTimeDiff;
using teb_local_planner::VertexSteeringAngle;
using teb_local_planner::steeringFromSegment;
using teb_local_planner::closestSteeringBranch;

namespace
{
constexpr double kWheelbase = 1.2526;

// build a pose pair with the given signed longitudinal displacement and heading change
std::pair<PoseSE2, PoseSE2> makeSegment(double ds, double dtheta)
{
  const double theta_m = 0.5 * dtheta;
  return {PoseSE2(0.0, 0.0, 0.0),
          PoseSE2(ds * std::cos(theta_m), ds * std::sin(theta_m), dtheta)};
}
} // namespace


TEST(SteeringFromSegment, straightForward)
{
  const auto segment = makeSegment(0.5, 0.0);
  const auto result = steeringFromSegment(segment.first, segment.second, kWheelbase);
  ASSERT_TRUE(result.first);
  EXPECT_NEAR(result.second, 0.0, 1e-9);
}

TEST(SteeringFromSegment, forwardLeftArc)
{
  const double ds = 0.2, dtheta = 0.1;
  const auto segment = makeSegment(ds, dtheta);
  const auto result = steeringFromSegment(segment.first, segment.second, kWheelbase);
  ASSERT_TRUE(result.first);
  EXPECT_NEAR(result.second, std::atan2(kWheelbase * dtheta, ds), 1e-9);
  EXPECT_GT(result.second, 0.0);
}

TEST(SteeringFromSegment, reverseArcSameCircle)
{
  // driving the same left-turn circle backwards: ds < 0, dtheta < 0
  const double ds = -0.2, dtheta = -0.1;
  const auto segment = makeSegment(ds, dtheta);
  const auto result = steeringFromSegment(segment.first, segment.second, kWheelbase);
  ASSERT_TRUE(result.first);
  // raw atan2 value lies on the reverse-rolling branch; the physical branch is one period (pi) away
  const double physical = closestSteeringBranch(result.second, 0.0);
  EXPECT_NEAR(physical, std::atan(kWheelbase * dtheta / ds), 1e-9);
  EXPECT_GT(physical, 0.0); // steering to the left, rolling backwards
}

TEST(SteeringFromSegment, turnInPlace)
{
  const auto segment = makeSegment(0.0, 0.3);
  const auto result = steeringFromSegment(segment.first, segment.second, kWheelbase);
  ASSERT_TRUE(result.first);
  EXPECT_NEAR(result.second, M_PI_2, 1e-9);
}

TEST(SteeringFromSegment, degenerateStandstill)
{
  const auto segment = makeSegment(0.0, 0.0);
  const auto result = steeringFromSegment(segment.first, segment.second, kWheelbase);
  EXPECT_FALSE(result.first);
}

TEST(ClosestSteeringBranch, picksBranchNearReference)
{
  // turn-in-place flip: atan2 yields -pi/2 but the chain is at +pi/2 -> stay on +pi/2 branch
  EXPECT_NEAR(closestSteeringBranch(-M_PI_2, M_PI_2), M_PI_2, 1e-9);
  EXPECT_NEAR(closestSteeringBranch(M_PI, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(closestSteeringBranch(0.3, 0.2), 0.3, 1e-9);
  EXPECT_NEAR(closestSteeringBranch(-1.5, 1.5), -1.5 + M_PI, 1e-9);
}


class SteeringEdgeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    cfg_.robot.wheelbase = kWheelbase;
    cfg_.robot.max_steering_rate = 1.0;
    cfg_.robot.max_steering_angle = 1.5708;
    cfg_.robot.max_steering_angle_right = 0.0;
    cfg_.optim.penalty_epsilon = 0.0;
  }

  double consistencyError(double ds, double dtheta, double phi)
  {
    const auto segment = makeSegment(ds, dtheta);
    VertexPose pose1(segment.first);
    VertexPose pose2(segment.second);
    VertexSteeringAngle steering(phi);

    teb_local_planner::EdgeSteeringConsistency edge;
    edge.setVertex(0, &pose1);
    edge.setVertex(1, &pose2);
    edge.setVertex(2, &steering);
    edge.setTebConfig(cfg_);
    edge.computeError();
    return edge.error()[0];
  }

  TebConfig cfg_;
};

TEST_F(SteeringEdgeTest, consistencyZeroOnMatchingArc)
{
  const double ds = 0.2, dtheta = 0.1;
  const double phi = std::atan2(kWheelbase * dtheta, ds);
  EXPECT_NEAR(consistencyError(ds, dtheta, phi), 0.0, 1e-12);
}

TEST_F(SteeringEdgeTest, consistencyZeroOnReverseWithPhysicalBranch)
{
  const double ds = -0.2, dtheta = -0.1;
  const double phi = std::atan(kWheelbase * dtheta / ds); // physical branch, in (-pi/2, pi/2)
  EXPECT_NEAR(consistencyError(ds, dtheta, phi), 0.0, 1e-12);
}

TEST_F(SteeringEdgeTest, consistencyTurnInPlaceForcesNinetyDegrees)
{
  // wheel at +-90 degrees: zero error; wheel straight: |error| = wheelbase * |dtheta|
  EXPECT_NEAR(consistencyError(0.0, 0.4, M_PI_2), 0.0, 1e-12);
  EXPECT_NEAR(consistencyError(0.0, 0.4, -M_PI_2), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(consistencyError(0.0, 0.4, 0.0)), kWheelbase * 0.4, 1e-12);
}

TEST_F(SteeringEdgeTest, consistencyVanishesAtStandstill)
{
  EXPECT_NEAR(consistencyError(0.0, 0.0, 0.7), 0.0, 1e-12);
  EXPECT_NEAR(consistencyError(0.0, 0.0, -1.2), 0.0, 1e-12);
}

TEST_F(SteeringEdgeTest, rateEdgePenalizesExcessRate)
{
  VertexSteeringAngle steering1(0.0);
  VertexSteeringAngle steering2(1.0);
  VertexTimeDiff dt1(0.5), dt2(0.5);

  teb_local_planner::EdgeSteeringRate edge;
  edge.setVertex(0, &steering1);
  edge.setVertex(1, &steering2);
  edge.setVertex(2, &dt1);
  edge.setVertex(3, &dt2);
  edge.setTebConfig(cfg_);
  edge.computeError();
  // rate = 2 * 1.0 / 1.0 = 2.0, bound = 1.0 -> error = 1.0
  EXPECT_NEAR(edge.error()[0], 1.0, 1e-12);

  steering2.steering() = 0.4; // rate = 0.8 -> within bound
  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-12);
}

TEST_F(SteeringEdgeTest, rateStartEdgeAnchorsMeasuredAngle)
{
  VertexSteeringAngle steering(M_PI_2);
  VertexTimeDiff dt(1.0);

  teb_local_planner::EdgeSteeringRateStart edge;
  edge.setVertex(0, &steering);
  edge.setVertex(1, &dt);
  edge.setInitialSteeringAngle(-M_PI_2);
  edge.setTebConfig(cfg_);
  edge.computeError();
  // rate = 2 * pi / 1.0, bound = 1.0 -> error = 2*pi - 1
  EXPECT_NEAR(edge.error()[0], 2.0 * M_PI - 1.0, 1e-12);

  edge.setInitialSteeringAngle(M_PI_2 - 0.2); // rate = 0.4 -> within bound
  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-12);
}

TEST_F(SteeringEdgeTest, boundEdgeSymmetricAndAsymmetric)
{
  VertexSteeringAngle steering(1.7);
  teb_local_planner::EdgeSteeringBound edge;
  edge.setVertex(0, &steering);
  edge.setTebConfig(cfg_);

  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 1.7 - 1.5708, 1e-9);

  steering.steering() = -1.7;
  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 1.7 - 1.5708, 1e-9);

  steering.steering() = 0.3;
  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-12);

  // asymmetric right bound (mirrors min_turning_radius_right semantics)
  cfg_.robot.max_steering_angle_right = 0.5;
  steering.steering() = -0.6;
  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 0.1, 1e-9);
  steering.steering() = 0.6; // left side unaffected
  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 0.0, 1e-12);
}


class SteeringPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("steering_state_test");

    cfg_.robot.wheelbase = kWheelbase;
    cfg_.robot.min_turning_radius = 0.0;
    cfg_.robot.max_vel_x = 1.0;
    cfg_.robot.max_vel_x_backwards = 0.5;
    cfg_.robot.max_vel_theta = 1.0;
    cfg_.robot.acc_lim_x = 0.5;
    cfg_.robot.acc_lim_theta = 0.5;
    cfg_.robot.max_steering_rate = 0.5;
    cfg_.optim.weight_steering_rate = 100.0; // make the rate constraint firmly binding for timing checks
    cfg_.obstacles.include_dynamic_obstacles = false;
  }

  std::shared_ptr<teb_local_planner::TebOptimalPlanner> makePlanner()
  {
    return std::make_shared<teb_local_planner::TebOptimalPlanner>(
        node_, cfg_, &obstacles_, teb_local_planner::TebVisualizationPtr(), &via_points_);
  }

  nav2::LifecycleNode::SharedPtr node_;
  TebConfig cfg_;
  teb_local_planner::ObstContainer obstacles_;
  teb_local_planner::ViaPointContainer via_points_;
};

TEST_F(SteeringPlannerTest, featureOffKeepsLegacyBehavior)
{
  cfg_.robot.steering_state_enabled = false;
  auto planner = makePlanner();

  ASSERT_TRUE(planner->plan(PoseSE2(0, 0, 0), PoseSE2(4, 0, 0)));
  std::vector<double> profile;
  EXPECT_FALSE(planner->getSteeringProfile(profile));
  EXPECT_TRUE(profile.empty());
}

TEST_F(SteeringPlannerTest, straightPlanYieldsConsistentBoundedProfile)
{
  cfg_.robot.steering_state_enabled = true;
  auto planner = makePlanner();
  planner->setInitialSteeringAngle(0.0);

  ASSERT_TRUE(planner->plan(PoseSE2(0, 0, 0), PoseSE2(4, 0, 0)));

  std::vector<double> profile;
  ASSERT_TRUE(planner->getSteeringProfile(profile));
  ASSERT_EQ(static_cast<int>(profile.size()), planner->teb().sizeTimeDiffs());

  for (size_t i = 0; i < profile.size(); ++i)
  {
    EXPECT_LE(std::abs(profile[i]), cfg_.robot.max_steering_angle + 0.1) << "steering bound violated at " << i;
    EXPECT_LE(std::abs(profile[i]), 0.3) << "straight plan should need almost no steering at " << i;
  }

  // the converged steering state must be consistent with the trajectory geometry
  for (size_t i = 0; i + 1 < static_cast<size_t>(planner->teb().sizePoses()); ++i)
  {
    const double dtheta = g2o::normalize_theta(planner->teb().Pose(i + 1).theta() - planner->teb().Pose(i).theta());
    const double theta_m = planner->teb().Pose(i).theta() + 0.5 * dtheta;
    const Eigen::Vector2d delta = planner->teb().Pose(i + 1).position() - planner->teb().Pose(i).position();
    const double ds = delta.x() * std::cos(theta_m) + delta.y() * std::sin(theta_m);
    const double residual = ds * std::sin(profile[i]) - kWheelbase * dtheta * std::cos(profile[i]);
    EXPECT_LE(std::abs(residual), 0.05) << "consistency residual too large at " << i;
  }
}

TEST_F(SteeringPlannerTest, mismatchedStartSteeringIsAnchoredAndCostsTime)
{
  cfg_.robot.steering_state_enabled = true;

  auto planner_matched = makePlanner();
  planner_matched->setInitialSteeringAngle(0.0);
  ASSERT_TRUE(planner_matched->plan(PoseSE2(0, 0, 0), PoseSE2(4, 0, 0)));
  const double time_matched = planner_matched->teb().getSumOfAllTimeDiffs();

  auto planner_mismatched = makePlanner();
  planner_mismatched->setInitialSteeringAngle(M_PI_2); // wheel fully left, plan is straight ahead
  ASSERT_TRUE(planner_mismatched->plan(PoseSE2(0, 0, 0), PoseSE2(4, 0, 0)));
  const double time_mismatched = planner_mismatched->teb().getSumOfAllTimeDiffs();

  std::vector<double> profile;
  ASSERT_TRUE(planner_mismatched->getSteeringProfile(profile));
  ASSERT_GE(profile.size(), 2u);

  // the start anchor must pull the first steering state towards the measured wheel angle
  EXPECT_GT(profile.front(), std::abs(profile.back()))
      << "first steering state should stay close to the measured +pi/2 wheel angle";

  // the steering rate along the converged chain must respect the configured limit (soft constraint)
  for (size_t i = 0; i + 1 < profile.size(); ++i)
  {
    const double dt_sum = planner_mismatched->teb().TimeDiff(i) + planner_mismatched->teb().TimeDiff(i + 1);
    const double rate = std::abs(2.0 * (profile[i + 1] - profile[i]) / dt_sum);
    EXPECT_LE(rate, cfg_.robot.max_steering_rate * 1.5 + 0.05) << "steering rate exceeded at " << i;
  }

  // unwinding the wheel from +90 degrees must cost extra trajectory time
  EXPECT_GT(time_mismatched, time_matched + 0.2);
}

TEST_F(SteeringPlannerTest, turnInPlaceProfileSaturatesAtNinetyDegrees)
{
  cfg_.robot.steering_state_enabled = true;
  auto planner = makePlanner();
  planner->setInitialSteeringAngle(M_PI_2);

  // use the pose-vector overload: it estimates timesteps from the angular displacement,
  // which the (start, goal) overload cannot do for a pure rotation (zero distance)
  std::vector<geometry_msgs::msg::PoseStamped> initial_plan(2);
  PoseSE2(0, 0, 0).toPoseMsg(initial_plan.front().pose);
  PoseSE2(0, 0, M_PI).toPoseMsg(initial_plan.back().pose);
  ASSERT_TRUE(planner->plan(initial_plan));

  std::vector<double> profile;
  ASSERT_TRUE(planner->getSteeringProfile(profile));

  for (size_t i = 0; i + 1 < static_cast<size_t>(planner->teb().sizePoses()); ++i)
  {
    const double dtheta = std::abs(g2o::normalize_theta(planner->teb().Pose(i + 1).theta() - planner->teb().Pose(i).theta()));
    const Eigen::Vector2d delta = planner->teb().Pose(i + 1).position() - planner->teb().Pose(i).position();
    if (dtheta > 0.05 && delta.norm() < 0.01)
    {
      EXPECT_LE(std::abs(std::abs(profile[i]) - M_PI_2), 0.3)
          << "turn-in-place segment " << i << " requires the wheel near +-90 degrees";
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
