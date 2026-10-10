// Control latency compensation (ControlLatencyPredictor) with the real TEB optimizer in a closed loop
// against a drive whose measured motion follows the command ~0.45 s late (amr47 traction, 2026-10-10).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "teb_local_planner/control_latency_predictor.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/robot_footprint_model.h"
#include "teb_local_planner/teb_config.h"

using teb_local_planner::ControlLatencyPredictor;

namespace
{

constexpr double kControlPeriod = 0.05;  // controller_frequency 20 Hz
constexpr double kPlantStep = 0.005;
constexpr double kDeadTime = 0.25;  // s, command -> drive reacts
constexpr double kLag = 0.2;  // s, first-order lag of the drive; with the dead time ~0.45 s as on amr47  // command -> measured motion, amr47 traction
constexpr double kPlantMaxAccel = 1.5;  // the drive brakes harder than TEB asks for
constexpr double kGoalTolerance = 0.16;  // general_goal_checker xy_goal_tolerance

// GeneralFollowPath (amr_bringup navigation/params.yaml) as far as it matters without obstacles.
void configureGeneralFollowPath(teb_local_planner::TebConfig& cfg)
{
  cfg.trajectory.dt_ref = 0.4;
  cfg.trajectory.dt_hysteresis = 0.1;
  cfg.trajectory.allow_init_with_backwards_motion = true;
  cfg.trajectory.control_look_ahead_poses = 2;
  cfg.robot.max_vel_x = cfg.robot.base_max_vel_x = 1.3;
  cfg.robot.max_vel_x_backwards = cfg.robot.base_max_vel_x_backwards = 0.5;
  cfg.robot.max_vel_theta = cfg.robot.base_max_vel_theta = 0.4;
  cfg.robot.acc_lim_x = 0.4;
  cfg.robot.acc_lim_theta = 0.4;
  cfg.robot.min_turning_radius = 0.3;
  cfg.robot.use_proportional_saturation = true;
  cfg.obstacles.min_obstacle_dist = 0.25;
  cfg.optim.no_inner_iterations = 5;
  cfg.optim.no_outer_iterations = 4;
  cfg.optim.weight_kinematics_forward_drive = 0.7;
  cfg.optim.weight_obstacle = 50.0;
  cfg.optim.weight_inflation = 0.3;
  cfg.optim.weight_viapoint = 0.5;
  cfg.goal_tolerance.free_goal_vel = false;
  teb_local_planner::Point2dContainer footprint;
  footprint.emplace_back(1.613, 0.4);
  footprint.emplace_back(1.613, -0.4);
  footprint.emplace_back(-0.347, -0.4);
  footprint.emplace_back(-0.347, 0.4);
  cfg.robot_model = std::make_shared<teb_local_planner::PolygonRobotFootprint>(footprint);
}

struct ApproachResult
{
  double max_commanded_decel = 0.0;  // m/s^2 over 0.25 s
  double speed_at_tolerance = -1.0;  // m/s when the robot first is within kGoalTolerance (SimpleGoalChecker end)
  double overshoot = 0.0;  // m past the goal (StoppedGoalChecker keeps controlling)
  double final_error = 0.0;  // m to the goal at the end
};

// Robot at the origin reversing at 0.5 m/s, goal 1.5 m behind it. TEB plans every 50 ms, the drive
// follows each command kDeadTime later with a first-order lag kLag (rate limited).
ApproachResult reverseApproach(const nav2::LifecycleNode::SharedPtr& node, double compensated_latency)
{
  teb_local_planner::TebConfig cfg;
  configureGeneralFollowPath(cfg);
  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::ViaPointContainer via_points;
  teb_local_planner::TebOptimalPlanner planner(node, cfg, &obstacles, nullptr, &via_points);

  ControlLatencyPredictor predictor;
  ControlLatencyPredictor::Params params;
  params.latency = compensated_latency;
  predictor.setParams(params);

  const teb_local_planner::PoseSE2 goal(-1.5, 0.0, 0.0);
  ControlLatencyPredictor::State robot;
  robot.v = -0.5;
  // commands in flight: the robot was told to reverse at 0.5 m/s before the approach started
  std::deque<std::pair<double, double>> in_flight{{-kDeadTime, -0.5}};

  ApproachResult result;
  std::vector<double> commands;
  const int plant_steps_per_cycle = static_cast<int>(std::lround(kControlPeriod / kPlantStep));
  for (int cycle = 0; cycle * kControlPeriod < 10.0; ++cycle)
  {
    const double t = cycle * kControlPeriod;
    ControlLatencyPredictor::State start;
    predictor.predict(t, robot, start);
    geometry_msgs::msg::Twist start_vel;
    start_vel.linear.x = start.v;
    start_vel.angular.z = start.omega;
    double vx = 0.0, vy = 0.0, omega = 0.0;
    if (planner.plan(teb_local_planner::PoseSE2(start.x, start.y, start.theta), goal, &start_vel, false))
      planner.getVelocityCommand(vx, vy, omega, cfg.trajectory.control_look_ahead_poses);
    else
      planner.clearPlanner();
    vx = std::clamp(vx, -cfg.robot.max_vel_x_backwards, cfg.robot.max_vel_x);
    predictor.addCommand(t, vx, omega);
    commands.push_back(vx);
    in_flight.emplace_back(t, vx);

    for (int i = 0; i < plant_steps_per_cycle; ++i)
    {
      const double ts = t + i * kPlantStep;
      while (in_flight.size() > 1 && in_flight[1].first + kDeadTime <= ts)
        in_flight.pop_front();
      const double max_dv = kPlantMaxAccel * kPlantStep;
      robot.v += std::clamp((in_flight.front().second - robot.v) * kPlantStep / kLag, -max_dv, max_dv);
      robot.x += robot.v * kPlantStep;  // straight approach, no turn commanded
      const double dist = std::fabs(robot.x - goal.x());
      if (result.speed_at_tolerance < 0.0 && dist < kGoalTolerance)
        result.speed_at_tolerance = std::fabs(robot.v);
      result.overshoot = std::max(result.overshoot, goal.x() - robot.x);
    }
  }
  const int window = static_cast<int>(std::lround(0.25 / kControlPeriod));
  for (size_t k = 0; k + window < commands.size(); ++k)
    result.max_commanded_decel =
        std::max(result.max_commanded_decel, (std::fabs(commands[k]) - std::fabs(commands[k + window])) / 0.25);
  result.final_error = std::fabs(robot.x - goal.x());
  return result;
}

class ControlLatencyTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  nav2::LifecycleNode::SharedPtr node_ = std::make_shared<nav2::LifecycleNode>("control_latency_test");
};

TEST_F(ControlLatencyTest, compensationKeepsTheDecelerationAndStopsAtTheGoal)
{
  const ApproachResult plain = reverseApproach(node_, 0.0);
  const ApproachResult compensated = reverseApproach(node_, kDeadTime + kLag);
  RCLCPP_INFO(node_->get_logger(),
              "plain: decel %.2f m/s^2, %.2f m/s at tolerance, overshoot %.3f m, final error %.3f m | "
              "compensated: decel %.2f m/s^2, %.2f m/s at tolerance, overshoot %.3f m, final error %.3f m",
              plain.max_commanded_decel, plain.speed_at_tolerance, plain.overshoot, plain.final_error,
              compensated.max_commanded_decel, compensated.speed_at_tolerance, compensated.overshoot,
              compensated.final_error);

  // the incident: TEB brakes late and hard against acc_lim_x 0.4 and the robot rolls past the goal
  EXPECT_GT(plain.max_commanded_decel, 0.56);
  EXPECT_GT(plain.overshoot, 0.05);

  EXPECT_LT(compensated.max_commanded_decel, 0.44);
  EXPECT_LT(compensated.overshoot, 0.02);
  EXPECT_LT(compensated.final_error, 0.01);
  EXPECT_LT(compensated.speed_at_tolerance, plain.speed_at_tolerance);
}

// The plant does not follow (collision monitor stop): TEB must plan from the measured standstill,
// not from the motion its own commands would have caused.
TEST(ControlLatencyPredictor, plantNotFollowingFallsBackToTheMeasuredState)
{
  ControlLatencyPredictor predictor;
  ControlLatencyPredictor::Params params;
  params.latency = 0.45;
  predictor.setParams(params);
  for (int k = 0; k < 20; ++k)
    predictor.addCommand(k * kControlPeriod, 0.5, 0.0);

  ControlLatencyPredictor::State held;  // standing although 0.5 m/s was commanded for 1 s
  ControlLatencyPredictor::State start;
  EXPECT_FALSE(predictor.predict(1.0, held, start));
  EXPECT_DOUBLE_EQ(start.x, 0.0);
  EXPECT_DOUBLE_EQ(start.v, 0.0);

  ControlLatencyPredictor::State following;
  following.v = 0.5;
  ASSERT_TRUE(predictor.predict(1.0, following, start));
  EXPECT_NEAR(start.x, 0.5 * 0.45, 1e-6);
  EXPECT_NEAR(start.v, 0.5, 1e-9);
}

// Steering model: a turn commanded just now has not reached the wheel within the window (dead time),
// so the predicted heading and start turn rate follow the slow wheel, not the command.
TEST(ControlLatencyPredictor, steeringModelFollowsTheWheelNotTheCommand)
{
  ControlLatencyPredictor predictor;
  ControlLatencyPredictor::Params params;
  params.latency = 0.45;
  params.steering_latency = 0.3;
  params.steering_rate_limit = 0.35;  // ~20 deg/s
  params.wheelbase = 1.2526;
  predictor.setParams(params);
  // straight at 0.5 m/s, then a turn with a wheel angle of ~0.5 rad from t = 0.8 s
  const double omega_turn = 0.5 * std::tan(0.5) / params.wheelbase;
  for (int k = 0; k <= 20; ++k)
    predictor.addCommand(k * kControlPeriod, 0.5, k >= 16 ? omega_turn : 0.0);

  ControlLatencyPredictor::State measured;
  measured.v = 0.5;
  ControlLatencyPredictor::State start;
  ASSERT_TRUE(predictor.predict(1.0, measured, start));
  // the wheel starts moving at 1.1 s and reaches 0.35 * 0.35 rad at 1.45 s
  const double wheel = 0.35 * 0.35;
  EXPECT_NEAR(start.omega, 0.5 * std::tan(wheel) / params.wheelbase, 1e-3);
  EXPECT_LT(start.omega, 0.5 * omega_turn);
  EXPECT_GT(start.theta, 0.0);
  EXPECT_LT(start.theta, 0.02);
}

}  // namespace
