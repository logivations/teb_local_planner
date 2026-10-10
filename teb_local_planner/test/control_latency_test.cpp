// Control latency compensation (ControlLatencyPredictor) with the real TEB optimizer in a closed loop
// against a drive whose measured motion follows the command ~0.45 s late (amr47 traction, 2026-10-10).

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "teb_local_planner/control_latency_predictor.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/robot_footprint_model.h"
#include "teb_local_planner/teb_config.h"
#include "teb_local_planner/teb_local_planner_ros.h"

using teb_local_planner::ControlLatencyPredictor;

namespace
{

constexpr double kControlPeriod = 0.05;  // controller_frequency 20 Hz
constexpr double kPlantStep = 0.005;
constexpr double kDeadTime = 0.25;  // s, command -> drive reacts
constexpr double kLag = 0.2;  // s, first-order lag of the drive; with the dead time ~0.45 s as on amr47
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
// follows each command kDeadTime later with a first-order lag kLag (rate limited). The predictor is
// fed with TEB's commands: nothing downstream changes them here.
ApproachResult reverseApproach(const nav2::LifecycleNode::SharedPtr& node, double compensated_latency,
                               double compensated_lag = 0.0)
{
  teb_local_planner::TebConfig cfg;
  configureGeneralFollowPath(cfg);
  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::ViaPointContainer via_points;
  teb_local_planner::TebOptimalPlanner planner(node, cfg, &obstacles, nullptr, &via_points);

  ControlLatencyPredictor predictor;
  ControlLatencyPredictor::Params params;
  params.latency = compensated_latency;
  params.lag = compensated_lag;
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

// ---------------------------------------------------------------------------------------------
// Tricycle drive with TEB's downstream as in deep_cv: the flickering protection turns on the spot
// (v = 0) instead of flipping the wheel by ~180 deg, and the drive only drives once its wheel has
// reached the commanded angle (rate limited). At low speed many commands TEB sends are not executed
// as sent.

constexpr double kWheelbase = 1.2526;
constexpr double kWheelRate = 0.35;  // rad/s, steering wheel rate of the drive
constexpr double kSteeringDeadTime = 0.3;  // s, command -> wheel starts turning

double impliedWheelAngle(double v, double omega, double fallback)
{
  if (std::fabs(v) < 1e-3)
    return omega == 0.0 ? fallback : std::copysign(M_PI_2, omega);
  return std::atan(kWheelbase * omega / v);
}

// teb_flickering_protection.cpp (deep_cv) prevent_flickering(): a wheel demand that flips between the
// two sides (both beyond 1.2 rad) becomes a turn on the spot until the demand is below 1.0 rad.
struct FlickerFilter
{
  bool turning_on_spot = false;
  double last_angle = 0.0;
  double last_v = 0.0;

  void apply(double& v, double& omega)
  {
    const double angle = impliedWheelAngle(v, omega, 0.0);
    if (turning_on_spot)
    {
      if (std::fabs(angle) <= 1.0)
      {
        turning_on_spot = false;
        return;
      }
      v = last_v < 0.0 ? -0.01 : 0.0;
      return;
    }
    if ((last_angle < 0) != (angle < 0) && std::fabs(angle) >= 1.2 && std::fabs(last_angle) >= 1.2 &&
        std::fabs(omega) >= 0.2)
    {
      turning_on_spot = true;
      v = last_v < 0.0 ? -0.01 : 0.0;
      return;
    }
    last_angle = angle;
    last_v = v;
  }
};

enum class Feed
{
  kOwnCommands,  // predictor fed with TEB's own commands (control_latency_executed_cmd_topic empty)
  kExecutedCommands,  // predictor fed with the commands after the flickering protection
};

struct ManeuverResult
{
  int direction_flips = 0;  // sign changes of TEB's linear command (|v| >= 0.02 m/s)
  int wheel_flips = 0;  // wheel demand of TEB's command jumping by more than 90 deg
  double max_start_offset = 0.0;  // m, plan start vs. measured pose while the robot stands
  double time_to_goal = -1.0;  // s until within 5 cm / 0.1 rad (the goal checker ends the run), -1 = never
};

// Robot standing at the origin with the wheel straight, goal 1 m ahead and 0.4 m to the side: TEB
// has to steer hard at low speed. 20 s at most.
ManeuverResult tricycleManeuver(const nav2::LifecycleNode::SharedPtr& node, Feed feed,
                                const ControlLatencyPredictor::Params& params)
{
  const teb_local_planner::PoseSE2 goal(1.0, 0.4, 0.0);
  teb_local_planner::TebConfig cfg;
  configureGeneralFollowPath(cfg);
  cfg.robot.wheelbase = kWheelbase;
  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::ViaPointContainer via_points;
  teb_local_planner::TebOptimalPlanner planner(node, cfg, &obstacles, nullptr, &via_points);

  ControlLatencyPredictor predictor;
  predictor.setParams(params);
  FlickerFilter flicker;

  struct
  {
    double x = 0.0, y = 0.0, theta = 0.0;
    double delta = 0.0;  // wheel angle
    double s = 0.0;  // wheel speed
  } robot;
  struct Sent
  {
    double t, v, omega;
  };
  std::deque<Sent> in_flight{{-1.0, 0.0, 0.0}};
  auto commandAt = [&in_flight](double t) {
    Sent c = in_flight.front();
    for (const Sent& s : in_flight)
      if (s.t <= t)
        c = s;
    return c;
  };
  double wheel_target = 0.0;

  ManeuverResult result;
  double last_dir = 0.0, last_wheel = 0.0;
  bool have_wheel = false;
  const int plant_steps_per_cycle = static_cast<int>(std::lround(kControlPeriod / kPlantStep));
  for (int cycle = 0; cycle * kControlPeriod < 20.0; ++cycle)
  {
    const double t = cycle * kControlPeriod;
    if (std::hypot(robot.x - goal.x(), robot.y - goal.y()) < 0.05 &&
        std::fabs(std::remainder(robot.theta - goal.theta(), 2 * M_PI)) < 0.1)
    {
      result.time_to_goal = t;
      break;
    }
    const ControlLatencyPredictor::State measured{robot.x, robot.y, robot.theta, robot.s * std::cos(robot.delta),
                                                  robot.s * std::sin(robot.delta) / kWheelbase};
    ControlLatencyPredictor::State start;
    predictor.predict(t, measured, start);
    if (std::fabs(measured.v) < 0.02)
      result.max_start_offset = std::max(result.max_start_offset, std::hypot(start.x - robot.x, start.y - robot.y));
    geometry_msgs::msg::Twist start_vel;
    start_vel.linear.x = start.v;
    start_vel.angular.z = start.omega;
    double vx = 0.0, vy = 0.0, omega = 0.0;
    if (planner.plan(teb_local_planner::PoseSE2(start.x, start.y, start.theta), goal, &start_vel, false))
      planner.getVelocityCommand(vx, vy, omega, cfg.trajectory.control_look_ahead_poses);
    else
      planner.clearPlanner();
    vx = std::clamp(vx, -cfg.robot.max_vel_x_backwards, cfg.robot.max_vel_x);

    if (std::fabs(vx) >= 0.02)
    {
      const double dir = vx > 0.0 ? 1.0 : -1.0;
      result.direction_flips += last_dir != 0.0 && dir != last_dir;
      last_dir = dir;
    }
    if (std::fabs(vx) >= 1e-3 || std::fabs(omega) >= 1e-3)
    {
      const double wheel = impliedWheelAngle(vx, omega, last_wheel);
      result.wheel_flips += have_wheel && std::fabs(wheel - last_wheel) > M_PI_2;
      last_wheel = wheel;
      have_wheel = true;
    }

    double executed_v = vx, executed_omega = omega;
    flicker.apply(executed_v, executed_omega);
    if (feed == Feed::kExecutedCommands)
      predictor.addCommand(t, executed_v, executed_omega);
    else
      predictor.addCommand(t, vx, omega);
    in_flight.push_back({t, executed_v, executed_omega});
    while (in_flight.size() > 2 && in_flight[1].t < t - 1.0)
      in_flight.pop_front();

    for (int i = 0; i < plant_steps_per_cycle; ++i)
    {
      const double ts = t + i * kPlantStep;
      const Sent steering = commandAt(ts - kSteeringDeadTime);
      wheel_target = impliedWheelAngle(steering.v, steering.omega, wheel_target);
      robot.delta += std::clamp(wheel_target - robot.delta, -kWheelRate * kPlantStep, kWheelRate * kPlantStep);
      // wheel speed of the traction command; the drive waits until the wheel has the commanded angle
      const Sent traction = commandAt(ts - kDeadTime);
      const double traction_angle = impliedWheelAngle(traction.v, traction.omega, robot.delta);
      double s_cmd = std::fabs(std::cos(traction_angle)) > 0.05
                         ? traction.v / std::cos(traction_angle)
                         : traction.omega * kWheelbase / std::sin(traction_angle);
      if (std::fabs(robot.delta - traction_angle) > 0.15)
        s_cmd = 0.0;
      const double max_ds = kPlantMaxAccel * kPlantStep;
      robot.s += std::clamp((s_cmd - robot.s) * kPlantStep / kLag, -max_ds, max_ds);
      const double v = robot.s * std::cos(robot.delta);
      const double w = robot.s * std::sin(robot.delta) / kWheelbase;
      const double theta_mid = robot.theta + 0.5 * w * kPlantStep;
      robot.x += v * std::cos(theta_mid) * kPlantStep;
      robot.y += v * std::sin(theta_mid) * kPlantStep;
      robot.theta += w * kPlantStep;
    }
  }
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
  const ApproachResult with_lag = reverseApproach(node_, kDeadTime, kLag);
  RCLCPP_INFO(node_->get_logger(),
              "plain: decel %.2f m/s^2, %.2f m/s at tolerance, overshoot %.3f m, final error %.3f m | "
              "compensated: decel %.2f m/s^2, %.2f m/s at tolerance, overshoot %.3f m, final error %.3f m | "
              "dead time + lag model: decel %.2f m/s^2, %.2f m/s at tolerance, overshoot %.3f m, final error %.3f m",
              plain.max_commanded_decel, plain.speed_at_tolerance, plain.overshoot, plain.final_error,
              compensated.max_commanded_decel, compensated.speed_at_tolerance, compensated.overshoot,
              compensated.final_error, with_lag.max_commanded_decel, with_lag.speed_at_tolerance, with_lag.overshoot,
              with_lag.final_error);

  // the incident: TEB brakes late and hard against acc_lim_x 0.4 and the robot rolls past the goal
  EXPECT_GT(plain.max_commanded_decel, 0.56);
  EXPECT_GT(plain.overshoot, 0.05);

  for (const ApproachResult& r : {compensated, with_lag})
  {
    EXPECT_LT(r.overshoot, 0.02);
    EXPECT_LT(r.final_error, 0.01);
    EXPECT_LT(r.speed_at_tolerance, plain.speed_at_tolerance);
  }
  EXPECT_LT(compensated.max_commanded_decel, 0.44);
  // the lag model starts the plan with the speed the lagging drive will still have, TEB asks for more
  // deceleration than with the newest command as start velocity (the drive delivers less of it)
  EXPECT_LT(with_lag.max_commanded_decel, 0.56);
}

// amr47 2026-10-10 (put 55198): near standstill the drive did not execute what TEB sent (flickering
// protection, wheel still turning), the predictor integrated TEB's own commands anyway, the plan start
// ran up to 12 cm away from the standing robot and TEB flipped its wheel demand -86 <-> +89 deg until
// the progress checker gave up. With the executed commands, the dead time + lag model and the
// standstill / direction gates TEB steers like without compensation.
TEST_F(ControlLatencyTest, standstillManeuverDoesNotChurn)
{
  ControlLatencyPredictor::Params uncompensated;  // latency 0
  const ManeuverResult plain = tricycleManeuver(node_, Feed::kExecutedCommands, uncompensated);

  ControlLatencyPredictor::Params live_test;  // amr47 2026-10-10 19:06
  live_test.latency = 0.4;
  live_test.steering_latency = kSteeringDeadTime;
  live_test.steering_rate_limit = kWheelRate;
  live_test.wheelbase = kWheelbase;
  const ManeuverResult own = tricycleManeuver(node_, Feed::kOwnCommands, live_test);

  ControlLatencyPredictor::Params fixed = live_test;
  fixed.latency = kDeadTime;
  fixed.lag = kLag;
  fixed.min_speed = 0.05;
  const ManeuverResult executed = tricycleManeuver(node_, Feed::kExecutedCommands, fixed);

  for (const auto& [name, r] : {std::pair{"plain", plain}, {"own commands", own}, {"executed commands", executed}})
    RCLCPP_INFO(node_->get_logger(),
                "%s: %d direction flips, %d wheel flips, plan start up to %.3f m off the standing robot, goal after "
                "%.2f s",
                name, r.direction_flips, r.wheel_flips, r.max_start_offset, r.time_to_goal);

  ASSERT_GT(plain.time_to_goal, 0.0);
  // the incident: the plan starts far from the standing robot, the wheel demand flips, the goal takes long
  EXPECT_GT(own.max_start_offset, 0.1);
  EXPECT_GE(own.direction_flips, 2);
  EXPECT_TRUE(own.time_to_goal < 0.0 || own.time_to_goal > plain.time_to_goal + 4.0);

  EXPECT_LT(executed.max_start_offset, 0.05);
  EXPECT_LE(executed.wheel_flips, 1);
  EXPECT_LE(executed.direction_flips, 1);
  ASSERT_GT(executed.time_to_goal, 0.0);
  EXPECT_LT(executed.time_to_goal, plain.time_to_goal + 1.0);
}

// The plugin with control_latency_executed_cmd_topic: the robot drives 0.15 m/s because the collision
// monitor caps TEB's 0.5 m/s. The prediction integrates (and starts the plan with) the 0.15 m/s the
// drive receives. With TEB's own commands the start velocity is TEB's last output, every plan
// accelerates from there and the commands run away from the robot until the mismatch gate drops the
// prediction, then start over (command steps 4x larger on amr47 2026-10-10).
class GoalCheckerStub : public nav2_core::GoalChecker
{
public:
  void initialize(const nav2::LifecycleNode::WeakPtr&, const std::string&,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose&, const geometry_msgs::msg::Pose&, const geometry_msgs::msg::Twist&,
                     const nav_msgs::msg::Path&) override
  {
    return false;
  }
  bool getTolerances(geometry_msgs::msg::Pose& pose_tolerance, geometry_msgs::msg::Twist&) override
  {
    pose_tolerance.position.x = pose_tolerance.position.y = 0.16;
    return true;
  }
};

struct CappedDriveResult
{
  std::vector<double> commands;  // TEB's linear commands
  std::vector<double> start_offsets;  // m, start of TEB's local plan ahead of the measured pose
};

CappedDriveResult commandsWithCappedDrive(const std::string& executed_cmd_topic)
{
  constexpr double kCap = 0.15;  // collision monitor limit
  const std::string name = "FollowPath";
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides({
      {name + ".control_latency_s", 0.4},
      {name + ".control_latency_min_speed", 0.05},
      {name + ".control_latency_executed_cmd_topic", executed_cmd_topic},
      {name + ".enable_homotopy_class_planning", false},
      {name + ".max_vel_x", 0.5},
      {name + ".acc_lim_x", 0.4},
      {name + ".dt_ref", 0.4},
      {name + ".allow_init_with_backwards_motion", false},
  });
  auto node = std::make_shared<nav2::LifecycleNode>("controller_server", options);
  auto drive = std::make_shared<rclcpp::Node>("collision_monitor", rclcpp::NodeOptions().use_intra_process_comms(true));
  auto executed_pub = drive->create_publisher<geometry_msgs::msg::TwistStamped>("collision_monitor/cmd_vel", 10);
  double plan_start_x = std::nan("");
  auto local_plan_sub = drive->create_subscription<nav_msgs::msg::Path>(
      "local_plan", 10, [&plan_start_x](nav_msgs::msg::Path::ConstSharedPtr msg) {
        if (!msg->poses.empty())
          plan_start_x = msg->poses.front().pose.position.x;
      });

  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped map_to_base;
  map_to_base.header.frame_id = "map";
  map_to_base.child_frame_id = "base_link";
  map_to_base.transform.rotation.w = 1.0;
  tf->setTransform(map_to_base, "test", true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
  costmap->on_configure(rclcpp_lifecycle::State());

  teb_local_planner::TebLocalPlannerROS teb;
  teb.configure(node, name, tf, costmap);
  teb.activate();

  // straight 4 m along x at y = 2.5 (inside the 5 x 5 m costmap)
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (double x = 0.5; x <= 4.5; x += 0.05)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = x;
    p.pose.position.y = 2.5;
    p.pose.orientation.w = 1.0;
    plan.poses.push_back(p);
  }
  teb.newPathReceived(plan);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(drive);
  GoalCheckerStub goal_checker;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.5;
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = kCap;

  CappedDriveResult result;
  auto next = std::chrono::steady_clock::now();
  for (int cycle = 0; cycle < 40; ++cycle)
  {
    executor.spin_some();
    pose.header.stamp = node->now();
    pose.pose.position.x += kCap * kControlPeriod;
    double v = 0.0;
    try
    {
      v = teb.computeVelocityCommands(pose, velocity, &goal_checker, nav_msgs::msg::Path(), pose).twist.linear.x;
    }
    catch (const std::exception& e)
    {
      ADD_FAILURE() << e.what();
    }
    result.commands.push_back(v);
    auto executed = std::make_unique<geometry_msgs::msg::TwistStamped>();
    executed->header.stamp = node->now();
    executed->twist.linear.x = std::min(v, kCap);
    executed_pub->publish(std::move(executed));
    plan_start_x = std::nan("");
    executor.spin_some();
    result.start_offsets.push_back(plan_start_x - pose.pose.position.x);
    next += std::chrono::milliseconds(50);
    std::this_thread::sleep_until(next);
  }
  teb.deactivate();
  teb.cleanup();
  return result;
}

TEST_F(ControlLatencyTest, predictionUsesTheExecutedCommands)
{
  const CappedDriveResult executed = commandsWithCappedDrive("collision_monitor/cmd_vel");
  const CappedDriveResult own = commandsWithCappedDrive("");
  // after the first 0.5 s (buffer filled)
  auto maxStep = [](const std::vector<double>& c) {
    double step = 0.0;
    for (size_t k = 11; k < c.size(); ++k)
      step = std::max(step, std::fabs(c[k] - c[k - 1]));
    return step;
  };
  auto range = [](const std::vector<double>& c) {
    const auto [lo, hi] = std::minmax_element(c.begin() + 10, c.end());
    return std::pair{*lo, *hi};
  };
  RCLCPP_INFO(node_->get_logger(),
              "drive capped at 0.15 m/s | executed commands: plan start %.3f..%.3f m ahead, command steps up to %.3f "
              "m/s | own commands: plan start %.3f..%.3f m ahead, command steps up to %.3f m/s",
              range(executed.start_offsets).first, range(executed.start_offsets).second, maxStep(executed.commands),
              range(own.start_offsets).first, range(own.start_offsets).second, maxStep(own.commands));

  // executed commands: the plan starts where 0.15 m/s for control_latency_s takes the robot
  EXPECT_NEAR(range(executed.start_offsets).first, 0.15 * 0.4, 0.015);
  EXPECT_NEAR(range(executed.start_offsets).second, 0.15 * 0.4, 0.015);
  EXPECT_LT(maxStep(executed.commands), 0.05);
  // own commands: TEB believes it drives what it commanded, the start runs ahead of the robot until
  // the mismatch gate drops the prediction, and the commands jump
  EXPECT_GT(range(own.start_offsets).second, 0.1);
  EXPECT_GT(maxStep(own.commands), 0.08);
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
