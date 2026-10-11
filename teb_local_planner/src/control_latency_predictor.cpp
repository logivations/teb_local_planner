/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Logivations GmbH.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the conditions of the
 *  BSD license of this package are met.
 *
 *********************************************************************/

#include "teb_local_planner/control_latency_predictor.h"

#include <algorithm>
#include <cmath>

namespace teb_local_planner
{

namespace
{
constexpr double kIntegrationStep = 0.01;  // s
constexpr double kMinSteeringSpeed = 0.01;  // m/s, below this a command implies no wheel angle
constexpr double kMinMeasuredSteeringSpeed = 0.05;  // m/s, below this the measured turn rate gives no wheel angle
constexpr double kMinCurvatureSpeed = 0.05;  // m/s, below this the commanded turn rate is taken as is
}  // namespace

double ControlLatencyPredictor::impliedSteeringAngle(double v, double omega, double fallback) const
{
  if (params_.wheelbase <= 0.0 || std::fabs(v) < kMinSteeringSpeed)
    return fallback;
  const double delta = std::atan(params_.wheelbase * omega / v);
  return std::clamp(delta, -params_.max_steering_angle, params_.max_steering_angle);
}

void ControlLatencyPredictor::addCommand(double t, double v, double omega)
{
  if (!commands_.empty() && t < commands_.back().t)
    commands_.clear();  // clock jumped back
  const double previous_delta = commands_.empty() ? 0.0 : commands_.back().delta;
  commands_.push_back({t, v, omega, impliedSteeringAngle(v, omega, previous_delta)});

  // keep the command in effect at the oldest time a prediction looks up
  const double oldest = t - std::max(params_.latency, params_.steering_latency) - params_.max_command_gap;
  while (commands_.size() > 1 && commands_[1].t <= oldest)
    commands_.pop_front();
}

const ControlLatencyPredictor::Command* ControlLatencyPredictor::commandAt(double t) const
{
  if (commands_.empty() || t < commands_.front().t)
    return nullptr;
  auto it = std::upper_bound(commands_.begin(), commands_.end(), t,
                             [](double time, const Command& c) { return time < c.t; });
  return &*std::prev(it);
}

bool ControlLatencyPredictor::predict(double t, const State& measured, State& predicted)
{
  predicted = measured;
  if (!enabled())
    return false;
  if (commands_.empty() || t - commands_.back().t > params_.max_command_gap || t < commands_.back().t)
  {
    // the commands do not describe what the robot does now (another controller drove, clock jump)
    commands_.clear();
    return false;
  }

  const double latency = params_.latency;
  // The measured speed should be the command sent one latency ago. If it is not, something else
  // decides how the robot moves (collision monitor, speed limits downstream, traction stuck).
  const Command* in_effect = commandAt(t - latency);
  if (in_effect && std::fabs(measured.v - in_effect->v) > params_.max_velocity_error)
    return false;

  const bool steering = steeringModelEnabled();
  const double wheelbase = params_.wheelbase;
  double delta = 0.0;
  if (steering)
  {
    if (std::isfinite(measured.steering))
      delta = measured.steering;
    else if (std::fabs(measured.v) >= kMinMeasuredSteeringSpeed)
      delta = impliedSteeringAngle(measured.v, measured.omega, 0.0);
    else if (const Command* c = commandAt(t - params_.steering_latency))
      delta = c->delta;  // standstill: assume the wheel reached the angle commanded one dead time ago
  }

  // speed and turn rate at time t + tau (from the command of t + tau - latency)
  auto velocityAt = [&](double tau, double& v, double& omega) {
    const Command* c = commandAt(t + tau - latency);
    if (!c)
    {
      // before the first command TEB sent: the robot keeps its measured motion
      v = measured.v;
      omega = measured.omega;
      return;
    }
    v = c->v;
    if (std::fabs(c->v) >= kMinCurvatureSpeed)
      omega = v * c->omega / c->v;  // keep the commanded curvature
    else
      omega = c->omega;
  };

  const int steps = std::max(1, static_cast<int>(std::ceil(latency / kIntegrationStep)));
  const double h = latency / steps;
  double x = measured.x, y = measured.y, theta = measured.theta;
  double v = measured.v, omega = measured.omega;
  for (int i = 0; i < steps; ++i)
  {
    const double tau = (i + 0.5) * h;
    velocityAt(tau, v, omega);
    if (steering)
    {
      const Command* c = commandAt(t + tau - params_.steering_latency);
      const double target = c ? c->delta : delta;
      if (params_.steering_rate_limit > 0.0)
      {
        const double max_step = params_.steering_rate_limit * h;
        delta += std::clamp(target - delta, -max_step, max_step);
      }
      else
      {
        delta = target;
      }
      omega = v * std::tan(delta) / wheelbase;
    }
    const double theta_mid = theta + 0.5 * omega * h;
    x += v * std::cos(theta_mid) * h;
    y += v * std::sin(theta_mid) * h;
    theta += omega * h;
  }

  // start velocity of the plan: the newest command is the one in effect at t + latency
  velocityAt(latency, v, omega);
  if (steering)
    omega = v * std::tan(delta) / wheelbase;

  predicted.x = x;
  predicted.y = y;
  predicted.theta = std::remainder(theta, 2.0 * M_PI);
  predicted.v = v;
  predicted.omega = omega;
  predicted.steering = steering ? delta : std::numeric_limits<double>::quiet_NaN();
  return true;
}

}  // namespace teb_local_planner
