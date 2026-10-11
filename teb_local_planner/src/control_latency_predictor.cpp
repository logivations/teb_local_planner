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
// m/s, a speed (command or measured) at or above this has a direction; below it is standstill (the
// flickering protection holds the drive direction with -0.01 m/s while it turns on the spot)
constexpr double kMinDirectionSpeed = 0.02;
// rad, a forward/reverse change with a wheel angle beyond this turns the wheel by more than 90 deg
constexpr double kMinFlipSteeringAngle = M_PI_4;
constexpr double kMaxCommandAhead = 0.1;  // s, a newest command further ahead of the cycle time is a clock jump
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

bool ControlLatencyPredictor::wheelFlips(double t, double window, double measured_v) const
{
  bool forward = measured_v >= kMinDirectionSpeed;
  bool reverse = measured_v <= -kMinDirectionSpeed;
  bool steered = false;
  // the command in effect at t - window and every later one
  const Command* first = commandAt(t - window);
  for (const Command& c : commands_)
  {
    if (first ? &c < first : c.t < t - window)
      continue;
    forward = forward || c.v >= kMinDirectionSpeed;
    reverse = reverse || c.v <= -kMinDirectionSpeed;
    steered = steered || std::fabs(c.delta) >= kMinFlipSteeringAngle;
  }
  return forward && reverse && steered;
}

bool ControlLatencyPredictor::predict(double t, const State& measured, State& predicted)
{
  predicted = measured;
  skip_reason_ = "";
  if (!enabled())
  {
    skip_reason_ = "disabled";
    return false;
  }
  // (executed commands are stamped on arrival and may be a little newer than the cycle stamp t)
  if (commands_.empty() || t - commands_.back().t > params_.max_command_gap ||
      commands_.back().t - t > kMaxCommandAhead)
  {
    // the commands do not describe what the robot does now (another controller drove, clock jump)
    commands_.clear();
    skip_reason_ = "no recent command";
    return false;
  }

  const double latency = params_.latency;
  // Standstill: nothing is under way that the plan has to account for. Planning from a pose rolled
  // forward with the last small commands only makes TEB chase its own corrections around a robot
  // that does not move (amr47 2026-10-10: forward/reverse flip every cycle, wheel demand +-90 deg).
  if (std::fabs(measured.v) < params_.min_speed && std::fabs(commands_.back().v) < params_.min_speed)
  {
    skip_reason_ = "standstill";
    return false;
  }
  // Forward/reverse change while steering (car-like model): the same turn rate in the other direction
  // needs the wheel on the other side (more than 90 deg away), the drive turns the wheel before it
  // moves, which the dead-time model does not describe. A straight reversal (wheel stays) is predicted.
  if (wheelFlips(t, std::max(latency + params_.lag, params_.steering_latency), measured.v))
  {
    skip_reason_ = "wheel flips";
    return false;
  }
  // The measured speed should be the command sent one latency ago. If it is not, something else
  // decides how the robot moves (collision monitor, speed limits downstream, traction stuck).
  const double lag = params_.lag;
  const Command* in_effect = commandAt(t - latency - lag);
  if (in_effect && std::fabs(measured.v - in_effect->v) > params_.max_velocity_error)
  {
    skip_reason_ = "drive does not follow";
    return false;
  }

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

  // speed and turn rate at time t + tau (from the command of t + tau - latency), dead time only
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
  // first-order lag of the drive: the speed starts at the measured one and follows the commands
  const double lag_gain = lag > 0.0 ? 1.0 - std::exp(-h / lag) : 1.0;
  for (int i = 0; i < steps; ++i)
  {
    const double tau = (i + 0.5) * h;
    const double v_before = v;
    if (lag > 0.0)
    {
      double v_cmd, omega_cmd;
      velocityAt(tau, v_cmd, omega_cmd);
      const Command* c = commandAt(t + tau - latency);
      v += (v_cmd - v) * lag_gain;
      if (c && std::fabs(c->v) >= kMinCurvatureSpeed)
        omega = v * c->omega / c->v;  // keep the commanded curvature
      else
        omega += (omega_cmd - omega) * lag_gain;
    }
    else
    {
      velocityAt(tau, v, omega);
    }
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
    const double v_mid = lag > 0.0 ? 0.5 * (v_before + v) : v;
    x += v_mid * std::cos(theta_mid) * h;
    y += v_mid * std::sin(theta_mid) * h;
    theta += omega * h;
  }

  // Start velocity of the plan: the speed the model reaches at t + latency. Dead time only: the
  // newest command the drive received. With the lag: the speed rolled forward from the measured one,
  // in which the newest command has only a small share, so TEB's output is not fed back as its next
  // start velocity.
  if (lag <= 0.0)
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
