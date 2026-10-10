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

#ifndef CONTROL_LATENCY_PREDICTOR_H_
#define CONTROL_LATENCY_PREDICTOR_H_

#include <deque>

namespace teb_local_planner
{

/**
 * @brief Predicts the robot state at the time the next command takes effect (Smith predictor).
 *
 * TEB plans from the measured pose and velocity. If the drive needs control_latency_s until a command
 * shows up in the measured motion, the robot keeps doing what the commands of the last
 * control_latency_s told it to while TEB already plans the next one: near a goal it keeps rolling
 * while TEB brakes, the remaining distance shrinks faster than planned and every cycle demands a
 * harder deceleration (amr47 2026-10-10, ~0.45 s: 0.3 -> 1.0 m/s^2 against acc_lim_x 0.4, legs
 * ending at 0.12-0.48 m/s or overshooting).
 *
 * The predictor keeps the commands the drive received and integrates the measured pose over the
 * latency window with them (dead-time model: the motion at now + tau is the command of now + tau -
 * latency). The plan then starts at the predicted pose with the newest of these commands as start
 * velocity, so the command TEB computes is the one for the moment it takes effect.
 *
 * Which commands: the ones the drive executes, i.e. after everything downstream of TEB that changes
 * them (flickering protection, velocity smoother, collision monitor). Integrating TEB's own commands
 * instead lets the prediction run away from the robot whenever the downstream zeroes or caps them,
 * and feeds TEB's last output back as its next start velocity. On amr47 (2026-10-10) this made TEB
 * flip forward/reverse every cycle at standstill (start predicted up to 12 cm off the standing
 * robot, wheel demand -86 <-> +89 deg) until the progress checker gave up. TebLocalPlannerROS feeds
 * the executed commands when control_latency_executed_cmd_topic is set, else TEB's own ones (only
 * right when nothing downstream changes them, e.g. in simulation).
 *
 * The prediction is also skipped near standstill (measured speed and newest command below
 * min_speed: nothing is under way that the plan has to account for, and the planner should see the
 * robot where it stands) and, with a wheelbase, when the commands of the window or the measured
 * motion change between forward and reverse at a wheel angle beyond 45 deg (the wheel has to turn
 * to the other side before the drive moves, the dead-time model does not describe that).
 *
 * Optional first-order lag after the dead time (lag > 0): the speed is rolled forward from the
 * measured one and follows the commands with this time constant, so the start velocity of the plan
 * is the speed the robot will have, not TEB's newest command. Pure dead time (lag = 0) starts the
 * plan with the newest command.
 *
 * The plant may not follow TEB (collision monitor, speed limits downstream, traction stuck): if the
 * measured speed differs from the command sent one latency ago by more than max_velocity_error, the
 * prediction is skipped for that cycle and TEB plans from the measured state as without compensation.
 * Smaller differences are not corrected: carrying the measured difference over the window feeds
 * measurement noise and the cycle-to-cycle jitter of TEB's commands back into the start velocity
 * and makes TEB chatter around the goal.
 *
 * Optional steering model for car-like robots (tricycle: omega = v * tan(delta) / wheelbase): the
 * wheel angle follows the commanded angle with steering_latency_s dead time and at most
 * steering_rate_limit; without it the turn rate keeps the commanded curvature.
 *
 * Pure, ROS-independent and not thread-safe (owned by one controller instance).
 */
class ControlLatencyPredictor
{
public:
  struct Params
  {
    double latency = 0.0;  //!< [s] dead time command -> drive reacts (with lag = 0: -> measured motion); 0 disables the predictor
    double lag = 0.0;  //!< [s] time constant of the drive's first-order speed response after the dead time; 0 = none
    double steering_latency = 0.0;  //!< [s] dead time command -> wheel angle (steering model only)
    double steering_rate_limit = 0.0;  //!< [rad/s] wheel angle rate (steering model only), 0 = unlimited
    double wheelbase = 0.0;  //!< [m] steering model only; the model is off when <= 0
    double max_velocity_error = 0.3;  //!< [m/s] measured vs. expected speed above which the prediction is skipped
    double min_speed = 0.0;  //!< [m/s] measured speed and newest command below this: skip the prediction (standstill)
    double max_command_gap = 0.5;  //!< [s] older commands are dropped (controller was not running)
    double max_steering_angle = 1.4;  //!< [rad] clamp of the wheel angle in the steering model
  };

  struct State
  {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;  //!< linear velocity in the robot frame
    double omega = 0.0;  //!< turn rate
  };

  void setParams(const Params& params) { params_ = params; }
  const Params& params() const { return params_; }
  bool enabled() const { return params_.latency > 0.0; }
  bool steeringModelEnabled() const
  {
    return params_.wheelbase > 0.0 && (params_.steering_latency > 0.0 || params_.steering_rate_limit > 0.0);
  }

  /**
   * @brief Remember the command the drive received at time t (call for every command, also zero ones).
   * Feed the executed commands (after the downstream filters), not the planner's own output.
   */
  void addCommand(double t, double v, double omega);

  /** @brief Forget all commands (new controller run, reset). */
  void reset() { commands_.clear(); }

  /**
   * @brief Predict the state at t + latency from the state measured at t.
   * @return false if nothing was predicted (disabled, no recent commands, plant does not follow); then
   *         predicted equals measured.
   */
  bool predict(double t, const State& measured, State& predicted);

  /** @brief Why the last predict() returned false (static string, empty after a prediction). */
  const char* skipReason() const { return skip_reason_; }

private:
  struct Command
  {
    double t;
    double v;
    double omega;
    double delta;  //!< implied wheel angle (held through standstill commands)
  };

  //! The command in effect at time t, nullptr if t is before the first remembered command.
  const Command* commandAt(double t) const;
  double impliedSteeringAngle(double v, double omega, double fallback) const;

  //! Do the commands in effect over [t - window, t] (or the measured speed) change between forward and
  //! reverse with a wheel angle (steering model) that puts the wheel on the other side?
  bool wheelFlips(double t, double window, double measured_v) const;

  Params params_;
  std::deque<Command> commands_;
  const char* skip_reason_ = "";
};

}  // namespace teb_local_planner

#endif  // CONTROL_LATENCY_PREDICTOR_H_
