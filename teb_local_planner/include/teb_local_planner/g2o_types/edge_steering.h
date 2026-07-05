/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following classes are derived from classes defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 *********************************************************************/
#ifndef EDGE_STEERING_H_
#define EDGE_STEERING_H_

#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/vertex_timediff.h"
#include "teb_local_planner/g2o_types/vertex_steering_angle.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/penalties.h"
#include "teb_local_planner/misc.h"

#include <cmath>

namespace teb_local_planner
{

/**
 * @class EdgeSteeringConsistency
 * @brief Edge coupling the explicit steering-angle state to the trajectory geometry.
 *
 * For a bicycle model with wheelbase \f$ L \f$ and the base frame at the fixed axle, a segment
 * with signed longitudinal displacement \f$ \Delta s \f$ (projected onto the segment mid-heading)
 * and heading change \f$ \Delta\theta \f$ satisfies \f$ \tan\phi = L \Delta\theta / \Delta s \f$.
 * This is enforced in the division- and atan-free form
 * \f[ e = \Delta s \sin\phi - L \Delta\theta \cos\phi \f]
 * which is smooth in all optimization variables and degrades correctly at the limits:
 * - reverse driving is handled by the sign of \f$ \Delta s \f$ (no direction heuristics needed),
 * - turn-in-place (\f$ \Delta s = 0, \Delta\theta \neq 0 \f$) forces \f$ \cos\phi = 0 \f$,
 *   i.e. the steering wheel at +-90 degrees,
 * - at standstill the error vanishes for every \f$ \phi \f$, so the steering state can evolve
 *   freely while the robot dwells (bounded by EdgeSteeringRate) — this is what makes a
 *   stop-and-steer maneuver representable.
 *
 * The residual deliberately scales with the motion magnitude: near standstill the geometry
 * carries no steering information and the constraint fades out; normalizing would reintroduce
 * the zero-velocity singularity of steering angles derived from pose geometry.
 * @see TebOptimalPlanner::AddEdgesSteering
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeSteeringConsistency : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge and resize to 3 vertices (pose_i, pose_ip1, steering_i).
   */
  EdgeSteeringConsistency()
  {
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeSteeringConsistency()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexSteeringAngle* steering = static_cast<const VertexSteeringAngle*>(_vertices[2]);

    const double dtheta = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double theta_m = pose1->theta() + 0.5 * dtheta;
    const Eigen::Vector2d deltaS = pose2->position() - pose1->position();
    const double ds = deltaS.x() * std::cos(theta_m) + deltaS.y() * std::sin(theta_m);

    _error[0] = ds * std::sin(steering->steering()) - cfg_->robot.wheelbase * dtheta * std::cos(steering->steering());

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringConsistency::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeSteeringRate
 * @brief Edge limiting the rate of change between two consecutive steering-angle states.
 *
 * The steering angle \f$ \phi_i \f$ is associated with segment i (nominally its midpoint), so the
 * change \f$ \phi_{i+1} - \phi_i \f$ spans half of \f$ \Delta T_i \f$ and half of
 * \f$ \Delta T_{i+1} \f$ (same convention as EdgeAcceleration):
 * \f[ \dot\phi = 2 (\phi_{i+1} - \phi_i) / (\Delta T_i + \Delta T_{i+1}) \f]
 * bounded to [-max_steering_rate, max_steering_rate].
 *
 * Because the time differences are optimization variables coupled to EdgeTimeOptimal, swinging
 * the steering wheel costs trajectory time: reversing the wheel from -90 to +90 degrees requires
 * at least pi/max_steering_rate seconds, which makes "flickering" between forward-left and
 * backward-right maneuvers intrinsically expensive instead of free.
 * @see TebOptimalPlanner::AddEdgesSteering
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeSteeringRate : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge and resize to 4 vertices (steering_i, steering_ip1, dt_i, dt_ip1).
   */
  EdgeSteeringRate()
  {
    this->resize(4);
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeSteeringRate()");
    const VertexSteeringAngle* steering1 = static_cast<const VertexSteeringAngle*>(_vertices[0]);
    const VertexSteeringAngle* steering2 = static_cast<const VertexSteeringAngle*>(_vertices[1]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[2]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[3]);

    const double rate = 2.0 * (steering2->steering() - steering1->steering()) / (dt1->dt() + dt2->dt());

    _error[0] = penaltyBoundToInterval(rate, cfg_->robot.max_steering_rate, cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringRate::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeSteeringRateStart
 * @brief Edge limiting the steering rate between the measured steering angle and the first
 * steering-angle state.
 *
 * The measured (or last commanded) wheel angle is stored as measurement and anchors the start of
 * the steering chain: if the physical wheel is at -90 degrees and the plan requires +90 degrees,
 * the first segment must last at least pi/max_steering_rate seconds. A soft rate bound is used
 * instead of fixing the vertex, since \f$ \phi_0 \f$ is a segment-average quantity and a hard fix
 * would fight the consistency edge whenever odometry and band geometry disagree slightly.
 * @see TebOptimalPlanner::AddEdgesSteering
 * @remarks Do not forget to call setTebConfig() and setInitialSteeringAngle()
 */
class EdgeSteeringRateStart : public BaseTebBinaryEdge<1, double, VertexSteeringAngle, VertexTimeDiff>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeSteeringRateStart()
  {
    _measurement = 0.0;
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeSteeringRateStart()");
    const VertexSteeringAngle* steering = static_cast<const VertexSteeringAngle*>(_vertices[0]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[1]);

    const double rate = 2.0 * (steering->steering() - _measurement) / dt->dt();

    _error[0] = penaltyBoundToInterval(rate, cfg_->robot.max_steering_rate, cfg_->optim.penalty_epsilon);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringRateStart::computeError() _error[0]=%f\n", _error[0]);
  }

  /**
   * @brief Set the measured (or last commanded) steering angle at the start of the trajectory.
   * @param phi steering angle [rad]
   */
  void setInitialSteeringAngle(double phi)
  {
    _measurement = phi;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeSteeringBound
 * @brief Edge keeping a steering-angle state within its physical bounds
 * [-max_steering_angle_right, max_steering_angle] (approximated hard constraint).
 *
 * If max_steering_angle_right is 0 (default), max_steering_angle bounds both directions.
 * A soft bound edge is used instead of clamping inside VertexSteeringAngle::oplusImpl, since
 * clamping would corrupt the Levenberg-Marquardt step-quality estimate. Epsilon is 0 by intention
 * (add a margin to the parameters if needed, analogous to min_turning_radius).
 * @see TebOptimalPlanner::AddEdgesSteering
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeSteeringBound : public BaseTebUnaryEdge<1, double, VertexSteeringAngle>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeSteeringBound() = default;

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeSteeringBound()");
    const VertexSteeringAngle* steering = static_cast<const VertexSteeringAngle*>(_vertices[0]);

    const double bound_left = cfg_->robot.max_steering_angle;
    const double bound_right = cfg_->robot.max_steering_angle_right > 0
                                 ? cfg_->robot.max_steering_angle_right
                                 : cfg_->robot.max_steering_angle;

    _error[0] = penaltyBoundToInterval(steering->steering(), -bound_right, bound_left, 0.0);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringBound::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // end namespace

#endif
