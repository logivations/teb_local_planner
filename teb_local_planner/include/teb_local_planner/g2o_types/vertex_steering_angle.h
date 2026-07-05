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
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 *********************************************************************/

#ifndef VERTEX_STEERING_ANGLE_H
#define VERTEX_STEERING_ANGLE_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/stuff/misc.h"

#include "teb_local_planner/pose_se2.h"

#include <Eigen/Core>

#include <cmath>
#include <utility>

namespace teb_local_planner
{

/**
  * @class VertexSteeringAngle
  * @brief This class stores and wraps the steering angle \f$ \phi \f$ of a carlike/tricycle robot
  * into a vertex that can be optimized via g2o.
  *
  * One steering vertex is associated with each trajectory segment (pose pair), analogous to
  * VertexTimeDiff. The estimate is deliberately NOT normalized/wrapped in oplusImpl:
  * \f$ \phi \f$ is kept within its physical bounds by EdgeSteeringBound, and wrapping would
  * reintroduce the cost-landscape discontinuities that the explicit steering state is
  * supposed to remove.
  */
class VertexSteeringAngle : public g2o::BaseVertex<1, double>
{
public:

  /**
    * @brief Default constructor
    * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
    */
  VertexSteeringAngle(bool fixed = false)
  {
    setToOriginImpl();
    setFixed(fixed);
  }

  /**
    * @brief Construct the SteeringAngle vertex with a value
    * @param phi steering angle of the vertex
    * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
    */
  VertexSteeringAngle(double phi, bool fixed = false)
  {
    _estimate = phi;
    setFixed(fixed);
  }

  /**
    * @brief Access the steering angle of the vertex
    * @see estimate
    * @return reference to phi
    */
  inline double& steering() {return _estimate;}

  /**
    * @brief Access the steering angle of the vertex (read-only)
    * @see estimate
    * @return const reference to phi
    */
  inline const double& steering() const {return _estimate;}

  /**
    * @brief Set the underlying estimate \f$ \phi \f$ to default.
    */
  virtual void setToOriginImpl() override
  {
    _estimate = 0.0;
  }

  /**
    * @brief Define the update increment \f$ \phi_{k+1} = \phi_k + update \f$.
    * A simple addition implements what we want (no angle wrapping, see class description).
    * @param update increment that should be added to the previous esimate
    */
  virtual void oplusImpl(const double* update) override
  {
      _estimate += *update;
  }

  /**
    * @brief Read an estimate of \f$ \phi \f$ from an input stream
    * @param is input stream
    * @return always \c true
    */
  virtual bool read(std::istream& is) override
  {
    is >> _estimate;
    return true;
  }

  /**
    * @brief Write the estimate \f$ \phi \f$ to an output stream
    * @param os output stream
    * @return \c true if the export was successful, otherwise \c false
    */
  virtual bool write(std::ostream& os) const override
  {
    os << estimate();
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @brief Estimate the steering angle implied by a trajectory segment (bicycle model).
 *
 * The segment between two consecutive poses corresponds to a (signed) longitudinal
 * displacement \f$ \Delta s \f$ (projected onto the segment mid-heading) and a heading
 * change \f$ \Delta\theta \f$. For a bicycle model with the base frame at the fixed axle,
 * \f$ \tan\phi = L \Delta\theta / \Delta s \f$.
 *
 * The returned angle lies in (-pi, pi]: values beyond +-pi/2 encode a turn-in-place or
 * reverse-rolling branch and should be mapped onto the branch closest to the previous
 * steering angle by the caller (the branches {phi, phi-pi, phi+pi} are indistinguishable
 * from positions alone, since they correspond to reversing the drive-wheel direction).
 *
 * @param pose1 first pose of the segment
 * @param pose2 second pose of the segment
 * @param wheelbase distance between fixed axle and steering axle
 * @return pair(valid, phi): \c valid is false for degenerate (near-standstill) segments,
 *         in which case the previous steering angle should be carried over.
 */
inline std::pair<bool, double> steeringFromSegment(const PoseSE2& pose1, const PoseSE2& pose2, double wheelbase)
{
  const double dtheta = g2o::normalize_theta(pose2.theta() - pose1.theta());
  const double theta_m = pose1.theta() + 0.5 * dtheta;
  const Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  const double ds = deltaS.x() * std::cos(theta_m) + deltaS.y() * std::sin(theta_m);

  const double arc = wheelbase * dtheta;
  if (std::hypot(ds, arc) < 1e-4)
    return {false, 0.0};

  return {true, std::atan2(arc, ds)};
}

/**
 * @brief Choose the representative of {phi, phi-pi, phi+pi} closest to a reference angle.
 *
 * Used when initializing the steering chain: keeps consecutive steering angles on a single
 * branch so the optimizer does not straddle the +-pi/2 turn-in-place ambiguity.
 */
inline double closestSteeringBranch(double phi, double phi_ref)
{
  double best = phi;
  double best_dist = std::abs(phi - phi_ref);
  for (double candidate : {phi - M_PI, phi + M_PI})
  {
    const double dist = std::abs(candidate - phi_ref);
    if (dist < best_dist)
    {
      best = candidate;
      best_dist = dist;
    }
  }
  return best;
}

}

#endif
