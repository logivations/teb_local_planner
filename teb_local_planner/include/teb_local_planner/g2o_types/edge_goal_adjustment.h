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
#ifndef EDGE_GOAL_ADJUSTMENT_H_
#define EDGE_GOAL_ADJUSTMENT_H_

#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/penalties.h"
#include "teb_local_planner/misc.h"

#include "g2o/core/base_unary_edge.h"

#include <cmath>


namespace teb_local_planner
{

/**
 * @class EdgeGoalAdjustment
 * @brief Edge that allows the (unfixed) goal pose to be adjusted slightly during optimization.
 *
 * If the requested goal cannot be approached smoothly (e.g. only a small lateral offset is left,
 * which a nonholonomic robot can only compensate by oscillating forward/backward), the goal vertex
 * can be released during optimization. Only the LATERAL component (y in the requested goal frame)
 * is adjustable: a longitudinal residual can always be driven to zero by driving further, so
 * giving the optimizer longitudinal freedom only trades away goal accuracy. This edge keeps the
 * optimized goal close to the requested goal:
 * - component 0: longitudinal deviation (approximated hard constraint, the goal must not move
 *   along its own x axis).
 * - component 1: quadratic lateral attraction towards the requested goal position, weighted
 *   with 'weight_adjust_goal'.
 * - component 2: penalty for exceeding the allowed lateral adjustment range
 *   [-max_adjust_goal_y, max_adjust_goal_y] (approximated hard constraint).
 * - component 3: deviation from the requested goal orientation (approximated hard constraint,
 *   the goal heading itself must not change).
 * @see TebOptimalPlanner::AddEdgesGoalAdjustment
 * @remarks Do not forget to call setTebConfig() and setGoal()
 */
class EdgeGoalAdjustment : public BaseTebUnaryEdge<4, const PoseSE2*, VertexPose>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeGoalAdjustment()
  {
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    TEB_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig(), setGoal() on EdgeGoalAdjustment()");
    const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);

    // express the deviation from the requested goal in the requested goal frame
    Eigen::Vector2d deviation = pose->position() - _measurement->position();
    const double cos_theta = std::cos(_measurement->theta());
    const double sin_theta = std::sin(_measurement->theta());
    const double dx =  cos_theta * deviation.x() + sin_theta * deviation.y(); // longitudinal
    const double dy = -sin_theta * deviation.x() + cos_theta * deviation.y(); // lateral
    const double dtheta = g2o::normalize_theta(pose->theta() - _measurement->theta());

    _error[0] = dx;
    _error[1] = dy;
    _error[2] = penaltyBoundToInterval(dy, cfg_->goal_tolerance.max_adjust_goal_y, 0.0);
    _error[3] = dtheta;

    TEB_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]) && std::isfinite(_error[3]),
                   "EdgeGoalAdjustment::computeError() _error[0]=%f _error[1]=%f _error[3]=%f\n", _error[0], _error[1], _error[3]);
  }

  /**
   * @brief Set pointer to the requested (original) goal pose for the underlying cost function
   * @param goal PoseSE2 containing the requested goal pose
   */
  void setGoal(const PoseSE2* goal)
  {
    _measurement = goal;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param goal PoseSE2 containing the requested goal pose
   */
  void setParameters(const TebConfig& cfg, const PoseSE2* goal)
  {
    cfg_ = &cfg;
    _measurement = goal;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



} // end namespace

#endif
