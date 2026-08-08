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
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_BASE_LINK_OBSTACLE_H_
#define EDGE_BASE_LINK_OBSTACLE_H_

#include "teb_local_planner/obstacles.h"
#include "teb_local_planner/g2o_types/vertex_pose.h"
#include "teb_local_planner/g2o_types/base_teb_edges.h"
#include "teb_local_planner/g2o_types/penalties.h"
#include "teb_local_planner/teb_config.h"


namespace teb_local_planner
{

/**
 * @class EdgeBaseLinkObstacle
 * @brief Edge pushing the base_link (the pose vertex itself, i.e. the rear
 * axle) away from obstacles, independent of the footprint model.
 *
 * The regular obstacle edges measure the FOOTPRINT distance and go silent as
 * soon as min_obstacle_dist is satisfied — a long vehicle can then ride along
 * a wall with its body permanently beside the obstacle, which makes any later
 * turn infeasible in place. This edge instead measures the distance of the
 * pose vertex position to the obstacle and penalizes it below
 * obstacles.base_link_obstacle_dist, providing a gradient toward free space
 * (e.g. a lane center) even when the footprint margin is already satisfied.
 *
 * @see TebOptimalPlanner::AddEdgesBaseLinkObstacles
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */
class EdgeBaseLinkObstacle : public BaseTebUnaryEdge<1, const Obstacle*, VertexPose>
{
public:

  EdgeBaseLinkObstacle()
  {
    _measurement = NULL;
  }

  void computeError()
  {
    TEB_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setObstacle() on EdgeBaseLinkObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    double dist = _measurement->getMinimumDistance(bandpt->position());

    // Soft preference, not a safety margin: no penalty_epsilon widening.
    _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.base_link_obstacle_dist, 0.0);

    TEB_ASSERT_MSG(std::isfinite(_error[0]), "EdgeBaseLinkObstacle::computeError() _error[0]=%f\n", _error[0]);
  }

  /**
   * @brief Set pointer to associated obstacle for the underlying cost function
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // end namespace

#endif
