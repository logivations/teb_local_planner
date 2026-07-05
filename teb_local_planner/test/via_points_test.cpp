#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "teb_local_planner/g2o_types/edge_via_point.h"
#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/teb_config.h"

using teb_local_planner::EdgeViaPoint;
using teb_local_planner::PoseSE2;
using teb_local_planner::TebConfig;
using teb_local_planner::VertexPose;
using teb_local_planner::ViaPoint;


TEST(EdgeViaPointTest, positionAndOrientationError)
{
  TebConfig cfg;
  VertexPose pose(1.0, 0.5, 0.3);
  const ViaPoint via_point(1.0, 0.0, 0.5);

  EdgeViaPoint edge;
  edge.setVertex(0, &pose);
  edge.setParameters(cfg, &via_point);

  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 0.5, 1e-12);
  EXPECT_NEAR(edge.error()[1], -0.2, 1e-12);
}

TEST(EdgeViaPointTest, orientationErrorIsNormalized)
{
  TebConfig cfg;
  VertexPose pose(0.0, 0.0, 3.0);
  const ViaPoint via_point(0.0, 0.0, -3.0);

  EdgeViaPoint edge;
  edge.setVertex(0, &pose);
  edge.setParameters(cfg, &via_point);

  edge.computeError();
  EXPECT_NEAR(edge.error()[1], 6.0 - 2.0 * M_PI, 1e-12);
}

TEST(EdgeViaPointTest, orientationTermInactiveWithoutHeading)
{
  TebConfig cfg;
  VertexPose pose(1.0, 0.5, 0.3);
  const ViaPoint via_point(1.0, 0.0); // position-only via point (e.g. custom via points)

  EdgeViaPoint edge;
  edge.setVertex(0, &pose);
  edge.setParameters(cfg, &via_point);

  edge.computeError();
  EXPECT_NEAR(edge.error()[0], 0.5, 1e-12);
  EXPECT_NEAR(edge.error()[1], 0.0, 1e-12);
}


class ViaPointPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("via_points_test");

    cfg_.robot.min_turning_radius = 0.0;
    cfg_.robot.max_vel_x = 1.0;
    cfg_.robot.max_vel_x_backwards = 0.5;
    cfg_.robot.max_vel_theta = 1.0;
    cfg_.robot.acc_lim_x = 0.5;
    cfg_.robot.acc_lim_theta = 0.5;
    cfg_.obstacles.include_dynamic_obstacles = false;
  }

  std::shared_ptr<teb_local_planner::TebOptimalPlanner> makePlanner()
  {
    return std::make_shared<teb_local_planner::TebOptimalPlanner>(
        node_, cfg_, &obstacles_, teb_local_planner::TebVisualizationPtr(), &via_points_);
  }

  // heading of the optimized band pose closest to the via-point position
  double planAndGetHeadingAtViaPoint()
  {
    auto planner = makePlanner();
    EXPECT_TRUE(planner->plan(PoseSE2(0, 0, 0), PoseSE2(4, 0, 0)));

    const Eigen::Vector2d via_position = via_points_.front();
    int closest = 0;
    double closest_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < planner->teb().sizePoses(); ++i)
    {
      const double dist = (planner->teb().Pose(i).position() - via_position).norm();
      if (dist < closest_dist)
      {
        closest_dist = dist;
        closest = i;
      }
    }
    EXPECT_LE(closest_dist, 0.3) << "band does not pass close to the via point";
    return planner->teb().Pose(closest).theta();
  }

  nav2::LifecycleNode::SharedPtr node_;
  TebConfig cfg_;
  teb_local_planner::ObstContainer obstacles_;
  teb_local_planner::ViaPointContainer via_points_;
};

TEST_F(ViaPointPlannerTest, headingFollowsViaPointOrientation)
{
  via_points_.emplace_back(2.0, 0.0, 0.6);
  cfg_.optim.weight_viapoint_orientation = 200.0;

  EXPECT_GT(planAndGetHeadingAtViaPoint(), 0.2)
      << "band heading should be pulled towards the via-point heading";
}

TEST_F(ViaPointPlannerTest, headingUnaffectedWithZeroWeight)
{
  via_points_.emplace_back(2.0, 0.0, 0.6);
  cfg_.optim.weight_viapoint_orientation = 0.0; // default: legacy position-only behavior

  EXPECT_LT(std::abs(planAndGetHeadingAtViaPoint()), 0.05);
}

TEST_F(ViaPointPlannerTest, headingUnaffectedWithoutViaPointOrientation)
{
  via_points_.emplace_back(2.0, 0.0); // position-only via point
  cfg_.optim.weight_viapoint_orientation = 200.0;

  EXPECT_LT(std::abs(planAndGetHeadingAtViaPoint()), 0.05);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
