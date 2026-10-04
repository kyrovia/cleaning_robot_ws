#include <cmath>
#include <memory>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "traj_cover/coverage_planner.hpp"

namespace
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr make_tilted_rectangle()
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  const Eigen::Vector3d origin(0.2, -0.1, 0.7);
  const Eigen::Vector3d u_axis(1.0, 0.0, 0.08);
  const Eigen::Vector3d normalized_u = u_axis.normalized();
  const Eigen::Vector3d normal(-0.08, -0.20, 1.0);
  const Eigen::Vector3d normalized_normal = normal.normalized();
  const Eigen::Vector3d v_axis = normalized_normal.cross(normalized_u).normalized();

  for (int u_index = 0; u_index <= 80; ++u_index) {
    for (int v_index = 0; v_index <= 50; ++v_index) {
      const double u = -0.4 + 0.01 * static_cast<double>(u_index);
      const double v = -0.25 + 0.01 * static_cast<double>(v_index);
      const Eigen::Vector3d position = origin + u * normalized_u + v * v_axis;
      pcl::PointXYZRGB point;
      point.x = static_cast<float>(position.x());
      point.y = static_cast<float>(position.y());
      point.z = static_cast<float>(position.z());
      point.r = 120;
      point.g = 120;
      point.b = 120;
      cloud->push_back(point);
    }
  }
  return cloud;
}

traj_cover::PlanningResult plan_rectangle()
{
  traj_cover::PlannerConfig config;
  config.ransac_distance_threshold = 0.002;
  config.ransac_max_iterations = 500;
  config.min_plane_points = 100;
  config.row_spacing = 0.06;
  config.wall_inset = 0.02;
  config.resample_step = 0.01;
  config.connector_smoothing_distance = 0.02;
  config.v_max = 0.05;
  config.a_max = 0.10;
  config.a_lat_max = 0.10;
  return traj_cover::CoveragePlanner(config).plan(make_tilted_rectangle());
}

TEST(CoveragePlanner, ProducesInsetSixtyMillimeterRows)
{
  const auto result = plan_rectangle();
  ASSERT_GE(result.sweep_segments.size(), 7U);
  for (const auto & segment : result.sweep_segments) {
    EXPECT_NEAR(segment.start.u, segment.end.u, 1e-6);
  }
  for (std::size_t i = 1; i < result.sweep_segments.size(); ++i) {
    EXPECT_NEAR(
      result.sweep_segments[i].start.u - result.sweep_segments[i - 1].start.u,
      0.06, 1e-6);
  }

  double min_u = result.inset_boundary.front().u;
  double max_u = min_u;
  double min_v = result.inset_boundary.front().v;
  double max_v = min_v;
  for (const auto & point : result.inset_boundary) {
    min_u = std::min(min_u, point.u);
    max_u = std::max(max_u, point.u);
    min_v = std::min(min_v, point.v);
    max_v = std::max(max_v, point.v);
  }
  EXPECT_NEAR(max_u - min_u, 0.76, 0.015);
  EXPECT_NEAR(max_v - min_v, 0.46, 0.015);
}

TEST(CoveragePlanner, KeepsEntirePathInsideInsetBoundary)
{
  const auto result = plan_rectangle();
  ASSERT_FALSE(result.path.empty());
  EXPECT_NEAR(result.path.front().speed, 0.0, 1e-12);
  EXPECT_NEAR(result.path.back().speed, 0.0, 1e-12);
  for (std::size_t index = 0; index < result.path.size(); ++index) {
    const auto & point = result.path[index];
    ASSERT_TRUE(traj_cover::point_in_convex_polygon(point.position, result.inset_boundary))
      << "index=" << index << " u=" << point.position.u << " v=" << point.position.v;
    EXPECT_TRUE(std::isfinite(point.yaw));
    EXPECT_TRUE(std::isfinite(point.s));
    EXPECT_GE(point.speed, 0.0);
    EXPECT_LE(point.speed, 0.05 + 1e-9);
  }
}

TEST(CoveragePlanner, SmoothsConnectorHeadingWithLocalBSplines)
{
  const auto result = plan_rectangle();
  double maximum_heading_change = 0.0;
  for (std::size_t index = 1; index + 1 < result.path.size(); ++index) {
    double heading_change = result.path[index].yaw - result.path[index - 1].yaw;
    heading_change = std::atan2(std::sin(heading_change), std::cos(heading_change));
    maximum_heading_change = std::max(maximum_heading_change, std::abs(heading_change));
  }

  // An unsmoothed connector contains approximately 90-degree heading jumps.
  EXPECT_LT(maximum_heading_change, 0.8);
}

TEST(CoveragePlanner, RejectsInvalidSpacing)
{
  traj_cover::PlannerConfig config;
  config.row_spacing = 0.0;
  EXPECT_THROW((void)traj_cover::CoveragePlanner(config), std::invalid_argument);
}

TEST(CoveragePlanner, RejectsNegativeConnectorSmoothingDistance)
{
  traj_cover::PlannerConfig config;
  config.connector_smoothing_distance = -0.01;
  EXPECT_THROW((void)traj_cover::CoveragePlanner(config), std::invalid_argument);
}

}  // namespace
