#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace traj_cover
{

struct Point2
{
  double u{0.0};
  double v{0.0};
};

struct PlaneFrame
{
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d u_axis{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d v_axis{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
};

struct SweepSegment
{
  Point2 start;
  Point2 end;
};

struct PathPoint
{
  Point2 position;
  double yaw{0.0};
  double s{0.0};
  double speed{0.0};
  bool is_coverage{false};
};

struct PlannerConfig
{
  double ransac_distance_threshold{0.005};
  int ransac_max_iterations{1000};
  std::size_t min_plane_points{1000};
  double boundary_simplify_tolerance{0.002};
  // u：沿 U 横扫，间距沿 V；v：沿 V 竖扫，间距沿 U。
  std::string sweep_axis{"v"};
  double row_spacing{0.06};
  double wall_inset{0.02};
  double resample_step{0.005};
  // Length trimmed from each side of a connector corner for local B-spline smoothing.
  // Set to zero to retain sharp polyline corners.
  double connector_smoothing_distance{0.02};
  double v_max{0.05};
  double a_max{0.10};
  double a_lat_max{0.10};
};

struct PlanningResult
{
  PlaneFrame frame;
  std::vector<Point2> projected_points;
  std::vector<Point2> boundary;
  std::vector<Point2> inset_boundary;
  std::vector<SweepSegment> sweep_segments;
  std::vector<PathPoint> path;
};

class CoveragePlanner
{
public:
  explicit CoveragePlanner(PlannerConfig config);

  PlanningResult plan(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) const;

private:
  PlannerConfig config_;
};

double distance(const Point2 & lhs, const Point2 & rhs);
bool point_in_convex_polygon(const Point2 & point, const std::vector<Point2> & polygon);

}  // namespace traj_cover
