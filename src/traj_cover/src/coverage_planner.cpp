#include "traj_cover/coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Eigenvalues>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace traj_cover
{
namespace
{

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

double cross(const Point2 & lhs, const Point2 & rhs)
{
  return lhs.u * rhs.v - lhs.v * rhs.u;
}

Point2 subtract(const Point2 & lhs, const Point2 & rhs)
{
  return {lhs.u - rhs.u, lhs.v - rhs.v};
}

Point2 add(const Point2 & lhs, const Point2 & rhs)
{
  return {lhs.u + rhs.u, lhs.v + rhs.v};
}

Point2 scale(const Point2 & point, double factor)
{
  return {point.u * factor, point.v * factor};
}

double signed_area(const std::vector<Point2> & polygon)
{
  double area = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & current = polygon[i];
    const auto & next = polygon[(i + 1) % polygon.size()];
    area += cross(current, next);
  }
  return 0.5 * area;
}

std::vector<Point2> convex_hull(std::vector<Point2> points)
{
  std::sort(
    points.begin(), points.end(), [](const Point2 & lhs, const Point2 & rhs) {
      return lhs.u < rhs.u || (lhs.u == rhs.u && lhs.v < rhs.v);
    });
  points.erase(
    std::unique(
      points.begin(), points.end(), [](const Point2 & lhs, const Point2 & rhs) {
        return std::abs(lhs.u - rhs.u) < kEpsilon &&
               std::abs(lhs.v - rhs.v) < kEpsilon;
      }),
    points.end());
  if (points.size() < 3) {
    throw std::runtime_error("UV projection contains fewer than three unique points");
  }

  std::vector<Point2> hull(2 * points.size());
  std::size_t size = 0;
  for (const auto & point : points) {
    while (size >= 2 &&
      cross(subtract(hull[size - 1], hull[size - 2]), subtract(point, hull[size - 1])) <= 0.0)
    {
      --size;
    }
    hull[size++] = point;
  }
  const std::size_t lower_size = size;
  for (auto iterator = points.rbegin() + 1; iterator != points.rend(); ++iterator) {
    while (size > lower_size &&
      cross(
        subtract(hull[size - 1], hull[size - 2]),
        subtract(*iterator, hull[size - 1])) <= 0.0)
    {
      --size;
    }
    hull[size++] = *iterator;
  }
  hull.resize(size - 1);
  if (signed_area(hull) < 0.0) {
    std::reverse(hull.begin(), hull.end());
  }
  return hull;
}

std::vector<Point2> simplify_convex_polygon(
  std::vector<Point2> polygon,
  double tolerance)
{
  if (tolerance <= 0.0) {
    return polygon;
  }
  bool changed = true;
  while (changed && polygon.size() > 3) {
    changed = false;
    for (std::size_t i = 0; i < polygon.size() && polygon.size() > 3;) {
      const Point2 & previous = polygon[(i + polygon.size() - 1) % polygon.size()];
      const Point2 & current = polygon[i];
      const Point2 & next = polygon[(i + 1) % polygon.size()];
      const Point2 chord = subtract(next, previous);
      const double chord_length = std::hypot(chord.u, chord.v);
      const Point2 incoming = subtract(current, previous);
      const Point2 outgoing = subtract(next, current);
      const double distance_to_chord =
        chord_length > kEpsilon ? std::abs(cross(chord, incoming)) / chord_length :
        std::numeric_limits<double>::infinity();
      const double forward_dot = incoming.u * outgoing.u + incoming.v * outgoing.v;
      if (distance_to_chord <= tolerance && forward_dot >= 0.0) {
        polygon.erase(polygon.begin() + static_cast<std::ptrdiff_t>(i));
        changed = true;
      } else {
        ++i;
      }
    }
  }
  return polygon;
}

std::vector<Point2> inset_convex_polygon(
  const std::vector<Point2> & polygon,
  double inset)
{
  if (polygon.size() < 3 || inset < 0.0) {
    throw std::invalid_argument("invalid polygon or negative wall inset");
  }
  if (inset == 0.0) {
    return polygon;
  }

  std::vector<Point2> result;
  result.reserve(polygon.size());
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Point2 & previous = polygon[(i + polygon.size() - 1) % polygon.size()];
    const Point2 & current = polygon[i];
    const Point2 & next = polygon[(i + 1) % polygon.size()];

    Point2 first_direction = subtract(current, previous);
    Point2 second_direction = subtract(next, current);
    const double first_length = std::hypot(first_direction.u, first_direction.v);
    const double second_length = std::hypot(second_direction.u, second_direction.v);
    if (first_length < kEpsilon || second_length < kEpsilon) {
      throw std::runtime_error("boundary contains a zero-length edge");
    }
    first_direction = scale(first_direction, 1.0 / first_length);
    second_direction = scale(second_direction, 1.0 / second_length);
    const Point2 first_normal{-first_direction.v, first_direction.u};
    const Point2 second_normal{-second_direction.v, second_direction.u};
    const Point2 first_point = add(current, scale(first_normal, inset));
    const Point2 second_point = add(current, scale(second_normal, inset));
    const double denominator = cross(first_direction, second_direction);
    if (std::abs(denominator) < kEpsilon) {
      const Point2 normal_sum = add(first_normal, second_normal);
      const double normal_length = std::hypot(normal_sum.u, normal_sum.v);
      result.push_back(add(current, scale(normal_sum, inset / normal_length)));
      continue;
    }
    const double first_parameter =
      cross(subtract(second_point, first_point), second_direction) / denominator;
    result.push_back(add(first_point, scale(first_direction, first_parameter)));
  }

  if (result.size() < 3 || signed_area(result) <= kEpsilon) {
    throw std::runtime_error("wall inset is too large for the detected boundary");
  }
  for (const auto & point : result) {
    if (!point_in_convex_polygon(point, polygon)) {
      throw std::runtime_error("failed to create a valid inset boundary");
    }
  }
  return result;
}

std::vector<SweepSegment> slice_rows(
  const std::vector<Point2> & polygon,
  double row_spacing,
  bool sweep_along_v)
{
  const auto across = [sweep_along_v](const Point2 & point) {
      return sweep_along_v ? point.u : point.v;
    };
  const auto along = [sweep_along_v](const Point2 & point) {
      return sweep_along_v ? point.v : point.u;
    };
  const auto make_point = [sweep_along_v](double across_value, double along_value) {
      return sweep_along_v ? Point2{across_value, along_value} :
             Point2{along_value, across_value};
    };

  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const auto & point : polygon) {
    minimum = std::min(minimum, across(point));
    maximum = std::max(maximum, across(point));
  }
  const double span = maximum - minimum;
  if (span <= kEpsilon) {
    throw std::runtime_error("inset boundary has no extent across the sweep direction");
  }

  std::vector<double> rows;
  if (span <= row_spacing) {
    rows.push_back(0.5 * (minimum + maximum));
  } else {
    for (double row = minimum + 0.5 * row_spacing;
      row <= maximum - 0.5 * row_spacing + kEpsilon; row += row_spacing)
    {
      rows.push_back(row);
    }
    if (rows.empty()) {
      rows.push_back(0.5 * (minimum + maximum));
    }
  }

  std::vector<SweepSegment> segments;
  for (const double row : rows) {
    std::vector<double> intersections;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const Point2 & first = polygon[i];
      const Point2 & second = polygon[(i + 1) % polygon.size()];
      const double first_across = across(first);
      const double second_across = across(second);
      const bool crosses =
        (first_across <= row && row < second_across) ||
        (second_across <= row && row < first_across);
      if (!crosses || std::abs(second_across - first_across) < kEpsilon) {
        continue;
      }
      const double ratio = (row - first_across) / (second_across - first_across);
      intersections.push_back(along(first) + ratio * (along(second) - along(first)));
    }
    if (intersections.size() < 2) {
      continue;
    }
    std::sort(intersections.begin(), intersections.end());
    segments.push_back(
      {make_point(row, intersections.front()), make_point(row, intersections.back())});
  }
  if (segments.empty()) {
    throw std::runtime_error("no sweep row intersects the inset boundary");
  }
  return segments;
}

void append_line(
  std::vector<PathPoint> & path,
  const Point2 & start,
  const Point2 & end,
  bool is_coverage,
  double step)
{
  const double length = distance(start, end);
  if (path.empty()) {
    path.push_back({start, 0.0, 0.0, 0.0, is_coverage});
  } else if (distance(path.back().position, start) > kEpsilon) {
    path.push_back({start, 0.0, 0.0, 0.0, is_coverage});
  } else {
    path.back().is_coverage = is_coverage;
  }
  if (length < kEpsilon) {
    return;
  }
  const int count = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int index = 1; index <= count; ++index) {
    const double ratio = static_cast<double>(index) / static_cast<double>(count);
    path.push_back(
      {{start.u + ratio * (end.u - start.u), start.v + ratio * (end.v - start.v)},
        0.0, 0.0, 0.0, is_coverage});
  }
}

void append_quadratic_bspline(
  std::vector<PathPoint> & path,
  const Point2 & start,
  const Point2 & control,
  const Point2 & end,
  double step)
{
  // A clamped quadratic B-spline with three control points is equivalent to
  // this quadratic Bezier form. Its endpoint tangents follow start-control
  // and control-end, so it joins both neighboring straight lines smoothly.
  const double control_polygon_length =
    distance(start, control) + distance(control, end);
  const int count =
    std::max(1, static_cast<int>(std::ceil(control_polygon_length / step)));
  if (path.empty()) {
    path.push_back({start, 0.0, 0.0, 0.0, false});
  } else if (distance(path.back().position, start) > kEpsilon) {
    path.push_back({start, 0.0, 0.0, 0.0, false});
  }
  for (int index = 1; index <= count; ++index) {
    const double t = static_cast<double>(index) / static_cast<double>(count);
    const double one_minus_t = 1.0 - t;
    path.push_back(
      {{one_minus_t * one_minus_t * start.u +
          2.0 * one_minus_t * t * control.u + t * t * end.u,
        one_minus_t * one_minus_t * start.v +
          2.0 * one_minus_t * t * control.v + t * t * end.v},
        0.0, 0.0, 0.0, false});
  }
}

std::vector<PathPoint> connect_and_resample(
  const std::vector<SweepSegment> & segments,
  double step,
  double smoothing_distance)
{
  std::vector<SweepSegment> oriented_segments = segments;
  for (std::size_t i = 0; i < oriented_segments.size(); ++i) {
    if (i % 2 == 1) {
      std::swap(oriented_segments[i].start, oriented_segments[i].end);
    }
  }

  std::vector<PathPoint> path;
  Point2 coverage_start = oriented_segments.front().start;
  for (std::size_t i = 0; i < oriented_segments.size(); ++i) {
    const Point2 coverage_end = oriented_segments[i].end;
    if (i + 1 == oriented_segments.size() || smoothing_distance <= kEpsilon) {
      append_line(path, coverage_start, coverage_end, true, step);
      if (i + 1 < oriented_segments.size()) {
        append_line(
          path, coverage_end, oriented_segments[i + 1].start, false, step);
        coverage_start = oriented_segments[i + 1].start;
      }
      continue;
    }

    const Point2 next_start = oriented_segments[i + 1].start;
    const Point2 next_end = oriented_segments[i + 1].end;
    const Point2 incoming = subtract(coverage_end, oriented_segments[i].start);
    const Point2 connector = subtract(next_start, coverage_end);
    const Point2 outgoing = subtract(next_end, next_start);
    const double incoming_length = std::hypot(incoming.u, incoming.v);
    const double connector_length = std::hypot(connector.u, connector.v);
    const double outgoing_length = std::hypot(outgoing.u, outgoing.v);
    const double trim = std::min(
      {smoothing_distance, 0.5 * incoming_length, 0.5 * connector_length,
        0.5 * outgoing_length});

    if (trim <= kEpsilon) {
      append_line(path, coverage_start, coverage_end, true, step);
      append_line(path, coverage_end, next_start, false, step);
      coverage_start = next_start;
      continue;
    }

    const Point2 incoming_unit = scale(incoming, 1.0 / incoming_length);
    const Point2 connector_unit = scale(connector, 1.0 / connector_length);
    const Point2 outgoing_unit = scale(outgoing, 1.0 / outgoing_length);
    const Point2 first_curve_start =
      subtract(coverage_end, scale(incoming_unit, trim));
    const Point2 first_curve_end =
      add(coverage_end, scale(connector_unit, trim));
    const Point2 second_curve_start =
      subtract(next_start, scale(connector_unit, trim));
    const Point2 second_curve_end =
      add(next_start, scale(outgoing_unit, trim));

    append_line(path, coverage_start, first_curve_start, true, step);
    append_quadratic_bspline(
      path, first_curve_start, coverage_end, first_curve_end, step);
    append_line(path, first_curve_end, second_curve_start, false, step);
    append_quadratic_bspline(
      path, second_curve_start, next_start, second_curve_end, step);
    coverage_start = second_curve_end;
  }
  return path;
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void parameterize(std::vector<PathPoint> & path, const PlannerConfig & config)
{
  if (path.size() < 2) {
    throw std::runtime_error("coverage path contains fewer than two points");
  }
  std::vector<double> speed_limit(path.size(), config.v_max);
  path.front().s = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Point2 delta = subtract(path[i].position, path[i - 1].position);
    const double segment_length = std::hypot(delta.u, delta.v);
    path[i].s = path[i - 1].s + segment_length;
    path[i - 1].yaw = std::atan2(delta.v, delta.u);
  }
  path.back().yaw = path[path.size() - 2].yaw;

  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const double ds = std::max(kEpsilon, path[i + 1].s - path[i - 1].s);
    const double yaw_delta = normalize_angle(path[i].yaw - path[i - 1].yaw);
    const double curvature = std::abs(yaw_delta) * 2.0 / ds;
    if (curvature > kEpsilon) {
      speed_limit[i] = std::min(speed_limit[i], std::sqrt(config.a_lat_max / curvature));
    }
  }
  speed_limit.front() = 0.0;
  speed_limit.back() = 0.0;
  path.front().speed = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double ds = path[i].s - path[i - 1].s;
    path[i].speed =
      std::min(speed_limit[i], std::sqrt(path[i - 1].speed * path[i - 1].speed +
      2.0 * config.a_max * ds));
  }
  for (std::size_t i = path.size() - 1; i-- > 0;) {
    const double ds = path[i + 1].s - path[i].s;
    path[i].speed =
      std::min(path[i].speed, std::sqrt(path[i + 1].speed * path[i + 1].speed +
      2.0 * config.a_max * ds));
  }
}

}  // namespace

CoveragePlanner::CoveragePlanner(PlannerConfig config)
: config_(std::move(config))
{
  if (config_.sweep_axis != "u" && config_.sweep_axis != "v") {
    throw std::invalid_argument("sweep_axis must be u or v");
  }
  if (config_.ransac_distance_threshold <= 0.0 || config_.ransac_max_iterations <= 0 ||
    config_.min_plane_points < 3 || config_.boundary_simplify_tolerance < 0.0 ||
    config_.row_spacing <= 0.0 ||
    config_.wall_inset < 0.0 || config_.resample_step <= 0.0 ||
    config_.connector_smoothing_distance < 0.0 ||
    config_.v_max <= 0.0 || config_.a_max <= 0.0 || config_.a_lat_max <= 0.0)
  {
    throw std::invalid_argument("planner parameters must be positive");
  }
}

PlanningResult CoveragePlanner::plan(
  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & cloud) const
{
  if (!cloud || cloud->size() < config_.min_plane_points) {
    throw std::runtime_error("input point cloud is empty or too small");
  }

  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(config_.ransac_distance_threshold);
  segmentation.setMaxIterations(config_.ransac_max_iterations);
  segmentation.setInputCloud(cloud);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers, coefficients);
  if (inliers.indices.size() < config_.min_plane_points || coefficients.values.size() < 4) {
    throw std::runtime_error("RANSAC did not find enough plane points");
  }

  Eigen::Vector3d normal(
    coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  normal.normalize();
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  for (const int index : inliers.indices) {
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    origin += Eigen::Vector3d(point.x, point.y, point.z);
  }
  origin /= static_cast<double>(inliers.indices.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const int index : inliers.indices) {
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    Eigen::Vector3d delta(point.x, point.y, point.z);
    delta -= origin;
    delta -= normal * delta.dot(normal);
    covariance += delta * delta.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
  Eigen::Vector3d u_axis = eigen_solver.eigenvectors().col(2).normalized();
  u_axis = (u_axis - normal * u_axis.dot(normal)).normalized();
  Eigen::Index dominant_index = 0;
  u_axis.cwiseAbs().maxCoeff(&dominant_index);
  if (u_axis[dominant_index] < 0.0) {
    u_axis = -u_axis;
  }
  Eigen::Vector3d v_axis = normal.cross(u_axis).normalized();

  PlanningResult result;
  result.frame = {origin, u_axis, v_axis, normal};
  result.projected_points.reserve(inliers.indices.size());
  for (const int index : inliers.indices) {
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    const Eigen::Vector3d delta =
      Eigen::Vector3d(point.x, point.y, point.z) - origin;
    result.projected_points.push_back({delta.dot(u_axis), delta.dot(v_axis)});
  }
  result.boundary = simplify_convex_polygon(
    convex_hull(result.projected_points), config_.boundary_simplify_tolerance);
  result.inset_boundary = inset_convex_polygon(result.boundary, config_.wall_inset);
  result.sweep_segments = slice_rows(
    result.inset_boundary, config_.row_spacing, config_.sweep_axis == "v");
  result.path = connect_and_resample(
    result.sweep_segments, config_.resample_step, config_.connector_smoothing_distance);
  parameterize(result.path, config_);
  return result;
}

double distance(const Point2 & lhs, const Point2 & rhs)
{
  return std::hypot(lhs.u - rhs.u, lhs.v - rhs.v);
}

bool point_in_convex_polygon(const Point2 & point, const std::vector<Point2> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Point2 edge = subtract(polygon[(i + 1) % polygon.size()], polygon[i]);
    const Point2 offset = subtract(point, polygon[i]);
    // PCD coordinates are float32 and sweep endpoints are reconstructed from
    // line intersections. Allow a small metric-scale numerical tolerance.
    if (cross(edge, offset) < -1e-5) {
      return false;
    }
  }
  return true;
}

}  // namespace traj_cover
