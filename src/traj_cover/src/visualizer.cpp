#include "traj_cover/visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace traj_cover
{
namespace
{

class Canvas
{
public:
  explicit Canvas(const PlanningResult & result)
  : image_(1000, 1400, CV_8UC3, cv::Scalar(250, 250, 250))
  {
    double min_u = result.projected_points.front().u;
    double max_u = min_u;
    double min_v = result.projected_points.front().v;
    double max_v = min_v;
    for (const auto & point : result.projected_points) {
      min_u = std::min(min_u, point.u);
      max_u = std::max(max_u, point.u);
      min_v = std::min(min_v, point.v);
      max_v = std::max(max_v, point.v);
    }
    const double width = std::max(1e-6, max_u - min_u);
    const double height = std::max(1e-6, max_v - min_v);
    const double margin = 70.0;
    scale_ = std::min(
      (static_cast<double>(image_.cols) - 2.0 * margin) / width,
      (static_cast<double>(image_.rows) - 2.0 * margin) / height);
    center_u_ = 0.5 * (min_u + max_u);
    center_v_ = 0.5 * (min_v + max_v);
  }

  cv::Point pixel(const Point2 & point) const
  {
    return {
      static_cast<int>(std::lround(0.5 * image_.cols + (point.u - center_u_) * scale_)),
      static_cast<int>(std::lround(0.5 * image_.rows - (point.v - center_v_) * scale_))};
  }

  cv::Mat & image() {return image_;}
  const cv::Mat & image() const {return image_;}

  void title(const std::string & text)
  {
    cv::putText(
      image_, text, {30, 42}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
      cv::Scalar(30, 30, 30), 2, cv::LINE_AA);
  }

private:
  cv::Mat image_;
  double center_u_{0.0};
  double center_v_{0.0};
  double scale_{1.0};
};

void draw_projection(Canvas & canvas, const std::vector<Point2> & points)
{
  const std::size_t stride = std::max<std::size_t>(1, points.size() / 50000);
  for (std::size_t i = 0; i < points.size(); i += stride) {
    cv::circle(canvas.image(), canvas.pixel(points[i]), 1, cv::Scalar(180, 120, 40), -1);
  }
}

void draw_polygon(
  Canvas & canvas,
  const std::vector<Point2> & polygon,
  const cv::Scalar & color,
  int thickness)
{
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    cv::line(
      canvas.image(), canvas.pixel(polygon[i]),
      canvas.pixel(polygon[(i + 1) % polygon.size()]), color, thickness, cv::LINE_AA);
  }
}

void draw_sweeps(Canvas & canvas, const std::vector<SweepSegment> & segments)
{
  for (const auto & segment : segments) {
    cv::line(
      canvas.image(), canvas.pixel(segment.start), canvas.pixel(segment.end),
      cv::Scalar(30, 150, 30), 2, cv::LINE_AA);
    cv::circle(canvas.image(), canvas.pixel(segment.start), 4, cv::Scalar(20, 80, 220), -1);
    cv::circle(canvas.image(), canvas.pixel(segment.end), 4, cv::Scalar(220, 80, 20), -1);
  }
}

void draw_path(Canvas & canvas, const std::vector<PathPoint> & path, bool arrows)
{
  for (std::size_t i = 1; i < path.size(); ++i) {
    const cv::Scalar color =
      path[i].is_coverage ? cv::Scalar(40, 160, 40) : cv::Scalar(20, 90, 220);
    cv::line(
      canvas.image(), canvas.pixel(path[i - 1].position), canvas.pixel(path[i].position),
      color, 2, cv::LINE_AA);
  }
  if (!path.empty()) {
    cv::circle(canvas.image(), canvas.pixel(path.front().position), 8, cv::Scalar(30, 180, 30), -1);
    cv::circle(canvas.image(), canvas.pixel(path.back().position), 8, cv::Scalar(30, 30, 220), -1);
  }
  if (arrows) {
    const std::size_t stride = std::max<std::size_t>(1, path.size() / 25);
    constexpr double arrow_length = 0.025;
    for (std::size_t i = 0; i < path.size(); i += stride) {
      const Point2 arrow_end{
        path[i].position.u + arrow_length * std::cos(path[i].yaw),
        path[i].position.v + arrow_length * std::sin(path[i].yaw)};
      cv::arrowedLine(
        canvas.image(), canvas.pixel(path[i].position), canvas.pixel(arrow_end),
        cv::Scalar(70, 70, 70), 1, cv::LINE_AA, 0, 0.25);
    }
  }
}

void save(const Canvas & canvas, const std::filesystem::path & path)
{
  if (!cv::imwrite(path.string(), canvas.image())) {
    throw std::runtime_error("failed to write visualization: " + path.string());
  }
}

Canvas base_canvas(const PlanningResult & result, const std::string & title)
{
  Canvas canvas(result);
  draw_projection(canvas, result.projected_points);
  canvas.title(title);
  return canvas;
}

}  // namespace

void Visualizer::write_all(
  const PlanningResult & result,
  const std::filesystem::path & output_directory)
{
  if (result.projected_points.empty()) {
    throw std::runtime_error("cannot visualize an empty projection");
  }
  std::filesystem::create_directories(output_directory);

  {
    Canvas canvas = base_canvas(result, "01 UV projection");
    save(canvas, output_directory / "01_uv_projection.png");
  }
  {
    Canvas canvas = base_canvas(result, "02 convex boundary");
    draw_polygon(canvas, result.boundary, cv::Scalar(30, 30, 220), 3);
    save(canvas, output_directory / "02_boundary_hull.png");
  }
  {
    Canvas canvas = base_canvas(result, "03 inset boundary");
    draw_polygon(canvas, result.boundary, cv::Scalar(170, 170, 170), 2);
    draw_polygon(canvas, result.inset_boundary, cv::Scalar(180, 40, 180), 3);
    save(canvas, output_directory / "03_inset_boundary.png");
  }
  {
    Canvas canvas = base_canvas(result, "04 sweep intersections");
    draw_polygon(canvas, result.inset_boundary, cv::Scalar(180, 40, 180), 2);
    draw_sweeps(canvas, result.sweep_segments);
    save(canvas, output_directory / "04_sweep_intersections.png");
  }
  {
    Canvas canvas = base_canvas(result, "05 boustrophedon connections");
    draw_polygon(canvas, result.inset_boundary, cv::Scalar(170, 170, 170), 2);
    draw_path(canvas, result.path, false);
    save(canvas, output_directory / "05_boustrophedon.png");
  }
  {
    Canvas canvas = base_canvas(result, "06 resampled path");
    draw_polygon(canvas, result.inset_boundary, cv::Scalar(170, 170, 170), 2);
    draw_path(canvas, result.path, true);
    save(canvas, output_directory / "06_resampled_path.png");
  }
  {
    Canvas canvas = base_canvas(result, "07 velocity");
    double max_speed = 0.0;
    for (const auto & point : result.path) {
      max_speed = std::max(max_speed, point.speed);
    }
    max_speed = std::max(max_speed, 1e-9);
    for (std::size_t i = 1; i < result.path.size(); ++i) {
      const double ratio = std::clamp(result.path[i].speed / max_speed, 0.0, 1.0);
      const cv::Scalar color(255.0 * (1.0 - ratio), 80.0, 255.0 * ratio);
      cv::line(
        canvas.image(), canvas.pixel(result.path[i - 1].position),
        canvas.pixel(result.path[i].position), color, 3, cv::LINE_AA);
    }
    save(canvas, output_directory / "07_velocity.png");
  }
}

}  // namespace traj_cover
