#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "traj_cover/coverage_planner.hpp"
#include "traj_cover/package_paths.hpp"
#include "traj_cover/visualizer.hpp"

namespace traj_cover
{
namespace
{

void save_csv(const std::filesystem::path & path, const std::vector<PathPoint> & points)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open CSV output: " + path.string());
  }
  stream << "u,v,yaw,s,speed,is_coverage\n";
  stream << std::fixed << std::setprecision(9);
  for (const auto & point : points) {
    stream << point.position.u << ',' << point.position.v << ',' << point.yaw << ',' <<
      point.s << ',' << point.speed << ',' << (point.is_coverage ? 1 : 0) << '\n';
  }
  if (!stream) {
    throw std::runtime_error("failed while writing CSV output: " + path.string());
  }
}

}  // namespace

class CoverPlanNode : public rclcpp::Node
{
public:
  CoverPlanNode()
  : Node("cover_plan_node")
  {
    const std::string pcd_package = declare_parameter<std::string>("pcd_package", "point_extract");
    const std::string pcd_file =
      declare_parameter<std::string>("pcd_file", "data/table_fit.pcd");
    frame_id_ = declare_parameter<std::string>("uv_frame_id", "table_uv");
    const std::string output_package =
      declare_parameter<std::string>("output_package", "traj_cover");
    const std::string output_csv =
      declare_parameter<std::string>("output_csv", "outputs/cover_path.csv");
    const std::string viz_dir = declare_parameter<std::string>("viz_dir", "viz");

    PlannerConfig config;
    config.ransac_distance_threshold =
      declare_parameter<double>("ransac_distance_threshold", 0.005);
    config.ransac_max_iterations =
      declare_parameter<int>("ransac_max_iterations", 1000);
    const int min_plane_points = declare_parameter<int>("min_plane_points", 1000);
    if (min_plane_points < 3) {
      throw std::invalid_argument("min_plane_points must be at least 3");
    }
    config.min_plane_points = static_cast<std::size_t>(min_plane_points);
    config.boundary_simplify_tolerance =
      declare_parameter<double>("boundary_simplify_tolerance", 0.002);
    config.sweep_axis = declare_parameter<std::string>("sweep_axis", "v");
    config.row_spacing = declare_parameter<double>("row_spacing", 0.06);
    config.wall_inset = declare_parameter<double>("wall_inset", 0.02);
    config.resample_step = declare_parameter<double>("resample_step", 0.005);
    config.connector_smoothing_distance =
      declare_parameter<double>("connector_smoothing_distance", 0.02);
    config.v_max = declare_parameter<double>("v_max", 0.05);
    config.a_max = declare_parameter<double>("a_max", 0.10);
    config.a_lat_max = declare_parameter<double>("a_lat_max", 0.10);

    const std::string path_topic =
      declare_parameter<std::string>("path_topic", "cover_path");
    const std::string marker_topic =
      declare_parameter<std::string>("marker_topic", "cover_marker");
    const double publish_rate = declare_parameter<double>("publish_rate", 1.0);
    if (frame_id_.empty() || publish_rate <= 0.0) {
      throw std::invalid_argument("uv_frame_id must be non-empty and publish_rate positive");
    }

    const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic, transient_qos);
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, transient_qos);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
    const auto input_path = package_share_relative_path(pcd_package, pcd_file);
    if (pcl::io::loadPCDFile(input_path.string(), *cloud) != 0) {
      throw std::runtime_error("failed to load PCD: " + input_path.string());
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu points from %s", cloud->size(), input_path.c_str());

    CoveragePlanner planner(config);
    result_ = planner.plan(cloud);
    const auto csv_path = source_package_relative_path(output_package, output_csv);
    const auto visualization_path = source_package_relative_path(output_package, viz_dir);
    save_csv(csv_path, result_.path);
    Visualizer::write_all(result_, visualization_path);
    build_messages();

    RCLCPP_INFO(
      get_logger(),
      "Generated %zu rows along %s and %zu path points; CSV: %s; marker: %s (%s)",
      result_.sweep_segments.size(), config.sweep_axis.c_str(), result_.path.size(),
      csv_path.c_str(), marker_topic.c_str(), frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "Step visualizations: %s", visualization_path.c_str());

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publish();});
    publish();
  }

private:
  void build_messages()
  {
    path_message_.header.frame_id = frame_id_;
    path_message_.poses.reserve(result_.path.size());

    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = frame_id_;
    path_marker.ns = "coverage_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.006;
    path_marker.color.r = 0.1F;
    path_marker.color.g = 0.85F;
    path_marker.color.b = 0.2F;
    path_marker.color.a = 1.0F;
    path_marker.points.reserve(result_.path.size());

    for (const auto & point : result_.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id_;
      pose.pose.position.x = point.position.u;
      pose.pose.position.y = point.position.v;
      pose.pose.orientation.z = std::sin(0.5 * point.yaw);
      pose.pose.orientation.w = std::cos(0.5 * point.yaw);
      path_message_.poses.push_back(pose);

      geometry_msgs::msg::Point marker_point;
      marker_point.x = point.position.u;
      marker_point.y = point.position.v;
      path_marker.points.push_back(marker_point);
    }

    auto endpoint = [this](const PathPoint & point, int id, float red, float green, float blue) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "coverage_path";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.position.u;
        marker.pose.position.y = point.position.v;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = 1.0F;
        return marker;
      };
    marker_message_.markers = {
      path_marker,
      endpoint(result_.path.front(), 1, 0.1F, 0.8F, 0.1F),
      endpoint(result_.path.back(), 2, 0.9F, 0.15F, 0.1F)};
  }

  void publish()
  {
    const auto stamp = now();
    path_message_.header.stamp = stamp;
    for (auto & pose : path_message_.poses) {
      pose.header.stamp = stamp;
    }
    for (auto & marker : marker_message_.markers) {
      marker.header.stamp = stamp;
    }
    path_publisher_->publish(path_message_);
    marker_publisher_->publish(marker_message_);
  }

  std::string frame_id_;
  PlanningResult result_;
  nav_msgs::msg::Path path_message_;
  visualization_msgs::msg::MarkerArray marker_message_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace traj_cover

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<traj_cover::CoverPlanNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cover_plan_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
