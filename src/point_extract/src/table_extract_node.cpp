#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/common/centroid.h"
#include "pcl/conversions.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;

struct ExtractParams
{
  int mean_k;
  double stddev_mul_thresh;
  double distance_threshold;
  int max_iterations;
  double probability;
  int min_plane_points;
  double min_plane_separation;
  bool table_is_closer;
  double cluster_tolerance;
};

struct PlaneCloud
{
  Cloud::Ptr cloud;
  pcl::PointIndices::Ptr inliers;
  pcl::ModelCoefficients coefficients;
};

struct ExtractResult
{
  Cloud::Ptr table;
  pcl::ModelCoefficients coefficients;
  std::size_t input_count{0};
  std::size_t filtered_count{0};
  std::size_t plane_count{0};
  std::size_t other_count{0};
  double table_distance{0.0};
  double other_distance{0.0};
};

std::filesystem::path resolve_package_path(const std::string & relative)
{
  const std::filesystem::path path(relative);
  if (path.is_absolute()) {
    throw std::runtime_error("pcd_file 必须是相对包 share 目录的路径");
  }

  const auto share = ament_index_cpp::get_package_share_directory("point_extract");
  return std::filesystem::path(share) / path;
}

PlaneCloud fit_plane(const Cloud::ConstPtr & cloud, const ExtractParams & params, const char * name)
{
  pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(params.distance_threshold);
  segmentation.setMaxIterations(params.max_iterations);
  segmentation.setProbability(params.probability);
  segmentation.setInputCloud(cloud);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients coefficients;
  segmentation.segment(*inliers, coefficients);
  if (inliers->indices.size() < static_cast<std::size_t>(params.min_plane_points) ||
    coefficients.values.size() < 4)
  {
    throw std::runtime_error(std::string("RANSAC 没有拟合出") + name);
  }

  std::sort(inliers->indices.begin(), inliers->indices.end());
  auto plane = std::make_shared<Cloud>();
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane);
  return PlaneCloud{plane, inliers, coefficients};
}

Cloud::Ptr extract_outside(
  const Cloud::ConstPtr & cloud,
  const pcl::PointIndices::ConstPtr & inliers)
{
  auto outside = std::make_shared<Cloud>();
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(std::const_pointer_cast<pcl::PointIndices>(inliers));
  extract.setNegative(true);
  extract.filter(*outside);
  return outside;
}

Eigen::Vector3f outward_axis(const pcl::ModelCoefficients & coefficients)
{
  Eigen::Vector3f axis(
    coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  const float norm = axis.norm();
  if (norm < 1e-6f) {
    throw std::runtime_error("平面法向无效");
  }
  axis /= norm;
  // 相机在原点、光轴朝 +z。法向转到离开相机的方向，投影越大表示越远。
  if (axis.z() < 0.0f) {
    axis = -axis;
  }
  return axis;
}

double distance_along(const Cloud & cloud, const Eigen::Vector3f & axis)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(cloud, centroid);
  return static_cast<double>(centroid.head<3>().dot(axis));
}

ExtractResult extract_table(const Cloud::ConstPtr & input, const ExtractParams & params)
{
  auto finite = std::make_shared<Cloud>();
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*input, *finite, finite_indices);
  if (finite->empty()) {
    throw std::runtime_error("点云去掉无效点后为空");
  }

  auto filtered = std::make_shared<Cloud>();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter;
  outlier_filter.setInputCloud(finite);
  outlier_filter.setMeanK(params.mean_k);
  outlier_filter.setStddevMulThresh(params.stddev_mul_thresh);
  outlier_filter.filter(*filtered);
  if (filtered->empty()) {
    throw std::runtime_error("统计离群点滤波后点云为空");
  }

  const auto first = fit_plane(filtered, params, "第一张平面");
  const auto axis = outward_axis(first.coefficients);
  const auto rest = extract_outside(filtered, first.inliers);
  const auto second = fit_plane(rest, params, "第二张平面");

  const double first_distance = distance_along(*first.cloud, axis);
  const double second_distance = distance_along(*second.cloud, axis);
  if (std::abs(first_distance - second_distance) < params.min_plane_separation) {
    throw std::runtime_error("两张平面高度差太小，无法区分桌面和地面");
  }

  const bool first_is_table = params.table_is_closer
    ? first_distance < second_distance
    : first_distance > second_distance;
  const auto & table = first_is_table ? first : second;
  const auto & other = first_is_table ? second : first;

  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
  tree->setInputCloud(table.cloud);
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
  clustering.setClusterTolerance(params.cluster_tolerance);
  clustering.setMinClusterSize(1);
  clustering.setMaxClusterSize(static_cast<int>(table.cloud->size()));
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(table.cloud);
  clustering.extract(clusters);
  if (clusters.empty()) {
    throw std::runtime_error("欧式聚类没有得到簇");
  }
  const auto largest = std::max_element(
    clusters.begin(), clusters.end(),
    [](const pcl::PointIndices & left, const pcl::PointIndices & right) {
      return left.indices.size() < right.indices.size();
    });
  auto table_cloud = std::make_shared<Cloud>();
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_cluster;
  extract_cluster.setInputCloud(table.cloud);
  extract_cluster.setIndices(std::make_shared<pcl::PointIndices>(*largest));
  extract_cluster.filter(*table_cloud);
  if (table_cloud->empty()) {
    throw std::runtime_error("最大簇为空");
  }

  ExtractResult result;
  result.table = table_cloud;
  result.plane_count = table.cloud->size();
  result.coefficients = table.coefficients;
  result.input_count = input->size();
  result.filtered_count = filtered->size();
  result.other_count = other.cloud->size();
  result.table_distance = first_is_table ? first_distance : second_distance;
  result.other_distance = first_is_table ? second_distance : first_distance;
  return result;
}

void save_cloud(const std::string & relative, const Cloud & cloud)
{
  const auto path = resolve_package_path(relative);
  std::filesystem::create_directories(path.parent_path());
  if (pcl::io::savePCDFileBinary(path.string(), cloud) < 0) {
    throw std::runtime_error("无法保存拟合点云: " + relative);
  }
}

}  // namespace

class TableExtractNode : public rclcpp::Node
{
public:
  TableExtractNode()
  : Node("table_extract_node")
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "data/table.pcd");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");
    const auto topic = declare_parameter<std::string>("topic", "table_point");
    mean_k_ = declare_parameter<int>("sor_mean_k", 50);
    stddev_mul_thresh_ = declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.02);
    distance_threshold_ = declare_parameter<double>("ransac_distance_threshold", 0.01);
    max_iterations_ = declare_parameter<int>("ransac_max_iterations", 1000);
    probability_ = declare_parameter<double>("ransac_probability", 0.99);
    min_plane_points_ = declare_parameter<int>("min_plane_points", 1000);
    min_plane_separation_ = declare_parameter<double>("min_plane_separation", 0.05);
    table_is_closer_ = declare_parameter<bool>("table_is_closer", true);
    publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
    output_pcd_ = declare_parameter<std::string>("output_pcd", "data/table_fit.pcd");

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);
  }

  void start()
  {
    validate_params();

    const auto pcd_path = resolve_package_path(pcd_file_);
    if (!std::filesystem::is_regular_file(pcd_path)) {
      throw std::runtime_error("找不到点云文件: " + pcd_path.string());
    }

    auto input = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile(pcd_path.string(), *input) < 0) {
      throw std::runtime_error("无法读取点云: " + pcd_path.string());
    }

    const ExtractParams params{
      mean_k_,
      stddev_mul_thresh_,
      distance_threshold_,
      max_iterations_,
      probability_,
      min_plane_points_,
      min_plane_separation_,
      table_is_closer_,
      cluster_tolerance_,
    };
    const auto result = extract_table(input, params);
    const auto & coeff = result.coefficients.values;

    pcl::toROSMsg(*result.table, cloud_msg_);
    cloud_msg_.header.frame_id = frame_id_;

    RCLCPP_INFO(get_logger(), "读取点云 %zu 点", result.input_count);
    RCLCPP_INFO(get_logger(), "统计离群点滤波后 %zu 点", result.filtered_count);
    RCLCPP_INFO(
      get_logger(),
      "两张平面距相机 %.3f m（%zu 点）和 %.3f m（%zu 点）",
      result.table_distance,
      result.plane_count,
      result.other_distance,
      result.other_count);
    RCLCPP_INFO(
      get_logger(),
      "选择距相机 %.3f m 的平面作为桌子，平面内点 %zu，最大簇 %zu 点，模型 %.4fx + %.4fy + %.4fz + %.4f = 0",
      result.table_distance,
      result.plane_count,
      result.table->size(),
      coeff[0],
      coeff[1],
      coeff[2],
      coeff[3]);
    if (!output_pcd_.empty()) {
      save_cloud(output_pcd_, *result.table);
      RCLCPP_INFO(get_logger(), "拟合点云已写入 %s", output_pcd_.c_str());
    }

    RCLCPP_INFO(
      get_logger(), "发布 %s，坐标系 %s", publisher_->get_topic_name(), frame_id_.c_str());

    publish();
    if (publish_rate_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
      timer_ = create_wall_timer(period, [this]() { publish(); });
    }
  }

private:
  void validate_params() const
  {
    if (mean_k_ < 1) {
      throw std::runtime_error("sor_mean_k 必须大于 0");
    }
    if (stddev_mul_thresh_ <= 0.0) {
      throw std::runtime_error("sor_stddev_mul_thresh 必须大于 0");
    }
    if (cluster_tolerance_ <= 0.0) {
      throw std::runtime_error("cluster_tolerance 必须大于 0");
    }
    if (distance_threshold_ <= 0.0) {
      throw std::runtime_error("ransac_distance_threshold 必须大于 0");
    }
    if (max_iterations_ < 1) {
      throw std::runtime_error("ransac_max_iterations 必须大于 0");
    }
    if (probability_ <= 0.0 || probability_ > 1.0) {
      throw std::runtime_error("ransac_probability 必须在 (0, 1] 内");
    }
    if (min_plane_points_ < 3) {
      throw std::runtime_error("min_plane_points 必须不小于 3");
    }
    if (min_plane_separation_ < 0.0) {
      throw std::runtime_error("min_plane_separation 不能为负");
    }
  }

  void publish()
  {
    const auto stamp = now();
    cloud_msg_.header.stamp = stamp;
    publisher_->publish(cloud_msg_);
  }

  std::string pcd_file_;
  std::string frame_id_;
  int mean_k_{50};
  double stddev_mul_thresh_{1.0};
  double cluster_tolerance_{0.02};
  double distance_threshold_{0.01};
  int max_iterations_{1000};
  double probability_{0.99};
  int min_plane_points_{1000};
  double min_plane_separation_{0.05};
  bool table_is_closer_{true};
  double publish_rate_{1.0};
  std::string output_pcd_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TableExtractNode>();
  try {
    node->start();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
