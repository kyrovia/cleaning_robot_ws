#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/filters/extract_indices.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl_conversions/pcl_conversions.h"

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZRGB>;

struct ClusterResult
{
  Cloud::Ptr cloud;
  std::size_t input_count{0};
  std::size_t finite_count{0};
  std::size_t cluster_count{0};
};

std::filesystem::path resolve_package_path(const std::string & relative)
{
  const std::filesystem::path path(relative);
  if (path.is_absolute()) {
    throw std::runtime_error("路径必须相对包 share 目录");
  }

  const auto share = ament_index_cpp::get_package_share_directory("point_extract");
  return std::filesystem::path(share) / path;
}

ClusterResult keep_largest_cluster(
  const Cloud::ConstPtr & input, double tolerance, int min_cluster_size)
{
  auto finite = std::make_shared<Cloud>();
  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(*input, *finite, finite_indices);
  if (finite->empty()) {
    throw std::runtime_error("点云去掉无效点后为空");
  }

  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
  tree->setInputCloud(finite);
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
  clustering.setClusterTolerance(tolerance);
  clustering.setMinClusterSize(min_cluster_size);
  clustering.setMaxClusterSize(static_cast<int>(finite->size()));
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(finite);
  clustering.extract(clusters);
  if (clusters.empty()) {
    throw std::runtime_error("欧式聚类没有得到簇");
  }

  const auto largest = std::max_element(
    clusters.begin(), clusters.end(),
    [](const pcl::PointIndices & left, const pcl::PointIndices & right) {
      return left.indices.size() < right.indices.size();
    });
  auto indices = std::make_shared<pcl::PointIndices>(*largest);
  std::sort(indices->indices.begin(), indices->indices.end());

  auto cloud = std::make_shared<Cloud>();
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(finite);
  extract.setIndices(indices);
  extract.filter(*cloud);
  if (cloud->empty()) {
    throw std::runtime_error("最大簇为空");
  }

  ClusterResult result;
  result.cloud = cloud;
  result.input_count = input->size();
  result.finite_count = finite->size();
  result.cluster_count = clusters.size();
  return result;
}

void save_cloud(const std::string & relative, const Cloud & cloud)
{
  const auto path = resolve_package_path(relative);
  std::filesystem::create_directories(path.parent_path());
  if (pcl::io::savePCDFileBinary(path.string(), cloud) < 0) {
    throw std::runtime_error("无法保存聚类点云: " + relative);
  }
}

}  // namespace

class ClusterExtractNode : public rclcpp::Node
{
public:
  ClusterExtractNode()
  : Node("cluster_extract_node")
  {
    pcd_file_ = declare_parameter<std::string>("pcd_file", "data/table_01.pcd");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");
    const auto topic = declare_parameter<std::string>("topic", "table_cluster");
    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.02);
    min_cluster_size_ = declare_parameter<int>("min_cluster_size", 1);
    publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
    output_pcd_ = declare_parameter<std::string>("output_pcd", "data/table_cluster.pcd");

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

    const auto result = keep_largest_cluster(input, cluster_tolerance_, min_cluster_size_);
    pcl::toROSMsg(*result.cloud, cloud_msg_);
    cloud_msg_.header.frame_id = frame_id_;

    RCLCPP_INFO(get_logger(), "读取点云 %zu 点", result.input_count);
    RCLCPP_INFO(get_logger(), "去掉无效点后 %zu 点", result.finite_count);
    RCLCPP_INFO(
      get_logger(), "欧式聚类得到 %zu 个簇，最大簇 %zu 点",
      result.cluster_count, result.cloud->size());
    if (!output_pcd_.empty()) {
      save_cloud(output_pcd_, *result.cloud);
      RCLCPP_INFO(get_logger(), "聚类点云已写入 %s", output_pcd_.c_str());
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
    if (cluster_tolerance_ <= 0.0) {
      throw std::runtime_error("cluster_tolerance 必须大于 0");
    }
    if (min_cluster_size_ < 1) {
      throw std::runtime_error("min_cluster_size 必须大于 0");
    }
    if (publish_rate_ < 0.0) {
      throw std::runtime_error("publish_rate 不能为负");
    }
  }

  void publish()
  {
    cloud_msg_.header.stamp = now();
    publisher_->publish(cloud_msg_);
  }

  std::string pcd_file_;
  std::string frame_id_;
  double cluster_tolerance_{0.02};
  int min_cluster_size_{1};
  double publish_rate_{1.0};
  std::string output_pcd_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClusterExtractNode>();
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
