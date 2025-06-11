#include "pointcloud_preprocessor/distance_based_compare_map_filter_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <vector>

namespace pointcloud_preprocessor {

DistanceBasedCompareMapFilterNode::DistanceBasedCompareMapFilterNode(const rclcpp::NodeOptions & options)
: Filter("distance_based_compare_map_filter", options)
{
  distance_threshold_ = this->declare_parameter("distance_threshold", 0.3); // 0 ~ 1.0
  sub_map_ = this->create_subscription<PointCloud2>(
    "map", 1,
    std::bind(&DistanceBasedCompareMapFilterNode::input_target_callback, this, std::placeholders::_1));
  tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
}

void DistanceBasedCompareMapFilterNode::input_target_callback(const PointCloud2::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_ptr_ = msg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*map_ptr_, *pcl_map);
  tree_->setInputCloud(pcl_map);
}

void pointcloud_preprocessor::DistanceBasedCompareMapFilterNode::filter(const PointCloud2ConstPtr & input, const PointIndicesConstPtr & indices, PointCloud2 & output)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_ptr_ || !tree_) {
    output = *input;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZINormal>());
  pcl::fromROSMsg(*input, *pcl_xyz);
  pcl::fromROSMsg(*input, *pcl_input);

  pcl_output->points.reserve(pcl_input->points.size());
  double threshold = distance_threshold_ * distance_threshold_;

  std::vector<int> nn_indices(1);
  std::vector<float> nn_distances(1);

  for (size_t i = 0; i < pcl_xyz->points.size(); ++i) {
    if (!tree_->nearestKSearch(pcl_xyz->points[i], 1, nn_indices, nn_distances)) continue;
    if (nn_distances[0] < threshold) {
      pcl_output->points.push_back(pcl_input->points[i]);
    }
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

}  // namespace pointcloud_preprocessor
