#ifndef POINTCLOUD_PREPROCESSOR__DISTANCE_BASED_COMPARE_MAP_FILTER_NODE_HPP_
#define POINTCLOUD_PREPROCESSOR__DISTANCE_BASED_COMPARE_MAP_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "pointcloud_preprocessor/filter.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_msgs/msg/point_indices.hpp>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>  // pcl::search::KdTree 用

namespace pointcloud_preprocessor
{

class DistanceBasedCompareMapFilterNode : public Filter
{
public:
  explicit DistanceBasedCompareMapFilterNode(const rclcpp::NodeOptions & options);

  // using PointCloud2 = sensor_msgs::msg::PointCloud2;
  // using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  // using PointIndices = pcl_msgs::msg::PointIndices;
  // using PointIndicesConstPtr = pcl_msgs::msg::PointIndices::ConstSharedPtr;

  // DistanceBasedCompareMapFilterNode();

protected:
  void input_target_callback(const PointCloud2::SharedPtr msg);
  void filter(
    const PointCloud2ConstPtr & input,
    const PointIndicesConstPtr & indices,
    PointCloud2 & output) override;

  double distance_threshold_{0.3};  // デフォルト値は旧コードのdynamic_reconfigureより
  PointCloud2ConstPtr map_ptr_;
  // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_;
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__DISTANCE_BASED_COMPARE_MAP_FILTER_NODE_HPP_
