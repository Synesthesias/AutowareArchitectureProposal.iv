#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "pointcloud_preprocessor/visibility_control.hpp"

namespace pointcloud_preprocessor
{
class DistanceBasedCompareMapFilterNode : public rclcpp::Node
{
public:
  POINTCLOUD_PREPROCESSOR_PUBLIC explicit DistanceBasedCompareMapFilterNode(
    const rclcpp::NodeOptions & options);

private:
  void on_input(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void on_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameter_event(
    const std::vector<rclcpp::Parameter> & parameters);

  bool transform_cloud_to_frame(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output,
    const std::string & target_frame) const;

  bool try_transform_cloud(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output,
    const std::string & target_frame) const;

  void rebuild_search_structure(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_xyz);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
  rclcpp::CallbackGroup::SharedPtr map_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Clock::SharedPtr clock_;

  double distance_threshold_;  // [m]
  double distance_threshold_sq_;
  double tf_timeout_sec_;
  std::string target_frame_;

  std::string map_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_xyz_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr map_tree_;

  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
}  // namespace pointcloud_preprocessor
