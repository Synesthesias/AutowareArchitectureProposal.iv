#ifndef POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA_NODE_HPP_
#define POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <mutex>
#include <deque>
#include <map>
#include <vector>
#include <string>
#include <memory>

namespace pointcloud_preprocessor
{

class ConcatenateDataNode : public rclcpp::Node
{
public:
  explicit ConcatenateDataNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using TwistStamped = geometry_msgs::msg::TwistStamped;

  void cloud_callback(const PointCloud2::SharedPtr msg, const std::string & topic_name);
  void twist_callback(const TwistStamped::SharedPtr msg);
  void timer_callback();
  void transformPointCloud(const PointCloud2::SharedPtr & in, PointCloud2::SharedPtr & out);
  void combineClouds(const PointCloud2::SharedPtr & in1, const PointCloud2::SharedPtr & in2, PointCloud2::SharedPtr & out);
  void convertToXYZCloud(const PointCloud2 & input, PointCloud2 & output);
  void publish();

  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> filters_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_concat_num_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_not_subscribed_topic_name_;

  std::map<std::string, PointCloud2::SharedPtr> cloud_stdmap_;
  std::map<std::string, PointCloud2::SharedPtr> cloud_stdmap_tmp_;
  std::deque<TwistStamped::SharedPtr> twist_ptr_queue_;

  std::vector<std::string> input_topics_;
  std::string output_frame_;
  int maximum_queue_size_ = 10;
  double timeout_sec_ = 1.0;
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA_NODE_HPP_