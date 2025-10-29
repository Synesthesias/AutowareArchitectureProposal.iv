#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pointcloud_preprocessor/visibility_control.hpp"

namespace pointcloud_preprocessor
{
class ConcatenateDataNode : public rclcpp::Node
{
public:
  POINTCLOUD_PREPROCESSOR_PUBLIC explicit ConcatenateDataNode(const rclcpp::NodeOptions & options);

private:
  void on_pointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const std::string & topic_name);
  void on_twist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void on_timeout();

  sensor_msgs::msg::PointCloud2::SharedPtr transform_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud) const;
  sensor_msgs::msg::PointCloud2::SharedPtr merge_clouds(
    const sensor_msgs::msg::PointCloud2 & base,
    const sensor_msgs::msg::PointCloud2 & addition) const;

  void publish();
  void reset_buffers();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr concat_num_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr skipped_topic_pub_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subs_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr pointcloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr twist_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Clock::SharedPtr clock_;

  std::vector<std::string> input_topics_;
  std::string output_frame_;
  std::string twist_topic_;
  double timeout_sec_;
  int queue_size_;
  bool use_twist_compensation_;
  double max_twist_dt_;  // used to reject stale twist samples
  bool timer_active_;

  mutable std::mutex mutex_;
  std::map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> clouds_;
  std::map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> pending_clouds_;
  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_buffer_;
};
}  // namespace pointcloud_preprocessor
