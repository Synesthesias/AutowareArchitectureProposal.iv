#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/callback_group.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "pointcloud_preprocessor/visibility_control.hpp"

namespace pointcloud_preprocessor
{
class FilterNodeBase : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  POINTCLOUD_PREPROCESSOR_PUBLIC explicit FilterNodeBase(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  virtual bool filter(const PointCloud2 & input, PointCloud2 & output) = 0;

  virtual rcl_interfaces::msg::SetParametersResult handle_filter_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  tf2_ros::Buffer & tf_buffer() const { return *tf_buffer_; }

  rclcpp::Clock::SharedPtr ros_clock() const { return clock_; }

  double tf_timeout_sec() const { return tf_timeout_sec_; }
  const std::string & input_frame() const { return input_frame_; }
  const std::string & output_frame() const { return output_frame_; }

  void set_output_qos(const rclcpp::QoS & qos);

private:
  void process_pointcloud(const PointCloud2::ConstSharedPtr msg);
  bool transform_cloud(
    const PointCloud2 & input, PointCloud2 & output, const std::string & target_frame) const;

  rclcpp::Subscription<PointCloud2>::SharedPtr input_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr output_pub_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Clock::SharedPtr clock_;

  double tf_timeout_sec_;
  std::string input_frame_;
  std::string output_frame_;

  mutable std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
}  // namespace pointcloud_preprocessor
