#include "pointcloud_preprocessor/filter_node_base.hpp"

#include <functional>
#include <utility>

#include <rclcpp/subscription_options.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace pointcloud_preprocessor
{
FilterNodeBase::FilterNodeBase(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  clock_ = this->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 0.2);
  input_frame_ = this->declare_parameter<std::string>("input_frame", "");
  output_frame_ = this->declare_parameter<std::string>("output_frame", "");

  sensor_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto qos = rclcpp::SensorDataQoS();
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sensor_callback_group_;
  input_sub_ = this->create_subscription<PointCloud2>(
    "input", qos,
    std::bind(&FilterNodeBase::process_pointcloud, this, std::placeholders::_1), sub_options);
  output_pub_ = this->create_publisher<PointCloud2>("output", qos);

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      const std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & param : params) {
        if (param.get_name() == "tf_timeout_sec") {
          tf_timeout_sec_ = param.as_double();
        } else if (param.get_name() == "input_frame") {
          input_frame_ = param.as_string();
        } else if (param.get_name() == "output_frame") {
          output_frame_ = param.as_string();
        }
      }
      return handle_filter_parameters(params);
    });
}

void FilterNodeBase::set_output_qos(const rclcpp::QoS & qos)
{
  output_pub_ = this->create_publisher<PointCloud2>("output", qos);
}

rcl_interfaces::msg::SetParametersResult FilterNodeBase::handle_filter_parameters(
  const std::vector<rclcpp::Parameter> &)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void FilterNodeBase::process_pointcloud(const PointCloud2::ConstSharedPtr msg)
{
  PointCloud2 transformed_input = *msg;
  if (!input_frame_.empty() && msg->header.frame_id != input_frame_) {
    if (!transform_cloud(*msg, transformed_input, input_frame_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 2000,
        "Failed to transform input cloud from %s to %s", msg->header.frame_id.c_str(),
        input_frame_.c_str());
      return;
    }
  }

  PointCloud2 filtered_cloud;
  if (!filter(transformed_input, filtered_cloud)) {
    RCLCPP_DEBUG(this->get_logger(), "Filter returned false; skipping publish");
    return;
  }

  filtered_cloud.header.stamp = msg->header.stamp;

  PointCloud2 output_cloud = filtered_cloud;
  if (!output_frame_.empty() && filtered_cloud.header.frame_id != output_frame_) {
    if (!transform_cloud(filtered_cloud, output_cloud, output_frame_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 2000,
        "Failed to transform output cloud to %s", output_frame_.c_str());
      return;
    }
  } else if (output_frame_.empty() && filtered_cloud.header.frame_id != msg->header.frame_id) {
    if (!transform_cloud(filtered_cloud, output_cloud, msg->header.frame_id)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 2000,
        "Failed to restore cloud to original frame %s", msg->header.frame_id.c_str());
      return;
    }
  }

  output_cloud.header.stamp = msg->header.stamp;
  output_pub_->publish(output_cloud);
}

bool FilterNodeBase::transform_cloud(
  const PointCloud2 & input, PointCloud2 & output, const std::string & target_frame) const
{
  if (target_frame.empty() || input.header.frame_id == target_frame) {
    output = input;
    output.header.frame_id = target_frame.empty() ? input.header.frame_id : target_frame;
    return true;
  }

  try {
    const auto timeout = tf2::durationFromSec(tf_timeout_sec_);
    const auto transform = tf_buffer_->lookupTransform(
      target_frame, input.header.frame_id, tf2::TimePointZero, timeout);
    tf2::doTransform(input, output, transform);
    output.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "Transform lookup failed (%s -> %s): %s", input.header.frame_id.c_str(),
      target_frame.c_str(), ex.what());
    return false;
  }
}

}  // namespace pointcloud_preprocessor
