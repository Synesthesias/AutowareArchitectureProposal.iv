/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "map_tf_generator/map_tf_generator.hpp"

#include <algorithm>
#include <functional>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace map_tf_generator
{
namespace
{
constexpr char kDefaultMapFrame[] = "map";
constexpr char kDefaultViewerFrame[] = "viewer";
constexpr char kSubscriptionTopic[] = "pointcloud_map";
}  // namespace

MapTfGeneratorNode::MapTfGeneratorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("map_tf_generator", options)
{
  clock_ = this->get_clock();

  map_frame_ = this->declare_parameter<std::string>("map_frame", kDefaultMapFrame);
  viewer_frame_ = this->declare_parameter<std::string>("viewer_frame", kDefaultViewerFrame);

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    kSubscriptionTopic, sensor_qos,
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onPointCloud(msg); });

  RCLCPP_INFO(get_logger(), "map_tf_generator initialized. map_frame: %s, viewer_frame: %s",
    map_frame_.c_str(), viewer_frame_.c_str());
}

void MapTfGeneratorNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const std::size_t point_count = static_cast<std::size_t>(msg->width) * msg->height;
  if (point_count == 0U) {
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000, "Received empty pointcloud");
    return;
  }

  const auto has_field = [&msg](const std::string & name) {
    return std::any_of(
      msg->fields.begin(), msg->fields.end(),
      [&name](const sensor_msgs::msg::PointField & field) { return field.name == name; });
  };

  if (!has_field("x") || !has_field("y") || !has_field("z")) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *clock_, 1000, "Pointcloud lacks x/y/z fields. TF not generated.");
    return;
  }

  const auto transform = createTransform(*msg, point_count);
  tf_broadcaster_->sendTransform(transform);

  RCLCPP_INFO_THROTTLE(get_logger(), *clock_, 1000,
    "Broadcast static TF map_frame: %s viewer_frame: %s x: %.3f y: %.3f z: %.3f",
    map_frame_.c_str(), viewer_frame_.c_str(),
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z);
}

geometry_msgs::msg::TransformStamped MapTfGeneratorNode::createTransform(
  const sensor_msgs::msg::PointCloud2 & msg, std::size_t point_count) const
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  std::size_t valid_count = 0U;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    sum_x += static_cast<double>(*iter_x);
    sum_y += static_cast<double>(*iter_y);
    sum_z += static_cast<double>(*iter_z);
    ++valid_count;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header = msg.header;
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = viewer_frame_;

  const std::size_t count = valid_count == 0U ? point_count : valid_count;
  const double inv_count = 1.0 / static_cast<double>(count);
  transform.transform.translation.x = sum_x * inv_count;
  transform.transform.translation.y = sum_y * inv_count;
  transform.transform.translation.z = sum_z * inv_count;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, 0.0);
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();

  return transform;
}

}  // namespace map_tf_generator
