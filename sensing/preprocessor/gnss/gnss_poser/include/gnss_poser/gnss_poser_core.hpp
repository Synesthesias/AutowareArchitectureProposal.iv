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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "gnss_poser/convert.hpp"
#include "gnss_poser/gnss_stat.hpp"

namespace GNSSPoser
{
class GNSSPoser : public rclcpp::Node
{
public:
  explicit GNSSPoser(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void callbackNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr);

  bool isFixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg) const;
  bool canGetCovariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg) const;
  GNSSStat convert(
    const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system) const;
  geometry_msgs::msg::Point getPosition(const GNSSStat & gnss_stat) const;
  geometry_msgs::msg::Point getMedianPosition(
    const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer) const;
  geometry_msgs::msg::Quaternion getQuaternionByPositionDiffence(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point) const;

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform_stamped);
  bool getStaticTransform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform_stamped, const rclcpp::Time & stamp);
  void publishTF(
    const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::msg::PoseStamped & pose_msg);

  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fixed_pub_;

  CoordinateSystem coordinate_system_;
  std::string base_frame_;
  std::string gnss_frame_;
  std::string gnss_base_frame_;
  std::string map_frame_;

  int plane_zone_;

  boost::circular_buffer<geometry_msgs::msg::Point> position_buffer_;
  geometry_msgs::msg::Point prev_position_{};
  bool has_prev_position_{false};
};

}  // namespace GNSSPoser
