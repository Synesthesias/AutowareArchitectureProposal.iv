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
/**
 * Copyright 2020 Tier IV, Inc.
 */

#include "gnss_poser/gnss_poser_core.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <tf2/LinearMath/Quaternion.h>

namespace GNSSPoser
{
using std::placeholders::_1;

GNSSPoser::GNSSPoser(const rclcpp::NodeOptions & options)
: rclcpp::Node("gnss_poser", options),
  clock_(this->get_clock()),
  tf2_buffer_(clock_),
  coordinate_system_(CoordinateSystem::MGRS),
  base_frame_("base_link"),
  gnss_frame_("gnss"),
  gnss_base_frame_("gnss_base_link"),
  map_frame_("map"),
  plane_zone_(9),
  position_buffer_(1)
{
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(tf2_buffer_, this, false);

  tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  coordinate_system_ = static_cast<CoordinateSystem>(this->declare_parameter<int>(
    "coordinate_system", static_cast<int>(coordinate_system_)));
  base_frame_ = this->declare_parameter<std::string>("base_frame", base_frame_);
  gnss_frame_ = this->declare_parameter<std::string>("gnss_frame", gnss_frame_);
  gnss_base_frame_ = this->declare_parameter<std::string>("gnss_base_frame", gnss_base_frame_);
  map_frame_ = this->declare_parameter<std::string>("map_frame", map_frame_);
  plane_zone_ = this->declare_parameter<int>("plane_zone", plane_zone_);

  const int buff_epoch_param = this->declare_parameter<int>("buff_epoch", 1);
  const int buff_epoch = std::max(1, buff_epoch_param);
  position_buffer_.set_capacity(buff_epoch);

  nav_sat_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix", rclcpp::QoS(10), std::bind(&GNSSPoser::callbackNavSatFix, this, _1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", rclcpp::QoS(10));
  pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", rclcpp::QoS(10));
  fixed_pub_ = this->create_publisher<std_msgs::msg::Bool>("gnss_fixed", rclcpp::QoS(10));
}

void GNSSPoser::callbackNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr)
{
  const bool is_fixed = isFixed(nav_sat_fix_msg_ptr->status);

  std_msgs::msg::Bool is_fixed_msg;
  is_fixed_msg.data = is_fixed;
  fixed_pub_->publish(is_fixed_msg);

  if (!is_fixed) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *clock_, 1000, "Not fixed topic. Skipping calculation.");
    return;
  }

  const auto gnss_stat = convert(*nav_sat_fix_msg_ptr, coordinate_system_);
  const auto position = getPosition(gnss_stat);

  position_buffer_.push_front(position);
  if (!position_buffer_.full()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *clock_, 1000, "Buffering position. Output skipped.");
    return;
  }
  const auto median_position = getMedianPosition(position_buffer_);

  geometry_msgs::msg::Quaternion orientation;
  if (has_prev_position_) {
    orientation = getQuaternionByPositionDiffence(median_position, prev_position_);
  } else {
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);
    orientation = tf2::toMsg(quaternion);
  }
  prev_position_ = median_position;
  has_prev_position_ = true;

  geometry_msgs::msg::PoseStamped gnss_antenna_pose_msg;
  gnss_antenna_pose_msg.header.stamp = nav_sat_fix_msg_ptr->header.stamp;
  gnss_antenna_pose_msg.header.frame_id = map_frame_;
  gnss_antenna_pose_msg.pose.position = median_position;
  gnss_antenna_pose_msg.pose.orientation = orientation;

  geometry_msgs::msg::TransformStamped tf_base_to_gnss;
  getStaticTransform(
    gnss_frame_, base_frame_, tf_base_to_gnss, rclcpp::Time(nav_sat_fix_msg_ptr->header.stamp));

  tf_base_to_gnss.transform.rotation.x = 0.0;
  tf_base_to_gnss.transform.rotation.y = 0.0;
  tf_base_to_gnss.transform.rotation.z = 0.0;
  tf_base_to_gnss.transform.rotation.w = 1.0;

  geometry_msgs::msg::PoseStamped gnss_base_pose_msg;
  tf2::doTransform(gnss_antenna_pose_msg, gnss_base_pose_msg, tf_base_to_gnss);
  gnss_base_pose_msg.header.frame_id = map_frame_;

  pose_pub_->publish(gnss_base_pose_msg);

  geometry_msgs::msg::PoseWithCovarianceStamped gnss_base_pose_cov_msg;
  gnss_base_pose_cov_msg.header = gnss_base_pose_msg.header;
  gnss_base_pose_cov_msg.pose.pose = gnss_base_pose_msg.pose;
  gnss_base_pose_cov_msg.pose.covariance[6 * 0 + 0] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[0] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[6 * 1 + 1] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[4] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[6 * 2 + 2] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[8] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[6 * 3 + 3] = 0.1;
  gnss_base_pose_cov_msg.pose.covariance[6 * 4 + 4] = 0.1;
  gnss_base_pose_cov_msg.pose.covariance[6 * 5 + 5] = 1.0;
  pose_cov_pub_->publish(gnss_base_pose_cov_msg);

  publishTF(map_frame_, gnss_base_frame_, gnss_base_pose_msg);
}

bool GNSSPoser::isFixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg) const
{
  return nav_sat_status_msg.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

bool GNSSPoser::canGetCovariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg) const
{
  return nav_sat_fix_msg.position_covariance_type >
         sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

GNSSStat GNSSPoser::convert(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system) const
{
  GNSSStat gnss_stat;
  if (coordinate_system == CoordinateSystem::UTM) {
    gnss_stat = NavSatFix2UTM(nav_sat_fix_msg);
  } else if (coordinate_system == CoordinateSystem::MGRS) {
    gnss_stat = NavSatFix2MGRS(nav_sat_fix_msg, MGRSPrecision::_100MICRO_METER);
  } else if (coordinate_system == CoordinateSystem::PLANE) {
    gnss_stat = NavSatFix2PLANE(nav_sat_fix_msg, plane_zone_);
  } else {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *clock_, 1000, "Unknown coordinate system");
  }
  return gnss_stat;
}

geometry_msgs::msg::Point GNSSPoser::getPosition(const GNSSStat & gnss_stat) const
{
  geometry_msgs::msg::Point point;
  point.x = gnss_stat.x;
  point.y = gnss_stat.y;
  point.z = gnss_stat.z;
  return point;
}

geometry_msgs::msg::Point GNSSPoser::getMedianPosition(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer) const
{
  auto getMedian = [](std::vector<double> array) {
    std::sort(array.begin(), array.end());
    const size_t median_index = array.size() / 2;
    const double median = (array.size() % 2 == 1)
      ? array.at(median_index)
      : ((array.at(median_index) + array.at(median_index - 1)) / 2.0);
    return median;
  };

  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  array_x.reserve(position_buffer.size());
  array_y.reserve(position_buffer.size());
  array_z.reserve(position_buffer.size());
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point median_point;
  median_point.x = getMedian(array_x);
  median_point.y = getMedian(array_y);
  median_point.z = getMedian(array_z);
  return median_point;
}

geometry_msgs::msg::Quaternion GNSSPoser::getQuaternionByPositionDiffence(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point) const
{
  const double yaw = std::atan2(point.y - prev_point.y, point.x - prev_point.x);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

bool GNSSPoser::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform_stamped)
{
  if (target_frame == source_frame) {
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = target_frame;
    transform_stamped.child_frame_id = source_frame;
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    return true;
  }

  try {
    transform_stamped = tf2_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000, "%s", ex.what());
    RCLCPP_WARN_THROTTLE(
      get_logger(), *clock_, 1000,
      "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = target_frame;
    transform_stamped.child_frame_id = source_frame;
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool GNSSPoser::getStaticTransform(
  const std::string & target_frame, const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform_stamped, const rclcpp::Time & stamp)
{
  if (target_frame == source_frame) {
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = target_frame;
    transform_stamped.child_frame_id = source_frame;
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    return true;
  }

  try {
    transform_stamped = tf2_buffer_.lookupTransform(target_frame, source_frame, stamp);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000, "%s", ex.what());
    RCLCPP_WARN_THROTTLE(
      get_logger(), *clock_, 1000,
      "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = target_frame;
    transform_stamped.child_frame_id = source_frame;
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void GNSSPoser::publishTF(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::msg::PoseStamped & pose_msg)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, quaternion);
  transform_stamped.transform.rotation.x = quaternion.x();
  transform_stamped.transform.rotation.y = quaternion.y();
  transform_stamped.transform.rotation.z = quaternion.z();
  transform_stamped.transform.rotation.w = quaternion.w();

  tf2_broadcaster_->sendTransform(transform_stamped);
}

}  // namespace GNSSPoser
