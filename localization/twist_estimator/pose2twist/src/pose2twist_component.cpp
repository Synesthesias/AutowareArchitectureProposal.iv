// Copyright 2015-2025 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pose2twist/pose2twist_component.hpp"

#include <cmath>
#include <functional>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
double normalize_angle_difference(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad > M_PI) {
    diff_rad -= 2.0 * M_PI;
  } else if (diff_rad < -M_PI) {
    diff_rad += 2.0 * M_PI;
  }
  return diff_rad;
}

auto get_rpy(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Vector3 rpy;
  const tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

}  // namespace

namespace pose2twist
{

Pose2TwistComponent::Pose2TwistComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("pose2twist", options)
{
  twist_frame_id_ = this->declare_parameter<std::string>("twist_frame_id", "base_link");

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;

  pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose", rclcpp::SensorDataQoS(),
    std::bind(&Pose2TwistComponent::on_pose, this, std::placeholders::_1),
    subscription_options);

  const auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("twist", default_qos);
  linear_x_publisher_ = this->create_publisher<std_msgs::msg::Float32>("linear_x", default_qos);
  angular_z_publisher_ = this->create_publisher<std_msgs::msg::Float32>("angular_z", default_qos);
}

void Pose2TwistComponent::on_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (!previous_pose_) {
    previous_pose_ = *msg;

    auto zero_twist = geometry_msgs::msg::TwistStamped{};
    zero_twist.header = msg->header;
    zero_twist.header.frame_id = twist_frame_id_;
    twist_publisher_->publish(zero_twist);

    std_msgs::msg::Float32 linear{};
    std_msgs::msg::Float32 angular{};
    linear.data = 0.0F;
    angular.data = 0.0F;
    linear_x_publisher_->publish(linear);
    angular_z_publisher_->publish(angular);
    return;
  }

  auto twist = compute_twist(previous_pose_.value(), *msg);
  twist_publisher_->publish(twist);

  std_msgs::msg::Float32 linear;
  std_msgs::msg::Float32 angular;
  linear.data = static_cast<float>(twist.twist.linear.x);
  angular.data = static_cast<float>(twist.twist.angular.z);
  linear_x_publisher_->publish(linear);
  angular_z_publisher_->publish(angular);

  previous_pose_ = *msg;
}

geometry_msgs::msg::TwistStamped Pose2TwistComponent::compute_twist(
  const geometry_msgs::msg::PoseStamped & previous,
  const geometry_msgs::msg::PoseStamped & current)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header = current.header;
  twist.header.frame_id = twist_frame_id_;

  const rclcpp::Time previous_stamp(previous.header.stamp);
  const rclcpp::Time current_stamp(current.header.stamp);
  const auto duration = current_stamp - previous_stamp;
  const double dt = duration.seconds();

  if (dt <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Received pose messages with non-positive delta time (%.6f s); publishing zero twist", dt);
    return twist;
  }

  const auto previous_rpy = get_rpy(previous.pose);
  const auto current_rpy = get_rpy(current.pose);

  const double diff_x = current.pose.position.x - previous.pose.position.x;
  const double diff_y = current.pose.position.y - previous.pose.position.y;
  const double diff_z = current.pose.position.z - previous.pose.position.z;

  const double diff_roll = normalize_angle_difference(current_rpy.x, previous_rpy.x);
  const double diff_pitch = normalize_angle_difference(current_rpy.y, previous_rpy.y);
  const double diff_yaw = normalize_angle_difference(current_rpy.z, previous_rpy.z);

  twist.twist.linear.x = std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) / dt;
  twist.twist.linear.y = 0.0;
  twist.twist.linear.z = 0.0;
  twist.twist.angular.x = diff_roll / dt;
  twist.twist.angular.y = diff_pitch / dt;
  twist.twist.angular.z = diff_yaw / dt;

  return twist;
}

}  // namespace pose2twist

RCLCPP_COMPONENTS_REGISTER_NODE(pose2twist::Pose2TwistComponent)
