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

#ifndef POSE2TWIST__POSE2TWIST_COMPONENT_HPP_
#define POSE2TWIST__POSE2TWIST_COMPONENT_HPP_

#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace pose2twist
{

class Pose2TwistComponent : public rclcpp::Node
{
public:
  explicit Pose2TwistComponent(const rclcpp::NodeOptions & options);

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  geometry_msgs::msg::TwistStamped compute_twist(
    const geometry_msgs::msg::PoseStamped & previous,
    const geometry_msgs::msg::PoseStamped & current);

  std::string twist_frame_id_;
  std::optional<geometry_msgs::msg::PoseStamped> previous_pose_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr linear_x_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_z_publisher_;
};

}  // namespace pose2twist

#endif  // POSE2TWIST__POSE2TWIST_COMPONENT_HPP_
