#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

class Pose2Twist
{
public:
  explicit Pose2Twist(const rclcpp::Node::SharedPtr& node);

private:
  void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_linear_x_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_angular_z_;
};
