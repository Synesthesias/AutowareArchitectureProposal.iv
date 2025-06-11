#include "pose2twist/pose2twist_core.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace
{
double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad > M_PI) {
    diff_rad -= 2.0 * M_PI;
  } else if (diff_rad < -M_PI) {
    diff_rad += 2.0 * M_PI;
  }
  return diff_rad;
}

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseStamped & pose)
{
  return getRPY(pose.pose);
}

geometry_msgs::msg::TwistStamped calcTwist(
  const geometry_msgs::msg::PoseStamped & pose_a,
  const geometry_msgs::msg::PoseStamped & pose_b)
{
  rclcpp::Time time_a(pose_a.header.stamp);
  rclcpp::Time time_b(pose_b.header.stamp);
  const double dt = (time_b - time_a).seconds();

  geometry_msgs::msg::TwistStamped twist;
  twist.header = pose_b.header;
  twist.header.frame_id = "base_link";

  if (dt == 0.0) return twist;

  const auto pose_a_rpy = getRPY(pose_a);
  const auto pose_b_rpy = getRPY(pose_b);

  geometry_msgs::msg::Vector3 diff_xyz;
  geometry_msgs::msg::Vector3 diff_rpy;

  diff_xyz.x = pose_b.pose.position.x - pose_a.pose.position.x;
  diff_xyz.y = pose_b.pose.position.y - pose_a.pose.position.y;
  diff_xyz.z = pose_b.pose.position.z - pose_a.pose.position.z;
  diff_rpy.x = calcDiffForRadian(pose_b_rpy.x, pose_a_rpy.x);
  diff_rpy.y = calcDiffForRadian(pose_b_rpy.y, pose_a_rpy.y);
  diff_rpy.z = calcDiffForRadian(pose_b_rpy.z, pose_a_rpy.z);

  twist.twist.linear.x =
    std::sqrt(std::pow(diff_xyz.x, 2.0) + std::pow(diff_xyz.y, 2.0) + std::pow(diff_xyz.z, 2.0)) / dt;
  twist.twist.angular.x = diff_rpy.x / dt;
  twist.twist.angular.y = diff_rpy.y / dt;
  twist.twist.angular.z = diff_rpy.z / dt;

  return twist;
}
}  // namespace

Pose2Twist::Pose2Twist(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose", 10, std::bind(&Pose2Twist::callback, this, std::placeholders::_1));

  pub_twist_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("twist", 10);
  pub_linear_x_ = node_->create_publisher<std_msgs::msg::Float32>("linear_x", 10);
  pub_angular_z_ = node_->create_publisher<std_msgs::msg::Float32>("angular_z", 10);
}

void Pose2Twist::callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  static geometry_msgs::msg::PoseStamped prev_pose = *msg;

  geometry_msgs::msg::TwistStamped twist_msg = calcTwist(prev_pose, *msg);
  prev_pose = *msg;

  pub_twist_->publish(twist_msg);

  std_msgs::msg::Float32 linear_x_msg;
  linear_x_msg.data = twist_msg.twist.linear.x;
  pub_linear_x_->publish(linear_x_msg);

  std_msgs::msg::Float32 angular_z_msg;
  angular_z_msg.data = twist_msg.twist.angular.z;
  pub_angular_z_->publish(angular_z_msg);
}
