#include "pose_initializer/pose_initializer_core.hpp"

PoseInitializer::PoseInitializer() : Node("pose_initializer")
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received odometry");
    });

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      RCLCPP_INFO(this->get_logger(), "Received IMU");
    });

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10);
}
