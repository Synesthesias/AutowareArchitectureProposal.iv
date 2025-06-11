#ifndef POSE_INITIALIZER_CORE_HPP
#define POSE_INITIALIZER_CORE_HPP

#include <memory>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <autoware_localization_srvs/srv/pose_with_covariance_stamped.hpp>

class PoseInitializer : public rclcpp::Node
{
public:
  PoseInitializer();

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Client<autoware_localization_srvs::srv::PoseWithCovarianceStamped>::SharedPtr ndt_client_;
  rclcpp::Service<autoware_localization_srvs::srv::PoseWithCovarianceStamped>::SharedPtr srv_server_;

  std::string map_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr_;

  void callbackMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void callbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void callbackGNSSPoseCov(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  bool getHeight(const geometry_msgs::msg::PoseWithCovarianceStamped & input,
                 geometry_msgs::msg::PoseWithCovarianceStamped & output);
  bool callAlignService(const geometry_msgs::msg::PoseWithCovarianceStamped & input,
                        geometry_msgs::msg::PoseWithCovarianceStamped & output);
  bool serviceInitial(
    const std::shared_ptr<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Request> req,
    std::shared_ptr<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Response> res);
};

#endif  // POSE_INITIALIZER_CORE_HPP
