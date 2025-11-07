#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <autoware_localization_srvs/srv/pose_with_covariance_stamped.hpp>

namespace pose_initializer
{
class PoseInitializer : public rclcpp::Node
{
public:
  explicit PoseInitializer(const rclcpp::NodeOptions & options);
  ~PoseInitializer() override = default;

private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PoseWithCovarianceSrv = autoware_localization_srvs::srv::PoseWithCovarianceStamped;

  void handleInitialPose(const PoseWithCovarianceStamped::SharedPtr msg);
  void handleGnssPose(const PoseWithCovarianceStamped::SharedPtr msg);
  void handleMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void handleService(
    const std::shared_ptr<PoseWithCovarianceSrv::Request> request,
    std::shared_ptr<PoseWithCovarianceSrv::Response> response);

  bool getHeight(const PoseWithCovarianceStamped & input_pose_msg, PoseWithCovarianceStamped & output_pose_msg);
  bool callAlignService(const PoseWithCovarianceStamped & input_pose_msg, PoseWithCovarianceStamped & output_pose_msg);
  void shutdownGnssSubscription();

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  rclcpp::Client<PoseWithCovarianceSrv>::SharedPtr ndt_client_;
  rclcpp::Service<PoseWithCovarianceSrv>::SharedPtr pose_initializer_service_;

  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex map_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr_;
  std::string map_frame_;
  bool use_first_gnss_topic_;
};
}  // namespace pose_initializer
