#include "pose_initializer/pose_initializer_core.hpp"
#include "autoware_localization_srvs/srv/pose_with_covariance_stamped.hpp"

#include <sensor_msgs/msg/point_field.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <limits>
#include <memory>
#include <chrono>
#include <cstring>

using namespace std::chrono_literals;
using PoseWithCovarianceStampedSrv = autoware_localization_srvs::srv::PoseWithCovarianceStamped;

double getGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcdmap, const tf2::Vector3 & point)
{
  constexpr double radius = 1.0 * 1.0;
  const double x = point.getX();
  const double y = point.getY();

  double height = std::numeric_limits<double>::infinity();
  for (const auto & p : pcdmap->points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }
  return std::isfinite(height) ? height : point.getZ();
}

// 自前の PointCloud2 → pcl::PointCloud<pcl::PointXYZ> 変換関数
pcl::PointCloud<pcl::PointXYZ>::Ptr convertPointCloud2ToXYZ(const sensor_msgs::msg::PointCloud2 & msg)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  int x_offset = -1, y_offset = -1, z_offset = -1;
  for (const auto & field : msg.fields) {
    if (field.name == "x") x_offset = field.offset;
    if (field.name == "y") y_offset = field.offset;
    if (field.name == "z") z_offset = field.offset;
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    throw std::runtime_error("PointCloud2 missing x/y/z fields");
  }

  const size_t point_step = msg.point_step;
  const size_t row_step = msg.row_step;
  const size_t num_points = msg.width * msg.height;

  cloud->points.resize(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t* row_data = &msg.data[i * point_step];
    std::memcpy(&cloud->points[i].x, row_data + x_offset, sizeof(float));
    std::memcpy(&cloud->points[i].y, row_data + y_offset, sizeof(float));
    std::memcpy(&cloud->points[i].z, row_data + z_offset, sizeof(float));
  }

  cloud->width = msg.width;
  cloud->height = msg.height;
  cloud->is_dense = false;

  return cloud;
}

PoseInitializer::PoseInitializer() : Node("pose_initializer")
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", rclcpp::QoS(1),
    std::bind(&PoseInitializer::callbackMapPoints, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(10),
    std::bind(&PoseInitializer::callbackInitialPose, this, std::placeholders::_1));

  gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", rclcpp::QoS(10),
    std::bind(&PoseInitializer::callbackGNSSPoseCov, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose3d", 10);

  ndt_client_ = this->create_client<PoseWithCovarianceStampedSrv>("ndt_align_srv");

  srv_server_ = this->create_service<PoseWithCovarianceStampedSrv>(
    "pose_initializer_srv",
    std::bind(&PoseInitializer::serviceInitial, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void PoseInitializer::callbackMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  map_frame_ = msg->header.frame_id;
  map_ptr_ = convertPointCloud2ToXYZ(*msg);
}

void PoseInitializer::callbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  getHeight(*msg, out);
  out.pose.covariance[0] = 2.0;
  out.pose.covariance[7] = 2.0;
  out.pose.covariance[14] = 0.01;
  out.pose.covariance[21] = 0.01;
  out.pose.covariance[28] = 0.01;
  out.pose.covariance[35] = 0.3;

  geometry_msgs::msg::PoseWithCovarianceStamped aligned;
  if (callAlignService(out, aligned)) {
    pose_pub_->publish(aligned);
  }
}

void PoseInitializer::callbackGNSSPoseCov(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseWithCovarianceStamped out;
  getHeight(*msg, out);
  out.pose.covariance[0] = 1.0;
  out.pose.covariance[7] = 1.0;
  out.pose.covariance[14] = 0.01;
  out.pose.covariance[21] = 0.01;
  out.pose.covariance[28] = 0.01;
  out.pose.covariance[35] = 3.14;

  geometry_msgs::msg::PoseWithCovarianceStamped aligned;
  if (callAlignService(out, aligned)) {
    pose_pub_->publish(aligned);
  }
}

bool PoseInitializer::getHeight(
  const geometry_msgs::msg::PoseWithCovarianceStamped & input,
  geometry_msgs::msg::PoseWithCovarianceStamped & output)
{
  tf2::Vector3 point(
    input.pose.pose.position.x,
    input.pose.pose.position.y,
    input.pose.pose.position.z);

  if (map_ptr_) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(map_frame_, input.header.frame_id, tf2::TimePointZero);

      tf2::Transform transform;
      tf2::fromMsg(transform_stamped.transform, transform);
      point = transform * point;
      point.setZ(getGroundHeight(map_ptr_, point));
      point = transform.inverse() * point;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  output = input;
  output.pose.pose.position.x = point.getX();
  output.pose.pose.position.y = point.getY();
  output.pose.pose.position.z = point.getZ();
  return true;
}

bool PoseInitializer::callAlignService(
  const geometry_msgs::msg::PoseWithCovarianceStamped & input,
  geometry_msgs::msg::PoseWithCovarianceStamped & output)
{
  if (!ndt_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(), "NDT align service not available");
    return false;
  }

  auto request = std::make_shared<PoseWithCovarianceStampedSrv::Request>();
  request->pose_with_cov = input;

  auto result_future = ndt_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 1s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call NDT align service");
    return false;
  }

  output = result_future.get()->pose_with_cov;
  return true;
}


bool PoseInitializer::serviceInitial(
  const std::shared_ptr<PoseWithCovarianceStampedSrv::Request> req,
  std::shared_ptr<PoseWithCovarianceStampedSrv::Response> res)
{
  gnss_pose_sub_.reset();  // shutdown subscriber

  geometry_msgs::msg::PoseWithCovarianceStamped temp_pose;
  getHeight(req->pose_with_cov, temp_pose);

  temp_pose.pose.covariance[0] = 1.0;
  temp_pose.pose.covariance[7] = 1.0;
  temp_pose.pose.covariance[14] = 0.01;
  temp_pose.pose.covariance[21] = 0.01;
  temp_pose.pose.covariance[28] = 0.01;
  temp_pose.pose.covariance[35] = 1.0;

  geometry_msgs::msg::PoseWithCovarianceStamped aligned;
  bool success = callAlignService(temp_pose, aligned);

  if (success) {
    pose_pub_->publish(aligned);
    res->pose_with_cov = aligned;
  }

  return success;
}
