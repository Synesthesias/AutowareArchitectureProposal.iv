#include "pose_initializer/pose_initializer_component.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rmw/qos_profiles.h>

using namespace std::chrono_literals;

namespace pose_initializer
{
namespace
{
double getGroundHeight(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & map, const tf2::Vector3 & point)
{
  if (!map || map->empty()) {
    return point.getZ();
  }

  constexpr double radius_squared = 1.0 * 1.0;
  const double x = point.getX();
  const double y = point.getY();

  double height = std::numeric_limits<double>::infinity();
  for (const auto & map_point : map->points) {
    const double dx = x - map_point.x;
    const double dy = y - map_point.y;
    const double squared_distance = (dx * dx) + (dy * dy);
    if (squared_distance < radius_squared) {
      height = std::min(height, static_cast<double>(map_point.z));
    }
  }

  return std::isfinite(height) ? height : point.getZ();
}

void applyDefaultCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg, const double roll_cov)
{
  auto & covariance = pose_msg.pose.covariance;
  covariance[0] = 1.0;
  covariance[1 * 6 + 1] = 1.0;
  covariance[2 * 6 + 2] = 0.01;
  covariance[3 * 6 + 3] = 0.01;
  covariance[4 * 6 + 4] = 0.01;
  covariance[5 * 6 + 5] = roll_cov;
}
}  // namespace

PoseInitializer::PoseInitializer(const rclcpp::NodeOptions & options)
: Node("pose_initializer", options),
  subscription_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
  service_group_(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
  client_group_(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  map_ptr_(nullptr),
  map_frame_("map"),
  use_first_gnss_topic_(true)
{
  map_frame_ = this->declare_parameter<std::string>("map_frame", map_frame_);
  use_first_gnss_topic_ = this->declare_parameter<bool>("use_first_gnss_topic", use_first_gnss_topic_);

  rclcpp::SubscriptionOptions sensor_subscription_options;
  sensor_subscription_options.callback_group = subscription_group_;

  initial_pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SensorDataQoS(),
    std::bind(&PoseInitializer::handleInitialPose, this, std::placeholders::_1),
    sensor_subscription_options);

  if (use_first_gnss_topic_) {
    gnss_pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
      "gnss_pose_cov", rclcpp::SensorDataQoS(),
      std::bind(&PoseInitializer::handleGnssPose, this, std::placeholders::_1),
      sensor_subscription_options);
  }

  rclcpp::SubscriptionOptions map_subscription_options;
  map_subscription_options.callback_group = subscription_group_;
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", map_qos,
    std::bind(&PoseInitializer::handleMapPoints, this, std::placeholders::_1),
    map_subscription_options);

  initial_pose_pub_ = this->create_publisher<PoseWithCovarianceStamped>(
    "initialpose3d", rclcpp::QoS(rclcpp::KeepLast(10)));

  ndt_client_ = this->create_client<PoseWithCovarianceSrv>(
    "ndt_align_srv", rmw_qos_profile_services_default, client_group_);

  pose_initializer_service_ = this->create_service<PoseWithCovarianceSrv>(
    "pose_initializer_srv",
    [this](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<PoseWithCovarianceSrv::Request> request,
      std::shared_ptr<PoseWithCovarianceSrv::Response> response) {
        this->handleService(request, response);
      },
    rmw_qos_profile_services_default, service_group_);

  if (!ndt_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "NDT align service 'ndt_align_srv' is not available yet.");
  }
}

void PoseInitializer::handleInitialPose(const PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  shutdownGnssSubscription();

  PoseWithCovarianceStamped height_adjusted_pose;
  getHeight(*msg, height_adjusted_pose);

  auto & cov = height_adjusted_pose.pose.covariance;
  cov[0] = 2.0;
  cov[1 * 6 + 1] = 2.0;
  cov[2 * 6 + 2] = 0.01;
  cov[3 * 6 + 3] = 0.01;
  cov[4 * 6 + 4] = 0.01;
  cov[5 * 6 + 5] = 0.3;

  PoseWithCovarianceStamped aligned_pose;
  if (callAlignService(height_adjusted_pose, aligned_pose)) {
    initial_pose_pub_->publish(aligned_pose);
  }
}

void PoseInitializer::handleGnssPose(const PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  PoseWithCovarianceStamped height_adjusted_pose;
  getHeight(*msg, height_adjusted_pose);
  applyDefaultCovariance(height_adjusted_pose, 3.14);

  PoseWithCovarianceStamped aligned_pose;
  if (callAlignService(height_adjusted_pose, aligned_pose)) {
    initial_pose_pub_->publish(aligned_pose);
    shutdownGnssSubscription();
  }
}

void PoseInitializer::handleMapPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *pointcloud);

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ptr_ = pointcloud;
    map_frame_ = msg->header.frame_id.empty() ? map_frame_ : msg->header.frame_id;
  }
}

void PoseInitializer::handleService(
  const std::shared_ptr<PoseWithCovarianceSrv::Request> request,
  std::shared_ptr<PoseWithCovarianceSrv::Response> response)
{
  if (!request || !response) {
    RCLCPP_WARN(this->get_logger(), "Received invalid request for pose initializer service.");
    return;
  }

  shutdownGnssSubscription();

  PoseWithCovarianceStamped height_adjusted_pose;
  getHeight(request->pose_with_cov, height_adjusted_pose);
  applyDefaultCovariance(height_adjusted_pose, 1.0);

  PoseWithCovarianceStamped aligned_pose;
  const bool succeeded_align = callAlignService(height_adjusted_pose, aligned_pose);

  if (succeeded_align) {
    response->pose_with_cov = aligned_pose;
    initial_pose_pub_->publish(aligned_pose);
    return;
  }

  response->pose_with_cov = height_adjusted_pose;
  RCLCPP_WARN(this->get_logger(), "Failed to call NDT align service from pose initializer service.");
}

bool PoseInitializer::getHeight(
  const PoseWithCovarianceStamped & input_pose_msg, PoseWithCovarianceStamped & output_pose_msg)
{
  output_pose_msg = input_pose_msg;

  const std::string fixed_frame = input_pose_msg.header.frame_id;
  tf2::Vector3 point(
    input_pose_msg.pose.pose.position.x, input_pose_msg.pose.pose.position.y,
    input_pose_msg.pose.pose.position.z);

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr map;
  std::string map_frame_copy;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_ptr_;
    map_frame_copy = map_frame_;
  }

  if (map) {
    try {
      const auto transform_stamped = tf_buffer_.lookupTransform(
        map_frame_copy, fixed_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
      tf2::Transform transform;
      tf2::fromMsg(transform_stamped.transform, transform);

      point = transform * point;
      point.setZ(getGroundHeight(map, point));
      point = transform.inverse() * point;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Failed to lookup transform from '%s' to '%s': %s",
        fixed_frame.c_str(), map_frame_copy.c_str(), ex.what());
    }
  }

  output_pose_msg.pose.pose.position.x = point.getX();
  output_pose_msg.pose.pose.position.y = point.getY();
  output_pose_msg.pose.pose.position.z = point.getZ();
  return true;
}

bool PoseInitializer::callAlignService(
  const PoseWithCovarianceStamped & input_pose_msg, PoseWithCovarianceStamped & output_pose_msg)
{
  if (!ndt_client_) {
    RCLCPP_WARN(this->get_logger(), "NDT align service client is not initialized.");
    return false;
  }

  if (!ndt_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "NDT align service is unavailable.");
    return false;
  }

  auto request = std::make_shared<PoseWithCovarianceSrv::Request>();
  request->pose_with_cov = input_pose_msg;

  RCLCPP_INFO(this->get_logger(), "Calling NDT align service.");
  auto future = ndt_client_->async_send_request(request);
  const auto status = future.wait_for(3s);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Timed out while waiting for NDT align service response.");
    return false;
  }

  auto response = future.get();
  if (!response) {
    RCLCPP_WARN(this->get_logger(), "Received empty response from NDT align service.");
    return false;
  }

  auto & covariance = response->pose_with_cov.pose.covariance;
  covariance[0] = 1.0;
  covariance[1 * 6 + 1] = 1.0;
  covariance[2 * 6 + 2] = 0.01;
  covariance[3 * 6 + 3] = 0.01;
  covariance[4 * 6 + 4] = 0.01;
  covariance[5 * 6 + 5] = 0.2;

  output_pose_msg = response->pose_with_cov;
  RCLCPP_INFO(this->get_logger(), "NDT align service call succeeded.");
  return true;
}

void PoseInitializer::shutdownGnssSubscription()
{
  if (use_first_gnss_topic_ && gnss_pose_sub_) {
    gnss_pose_sub_.reset();
    RCLCPP_DEBUG(this->get_logger(), "Shut down GNSS pose subscription after first message.");
  }
}
}  // namespace pose_initializer

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(pose_initializer::PoseInitializer)
