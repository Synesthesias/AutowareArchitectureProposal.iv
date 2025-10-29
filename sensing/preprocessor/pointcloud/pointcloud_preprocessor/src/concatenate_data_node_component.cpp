#include "pointcloud_preprocessor/concatenate_data_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/subscription_options.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace pointcloud_preprocessor
{
namespace
{
constexpr double kDefaultTimeout = 0.1;             // [s]
constexpr int kDefaultQueueSize = 10;
constexpr double kDefaultMaxTwistDt = 0.1;          // [s]
constexpr bool kDefaultUseTwistCompensation = true;
const char kDefaultTwistTopic[] = "/vehicle/status/twist";
}  // namespace

ConcatenateDataNode::ConcatenateDataNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("concatenate_data", options), timer_active_(false)
{
  clock_ = this->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  input_topics_ = this->declare_parameter<std::vector<std::string>>(
    "input_topics", std::vector<std::string>{});
  output_frame_ = this->declare_parameter<std::string>("output_frame", "");
  queue_size_ = this->declare_parameter<int>("max_queue_size", kDefaultQueueSize);
  timeout_sec_ = this->declare_parameter<double>("timeout_sec", kDefaultTimeout);
  twist_topic_ = this->declare_parameter<std::string>("twist_topic", kDefaultTwistTopic);
  use_twist_compensation_ = this->declare_parameter<bool>(
    "use_twist_compensation", kDefaultUseTwistCompensation);
  max_twist_dt_ = this->declare_parameter<double>("max_twist_dt", kDefaultMaxTwistDt);

  if (input_topics_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No input topics configured. Node will remain idle.");
  }

  auto sensor_qos = rclcpp::SensorDataQoS();
  sensor_qos.keep_last(static_cast<size_t>(std::max(1, queue_size_)));

  output_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", sensor_qos);
  concat_num_pub_ = this->create_publisher<std_msgs::msg::Int32>("concat_num", rclcpp::QoS(10));
  skipped_topic_pub_ = this->create_publisher<std_msgs::msg::String>(
    "not_subscribed_topic_name", rclcpp::QoS(10));

  pointcloud_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  twist_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  pointcloud_subs_.reserve(input_topics_.size());
  auto pointcloud_qos = rclcpp::SensorDataQoS();
  pointcloud_qos.keep_last(static_cast<size_t>(std::max(1, queue_size_)));
  rclcpp::SubscriptionOptions pointcloud_options;
  pointcloud_options.callback_group = pointcloud_callback_group_;
  for (const auto & topic : input_topics_) {
    clouds_[topic] = nullptr;
    pending_clouds_[topic] = nullptr;
    auto callback = [this, topic](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      this->on_pointcloud(msg, topic);
    };
    pointcloud_subs_.emplace_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, pointcloud_qos, callback, pointcloud_options));
  }

  if (use_twist_compensation_) {
    auto twist_qos = rclcpp::QoS(static_cast<size_t>(std::max(1, queue_size_))).best_effort();
    rclcpp::SubscriptionOptions twist_options;
    twist_options.callback_group = twist_callback_group_;
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, twist_qos, std::bind(&ConcatenateDataNode::on_twist, this, std::placeholders::_1),
      twist_options);
  }

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timeout_sec_),
    std::bind(&ConcatenateDataNode::on_timeout, this),
    timer_callback_group_);
  timer_->cancel();
}

void ConcatenateDataNode::on_pointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto transformed = transform_cloud(*msg);
  if (!transformed) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "Failed to transform point cloud from topic %s", topic_name.c_str());
    return;
  }

  auto & current_slot = clouds_[topic_name];
  if (current_slot) {
    pending_clouds_[topic_name] = transformed;
    if (!timer_active_) {
      timer_->reset();
      timer_active_ = true;
    }
    return;
  }

  current_slot = transformed;

  const bool all_ready = std::all_of(
    input_topics_.begin(), input_topics_.end(),
    [this](const std::string & topic) { return clouds_.at(topic) != nullptr; });

  if (all_ready) {
    timer_->cancel();
    timer_active_ = false;
    publish();
    reset_buffers();
  } else if (!timer_active_) {
    timer_->reset();
    timer_active_ = true;
  }
}

void ConcatenateDataNode::on_twist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  while (!twist_buffer_.empty()) {
    const auto oldest_stamp = rclcpp::Time(twist_buffer_.front()->header.stamp);
    if ((rclcpp::Time(msg->header.stamp) - oldest_stamp).seconds() <= 1.0) {
      break;
    }
    twist_buffer_.pop_front();
  }
  twist_buffer_.push_back(msg);
}

void ConcatenateDataNode::on_timeout()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publish();
  reset_buffers();
  timer_->cancel();
  timer_active_ = false;
}

sensor_msgs::msg::PointCloud2::SharedPtr ConcatenateDataNode::transform_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  sensor_msgs::msg::PointCloud2 transformed = cloud;
  if (!output_frame_.empty() && cloud.header.frame_id != output_frame_) {
    try {
      const auto timeout = tf2::durationFromSec(timeout_sec_);
      const auto transform = tf_buffer_->lookupTransform(
        output_frame_, cloud.header.frame_id, tf2::TimePointZero, timeout);
      tf2::doTransform(cloud, transformed, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 2000,
        "Transform (%s -> %s) failed: %s", cloud.header.frame_id.c_str(), output_frame_.c_str(),
        ex.what());
      return nullptr;
    }
    transformed.header.frame_id = output_frame_;
  }

  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  pcl::fromROSMsg(transformed, xyz_cloud);

  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(xyz_cloud, *output);
  output->header = transformed.header;
  return output;
}

sensor_msgs::msg::PointCloud2::SharedPtr ConcatenateDataNode::merge_clouds(
  const sensor_msgs::msg::PointCloud2 & base, const sensor_msgs::msg::PointCloud2 & addition) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr base_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr addition_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(base, *base_xyz);
  pcl::fromROSMsg(addition, *addition_xyz);

  if (use_twist_compensation_ && !twist_buffer_.empty()) {
    const rclcpp::Time stamp_base(base.header.stamp);
    const rclcpp::Time stamp_add(addition.header.stamp);
    const auto old_stamp = std::min(stamp_base, stamp_add);
    const auto new_stamp = std::max(stamp_base, stamp_add);
    const double total_dt = (new_stamp - old_stamp).seconds();

    if (total_dt > std::numeric_limits<double>::epsilon()) {
      const auto lower_it = std::lower_bound(
        twist_buffer_.begin(), twist_buffer_.end(), old_stamp,
        [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr & twist, const rclcpp::Time & stamp) {
          return rclcpp::Time(twist->header.stamp) < stamp;
        });
      auto upper_it = std::lower_bound(
        twist_buffer_.begin(), twist_buffer_.end(), new_stamp,
        [](const geometry_msgs::msg::TwistStamped::ConstSharedPtr & twist, const rclcpp::Time & stamp) {
          return rclcpp::Time(twist->header.stamp) < stamp;
        });
      if (upper_it == twist_buffer_.end() && !twist_buffer_.empty()) {
        upper_it = twist_buffer_.end() - 1;
      }

      double yaw = 0.0;
      Eigen::Vector2d translation = Eigen::Vector2d::Zero();
      rclcpp::Time prev_time = old_stamp;

      for (auto it = lower_it; it != twist_buffer_.end() && it != upper_it + 1; ++it) {
        const rclcpp::Time current_time((*it)->header.stamp);
        const double dt = (current_time - prev_time).seconds();
        if (dt > max_twist_dt_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *clock_, 5000,
            "Twist delta time %.3f is larger than threshold %.3f. Skipping twist compensation.",
            dt, max_twist_dt_);
          translation = Eigen::Vector2d::Zero();
          yaw = 0.0;
          break;
        }
        const double linear = (*it)->twist.linear.x;
        const double angular = (*it)->twist.angular.z;
        const double distance = linear * dt;
        yaw += angular * dt;
        translation.x() += distance * std::cos(yaw);
        translation.y() += distance * std::sin(yaw);
        prev_time = current_time;
      }

      const bool base_is_newer = stamp_base > stamp_add;
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.translation() << translation.x(), translation.y(), 0.0;
      transform.rotate(Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()));

      if (base_is_newer) {
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(*base_xyz, transformed_cloud, transform.matrix());
        *base_xyz = transformed_cloud;
      } else {
        Eigen::Affine3f inverse_transform = transform.inverse();
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud(*addition_xyz, transformed_cloud, inverse_transform.matrix());
        *addition_xyz = transformed_cloud;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> merged = *base_xyz;
  merged += *addition_xyz;

  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(merged, *output);
  output->header = base.header;
  const rclcpp::Time base_time(base.header.stamp);
  const rclcpp::Time addition_time(addition.header.stamp);
  output->header.stamp = addition_time < base_time ? addition.header.stamp : base.header.stamp;
  return output;
}

void ConcatenateDataNode::publish()
{
  sensor_msgs::msg::PointCloud2::SharedPtr concatenated;
  std::string missing_topics;
  size_t concat_count = 0U;

  for (const auto & topic : input_topics_) {
    const auto & cloud = clouds_.at(topic);
    if (!cloud) {
      if (!missing_topics.empty()) {
        missing_topics += ",";
      }
      missing_topics += topic;
      continue;
    }

    if (!concatenated) {
      concatenated = cloud;
    } else {
      concatenated = merge_clouds(*concatenated, *cloud);
    }
    ++concat_count;
  }

  if (concatenated) {
    output_pub_->publish(*concatenated);
  }

  std_msgs::msg::Int32 count_msg;
  count_msg.data = static_cast<int>(concat_count);
  concat_num_pub_->publish(count_msg);

  std_msgs::msg::String skipped_msg;
  skipped_msg.data = missing_topics;
  skipped_topic_pub_->publish(skipped_msg);
}

void ConcatenateDataNode::reset_buffers()
{
  for (const auto & topic : input_topics_) {
    clouds_[topic] = pending_clouds_[topic];
    pending_clouds_[topic] = nullptr;
  }
}

}  // namespace pointcloud_preprocessor

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::ConcatenateDataNode)
