#include "pointcloud_preprocessor/distance_based_compare_map_filter_node.hpp"

#include <algorithm>
#include <functional>
#include <utility>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/subscription_options.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace pointcloud_preprocessor
{
namespace
{
constexpr double kDefaultDistanceThreshold = 1.0;       // [m]
constexpr double kDefaultTfTimeout = 0.2;               // [s]
}  // namespace

DistanceBasedCompareMapFilterNode::DistanceBasedCompareMapFilterNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("distance_based_compare_map_filter", options),
  map_cloud_xyz_(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
  map_tree_(std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false))
{
  clock_ = this->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  distance_threshold_ = this->declare_parameter("distance_threshold", kDefaultDistanceThreshold);
  distance_threshold_sq_ = distance_threshold_ * distance_threshold_;
  target_frame_ = this->declare_parameter<std::string>("target_frame", "");
  tf_timeout_sec_ = this->declare_parameter("tf_timeout_sec", kDefaultTfTimeout);

  sensor_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sensor_qos = rclcpp::SensorDataQoS();
  rclcpp::SubscriptionOptions input_options;
  input_options.callback_group = sensor_callback_group_;
  input_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", sensor_qos,
    std::bind(&DistanceBasedCompareMapFilterNode::on_input, this, std::placeholders::_1),
    input_options);

  auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  rclcpp::SubscriptionOptions map_options;
  map_options.callback_group = map_callback_group_;
  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", map_qos,
    std::bind(&DistanceBasedCompareMapFilterNode::on_map, this, std::placeholders::_1),
    map_options);

  output_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", sensor_qos);

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DistanceBasedCompareMapFilterNode::on_parameter_event, this, std::placeholders::_1));
}

void DistanceBasedCompareMapFilterNode::on_input(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!map_cloud_xyz_ || map_cloud_xyz_->empty() || !map_tree_ || map_frame_.empty()) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "No target map available yet. Republishing raw input cloud.");
    auto passthrough = *msg;
    if (!target_frame_.empty() && target_frame_ != passthrough.header.frame_id) {
      sensor_msgs::msg::PointCloud2 transformed;
      if (transform_cloud_to_frame(*msg, transformed, target_frame_)) {
        passthrough = transformed;
      }
    }
    passthrough.header.stamp = msg->header.stamp;
    output_pub_->publish(passthrough);
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_in_map = *msg;
  if (!transform_cloud_to_frame(*msg, cloud_in_map, map_frame_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "Failed to transform input cloud from %s to %s", msg->header.frame_id.c_str(),
      map_frame_.c_str());
    return;
  }

  pcl::PCLPointCloud2 pcl_input_blob;
  pcl_conversions::toPCL(cloud_in_map, pcl_input_blob);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(pcl_input_blob, *input_xyz);

  pcl::PointIndices::Ptr keep_indices(new pcl::PointIndices());
  keep_indices->indices.reserve(input_xyz->size());

  std::vector<int> nn_indices(1);
  std::vector<float> nn_distances(1);
  for (size_t idx = 0; idx < input_xyz->size(); ++idx) {
    if (map_tree_->nearestKSearch((*input_xyz)[idx], 1, nn_indices, nn_distances) > 0) {
      if (nn_distances[0] <= distance_threshold_sq_) {
        keep_indices->indices.push_back(static_cast<int>(idx));
      }
    }
  }

  pcl::PCLPointCloud2 pcl_filtered_blob;
  if (!keep_indices->indices.empty()) {
  pcl::ExtractIndices<pcl::PCLPointCloud2> extractor;
  auto pcl_input_blob_ptr = pcl::make_shared<const pcl::PCLPointCloud2>(pcl_input_blob);
  extractor.setInputCloud(pcl_input_blob_ptr);
    extractor.setIndices(keep_indices);
    extractor.filter(pcl_filtered_blob);
  } else {
    pcl_filtered_blob.header = pcl_input_blob.header;
    pcl_filtered_blob.fields = pcl_input_blob.fields;
    pcl_filtered_blob.point_step = pcl_input_blob.point_step;
    pcl_filtered_blob.row_step = 0U;
    pcl_filtered_blob.height = 1U;
    pcl_filtered_blob.width = 0U;
    pcl_filtered_blob.is_dense = pcl_input_blob.is_dense;
    pcl_filtered_blob.data.clear();
  }

  sensor_msgs::msg::PointCloud2 filtered_ros;
  pcl_conversions::fromPCL(pcl_filtered_blob, filtered_ros);
  filtered_ros.header = cloud_in_map.header;
  filtered_ros.header.stamp = msg->header.stamp;

  if (target_frame_.empty() && filtered_ros.header.frame_id != msg->header.frame_id) {
    sensor_msgs::msg::PointCloud2 original_frame_cloud;
    if (transform_cloud_to_frame(filtered_ros, original_frame_cloud, msg->header.frame_id)) {
      filtered_ros = original_frame_cloud;
      filtered_ros.header.stamp = msg->header.stamp;
    }
  }

  output_pub_->publish(filtered_ros);
}

void DistanceBasedCompareMapFilterNode::on_map(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud = *msg;
  if (!target_frame_.empty() && cloud.header.frame_id != target_frame_) {
    if (!transform_cloud_to_frame(*msg, cloud, target_frame_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to transform target map from %s to %s. Keeping previous map.",
        msg->header.frame_id.c_str(), target_frame_.c_str());
      return;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud, *map_xyz);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_frame_ = cloud.header.frame_id;
    map_cloud_xyz_ = map_xyz;
    rebuild_search_structure(map_cloud_xyz_);
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *clock_, 5000,
    "Updated map cloud with %zu points in frame %s", map_cloud_xyz_->size(),
    map_frame_.c_str());
}

rcl_interfaces::msg::SetParametersResult DistanceBasedCompareMapFilterNode::on_parameter_event(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "distance_threshold") {
      const double new_threshold = parameter.as_double();
      distance_threshold_ = new_threshold;
      distance_threshold_sq_ = distance_threshold_ * distance_threshold_;
      RCLCPP_INFO(this->get_logger(), "Distance threshold set to %.3f m", distance_threshold_);
    } else if (parameter.get_name() == "target_frame") {
      target_frame_ = parameter.as_string();
      RCLCPP_INFO(
        this->get_logger(), "Target frame set to '%s'", target_frame_.c_str());
    } else if (parameter.get_name() == "tf_timeout_sec") {
      tf_timeout_sec_ = parameter.as_double();
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

bool DistanceBasedCompareMapFilterNode::transform_cloud_to_frame(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output,
  const std::string & target_frame) const
{
  if (target_frame.empty() || input.header.frame_id == target_frame) {
    output = input;
    output.header.frame_id = target_frame.empty() ? input.header.frame_id : target_frame;
    return true;
  }

  return try_transform_cloud(input, output, target_frame);
}

bool DistanceBasedCompareMapFilterNode::try_transform_cloud(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output,
  const std::string & target_frame) const
{
  try {
    const auto timeout = tf2::durationFromSec(tf_timeout_sec_);
    const auto transform = tf_buffer_->lookupTransform(
      target_frame, input.header.frame_id, tf2::TimePointZero, timeout);
    tf2::doTransform(input, output, transform);
    output.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 2000,
      "Transform lookup failed (%s -> %s): %s", input.header.frame_id.c_str(),
      target_frame.c_str(), ex.what());
    return false;
  }
}

void DistanceBasedCompareMapFilterNode::rebuild_search_structure(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud_xyz)
{
  if (!map_tree_) {
    map_tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false);
  }
  map_tree_->setInputCloud(cloud_xyz);
}

}  // namespace pointcloud_preprocessor

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::DistanceBasedCompareMapFilterNode)
