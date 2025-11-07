#include "ndt_scan_matcher/ndt_scan_matcher_core.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "ndt_scan_matcher/util_func.h"

using namespace std::chrono_literals;
using PoseWithCovarianceSrv = autoware_localization_srvs::srv::PoseWithCovarianceStamped;
using PoseArraySrv = autoware_localization_srvs::srv::PoseArray;

NDTScanMatcherComponent::NDTScanMatcherComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("ndt_scan_matcher", options), ndt_implement_type_(NDTImplementType::PCL_GENERIC)
{
  key_value_stdmap_["state"] = "Initializing";

  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_buffer_->setUsingDedicatedThread(true);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  diagnostic_timer_ = this->create_wall_timer(
    100ms, std::bind(&NDTScanMatcherComponent::onDiagnosticTimer, this));

  const int ndt_impl_param = this->declare_parameter<int>(
    "ndt_implement_type", static_cast<int>(NDTImplementType::OMP));
  ndt_implement_type_ = static_cast<NDTImplementType>(ndt_impl_param);

  omp_params_.search_method = static_cast<ndt_omp::NeighborSearchMethod>(
    this->declare_parameter<int>(
      "omp_neighborhood_search_method", static_cast<int>(omp_params_.search_method)));
  const int omp_num_threads_param =
    this->declare_parameter<int>("omp_num_threads", omp_params_.num_threads);
  omp_params_.num_threads = std::max(omp_num_threads_param, 1);

  const int input_queue_size_param =
    this->declare_parameter<int>("input_sensor_points_queue_size", 1);
  points_queue_size_ = static_cast<size_t>(std::max(input_queue_size_param, 1));
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  ndt_base_frame_ = this->declare_parameter<std::string>("ndt_base_frame", "ndt_base_link");
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  converged_param_transform_probability_ = this->declare_parameter<double>(
    "converged_param_transform_probability", 3.0);

  const double trans_epsilon = this->declare_parameter<double>("trans_epsilon", 0.01);
  const double step_size = this->declare_parameter<double>("step_size", 0.1);
  const double resolution = this->declare_parameter<double>("resolution", 2.0);
  const int max_iterations = this->declare_parameter<int>("max_iterations", 30);

  ndt_ptr_ = createNDTInstance();
  configureNDT(ndt_ptr_);
  ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setResolution(resolution);
  ndt_ptr_->setMaximumIterations(max_iterations);

  using std::placeholders::_1;
  using std::placeholders::_2;

  initial_pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", rclcpp::QoS(100),
    std::bind(&NDTScanMatcherComponent::callbackInitialPose, this, _1));

  map_points_sub_ = this->create_subscription<PointCloud2>(
    "pointcloud_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&NDTScanMatcherComponent::callbackMapPoints, this, _1));

  sensor_points_sub_ = this->create_subscription<PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS(),
    std::bind(&NDTScanMatcherComponent::callbackSensorPoints, this, _1));

  sensor_aligned_points_pub_ = this->create_publisher<PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = this->create_publisher<PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<PoseWithCovarianceStamped>("ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<PoseWithCovarianceStamped>("initial_pose_with_covariance", 10);
  exe_time_pub_ = this->create_publisher<Float32>("exe_time_ms", 10);
  transform_probability_pub_ = this->create_publisher<Float32>("transform_probability", 10);
  iteration_num_pub_ = this->create_publisher<Float32>("iteration_num", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<Float32>("initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<Float32>("initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<Float32>("initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<MarkerArray>("monte_carlo_initial_pose_marker", 10);
  diagnostics_pub_ = this->create_publisher<DiagnosticArray>("/diagnostics", 10);

  service_ = this->create_service<PoseWithCovarianceSrv>(
    "ndt_align_srv", std::bind(&NDTScanMatcherComponent::handleNDTAlign, this, _1, _2));
  service_ndt_align_pose_array_ = this->create_service<PoseArraySrv>(
    "ndt_align_pose_array_srv",
    std::bind(&NDTScanMatcherComponent::handleNDTAlignPoseArray, this, _1, _2));
}

void NDTScanMatcherComponent::handleNDTAlign(
  const std::shared_ptr<PoseWithCovarianceSrv::Request> req,
  std::shared_ptr<PoseWithCovarianceSrv::Response> res)
{
  geometry_msgs::msg::TransformStamped tf_pose_to_map;
  getTransform(map_frame_, req->pose_with_cov.header.frame_id, tf_pose_to_map);

  auto map_initial_pose = std::make_shared<PoseWithCovarianceStamped>();
  tf2::doTransform(req->pose_with_cov, *map_initial_pose, tf_pose_to_map);

  if (ndt_ptr_->getInputTarget() == nullptr || ndt_ptr_->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(),
      "Align service requested but map or sensor pointcloud is not available yet.");
    res->pose_with_cov = PoseWithCovarianceStamped();
    return;
  }

  std::lock_guard<std::mutex> lock(ndt_map_mtx_);
  key_value_stdmap_["state"] = "Aligning";
  res->pose_with_cov = alignUsingMonteCarlo(ndt_ptr_, *map_initial_pose);
  key_value_stdmap_["state"] = "Sleeping";
  res->pose_with_cov.pose.covariance = req->pose_with_cov.pose.covariance;
}

void NDTScanMatcherComponent::handleNDTAlignPoseArray(
  const std::shared_ptr<PoseArraySrv::Request> req,
  std::shared_ptr<PoseArraySrv::Response> res)
{
  if (req->pose_array.header.frame_id != map_frame_) {
    RCLCPP_ERROR(
      get_logger(), "Pose array frame_id '%s' is invalid. Transform to '%s' frame in advance.",
      req->pose_array.header.frame_id.c_str(), map_frame_.c_str());
    res->pose = geometry_msgs::msg::PoseStamped();
    res->pose.header = req->pose_array.header;
    return;
  }

  if (ndt_ptr_->getInputTarget() == nullptr) {
    RCLCPP_WARN(get_logger(), "Align pose array requested but no map pointcloud is set.");
    res->pose = geometry_msgs::msg::PoseStamped();
    res->pose.header = req->pose_array.header;
    return;
  }

  if (ndt_ptr_->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(), "Align pose array requested but no sensor pointcloud is set.");
    res->pose = geometry_msgs::msg::PoseStamped();
    res->pose.header = req->pose_array.header;
    return;
  }

  std::lock_guard<std::mutex> lock(ndt_map_mtx_);
  key_value_stdmap_["state"] = "Aligning";
  const auto aligned_pose = alignUsingMonteCarlo(ndt_ptr_, req->pose_array);
  key_value_stdmap_["state"] = "Sleeping";
  res->pose.header = aligned_pose.header;
  res->pose.pose = aligned_pose.pose.pose;
}

void NDTScanMatcherComponent::callbackInitialPose(
  const PoseWithCovarianceStamped::ConstSharedPtr pose_msg)
{
  if (!initial_pose_msg_ptr_array_.empty()) {
    if (rclcpp::Time(initial_pose_msg_ptr_array_.front()->header.stamp) >
      rclcpp::Time(pose_msg->header.stamp))
    {
      initial_pose_msg_ptr_array_.clear();
    }
  }

  PoseWithCovarianceStamped::ConstSharedPtr pose_in_map;

  if (pose_msg->header.frame_id == map_frame_) {
    pose_in_map = pose_msg;
  } else {
    geometry_msgs::msg::TransformStamped tf_pose_to_map;
    if (!getTransform(map_frame_, pose_msg->header.frame_id, tf_pose_to_map,
        rclcpp::Time(pose_msg->header.stamp)))
    {
      RCLCPP_WARN(
        get_logger(), "Failed to transform initial pose from %s to %s",
        pose_msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    auto transformed_pose = std::make_shared<PoseWithCovarianceStamped>();
    tf2::doTransform(*pose_msg, *transformed_pose, tf_pose_to_map);
    transformed_pose->header.stamp = pose_msg->header.stamp;
    pose_in_map = transformed_pose;
  }

  initial_pose_msg_ptr_array_.push_back(pose_in_map);
  if (points_queue_size_ > 0) {
    while (initial_pose_msg_ptr_array_.size() > points_queue_size_) {
      initial_pose_msg_ptr_array_.pop_front();
    }
  }
}

void NDTScanMatcherComponent::callbackMapPoints(const PointCloud2::ConstSharedPtr map_points_msg)
{
  const auto trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  const auto step_size = ndt_ptr_->getStepSize();
  const auto resolution = ndt_ptr_->getResolution();
  const auto max_iterations = ndt_ptr_->getMaximumIterations();

  auto new_ndt_ptr = createNDTInstance();
  configureNDT(new_ndt_ptr);
  new_ndt_ptr->setTransformationEpsilon(trans_epsilon);
  new_ndt_ptr->setStepSize(step_size);
  new_ndt_ptr->setResolution(resolution);
  new_ndt_ptr->setMaximumIterations(max_iterations);

  auto map_points = pcl::make_shared<pcl::PointCloud<PointTarget>>();
  pcl::fromROSMsg(*map_points_msg, *map_points);
  new_ndt_ptr->setInputTarget(map_points);

  auto output_cloud = pcl::make_shared<pcl::PointCloud<PointSource>>();
  new_ndt_ptr->align(*output_cloud, Eigen::Matrix4f::Identity());

  std::lock_guard<std::mutex> lock(ndt_map_mtx_);
  ndt_ptr_ = new_ndt_ptr;
  key_value_stdmap_["state"] = "MapReady";
}

void NDTScanMatcherComponent::callbackSensorPoints(
  const PointCloud2::ConstSharedPtr sensor_points_msg)
{
  const auto exe_start_time = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No map pointcloud set.");
    return;
  }

  if (initial_pose_msg_ptr_array_.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "No initial pose available.");
    return;
  }

  const std::string sensor_frame = sensor_points_msg->header.frame_id;
  const rclcpp::Time sensor_time(sensor_points_msg->header.stamp);

  geometry_msgs::msg::TransformStamped tf_base_to_sensor;
  if (!getTransform(base_frame_, sensor_frame, tf_base_to_sensor, sensor_time)) {
    RCLCPP_WARN(
      get_logger(), "Failed to get transform from %s to %s", sensor_frame.c_str(),
      base_frame_.c_str());
    return;
  }

  auto sensor_points_sensor_tf = pcl::make_shared<pcl::PointCloud<PointSource>>();
  pcl::fromROSMsg(*sensor_points_msg, *sensor_points_sensor_tf);

  const Eigen::Matrix4f base_to_sensor_matrix =
    tf2::transformToEigen(tf_base_to_sensor).matrix().cast<float>();
  auto sensor_points_base_tf = pcl::make_shared<pcl::PointCloud<PointSource>>();
  pcl::transformPointCloud(*sensor_points_sensor_tf, *sensor_points_base_tf, base_to_sensor_matrix);
  ndt_ptr_->setInputSource(sensor_points_base_tf);

  auto initial_pose_old = PoseWithCovarianceStamped::ConstSharedPtr(
    std::make_shared<PoseWithCovarianceStamped>());
  auto initial_pose_new = PoseWithCovarianceStamped::ConstSharedPtr(
    std::make_shared<PoseWithCovarianceStamped>());

  getNearestTimeStampPose(initial_pose_msg_ptr_array_, sensor_time, initial_pose_old, initial_pose_new);
  popOldPose(initial_pose_msg_ptr_array_, sensor_time);

  const auto interpolated_pose = interpolatePose(*initial_pose_old, *initial_pose_new, sensor_time);

  PoseWithCovarianceStamped initial_pose_cov;
  initial_pose_cov.header = interpolated_pose.header;
  initial_pose_cov.pose.pose = interpolated_pose.pose;

  Eigen::Affine3d initial_pose_affine;
  tf2::fromMsg(initial_pose_cov.pose.pose, initial_pose_affine);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

  auto output_cloud = pcl::make_shared<pcl::PointCloud<PointSource>>();
  const auto align_start_time = std::chrono::steady_clock::now();
  key_value_stdmap_["state"] = "Aligning";
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";
  const auto align_end_time = std::chrono::steady_clock::now();
  const double align_time_ms =
    std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /
    1000.0;

  const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::msg::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const auto transform_probability = ndt_ptr_->getTransformationProbability();
  const size_t iteration_num = ndt_ptr_->getFinalNumIteration();

  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if (
    iteration_num >= static_cast<size_t>(ndt_ptr_->getMaximumIterations()) + 2 ||
    transform_probability < converged_param_transform_probability_)
  {
    is_converged = false;
    ++skipping_publish_num;
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "NDT did not converge.");
  } else {
    skipping_publish_num = 0;
  }

  PoseStamped result_pose_stamped;
  result_pose_stamped.header = sensor_points_msg->header;
  result_pose_stamped.header.frame_id = map_frame_;
  result_pose_stamped.pose = result_pose_msg;

  PoseWithCovarianceStamped result_pose_with_cov;
  result_pose_with_cov.header = sensor_points_msg->header;
  result_pose_with_cov.header.frame_id = map_frame_;
  result_pose_with_cov.pose.pose = result_pose_msg;
  result_pose_with_cov.pose.covariance[0] = 0.025;
  result_pose_with_cov.pose.covariance[1 * 6 + 1] = 0.025;
  result_pose_with_cov.pose.covariance[2 * 6 + 2] = 0.025;
  result_pose_with_cov.pose.covariance[3 * 6 + 3] = 0.000625;
  result_pose_with_cov.pose.covariance[4 * 6 + 4] = 0.000625;
  result_pose_with_cov.pose.covariance[5 * 6 + 5] = 0.000625;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov);
  }

  publishTF(map_frame_, ndt_base_frame_, result_pose_stamped);

  auto sensor_points_map_tf = pcl::make_shared<pcl::PointCloud<PointSource>>();
  pcl::transformPointCloud(*sensor_points_base_tf, *sensor_points_map_tf, result_pose_matrix);
  sensor_msgs::msg::PointCloud2 sensor_points_map_msg;
  pcl::toROSMsg(*sensor_points_map_tf, sensor_points_map_msg);
  sensor_points_map_msg.header = sensor_points_msg->header;
  sensor_points_map_msg.header.frame_id = map_frame_;
  sensor_aligned_points_pub_->publish(sensor_points_map_msg);

  initial_pose_with_covariance_pub_->publish(initial_pose_cov);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header = sensor_points_msg->header;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.3F;
  marker.scale.y = 0.1F;
  marker.scale.z = 0.1F;

  const auto result_pose_matrix_array = ndt_ptr_->getFinalTransformationArray();
  int marker_id = 0;
  marker.ns = "result_pose_matrix_array";
  for (const auto & pose_matrix : result_pose_matrix_array) {
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose_matrix.cast<double>();
    marker.id = marker_id++;
    marker.pose = tf2::toMsg(pose_affine);
    marker.color = ExchangeColorCrc((1.0 * marker_id) / 15.0);
    marker_array.markers.push_back(marker);
  }
  for (; marker_id < ndt_ptr_->getMaximumIterations() + 2; ++marker_id) {
    marker.id = marker_id;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = ExchangeColorCrc(0.0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_->publish(marker_array);

  const auto exe_end_time = std::chrono::steady_clock::now();
  const double exe_time_ms =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
    1000.0;

  Float32 exe_time_msg;
  exe_time_msg.data = static_cast<float>(exe_time_ms);
  exe_time_pub_->publish(exe_time_msg);

  Float32 transform_probability_msg;
  transform_probability_msg.data = static_cast<float>(transform_probability);
  transform_probability_pub_->publish(transform_probability_msg);

  Float32 iteration_num_msg;
  iteration_num_msg.data = static_cast<float>(iteration_num);
  iteration_num_pub_->publish(iteration_num_msg);

  const auto compute_distance = [](const PoseWithCovarianceStamped & lhs,
                                   const PoseWithCovarianceStamped & rhs) {
    return static_cast<float>(std::sqrt(
      std::pow(lhs.pose.pose.position.x - rhs.pose.pose.position.x, 2.0) +
      std::pow(lhs.pose.pose.position.y - rhs.pose.pose.position.y, 2.0) +
      std::pow(lhs.pose.pose.position.z - rhs.pose.pose.position.z, 2.0)));
  };

  Float32 initial_to_result_distance_msg;
  initial_to_result_distance_msg.data = compute_distance(initial_pose_cov, result_pose_with_cov);
  initial_to_result_distance_pub_->publish(initial_to_result_distance_msg);

  Float32 initial_to_result_distance_old_msg;
  initial_to_result_distance_old_msg.data = compute_distance(*initial_pose_old, result_pose_with_cov);
  initial_to_result_distance_old_pub_->publish(initial_to_result_distance_old_msg);

  Float32 initial_to_result_distance_new_msg;
  initial_to_result_distance_new_msg.data = compute_distance(*initial_pose_new, result_pose_with_cov);
  initial_to_result_distance_new_pub_->publish(initial_to_result_distance_new_msg);

  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);
  key_value_stdmap_["timestamp_ns"] = std::to_string(sensor_time.nanoseconds());
  key_value_stdmap_["last_align_time_ms"] = std::to_string(align_time_ms);
}

NDTScanMatcherComponent::PoseWithCovarianceStamped NDTScanMatcherComponent::alignUsingMonteCarlo(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
  const PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(), "Monte Carlo alignment requested without map or sensor data.");
    return PoseWithCovarianceStamped();
  }

  const auto initial_pose_array = createRandomPoseArray(initial_pose_with_cov, 100);
  return alignUsingMonteCarlo(ndt_ptr, initial_pose_array);
}

NDTScanMatcherComponent::PoseWithCovarianceStamped NDTScanMatcherComponent::alignUsingMonteCarlo(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
  const PoseArray & initial_pose_array)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(), "Monte Carlo alignment requested without map or sensor data.");
    return PoseWithCovarianceStamped();
  }

  std::vector<Particle> particle_array;
  particle_array.reserve(initial_pose_array.poses.size());
  auto output_cloud = pcl::make_shared<pcl::PointCloud<PointSource>>();

  size_t index = 0;
  for (const auto & initial_pose : initial_pose_array.poses) {
    Eigen::Affine3d initial_pose_affine;
    tf2::fromMsg(initial_pose, initial_pose_affine);
    const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

    ndt_ptr->align(*output_cloud, initial_pose_matrix);

    const Eigen::Matrix4f result_pose_matrix = ndt_ptr->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const auto result_pose = tf2::toMsg(result_pose_affine);

    const auto transform_probability = ndt_ptr->getTransformationProbability();
    const auto num_iteration = ndt_ptr->getFinalNumIteration();

    Particle particle(initial_pose, result_pose, transform_probability, num_iteration);
    particle_array.push_back(particle);
    publishMarkerForDebug(particle, index++);

    auto sensor_points_map_tf = pcl::make_shared<pcl::PointCloud<PointSource>>();
    const auto sensor_points_base_tf = ndt_ptr->getInputSource();
    pcl::transformPointCloud(*sensor_points_base_tf, *sensor_points_map_tf, result_pose_matrix);
    sensor_msgs::msg::PointCloud2 sensor_points_map_msg;
    pcl::toROSMsg(*sensor_points_map_tf, sensor_points_map_msg);
    sensor_points_map_msg.header = initial_pose_array.header;
    sensor_points_map_msg.header.frame_id = map_frame_;
    sensor_aligned_points_pub_->publish(sensor_points_map_msg);
  }

  auto best_particle_iter = std::max_element(
    particle_array.cbegin(), particle_array.cend(),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  PoseWithCovarianceStamped result;
  result.header.frame_id = map_frame_;
  if (best_particle_iter != particle_array.cend()) {
    result.pose.pose = best_particle_iter->result_pose;
  }
  return result;
}

std::shared_ptr<NormalDistributionsTransformBase<NDTScanMatcherComponent::PointSource,
NDTScanMatcherComponent::PointTarget>> NDTScanMatcherComponent::createNDTInstance()
{
  if (ndt_implement_type_ == NDTImplementType::PCL_MODIFIED) {
    RCLCPP_INFO_ONCE(get_logger(), "NDT Implement Type is PCL MODIFIED");
    return std::make_shared<NormalDistributionsTransformPCLModified<PointSource, PointTarget>>();
  }
  if (ndt_implement_type_ == NDTImplementType::OMP) {
    RCLCPP_INFO_ONCE(get_logger(), "NDT Implement Type is OMP");
    auto ndt_omp_ptr =
      std::make_shared<NormalDistributionsTransformOMP<PointSource, PointTarget>>();
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
    return ndt_omp_ptr;
  }
  RCLCPP_INFO_ONCE(get_logger(), "NDT Implement Type is PCL GENERIC");
  return std::make_shared<NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>>();
}

void NDTScanMatcherComponent::configureNDT(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr)
{
  if (auto ndt_omp_ptr =
        std::dynamic_pointer_cast<NormalDistributionsTransformOMP<PointSource, PointTarget>>(ndt_ptr))
  {
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
  }
}

void NDTScanMatcherComponent::updateTransforms() {}

void NDTScanMatcherComponent::publishTF(
  const std::string & frame_id, const std::string & child_frame_id, const PoseStamped & pose_msg)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = child_frame_id;
  transform.header.stamp = pose_msg.header.stamp;
  transform.transform.translation.x = pose_msg.pose.position.x;
  transform.transform.translation.y = pose_msg.pose.position.y;
  transform.transform.translation.z = pose_msg.pose.position.z;
  transform.transform.rotation = pose_msg.pose.orientation;

  tf2_broadcaster_->sendTransform(transform);
}

bool NDTScanMatcherComponent::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform_stamped, const rclcpp::Time & time_stamp)
{
  if (target_frame == source_frame) {
    transform_stamped.header.stamp = time_stamp;
    transform_stamped.header.frame_id = target_frame;
    transform_stamped.child_frame_id = source_frame;
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    return true;
  }

  try {
    transform_stamped = tf2_buffer_->lookupTransform(
      target_frame, source_frame, time_stamp, rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());
    transform_stamped.header.stamp = time_stamp;
    transform_stamped.header.frame_id = target_frame;
    transform_stamped.child_frame_id = source_frame;
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NDTScanMatcherComponent::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform_stamped)
{
  return getTransform(target_frame, source_frame, transform_stamped, this->now());
}

void NDTScanMatcherComponent::publishMarkerForDebug(const Particle & particle, const size_t index)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.3F;
  marker.scale.y = 0.1F;
  marker.scale.z = 0.1F;
  marker.id = static_cast<int>(index);

  marker.ns = "initial_pose_transform_probability_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_iteration_color_marker";
  marker.color = ExchangeColorCrc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_index_color_marker";
  marker.color = ExchangeColorCrc((1.0 * index) / 100.0);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_transform_probability_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_iteration_color_marker";
  marker.color = ExchangeColorCrc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_index_color_marker";
  marker.color = ExchangeColorCrc((1.0 * index) / 100.0);
  marker_array.markers.push_back(marker);

  ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
}

void NDTScanMatcherComponent::onDiagnosticTimer()
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "ndt_scan_matcher";
  status.hardware_id = "";

  for (const auto & key_value : key_value_stdmap_) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key_value.first;
    kv.value = key_value.second;
    status.values.push_back(kv);
  }

  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message;

  auto it_state = key_value_stdmap_.find("state");
  if (it_state != key_value_stdmap_.end() && it_state->second == "Initializing") {
  level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::WARN));
    message += "Initializing State. ";
  }

  int skipping_publish_num = 0;
  auto it_skip = key_value_stdmap_.find("skipping_publish_num");
  if (it_skip != key_value_stdmap_.end()) {
    try {
      skipping_publish_num = std::stoi(it_skip->second);
    } catch (...) {
      skipping_publish_num = 0;
    }
  }

  if (skipping_publish_num > 1) {
  level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::WARN));
    message += "skipping_publish_num > 1. ";
  }
  if (skipping_publish_num >= 5) {
  level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::ERROR));
    message += "skipping_publish_num exceed limit. ";
  }

  status.level = level;
  status.message = message;

  DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();
  diag_msg.status.push_back(status);
  diagnostics_pub_->publish(diag_msg);
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(NDTScanMatcherComponent)
