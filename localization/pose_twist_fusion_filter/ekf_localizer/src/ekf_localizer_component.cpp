// Copyright 2018-2025 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ekf_localizer/ekf_localizer_component.hpp"

#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ekf_localizer
{

namespace
{
constexpr int kDimX = 6;  // x, y, yaw, yaw_bias, vx, wz
constexpr double kMinEkfRate = 0.1;
constexpr double kMinTfRate = 0.1;
constexpr double kDegPerRad = 180.0 / M_PI;
constexpr double kThrottleDefaultMs = 2000.0;

inline int to_throttle_ms(double seconds)
{
  return seconds > 0.0 ? static_cast<int>(seconds * 1000.0) : static_cast<int>(kThrottleDefaultMs);
}
}  // namespace

EKFLocalizerComponent::EKFLocalizerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("ekf_localizer", options)
{
  current_pose_covariance_.fill(0.0);
  current_twist_covariance_.fill(0.0);

  show_debug_info_ = this->declare_parameter<bool>("show_debug_info", false);
  ekf_rate_ = this->declare_parameter<double>("predict_frequency", 50.0);
  ekf_rate_ = std::max(ekf_rate_, kMinEkfRate);
  ekf_dt_ = 1.0 / ekf_rate_;
  tf_rate_ = this->declare_parameter<double>("tf_rate", 10.0);
  tf_rate_ = std::max(tf_rate_, kMinTfRate);
  enable_yaw_bias_estimation_ = this->declare_parameter<bool>("enable_yaw_bias_estimation", true);
  extend_state_step_ = this->declare_parameter<int>("extend_state_step", 50);
  extend_state_step_ = std::max(extend_state_step_, 1);
  pose_frame_id_ = this->declare_parameter<std::string>("pose_frame_id", "map");

  pose_additional_delay_ = this->declare_parameter<double>("pose_additional_delay", 0.0);
  pose_measure_uncertainty_time_ = this->declare_parameter<double>(
    "pose_measure_uncertainty_time", 0.01);
  pose_rate_ = this->declare_parameter<double>("pose_rate", 10.0);
  pose_gate_dist_ = this->declare_parameter<double>("pose_gate_dist", 10000.0);
  pose_stddev_x_ = this->declare_parameter<double>("pose_stddev_x", 0.05);
  pose_stddev_y_ = this->declare_parameter<double>("pose_stddev_y", 0.05);
  pose_stddev_yaw_ = this->declare_parameter<double>("pose_stddev_yaw", 0.035);
  use_pose_with_covariance_ =
    this->declare_parameter<bool>("use_pose_with_covariance", false);

  twist_additional_delay_ = this->declare_parameter<double>("twist_additional_delay", 0.0);
  twist_rate_ = this->declare_parameter<double>("twist_rate", 10.0);
  twist_gate_dist_ = this->declare_parameter<double>("twist_gate_dist", 10000.0);
  twist_stddev_vx_ = this->declare_parameter<double>("twist_stddev_vx", 0.2);
  twist_stddev_wz_ = this->declare_parameter<double>("twist_stddev_wz", 0.03);
  use_twist_with_covariance_ =
    this->declare_parameter<bool>("use_twist_with_covariance", false);

  double proc_stddev_yaw_c = this->declare_parameter<double>("proc_stddev_yaw_c", 0.005);
  double proc_stddev_yaw_bias_c = this->declare_parameter<double>("proc_stddev_yaw_bias_c", 0.001);
  double proc_stddev_vx_c = this->declare_parameter<double>("proc_stddev_vx_c", 5.0);
  double proc_stddev_wz_c = this->declare_parameter<double>("proc_stddev_wz_c", 1.0);

  if (!enable_yaw_bias_estimation_) {
    proc_stddev_yaw_bias_c = 0.0;
  }

  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c * ekf_dt_, 2.0);
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c * ekf_dt_, 2.0);
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c * ekf_dt_, 2.0);
  proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c * ekf_dt_, 2.0);

  dim_x_ = kDimX;
  dim_x_ex_ = dim_x_ * extend_state_step_;

  callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_tf_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_subscription_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_control_ = this->create_wall_timer(
    std::chrono::duration<double>(ekf_dt_),
    std::bind(&EKFLocalizerComponent::timer_callback, this), callback_group_timer_);

  timer_tf_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / tf_rate_),
    std::bind(&EKFLocalizerComponent::timer_tf_callback, this), callback_group_tf_);

  const auto default_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", default_qos);
  pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", default_qos);
  pub_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "ekf_twist", default_qos);
  pub_twist_cov_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "ekf_twist_with_covariance", default_qos);
  pub_yaw_bias_ = this->create_publisher<std_msgs::msg::Float64>(
    "~/estimated_yaw_bias", default_qos);
  pub_pose_no_yawbias_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "ekf_pose_without_yawbias", default_qos);
  pub_pose_cov_no_yawbias_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance_without_yawbias", default_qos);
  pub_debug_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("~/debug", default_qos);
  pub_measured_pose_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("~/debug/measured_pose", default_qos);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_subscription_;

  sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&EKFLocalizerComponent::callback_initial_pose, this, std::placeholders::_1),
    subscription_options);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  sub_pose_with_cov_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", sensor_qos,
    std::bind(&EKFLocalizerComponent::callback_pose_with_covariance, this, std::placeholders::_1),
    subscription_options);
  sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "in_pose", sensor_qos,
    std::bind(&EKFLocalizerComponent::callback_pose, this, std::placeholders::_1),
    subscription_options);
  sub_twist_with_cov_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", sensor_qos,
    std::bind(&EKFLocalizerComponent::callback_twist_with_covariance, this, std::placeholders::_1),
    subscription_options);
  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "in_twist", sensor_qos,
    std::bind(&EKFLocalizerComponent::callback_twist, this, std::placeholders::_1),
    subscription_options);

  const auto node_ptr = rclcpp::Node::SharedPtr(this, [](auto *) {});
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_ptr);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_ptr, false);

  init_ekf();
}

void EKFLocalizerComponent::timer_callback()
{
  debug_info("========================= timer called =========================");

  const auto prediction_start = std::chrono::steady_clock::now();
  debug_info("------------------------- start prediction -------------------------");
  predict_kinematics_model();
  const auto prediction_elapsed =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
                                                         prediction_start)
      .count();
  debug_info("[EKF] predictKinematicsModel calculation time = %f [ms]", prediction_elapsed * 1.0e-6);
  debug_info("------------------------- end prediction -------------------------\n");

  if (current_pose_ptr_) {
    debug_info("------------------------- start Pose -------------------------");
    const auto pose_start = std::chrono::steady_clock::now();
    measurement_update_pose(*current_pose_ptr_);
    const auto pose_elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
                                                           pose_start)
        .count();
    debug_info(
      "[EKF] measurementUpdatePose calculation time = %f [ms]", pose_elapsed * 1.0e-6);
    debug_info("------------------------- end Pose -------------------------\n");
  }

  if (current_twist_ptr_) {
    debug_info("------------------------- start twist -------------------------");
    const auto twist_start = std::chrono::steady_clock::now();
    measurement_update_twist(*current_twist_ptr_);
    const auto twist_elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
                                                           twist_start)
        .count();
    debug_info(
      "[EKF] measurementUpdateTwist calculation time = %f [ms]", twist_elapsed * 1.0e-6);
    debug_info("------------------------- end twist -------------------------\n");
  }

  set_current_result();
  publish_estimate_result();
}

void EKFLocalizerComponent::timer_tf_callback()
{
  if (current_ekf_pose_.header.frame_id.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = current_ekf_pose_.header.frame_id;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = current_ekf_pose_.pose.position.x;
  transform.transform.translation.y = current_ekf_pose_.pose.position.y;
  transform.transform.translation.z = current_ekf_pose_.pose.position.z;
  transform.transform.rotation = current_ekf_pose_.pose.orientation;

  tf_broadcaster_->sendTransform(transform);
}

void EKFLocalizerComponent::callback_pose(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (use_pose_with_covariance_) {
    return;
  }
  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
}

void EKFLocalizerComponent::callback_twist(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  if (use_twist_with_covariance_) {
    return;
  }
  current_twist_ptr_ = std::make_shared<geometry_msgs::msg::TwistStamped>(*msg);
}

void EKFLocalizerComponent::callback_pose_with_covariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!use_pose_with_covariance_) {
    return;
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
  current_pose_covariance_ = msg->pose.covariance;
}

void EKFLocalizerComponent::callback_twist_with_covariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  if (!use_twist_with_covariance_) {
    return;
  }
  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  current_twist_ptr_ = std::make_shared<geometry_msgs::msg::TwistStamped>(twist);
  current_twist_covariance_ = msg->twist.covariance;
}

void EKFLocalizerComponent::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!get_transform_from_tf(pose_frame_id_, msg->header.frame_id, transform)) {
    RCLCPP_ERROR(
      this->get_logger(), "[EKF] TF transform failed. parent = %s, child = %s",
      pose_frame_id_.c_str(), msg->header.frame_id.c_str());
  }

  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  X(IDX::X) = msg->pose.pose.position.x + transform.transform.translation.x;
  X(IDX::Y) = msg->pose.pose.position.y + transform.transform.translation.y;
  current_ekf_pose_.pose.position.z =
    msg->pose.pose.position.z + transform.transform.translation.z;
  X(IDX::YAW) = tf2::getYaw(msg->pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
  X(IDX::YAWB) = 0.0;
  X(IDX::VX) = 0.0;
  X(IDX::WZ) = 0.0;

  P(IDX::X, IDX::X) = msg->pose.covariance[0];
  P(IDX::Y, IDX::Y) = msg->pose.covariance[6 + 1];
  P(IDX::YAW, IDX::YAW) = msg->pose.covariance[6 * 5 + 5];
  P(IDX::YAWB, IDX::YAWB) = 0.0001;
  P(IDX::VX, IDX::VX) = 0.01;
  P(IDX::WZ, IDX::WZ) = 0.01;

  ekf_.init(X, P, extend_state_step_);
  current_pose_ptr_.reset();
}

bool EKFLocalizerComponent::get_transform_from_tf(
  std::string parent_frame, std::string child_frame, geometry_msgs::msg::TransformStamped & transform)
{
  if (!tf_buffer_) {
    return false;
  }

  if (!parent_frame.empty() && parent_frame.front() == '/') {
    parent_frame.erase(0, 1);
  }
  if (!child_frame.empty() && child_frame.front() == '/') {
    child_frame.erase(0, 1);
  }

  for (int i = 0; i < 50; ++i) {
    try {
      transform = tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return false;
}

void EKFLocalizerComponent::init_ekf()
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;
  P(IDX::YAW, IDX::YAW) = 50.0;
  P(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;
  P(IDX::VX, IDX::VX) = 1000.0;
  P(IDX::WZ, IDX::WZ) = 50.0;

  ekf_.init(X, P, extend_state_step_);
}

void EKFLocalizerComponent::predict_kinematics_model()
{
  Eigen::MatrixXd X_curr(dim_x_, 1);
  Eigen::MatrixXd X_next(dim_x_, 1);
  ekf_.getLatestX(X_curr);
  debug_print_matrix(X_curr.transpose(), "X_curr");

  const double yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double vx = X_curr(IDX::VX);
  const double wz = X_curr(IDX::WZ);
  const double dt = ekf_dt_;

  X_next(IDX::X) = X_curr(IDX::X) + vx * std::cos(yaw + yaw_bias) * dt;
  X_next(IDX::Y) = X_curr(IDX::Y) + vx * std::sin(yaw + yaw_bias) * dt;
  X_next(IDX::YAW) = normalize_yaw(X_curr(IDX::YAW) + wz * dt);
  X_next(IDX::YAWB) = yaw_bias;
  X_next(IDX::VX) = vx;
  X_next(IDX::WZ) = wz;

  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(IDX::X, IDX::YAW) = -vx * std::sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::YAWB) = -vx * std::sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::VX) = std::cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAW) = vx * std::cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAWB) = vx * std::cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::VX) = std::sin(yaw + yaw_bias) * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
  Q(IDX::YAW, IDX::YAW) = proc_cov_yaw_d_;
  Q(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;
  Q(IDX::VX, IDX::VX) = proc_cov_vx_d_;
  Q(IDX::WZ, IDX::WZ) = proc_cov_wz_d_;

  if (!ekf_.predictWithDelay(X_next, A, Q)) {
    RCLCPP_WARN(this->get_logger(), "[EKF] predictWithDelay failed");
  }

  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  debug_print_matrix(X_result.transpose(), "X_result");
}

void EKFLocalizerComponent::measurement_update_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (pose.header.frame_id != pose_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), to_throttle_ms(2.0),
      "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
      pose.header.frame_id.c_str(), pose_frame_id_.c_str());
  }

  Eigen::MatrixXd X_curr(dim_x_, 1);
  ekf_.getLatestX(X_curr);
  debug_print_matrix(X_curr.transpose(), "X_curr_pose");

  constexpr int dim_y = 3;
  const rclcpp::Time current_time = this->now();
  const rclcpp::Time pose_time(pose.header.stamp);

  double delay_time = (current_time - pose_time).seconds() + pose_additional_delay_;
  if (delay_time < 0.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), to_throttle_ms(1.0),
      "Pose time stamp is inappropriate, set delay to 0[s]. delay = %f", delay_time);
    delay_time = 0.0;
  }

  int delay_step = static_cast<int>(std::round(delay_time / ekf_dt_));
  if (delay_step > extend_state_step_ - 1) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), to_throttle_ms(1.0),
      "Pose delay exceeds the compensation limit, ignored. delay: %f[s], limit = %f [s]",
      delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  debug_info("delay_time: %f [s]", delay_time);

  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  double yaw = tf2::getYaw(pose.pose.orientation);
  const double yaw_error = normalize_yaw(yaw - ekf_yaw);
  yaw = yaw_error + ekf_yaw;

  Eigen::MatrixXd measurement(dim_y, 1);
  measurement << pose.pose.position.x, pose.pose.position.y, yaw;

  if (!measurement.allFinite()) {
    RCLCPP_WARN(this->get_logger(), "[EKF] pose measurement contains NaN/Inf. Ignoring update.");
    return;
  }

  Eigen::MatrixXd measurement_pred(dim_y, 1);
  measurement_pred << ekf_.getXelement(delay_step * dim_x_ + IDX::X),
    ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;

  Eigen::MatrixXd P_curr;
  ekf_.getLatestP(P_curr);
  Eigen::MatrixXd P_y = P_curr.block(0, 0, dim_y, dim_y);

  if (!mahalanobis_gate(pose_gate_dist_, measurement_pred, measurement, P_y)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), to_throttle_ms(2.0),
      "[EKF] Pose measurement update, mahalanobis distance is over limit. Ignoring measurement.");
    return;
  }

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;
  C(1, IDX::Y) = 1.0;
  C(2, IDX::YAW) = 1.0;

  if (enable_yaw_bias_estimation_) {
    C(2, IDX::YAWB) = 1.0;
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_pose_with_covariance_) {
    R(0, 0) = current_pose_covariance_.at(0);
    R(0, 1) = current_pose_covariance_.at(1);
    R(0, 2) = current_pose_covariance_.at(5);
    R(1, 0) = current_pose_covariance_.at(6);
    R(1, 1) = current_pose_covariance_.at(7);
    R(1, 2) = current_pose_covariance_.at(11);
    R(2, 0) = current_pose_covariance_.at(30);
    R(2, 1) = current_pose_covariance_.at(31);
    R(2, 2) = current_pose_covariance_.at(35);
  } else {
    const double vx = ekf_.getXelement(delay_step * dim_x_ + IDX::VX);
    const double wz = ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
    const double cov_tu_pos_x = std::pow(pose_measure_uncertainty_time_ * vx * std::cos(ekf_yaw), 2.0);
    const double cov_tu_pos_y = std::pow(pose_measure_uncertainty_time_ * vx * std::sin(ekf_yaw), 2.0);
    const double cov_tu_yaw = std::pow(pose_measure_uncertainty_time_ * wz, 2.0);
    R(0, 0) = pose_stddev_x_ * pose_stddev_x_ + cov_tu_pos_x;
    R(1, 1) = pose_stddev_y_ * pose_stddev_y_ + cov_tu_pos_y;
    R(2, 2) = pose_stddev_yaw_ * pose_stddev_yaw_ + cov_tu_yaw;
  }

  const double pose_rate = std::max(pose_rate_, kMinEkfRate);
  R *= (ekf_rate_ / pose_rate);

  if (!ekf_.updateWithDelay(measurement, C, R, delay_step)) {
    RCLCPP_WARN(this->get_logger(), "[EKF] updateWithDelay for pose failed");
    return;
  }

  if (pub_measured_pose_) {
    pub_measured_pose_->publish(pose);
  }

  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  debug_print_matrix(X_result.transpose(), "X_result_pose");
}

void EKFLocalizerComponent::measurement_update_twist(const geometry_msgs::msg::TwistStamped & twist)
{
  if (twist.header.frame_id != "base_link") {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), to_throttle_ms(2.0),
      "twist frame_id must be base_link");
  }

  Eigen::MatrixXd X_curr(dim_x_, 1);
  ekf_.getLatestX(X_curr);
  debug_print_matrix(X_curr.transpose(), "X_curr_twist");

  constexpr int dim_y = 2;
  const rclcpp::Time current_time = this->now();
  const rclcpp::Time twist_time(twist.header.stamp);

  double delay_time = (current_time - twist_time).seconds() + twist_additional_delay_;
  if (delay_time < 0.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), to_throttle_ms(1.0),
      "Twist time stamp is inappropriate (delay = %f [s]), set delay to 0[s].", delay_time);
    delay_time = 0.0;
  }

  int delay_step = static_cast<int>(std::round(delay_time / ekf_dt_));
  if (delay_step > extend_state_step_ - 1) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), to_throttle_ms(1.0),
      "Twist delay exceeds the compensation limit, ignored. delay: %f[s], limit = %f [s]",
      delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  debug_info("delay_time: %f [s]", delay_time);

  Eigen::MatrixXd measurement(dim_y, 1);
  measurement << twist.twist.linear.x, twist.twist.angular.z;

  if (!measurement.allFinite()) {
    RCLCPP_WARN(this->get_logger(), "[EKF] twist measurement contains NaN/Inf. Ignoring update.");
    return;
  }

  Eigen::MatrixXd measurement_pred(dim_y, 1);
  measurement_pred << ekf_.getXelement(delay_step * dim_x_ + IDX::VX),
    ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);

  Eigen::MatrixXd P_curr;
  ekf_.getLatestP(P_curr);
  Eigen::MatrixXd P_y = P_curr.block(4, 4, dim_y, dim_y);

  if (!mahalanobis_gate(twist_gate_dist_, measurement_pred, measurement, P_y)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), to_throttle_ms(2.0),
      "[EKF] Twist measurement update, mahalanobis distance is over limit. Ignoring measurement.");
    return;
  }

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::VX) = 1.0;
  C(1, IDX::WZ) = 1.0;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_twist_with_covariance_) {
    R(0, 0) = current_twist_covariance_.at(0);
    R(0, 1) = current_twist_covariance_.at(5);
    R(1, 0) = current_twist_covariance_.at(30);
    R(1, 1) = current_twist_covariance_.at(35);
  } else {
    R(0, 0) = twist_stddev_vx_ * twist_stddev_vx_;
    R(1, 1) = twist_stddev_wz_ * twist_stddev_wz_;
  }

  const double twist_rate = std::max(twist_rate_, kMinEkfRate);
  R *= (ekf_rate_ / twist_rate);

  if (!ekf_.updateWithDelay(measurement, C, R, delay_step)) {
    RCLCPP_WARN(this->get_logger(), "[EKF] updateWithDelay for twist failed");
  }

  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  debug_print_matrix(X_result.transpose(), "X_result_twist");
}

bool EKFLocalizerComponent::mahalanobis_gate(
  const double & dist_max, const Eigen::MatrixXd & estimated, const Eigen::MatrixXd & measured,
  const Eigen::MatrixXd & estimated_cov) const
{
  Eigen::MatrixXd diff = measured - estimated;
  Eigen::MatrixXd mahalanobis = diff.transpose() * estimated_cov.inverse() * diff;
  const double distance = mahalanobis(0, 0);
  debug_info("measurement update: mahalanobis = %f, gate limit = %f", std::sqrt(distance), dist_max);
  return distance <= dist_max * dist_max;
}

double EKFLocalizerComponent::normalize_yaw(const double & yaw) const
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

geometry_msgs::msg::Quaternion EKFLocalizerComponent::create_quaternion_from_rpy(
  double r, double p, double y) const
{
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  return tf2::toMsg(q);
}

void EKFLocalizerComponent::set_current_result()
{
  current_ekf_pose_.header.frame_id = pose_frame_id_;
  current_ekf_pose_.header.stamp = this->now();
  current_ekf_pose_.pose.position.x = ekf_.getXelement(IDX::X);
  current_ekf_pose_.pose.position.y = ekf_.getXelement(IDX::Y);

  tf2::Quaternion q_tf;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  if (current_pose_ptr_) {
    current_ekf_pose_.pose.position.z = current_pose_ptr_->pose.position.z;
    tf2::fromMsg(current_pose_ptr_->pose.orientation, q_tf);
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  }

  yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);
  current_ekf_pose_.pose.orientation = create_quaternion_from_rpy(roll, pitch, yaw);

  current_ekf_pose_no_yawbias_ = current_ekf_pose_;
  current_ekf_pose_no_yawbias_.pose.orientation =
    create_quaternion_from_rpy(roll, pitch, ekf_.getXelement(IDX::YAW));

  current_ekf_twist_.header.frame_id = "base_link";
  current_ekf_twist_.header.stamp = this->now();
  current_ekf_twist_.twist.linear.x = ekf_.getXelement(IDX::VX);
  current_ekf_twist_.twist.angular.z = ekf_.getXelement(IDX::WZ);
}

void EKFLocalizerComponent::publish_estimate_result()
{
  const rclcpp::Time current_time = this->now();
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P(dim_x_, dim_x_);
  ekf_.getLatestX(X);
  ekf_.getLatestP(P);

  if (pub_pose_) {
    pub_pose_->publish(current_ekf_pose_);
  }
  if (pub_pose_no_yawbias_) {
    pub_pose_no_yawbias_->publish(current_ekf_pose_no_yawbias_);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_ekf_pose_.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose_.pose;
  pose_cov.pose.covariance[0] = P(IDX::X, IDX::X);
  pose_cov.pose.covariance[1] = P(IDX::X, IDX::Y);
  pose_cov.pose.covariance[5] = P(IDX::X, IDX::YAW);
  pose_cov.pose.covariance[6] = P(IDX::Y, IDX::X);
  pose_cov.pose.covariance[7] = P(IDX::Y, IDX::Y);
  pose_cov.pose.covariance[11] = P(IDX::Y, IDX::YAW);
  pose_cov.pose.covariance[30] = P(IDX::YAW, IDX::X);
  pose_cov.pose.covariance[31] = P(IDX::YAW, IDX::Y);
  pose_cov.pose.covariance[35] = P(IDX::YAW, IDX::YAW);
  if (pub_pose_cov_) {
    pub_pose_cov_->publish(pose_cov);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_no_yawbias = pose_cov;
  pose_cov_no_yawbias.pose.pose = current_ekf_pose_no_yawbias_.pose;
  if (pub_pose_cov_no_yawbias_) {
    pub_pose_cov_no_yawbias_->publish(pose_cov_no_yawbias);
  }

  if (pub_twist_) {
    pub_twist_->publish(current_ekf_twist_);
  }

  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_time;
  twist_cov.header.frame_id = current_ekf_twist_.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist_.twist;
  twist_cov.twist.covariance[0] = P(IDX::VX, IDX::VX);
  twist_cov.twist.covariance[5] = P(IDX::VX, IDX::WZ);
  twist_cov.twist.covariance[30] = P(IDX::WZ, IDX::VX);
  twist_cov.twist.covariance[35] = P(IDX::WZ, IDX::WZ);
  if (pub_twist_cov_) {
    pub_twist_cov_->publish(twist_cov);
  }

  std_msgs::msg::Float64 yaw_bias_msg;
  yaw_bias_msg.data = X(IDX::YAWB);
  if (pub_yaw_bias_) {
    pub_yaw_bias_->publish(yaw_bias_msg);
  }

  if (current_pose_ptr_) {
    auto measured_pose = *current_pose_ptr_;
    measured_pose.header.stamp = current_time;
    if (pub_measured_pose_) {
      pub_measured_pose_->publish(measured_pose);
    }
  }

  std_msgs::msg::Float64MultiArray debug_msg;
  debug_msg.data.reserve(3);
  debug_msg.data.push_back(X(IDX::YAW) * kDegPerRad);
  double pose_yaw_deg = 0.0;
  if (current_pose_ptr_) {
    pose_yaw_deg = tf2::getYaw(current_pose_ptr_->pose.orientation) * kDegPerRad;
  }
  debug_msg.data.push_back(pose_yaw_deg);
  debug_msg.data.push_back(X(IDX::YAWB) * kDegPerRad);
  if (pub_debug_) {
    pub_debug_->publish(debug_msg);
  }
}

void EKFLocalizerComponent::show_current_x()
{
  if (!show_debug_info_) {
    return;
  }
  Eigen::MatrixXd X(dim_x_, 1);
  ekf_.getLatestX(X);
  debug_print_matrix(X.transpose(), "X");
}

void EKFLocalizerComponent::debug_print_matrix(
  const Eigen::MatrixXd & matrix, const std::string & label) const
{
  if (!show_debug_info_) {
    return;
  }
  std::stringstream ss;
  ss << label << ":\n" << matrix;
  RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
}

}  // namespace ekf_localizer

RCLCPP_COMPONENTS_REGISTER_NODE(ekf_localizer::EKFLocalizerComponent)
