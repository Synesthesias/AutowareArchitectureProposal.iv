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

#ifndef EKF_LOCALIZER__EKF_LOCALIZER_COMPONENT_HPP_
#define EKF_LOCALIZER__EKF_LOCALIZER_COMPONENT_HPP_

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "kalman_filter/time_delay_kalman_filter.hpp"

namespace ekf_localizer
{

class EKFLocalizerComponent : public rclcpp::Node
{
public:
  explicit EKFLocalizerComponent(const rclcpp::NodeOptions & options);

private:
  void timer_callback();
  void timer_tf_callback();

  void callback_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void callback_twist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void callback_pose_with_covariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void callback_twist_with_covariance(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
  void callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  void init_ekf();
  void predict_kinematics_model();
  void measurement_update_pose(const geometry_msgs::msg::PoseStamped & pose);
  void measurement_update_twist(const geometry_msgs::msg::TwistStamped & twist);

  bool mahalanobis_gate(
    const double & dist_max, const Eigen::MatrixXd & estimated, const Eigen::MatrixXd & measured,
    const Eigen::MatrixXd & estimated_cov) const;
  bool get_transform_from_tf(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  double normalize_yaw(const double & yaw) const;
  geometry_msgs::msg::Quaternion create_quaternion_from_rpy(double r, double p, double y) const;

  void set_current_result();
  void publish_estimate_result();
  void show_current_x();

  template<typename... Args>
  void debug_info(const char * format, Args &&... args) const
  {
    if (show_debug_info_) {
      RCLCPP_INFO(this->get_logger(), format, std::forward<Args>(args)...);
    }
  }

  void debug_print_matrix(const Eigen::MatrixXd & matrix, const std::string & label) const;

  rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_tf_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscription_;

  rclcpp::TimerBase::SharedPtr timer_control_;
  rclcpp::TimerBase::SharedPtr timer_tf_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_yaw_bias_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_no_yawbias_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pub_pose_cov_no_yawbias_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_measured_pose_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_pose_with_cov_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  TimeDelayKalmanFilter ekf_;

  bool show_debug_info_;
  double ekf_rate_;
  double ekf_dt_;
  double tf_rate_;
  bool enable_yaw_bias_estimation_;
  std::string pose_frame_id_;

  int dim_x_;
  int extend_state_step_;
  int dim_x_ex_;

  double pose_additional_delay_;
  double pose_measure_uncertainty_time_;
  double pose_rate_;
  double pose_gate_dist_;
  double pose_stddev_x_;
  double pose_stddev_y_;
  double pose_stddev_yaw_;
  bool use_pose_with_covariance_;
  bool use_twist_with_covariance_;

  double twist_additional_delay_;
  double twist_rate_;
  double twist_gate_dist_;
  double twist_stddev_vx_;
  double twist_stddev_wz_;

  double proc_cov_yaw_d_;
  double proc_cov_yaw_bias_d_;
  double proc_cov_vx_d_;
  double proc_cov_wz_d_;

  enum IDX {
    X = 0,
    Y = 1,
    YAW = 2,
    YAWB = 3,
    VX = 4,
    WZ = 5,
  };

  std::shared_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_ptr_;
  geometry_msgs::msg::PoseStamped current_ekf_pose_;
  geometry_msgs::msg::PoseStamped current_ekf_pose_no_yawbias_;
  geometry_msgs::msg::TwistStamped current_ekf_twist_;
  std::array<double, 36U> current_pose_covariance_{};
  std::array<double, 36U> current_twist_covariance_{};
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__EKF_LOCALIZER_COMPONENT_HPP_
