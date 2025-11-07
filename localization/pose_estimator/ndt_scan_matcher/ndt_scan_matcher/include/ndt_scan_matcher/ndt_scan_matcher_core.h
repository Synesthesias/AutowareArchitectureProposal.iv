/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <autoware_localization_srvs/srv/pose_array.hpp>
#include <autoware_localization_srvs/srv/pose_with_covariance_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ndt/omp.h"
#include "ndt/pcl_generic.h"
#include "ndt/pcl_modified.h"

class NDTScanMatcherComponent : public rclcpp::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseArray = geometry_msgs::msg::PoseArray;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Float32 = std_msgs::msg::Float32;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Odom = nav_msgs::msg::Odometry;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  struct OMPParams
  {
    OMPParams() : search_method(ndt_omp::NeighborSearchMethod::KDTREE), num_threads(1) {}
    ndt_omp::NeighborSearchMethod search_method;
    int num_threads;
  };

  struct Particle
  {
    Particle(
      const geometry_msgs::msg::Pose & a_initial_pose,
      const geometry_msgs::msg::Pose & a_result_pose, const double a_score, const int a_iteration)
    : initial_pose(a_initial_pose),
      result_pose(a_result_pose),
      score(a_score),
      iteration(a_iteration)
    {
    }
    geometry_msgs::msg::Pose initial_pose;
    geometry_msgs::msg::Pose result_pose;
    double score;
    int iteration;
  };

  enum class NDTImplementType { PCL_GENERIC = 0, PCL_MODIFIED = 1, OMP = 2 };

public:
  explicit NDTScanMatcherComponent(const rclcpp::NodeOptions & options);
  ~NDTScanMatcherComponent() override = default;

private:
  void handleNDTAlign(
    const std::shared_ptr<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Request> req,
    std::shared_ptr<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Response> res);
  void handleNDTAlignPoseArray(
    const std::shared_ptr<autoware_localization_srvs::srv::PoseArray::Request> req,
    std::shared_ptr<autoware_localization_srvs::srv::PoseArray::Response> res);

  void callbackMapPoints(const PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callbackSensorPoints(const PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callbackInitialPose(const PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);

  PoseWithCovarianceStamped alignUsingMonteCarlo(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
    const PoseWithCovarianceStamped & initial_pose_with_cov);
  PoseWithCovarianceStamped alignUsingMonteCarlo(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
    const PoseArray & initial_pose_array);

  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> createNDTInstance();
  void configureNDT(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr);

  void updateTransforms();

  void publishTF(
    const std::string & frame_id, const std::string & child_frame_id, const PoseStamped & pose_msg);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform_stamped, const rclcpp::Time & time_stamp);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform_stamped);

  void publishMarkerForDebug(const Particle & particle_array, const size_t i);

  void onDiagnosticTimer();

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr map_points_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sensor_points_sub_;

  rclcpp::Publisher<PointCloud2>::SharedPtr sensor_aligned_points_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr ndt_pose_with_covariance_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr initial_pose_with_covariance_pub_;
  rclcpp::Publisher<Float32>::SharedPtr exe_time_pub_;
  rclcpp::Publisher<Float32>::SharedPtr transform_probability_pub_;
  rclcpp::Publisher<Float32>::SharedPtr iteration_num_pub_;
  rclcpp::Publisher<Float32>::SharedPtr initial_to_result_distance_pub_;
  rclcpp::Publisher<Float32>::SharedPtr initial_to_result_distance_old_pub_;
  rclcpp::Publisher<Float32>::SharedPtr initial_to_result_distance_new_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr ndt_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr ndt_monte_carlo_initial_pose_marker_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Service<autoware_localization_srvs::srv::PoseWithCovarianceStamped>::SharedPtr service_;
  rclcpp::Service<autoware_localization_srvs::srv::PoseArray>::SharedPtr
    service_ndt_align_pose_array_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  NDTImplementType ndt_implement_type_;
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr_;

  Eigen::Matrix4f base_to_sensor_matrix_;
  std::string base_frame_;
  std::string ndt_base_frame_;
  std::string map_frame_;
  double converged_param_transform_probability_;
  size_t points_queue_size_;

  std::deque<PoseWithCovarianceStamped::ConstSharedPtr> initial_pose_msg_ptr_array_;
  std::mutex ndt_map_mtx_;

  OMPParams omp_params_;

  std::map<std::string, std::string> key_value_stdmap_;
};
