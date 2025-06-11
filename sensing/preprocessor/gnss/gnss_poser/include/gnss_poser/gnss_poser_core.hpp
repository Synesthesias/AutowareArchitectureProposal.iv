#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <deque>
#include <string>

#include "gnss_poser/gnss_stat.hpp"

namespace GNSSPoser
{

class GNSSPoserNode : public rclcpp::Node
{
public:
  explicit GNSSPoserNode(const rclcpp::NodeOptions & options);

private:
  void callbackNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  bool isFixed(const sensor_msgs::msg::NavSatStatus & status);
  bool canGetCovariance(const sensor_msgs::msg::NavSatFix & msg);
  GNSSStat convert(const sensor_msgs::msg::NavSatFix & msg);
  geometry_msgs::msg::Point getPosition(const GNSSStat & stat);
  geometry_msgs::msg::Point getMedianPosition(const std::deque<geometry_msgs::msg::Point> & buffer);
  geometry_msgs::msg::Quaternion getQuaternionByPositionDiff(const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & prev_p);

  bool getStaticTransform(
    const std::string & target_frame, const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform_stamped,
    const rclcpp::Time & stamp);

  void publishTF(
    const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::msg::PoseStamped & pose_msg);

  std::string base_frame_;
  std::string gnss_frame_;
  std::string gnss_base_frame_;
  std::string map_frame_;
  int plane_zone_;
  int buffer_capacity_;
  CoordinateSystem coordinate_system_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fixed_pub_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  std::deque<geometry_msgs::msg::Point> position_buffer_;
  geometry_msgs::msg::Point prev_position_;
};

}  // namespace GNSSPoser
