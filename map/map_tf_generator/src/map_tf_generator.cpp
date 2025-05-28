// ROS2対応: map_tf_generator.cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class MapTfGenerator : public rclcpp::Node
{
public:
  MapTfGenerator()
  : Node("map_tf_generator"), tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    this->declare_parameter("child_frame_id", "base_link");
    this->declare_parameter("parent_frame_id", "map");
    this->declare_parameter("x", 0.0);
    this->declare_parameter("y", 0.0);
    this->declare_parameter("z", 0.0);
    this->declare_parameter("yaw", 0.0);

    timer_ = this->create_wall_timer(100ms, std::bind(&MapTfGenerator::broadcast_transform, this));
  }

private:
  void broadcast_transform()
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    std::string child_frame_id = this->get_parameter("child_frame_id").as_string();
    std::string parent_frame_id = this->get_parameter("parent_frame_id").as_string();
    double x = this->get_parameter("x").as_double();
    double y = this->get_parameter("y").as_double();
    double z = this->get_parameter("z").as_double();
    double yaw = this->get_parameter("yaw").as_double();

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = parent_frame_id;
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapTfGenerator>());
  rclcpp::shutdown();
  return 0;
}
