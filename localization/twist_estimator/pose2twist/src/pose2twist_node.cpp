#include "pose2twist/pose2twist_core.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pose2twist");

  Pose2Twist converter(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
