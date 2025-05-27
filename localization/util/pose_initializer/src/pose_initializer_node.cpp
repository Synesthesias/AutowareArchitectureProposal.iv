#include "rclcpp/rclcpp.hpp"
#include "pose_initializer/pose_initializer_core.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseInitializer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
