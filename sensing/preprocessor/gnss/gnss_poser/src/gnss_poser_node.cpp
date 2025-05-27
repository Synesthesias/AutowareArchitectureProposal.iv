#include "rclcpp/rclcpp.hpp"
#include "gnss_poser/gnss_poser_core.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GNSSPoser::GNSSPoserNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
