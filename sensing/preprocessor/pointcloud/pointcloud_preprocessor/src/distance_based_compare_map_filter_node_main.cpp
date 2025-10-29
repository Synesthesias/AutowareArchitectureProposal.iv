#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pointcloud_preprocessor/distance_based_compare_map_filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pointcloud_preprocessor::DistanceBasedCompareMapFilterNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
