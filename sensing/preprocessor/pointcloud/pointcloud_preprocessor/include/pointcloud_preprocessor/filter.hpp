#pragma once
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_msgs/msg/point_indices.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace pointcloud_preprocessor {
class Filter : public rclcpp::Node {
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointIndices = pcl_msgs::msg::PointIndices;
  using PointCloud2ConstPtr = std::shared_ptr<const PointCloud2>;
  using PointIndicesConstPtr = std::shared_ptr<const PointIndices>;

  explicit Filter(const std::string & name, const rclcpp::NodeOptions & options);

protected:
  virtual void filter(const PointCloud2ConstPtr & input, const PointIndicesConstPtr & indices, PointCloud2 & output) = 0;
  virtual void subscribe();
  virtual void unsubscribe();
  virtual void onInit();

  void input_indices_callback(const PointCloud2ConstPtr input, const PointIndicesConstPtr indices);

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;
  rclcpp::Subscription<PointIndices>::SharedPtr sub_indices_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  std::string tf_input_frame_;
  std::string tf_output_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool use_indices_ = false;
  size_t max_queue_size_ = 10;
  std::mutex mutex_;
};
}  // namespace pointcloud_preprocessor
