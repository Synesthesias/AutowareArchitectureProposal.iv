#include "pointcloud_preprocessor/filter.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace pointcloud_preprocessor {

Filter::Filter(const std::string & name, const rclcpp::NodeOptions & options)
: Node(name, options) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_input_frame_ = this->declare_parameter("input_frame", std::string(""));
  tf_output_frame_ = this->declare_parameter("output_frame", std::string(""));
  use_indices_ = this->declare_parameter("use_indices", false);
  max_queue_size_ = this->declare_parameter("max_queue_size", 10);

  pub_output_ = this->create_publisher<PointCloud2>("output", 10);
  onInit();
  subscribe();
}

void Filter::onInit() {
  RCLCPP_INFO(this->get_logger(), "Filter base initialized");
}

void Filter::subscribe() {
  sub_input_ = this->create_subscription<PointCloud2>(
    "input", max_queue_size_,
    [this](PointCloud2::SharedPtr msg) {
      this->input_indices_callback(msg, nullptr);
    });

  if (use_indices_) {
    sub_indices_ = this->create_subscription<PointIndices>(
      "indices", max_queue_size_,
      [this](PointIndicesConstPtr indices_msg) {
        // TODO: Implement full sync logic
      });
  }
}

void Filter::unsubscribe() {
  sub_input_.reset();
  sub_indices_.reset();
}

void Filter::input_indices_callback(const PointCloud2ConstPtr input, const PointIndicesConstPtr indices) {
  PointCloud2 transformed = *input;
  try {
    if (!tf_input_frame_.empty() && input->header.frame_id != tf_input_frame_) {
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(tf_input_frame_, input->header.frame_id, tf2::TimePointZero);
      tf2::doTransform(*input, transformed, transform);
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
    return;
  }

  PointCloud2 filtered;
  filter(std::make_shared<PointCloud2>(transformed), indices, filtered);

  try {
    if (!tf_output_frame_.empty() && filtered.header.frame_id != tf_output_frame_) {
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform(tf_output_frame_, filtered.header.frame_id, tf2::TimePointZero);
      PointCloud2 transformed_out;
      tf2::doTransform(filtered, transformed_out, transform);
      pub_output_->publish(transformed_out);
      return;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF output transform failed: %s", ex.what());
  }
  pub_output_->publish(filtered);
}

}  // namespace pointcloud_preprocessor
