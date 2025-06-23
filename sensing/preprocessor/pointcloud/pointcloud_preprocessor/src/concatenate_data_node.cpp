
#include "pointcloud_preprocessor/concatenate_data_node.hpp"

using namespace pointcloud_preprocessor;

ConcatenateDataNode::ConcatenateDataNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("concatenate_data_node", options)
{
  // Parameter setup
  this->declare_parameter("output_frame", "");
  this->declare_parameter("input_topics", std::vector<std::string>{});
  this->declare_parameter("max_queue_size", 10);
  this->declare_parameter("timeout_sec", 1.0);

  // Retrieve parameters
  this->get_parameter("output_frame", output_frame_);
  this->get_parameter("input_topics", input_topics_);
  this->get_parameter("max_queue_size", maximum_queue_size_);
  this->get_parameter("timeout_sec", timeout_sec_);

  // Output publisher
  pub_output_ = this->create_publisher<PointCloud2>("output", maximum_queue_size_);
  pub_concat_num_ = this->create_publisher<std_msgs::msg::Int32>("concat_num", 10);
  pub_not_subscribed_topic_name_ = this->create_publisher<std_msgs::msg::String>("not_subscribed_topic_name", 10);

  // Subscribe to topics
  for (const auto & topic_name : input_topics_) {
    filters_.push_back(this->create_subscription<PointCloud2>(topic_name, maximum_queue_size_, 
      [this, topic_name](const PointCloud2::SharedPtr msg) { cloud_callback(msg, topic_name); }
    ));
  }
  sub_twist_ = this->create_subscription<TwistStamped>("/vehicle/status/twist", 10, 
    [this](const TwistStamped::SharedPtr msg) { twist_callback(msg); }
  );

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timeout_sec_),
    [this]() { timer_callback(); }
  );

  timer_->cancel();
}

void ConcatenateDataNode::cloud_callback(const PointCloud2::SharedPtr msg, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Convert to XYZ cloud
  PointCloud2 xyz_cloud;
  convertToXYZCloud(*msg, xyz_cloud);
  auto xyz_input_ptr = std::make_shared<PointCloud2>(xyz_cloud);

  if (cloud_stdmap_[topic_name] != nullptr) {
    cloud_stdmap_tmp_[topic_name] = xyz_input_ptr;
    if (std::all_of(cloud_stdmap_.begin(), cloud_stdmap_.end(), 
      [](const auto & e) { return e.second != nullptr; })) {
      publish();
    }
  } else {
    cloud_stdmap_[topic_name] = xyz_input_ptr;
    timer_->reset();
  }
}

void ConcatenateDataNode::twist_callback(const TwistStamped::SharedPtr msg)
{
  // Handle twist message (e.g., buffer management)
  twist_ptr_queue_.push_back(msg);
}

void ConcatenateDataNode::timer_callback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publish();
}

void ConcatenateDataNode::transformPointCloud(const PointCloud2::SharedPtr & in, PointCloud2::SharedPtr & out)
{
  if (output_frame_ != in->header.frame_id) {
    // Transform logic (placeholder)
  } else {
    out = std::make_shared<PointCloud2>(*in);
  }
}

void ConcatenateDataNode::combineClouds(const PointCloud2::SharedPtr & in1, const PointCloud2::SharedPtr & in2, PointCloud2::SharedPtr & out)
{
  pcl::concatenatePointCloud(*in1, *in2, *out);
}

void ConcatenateDataNode::convertToXYZCloud(const PointCloud2 & input, PointCloud2 & output)
{
  pcl::PointCloud<pcl::PointXYZ> tmp_xyz_cloud;
  pcl::fromROSMsg(input, tmp_xyz_cloud);
  pcl::toROSMsg(tmp_xyz_cloud, output);
  output.header = input.header;
}

void ConcatenateDataNode::publish()
{
  PointCloud2::SharedPtr concat_cloud_ptr_ = nullptr;
  std::string not_subscribed_topic_name = "";
  size_t concat_num = 0;

  for (const auto & e : cloud_stdmap_) {
    if (e.second != nullptr) {
      PointCloud2::SharedPtr transed_cloud_ptr = std::make_shared<PointCloud2>();
      transformPointCloud(e.second, transed_cloud_ptr);
      if (concat_cloud_ptr_ == nullptr) {
        concat_cloud_ptr_ = transed_cloud_ptr;
      } else {
        combineClouds(concat_cloud_ptr_, transed_cloud_ptr, concat_cloud_ptr_);
      }
      ++concat_num;
    } else {
      if (not_subscribed_topic_name.empty()) {
        not_subscribed_topic_name = e.first;
      } else {
        not_subscribed_topic_name += "," + e.first;
      }
    }
  }

  if (!not_subscribed_topic_name.empty()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Skipped " << not_subscribed_topic_name << ". Please confirm topic.");
  }

  pub_output_->publish(*concat_cloud_ptr_);

  // Fix: Cast concat_num to int32_t
  std_msgs::msg::Int32 concat_num_msg;
  concat_num_msg.data = static_cast<int32_t>(concat_num);
  pub_concat_num_->publish(concat_num_msg);

  // Fix: Correctly create String message and set data
  std_msgs::msg::String not_subscribed_topic_name_msg;
  not_subscribed_topic_name_msg.data = not_subscribed_topic_name;
  pub_not_subscribed_topic_name_->publish(not_subscribed_topic_name_msg);
}
