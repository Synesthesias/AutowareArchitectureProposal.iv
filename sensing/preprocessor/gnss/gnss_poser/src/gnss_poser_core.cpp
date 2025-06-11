#include "gnss_poser/gnss_poser_core.hpp"
#include "gnss_poser/convert.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

namespace GNSSPoser
{

GNSSPoserNode::GNSSPoserNode(const rclcpp::NodeOptions & options)
: Node("gnss_poser", options),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_),
  tf2_broadcaster_(this),
  coordinate_system_(CoordinateSystem::MGRS),
  base_frame_("base_link"),
  gnss_frame_("gnss"),
  gnss_base_frame_("gnss_base_link"),
  map_frame_("map"),
  plane_zone_(9),
  buffer_capacity_(10)
{
  this->declare_parameter("coordinate_system", static_cast<int>(coordinate_system_));
  this->declare_parameter("base_frame", base_frame_);
  this->declare_parameter("gnss_frame", gnss_frame_);
  this->declare_parameter("gnss_base_frame", gnss_base_frame_);
  this->declare_parameter("map_frame", map_frame_);
  this->declare_parameter("plane_zone", plane_zone_);
  this->declare_parameter("buff_epoch", buffer_capacity_);

  int coord_system;
  this->get_parameter("coordinate_system", coord_system);
  coordinate_system_ = static_cast<CoordinateSystem>(coord_system);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("gnss_frame", gnss_frame_);
  this->get_parameter("gnss_base_frame", gnss_base_frame_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("plane_zone", plane_zone_);
  this->get_parameter("buff_epoch", buffer_capacity_);

  fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix", 10, std::bind(&GNSSPoserNode::callbackNavSatFix, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", 10);
  pose_cov_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gnss_pose_cov", 10);
  fixed_pub_ = this->create_publisher<std_msgs::msg::Bool>("gnss_fixed", 10);
}

void GNSSPoserNode::callbackNavSatFix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std_msgs::msg::Bool fixed_msg;
  fixed_msg.data = isFixed(msg->status);
  fixed_pub_->publish(fixed_msg);
  if (!fixed_msg.data) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Not fixed. Skipping.");
    return;
  }

  const auto gnss_stat = convert(*msg);
  const auto position = getPosition(gnss_stat);

  position_buffer_.push_front(position);
  if (position_buffer_.size() > static_cast<size_t>(buffer_capacity_)) {
    position_buffer_.pop_back();
  }

  if (position_buffer_.size() < static_cast<size_t>(buffer_capacity_)) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Buffering. Skipping.");
    return;
  }

  const auto median = getMedianPosition(position_buffer_);
  const auto orientation = getQuaternionByPositionDiff(median, prev_position_);
  prev_position_ = median;

  geometry_msgs::msg::PoseStamped antenna_pose;
  antenna_pose.header.stamp = msg->header.stamp;
  antenna_pose.header.frame_id = map_frame_;
  antenna_pose.pose.position = median;
  antenna_pose.pose.orientation = orientation;

  geometry_msgs::msg::TransformStamped tf;
  if (!getStaticTransform(gnss_frame_, base_frame_, tf, msg->header.stamp)) return;

  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;

  geometry_msgs::msg::PoseStamped base_pose;
  tf2::doTransform(antenna_pose, base_pose, tf);
  base_pose.header.frame_id = map_frame_;

  pose_pub_->publish(base_pose);

  geometry_msgs::msg::PoseWithCovarianceStamped base_cov;
  base_cov.header = base_pose.header;
  base_cov.pose.pose = base_pose.pose;
  base_cov.pose.covariance[0] = canGetCovariance(*msg) ? msg->position_covariance[0] : 10.0;
  base_cov.pose.covariance[7] = canGetCovariance(*msg) ? msg->position_covariance[4] : 10.0;
  base_cov.pose.covariance[14] = canGetCovariance(*msg) ? msg->position_covariance[8] : 10.0;
  base_cov.pose.covariance[21] = 0.1;
  base_cov.pose.covariance[28] = 0.1;
  base_cov.pose.covariance[35] = 1.0;
  pose_cov_pub_->publish(base_cov);

  publishTF(map_frame_, gnss_base_frame_, base_pose);
}

bool GNSSPoserNode::isFixed(const sensor_msgs::msg::NavSatStatus & status)
{
  return status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

bool GNSSPoserNode::canGetCovariance(const sensor_msgs::msg::NavSatFix & msg)
{
  return msg.position_covariance_type > sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

GNSSStat GNSSPoserNode::convert(const sensor_msgs::msg::NavSatFix & msg)
{
  using GNSSPoser::NavSatFix2UTM;
  using GNSSPoser::NavSatFix2MGRS;
  using GNSSPoser::NavSatFix2PLANE;
  if (coordinate_system_ == CoordinateSystem::UTM) return NavSatFix2UTM(msg);
  if (coordinate_system_ == CoordinateSystem::MGRS) return NavSatFix2MGRS(msg, MGRSPrecision::_100MICRO_METER);
  if (coordinate_system_ == CoordinateSystem::PLANE) return NavSatFix2PLANE(msg, plane_zone_);
  RCLCPP_ERROR(this->get_logger(), "Unknown coordinate system");
  return {};
}

geometry_msgs::msg::Point GNSSPoserNode::getPosition(const GNSSStat & stat)
{
  geometry_msgs::msg::Point p;
  p.x = stat.x;
  p.y = stat.y;
  p.z = stat.z;
  return p;
}

geometry_msgs::msg::Point GNSSPoserNode::getMedianPosition(const std::deque<geometry_msgs::msg::Point> & buf)
{
  auto getMedian = [](std::vector<double> vals) {
    std::sort(vals.begin(), vals.end());
    size_t n = vals.size();
    return n % 2 ? vals[n / 2] : (vals[n / 2 - 1] + vals[n / 2]) / 2.0;
  };

  std::vector<double> xs, ys, zs;
  for (const auto & p : buf) {
    xs.push_back(p.x);
    ys.push_back(p.y);
    zs.push_back(p.z);
  }

  geometry_msgs::msg::Point median;
  median.x = getMedian(xs);
  median.y = getMedian(ys);
  median.z = getMedian(zs);
  return median;
}

geometry_msgs::msg::Quaternion GNSSPoserNode::getQuaternionByPositionDiff(
  const geometry_msgs::msg::Point & p, const geometry_msgs::msg::Point & prev)
{
  const double yaw = std::atan2(p.y - prev.y, p.x - prev.x);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

bool GNSSPoserNode::getStaticTransform(
  const std::string & target, const std::string & source,
  geometry_msgs::msg::TransformStamped & tf, const rclcpp::Time & stamp)
{
  try {
    tf = tf2_buffer_.lookupTransform(target, source, tf2::TimePointZero);
    tf.header.stamp = stamp;
    return true;
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, e.what());
    return false;
  }
}

void GNSSPoserNode::publishTF(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::msg::PoseStamped & pose_msg)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = frame_id;
  tf.child_frame_id = child_frame_id;
  tf.header.stamp = pose_msg.header.stamp;
  tf.transform.translation.x = pose_msg.pose.position.x;
  tf.transform.translation.y = pose_msg.pose.position.y;
  tf.transform.translation.z = pose_msg.pose.position.z;
  tf2::Quaternion q;
  tf2::fromMsg(pose_msg.pose.orientation, q);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf2_broadcaster_.sendTransform(tf);
}

}  // namespace GNSSPoser
