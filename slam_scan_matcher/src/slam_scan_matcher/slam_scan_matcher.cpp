#include "slam_scan_matcher/slam_scan_matcher.hpp"

namespace slam::scan::matcher
{

SlamScanMatcher::SlamScanMatcher(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("slam_scan_matcher", options),
  odom_topic_("/odom"),
  imu_topic_("/imu"),
  scan_topic_("/scan")
{
  this->declare_parameter("topics.odom", this->odom_topic_);
  this->declare_parameter("topics.imu", this->imu_topic_);
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->scan_matcher_.declare_parameters(*this);
}

SlamScanMatcher::CallbackReturn SlamScanMatcher::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("topics.odom", this->odom_topic_);
  this->get_parameter("topics.imu", this->imu_topic_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->scan_matcher_.load_parameters(*this);
  bool use_imu_heading = false;
  double imu_heading_rotation_threshold = 0.0;
  double imu_heading_blend_gain = 0.0;
  double linear_window = 0.0;
  double angular_window_deg = 0.0;
  double max_translation_correction = 0.0;
  double max_yaw_correction_deg = 0.0;
  this->get_parameter("motion_prior.use_imu_heading", use_imu_heading);
  this->get_parameter(
    "motion_prior.imu_heading_rotation_threshold",
    imu_heading_rotation_threshold);
  this->get_parameter("motion_prior.imu_heading_blend_gain", imu_heading_blend_gain);
  this->get_parameter("scan_matching.linear_window", linear_window);
  this->get_parameter("scan_matching.angular_window_deg", angular_window_deg);
  this->get_parameter(
    "scan_matching.max_translation_correction",
    max_translation_correction);
  this->get_parameter("scan_matching.max_yaw_correction_deg", max_yaw_correction_deg);
  RCLCPP_INFO(
    this->get_logger(),
    "Configured SLAM scan matcher utility node with odom='%s', imu='%s', scan='%s'",
    this->odom_topic_.c_str(),
    this->imu_topic_.c_str(),
    this->scan_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Scan matcher params use_imu_heading=%s, imu_rotation_threshold=%.3f rad/s, imu_blend_gain=%.3f",
    use_imu_heading ? "true" : "false",
    imu_heading_rotation_threshold,
    imu_heading_blend_gain);
  RCLCPP_INFO(
    this->get_logger(),
    "Scan matcher windows linear=%.3f m, angular=%.3f deg, max_translation_correction=%.3f m, max_yaw_correction=%.3f deg",
    linear_window,
    angular_window_deg,
    max_translation_correction,
    max_yaw_correction_deg);
  return CallbackReturn::SUCCESS;
}

SlamScanMatcher::CallbackReturn SlamScanMatcher::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Activated SLAM scan matcher utility node");
  return CallbackReturn::SUCCESS;
}

SlamScanMatcher::CallbackReturn SlamScanMatcher::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Deactivated SLAM scan matcher utility node");
  return CallbackReturn::SUCCESS;
}

SlamScanMatcher::CallbackReturn SlamScanMatcher::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Cleaned up SLAM scan matcher utility node");
  return CallbackReturn::SUCCESS;
}

SlamScanMatcher::CallbackReturn SlamScanMatcher::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return this->on_cleanup(state);
}

}  // namespace slam::scan::matcher
