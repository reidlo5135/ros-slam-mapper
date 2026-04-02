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
  RCLCPP_INFO(
    this->get_logger(),
    "Configured SLAM scan matcher utility node with odom='%s', imu='%s', scan='%s'",
    this->odom_topic_.c_str(),
    this->imu_topic_.c_str(),
    this->scan_topic_.c_str());
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
  return CallbackReturn::SUCCESS;
}

SlamScanMatcher::CallbackReturn SlamScanMatcher::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

SlamScanMatcher::CallbackReturn SlamScanMatcher::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return this->on_cleanup(state);
}

}  // namespace slam::scan::matcher
