#include "slam_laser_filter/slam_laser_filter.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace slam::laser::filter
{

namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kReasonableMaskAbsDeg = 360.0;

}  // namespace

SlamLaserFilter::SlamLaserFilter(const rclcpp::NodeOptions &options)
: rclcpp::Node("slam_laser_filter", options),
  scan_input_topic_("/scan"),
  scan_output_topic_("/slam/mapper/scan/filtered"),
  reject_near_max_range_(true),
  max_range_margin_(0.05),
  replace_with_infinity_(true)
{
  this->declare_parameter("topics.scan_in", this->scan_input_topic_);
  this->declare_parameter("topics.scan_out", this->scan_output_topic_);
  this->declare_parameter("filters.reject_near_max_range", this->reject_near_max_range_);
  this->declare_parameter("filters.max_range_margin", this->max_range_margin_);
  this->declare_parameter("filters.replace_with_infinity", this->replace_with_infinity_);
  this->declare_parameter(
    "filters.masked_angle_ranges_deg",
    std::vector<double>{9999.0, 9999.0});

  this->load_parameters();

  this->filtered_scan_publisher_ =
    this->create_publisher<sensor_msgs::msg::LaserScan>(this->scan_output_topic_, rclcpp::SensorDataQoS());
  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_input_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });

  RCLCPP_INFO(
    this->get_logger(),
    "Configured SLAM laser filter scan_in='%s', scan_out='%s', reject_near_max_range=%s, max_range_margin=%.3f m, masked_ranges=%zu",
    this->scan_input_topic_.c_str(),
    this->scan_output_topic_.c_str(),
    this->reject_near_max_range_ ? "true" : "false",
    this->max_range_margin_,
    this->masked_angle_ranges_rad_.size());
}

void SlamLaserFilter::load_parameters()
{
  this->get_parameter("topics.scan_in", this->scan_input_topic_);
  this->get_parameter("topics.scan_out", this->scan_output_topic_);
  this->get_parameter("filters.reject_near_max_range", this->reject_near_max_range_);
  this->get_parameter("filters.max_range_margin", this->max_range_margin_);
  this->get_parameter("filters.replace_with_infinity", this->replace_with_infinity_);
  this->get_parameter("filters.masked_angle_ranges_deg", this->masked_angle_ranges_deg_);
  this->rebuild_angle_masks();
}

void SlamLaserFilter::rebuild_angle_masks()
{
  this->masked_angle_ranges_rad_.clear();
  if (this->masked_angle_ranges_deg_.size() < 2U) {
    return;
  }

  const std::size_t pair_count = this->masked_angle_ranges_deg_.size() / 2U;
  this->masked_angle_ranges_rad_.reserve(pair_count);
  for (std::size_t index = 0; index + 1U < this->masked_angle_ranges_deg_.size(); index += 2U) {
    if (
      !std::isfinite(this->masked_angle_ranges_deg_[index]) ||
      !std::isfinite(this->masked_angle_ranges_deg_[index + 1U]) ||
      std::abs(this->masked_angle_ranges_deg_[index]) > kReasonableMaskAbsDeg ||
      std::abs(this->masked_angle_ranges_deg_[index + 1U]) > kReasonableMaskAbsDeg)
    {
      continue;
    }

    double start_rad = this->masked_angle_ranges_deg_[index] * kDegToRad;
    double end_rad = this->masked_angle_ranges_deg_[index + 1U] * kDegToRad;
    if (start_rad > end_rad) {
      std::swap(start_rad, end_rad);
    }
    this->masked_angle_ranges_rad_.emplace_back(start_rad, end_rad);
  }
}

bool SlamLaserFilter::is_angle_masked(double angle_rad) const
{
  for (const auto &range : this->masked_angle_ranges_rad_) {
    if (angle_rad >= range.first && angle_rad <= range.second) {
      return true;
    }
  }
  return false;
}

void SlamLaserFilter::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  sensor_msgs::msg::LaserScan filtered_scan = *message;
  const float invalid_range = this->replace_with_infinity_ ?
    std::numeric_limits<float>::infinity() :
    std::numeric_limits<float>::quiet_NaN();

  int masked_count = 0;
  int near_max_count = 0;
  const double effective_max_range = std::max(
    static_cast<double>(message->range_min),
    static_cast<double>(message->range_max) - this->max_range_margin_);

  for (std::size_t index = 0; index < filtered_scan.ranges.size(); ++index) {
    const double angle = static_cast<double>(filtered_scan.angle_min) +
      (static_cast<double>(index) * static_cast<double>(filtered_scan.angle_increment));
    const bool masked = this->is_angle_masked(angle);

    const double range = static_cast<double>(filtered_scan.ranges[index]);
    const bool near_max = this->reject_near_max_range_ &&
      std::isfinite(range) &&
      range >= effective_max_range;

    if (!masked && !near_max) {
      continue;
    }

    filtered_scan.ranges[index] = invalid_range;
    if (index < filtered_scan.intensities.size()) {
      filtered_scan.intensities[index] = 0.0F;
    }

    if (masked) {
      ++masked_count;
    }
    if (near_max) {
      ++near_max_count;
    }
  }

  this->filtered_scan_publisher_->publish(filtered_scan);

  if (masked_count > 0 || near_max_count > 0) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Laser filter masked=%d near_max=%d total=%zu",
      masked_count,
      near_max_count,
      filtered_scan.ranges.size());
  }
}

}  // namespace slam::laser::filter
