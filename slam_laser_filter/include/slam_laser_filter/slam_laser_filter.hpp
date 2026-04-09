#ifndef SLAM_LASER_FILTER__SLAM_LASER_FILTER_HPP_
#define SLAM_LASER_FILTER__SLAM_LASER_FILTER_HPP_

#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace slam::laser::filter
{

class SlamLaserFilter : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;

  std::string scan_input_topic_;
  std::string scan_output_topic_;
  bool reject_near_max_range_;
  double max_range_margin_;
  bool replace_with_infinity_;
  std::vector<double> masked_angle_ranges_deg_;
  std::vector<std::pair<double, double>> masked_angle_ranges_rad_;

  void load_parameters();
  void rebuild_angle_masks();
  bool is_angle_masked(double angle_rad) const;
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);

protected:
public:
  explicit SlamLaserFilter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~SlamLaserFilter() = default;
};

}  // namespace slam::laser::filter

#endif  // SLAM_LASER_FILTER__SLAM_LASER_FILTER_HPP_
