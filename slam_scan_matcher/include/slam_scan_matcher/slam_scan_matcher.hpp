#ifndef SLAM_SCAN_MATCHER__SLAM_SCAN_MATCHER_HPP_
#define SLAM_SCAN_MATCHER__SLAM_SCAN_MATCHER_HPP_

#include <string>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "slam_scan_matcher/scan_matcher.hpp"

namespace slam::scan::matcher
{

class SlamScanMatcher : public rclcpp_lifecycle::LifecycleNode
{
private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ScanMatcher scan_matcher_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string scan_topic_;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

protected:
public:
  explicit SlamScanMatcher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~SlamScanMatcher() = default;
};

}  // namespace slam::scan::matcher

#endif  // SLAM_SCAN_MATCHER__SLAM_SCAN_MATCHER_HPP_
