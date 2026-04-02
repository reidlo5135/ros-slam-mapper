#include "slam_scan_matcher/slam_scan_matcher.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<slam::scan::matcher::SlamScanMatcher> node =
    std::make_shared<slam::scan::matcher::SlamScanMatcher>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
