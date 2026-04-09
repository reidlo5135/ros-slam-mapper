#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "slam_laser_filter/slam_laser_filter.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<slam::laser::filter::SlamLaserFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
