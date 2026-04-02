#include "slam_submap_server/slam_submap_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<slam::submap::server::SlamSubmapServer> node =
    std::make_shared<slam::submap::server::SlamSubmapServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
