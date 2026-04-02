#include "slam_pgraph_server/slam_pgraph_server.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<slam::graph::server::SlamPGraphServer> node =
    std::make_shared<slam::graph::server::SlamPGraphServer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
