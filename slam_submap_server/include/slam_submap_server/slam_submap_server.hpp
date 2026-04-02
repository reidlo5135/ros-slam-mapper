#ifndef SLAM_SUBMAP_SERVER__SLAM_SUBMAP_SERVER_HPP_
#define SLAM_SUBMAP_SERVER__SLAM_SUBMAP_SERVER_HPP_

#include <string>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "slam_submap_server/submap_server.hpp"

namespace slam::submap::server
{

class SlamSubmapServer : public rclcpp_lifecycle::LifecycleNode
{
private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  SubmapServer submap_server_;
  std::string frame_id_;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

protected:
public:
  explicit SlamSubmapServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~SlamSubmapServer() = default;
};

}  // namespace slam::submap::server

#endif  // SLAM_SUBMAP_SERVER__SLAM_SUBMAP_SERVER_HPP_
