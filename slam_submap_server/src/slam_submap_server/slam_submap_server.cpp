#include "slam_submap_server/slam_submap_server.hpp"

namespace slam::submap::server
{

SlamSubmapServer::SlamSubmapServer(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("slam_submap_server", options),
  frame_id_("map")
{
  this->declare_parameter("frames.map", this->frame_id_);
  this->submap_server_.declare_parameters(*this);
}

SlamSubmapServer::CallbackReturn SlamSubmapServer::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("frames.map", this->frame_id_);
  this->submap_server_.load_parameters(*this);
  this->submap_server_.reset(this->frame_id_, this->now());
  RCLCPP_INFO(
    this->get_logger(),
    "Configured SLAM submap server utility node with frame='%s'",
    this->frame_id_.c_str());
  return CallbackReturn::SUCCESS;
}

SlamSubmapServer::CallbackReturn SlamSubmapServer::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Activated SLAM submap server utility node");
  return CallbackReturn::SUCCESS;
}

SlamSubmapServer::CallbackReturn SlamSubmapServer::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

SlamSubmapServer::CallbackReturn SlamSubmapServer::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

SlamSubmapServer::CallbackReturn SlamSubmapServer::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return this->on_cleanup(state);
}

}  // namespace slam::submap::server
