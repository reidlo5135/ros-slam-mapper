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
  double resolution = 0.0;
  int width = 0;
  int height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  int hit_score = 0;
  int free_score = 0;
  int occupied_score_threshold = 0;
  int min_occupied_neighbor_count = 0;
  int min_free_neighbor_count = 0;
  this->get_parameter("mapping.resolution", resolution);
  this->get_parameter("mapping.width", width);
  this->get_parameter("mapping.height", height);
  this->get_parameter("mapping.origin.x", origin_x);
  this->get_parameter("mapping.origin.y", origin_y);
  this->get_parameter("occupancy.hit_score", hit_score);
  this->get_parameter("occupancy.free_score", free_score);
  this->get_parameter(
    "occupancy.occupied_score_threshold",
    occupied_score_threshold);
  this->get_parameter(
    "refinement.min_occupied_neighbor_count",
    min_occupied_neighbor_count);
  this->get_parameter(
    "refinement.min_free_neighbor_count",
    min_free_neighbor_count);
  RCLCPP_INFO(
    this->get_logger(),
    "Configured SLAM submap server utility node with frame='%s'",
    this->frame_id_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Submap params resolution=%.3f m, size=%dx%d, origin=(%.3f, %.3f)",
    resolution,
    width,
    height,
    origin_x,
    origin_y);
  RCLCPP_INFO(
    this->get_logger(),
    "Occupancy params hit=%d, free=%d, occupied_threshold=%d, refine_neighbors occupied=%d free=%d",
    hit_score,
    free_score,
    occupied_score_threshold,
    min_occupied_neighbor_count,
    min_free_neighbor_count);
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
  RCLCPP_INFO(this->get_logger(), "Deactivated SLAM submap server utility node");
  return CallbackReturn::SUCCESS;
}

SlamSubmapServer::CallbackReturn SlamSubmapServer::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "Cleaned up SLAM submap server utility node");
  return CallbackReturn::SUCCESS;
}

SlamSubmapServer::CallbackReturn SlamSubmapServer::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return this->on_cleanup(state);
}

}  // namespace slam::submap::server
