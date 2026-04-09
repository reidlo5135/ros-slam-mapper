#ifndef SLAM_SUBMAP_SERVER__SUBMAP_SERVER_HPP_
#define SLAM_SUBMAP_SERVER__SUBMAP_SERVER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "slam_scan_matcher/scan_matcher.hpp"

namespace slam::submap::server
{

using slam::scan::matcher::Pose2D;

struct SubmapNode
{
  Pose2D map_pose{};
  sensor_msgs::msg::LaserScan scan{};
};

class SubmapServer
{
private:
  nav_msgs::msg::OccupancyGrid temporary_map_;
  nav_msgs::msg::OccupancyGrid refined_temporary_map_;
  std::vector<int16_t> occupancy_scores_;
  mutable std::mutex map_mutex_;

  std::string frame_id_;
  double mapping_resolution_;
  int mapping_width_;
  int mapping_height_;
  double mapping_origin_x_;
  double mapping_origin_y_;
  double mapping_origin_yaw_;
  double mapping_min_range_;
  double mapping_max_range_;
  int mapping_hit_score_;
  int mapping_free_score_;
  int mapping_occupied_score_threshold_;
  int mapping_free_score_threshold_;
  int mapping_score_min_;
  int mapping_score_max_;
  int refinement_min_occupied_neighbor_count_;
  int refinement_min_free_neighbor_count_;
  bool refinement_bridge_one_cell_gaps_;

  nav_msgs::msg::OccupancyGrid create_empty_map(const rclcpp::Time &stamp) const;
  std::vector<int16_t> create_empty_scores() const;
  void initialize_mapping_map(const rclcpp::Time &stamp);
  nav_msgs::msg::OccupancyGrid build_refined_map(
    const nav_msgs::msg::OccupancyGrid &source_map) const;
  void bridge_line_gaps(nav_msgs::msg::OccupancyGrid &map) const;
  int count_neighboring_cells(
    const nav_msgs::msg::OccupancyGrid &map,
    int grid_x,
    int grid_y,
    int minimum_value,
    int maximum_value) const;
  void integrate_scan_into_map(
    const sensor_msgs::msg::LaserScan &scan,
    const Pose2D &corrected_pose,
    nav_msgs::msg::OccupancyGrid &map,
    std::vector<int16_t> &occupancy_scores) const;
  bool world_to_grid(
    const nav_msgs::msg::OccupancyGrid &map,
    double x,
    double y,
    int &grid_x,
    int &grid_y) const;
  bool grid_index(
    const nav_msgs::msg::OccupancyGrid &map,
    int grid_x,
    int grid_y,
    std::size_t &index) const;
  void update_cell_score(
    nav_msgs::msg::OccupancyGrid &map,
    std::vector<int16_t> &occupancy_scores,
    int grid_x,
    int grid_y,
    int delta) const;
  void refresh_cell_from_score(
    nav_msgs::msg::OccupancyGrid &map,
    const std::vector<int16_t> &occupancy_scores,
    std::size_t index) const;
  void raytrace_free_cells(
    nav_msgs::msg::OccupancyGrid &map,
    std::vector<int16_t> &occupancy_scores,
    int start_x,
    int start_y,
    int end_x,
    int end_y) const;
  void mark_free_cell(
    nav_msgs::msg::OccupancyGrid &map,
    std::vector<int16_t> &occupancy_scores,
    int grid_x,
    int grid_y) const;
  void mark_occupied_cell(
    nav_msgs::msg::OccupancyGrid &map,
    std::vector<int16_t> &occupancy_scores,
    int grid_x,
    int grid_y) const;

protected:
public:
  SubmapServer();
  virtual ~SubmapServer() = default;

  void declare_parameters(rclcpp_lifecycle::LifecycleNode &node) const;
  void load_parameters(rclcpp_lifecycle::LifecycleNode &node);
  void reset(const std::string &frame_id, const rclcpp::Time &stamp);
  void integrate_scan(const sensor_msgs::msg::LaserScan &scan, const Pose2D &corrected_pose);
  void rebuild_map_from_pose_graph(const std::vector<SubmapNode> &graph_nodes, const rclcpp::Time &stamp);
  void refresh_refined_map(const rclcpp::Time &stamp);
  void refresh_refined_map_from_pose_graph(
    const std::vector<SubmapNode> &graph_nodes,
    const rclcpp::Time &stamp);
  nav_msgs::msg::OccupancyGrid raw_map() const;
  nav_msgs::msg::OccupancyGrid refined_map() const;
};

}  // namespace slam::submap::server

#endif  // SLAM_SUBMAP_SERVER__SUBMAP_SERVER_HPP_
