#ifndef SLAM_PGRAPH_SERVER__POSE_GRAPH_SERVER_HPP_
#define SLAM_PGRAPH_SERVER__POSE_GRAPH_SERVER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "slam_scan_matcher/scan_matcher.hpp"

namespace slam::graph::server
{

using slam::scan::matcher::Pose2D;

struct GraphNode
{
  int id{0};
  Pose2D map_pose{};
  Pose2D raw_odom_pose{};
  sensor_msgs::msg::LaserScan scan{};
  std::vector<float> descriptor;
  int submap_id{0};
};

struct GraphEdge
{
  int from{0};
  int to{0};
  Pose2D relative_pose{};
  double weight{1.0};
  bool loop_closure{false};
};

struct Submap
{
  int id{0};
  int start_node_index{0};
  int end_node_index{0};
  Pose2D anchor_pose{};
  int keyframe_count{0};
};

struct LoopClosureCandidate
{
  bool found{false};
  int node_index{-1};
  Pose2D matched_pose{};
  double descriptor_distance{0.0};
  double match_score{0.0};
};

struct PoseGraphUpdate
{
  bool node_added{false};
  bool loop_accepted{false};
  bool graph_rebuild_needed{false};
  Pose2D latest_pose{};
};

class PoseGraphServer
{
private:
  std::vector<GraphNode> graph_nodes_;
  std::vector<GraphEdge> graph_edges_;
  std::vector<Submap> submaps_;
  int next_graph_node_id_;
  int next_submap_id_;

  double keyframe_distance_threshold_;
  double keyframe_yaw_threshold_;
  double rotation_only_distance_threshold_;
  double rotation_only_yaw_threshold_;
  double rotation_only_angular_velocity_threshold_;
  int submap_nodes_per_submap_;
  int loop_closure_min_node_separation_;
  double loop_closure_descriptor_threshold_;
  double loop_closure_acceptance_score_;
  double loop_closure_search_linear_window_;
  double loop_closure_search_linear_step_;
  double loop_closure_search_angular_window_deg_;
  double loop_closure_search_angular_step_deg_;
  double odom_edge_weight_;
  double loop_edge_weight_;
  int graph_optimization_iterations_;
  double graph_optimization_step_size_;
  double graph_pose_prior_translation_weight_;
  double graph_pose_prior_yaw_weight_;
  int descriptor_beams_;
  double mapping_min_range_;
  double mapping_max_range_;

  std::vector<float> build_scan_descriptor(const sensor_msgs::msg::LaserScan &scan) const;
  double compute_descriptor_distance(
    const std::vector<float> &lhs,
    const std::vector<float> &rhs) const;
  LoopClosureCandidate search_loop_closure_candidate(
    const sensor_msgs::msg::LaserScan &scan,
    const std::vector<float> &descriptor,
    const nav_msgs::msg::OccupancyGrid &map,
    const slam::scan::matcher::ScanMatcher &scan_matcher) const;
  void optimize_pose_graph();
  void update_submap_accumulation(const GraphNode &node);
  Pose2D compose_pose(const Pose2D &lhs, const Pose2D &rhs) const;
  Pose2D inverse_pose(const Pose2D &pose) const;
  Pose2D relative_pose(const Pose2D &from, const Pose2D &to) const;
  double normalize_angle(double angle) const;

protected:
public:
  PoseGraphServer();
  virtual ~PoseGraphServer() = default;

  void declare_parameters(rclcpp_lifecycle::LifecycleNode &node) const;
  void load_parameters(rclcpp_lifecycle::LifecycleNode &node);
  void reset();
  PoseGraphUpdate process_scan(
    const sensor_msgs::msg::LaserScan &scan,
    const Pose2D &corrected_pose,
    const Pose2D &raw_odom_pose,
    double angular_velocity,
    const nav_msgs::msg::OccupancyGrid &map,
    const slam::scan::matcher::ScanMatcher &scan_matcher);
  std::string build_graph_debug_json(const std::string &frame_id) const;
  const std::vector<GraphNode> &graph_nodes() const;
};

}  // namespace slam::graph::server

#endif  // SLAM_PGRAPH_SERVER__POSE_GRAPH_SERVER_HPP_
