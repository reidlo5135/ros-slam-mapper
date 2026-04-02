#ifndef SLAM_SCAN_MATCHER__SCAN_MATCHER_HPP_
#define SLAM_SCAN_MATCHER__SCAN_MATCHER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace slam::scan::matcher
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct MotionPriorState
{
  double start_odom_yaw{0.0};
  double start_imu_yaw{0.0};
  double latest_imu_yaw{0.0};
  bool has_latest_imu{false};
  bool has_start_odom_yaw{false};
  bool has_start_imu_yaw{false};
};

class ScanMatcher
{
private:
  bool use_imu_heading_;
  double imu_heading_rotation_threshold_;
  double imu_heading_blend_gain_;
  double imu_heading_max_delta_deg_;

  double mapping_min_range_;
  double mapping_max_range_;

  double scan_matching_linear_window_;
  double scan_matching_linear_step_;
  double scan_matching_angular_window_deg_;
  double scan_matching_angular_step_deg_;
  int scan_matching_max_beams_;
  int scan_matching_min_valid_beams_;
  int scan_matching_occupied_search_radius_cells_;
  int scan_matching_distance_match_radius_cells_;
  int scan_matching_minimum_occupied_cells_;
  double scan_matching_occupied_match_score_;
  double scan_matching_distance_match_score_;
  double scan_matching_distance_penalty_per_cell_;
  double scan_matching_free_space_penalty_;
  double scan_matching_min_score_improvement_;
  double scan_matching_max_translation_correction_;
  double scan_matching_max_yaw_correction_deg_;
  double scan_matching_translation_regularization_weight_;
  double scan_matching_yaw_regularization_weight_;

  double nearest_occupied_distance_cells(
    const nav_msgs::msg::OccupancyGrid &map,
    int grid_x,
    int grid_y,
    int radius_cells) const;
  bool has_nearby_occupied_cell(
    const nav_msgs::msg::OccupancyGrid &map,
    int grid_x,
    int grid_y,
    int radius_cells) const;
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

protected:
public:
  ScanMatcher();
  virtual ~ScanMatcher() = default;

  void declare_parameters(rclcpp_lifecycle::LifecycleNode &node) const;
  void load_parameters(rclcpp_lifecycle::LifecycleNode &node);

  Pose2D build_raw_odom_pose(
    const nav_msgs::msg::Odometry &odometry,
    const MotionPriorState &motion_prior_state) const;
  Pose2D refine_pose_with_scan_matching(
    const nav_msgs::msg::OccupancyGrid &map,
    const sensor_msgs::msg::LaserScan &scan,
    const Pose2D &predicted_pose) const;
  double score_scan_candidate(
    const nav_msgs::msg::OccupancyGrid &map,
    const sensor_msgs::msg::LaserScan &scan,
    const Pose2D &candidate_pose) const;

  Pose2D compose_pose(const Pose2D &lhs, const Pose2D &rhs) const;
  Pose2D inverse_pose(const Pose2D &pose) const;
  Pose2D relative_pose(const Pose2D &from, const Pose2D &to) const;
  void set_quaternion_from_yaw(geometry_msgs::msg::Quaternion &orientation, double yaw) const;
  double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &orientation) const;
  double normalize_angle(double angle) const;
};

}  // namespace slam::scan::matcher

#endif  // SLAM_SCAN_MATCHER__SCAN_MATCHER_HPP_
