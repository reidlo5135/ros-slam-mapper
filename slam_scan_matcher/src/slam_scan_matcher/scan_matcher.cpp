#include "slam_scan_matcher/scan_matcher.hpp"

namespace slam::scan::matcher
{

ScanMatcher::ScanMatcher()
: use_imu_heading_(false),
  imu_heading_rotation_threshold_(0.25),
  imu_heading_blend_gain_(0.35),
  imu_heading_max_delta_deg_(4.0),
  mapping_min_range_(0.05),
  mapping_max_range_(8.0),
  scan_matching_linear_window_(0.15),
  scan_matching_linear_step_(0.05),
  scan_matching_angular_window_deg_(12.0),
  scan_matching_angular_step_deg_(3.0),
  scan_matching_max_beams_(32),
  scan_matching_min_valid_beams_(8),
  scan_matching_occupied_search_radius_cells_(1),
  scan_matching_distance_match_radius_cells_(4),
  scan_matching_minimum_occupied_cells_(50),
  scan_matching_occupied_match_score_(3.0),
  scan_matching_distance_match_score_(2.0),
  scan_matching_distance_penalty_per_cell_(0.45),
  scan_matching_free_space_penalty_(1.0),
  scan_matching_min_score_improvement_(2.0),
  scan_matching_max_translation_correction_(0.08),
  scan_matching_max_yaw_correction_deg_(6.0),
  scan_matching_translation_regularization_weight_(5.0),
  scan_matching_yaw_regularization_weight_(0.75),
  scan_matching_coarse_linear_step_multiplier_(3.0),
  scan_matching_coarse_angular_step_multiplier_(3.0),
  scan_matching_fine_window_scale_(0.5)
{
}

void ScanMatcher::declare_parameters(rclcpp_lifecycle::LifecycleNode &node) const
{
  node.declare_parameter("motion_prior.use_imu_heading", this->use_imu_heading_);
  node.declare_parameter(
    "motion_prior.imu_heading_rotation_threshold",
    this->imu_heading_rotation_threshold_);
  node.declare_parameter("motion_prior.imu_heading_blend_gain", this->imu_heading_blend_gain_);
  node.declare_parameter(
    "motion_prior.imu_heading_max_delta_deg",
    this->imu_heading_max_delta_deg_);
  node.declare_parameter("mapping.range.min", this->mapping_min_range_);
  node.declare_parameter("mapping.range.max", this->mapping_max_range_);
  node.declare_parameter("scan_matching.linear_window", this->scan_matching_linear_window_);
  node.declare_parameter("scan_matching.linear_step", this->scan_matching_linear_step_);
  node.declare_parameter(
    "scan_matching.angular_window_deg", this->scan_matching_angular_window_deg_);
  node.declare_parameter(
    "scan_matching.angular_step_deg", this->scan_matching_angular_step_deg_);
  node.declare_parameter("scan_matching.max_beams", this->scan_matching_max_beams_);
  node.declare_parameter(
    "scan_matching.min_valid_beams", this->scan_matching_min_valid_beams_);
  node.declare_parameter(
    "scan_matching.occupied_search_radius_cells",
    this->scan_matching_occupied_search_radius_cells_);
  node.declare_parameter(
    "scan_matching.distance_match_radius_cells",
    this->scan_matching_distance_match_radius_cells_);
  node.declare_parameter(
    "scan_matching.minimum_occupied_cells", this->scan_matching_minimum_occupied_cells_);
  node.declare_parameter(
    "scan_matching.occupied_match_score", this->scan_matching_occupied_match_score_);
  node.declare_parameter(
    "scan_matching.distance_match_score", this->scan_matching_distance_match_score_);
  node.declare_parameter(
    "scan_matching.distance_penalty_per_cell",
    this->scan_matching_distance_penalty_per_cell_);
  node.declare_parameter(
    "scan_matching.free_space_penalty", this->scan_matching_free_space_penalty_);
  node.declare_parameter(
    "scan_matching.min_score_improvement", this->scan_matching_min_score_improvement_);
  node.declare_parameter(
    "scan_matching.max_translation_correction", this->scan_matching_max_translation_correction_);
  node.declare_parameter(
    "scan_matching.max_yaw_correction_deg", this->scan_matching_max_yaw_correction_deg_);
  node.declare_parameter(
    "scan_matching.translation_regularization_weight",
    this->scan_matching_translation_regularization_weight_);
  node.declare_parameter(
    "scan_matching.yaw_regularization_weight",
    this->scan_matching_yaw_regularization_weight_);
  node.declare_parameter(
    "scan_matching.coarse_linear_step_multiplier",
    this->scan_matching_coarse_linear_step_multiplier_);
  node.declare_parameter(
    "scan_matching.coarse_angular_step_multiplier",
    this->scan_matching_coarse_angular_step_multiplier_);
  node.declare_parameter(
    "scan_matching.fine_window_scale",
    this->scan_matching_fine_window_scale_);
}

void ScanMatcher::load_parameters(rclcpp_lifecycle::LifecycleNode &node)
{
  node.get_parameter("motion_prior.use_imu_heading", this->use_imu_heading_);
  node.get_parameter(
    "motion_prior.imu_heading_rotation_threshold",
    this->imu_heading_rotation_threshold_);
  node.get_parameter("motion_prior.imu_heading_blend_gain", this->imu_heading_blend_gain_);
  node.get_parameter(
    "motion_prior.imu_heading_max_delta_deg",
    this->imu_heading_max_delta_deg_);
  node.get_parameter("mapping.range.min", this->mapping_min_range_);
  node.get_parameter("mapping.range.max", this->mapping_max_range_);
  node.get_parameter("scan_matching.linear_window", this->scan_matching_linear_window_);
  node.get_parameter("scan_matching.linear_step", this->scan_matching_linear_step_);
  node.get_parameter(
    "scan_matching.angular_window_deg", this->scan_matching_angular_window_deg_);
  node.get_parameter(
    "scan_matching.angular_step_deg", this->scan_matching_angular_step_deg_);
  node.get_parameter("scan_matching.max_beams", this->scan_matching_max_beams_);
  node.get_parameter(
    "scan_matching.min_valid_beams", this->scan_matching_min_valid_beams_);
  node.get_parameter(
    "scan_matching.occupied_search_radius_cells",
    this->scan_matching_occupied_search_radius_cells_);
  node.get_parameter(
    "scan_matching.distance_match_radius_cells",
    this->scan_matching_distance_match_radius_cells_);
  node.get_parameter(
    "scan_matching.minimum_occupied_cells", this->scan_matching_minimum_occupied_cells_);
  node.get_parameter(
    "scan_matching.occupied_match_score", this->scan_matching_occupied_match_score_);
  node.get_parameter(
    "scan_matching.distance_match_score", this->scan_matching_distance_match_score_);
  node.get_parameter(
    "scan_matching.distance_penalty_per_cell",
    this->scan_matching_distance_penalty_per_cell_);
  node.get_parameter(
    "scan_matching.free_space_penalty", this->scan_matching_free_space_penalty_);
  node.get_parameter(
    "scan_matching.min_score_improvement", this->scan_matching_min_score_improvement_);
  node.get_parameter(
    "scan_matching.max_translation_correction", this->scan_matching_max_translation_correction_);
  node.get_parameter(
    "scan_matching.max_yaw_correction_deg", this->scan_matching_max_yaw_correction_deg_);
  node.get_parameter(
    "scan_matching.translation_regularization_weight",
    this->scan_matching_translation_regularization_weight_);
  node.get_parameter(
    "scan_matching.yaw_regularization_weight",
    this->scan_matching_yaw_regularization_weight_);
  node.get_parameter(
    "scan_matching.coarse_linear_step_multiplier",
    this->scan_matching_coarse_linear_step_multiplier_);
  node.get_parameter(
    "scan_matching.coarse_angular_step_multiplier",
    this->scan_matching_coarse_angular_step_multiplier_);
  node.get_parameter(
    "scan_matching.fine_window_scale",
    this->scan_matching_fine_window_scale_);
}

Pose2D ScanMatcher::build_raw_odom_pose(
  const nav_msgs::msg::Odometry &odometry,
  const MotionPriorState &motion_prior_state) const
{
  Pose2D pose{};
  pose.x = odometry.pose.pose.position.x;
  pose.y = odometry.pose.pose.position.y;
  pose.yaw = this->quaternion_to_yaw(odometry.pose.pose.orientation);

  if (
    this->use_imu_heading_ &&
    motion_prior_state.has_latest_imu &&
    motion_prior_state.has_start_imu_yaw &&
    motion_prior_state.has_start_odom_yaw)
  {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double angular_velocity = std::abs(odometry.twist.twist.angular.z);
    if (angular_velocity >= this->imu_heading_rotation_threshold_) {
      const double imu_yaw = this->normalize_angle(
        motion_prior_state.start_odom_yaw +
        (motion_prior_state.latest_imu_yaw - motion_prior_state.start_imu_yaw));
      const double max_delta =
        std::max(0.0, this->imu_heading_max_delta_deg_) * kDegToRad;
      double yaw_delta = this->normalize_angle(imu_yaw - pose.yaw);
      yaw_delta = std::clamp(yaw_delta, -max_delta, max_delta);
      const double blend_gain = std::clamp(this->imu_heading_blend_gain_, 0.0, 1.0);
      pose.yaw = this->normalize_angle(pose.yaw + (yaw_delta * blend_gain));
    }
  }

  return pose;
}

ScanMatchResult ScanMatcher::refine_pose_with_scan_matching_detailed(
  const nav_msgs::msg::OccupancyGrid &map,
  const sensor_msgs::msg::LaserScan &scan,
  const Pose2D &predicted_pose) const
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  ScanMatchResult result{};
  result.pose = predicted_pose;

  int occupied_cell_count = 0;
  for (const auto cell : map.data) {
    if (cell >= 50) {
      ++occupied_cell_count;
    }
  }
  result.debug.occupied_cell_count = occupied_cell_count;
  if (occupied_cell_count < this->scan_matching_minimum_occupied_cells_) {
    result.debug.reject_reason = ScanMatchRejectReason::insufficient_map;
    return result;
  }

  const double linear_window = std::max(0.0, this->scan_matching_linear_window_);
  const double linear_step = std::max(0.01, this->scan_matching_linear_step_);
  const double angular_window_rad =
    std::max(0.0, this->scan_matching_angular_window_deg_) * kDegToRad;
  const double angular_step_rad =
    std::max(1.0, this->scan_matching_angular_step_deg_) * kDegToRad;
  const double max_translation_correction =
    std::max(0.0, this->scan_matching_max_translation_correction_);
  const double max_yaw_correction_rad =
    std::max(0.0, this->scan_matching_max_yaw_correction_deg_) * kDegToRad;
  const double min_score_improvement =
    std::max(0.0, this->scan_matching_min_score_improvement_);
  const double translation_regularization_weight =
    std::max(0.0, this->scan_matching_translation_regularization_weight_);
  const double yaw_regularization_weight =
    std::max(0.0, this->scan_matching_yaw_regularization_weight_);
  const double coarse_linear_step = std::max(
    linear_step,
    linear_step * std::max(1.0, this->scan_matching_coarse_linear_step_multiplier_));
  const double coarse_angular_step_rad = std::max(
    angular_step_rad,
    angular_step_rad * std::max(1.0, this->scan_matching_coarse_angular_step_multiplier_));
  const double fine_window_scale = std::clamp(this->scan_matching_fine_window_scale_, 0.1, 1.0);
  const double fine_linear_window = std::max(linear_step, linear_window * fine_window_scale);
  const double fine_angular_window_rad = std::max(
    angular_step_rad,
    angular_window_rad * fine_window_scale);

  const CandidateScore predicted_candidate =
    this->evaluate_candidate_score(map, scan, predicted_pose);
  const double predicted_score = predicted_candidate.score;
  result.debug.predicted_score = predicted_score;
  result.debug.predicted_valid_beam_count = predicted_candidate.valid_beam_count;
  CandidateSearchResult coarse_result = this->search_best_pose_in_window(
    map,
    scan,
    predicted_pose,
    linear_window,
    coarse_linear_step,
    angular_window_rad,
    coarse_angular_step_rad,
    translation_regularization_weight,
    yaw_regularization_weight);
  CandidateSearchResult fine_result = this->search_best_pose_in_window(
    map,
    scan,
    coarse_result.pose,
    fine_linear_window,
    linear_step,
    fine_angular_window_rad,
    angular_step_rad,
    translation_regularization_weight,
    yaw_regularization_weight);
  result.debug.coarse_score = coarse_result.score;
  result.debug.coarse_valid_beam_count = coarse_result.valid_beam_count;
  result.debug.fine_score = fine_result.score;
  result.debug.fine_valid_beam_count = fine_result.valid_beam_count;

  Pose2D best_pose = fine_result.pose;
  double best_score = fine_result.score;
  result.debug.final_score = best_score;

  const double correction_x = best_pose.x - predicted_pose.x;
  const double correction_y = best_pose.y - predicted_pose.y;
  const double correction_translation = std::hypot(correction_x, correction_y);
  const double correction_yaw =
    std::abs(this->normalize_angle(best_pose.yaw - predicted_pose.yaw));
  result.debug.score_improvement = best_score - predicted_score;
  result.debug.correction_translation = correction_translation;
  result.debug.correction_yaw_deg = correction_yaw / kDegToRad;

  if (
    !std::isfinite(best_score) ||
    (best_score - predicted_score) < min_score_improvement ||
    correction_translation > max_translation_correction ||
    correction_yaw > max_yaw_correction_rad)
  {
    if (!std::isfinite(best_score)) {
      result.debug.reject_reason = ScanMatchRejectReason::non_finite_score;
    } else if ((best_score - predicted_score) < min_score_improvement) {
      result.debug.reject_reason = ScanMatchRejectReason::insufficient_score_improvement;
    } else if (correction_translation > max_translation_correction) {
      result.debug.reject_reason = ScanMatchRejectReason::translation_limit;
    } else if (correction_yaw > max_yaw_correction_rad) {
      result.debug.reject_reason = ScanMatchRejectReason::yaw_limit;
    }
    return result;
  }

  result.pose = best_pose;
  result.debug.correction_applied = true;
  return result;
}

Pose2D ScanMatcher::refine_pose_with_scan_matching(
  const nav_msgs::msg::OccupancyGrid &map,
  const sensor_msgs::msg::LaserScan &scan,
  const Pose2D &predicted_pose) const
{
  const ScanMatchResult result =
    this->refine_pose_with_scan_matching_detailed(map, scan, predicted_pose);
  return result.pose;
}

ScanMatcher::CandidateSearchResult ScanMatcher::search_best_pose_in_window(
  const nav_msgs::msg::OccupancyGrid &map,
  const sensor_msgs::msg::LaserScan &scan,
  const Pose2D &center_pose,
  double linear_window,
  double linear_step,
  double angular_window_rad,
  double angular_step_rad,
  double translation_regularization_weight,
  double yaw_regularization_weight) const
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

  CandidateSearchResult result{};
  result.pose = center_pose;
  const CandidateScore center_candidate =
    this->evaluate_candidate_score(map, scan, center_pose);
  result.score = center_candidate.score;
  result.valid_beam_count = center_candidate.valid_beam_count;

  const double safe_linear_window = std::max(0.0, linear_window);
  const double safe_linear_step = std::max(0.01, linear_step);
  const double safe_angular_window = std::max(0.0, angular_window_rad);
  const double safe_angular_step = std::max((1.0 * kDegToRad), angular_step_rad);

  for (
    double delta_x = -safe_linear_window;
    delta_x <= safe_linear_window + 1e-6;
    delta_x += safe_linear_step)
  {
    for (
      double delta_y = -safe_linear_window;
      delta_y <= safe_linear_window + 1e-6;
      delta_y += safe_linear_step)
    {
      for (
        double delta_yaw = -safe_angular_window;
        delta_yaw <= safe_angular_window + 1e-6;
        delta_yaw += safe_angular_step)
      {
        if (
          std::abs(delta_x) < 1e-9 &&
          std::abs(delta_y) < 1e-9 &&
          std::abs(delta_yaw) < 1e-9)
        {
          continue;
        }

        Pose2D candidate_pose{};
        candidate_pose.x = center_pose.x + delta_x;
        candidate_pose.y = center_pose.y + delta_y;
        candidate_pose.yaw = this->normalize_angle(center_pose.yaw + delta_yaw);

        const CandidateScore candidate =
          this->evaluate_candidate_score(map, scan, candidate_pose);
        if (!std::isfinite(candidate.score)) {
          continue;
        }

        const double correction_translation = std::hypot(
          candidate_pose.x - center_pose.x,
          candidate_pose.y - center_pose.y);
        const double correction_yaw_deg =
          std::abs(this->normalize_angle(candidate_pose.yaw - center_pose.yaw)) / kDegToRad;
        const double regularized_score =
          candidate.score -
          (translation_regularization_weight * correction_translation) -
          (yaw_regularization_weight * correction_yaw_deg);
        if (regularized_score > result.score) {
          result.score = regularized_score;
          result.pose = candidate_pose;
          result.valid_beam_count = candidate.valid_beam_count;
        }
      }
    }
  }

  return result;
}

ScanMatcher::CandidateScore ScanMatcher::evaluate_candidate_score(
  const nav_msgs::msg::OccupancyGrid &map,
  const sensor_msgs::msg::LaserScan &scan,
  const Pose2D &candidate_pose) const
{
  CandidateScore result{};
  if (scan.ranges.empty()) {
    return result;
  }

  const int beam_stride = std::max(
    1,
    static_cast<int>(scan.ranges.size()) / std::max(1, this->scan_matching_max_beams_));
  double score = 0.0;

  for (std::size_t index = 0; index < scan.ranges.size(); index += static_cast<std::size_t>(beam_stride)) {
    const double range = static_cast<double>(scan.ranges[index]);
    if (
      !std::isfinite(range) ||
      range < std::max(this->mapping_min_range_, static_cast<double>(scan.range_min)) ||
      range > std::min(this->mapping_max_range_, static_cast<double>(scan.range_max)))
    {
      continue;
    }

    const double beam_angle = candidate_pose.yaw + static_cast<double>(scan.angle_min) +
      (static_cast<double>(index) * static_cast<double>(scan.angle_increment));
    const double end_x_world = candidate_pose.x + (range * std::cos(beam_angle));
    const double end_y_world = candidate_pose.y + (range * std::sin(beam_angle));

    int grid_x = 0;
    int grid_y = 0;
    if (!this->world_to_grid(map, end_x_world, end_y_world, grid_x, grid_y)) {
      continue;
    }

    std::size_t cell_index = 0U;
    if (!this->grid_index(map, grid_x, grid_y, cell_index) || cell_index >= map.data.size()) {
      continue;
    }

    ++result.valid_beam_count;
    const int8_t cell_value = map.data[cell_index];
    if (cell_value >= 50) {
      const double occupied_confidence = std::clamp(
        (static_cast<double>(cell_value) - 50.0) / 50.0,
        0.0,
        1.0);
      score += this->scan_matching_occupied_match_score_ * (1.0 + (0.10 * occupied_confidence));
      continue;
    }

    const double nearest_distance_cells = this->nearest_occupied_distance_cells(
      map,
      grid_x,
      grid_y,
      this->scan_matching_distance_match_radius_cells_);
    if (std::isfinite(nearest_distance_cells)) {
      const double search_radius_cells = std::max(
        1.0,
        static_cast<double>(this->scan_matching_distance_match_radius_cells_));
      const double normalized_distance = std::clamp(
        nearest_distance_cells / search_radius_cells,
        0.0,
        1.0);
      const double closeness_score = 1.0 - normalized_distance;
      const double proximity_score =
        (this->scan_matching_distance_match_score_ * closeness_score) -
        (this->scan_matching_distance_penalty_per_cell_ * nearest_distance_cells);
      if (proximity_score > 0.0) {
        score += proximity_score;
        continue;
      }
    }

    if (
      this->has_nearby_occupied_cell(
        map,
        grid_x,
        grid_y,
        this->scan_matching_occupied_search_radius_cells_))
    {
      score += this->scan_matching_occupied_match_score_ * 0.5;
    } else if (cell_value == 0) {
      score -= this->scan_matching_free_space_penalty_;
    }
  }

  if (result.valid_beam_count < this->scan_matching_min_valid_beams_) {
    result.score = -std::numeric_limits<double>::infinity();
    return result;
  }

  result.score = score;
  return result;
}

double ScanMatcher::score_scan_candidate(
  const nav_msgs::msg::OccupancyGrid &map,
  const sensor_msgs::msg::LaserScan &scan,
  const Pose2D &candidate_pose) const
{
  const CandidateScore result = this->evaluate_candidate_score(map, scan, candidate_pose);
  return result.score;
}

double ScanMatcher::nearest_occupied_distance_cells(
  const nav_msgs::msg::OccupancyGrid &map,
  int grid_x,
  int grid_y,
  int radius_cells) const
{
  const int radius = std::max(0, radius_cells);
  double best_distance = std::numeric_limits<double>::infinity();

  for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
    for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
      std::size_t cell_index = 0U;
      if (!this->grid_index(map, grid_x + offset_x, grid_y + offset_y, cell_index)) {
        continue;
      }
      if (cell_index >= map.data.size() || map.data[cell_index] < 50) {
        continue;
      }

      const double distance = std::hypot(
        static_cast<double>(offset_x),
        static_cast<double>(offset_y));
      if (distance < best_distance) {
        best_distance = distance;
      }
    }
  }

  return best_distance;
}

bool ScanMatcher::has_nearby_occupied_cell(
  const nav_msgs::msg::OccupancyGrid &map,
  int grid_x,
  int grid_y,
  int radius_cells) const
{
  const int radius = std::max(0, radius_cells);
  for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
    for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
      std::size_t cell_index = 0U;
      if (!this->grid_index(map, grid_x + offset_x, grid_y + offset_y, cell_index)) {
        continue;
      }
      if (cell_index < map.data.size() && map.data[cell_index] >= 50) {
        return true;
      }
    }
  }

  return false;
}

Pose2D ScanMatcher::compose_pose(const Pose2D &lhs, const Pose2D &rhs) const
{
  Pose2D composed{};
  composed.x = lhs.x + (std::cos(lhs.yaw) * rhs.x) - (std::sin(lhs.yaw) * rhs.y);
  composed.y = lhs.y + (std::sin(lhs.yaw) * rhs.x) + (std::cos(lhs.yaw) * rhs.y);
  composed.yaw = this->normalize_angle(lhs.yaw + rhs.yaw);
  return composed;
}

Pose2D ScanMatcher::inverse_pose(const Pose2D &pose) const
{
  Pose2D inverse{};
  inverse.yaw = this->normalize_angle(-pose.yaw);
  inverse.x = -(std::cos(inverse.yaw) * pose.x - std::sin(inverse.yaw) * pose.y);
  inverse.y = -(std::sin(inverse.yaw) * pose.x + std::cos(inverse.yaw) * pose.y);
  return inverse;
}

Pose2D ScanMatcher::relative_pose(const Pose2D &from, const Pose2D &to) const
{
  return this->compose_pose(this->inverse_pose(from), to);
}

bool ScanMatcher::world_to_grid(
  const nav_msgs::msg::OccupancyGrid &map,
  double x,
  double y,
  int &grid_x,
  int &grid_y) const
{
  grid_x = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  grid_y = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return (
    grid_x >= 0 &&
    grid_x < static_cast<int>(map.info.width) &&
    grid_y >= 0 &&
    grid_y < static_cast<int>(map.info.height));
}

bool ScanMatcher::grid_index(
  const nav_msgs::msg::OccupancyGrid &map,
  int grid_x,
  int grid_y,
  std::size_t &index) const
{
  if (
    grid_x < 0 ||
    grid_y < 0 ||
    grid_x >= static_cast<int>(map.info.width) ||
    grid_y >= static_cast<int>(map.info.height))
  {
    return false;
  }

  index = static_cast<std::size_t>((grid_y * static_cast<int>(map.info.width)) + grid_x);
  return true;
}

void ScanMatcher::set_quaternion_from_yaw(
  geometry_msgs::msg::Quaternion &orientation,
  double yaw) const
{
  const double half_yaw = yaw * 0.5;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = std::sin(half_yaw);
  orientation.w = std::cos(half_yaw);
}

double ScanMatcher::quaternion_to_yaw(const geometry_msgs::msg::Quaternion &orientation) const
{
  const double siny_cosp = 2.0 * (
    (orientation.w * orientation.z) + (orientation.x * orientation.y));
  const double cosy_cosp = 1.0 - 2.0 * (
    (orientation.y * orientation.y) + (orientation.z * orientation.z));
  return std::atan2(siny_cosp, cosy_cosp);
}

double ScanMatcher::normalize_angle(double angle) const
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

const char *ScanMatcher::scan_match_reject_reason_to_cstr(ScanMatchRejectReason reason) const
{
  switch (reason) {
    case ScanMatchRejectReason::none:
      return "none";
    case ScanMatchRejectReason::insufficient_map:
      return "insufficient_map";
    case ScanMatchRejectReason::non_finite_score:
      return "non_finite_score";
    case ScanMatchRejectReason::insufficient_score_improvement:
      return "insufficient_score_improvement";
    case ScanMatchRejectReason::translation_limit:
      return "translation_limit";
    case ScanMatchRejectReason::yaw_limit:
      return "yaw_limit";
    default:
      return "unknown";
  }
}

}  // namespace slam::scan::matcher
