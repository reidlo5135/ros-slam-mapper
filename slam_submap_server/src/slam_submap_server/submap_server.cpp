#include "slam_submap_server/submap_server.hpp"

namespace slam::submap::server
{

SubmapServer::SubmapServer()
: frame_id_("map"),
  mapping_resolution_(0.05),
  mapping_width_(400),
  mapping_height_(400),
  mapping_origin_x_(-10.0),
  mapping_origin_y_(-10.0),
  mapping_origin_yaw_(0.0),
  mapping_min_range_(0.05),
  mapping_max_range_(8.0),
  mapping_hit_score_(20),
  mapping_free_score_(3),
  mapping_occupied_score_threshold_(20),
  mapping_free_score_threshold_(-5),
  mapping_score_min_(-20),
  mapping_score_max_(100),
  occupancy_endpoint_support_radius_cells_(1),
  occupancy_endpoint_support_score_(8),
  occupancy_endpoint_free_guard_cells_(1),
  refinement_min_occupied_neighbor_count_(2),
  refinement_min_free_neighbor_count_(4)
{
}

void SubmapServer::declare_parameters(rclcpp_lifecycle::LifecycleNode &node) const
{
  node.declare_parameter("mapping.resolution", this->mapping_resolution_);
  node.declare_parameter("mapping.width", this->mapping_width_);
  node.declare_parameter("mapping.height", this->mapping_height_);
  node.declare_parameter("mapping.origin.x", this->mapping_origin_x_);
  node.declare_parameter("mapping.origin.y", this->mapping_origin_y_);
  node.declare_parameter("mapping.origin.yaw", this->mapping_origin_yaw_);
  node.declare_parameter("occupancy.hit_score", this->mapping_hit_score_);
  node.declare_parameter("occupancy.free_score", this->mapping_free_score_);
  node.declare_parameter(
    "occupancy.occupied_score_threshold", this->mapping_occupied_score_threshold_);
  node.declare_parameter(
    "occupancy.free_score_threshold", this->mapping_free_score_threshold_);
  node.declare_parameter("occupancy.score_min", this->mapping_score_min_);
  node.declare_parameter("occupancy.score_max", this->mapping_score_max_);
  node.declare_parameter(
    "occupancy.endpoint_support_radius_cells", this->occupancy_endpoint_support_radius_cells_);
  node.declare_parameter(
    "occupancy.endpoint_support_score", this->occupancy_endpoint_support_score_);
  node.declare_parameter(
    "occupancy.endpoint_free_guard_cells", this->occupancy_endpoint_free_guard_cells_);
  node.declare_parameter(
    "refinement.min_occupied_neighbor_count", this->refinement_min_occupied_neighbor_count_);
  node.declare_parameter(
    "refinement.min_free_neighbor_count", this->refinement_min_free_neighbor_count_);
}

void SubmapServer::load_parameters(rclcpp_lifecycle::LifecycleNode &node)
{
  node.get_parameter("mapping.resolution", this->mapping_resolution_);
  node.get_parameter("mapping.width", this->mapping_width_);
  node.get_parameter("mapping.height", this->mapping_height_);
  node.get_parameter("mapping.origin.x", this->mapping_origin_x_);
  node.get_parameter("mapping.origin.y", this->mapping_origin_y_);
  node.get_parameter("mapping.origin.yaw", this->mapping_origin_yaw_);
  node.get_parameter("mapping.range.min", this->mapping_min_range_);
  node.get_parameter("mapping.range.max", this->mapping_max_range_);
  node.get_parameter("occupancy.hit_score", this->mapping_hit_score_);
  node.get_parameter("occupancy.free_score", this->mapping_free_score_);
  node.get_parameter(
    "occupancy.occupied_score_threshold", this->mapping_occupied_score_threshold_);
  node.get_parameter(
    "occupancy.free_score_threshold", this->mapping_free_score_threshold_);
  node.get_parameter("occupancy.score_min", this->mapping_score_min_);
  node.get_parameter("occupancy.score_max", this->mapping_score_max_);
  node.get_parameter(
    "occupancy.endpoint_support_radius_cells", this->occupancy_endpoint_support_radius_cells_);
  node.get_parameter(
    "occupancy.endpoint_support_score", this->occupancy_endpoint_support_score_);
  node.get_parameter(
    "occupancy.endpoint_free_guard_cells", this->occupancy_endpoint_free_guard_cells_);
  node.get_parameter(
    "refinement.min_occupied_neighbor_count", this->refinement_min_occupied_neighbor_count_);
  node.get_parameter(
    "refinement.min_free_neighbor_count", this->refinement_min_free_neighbor_count_);
}

void SubmapServer::reset(const std::string &frame_id, const rclcpp::Time &stamp)
{
  this->frame_id_ = frame_id;
  this->initialize_mapping_map(stamp);
}

nav_msgs::msg::OccupancyGrid SubmapServer::create_empty_map(const rclcpp::Time &stamp) const
{
  nav_msgs::msg::OccupancyGrid initialized_map;
  initialized_map.header.stamp = stamp;
  initialized_map.header.frame_id = this->frame_id_;
  initialized_map.info.map_load_time = initialized_map.header.stamp;
  initialized_map.info.resolution = static_cast<float>(this->mapping_resolution_);
  initialized_map.info.width = static_cast<uint32_t>(this->mapping_width_);
  initialized_map.info.height = static_cast<uint32_t>(this->mapping_height_);
  initialized_map.info.origin.position.x = this->mapping_origin_x_;
  initialized_map.info.origin.position.y = this->mapping_origin_y_;
  initialized_map.info.origin.position.z = 0.0;
  initialized_map.info.origin.orientation.x = 0.0;
  initialized_map.info.origin.orientation.y = 0.0;
  initialized_map.info.origin.orientation.z = std::sin(this->mapping_origin_yaw_ * 0.5);
  initialized_map.info.origin.orientation.w = std::cos(this->mapping_origin_yaw_ * 0.5);
  initialized_map.data.assign(
    static_cast<std::size_t>(this->mapping_width_ * this->mapping_height_),
    static_cast<int8_t>(-1));
  return initialized_map;
}

std::vector<int16_t> SubmapServer::create_empty_scores() const
{
  return std::vector<int16_t>(
    static_cast<std::size_t>(this->mapping_width_ * this->mapping_height_),
    0);
}

void SubmapServer::initialize_mapping_map(const rclcpp::Time &stamp)
{
  nav_msgs::msg::OccupancyGrid initialized_map = this->create_empty_map(stamp);

  std::scoped_lock<std::mutex> lock(this->map_mutex_);
  this->temporary_map_ = initialized_map;
  this->refined_temporary_map_ = this->temporary_map_;
  this->occupancy_scores_ = this->create_empty_scores();
}

void SubmapServer::integrate_scan(
  const sensor_msgs::msg::LaserScan &scan,
  const Pose2D &corrected_pose)
{
  std::scoped_lock<std::mutex> lock(this->map_mutex_);
  this->integrate_scan_into_map(scan, corrected_pose, this->temporary_map_, this->occupancy_scores_);
}

void SubmapServer::rebuild_map_from_pose_graph(
  const std::vector<SubmapNode> &graph_nodes,
  const rclcpp::Time &stamp)
{
  this->initialize_mapping_map(stamp);
  std::scoped_lock<std::mutex> lock(this->map_mutex_);
  for (const SubmapNode &node : graph_nodes) {
    this->integrate_scan_into_map(node.scan, node.map_pose, this->temporary_map_, this->occupancy_scores_);
  }
}

void SubmapServer::refresh_refined_map(const rclcpp::Time &stamp)
{
  nav_msgs::msg::OccupancyGrid raw_snapshot;
  {
    std::scoped_lock<std::mutex> lock(this->map_mutex_);
    raw_snapshot = this->temporary_map_;
  }

  nav_msgs::msg::OccupancyGrid refined_map = this->build_refined_map(raw_snapshot);
  refined_map.header.stamp = stamp;
  refined_map.info.map_load_time = refined_map.header.stamp;

  {
    std::scoped_lock<std::mutex> lock(this->map_mutex_);
    this->refined_temporary_map_ = refined_map;
  }
}

void SubmapServer::refresh_refined_map_from_pose_graph(
  const std::vector<SubmapNode> &graph_nodes,
  const rclcpp::Time &stamp)
{
  nav_msgs::msg::OccupancyGrid rendered_map = this->create_empty_map(stamp);
  std::vector<int16_t> rendered_scores = this->create_empty_scores();

  for (const SubmapNode &node : graph_nodes) {
    this->integrate_scan_into_map(node.scan, node.map_pose, rendered_map, rendered_scores);
  }

  nav_msgs::msg::OccupancyGrid refined_map = this->build_refined_map(rendered_map);
  refined_map.header.stamp = stamp;
  refined_map.info.map_load_time = refined_map.header.stamp;

  {
    std::scoped_lock<std::mutex> lock(this->map_mutex_);
    this->refined_temporary_map_ = refined_map;
  }
}

nav_msgs::msg::OccupancyGrid SubmapServer::raw_map() const
{
  nav_msgs::msg::OccupancyGrid map;
  std::scoped_lock<std::mutex> lock(this->map_mutex_);
  map = this->temporary_map_;
  return map;
}

nav_msgs::msg::OccupancyGrid SubmapServer::refined_map() const
{
  nav_msgs::msg::OccupancyGrid map;
  std::scoped_lock<std::mutex> lock(this->map_mutex_);
  map = this->refined_temporary_map_;
  return map;
}

nav_msgs::msg::OccupancyGrid SubmapServer::build_refined_map(
  const nav_msgs::msg::OccupancyGrid &source_map) const
{
  nav_msgs::msg::OccupancyGrid refined_map = source_map;
  if (source_map.data.empty()) {
    return refined_map;
  }

  const int width = static_cast<int>(source_map.info.width);
  const int height = static_cast<int>(source_map.info.height);
  for (int grid_y = 0; grid_y < height; ++grid_y) {
    for (int grid_x = 0; grid_x < width; ++grid_x) {
      std::size_t index = 0U;
      if (!this->grid_index(source_map, grid_x, grid_y, index) || index >= source_map.data.size()) {
        continue;
      }

      const int8_t cell_value = source_map.data[index];
      if (cell_value < 50) {
        continue;
      }

      const int occupied_neighbors =
        this->count_neighboring_cells(source_map, grid_x, grid_y, 50, 100);
      const int free_neighbors =
        this->count_neighboring_cells(source_map, grid_x, grid_y, 0, 49);

      if (
        occupied_neighbors < std::max(1, this->refinement_min_occupied_neighbor_count_) &&
        free_neighbors >= std::max(1, this->refinement_min_free_neighbor_count_))
      {
        refined_map.data[index] = -1;
      }
    }
  }

  return refined_map;
}

int SubmapServer::count_neighboring_cells(
  const nav_msgs::msg::OccupancyGrid &map,
  int grid_x,
  int grid_y,
  int minimum_value,
  int maximum_value) const
{
  int count = 0;
  for (int offset_y = -1; offset_y <= 1; ++offset_y) {
    for (int offset_x = -1; offset_x <= 1; ++offset_x) {
      if (offset_x == 0 && offset_y == 0) {
        continue;
      }

      std::size_t index = 0U;
      if (!this->grid_index(map, grid_x + offset_x, grid_y + offset_y, index) || index >= map.data.size()) {
        continue;
      }

      const int value = static_cast<int>(map.data[index]);
      if (value >= minimum_value && value <= maximum_value) {
        ++count;
      }
    }
  }

  return count;
}

void SubmapServer::integrate_scan_into_map(
  const sensor_msgs::msg::LaserScan &scan,
  const Pose2D &corrected_pose,
  nav_msgs::msg::OccupancyGrid &map,
  std::vector<int16_t> &occupancy_scores) const
{
  int start_x = 0;
  int start_y = 0;
  if (!this->world_to_grid(map, corrected_pose.x, corrected_pose.y, start_x, start_y)) {
    return;
  }

  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = static_cast<double>(scan.ranges[index]);
    if (!std::isfinite(range) || range < std::max(this->mapping_min_range_, static_cast<double>(scan.range_min))) {
      continue;
    }

    const double clipped_range = std::min(
      range,
      std::min(this->mapping_max_range_, static_cast<double>(scan.range_max)));
    const bool has_hit = range <= std::min(this->mapping_max_range_, static_cast<double>(scan.range_max));
    const double beam_angle = corrected_pose.yaw + static_cast<double>(scan.angle_min) +
      (static_cast<double>(index) * static_cast<double>(scan.angle_increment));
    const double end_x_world = corrected_pose.x + (clipped_range * std::cos(beam_angle));
    const double end_y_world = corrected_pose.y + (clipped_range * std::sin(beam_angle));

    int end_x = 0;
    int end_y = 0;
    if (!this->world_to_grid(map, end_x_world, end_y_world, end_x, end_y)) {
      continue;
    }

    this->raytrace_free_cells(
      map,
      occupancy_scores,
      start_x,
      start_y,
      end_x,
      end_y,
      std::max(0, this->occupancy_endpoint_free_guard_cells_));
    if (has_hit) {
      this->mark_occupied_cell(map, occupancy_scores, end_x, end_y);
      this->mark_occupied_endpoint_support(map, occupancy_scores, end_x, end_y);
    }
  }
}

bool SubmapServer::world_to_grid(
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

bool SubmapServer::grid_index(
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

void SubmapServer::update_cell_score(
  nav_msgs::msg::OccupancyGrid &map,
  std::vector<int16_t> &occupancy_scores,
  int grid_x,
  int grid_y,
  int delta) const
{
  std::size_t index = 0U;
  if (!this->grid_index(map, grid_x, grid_y, index) || index >= occupancy_scores.size()) {
    return;
  }

  const int updated_score = std::clamp(
    static_cast<int>(occupancy_scores[index]) + delta,
    this->mapping_score_min_,
    this->mapping_score_max_);
  occupancy_scores[index] = static_cast<int16_t>(updated_score);
  this->refresh_cell_from_score(map, occupancy_scores, index);
}

void SubmapServer::refresh_cell_from_score(
  nav_msgs::msg::OccupancyGrid &map,
  const std::vector<int16_t> &occupancy_scores,
  std::size_t index) const
{
  if (index >= map.data.size() || index >= occupancy_scores.size()) {
    return;
  }

  const int score = static_cast<int>(occupancy_scores[index]);
  if (score >= this->mapping_occupied_score_threshold_) {
    map.data[index] = 100;
  } else if (score <= this->mapping_free_score_threshold_) {
    map.data[index] = 0;
  } else {
    map.data[index] = -1;
  }
}

void SubmapServer::raytrace_free_cells(
  nav_msgs::msg::OccupancyGrid &map,
  std::vector<int16_t> &occupancy_scores,
  int start_x,
  int start_y,
  int end_x,
  int end_y,
  int endpoint_free_guard_cells) const
{
  int x = start_x;
  int y = start_y;
  const int delta_x = std::abs(end_x - start_x);
  const int delta_y = std::abs(end_y - start_y);
  const int step_x = (start_x < end_x) ? 1 : -1;
  const int step_y = (start_y < end_y) ? 1 : -1;
  int error = delta_x - delta_y;

  while (x != end_x || y != end_y) {
    const int distance_to_endpoint = std::max(std::abs(end_x - x), std::abs(end_y - y));
    if (distance_to_endpoint > endpoint_free_guard_cells) {
      this->mark_free_cell(map, occupancy_scores, x, y);
    }
    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      x += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      y += step_y;
    }
  }
}

void SubmapServer::mark_free_cell(
  nav_msgs::msg::OccupancyGrid &map,
  std::vector<int16_t> &occupancy_scores,
  int grid_x,
  int grid_y) const
{
  this->update_cell_score(map, occupancy_scores, grid_x, grid_y, -this->mapping_free_score_);
}

void SubmapServer::mark_occupied_cell(
  nav_msgs::msg::OccupancyGrid &map,
  std::vector<int16_t> &occupancy_scores,
  int grid_x,
  int grid_y) const
{
  this->update_cell_score(map, occupancy_scores, grid_x, grid_y, this->mapping_hit_score_);
}

void SubmapServer::mark_occupied_endpoint_support(
  nav_msgs::msg::OccupancyGrid &map,
  std::vector<int16_t> &occupancy_scores,
  int grid_x,
  int grid_y) const
{
  const int support_radius = std::max(0, this->occupancy_endpoint_support_radius_cells_);
  const int support_score = std::max(0, this->occupancy_endpoint_support_score_);
  if (support_radius == 0 || support_score == 0) {
    return;
  }

  for (int offset_y = -support_radius; offset_y <= support_radius; ++offset_y) {
    for (int offset_x = -support_radius; offset_x <= support_radius; ++offset_x) {
      if (offset_x == 0 && offset_y == 0) {
        continue;
      }

      if (std::max(std::abs(offset_x), std::abs(offset_y)) > support_radius) {
        continue;
      }

      this->update_cell_score(
        map,
        occupancy_scores,
        grid_x + offset_x,
        grid_y + offset_y,
        support_score);
    }
  }
}

}  // namespace slam::submap::server
