#include "slam_pgraph_server/pose_graph_server.hpp"

namespace slam::graph::server
{

PoseGraphServer::PoseGraphServer()
: next_graph_node_id_(1),
  next_submap_id_(1),
  keyframe_distance_threshold_(0.30),
  keyframe_yaw_threshold_(0.30),
  rotation_only_distance_threshold_(0.10),
  rotation_only_yaw_threshold_(0.70),
  rotation_only_angular_velocity_threshold_(0.20),
  submap_nodes_per_submap_(10),
  loop_closure_min_node_separation_(15),
  loop_closure_descriptor_threshold_(0.12),
  loop_closure_acceptance_score_(20.0),
  loop_closure_search_linear_window_(0.25),
  loop_closure_search_linear_step_(0.05),
  loop_closure_search_angular_window_deg_(18.0),
  loop_closure_search_angular_step_deg_(3.0),
  odom_edge_weight_(1.25),
  loop_edge_weight_(2.0),
  graph_optimization_iterations_(20),
  graph_optimization_step_size_(0.35),
  graph_pose_prior_translation_weight_(0.18),
  graph_pose_prior_yaw_weight_(0.08),
  descriptor_beams_(32),
  mapping_min_range_(0.05),
  mapping_max_range_(8.0)
{
}

void PoseGraphServer::declare_parameters(rclcpp_lifecycle::LifecycleNode &node) const
{
  node.declare_parameter("pose_graph.keyframe_distance_threshold", this->keyframe_distance_threshold_);
  node.declare_parameter("pose_graph.keyframe_yaw_threshold", this->keyframe_yaw_threshold_);
  node.declare_parameter(
    "pose_graph.rotation_only_distance_threshold",
    this->rotation_only_distance_threshold_);
  node.declare_parameter(
    "pose_graph.rotation_only_yaw_threshold",
    this->rotation_only_yaw_threshold_);
  node.declare_parameter(
    "pose_graph.rotation_only_angular_velocity_threshold",
    this->rotation_only_angular_velocity_threshold_);
  node.declare_parameter("pose_graph.submap_nodes_per_submap", this->submap_nodes_per_submap_);
  node.declare_parameter(
    "pose_graph.loop_closure_min_node_separation", this->loop_closure_min_node_separation_);
  node.declare_parameter(
    "pose_graph.loop_closure_descriptor_threshold", this->loop_closure_descriptor_threshold_);
  node.declare_parameter(
    "pose_graph.loop_closure_acceptance_score", this->loop_closure_acceptance_score_);
  node.declare_parameter(
    "pose_graph.loop_closure_search_linear_window", this->loop_closure_search_linear_window_);
  node.declare_parameter(
    "pose_graph.loop_closure_search_linear_step", this->loop_closure_search_linear_step_);
  node.declare_parameter(
    "pose_graph.loop_closure_search_angular_window_deg",
    this->loop_closure_search_angular_window_deg_);
  node.declare_parameter(
    "pose_graph.loop_closure_search_angular_step_deg",
    this->loop_closure_search_angular_step_deg_);
  node.declare_parameter("pose_graph.odom_edge_weight", this->odom_edge_weight_);
  node.declare_parameter("pose_graph.loop_edge_weight", this->loop_edge_weight_);
  node.declare_parameter(
    "pose_graph.graph_optimization_iterations", this->graph_optimization_iterations_);
  node.declare_parameter(
    "pose_graph.graph_optimization_step_size", this->graph_optimization_step_size_);
  node.declare_parameter(
    "pose_graph.graph_pose_prior_translation_weight",
    this->graph_pose_prior_translation_weight_);
  node.declare_parameter(
    "pose_graph.graph_pose_prior_yaw_weight",
    this->graph_pose_prior_yaw_weight_);
  node.declare_parameter("pose_graph.descriptor_beams", this->descriptor_beams_);
}

void PoseGraphServer::load_parameters(rclcpp_lifecycle::LifecycleNode &node)
{
  node.get_parameter("pose_graph.keyframe_distance_threshold", this->keyframe_distance_threshold_);
  node.get_parameter("pose_graph.keyframe_yaw_threshold", this->keyframe_yaw_threshold_);
  node.get_parameter(
    "pose_graph.rotation_only_distance_threshold",
    this->rotation_only_distance_threshold_);
  node.get_parameter(
    "pose_graph.rotation_only_yaw_threshold",
    this->rotation_only_yaw_threshold_);
  node.get_parameter(
    "pose_graph.rotation_only_angular_velocity_threshold",
    this->rotation_only_angular_velocity_threshold_);
  node.get_parameter("pose_graph.submap_nodes_per_submap", this->submap_nodes_per_submap_);
  node.get_parameter(
    "pose_graph.loop_closure_min_node_separation", this->loop_closure_min_node_separation_);
  node.get_parameter(
    "pose_graph.loop_closure_descriptor_threshold", this->loop_closure_descriptor_threshold_);
  node.get_parameter(
    "pose_graph.loop_closure_acceptance_score", this->loop_closure_acceptance_score_);
  node.get_parameter(
    "pose_graph.loop_closure_search_linear_window", this->loop_closure_search_linear_window_);
  node.get_parameter(
    "pose_graph.loop_closure_search_linear_step", this->loop_closure_search_linear_step_);
  node.get_parameter(
    "pose_graph.loop_closure_search_angular_window_deg",
    this->loop_closure_search_angular_window_deg_);
  node.get_parameter(
    "pose_graph.loop_closure_search_angular_step_deg",
    this->loop_closure_search_angular_step_deg_);
  node.get_parameter("pose_graph.odom_edge_weight", this->odom_edge_weight_);
  node.get_parameter("pose_graph.loop_edge_weight", this->loop_edge_weight_);
  node.get_parameter(
    "pose_graph.graph_optimization_iterations", this->graph_optimization_iterations_);
  node.get_parameter(
    "pose_graph.graph_optimization_step_size", this->graph_optimization_step_size_);
  node.get_parameter(
    "pose_graph.graph_pose_prior_translation_weight",
    this->graph_pose_prior_translation_weight_);
  node.get_parameter(
    "pose_graph.graph_pose_prior_yaw_weight",
    this->graph_pose_prior_yaw_weight_);
  node.get_parameter("pose_graph.descriptor_beams", this->descriptor_beams_);
  node.get_parameter("mapping.range.min", this->mapping_min_range_);
  node.get_parameter("mapping.range.max", this->mapping_max_range_);
}

void PoseGraphServer::reset()
{
  this->graph_nodes_.clear();
  this->graph_edges_.clear();
  this->submaps_.clear();
  this->next_graph_node_id_ = 1;
  this->next_submap_id_ = 1;
}

PoseGraphUpdate PoseGraphServer::process_scan(
  const sensor_msgs::msg::LaserScan &scan,
  const Pose2D &corrected_pose,
  const Pose2D &raw_odom_pose,
  double angular_velocity,
  const nav_msgs::msg::OccupancyGrid &map,
  const slam::scan::matcher::ScanMatcher &scan_matcher)
{
  PoseGraphUpdate update{};
  update.latest_pose = corrected_pose;

  const bool add_first = this->graph_nodes_.empty();
  bool should_add = add_first;
  if (!add_first) {
    const GraphNode &last_node = this->graph_nodes_.back();
    const double dx = corrected_pose.x - last_node.map_pose.x;
    const double dy = corrected_pose.y - last_node.map_pose.y;
    const double distance = std::hypot(dx, dy);
    const double dyaw =
      std::abs(this->normalize_angle(corrected_pose.yaw - last_node.map_pose.yaw));
    const bool rotation_only =
      distance < this->rotation_only_distance_threshold_ &&
      dyaw >= this->keyframe_yaw_threshold_ &&
      angular_velocity >= this->rotation_only_angular_velocity_threshold_;

    if (rotation_only) {
      should_add = dyaw >= this->rotation_only_yaw_threshold_;
    } else {
      should_add =
        distance >= this->keyframe_distance_threshold_ ||
        dyaw >= this->keyframe_yaw_threshold_;
    }
  }

  if (!should_add) {
    return update;
  }

  GraphNode node;
  node.id = this->next_graph_node_id_++;
  node.map_pose = corrected_pose;
  node.raw_odom_pose = raw_odom_pose;
  node.scan = scan;
  node.descriptor = this->build_scan_descriptor(scan);
  node.submap_id = this->submaps_.empty() ? this->next_submap_id_ : this->submaps_.back().id;

  if (
    this->submaps_.empty() ||
    this->submaps_.back().keyframe_count >= this->submap_nodes_per_submap_)
  {
    Submap submap;
    submap.id = this->next_submap_id_++;
    submap.anchor_pose = corrected_pose;
    submap.start_node_index = static_cast<int>(this->graph_nodes_.size());
    submap.end_node_index = submap.start_node_index;
    submap.keyframe_count = 0;
    this->submaps_.push_back(submap);
    node.submap_id = submap.id;
  }

  const int new_index = static_cast<int>(this->graph_nodes_.size());
  if (!this->graph_nodes_.empty()) {
    const GraphNode &previous_node = this->graph_nodes_.back();
    GraphEdge odom_edge;
    odom_edge.from = static_cast<int>(this->graph_nodes_.size()) - 1;
    odom_edge.to = new_index;
    odom_edge.relative_pose = this->relative_pose(previous_node.raw_odom_pose, raw_odom_pose);
    odom_edge.weight = std::max(0.1, this->odom_edge_weight_);
    odom_edge.loop_closure = false;
    this->graph_edges_.push_back(odom_edge);
  }

  this->graph_nodes_.push_back(node);
  this->update_submap_accumulation(node);
  update.node_added = true;

  const LoopClosureCandidate candidate = this->search_loop_closure_candidate(
    scan,
    node.descriptor,
    map,
    scan_matcher);

  if (!candidate.found || candidate.match_score < this->loop_closure_acceptance_score_) {
    return update;
  }

  GraphEdge loop_edge;
  loop_edge.from = candidate.node_index;
  loop_edge.to = new_index;
  loop_edge.relative_pose = this->relative_pose(
    this->graph_nodes_[static_cast<std::size_t>(candidate.node_index)].map_pose,
    candidate.matched_pose);
  loop_edge.weight = std::max(0.1, this->loop_edge_weight_);
  loop_edge.loop_closure = true;
  this->graph_edges_.push_back(loop_edge);

  this->optimize_pose_graph();
  update.loop_accepted = true;
  update.graph_rebuild_needed = true;
  update.latest_pose = this->graph_nodes_.back().map_pose;
  return update;
}

std::vector<float> PoseGraphServer::build_scan_descriptor(const sensor_msgs::msg::LaserScan &scan) const
{
  const int beam_count = std::max(8, this->descriptor_beams_);
  std::vector<float> descriptor(static_cast<std::size_t>(beam_count), 1.0F);
  if (scan.ranges.empty()) {
    return descriptor;
  }

  const double max_range =
    std::min(this->mapping_max_range_, static_cast<double>(scan.range_max));
  for (int descriptor_index = 0; descriptor_index < beam_count; ++descriptor_index) {
    const std::size_t source_index = static_cast<std::size_t>(
      std::floor(
        static_cast<double>(descriptor_index) *
        static_cast<double>(scan.ranges.size()) /
        static_cast<double>(beam_count)));
    const std::size_t clamped_index = std::min(source_index, scan.ranges.size() - 1U);
    double range = static_cast<double>(scan.ranges[clamped_index]);
    if (!std::isfinite(range)) {
      range = max_range;
    }
    range = std::clamp(
      range,
      std::max(this->mapping_min_range_, static_cast<double>(scan.range_min)),
      std::max(0.01, max_range));
    descriptor[static_cast<std::size_t>(descriptor_index)] =
      static_cast<float>(range / std::max(0.01, max_range));
  }

  return descriptor;
}

double PoseGraphServer::compute_descriptor_distance(
  const std::vector<float> &lhs,
  const std::vector<float> &rhs) const
{
  if (lhs.size() != rhs.size() || lhs.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double distance = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    distance += std::abs(static_cast<double>(lhs[index] - rhs[index]));
  }

  return distance / static_cast<double>(lhs.size());
}

LoopClosureCandidate PoseGraphServer::search_loop_closure_candidate(
  const sensor_msgs::msg::LaserScan &scan,
  const std::vector<float> &descriptor,
  const nav_msgs::msg::OccupancyGrid &map,
  const slam::scan::matcher::ScanMatcher &scan_matcher) const
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  LoopClosureCandidate best{};
  if (this->graph_nodes_.size() <= static_cast<std::size_t>(this->loop_closure_min_node_separation_)) {
    return best;
  }

  for (
    std::size_t index = 0;
    index + static_cast<std::size_t>(this->loop_closure_min_node_separation_) < this->graph_nodes_.size();
    ++index)
  {
    const GraphNode &node = this->graph_nodes_[index];
    const double descriptor_distance = this->compute_descriptor_distance(descriptor, node.descriptor);
    if (descriptor_distance > this->loop_closure_descriptor_threshold_) {
      continue;
    }

    const double linear_window = std::max(0.0, this->loop_closure_search_linear_window_);
    const double linear_step = std::max(0.01, this->loop_closure_search_linear_step_);
    const double angular_window_rad =
      std::max(0.0, this->loop_closure_search_angular_window_deg_) * kDegToRad;
    const double angular_step_rad =
      std::max(1.0, this->loop_closure_search_angular_step_deg_) * kDegToRad;

    Pose2D best_pose = node.map_pose;
    double best_score = scan_matcher.score_scan_candidate(map, scan, best_pose);
    for (double delta_x = -linear_window; delta_x <= linear_window + 1e-6; delta_x += linear_step) {
      for (double delta_y = -linear_window; delta_y <= linear_window + 1e-6; delta_y += linear_step) {
        for (
          double delta_yaw = -angular_window_rad;
          delta_yaw <= angular_window_rad + 1e-6;
          delta_yaw += angular_step_rad)
        {
          Pose2D candidate_pose{};
          candidate_pose.x = node.map_pose.x + delta_x;
          candidate_pose.y = node.map_pose.y + delta_y;
          candidate_pose.yaw = this->normalize_angle(node.map_pose.yaw + delta_yaw);
          const double candidate_score =
            scan_matcher.score_scan_candidate(map, scan, candidate_pose);
          if (candidate_score > best_score) {
            best_score = candidate_score;
            best_pose = candidate_pose;
          }
        }
      }
    }

    if (!best.found || best_score > best.match_score) {
      best.found = true;
      best.node_index = static_cast<int>(index);
      best.matched_pose = best_pose;
      best.descriptor_distance = descriptor_distance;
      best.match_score = best_score;
    }
  }

  return best;
}

void PoseGraphServer::optimize_pose_graph()
{
  if (this->graph_nodes_.size() < 2U || this->graph_edges_.empty()) {
    return;
  }

  std::vector<Pose2D> prior_poses;
  prior_poses.reserve(this->graph_nodes_.size());
  for (const GraphNode &node : this->graph_nodes_) {
    prior_poses.push_back(node.map_pose);
  }

  const double translation_prior_gain =
    std::max(0.0, this->graph_pose_prior_translation_weight_);
  const double yaw_prior_gain =
    std::max(0.0, this->graph_pose_prior_yaw_weight_);

  for (int iteration = 0; iteration < std::max(1, this->graph_optimization_iterations_); ++iteration) {
    for (const GraphEdge &edge : this->graph_edges_) {
      GraphNode &from_node = this->graph_nodes_[static_cast<std::size_t>(edge.from)];
      GraphNode &to_node = this->graph_nodes_[static_cast<std::size_t>(edge.to)];

      const Pose2D predicted_to = this->compose_pose(from_node.map_pose, edge.relative_pose);
      const double error_x = predicted_to.x - to_node.map_pose.x;
      const double error_y = predicted_to.y - to_node.map_pose.y;
      const double error_yaw = this->normalize_angle(predicted_to.yaw - to_node.map_pose.yaw);
      const double gain = this->graph_optimization_step_size_ * edge.weight;

      if (edge.to != 0) {
        to_node.map_pose.x += gain * error_x;
        to_node.map_pose.y += gain * error_y;
        to_node.map_pose.yaw = this->normalize_angle(to_node.map_pose.yaw + gain * error_yaw);
      }
      if (edge.loop_closure && edge.from != 0) {
        from_node.map_pose.x -= 0.5 * gain * error_x;
        from_node.map_pose.y -= 0.5 * gain * error_y;
        from_node.map_pose.yaw =
          this->normalize_angle(from_node.map_pose.yaw - 0.5 * gain * error_yaw);
      }
    }

    for (std::size_t index = 1; index < this->graph_nodes_.size(); ++index) {
      GraphNode &node = this->graph_nodes_[index];
      const Pose2D &prior_pose = prior_poses[index];

      node.map_pose.x += translation_prior_gain * (prior_pose.x - node.map_pose.x);
      node.map_pose.y += translation_prior_gain * (prior_pose.y - node.map_pose.y);
      node.map_pose.yaw = this->normalize_angle(
        node.map_pose.yaw +
        (yaw_prior_gain * this->normalize_angle(prior_pose.yaw - node.map_pose.yaw)));
    }
  }
}

void PoseGraphServer::update_submap_accumulation(const GraphNode &node)
{
  if (this->submaps_.empty()) {
    return;
  }

  Submap &submap = this->submaps_.back();
  submap.end_node_index = static_cast<int>(this->graph_nodes_.size()) - 1;
  submap.keyframe_count += 1;
  submap.anchor_pose = node.map_pose;
}

Pose2D PoseGraphServer::compose_pose(const Pose2D &lhs, const Pose2D &rhs) const
{
  Pose2D composed{};
  composed.x = lhs.x + (std::cos(lhs.yaw) * rhs.x) - (std::sin(lhs.yaw) * rhs.y);
  composed.y = lhs.y + (std::sin(lhs.yaw) * rhs.x) + (std::cos(lhs.yaw) * rhs.y);
  composed.yaw = this->normalize_angle(lhs.yaw + rhs.yaw);
  return composed;
}

Pose2D PoseGraphServer::inverse_pose(const Pose2D &pose) const
{
  Pose2D inverse{};
  inverse.yaw = this->normalize_angle(-pose.yaw);
  inverse.x = -(std::cos(inverse.yaw) * pose.x - std::sin(inverse.yaw) * pose.y);
  inverse.y = -(std::sin(inverse.yaw) * pose.x + std::cos(inverse.yaw) * pose.y);
  return inverse;
}

Pose2D PoseGraphServer::relative_pose(const Pose2D &from, const Pose2D &to) const
{
  return this->compose_pose(this->inverse_pose(from), to);
}

double PoseGraphServer::normalize_angle(double angle) const
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

std::string PoseGraphServer::build_graph_debug_json(const std::string &frame_id) const
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(6);

  std::size_t loop_count = 0U;
  for (const GraphEdge &edge : this->graph_edges_) {
    if (edge.loop_closure) {
      ++loop_count;
    }
  }

  stream
    << "{"
    << "\"frame_id\":\"" << frame_id << "\","
    << "\"node_count\":" << this->graph_nodes_.size() << ","
    << "\"edge_count\":" << this->graph_edges_.size() << ","
    << "\"loop_count\":" << loop_count << ","
    << "\"nodes\":[";

  for (std::size_t index = 0; index < this->graph_nodes_.size(); ++index) {
    const GraphNode &node = this->graph_nodes_[index];
    if (index > 0U) {
      stream << ",";
    }
    stream
      << "{"
      << "\"id\":" << node.id << ","
      << "\"x\":" << node.map_pose.x << ","
      << "\"y\":" << node.map_pose.y << ","
      << "\"yaw\":" << node.map_pose.yaw << ","
      << "\"submap_id\":" << node.submap_id
      << "}";
  }

  stream << "],\"edges\":[";
  bool first_edge = true;
  for (std::size_t index = 0; index < this->graph_edges_.size(); ++index) {
    const GraphEdge &edge = this->graph_edges_[index];
    if (edge.from < 0 || edge.to < 0) {
      continue;
    }
    const std::size_t from_index = static_cast<std::size_t>(edge.from);
    const std::size_t to_index = static_cast<std::size_t>(edge.to);
    if (from_index >= this->graph_nodes_.size() || to_index >= this->graph_nodes_.size()) {
      continue;
    }
    if (!first_edge) {
      stream << ",";
    }
    first_edge = false;
    const GraphNode &from_node = this->graph_nodes_[from_index];
    const GraphNode &to_node = this->graph_nodes_[to_index];
    stream
      << "{"
      << "\"from\":" << from_node.id << ","
      << "\"to\":" << to_node.id << ","
      << "\"from_x\":" << from_node.map_pose.x << ","
      << "\"from_y\":" << from_node.map_pose.y << ","
      << "\"to_x\":" << to_node.map_pose.x << ","
      << "\"to_y\":" << to_node.map_pose.y << ","
      << "\"weight\":" << edge.weight << ","
      << "\"loop_closure\":" << (edge.loop_closure ? "true" : "false")
      << "}";
  }

  stream << "],\"submaps\":[";
  for (std::size_t index = 0; index < this->submaps_.size(); ++index) {
    const Submap &submap = this->submaps_[index];
    if (index > 0U) {
      stream << ",";
    }
    stream
      << "{"
      << "\"id\":" << submap.id << ","
      << "\"start_node_index\":" << submap.start_node_index << ","
      << "\"end_node_index\":" << submap.end_node_index << ","
      << "\"keyframe_count\":" << submap.keyframe_count << ","
      << "\"anchor_x\":" << submap.anchor_pose.x << ","
      << "\"anchor_y\":" << submap.anchor_pose.y << ","
      << "\"anchor_yaw\":" << submap.anchor_pose.yaw
      << "}";
  }

  stream << "]}";
  return stream.str();
}

const std::vector<GraphNode> &PoseGraphServer::graph_nodes() const
{
  return this->graph_nodes_;
}

}  // namespace slam::graph::server
