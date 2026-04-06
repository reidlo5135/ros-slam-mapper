#include "slam_pgraph_server/slam_pgraph_server.hpp"

namespace slam::graph::server
{

namespace
{

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

}  // namespace

SlamPGraphServer::SlamPGraphServer(const rclcpp::NodeOptions &options)
: rclcpp_lifecycle::LifecycleNode("slam_pgraph_server", options),
  frame_id_("map"),
  odom_frame_("odom"),
  base_frame_("base_link"),
  odom_topic_("/odom"),
  imu_topic_("/imu"),
  scan_topic_("/scan"),
  temporary_map_topic_("/slam/map/temp"),
  raw_temporary_map_topic_("/slam/map/temp/raw"),
  refined_temporary_map_topic_("/slam/map/temp/refined"),
  corrected_odometry_topic_("/slam/mapper/odometry"),
  mapping_pose_topic_("/slam/mapper/pose"),
  graph_debug_topic_("/slam/mapper/graph_debug"),
  publish_period_ms_(250),
  publish_map_to_odom_tf_(true)
{
  this->declare_parameter("frames.map", this->frame_id_);
  this->declare_parameter("frames.odom", this->odom_frame_);
  this->declare_parameter("frames.base", this->base_frame_);
  this->declare_parameter("topics.odom", this->odom_topic_);
  this->declare_parameter("topics.imu", this->imu_topic_);
  this->declare_parameter("topics.scan", this->scan_topic_);
  this->declare_parameter("topics.temp_map", this->temporary_map_topic_);
  this->declare_parameter("topics.temp_map_raw", this->raw_temporary_map_topic_);
  this->declare_parameter("topics.temp_map_refined", this->refined_temporary_map_topic_);
  this->declare_parameter("topics.corrected_odometry", this->corrected_odometry_topic_);
  this->declare_parameter("topics.mapping_pose", this->mapping_pose_topic_);
  this->declare_parameter("topics.graph_debug", this->graph_debug_topic_);
  this->declare_parameter("publish_period_ms", this->publish_period_ms_);
  this->declare_parameter("mapping.publish_map_to_odom_tf", this->publish_map_to_odom_tf_);
  this->scan_matcher_.declare_parameters(*this);
  this->pose_graph_server_.declare_parameters(*this);
  this->submap_server_.declare_parameters(*this);
}

SlamPGraphServer::CallbackReturn SlamPGraphServer::on_configure(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->get_parameter("frames.map", this->frame_id_);
  this->get_parameter("frames.odom", this->odom_frame_);
  this->get_parameter("frames.base", this->base_frame_);
  this->get_parameter("topics.odom", this->odom_topic_);
  this->get_parameter("topics.imu", this->imu_topic_);
  this->get_parameter("topics.scan", this->scan_topic_);
  this->get_parameter("topics.temp_map", this->temporary_map_topic_);
  this->get_parameter("topics.temp_map_raw", this->raw_temporary_map_topic_);
  this->get_parameter("topics.temp_map_refined", this->refined_temporary_map_topic_);
  this->get_parameter("topics.corrected_odometry", this->corrected_odometry_topic_);
  this->get_parameter("topics.mapping_pose", this->mapping_pose_topic_);
  this->get_parameter("topics.graph_debug", this->graph_debug_topic_);
  this->get_parameter("publish_period_ms", this->publish_period_ms_);
  this->get_parameter("mapping.publish_map_to_odom_tf", this->publish_map_to_odom_tf_);

  this->scan_matcher_.load_parameters(*this);
  this->pose_graph_server_.load_parameters(*this);
  this->submap_server_.load_parameters(*this);
  this->pose_graph_server_.reset();
  this->submap_server_.reset(this->frame_id_, this->now());

  this->latest_odometry_ = nav_msgs::msg::Odometry();
  this->current_corrected_pose_ = Pose2D{};
  this->map_to_odom_ = Pose2D{};
  this->start_odom_yaw_ = 0.0;
  this->start_imu_yaw_ = 0.0;
  this->latest_imu_yaw_ = 0.0;
  this->has_latest_odometry_ = false;
  this->has_latest_scan_ = false;
  this->has_current_corrected_pose_ = false;
  this->has_start_odom_yaw_ = false;
  this->has_latest_imu_ = false;
  this->has_start_imu_yaw_ = false;

  this->temporary_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->temporary_map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->raw_temporary_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->raw_temporary_map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->refined_temporary_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    this->refined_temporary_map_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  this->corrected_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    this->corrected_odometry_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  this->mapping_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    this->mapping_pose_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  this->graph_debug_publisher_ = this->create_publisher<std_msgs::msg::String>(
    this->graph_debug_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  this->odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    this->odom_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr message) {
      this->handle_odometry(message);
    });
  this->imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    this->imu_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr message) {
      this->handle_imu(message);
    });
  this->scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    this->scan_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {
      this->handle_scan(message);
    });

  this->transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  this->publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(std::max(50, this->publish_period_ms_)),
    [this]() {
      this->publish_outputs();
    });

  double keyframe_distance_threshold = 0.0;
  double keyframe_yaw_threshold = 0.0;
  double loop_closure_acceptance_score = 0.0;
  double odom_edge_weight = 0.0;
  double loop_edge_weight = 0.0;
  this->get_parameter(
    "pose_graph.keyframe_distance_threshold",
    keyframe_distance_threshold);
  this->get_parameter("pose_graph.keyframe_yaw_threshold", keyframe_yaw_threshold);
  this->get_parameter(
    "pose_graph.loop_closure_acceptance_score",
    loop_closure_acceptance_score);
  this->get_parameter("pose_graph.odom_edge_weight", odom_edge_weight);
  this->get_parameter("pose_graph.loop_edge_weight", loop_edge_weight);

  RCLCPP_INFO(
    this->get_logger(),
    "Configured SLAM pgraph server with frames map='%s', odom='%s', base='%s', odom='%s', imu='%s', scan='%s', temp_map='%s', temp_raw='%s', temp_refined='%s', corrected_odom='%s', mapping_pose='%s', graph_debug='%s'",
    this->frame_id_.c_str(),
    this->odom_frame_.c_str(),
    this->base_frame_.c_str(),
    this->odom_topic_.c_str(),
    this->imu_topic_.c_str(),
    this->scan_topic_.c_str(),
    this->temporary_map_topic_.c_str(),
    this->raw_temporary_map_topic_.c_str(),
    this->refined_temporary_map_topic_.c_str(),
    this->corrected_odometry_topic_.c_str(),
    this->mapping_pose_topic_.c_str(),
    this->graph_debug_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "TF policy publish_map_to_odom_tf=%s, publish_period_ms=%d",
    this->publish_map_to_odom_tf_ ? "true" : "false",
    this->publish_period_ms_);
  RCLCPP_INFO(
    this->get_logger(),
    "Pose graph params keyframe_distance=%.3f m, keyframe_yaw=%.3f deg, loop_acceptance=%.3f, odom_edge_weight=%.3f, loop_edge_weight=%.3f",
    keyframe_distance_threshold,
    keyframe_yaw_threshold * kRadToDeg,
    loop_closure_acceptance_score,
    odom_edge_weight,
    loop_edge_weight);
  return CallbackReturn::SUCCESS;
}

SlamPGraphServer::CallbackReturn SlamPGraphServer::on_activate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->temporary_map_publisher_) {
    this->temporary_map_publisher_->on_activate();
  }
  if (this->raw_temporary_map_publisher_) {
    this->raw_temporary_map_publisher_->on_activate();
  }
  if (this->refined_temporary_map_publisher_) {
    this->refined_temporary_map_publisher_->on_activate();
  }
  if (this->corrected_odometry_publisher_) {
    this->corrected_odometry_publisher_->on_activate();
  }
  if (this->mapping_pose_publisher_) {
    this->mapping_pose_publisher_->on_activate();
  }
  if (this->graph_debug_publisher_) {
    this->graph_debug_publisher_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "Activated SLAM pgraph server");
  this->publish_outputs();
  return CallbackReturn::SUCCESS;
}

SlamPGraphServer::CallbackReturn SlamPGraphServer::on_deactivate(const rclcpp_lifecycle::State &state)
{
  (void)state;
  if (this->temporary_map_publisher_) {
    this->temporary_map_publisher_->on_deactivate();
  }
  if (this->raw_temporary_map_publisher_) {
    this->raw_temporary_map_publisher_->on_deactivate();
  }
  if (this->refined_temporary_map_publisher_) {
    this->refined_temporary_map_publisher_->on_deactivate();
  }
  if (this->corrected_odometry_publisher_) {
    this->corrected_odometry_publisher_->on_deactivate();
  }
  if (this->mapping_pose_publisher_) {
    this->mapping_pose_publisher_->on_deactivate();
  }
  if (this->graph_debug_publisher_) {
    this->graph_debug_publisher_->on_deactivate();
  }
  RCLCPP_INFO(this->get_logger(), "Deactivated SLAM pgraph server");
  return CallbackReturn::SUCCESS;
}

SlamPGraphServer::CallbackReturn SlamPGraphServer::on_cleanup(const rclcpp_lifecycle::State &state)
{
  (void)state;
  this->publish_timer_.reset();
  this->scan_subscription_.reset();
  this->imu_subscription_.reset();
  this->odometry_subscription_.reset();
  this->temporary_map_publisher_.reset();
  this->raw_temporary_map_publisher_.reset();
  this->refined_temporary_map_publisher_.reset();
  this->corrected_odometry_publisher_.reset();
  this->mapping_pose_publisher_.reset();
  this->graph_debug_publisher_.reset();
  this->transform_broadcaster_.reset();
  this->pose_graph_server_.reset();
  this->submap_server_.reset(this->frame_id_, this->now());
  RCLCPP_INFO(this->get_logger(), "Cleaned up SLAM pgraph server");
  return CallbackReturn::SUCCESS;
}

SlamPGraphServer::CallbackReturn SlamPGraphServer::on_shutdown(const rclcpp_lifecycle::State &state)
{
  (void)state;
  return this->on_cleanup(state);
}

void SlamPGraphServer::publish_outputs()
{
  this->submap_server_.refresh_refined_map(this->now());
  this->publish_raw_temporary_map();
  this->publish_refined_temporary_map();
  this->publish_temporary_map();
  this->publish_corrected_odometry();
  this->publish_mapping_pose();
  this->publish_graph_debug();
  if (this->publish_map_to_odom_tf_) {
    this->publish_map_to_odom_tf();
  }
}

void SlamPGraphServer::publish_temporary_map()
{
  if (!this->temporary_map_publisher_ || !this->temporary_map_publisher_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish = this->submap_server_.refined_map();
  map_to_publish.header.stamp = this->now();
  map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  this->temporary_map_publisher_->publish(map_to_publish);
}

void SlamPGraphServer::publish_raw_temporary_map()
{
  if (!this->raw_temporary_map_publisher_ || !this->raw_temporary_map_publisher_->is_activated()) {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish = this->submap_server_.raw_map();
  map_to_publish.header.stamp = this->now();
  map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  this->raw_temporary_map_publisher_->publish(map_to_publish);
}

void SlamPGraphServer::publish_refined_temporary_map()
{
  if (
    !this->refined_temporary_map_publisher_ ||
    !this->refined_temporary_map_publisher_->is_activated())
  {
    return;
  }

  nav_msgs::msg::OccupancyGrid map_to_publish = this->submap_server_.refined_map();
  map_to_publish.header.stamp = this->now();
  map_to_publish.info.map_load_time = map_to_publish.header.stamp;
  this->refined_temporary_map_publisher_->publish(map_to_publish);
}

void SlamPGraphServer::publish_corrected_odometry()
{
  if (
    !this->corrected_odometry_publisher_ ||
    !this->corrected_odometry_publisher_->is_activated() ||
    !this->has_current_corrected_pose_)
  {
    return;
  }

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = this->now();
  odometry.header.frame_id = this->frame_id_;
  odometry.child_frame_id = this->base_frame_;
  odometry.pose.pose.position.x = this->current_corrected_pose_.x;
  odometry.pose.pose.position.y = this->current_corrected_pose_.y;
  odometry.pose.pose.position.z = 0.0;
  this->scan_matcher_.set_quaternion_from_yaw(
    odometry.pose.pose.orientation,
    this->current_corrected_pose_.yaw);
  this->corrected_odometry_publisher_->publish(odometry);
}

void SlamPGraphServer::publish_mapping_pose()
{
  if (
    !this->mapping_pose_publisher_ ||
    !this->mapping_pose_publisher_->is_activated() ||
    !this->has_current_corrected_pose_)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = this->frame_id_;
  pose.pose.position.x = this->current_corrected_pose_.x;
  pose.pose.position.y = this->current_corrected_pose_.y;
  pose.pose.position.z = 0.0;
  this->scan_matcher_.set_quaternion_from_yaw(pose.pose.orientation, this->current_corrected_pose_.yaw);
  this->mapping_pose_publisher_->publish(pose);
}

void SlamPGraphServer::publish_graph_debug()
{
  if (!this->graph_debug_publisher_ || !this->graph_debug_publisher_->is_activated()) {
    return;
  }

  std_msgs::msg::String message;
  message.data = this->pose_graph_server_.build_graph_debug_json(this->frame_id_);
  this->graph_debug_publisher_->publish(message);
}

void SlamPGraphServer::publish_map_to_odom_tf()
{
  if (!this->transform_broadcaster_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = this->frame_id_;
  transform.child_frame_id = this->odom_frame_;
  transform.transform.translation.x = this->map_to_odom_.x;
  transform.transform.translation.y = this->map_to_odom_.y;
  transform.transform.translation.z = 0.0;
  this->scan_matcher_.set_quaternion_from_yaw(transform.transform.rotation, this->map_to_odom_.yaw);
  this->transform_broadcaster_->sendTransform(transform);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Publishing map->odom TF x=%.3f m, y=%.3f m, yaw=%.3f deg",
    this->map_to_odom_.x,
    this->map_to_odom_.y,
    this->map_to_odom_.yaw * kRadToDeg);
}

void SlamPGraphServer::handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  this->latest_odometry_ = *message;
  this->has_latest_odometry_ = true;
  if (!this->has_start_odom_yaw_) {
    this->start_odom_yaw_ = this->scan_matcher_.quaternion_to_yaw(message->pose.pose.orientation);
    this->has_start_odom_yaw_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Captured start odom yaw %.3f deg at x=%.3f m, y=%.3f m",
      this->start_odom_yaw_ * kRadToDeg,
      message->pose.pose.position.x,
      message->pose.pose.position.y);
  }
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Odometry update x=%.3f m, y=%.3f m, yaw=%.3f deg, angular_z=%.3f rad/s",
    message->pose.pose.position.x,
    message->pose.pose.position.y,
    this->scan_matcher_.quaternion_to_yaw(message->pose.pose.orientation) * kRadToDeg,
    message->twist.twist.angular.z);
}

void SlamPGraphServer::handle_imu(const sensor_msgs::msg::Imu::SharedPtr message)
{
  this->latest_imu_yaw_ = this->scan_matcher_.quaternion_to_yaw(message->orientation);
  this->has_latest_imu_ = true;
  if (!this->has_start_imu_yaw_) {
    this->start_imu_yaw_ = this->latest_imu_yaw_;
    this->has_start_imu_yaw_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Captured start IMU yaw %.3f deg",
      this->start_imu_yaw_ * kRadToDeg);
  }
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "IMU update yaw=%.3f deg",
    this->latest_imu_yaw_ * kRadToDeg);
}

void SlamPGraphServer::handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
{
  if (!this->has_latest_odometry_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Waiting for odometry before processing scans");
    return;
  }

  this->has_latest_scan_ = true;

  MotionPriorState motion_prior_state;
  motion_prior_state.start_odom_yaw = this->start_odom_yaw_;
  motion_prior_state.start_imu_yaw = this->start_imu_yaw_;
  motion_prior_state.latest_imu_yaw = this->latest_imu_yaw_;
  motion_prior_state.has_latest_imu = this->has_latest_imu_;
  motion_prior_state.has_start_odom_yaw = this->has_start_odom_yaw_;
  motion_prior_state.has_start_imu_yaw = this->has_start_imu_yaw_;

  const Pose2D raw_odom_pose =
    this->scan_matcher_.build_raw_odom_pose(this->latest_odometry_, motion_prior_state);
  const Pose2D predicted_pose = this->apply_map_to_odom_transform(raw_odom_pose);
  const nav_msgs::msg::OccupancyGrid front_end_map = this->submap_server_.raw_map();
  const slam::scan::matcher::ScanMatchResult scan_match_result =
    this->scan_matcher_.refine_pose_with_scan_matching_detailed(
    front_end_map,
    *message,
    predicted_pose);
  const Pose2D corrected_pose = scan_match_result.pose;
  const Pose2D front_end_delta =
    this->scan_matcher_.relative_pose(predicted_pose, corrected_pose);

  this->current_corrected_pose_ = corrected_pose;
  this->has_current_corrected_pose_ = true;
  this->update_map_to_odom_transform(corrected_pose, raw_odom_pose);
  this->submap_server_.integrate_scan(*message, corrected_pose);

  const nav_msgs::msg::OccupancyGrid graph_map = this->submap_server_.raw_map();
  const PoseGraphUpdate graph_update =
    this->pose_graph_server_.process_scan(
    *message,
    corrected_pose,
    raw_odom_pose,
    std::abs(this->latest_odometry_.twist.twist.angular.z),
    graph_map,
    this->scan_matcher_);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Scan cycle raw=(%.3f, %.3f, %.3f deg) predicted=(%.3f, %.3f, %.3f deg) corrected=(%.3f, %.3f, %.3f deg) delta=(%.3f m, %.3f m, %.3f deg) graph_nodes=%zu",
    raw_odom_pose.x,
    raw_odom_pose.y,
    raw_odom_pose.yaw * kRadToDeg,
    predicted_pose.x,
    predicted_pose.y,
    predicted_pose.yaw * kRadToDeg,
    corrected_pose.x,
    corrected_pose.y,
    corrected_pose.yaw * kRadToDeg,
    front_end_delta.x,
    front_end_delta.y,
    front_end_delta.yaw * kRadToDeg,
    this->pose_graph_server_.graph_nodes().size());
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    2000,
    "Scan matcher diag occupied_cells=%d predicted_score=%.3f coarse_score=%.3f fine_score=%.3f final_score=%.3f improvement=%.3f valid_beams(pred/coarse/fine)=%d/%d/%d applied=%s reject_reason=%s correction=(%.3f m, %.3f deg)",
    scan_match_result.debug.occupied_cell_count,
    scan_match_result.debug.predicted_score,
    scan_match_result.debug.coarse_score,
    scan_match_result.debug.fine_score,
    scan_match_result.debug.final_score,
    scan_match_result.debug.score_improvement,
    scan_match_result.debug.predicted_valid_beam_count,
    scan_match_result.debug.coarse_valid_beam_count,
    scan_match_result.debug.fine_valid_beam_count,
    scan_match_result.debug.correction_applied ? "true" : "false",
    this->scan_matcher_.scan_match_reject_reason_to_cstr(scan_match_result.debug.reject_reason),
    scan_match_result.debug.correction_translation,
    scan_match_result.debug.correction_yaw_deg);

  if (graph_update.node_added) {
    RCLCPP_INFO(
      this->get_logger(),
      "Added graph node count=%zu latest_pose=(%.3f, %.3f, %.3f deg)",
      this->pose_graph_server_.graph_nodes().size(),
      corrected_pose.x,
      corrected_pose.y,
      corrected_pose.yaw * kRadToDeg);
  }

  if (graph_update.loop_accepted) {
    RCLCPP_INFO(
      this->get_logger(),
      "Accepted loop closure; rebuilding map from pose graph");
  }

  if (graph_update.graph_rebuild_needed) {
    std::vector<slam::submap::server::SubmapNode> rebuild_nodes;
    rebuild_nodes.reserve(this->pose_graph_server_.graph_nodes().size());
    for (const GraphNode &graph_node : this->pose_graph_server_.graph_nodes()) {
      slam::submap::server::SubmapNode rebuild_node;
      rebuild_node.map_pose = graph_node.map_pose;
      rebuild_node.scan = graph_node.scan;
      rebuild_nodes.push_back(rebuild_node);
    }
    this->submap_server_.rebuild_map_from_pose_graph(rebuild_nodes, this->now());
    this->current_corrected_pose_ = graph_update.latest_pose;
    this->update_map_to_odom_transform(this->current_corrected_pose_, raw_odom_pose);
    RCLCPP_INFO(
      this->get_logger(),
      "Back-end updated map->odom after graph rebuild latest_pose=(%.3f, %.3f, %.3f deg) tf=(%.3f, %.3f, %.3f deg)",
      this->current_corrected_pose_.x,
      this->current_corrected_pose_.y,
      this->current_corrected_pose_.yaw * kRadToDeg,
      this->map_to_odom_.x,
      this->map_to_odom_.y,
      this->map_to_odom_.yaw * kRadToDeg);
  }
}

SlamPGraphServer::Pose2D SlamPGraphServer::apply_map_to_odom_transform(const Pose2D &odom_pose) const
{
  return this->scan_matcher_.compose_pose(this->map_to_odom_, odom_pose);
}

void SlamPGraphServer::update_map_to_odom_transform(
  const Pose2D &corrected_pose,
  const Pose2D &raw_odom_pose)
{
  this->map_to_odom_ = this->scan_matcher_.compose_pose(
    corrected_pose,
    this->scan_matcher_.inverse_pose(raw_odom_pose));
}

}  // namespace slam::graph::server
