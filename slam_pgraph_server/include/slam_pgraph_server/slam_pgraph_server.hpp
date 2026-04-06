#ifndef SLAM_PGRAPH_SERVER__SLAM_PGRAPH_SERVER_HPP_
#define SLAM_PGRAPH_SERVER__SLAM_PGRAPH_SERVER_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "slam_pgraph_server/pose_graph_server.hpp"
#include "slam_scan_matcher/scan_matcher.hpp"
#include "slam_submap_server/submap_server.hpp"

namespace slam::graph::server
{

class SlamPGraphServer : public rclcpp_lifecycle::LifecycleNode
{
private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Pose2D = slam::scan::matcher::Pose2D;
  using MotionPriorState = slam::scan::matcher::MotionPriorState;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr temporary_map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_temporary_map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr refined_temporary_map_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odometry_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr mapping_pose_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr graph_debug_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  slam::scan::matcher::ScanMatcher scan_matcher_;
  PoseGraphServer pose_graph_server_;
  slam::submap::server::SubmapServer submap_server_;

  nav_msgs::msg::Odometry latest_odometry_{};
  Pose2D current_corrected_pose_{};
  Pose2D map_to_odom_{};
  double start_odom_yaw_{0.0};
  double start_imu_yaw_{0.0};
  double latest_imu_yaw_{0.0};
  bool has_latest_odometry_{false};
  bool has_latest_scan_{false};
  bool has_current_corrected_pose_{false};
  bool has_start_odom_yaw_{false};
  bool has_latest_imu_{false};
  bool has_start_imu_yaw_{false};

  std::string frame_id_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string scan_topic_;
  std::string temporary_map_topic_;
  std::string raw_temporary_map_topic_;
  std::string refined_temporary_map_topic_;
  std::string corrected_odometry_topic_;
  std::string mapping_pose_topic_;
  std::string graph_debug_topic_;
  int publish_period_ms_;
  bool publish_map_to_odom_tf_;
  int front_end_recent_nodes_;
  int front_end_min_recent_nodes_;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  void publish_outputs();
  void publish_temporary_map();
  void publish_raw_temporary_map();
  void publish_refined_temporary_map();
  void publish_corrected_odometry();
  void publish_mapping_pose();
  void publish_graph_debug();
  void publish_map_to_odom_tf();

  void handle_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr message);
  void handle_scan(const sensor_msgs::msg::LaserScan::SharedPtr message);

  Pose2D apply_map_to_odom_transform(const Pose2D &odom_pose) const;
  void update_map_to_odom_transform(const Pose2D &corrected_pose, const Pose2D &raw_odom_pose);

protected:
public:
  explicit SlamPGraphServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~SlamPGraphServer() = default;
};

}  // namespace slam::graph::server

#endif  // SLAM_PGRAPH_SERVER__SLAM_PGRAPH_SERVER_HPP_
