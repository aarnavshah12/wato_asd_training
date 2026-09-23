#include "map_memory_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(this->declare_parameter<double>("map_size_m", 100.0)),
      distance_threshold_m_(this->declare_parameter<double>("update_distance_m", 1.5)) {
  costmap_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  // Keep the latest map available for viewers that connect after the first update.
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());
  update_timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = costmap;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry) {
  recent_odometry_.push_back(odometry);
  // Keep a few seconds of poses so a scan can use the pose from its timestamp.
  if (recent_odometry_.size() > 30) {
    recent_odometry_.pop_front();
  }
}

void MapMemoryNode::updateMap() {
  if (!latest_costmap_ || recent_odometry_.empty() || !costmap_updated_) {
    return;
  }

  // A small pose mismatch can shift distant laser hits a lot during a turn.
  const rclcpp::Time scan_time(latest_costmap_->header.stamp);
  auto matched_odometry = recent_odometry_.front();
  double smallest_gap_s = std::abs(
      (scan_time - rclcpp::Time(matched_odometry->header.stamp)).seconds());
  for (const auto& odometry : recent_odometry_) {
    const double gap_s = std::abs(
        (scan_time - rclcpp::Time(odometry->header.stamp)).seconds());
    if (gap_s < smallest_gap_s) {
      smallest_gap_s = gap_s;
      matched_odometry = odometry;
    }
  }
  if (smallest_gap_s > 0.15) {
    return;  // Wait for a scan and pose that belong together.
  }

  const double robot_x = matched_odometry->pose.pose.position.x;
  const double robot_y = matched_odometry->pose.pose.position.y;
  if (has_last_update_ &&
      std::hypot(robot_x - last_update_x_, robot_y - last_update_y_) <
          distance_threshold_m_) {
    return;
  }

  if (!map_memory_.integrateCostmap(*latest_costmap_, *matched_odometry)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Could not merge costmap: check its size, resolution, and frame IDs");
    return;
  }

  map_publisher_->publish(map_memory_.map());
  last_update_x_ = robot_x;
  last_update_y_ = robot_y;
  has_last_update_ = true;
  costmap_updated_ = false;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
