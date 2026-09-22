#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
 public:
  MapMemoryNode();

 private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);
  void updateMap();

  robot::MapMemoryCore map_memory_;
  double distance_threshold_m_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  nav_msgs::msg::Odometry::SharedPtr latest_odometry_;
  bool has_last_update_ = false;
  bool costmap_updated_ = false;
  double last_update_x_ = 0.0;
  double last_update_y_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

#endif  // MAP_MEMORY_NODE_HPP_
