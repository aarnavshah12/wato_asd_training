#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
 public:
  CostmapNode();

 private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  void publishCostmap(const sensor_msgs::msg::LaserScan& laser_scan);

  robot::CostmapCore costmap_grid_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;
};

#endif  // COSTMAP_NODE_HPP_
