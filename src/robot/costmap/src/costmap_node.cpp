#include "costmap_node.hpp"

#include <cmath>
#include <functional>
#include <memory>

CostmapNode::CostmapNode() : Node("costmap") {
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", rclcpp::SensorDataQoS(),
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Start with an empty grid for this scan.
  costmap_.initializeCostmap();

  // Step 2: Mark the grid cell at each valid laser hit.
  for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
    const double range = scan->ranges[i];
    if (!std::isfinite(range) || range <= scan->range_min || range >= scan->range_max) {
      continue;
    }

    const double angle = scan->angle_min + i * scan->angle_increment;
    int x_grid;
    int y_grid;
    if (costmap_.convertToGrid(range, angle, x_grid, y_grid)) {
      costmap_.markObstacle(x_grid, y_grid);
    }
  }

  // Step 3: Give cells near obstacles a lower, distance-based cost.
  costmap_.inflateObstacles();

  // Step 4: Send the grid to the rest of the robot.
  publishCostmap(*scan);
}

void CostmapNode::publishCostmap(const sensor_msgs::msg::LaserScan& scan) {
  nav_msgs::msg::OccupancyGrid message;
  message.header = scan.header;
  message.info.map_load_time = scan.header.stamp;
  message.info.resolution = robot::CostmapCore::kResolution;
  message.info.width = robot::CostmapCore::kWidth;
  message.info.height = robot::CostmapCore::kHeight;
  message.info.origin.position.x = -robot::CostmapCore::kWidth * robot::CostmapCore::kResolution / 2.0;
  message.info.origin.position.y = -robot::CostmapCore::kHeight * robot::CostmapCore::kResolution / 2.0;
  message.info.origin.orientation.w = 1.0;
  message.data = costmap_.data();
  costmap_pub_->publish(message);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
