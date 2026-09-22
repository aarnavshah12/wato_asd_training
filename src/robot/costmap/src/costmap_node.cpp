#include "costmap_node.hpp"

#include <cmath>
#include <functional>
#include <memory>

CostmapNode::CostmapNode() : Node("costmap") {
  costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", rclcpp::SensorDataQoS(),
      std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
  // Step 1: Start with an empty grid for this scan.
  costmap_grid_.initializeCostmap();

  // Step 2: Mark the grid cell at each valid laser hit.
  for (std::size_t beam_index = 0; beam_index < laser_scan->ranges.size(); ++beam_index) {
    const double measured_distance_m = laser_scan->ranges[beam_index];
    if (!std::isfinite(measured_distance_m) ||
        measured_distance_m <= laser_scan->range_min ||
        measured_distance_m >= laser_scan->range_max) {
      continue;
    }

    const double beam_angle_rad =
        laser_scan->angle_min + beam_index * laser_scan->angle_increment;
    int cell_x;
    int cell_y;
    if (costmap_grid_.convertToGrid(measured_distance_m, beam_angle_rad, cell_x, cell_y)) {
      costmap_grid_.markObstacle(cell_x, cell_y);
    }
  }

  // Step 3: Give cells near obstacles a lower, distance-based cost.
  costmap_grid_.inflateObstacles();

  // Step 4: Send the grid to the rest of the robot.
  publishCostmap(*laser_scan);
}

void CostmapNode::publishCostmap(const sensor_msgs::msg::LaserScan& laser_scan) {
  nav_msgs::msg::OccupancyGrid grid_message;
  grid_message.header = laser_scan.header;
  grid_message.info.map_load_time = laser_scan.header.stamp;
  grid_message.info.resolution = robot::CostmapCore::grid_resolution_m;
  grid_message.info.width = robot::CostmapCore::grid_width_cells;
  grid_message.info.height = robot::CostmapCore::grid_height_cells;
  grid_message.info.origin.position.x =
      -robot::CostmapCore::grid_width_cells * robot::CostmapCore::grid_resolution_m / 2.0;
  grid_message.info.origin.position.y =
      -robot::CostmapCore::grid_height_cells * robot::CostmapCore::grid_resolution_m / 2.0;
  grid_message.info.origin.orientation.w = 1.0;
  grid_message.data = costmap_grid_.cells();
  costmap_publisher_->publish(grid_message);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
