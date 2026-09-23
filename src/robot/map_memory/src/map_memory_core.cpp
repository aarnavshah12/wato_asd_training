#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace robot {

MapMemoryCore::MapMemoryCore(double map_size_m) : map_size_m_(map_size_m) {}

bool MapMemoryCore::initializeMap(const nav_msgs::msg::OccupancyGrid& costmap,
                                  const nav_msgs::msg::Odometry& odometry) {
  if (costmap.info.resolution <= 0.0 || map_size_m_ <= 0.0 ||
      odometry.header.frame_id.empty()) {
    return false;
  }

  const auto side_cells = static_cast<unsigned int>(
      std::ceil(map_size_m_ / costmap.info.resolution));
  if (side_cells == 0) {
    return false;
  }

  global_map_.header.frame_id = odometry.header.frame_id;
  global_map_.info.resolution = costmap.info.resolution;
  global_map_.info.width = side_cells;
  global_map_.info.height = side_cells;
  global_map_.info.map_load_time = costmap.header.stamp;
  const double half_map_size_m = side_cells * costmap.info.resolution / 2.0;
  global_map_.info.origin.position.x = -half_map_size_m;
  global_map_.info.origin.position.y = -half_map_size_m;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(static_cast<std::size_t>(side_cells) * side_cells, -1);
  return true;
}

bool MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     const nav_msgs::msg::Odometry& odometry) {
  const std::size_t expected_cells =
      static_cast<std::size_t>(costmap.info.width) * costmap.info.height;
  if (costmap.data.size() != expected_cells || costmap.info.resolution <= 0.0 ||
      costmap.header.frame_id != odometry.child_frame_id ||
      odometry.header.frame_id.empty()) {
    return false;
  }

  if (global_map_.data.empty() && !initializeMap(costmap, odometry)) {
    return false;
  }
  if (global_map_.header.frame_id != odometry.header.frame_id ||
      global_map_.info.resolution != costmap.info.resolution) {
    return false;
  }

  const auto& position = odometry.pose.pose.position;
  const auto& rotation = odometry.pose.pose.orientation;
  const double yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double local_resolution = costmap.info.resolution;
  const double map_resolution = global_map_.info.resolution;

  for (unsigned int cell_y = 0; cell_y < costmap.info.height; ++cell_y) {
    for (unsigned int cell_x = 0; cell_x < costmap.info.width; ++cell_x) {
      const std::size_t local_index =
          static_cast<std::size_t>(cell_y) * costmap.info.width + cell_x;
      const int8_t cost = costmap.data[local_index];
      if (cost < 0) {
        continue;  // Unknown local cells leave older map data alone.
      }

      // Find this local cell's centre, then rotate and move it into the world frame.
      const double local_x = costmap.info.origin.position.x +
                             (cell_x + 0.5) * local_resolution;
      const double local_y = costmap.info.origin.position.y +
                             (cell_y + 0.5) * local_resolution;
      const double world_x = position.x + cos_yaw * local_x - sin_yaw * local_y;
      const double world_y = position.y + sin_yaw * local_x + cos_yaw * local_y;
      const int map_x = static_cast<int>(std::floor(
          (world_x - global_map_.info.origin.position.x) / map_resolution));
      const int map_y = static_cast<int>(std::floor(
          (world_y - global_map_.info.origin.position.y) / map_resolution));
      if (map_x < 0 || map_y < 0 ||
          map_x >= static_cast<int>(global_map_.info.width) ||
          map_y >= static_cast<int>(global_map_.info.height)) {
        continue;
      }

      const std::size_t map_index =
          static_cast<std::size_t>(map_y) * global_map_.info.width + map_x;
      // Obstacles are static here. A later scan may not see one, so keep it.
      global_map_.data[map_index] = std::max(global_map_.data[map_index], cost);
    }
  }

  global_map_.header.stamp = costmap.header.stamp;
  return true;
}

}  // namespace robot
