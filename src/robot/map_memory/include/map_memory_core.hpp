#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot {

class MapMemoryCore {
 public:
  explicit MapMemoryCore(double map_size_m);

  bool integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                        const nav_msgs::msg::Odometry& odometry);
  const nav_msgs::msg::OccupancyGrid& map() const { return global_map_; }

 private:
  bool initializeMap(const nav_msgs::msg::OccupancyGrid& costmap,
                     const nav_msgs::msg::Odometry& odometry);

  double map_size_m_;
  nav_msgs::msg::OccupancyGrid global_map_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_
