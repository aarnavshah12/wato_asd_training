#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot {

class PlannerCore {
 public:
  std::vector<geometry_msgs::msg::Point> findPath(
      const nav_msgs::msg::OccupancyGrid& map,
      const geometry_msgs::msg::Point& start,
      const geometry_msgs::msg::Point& goal) const;

  bool pathIsClear(const nav_msgs::msg::OccupancyGrid& map,
                   const geometry_msgs::msg::Point& robot_position,
                   const std::vector<geometry_msgs::msg::Point>& path) const;

 private:
  struct Cell {
    int x;
    int y;
  };

  bool worldToCell(const nav_msgs::msg::OccupancyGrid& map,
                   const geometry_msgs::msg::Point& point, Cell& cell) const;
  geometry_msgs::msg::Point cellToWorld(
      const nav_msgs::msg::OccupancyGrid& map, Cell cell) const;
  std::vector<uint8_t> clearanceCosts(const nav_msgs::msg::OccupancyGrid& map) const;
  bool canUseCell(const nav_msgs::msg::OccupancyGrid& map, Cell cell,
                  const std::vector<uint8_t>& clearance_costs) const;
  double segmentCost(const nav_msgs::msg::OccupancyGrid& map,
                     Cell start, Cell end,
                     const std::vector<uint8_t>& clearance_costs) const;
  int cellIndex(const nav_msgs::msg::OccupancyGrid& map, Cell cell) const;
  Cell indexToCell(const nav_msgs::msg::OccupancyGrid& map, int index) const;
};

}  // namespace robot

#endif  // PLANNER_CORE_HPP_
