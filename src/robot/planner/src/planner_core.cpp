#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

namespace robot {

namespace {

constexpr int blocked_cost = 50;
// Prefer room for the whole car, but let it leave a close starting position.
constexpr double preferred_clearance_m = 2.3;
constexpr double minimum_clearance_m = 1.4;
// Unknown space is possible to use, but explored space is usually a better route.
constexpr double unknown_multiplier = 3.0;
constexpr double infinity = std::numeric_limits<double>::infinity();

struct OpenCell {
  int index;
  double path_cost;
  double total_cost;
};

struct HigherCostFirst {
  bool operator()(const OpenCell& left, const OpenCell& right) const {
    return left.total_cost > right.total_cost;
  }
};

}  // namespace

int PlannerCore::cellIndex(const nav_msgs::msg::OccupancyGrid& map, Cell cell) const {
  return cell.y * static_cast<int>(map.info.width) + cell.x;
}

PlannerCore::Cell PlannerCore::indexToCell(
    const nav_msgs::msg::OccupancyGrid& map, int index) const {
  return {index % static_cast<int>(map.info.width),
          index / static_cast<int>(map.info.width)};
}

bool PlannerCore::worldToCell(const nav_msgs::msg::OccupancyGrid& map,
                              const geometry_msgs::msg::Point& point, Cell& cell) const {
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return false;
  }
  const double grid_x =
      (point.x - map.info.origin.position.x) / map.info.resolution;
  const double grid_y =
      (point.y - map.info.origin.position.y) / map.info.resolution;
  if (grid_x < 0.0 || grid_y < 0.0 ||
      grid_x >= map.info.width || grid_y >= map.info.height) {
    return false;
  }
  cell = {static_cast<int>(std::floor(grid_x)),
          static_cast<int>(std::floor(grid_y))};
  return true;
}

geometry_msgs::msg::Point PlannerCore::cellToWorld(
    const nav_msgs::msg::OccupancyGrid& map, Cell cell) const {
  geometry_msgs::msg::Point point;
  point.x = map.info.origin.position.x +
            (cell.x + 0.5) * map.info.resolution;
  point.y = map.info.origin.position.y +
            (cell.y + 0.5) * map.info.resolution;
  return point;
}

std::vector<uint8_t> PlannerCore::clearanceCosts(
    const nav_msgs::msg::OccupancyGrid& map) const {
  std::vector<uint8_t> clearance_costs(map.data.size(), 0);
  const int radius_cells =
      static_cast<int>(std::ceil(preferred_clearance_m / map.info.resolution));

  struct OffsetCost {
    int x;
    int y;
    uint8_t cost;
  };
  std::vector<OffsetCost> nearby_offsets;
  for (int offset_y = -radius_cells; offset_y <= radius_cells; ++offset_y) {
    for (int offset_x = -radius_cells; offset_x <= radius_cells; ++offset_x) {
      const double distance_m =
          std::hypot(offset_x, offset_y) * map.info.resolution;
      if (distance_m <= preferred_clearance_m) {
        const auto cost = static_cast<uint8_t>(std::lround(
            100.0 * (1.0 - distance_m / preferred_clearance_m)));
        nearby_offsets.push_back({offset_x, offset_y, cost});
      }
    }
  }

  // Keep extra room for the car, without making the published map look larger.
  for (int cell_y = 0; cell_y < static_cast<int>(map.info.height); ++cell_y) {
    for (int cell_x = 0; cell_x < static_cast<int>(map.info.width); ++cell_x) {
      if (map.data[cellIndex(map, {cell_x, cell_y})] != 100) {
        continue;
      }
      for (const auto& offset : nearby_offsets) {
        const int nearby_x = cell_x + offset.x;
        const int nearby_y = cell_y + offset.y;
        if (nearby_x >= 0 && nearby_y >= 0 &&
            nearby_x < static_cast<int>(map.info.width) &&
            nearby_y < static_cast<int>(map.info.height)) {
          const int index = cellIndex(map, {nearby_x, nearby_y});
          clearance_costs[index] =
              std::max(clearance_costs[index], offset.cost);
        }
      }
    }
  }
  return clearance_costs;
}

bool PlannerCore::canUseCell(const nav_msgs::msg::OccupancyGrid& map, Cell cell,
                             const std::vector<uint8_t>& clearance_costs) const {
  if (cell.x < 0 || cell.y < 0 ||
      cell.x >= static_cast<int>(map.info.width) ||
      cell.y >= static_cast<int>(map.info.height)) {
    return false;
  }
  const int index = cellIndex(map, cell);
  const int minimum_clearance_cost = static_cast<int>(std::floor(
      100.0 * (1.0 - minimum_clearance_m / preferred_clearance_m)));
  return map.data[index] < blocked_cost &&
         clearance_costs[index] < minimum_clearance_cost;
}

double PlannerCore::segmentCost(const nav_msgs::msg::OccupancyGrid& map,
                                Cell start, Cell end,
                                const std::vector<uint8_t>& clearance_costs) const {
  const double length = std::hypot(end.x - start.x, end.y - start.y);
  const int samples = std::max(1, static_cast<int>(std::ceil(length * 2.0)));
  double multiplier_sum = 0.0;
  Cell previous = start;

  for (int step = 0; step <= samples; ++step) {
    const double fraction = static_cast<double>(step) / samples;
    const Cell cell = {
        static_cast<int>(std::lround(start.x + (end.x - start.x) * fraction)),
        static_cast<int>(std::lround(start.y + (end.y - start.y) * fraction))};
    if (!canUseCell(map, cell, clearance_costs)) {
      return infinity;
    }
    if (cell.x != previous.x && cell.y != previous.y &&
        (!canUseCell(map, {cell.x, previous.y}, clearance_costs) ||
         !canUseCell(map, {previous.x, cell.y}, clearance_costs))) {
      return infinity;  // Do not cut diagonally through an obstacle corner.
    }

    const int index = cellIndex(map, cell);
    const int cost = map.data[index];
    const double map_multiplier =
        cost < 0 ? unknown_multiplier : 1.0 + cost / 25.0;
    multiplier_sum += map_multiplier + clearance_costs[index] / 5.0;
    previous = cell;
  }
  return length * multiplier_sum / (samples + 1);
}

std::vector<geometry_msgs::msg::Point> PlannerCore::findPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal) const {
  const std::size_t cell_count =
      static_cast<std::size_t>(map.info.width) * map.info.height;
  if (map.info.resolution <= 0.0 || map.info.width == 0 || map.info.height == 0 ||
      map.data.size() != cell_count ||
      cell_count > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return {};
  }

  const auto clearance_costs = clearanceCosts(map);
  Cell start_cell;
  Cell goal_cell;
  if (!worldToCell(map, start, start_cell) ||
      !worldToCell(map, goal, goal_cell) ||
      !canUseCell(map, start_cell, clearance_costs) ||
      !canUseCell(map, goal_cell, clearance_costs)) {
    return {};
  }

  const int start_index = cellIndex(map, start_cell);
  const int goal_index = cellIndex(map, goal_cell);
  std::vector<double> best_cost(cell_count, infinity);
  std::vector<int> parent(cell_count, -1);
  std::vector<uint8_t> visited(cell_count, 0);
  std::priority_queue<OpenCell, std::vector<OpenCell>, HigherCostFirst> open_cells;
  best_cost[start_index] = 0.0;
  parent[start_index] = start_index;
  open_cells.push({start_index, 0.0,
                   std::hypot(goal_cell.x - start_cell.x, goal_cell.y - start_cell.y)});

  while (!open_cells.empty()) {
    const OpenCell current = open_cells.top();
    open_cells.pop();
    if (visited[current.index] || current.path_cost > best_cost[current.index]) {
      continue;
    }
    if (current.index == goal_index) {
      break;
    }
    visited[current.index] = 1;
    const Cell current_cell = indexToCell(map, current.index);

    for (int offset_y = -1; offset_y <= 1; ++offset_y) {
      for (int offset_x = -1; offset_x <= 1; ++offset_x) {
        if (offset_x == 0 && offset_y == 0) {
          continue;
        }
        const Cell neighbor = {current_cell.x + offset_x,
                               current_cell.y + offset_y};
        if (!canUseCell(map, neighbor, clearance_costs)) {
          continue;
        }
        const int neighbor_index = cellIndex(map, neighbor);
        if (visited[neighbor_index]) {
          continue;
        }

        // Compare a normal step with a straight Theta* shortcut.
        int new_parent = current.index;
        double new_cost = best_cost[current.index] +
            segmentCost(map, current_cell, neighbor, clearance_costs);
        const int shortcut_parent = parent[current.index];
        const double shortcut_cost = best_cost[shortcut_parent] +
            segmentCost(map, indexToCell(map, shortcut_parent),
                        neighbor, clearance_costs);
        if (shortcut_cost <= new_cost) {
          new_parent = shortcut_parent;
          new_cost = shortcut_cost;
        }
        if (new_cost >= best_cost[neighbor_index]) {
          continue;
        }

        best_cost[neighbor_index] = new_cost;
        parent[neighbor_index] = new_parent;
        const double distance_to_goal =
            std::hypot(goal_cell.x - neighbor.x, goal_cell.y - neighbor.y);
        open_cells.push({neighbor_index, new_cost, new_cost + distance_to_goal});
      }
    }
  }

  if (parent[goal_index] < 0) {
    return {};
  }

  std::vector<geometry_msgs::msg::Point> path;
  int current_index = goal_index;
  for (std::size_t step = 0; step < cell_count; ++step) {
    path.push_back(cellToWorld(map, indexToCell(map, current_index)));
    if (current_index == start_index) {
      break;
    }
    current_index = parent[current_index];
    if (current_index < 0) {
      return {};
    }
  }
  if (current_index != start_index) {
    return {};
  }
  std::reverse(path.begin(), path.end());
  path.front() = start;
  path.back() = goal;
  return path;
}

bool PlannerCore::pathIsClear(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Point& robot_position,
    const std::vector<geometry_msgs::msg::Point>& path) const {
  if (path.empty()) {
    return false;
  }
  const std::size_t cell_count =
      static_cast<std::size_t>(map.info.width) * map.info.height;
  if (map.info.resolution <= 0.0 || map.info.width == 0 || map.info.height == 0 ||
      map.data.size() != cell_count) {
    return false;
  }
  const auto clearance_costs = clearanceCosts(map);

  // Only check the route ahead of the robot. Old waypoints are already passed.
  std::size_t nearest_segment = 0;
  double nearest_fraction = 0.0;
  double nearest_distance = infinity;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const auto& start = path[index];
    const auto& end = path[index + 1];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    const double fraction = length_squared > 0.0
        ? std::clamp(((robot_position.x - start.x) * dx +
                      (robot_position.y - start.y) * dy) / length_squared,
                     0.0, 1.0)
        : 0.0;
    const double distance = std::hypot(
        robot_position.x - (start.x + fraction * dx),
        robot_position.y - (start.y + fraction * dy));
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_segment = index;
      nearest_fraction = fraction;
    }
  }

  geometry_msgs::msg::Point current_point = path[nearest_segment];
  if (nearest_segment + 1 < path.size()) {
    const auto& end = path[nearest_segment + 1];
    current_point.x += nearest_fraction * (end.x - current_point.x);
    current_point.y += nearest_fraction * (end.y - current_point.y);
  }
  Cell previous;
  if (!worldToCell(map, current_point, previous)) {
    return false;
  }
  for (std::size_t index = nearest_segment + 1; index < path.size(); ++index) {
    Cell next;
    if (!worldToCell(map, path[index], next) ||
        !std::isfinite(segmentCost(map, previous, next, clearance_costs))) {
      return false;
    }
    previous = next;
  }
  return canUseCell(map, previous, clearance_costs);
}

}  // namespace robot
