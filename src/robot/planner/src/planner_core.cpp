#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

namespace robot {

namespace {

constexpr int blocked_cost = 50;
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

bool PlannerCore::canUseCell(const nav_msgs::msg::OccupancyGrid& map, Cell cell) const {
  if (cell.x < 0 || cell.y < 0 ||
      cell.x >= static_cast<int>(map.info.width) ||
      cell.y >= static_cast<int>(map.info.height)) {
    return false;
  }
  return map.data[cellIndex(map, cell)] < blocked_cost;
}

double PlannerCore::segmentCost(const nav_msgs::msg::OccupancyGrid& map,
                                Cell start, Cell end) const {
  const double length = std::hypot(end.x - start.x, end.y - start.y);
  const int samples = std::max(1, static_cast<int>(std::ceil(length * 2.0)));
  double multiplier_sum = 0.0;
  Cell previous = start;

  for (int step = 0; step <= samples; ++step) {
    const double fraction = static_cast<double>(step) / samples;
    const Cell cell = {
        static_cast<int>(std::lround(start.x + (end.x - start.x) * fraction)),
        static_cast<int>(std::lround(start.y + (end.y - start.y) * fraction))};
    if (!canUseCell(map, cell)) {
      return infinity;
    }
    if (cell.x != previous.x && cell.y != previous.y &&
        (!canUseCell(map, {cell.x, previous.y}) ||
         !canUseCell(map, {previous.x, cell.y}))) {
      return infinity;  // Do not cut diagonally through an obstacle corner.
    }

    const int cost = map.data[cellIndex(map, cell)];
    multiplier_sum += cost < 0 ? unknown_multiplier : 1.0 + cost / 25.0;
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

  Cell start_cell;
  Cell goal_cell;
  if (!worldToCell(map, start, start_cell) ||
      !worldToCell(map, goal, goal_cell) ||
      !canUseCell(map, start_cell) || !canUseCell(map, goal_cell)) {
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
        if (!canUseCell(map, neighbor)) {
          continue;
        }
        const int neighbor_index = cellIndex(map, neighbor);
        if (visited[neighbor_index]) {
          continue;
        }

        // Theta*: try a straight connection from the current cell's parent.
        int new_parent = parent[current.index];
        double travel_cost = segmentCost(map, indexToCell(map, new_parent), neighbor);
        if (!std::isfinite(travel_cost)) {
          new_parent = current.index;
          travel_cost = segmentCost(map, current_cell, neighbor);
        }
        const double new_cost = best_cost[new_parent] + travel_cost;
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
        !std::isfinite(segmentCost(map, previous, next))) {
      return false;
    }
    previous = next;
  }
  return canUseCell(map, previous);
}

}  // namespace robot
