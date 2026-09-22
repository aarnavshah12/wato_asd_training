#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot {

CostmapCore::CostmapCore() : grid_cells_(grid_width_cells * grid_height_cells, 0) {}

void CostmapCore::initializeCostmap() {
  std::fill(grid_cells_.begin(), grid_cells_.end(), 0);
}

bool CostmapCore::convertToGrid(
    double distance_m, double angle_rad, int& cell_x, int& cell_y) const {
  const double scan_x_m = distance_m * std::cos(angle_rad);
  const double scan_y_m = distance_m * std::sin(angle_rad);

  // The laser is at the center of the grid, rather than at its lower-left corner.
  cell_x = static_cast<int>(std::floor(scan_x_m / grid_resolution_m + grid_width_cells / 2.0));
  cell_y = static_cast<int>(std::floor(scan_y_m / grid_resolution_m + grid_height_cells / 2.0));
  return inside_grid(cell_x, cell_y);
}

void CostmapCore::markObstacle(int cell_x, int cell_y) {
  if (inside_grid(cell_x, cell_y)) {
    grid_cells_[cell_index(cell_x, cell_y)] = obstacle_cost;
  }
}

void CostmapCore::inflateObstacles() {
  // Save the actual hits first. Otherwise an inflated cell could inflate its neighbors again.
  std::vector<std::pair<int, int>> obstacle_cells;
  for (int cell_y = 0; cell_y < grid_height_cells; ++cell_y) {
    for (int cell_x = 0; cell_x < grid_width_cells; ++cell_x) {
      if (grid_cells_[cell_index(cell_x, cell_y)] == obstacle_cost) {
        obstacle_cells.emplace_back(cell_x, cell_y);
      }
    }
  }

  const int radius_in_cells = static_cast<int>(std::ceil(inflation_radius_m / grid_resolution_m));
  for (const auto& obstacle_cell : obstacle_cells) {
    for (int offset_y = -radius_in_cells; offset_y <= radius_in_cells; ++offset_y) {
      for (int offset_x = -radius_in_cells; offset_x <= radius_in_cells; ++offset_x) {
        const int nearby_x = obstacle_cell.first + offset_x;
        const int nearby_y = obstacle_cell.second + offset_y;
        if (!inside_grid(nearby_x, nearby_y)) {
          continue;
        }

        const double distance_m = std::hypot(offset_x, offset_y) * grid_resolution_m;
        if (distance_m > inflation_radius_m) {
          continue;
        }

        const int8_t inflated_cost = static_cast<int8_t>(
            obstacle_cost * (1.0 - distance_m / inflation_radius_m));
        const std::size_t nearby_index = cell_index(nearby_x, nearby_y);
        grid_cells_[nearby_index] = std::max(grid_cells_[nearby_index], inflated_cost);
      }
    }
  }
}

bool CostmapCore::inside_grid(int cell_x, int cell_y) const {
  return cell_x >= 0 && cell_x < grid_width_cells &&
         cell_y >= 0 && cell_y < grid_height_cells;
}

std::size_t CostmapCore::cell_index(int cell_x, int cell_y) const {
  return static_cast<std::size_t>(cell_y) * grid_width_cells + cell_x;
}

}  // namespace robot
