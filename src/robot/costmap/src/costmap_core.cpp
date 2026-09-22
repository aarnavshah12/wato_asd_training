#include "costmap_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace robot {

CostmapCore::CostmapCore() : data_(kWidth * kHeight, 0) {}

void CostmapCore::initializeCostmap() {
  std::fill(data_.begin(), data_.end(), 0);
}

bool CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) const {
  const double x = range * std::cos(angle);
  const double y = range * std::sin(angle);

  // The laser is at the center of the grid, rather than at its lower-left corner.
  x_grid = static_cast<int>(std::floor(x / kResolution + kWidth / 2.0));
  y_grid = static_cast<int>(std::floor(y / kResolution + kHeight / 2.0));
  return inBounds(x_grid, y_grid);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
  if (inBounds(x_grid, y_grid)) {
    data_[index(x_grid, y_grid)] = 100;
  }
}

void CostmapCore::inflateObstacles() {
  // Save the actual hits first. Otherwise an inflated cell could inflate its neighbors again.
  std::vector<std::pair<int, int>> obstacles;
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      if (data_[index(x, y)] == 100) {
        obstacles.emplace_back(x, y);
      }
    }
  }

  const int radius_cells = static_cast<int>(std::ceil(kInflationRadius / kResolution));
  for (const auto& obstacle : obstacles) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int x = obstacle.first + dx;
        const int y = obstacle.second + dy;
        if (!inBounds(x, y)) {
          continue;
        }

        const double distance = std::hypot(dx, dy) * kResolution;
        if (distance > kInflationRadius) {
          continue;
        }

        const int8_t cost = static_cast<int8_t>(100 * (1.0 - distance / kInflationRadius));
        data_[index(x, y)] = std::max(data_[index(x, y)], cost);
      }
    }
  }
}

bool CostmapCore::inBounds(int x_grid, int y_grid) const {
  return x_grid >= 0 && x_grid < kWidth && y_grid >= 0 && y_grid < kHeight;
}

std::size_t CostmapCore::index(int x_grid, int y_grid) const {
  return static_cast<std::size_t>(y_grid) * kWidth + x_grid;
}

}  // namespace robot
