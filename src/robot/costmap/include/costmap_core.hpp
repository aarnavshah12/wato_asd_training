#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot {

// A square grid centered on the laser. Each cell covers 10 cm.
class CostmapCore {
 public:
  static constexpr float grid_resolution_m = 0.1f;
  static constexpr int grid_width_cells = 400;
  static constexpr int grid_height_cells = 400;
  static constexpr float inflation_radius_m = 1.0f;
  static constexpr int8_t obstacle_cost = 100;

  CostmapCore();

  void initializeCostmap();
  bool convertToGrid(double distance_m, double angle_rad, int& cell_x, int& cell_y) const;
  void markObstacle(int cell_x, int cell_y);
  void inflateObstacles();

  const std::vector<int8_t>& cells() const { return grid_cells_; }

 private:
  bool inside_grid(int cell_x, int cell_y) const;
  std::size_t cell_index(int cell_x, int cell_y) const;

  std::vector<int8_t> grid_cells_;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_
