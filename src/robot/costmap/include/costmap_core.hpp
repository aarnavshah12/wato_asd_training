#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot {

// A square grid centered on the laser. Each cell covers 10 cm.
class CostmapCore {
 public:
  static constexpr float kResolution = 0.1f;
  static constexpr int kWidth = 400;
  static constexpr int kHeight = 400;
  static constexpr float kInflationRadius = 1.0f;

  CostmapCore();

  void initializeCostmap();
  bool convertToGrid(double range, double angle, int& x_grid, int& y_grid) const;
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();

  const std::vector<int8_t>& data() const { return data_; }

 private:
  bool inBounds(int x_grid, int y_grid) const;
  std::size_t index(int x_grid, int y_grid) const;

  std::vector<int8_t> data_;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_
