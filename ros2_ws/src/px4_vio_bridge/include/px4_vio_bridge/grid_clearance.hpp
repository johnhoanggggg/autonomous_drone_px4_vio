#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace px4_vio_bridge
{

struct GridMap
{
  std::size_t width{};
  std::size_t height{};
  double resolution{};
  double origin_x{};
  double origin_y{};
  std::vector<std::int8_t> data;

  [[nodiscard]] bool valid() const;
  [[nodiscard]] bool in_bounds(int x, int y) const;
  [[nodiscard]] int value(int x, int y) const;
};

using Point2 = std::pair<double, double>;

// Exact C++ counterpart of grid_planner.segment_has_clearance(). Occupied
// cells are full axis-aligned squares, not points at their centres.
[[nodiscard]] bool segment_has_clearance(
  const GridMap & grid,
  const Point2 & start,
  const Point2 & end,
  double required_clearance,
  int occupied_threshold = 65);

}  // namespace px4_vio_bridge
