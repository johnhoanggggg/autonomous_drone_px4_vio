#pragma once

#include <cstdint>
#include <optional>
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

// Exact continuous distance from a world-frame segment to the nearest occupied
// cell, where an occupied cell is its full axis-aligned square rather than a
// point at its centre. Obstacle-free maps return +infinity.
//
// Returns no value for invalid input, or when any part of the segment leaves
// the map or enters unknown space -- those stay blocked, they are not "far from
// an obstacle". The search window expands until the closest cell it has not yet
// visited could not improve the answer, so the cost still scales with the
// answer rather than with the map.
[[nodiscard]] std::optional<double> segment_minimum_clearance(
  const GridMap & grid,
  const Point2 & start,
  const Point2 & end,
  int occupied_threshold = 65);

// segment_minimum_clearance over the degenerate segment at `point`.
[[nodiscard]] std::optional<double> point_clearance(
  const GridMap & grid,
  const Point2 & point,
  int occupied_threshold = 65);

// Exact C++ counterpart of grid_planner.segment_has_clearance(). Occupied
// cells are full axis-aligned squares, not points at their centres. Delegates
// to the primitive above so there is only one continuous-clearance
// implementation; `required_clearance` additionally bounds the search.
[[nodiscard]] bool segment_has_clearance(
  const GridMap & grid,
  const Point2 & start,
  const Point2 & end,
  double required_clearance,
  int occupied_threshold = 65);

}  // namespace px4_vio_bridge
