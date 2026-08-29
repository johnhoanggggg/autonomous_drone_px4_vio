#include "px4_vio_bridge/grid_clearance.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace px4_vio_bridge
{
namespace
{

double orientation(const Point2 & a, const Point2 & b, const Point2 & c)
{
  return (b.first - a.first) * (c.second - a.second) -
         (b.second - a.second) * (c.first - a.first);
}

bool on_segment(const Point2 & a, const Point2 & b, const Point2 & point)
{
  constexpr double tolerance = 1.0e-12;
  return point.first >= std::min(a.first, b.first) - tolerance &&
         point.first <= std::max(a.first, b.first) + tolerance &&
         point.second >= std::min(a.second, b.second) - tolerance &&
         point.second <= std::max(a.second, b.second) + tolerance &&
         std::abs(orientation(a, b, point)) <= tolerance;
}

bool segments_intersect(
  const Point2 & a, const Point2 & b, const Point2 & c, const Point2 & d)
{
  const auto oa = orientation(a, b, c);
  const auto ob = orientation(a, b, d);
  const auto oc = orientation(c, d, a);
  const auto od = orientation(c, d, b);
  const auto opposite = [](double first, double second) {
      return (first > 0.0 && second < 0.0) || (first < 0.0 && second > 0.0);
    };
  if (opposite(oa, ob) && opposite(oc, od)) {
    return true;
  }
  constexpr double tolerance = 1.0e-12;
  return (std::abs(oa) <= tolerance && on_segment(a, b, c)) ||
         (std::abs(ob) <= tolerance && on_segment(a, b, d)) ||
         (std::abs(oc) <= tolerance && on_segment(c, d, a)) ||
         (std::abs(od) <= tolerance && on_segment(c, d, b));
}

double point_segment_distance(
  const Point2 & point, const Point2 & start, const Point2 & end)
{
  const auto dx = end.first - start.first;
  const auto dy = end.second - start.second;
  const auto length_sq = dx * dx + dy * dy;
  if (length_sq <= 1.0e-18) {
    return std::hypot(point.first - start.first, point.second - start.second);
  }
  const auto fraction = std::clamp(
    ((point.first - start.first) * dx + (point.second - start.second) * dy) /
    length_sq, 0.0, 1.0);
  const Point2 projection{
    start.first + fraction * dx, start.second + fraction * dy};
  return std::hypot(point.first - projection.first, point.second - projection.second);
}

double segment_segment_distance(
  const Point2 & a, const Point2 & b, const Point2 & c, const Point2 & d)
{
  if (segments_intersect(a, b, c, d)) {
    return 0.0;
  }
  return std::min({
    point_segment_distance(a, c, d), point_segment_distance(b, c, d),
    point_segment_distance(c, a, b), point_segment_distance(d, a, b)});
}

double segment_box_distance(
  const Point2 & start, const Point2 & end,
  double x0, double x1, double y0, double y1)
{
  const auto inside = [=](const Point2 & point) {
      return point.first >= x0 && point.first <= x1 &&
             point.second >= y0 && point.second <= y1;
    };
  if (inside(start) || inside(end)) {
    return 0.0;
  }
  const std::array<Point2, 4> corners{{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}};
  auto result = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < corners.size(); ++index) {
    result = std::min(
      result,
      segment_segment_distance(start, end, corners[index], corners[(index + 1) % 4]));
  }
  return result;
}

bool finite(const Point2 & point)
{
  return std::isfinite(point.first) && std::isfinite(point.second);
}

// Shared continuous-clearance implementation.
//
// `window` bounds how far from the segment occupied cells are looked for. Any
// cell outside the segment's bounding box grown by `window` lies at least
// `window` away, so a result at or below the window is already exact. With no
// window supplied the search doubles until that holds, which makes the answer
// exact; with one supplied the caller only wants a threshold decision and a
// single bounded pass suffices.
std::optional<double> clearance_impl(
  const GridMap & grid, const Point2 & start, const Point2 & end,
  int occupied_threshold, std::optional<double> window_limit)
{
  if (!grid.valid() || !finite(start) || !finite(end) ||
    occupied_threshold < 0 || occupied_threshold > 100)
  {
    return std::nullopt;
  }

  const auto world_to_cell = [&grid](const Point2 & point) -> std::pair<int, int> {
      return {
        static_cast<int>(std::floor((point.first - grid.origin_x) / grid.resolution)),
        static_cast<int>(std::floor((point.second - grid.origin_y) / grid.resolution))};
    };
  const auto length = std::hypot(end.first - start.first, end.second - start.second);
  const auto known_steps = std::max(1, static_cast<int>(std::ceil(length * 2.0 / grid.resolution)));
  for (int index = 0; index <= known_steps; ++index) {
    const auto fraction = static_cast<double>(index) / known_steps;
    const Point2 point{
      start.first + fraction * (end.first - start.first),
      start.second + fraction * (end.second - start.second)};
    const auto [x, y] = world_to_cell(point);
    // Unknown and outside-map space stay blocked: they have no clearance, they
    // are not merely far from an obstacle.
    if (!grid.in_bounds(x, y) || grid.value(x, y) < 0) {
      return std::nullopt;
    }
  }

  const auto scan = [&](double window) -> std::optional<double> {
      auto x0 = static_cast<int>(std::floor(
          (std::min(start.first, end.first) - window - grid.origin_x) / grid.resolution));
      auto x1 = static_cast<int>(std::floor(
          (std::max(start.first, end.first) + window - grid.origin_x) / grid.resolution));
      auto y0 = static_cast<int>(std::floor(
          (std::min(start.second, end.second) - window - grid.origin_y) / grid.resolution));
      auto y1 = static_cast<int>(std::floor(
          (std::max(start.second, end.second) + window - grid.origin_y) / grid.resolution));
      x0 = std::max(0, x0);
      y0 = std::max(0, y0);
      x1 = std::min(static_cast<int>(grid.width) - 1, x1);
      y1 = std::min(static_cast<int>(grid.height) - 1, y1);
      if (x0 > x1 || y0 > y1) {
        return std::nullopt;
      }
      auto best = std::numeric_limits<double>::infinity();
      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          if (grid.value(x, y) < occupied_threshold) {
            continue;
          }
          const auto cell_x0 = grid.origin_x + x * grid.resolution;
          const auto cell_y0 = grid.origin_y + y * grid.resolution;
          best = std::min(
            best,
            segment_box_distance(
              start, end, cell_x0, cell_x0 + grid.resolution,
              cell_y0, cell_y0 + grid.resolution));
          if (best <= 0.0) {
            return 0.0;
          }
        }
      }
      return best;
    };

  if (window_limit.has_value()) {
    return scan(*window_limit);
  }
  const auto diagonal = std::hypot(
    static_cast<double>(grid.width) * grid.resolution,
    static_cast<double>(grid.height) * grid.resolution);
  for (auto window = grid.resolution; ; window = std::min(window * 2.0, diagonal)) {
    const auto best = scan(window);
    if (!best.has_value()) {
      return std::nullopt;
    }
    if (*best <= window || window >= diagonal) {
      return best;
    }
  }
}

}  // namespace

bool GridMap::valid() const
{
  return width > 0 && height > 0 && std::isfinite(resolution) && resolution > 0.0 &&
         std::isfinite(origin_x) && std::isfinite(origin_y) &&
         data.size() == width * height;
}

bool GridMap::in_bounds(int x, int y) const
{
  return x >= 0 && y >= 0 && static_cast<std::size_t>(x) < width &&
         static_cast<std::size_t>(y) < height;
}

int GridMap::value(int x, int y) const
{
  return static_cast<int>(data[static_cast<std::size_t>(y) * width +
                               static_cast<std::size_t>(x)]);
}

std::optional<double> segment_minimum_clearance(
  const GridMap & grid, const Point2 & start, const Point2 & end, int occupied_threshold)
{
  return clearance_impl(grid, start, end, occupied_threshold, std::nullopt);
}

std::optional<double> point_clearance(
  const GridMap & grid, const Point2 & point, int occupied_threshold)
{
  return clearance_impl(grid, point, point, occupied_threshold, std::nullopt);
}

bool segment_has_clearance(
  const GridMap & grid, const Point2 & start, const Point2 & end,
  double required_clearance,
  int occupied_threshold)
{
  if (!std::isfinite(required_clearance) || required_clearance < 0.0) {
    return false;
  }
  // The window only has to reach `required_clearance`: any cell it excludes is
  // at least that far away, so a bounded scan still decides the threshold.
  const auto clearance =
    clearance_impl(grid, start, end, occupied_threshold, required_clearance);
  return clearance.has_value() && *clearance + 1.0e-9 >= required_clearance;
}

}  // namespace px4_vio_bridge
