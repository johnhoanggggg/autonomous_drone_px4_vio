#include "px4_vio_bridge/grid_planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>

namespace px4_vio_bridge
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

// Python's round() is round-half-to-even, and the inflation kernel and the
// grid-line rasteriser both feed it exact .5 values. nearbyint under the
// default FE_TONEAREST is the same rule; std::round (half away from zero) is
// not, and would shift kernel costs by one against the Python reference.
double py_round(double value)
{
  return std::nearbyint(value);
}

double now_ms()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

struct Move
{
  int dx;
  int dy;
  double distance;
};

// Order matters: recover_start returns the first traversable cell BFS reaches,
// so the neighbour order is part of the behaviour, not an implementation
// detail. This is grid_planner.MOVES verbatim.
const std::array<Move, 8> kMoves{{
  {-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0},
  {-1, -1, M_SQRT2}, {-1, 1, M_SQRT2},
  {1, -1, M_SQRT2}, {1, 1, M_SQRT2},
}};

double point_distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}

double point_segment_distance(const Point2 & point, const Point2 & start, const Point2 & end)
{
  const auto dx = end.first - start.first;
  const auto dy = end.second - start.second;
  const auto length_sq = dx * dx + dy * dy;
  if (length_sq <= 1.0e-18) {
    return point_distance(point, start);
  }
  const auto fraction = std::clamp(
    ((point.first - start.first) * dx + (point.second - start.second) * dy) / length_sq,
    0.0, 1.0);
  return point_distance(point, {start.first + fraction * dx, start.second + fraction * dy});
}

}  // namespace

bool CostGrid::in_bounds(const Cell & cell) const
{
  return cell.first >= 0 && cell.second >= 0 &&
         static_cast<std::size_t>(cell.first) < width &&
         static_cast<std::size_t>(cell.second) < height;
}

std::size_t CostGrid::index(const Cell & cell) const
{
  return static_cast<std::size_t>(cell.second) * width + static_cast<std::size_t>(cell.first);
}

int CostGrid::value(const Cell & cell) const
{
  return static_cast<int>(data[index(cell)]);
}

std::optional<Cell> CostGrid::world_to_cell(const Point2 & point) const
{
  const Cell cell{
    static_cast<int>(std::floor((point.first - origin_x) / resolution)),
    static_cast<int>(std::floor((point.second - origin_y) / resolution))};
  if (!in_bounds(cell)) {
    return std::nullopt;
  }
  return cell;
}

Point2 CostGrid::cell_center(const Cell & cell) const
{
  return {
    origin_x + (cell.first + 0.5) * resolution,
    origin_y + (cell.second + 0.5) * resolution};
}

std::vector<InflationOffset> inflation_offsets(
  double resolution, double lethal_radius, double inflation_radius, double cost_scaling)
{
  const auto cells = static_cast<int>(std::ceil(inflation_radius / resolution));
  std::vector<InflationOffset> offsets;
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      const auto distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution;
      if (distance > inflation_radius + 1.0e-12) {
        continue;
      }
      int cost = LETHAL;
      if (distance > lethal_radius + 1.0e-12) {
        const auto span = std::max(resolution, inflation_radius - lethal_radius);
        const auto decay = std::exp(-cost_scaling * (distance - lethal_radius) / span);
        cost = std::max(1, std::min(LETHAL - 1, static_cast<int>(py_round((LETHAL - 1) * decay))));
      }
      offsets.push_back({dx, dy, cost});
    }
  }
  return offsets;
}

CostGrid inflate_occupancy(
  const GridMap & grid, int occupied_threshold, double lethal_radius,
  double inflation_radius, double cost_scaling)
{
  if (lethal_radius < 0.0 || inflation_radius < lethal_radius) {
    throw std::invalid_argument("inflation radius must be at least the lethal radius");
  }
  const auto width = static_cast<int>(grid.width);
  const auto height = static_cast<int>(grid.height);
  std::vector<std::uint8_t> obstacles(grid.data.size(), 0);
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    obstacles[i] = static_cast<int>(grid.data[i]) >= occupied_threshold ? 1 : 0;
  }
  std::vector<std::int16_t> costs(grid.data.size(), 0);
  const auto offsets = inflation_offsets(
    grid.resolution, lethal_radius, inflation_radius, cost_scaling);
  std::size_t obstacle_count = 0;
  for (const auto flag : obstacles) {
    obstacle_count += flag;
  }
  // Both loops take the same per-offset maximum and so produce identical
  // costmaps; they differ only in which one is cheaper to walk.
  //
  // grid_planner.py dilates the whole mask once per kernel offset because in
  // numpy each pass is a single vectorised max -- the offset count is what it
  // pays for. Scalar C++ pays per *cell touched*, so the same shape costs
  // width*height*offsets (~33M on a flight grid) where stamping the kernel
  // around each obstacle costs obstacles*offsets instead. Flight maps are ~8%
  // occupied, which makes the obstacle loop ~12x cheaper; the mask loop still
  // wins on a map dense enough that the obstacles outnumber the cells, so pick
  // between them rather than assuming.
  if (obstacle_count * offsets.size() <= grid.data.size() * offsets.size() / 2) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (!obstacles[static_cast<std::size_t>(y) * grid.width + static_cast<std::size_t>(x)]) {
          continue;
        }
        for (const auto & offset : offsets) {
          const auto ty = y + offset.dy;
          const auto tx = x + offset.dx;
          if (ty < 0 || ty >= height || tx < 0 || tx >= width) {
            continue;
          }
          auto & cell = costs[static_cast<std::size_t>(ty) * grid.width +
            static_cast<std::size_t>(tx)];
          if (cell < offset.cost) {
            cell = static_cast<std::int16_t>(offset.cost);
          }
        }
      }
    }
  } else {
    for (const auto & offset : offsets) {
      const auto y0 = std::max(0, offset.dy);
      const auto y1 = std::min(height, height + offset.dy);
      const auto x0 = std::max(0, offset.dx);
      const auto x1 = std::min(width, width + offset.dx);
      if (y0 >= y1 || x0 >= x1) {
        continue;
      }
      const auto cost = static_cast<std::int16_t>(offset.cost);
      for (int y = y0; y < y1; ++y) {
        const auto target_row = static_cast<std::size_t>(y) * grid.width;
        const auto source_row = static_cast<std::size_t>(y - offset.dy) * grid.width;
        for (int x = x0; x < x1; ++x) {
          if (obstacles[source_row + static_cast<std::size_t>(x - offset.dx)] &&
            costs[target_row + static_cast<std::size_t>(x)] < cost)
          {
            costs[target_row + static_cast<std::size_t>(x)] = cost;
          }
        }
      }
    }
  }
  // Unknown space is never inflated into: it is already untraversable, and
  // overwriting it would hide the frontier the goal selector flies towards.
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (static_cast<int>(grid.data[i]) < 0) {
      costs[i] = static_cast<std::int16_t>(UNKNOWN);
    }
  }
  return CostGrid{
    grid.width, grid.height, grid.resolution, grid.origin_x, grid.origin_y, std::move(costs)};
}

std::vector<std::int8_t> inflation_display_data(
  const GridMap & source_grid, const CostGrid & inflated_grid, int occupied_threshold)
{
  if (source_grid.width != inflated_grid.width || source_grid.height != inflated_grid.height ||
    source_grid.data.size() != inflated_grid.data.size())
  {
    throw std::invalid_argument("source and inflated grids must have matching geometry");
  }
  std::vector<std::int8_t> display;
  display.reserve(source_grid.data.size());
  for (std::size_t i = 0; i < source_grid.data.size(); ++i) {
    const auto source = static_cast<int>(source_grid.data[i]);
    const auto cost = static_cast<int>(inflated_grid.data[i]);
    if (source < 0) {
      display.push_back(static_cast<std::int8_t>(UNKNOWN));
    } else if (source >= occupied_threshold) {
      display.push_back(100);
    } else if (cost >= LETHAL) {
      display.push_back(static_cast<std::int8_t>(HARD_INFLATION_DISPLAY));
    } else if (cost > 0) {
      const auto scaled = static_cast<int>(
        py_round(static_cast<double>(cost) * (HARD_INFLATION_DISPLAY - 1) / (LETHAL - 1)));
      display.push_back(
        static_cast<std::int8_t>(std::max(1, std::min(HARD_INFLATION_DISPLAY - 1, scaled))));
    } else {
      display.push_back(0);
    }
  }
  return display;
}

bool traversable(const CostGrid & grid, const Cell & cell)
{
  if (!grid.in_bounds(cell)) {
    return false;
  }
  const auto value = grid.value(cell);
  return value >= 0 && value < LETHAL;
}

double octile_heuristic(const Cell & a, const Cell & b)
{
  const auto dx = static_cast<double>(std::abs(a.first - b.first));
  const auto dy = static_cast<double>(std::abs(a.second - b.second));
  return std::max(dx, dy) + (M_SQRT2 - 1.0) * std::min(dx, dy);
}

std::optional<StartRecovery> recover_start(
  const GridMap & source_grid, const CostGrid & inflated_grid, const Cell & start,
  int occupied_threshold, double max_radius)
{
  if (traversable(inflated_grid, start)) {
    return StartRecovery{start, 0.0};
  }
  if (max_radius <= 0.0) {
    return std::nullopt;
  }
  const auto limit = static_cast<int>(std::floor(max_radius / inflated_grid.resolution));
  if (limit < 1) {
    return std::nullopt;
  }
  const auto escapable = [&](const Cell & cell) {
      if (!source_grid.in_bounds(cell.first, cell.second)) {
        return false;
      }
      const auto value = source_grid.value(cell.first, cell.second);
      return value >= 0 && value < occupied_threshold;
    };
  if (!escapable(start)) {
    return std::nullopt;
  }
  std::vector<std::uint8_t> visited(inflated_grid.data.size(), 0);
  std::deque<Cell> queue{start};
  visited[inflated_grid.index(start)] = 1;
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop_front();
    for (const auto & move : kMoves) {
      const Cell next{current.first + move.dx, current.second + move.dy};
      if (!escapable(next) || visited[inflated_grid.index(next)]) {
        continue;
      }
      const auto offset = std::hypot(
        static_cast<double>(next.first - start.first),
        static_cast<double>(next.second - start.second));
      if (offset > limit) {
        continue;
      }
      visited[inflated_grid.index(next)] = 1;
      if (traversable(inflated_grid, next)) {
        return StartRecovery{next, offset * inflated_grid.resolution};
      }
      queue.push_back(next);
    }
  }
  return std::nullopt;
}

namespace
{

// grid_planner.traversable_neighbors: eight-connected, no diagonal corner cut.
template<typename Fn>
void for_each_traversable_neighbor(const CostGrid & grid, const Cell & cell, Fn && fn)
{
  for (const auto & move : kMoves) {
    const Cell next{cell.first + move.dx, cell.second + move.dy};
    if (!traversable(grid, next)) {
      continue;
    }
    if (move.dx != 0 && move.dy != 0) {
      if (!traversable(grid, {cell.first + move.dx, cell.second}) ||
        !traversable(grid, {cell.first, cell.second + move.dy}))
      {
        continue;
      }
    }
    fn(next, move.distance);
  }
}

}  // namespace

std::optional<GoalSelection> closest_reachable_goal(
  const CostGrid & grid, const Cell & start, const Point2 & requested_world)
{
  if (!traversable(grid, start)) {
    return std::nullopt;
  }
  if (!std::isfinite(requested_world.first) || !std::isfinite(requested_world.second)) {
    throw std::invalid_argument("requested goal must be finite");
  }
  // Python compares the whole (distance, heuristic, cell) tuple, so ties fall
  // through to the cell coordinates and the winner is order-independent.
  using Key = std::tuple<double, double, int, int>;
  std::vector<std::uint8_t> visited(grid.data.size(), 0);
  std::deque<Cell> queue{start};
  visited[grid.index(start)] = 1;
  int visited_count = 1;
  Cell best = start;
  Key best_key{point_distance(grid.cell_center(start), requested_world), 0.0, start.first,
    start.second};
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop_front();
    const Key key{
      point_distance(grid.cell_center(current), requested_world),
      octile_heuristic(start, current), current.first, current.second};
    if (key < best_key) {
      best = current;
      best_key = key;
    }
    for_each_traversable_neighbor(
      grid, current, [&](const Cell & next, double) {
        if (!visited[grid.index(next)]) {
          visited[grid.index(next)] = 1;
          ++visited_count;
          queue.push_back(next);
        }
      });
  }
  return GoalSelection{best, std::get<0>(best_key), visited_count};
}

std::pair<bool, bool> classify_goal(
  const GridMap & source_grid, const CostGrid & inflated_grid,
  const Point2 & requested_world, const Cell & selected_cell)
{
  const Cell requested_cell{
    static_cast<int>(std::floor((requested_world.first - source_grid.origin_x) /
    source_grid.resolution)),
    static_cast<int>(std::floor((requested_world.second - source_grid.origin_y) /
    source_grid.resolution))};
  const auto in_map = source_grid.in_bounds(requested_cell.first, requested_cell.second);
  const auto exact = in_map && requested_cell == selected_cell;
  const auto known = in_map && source_grid.value(requested_cell.first, requested_cell.second) >= 0;
  const auto safe = in_map && traversable(inflated_grid, requested_cell);
  return {exact, exact || (known && !safe)};
}

SearchResult astar(
  const CostGrid & grid, const Cell & start, const Cell & goal,
  double heuristic_weight, double cost_weight, double timeout_ms)
{
  const auto begun = now_ms();
  if (!traversable(grid, start)) {
    return SearchResult{{}, kInf, 0, 0.0, "START_BLOCKED"};
  }
  if (!traversable(grid, goal)) {
    return SearchResult{{}, kInf, 0, 0.0, "GOAL_BLOCKED"};
  }
  if (heuristic_weight < 1.0) {
    throw std::invalid_argument("heuristic_weight must be >= 1");
  }
  // heapq orders the whole (priority, cost, cell) tuple; reproduce that so a
  // priority tie resolves the same way it does in Python.
  struct Entry
  {
    double priority;
    double cost;
    Cell cell;
    bool operator>(const Entry & other) const
    {
      return std::tie(priority, cost, cell.first, cell.second) >
             std::tie(other.priority, other.cost, other.cell.first, other.cell.second);
    }
  };
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
  frontier.push({heuristic_weight * octile_heuristic(start, goal), 0.0, start});
  std::vector<double> best(grid.data.size(), kInf);
  std::vector<int> came_from(grid.data.size(), -1);
  best[grid.index(start)] = 0.0;
  int expanded = 0;
  while (!frontier.empty()) {
    const auto elapsed_ms = now_ms() - begun;
    if (timeout_ms > 0.0 && elapsed_ms > timeout_ms) {
      return SearchResult{{}, kInf, expanded, elapsed_ms, "TIMEOUT"};
    }
    const auto entry = frontier.top();
    frontier.pop();
    const auto current = entry.cell;
    if (entry.cost > best[grid.index(current)] + 1.0e-12) {
      continue;
    }
    ++expanded;
    if (current == goal) {
      std::vector<Cell> path{current};
      auto cursor = current;
      while (cursor != start) {
        const auto previous = came_from[grid.index(cursor)];
        cursor = Cell{
          previous % static_cast<int>(grid.width), previous / static_cast<int>(grid.width)};
        path.push_back(cursor);
      }
      std::reverse(path.begin(), path.end());
      return SearchResult{std::move(path), entry.cost, expanded, now_ms() - begun, "PATH_VALID"};
    }
    for_each_traversable_neighbor(
      grid, current, [&](const Cell & next, double distance) {
        const auto cell_cost = static_cast<double>(grid.value(next)) / (LETHAL - 1);
        const auto candidate = entry.cost + distance * (1.0 + cost_weight * cell_cost);
        const auto next_index = grid.index(next);
        if (candidate + 1.0e-12 >= best[next_index]) {
          return;
        }
        best[next_index] = candidate;
        came_from[next_index] = static_cast<int>(grid.index(current));
        frontier.push(
          {candidate + heuristic_weight * octile_heuristic(next, goal), candidate, next});
      });
  }
  return SearchResult{{}, kInf, expanded, now_ms() - begun, "NO_KNOWN_PATH"};
}

namespace
{

// Shared rasteriser for line_is_clear/line_max_cost. Returns LETHAL the moment
// the line (or either side of a diagonal step) leaves traversable space.
int rasterise_max_cost(const CostGrid & grid, const Cell & start, const Cell & end)
{
  const auto dx = end.first - start.first;
  const auto dy = end.second - start.second;
  const auto steps = std::max(std::abs(dx), std::abs(dy)) * 2 + 1;
  auto previous = start;
  int highest = 0;
  for (int i = 0; i <= steps; ++i) {
    const auto t = static_cast<double>(i) / std::max(1, steps);
    const Cell cell{
      static_cast<int>(py_round(start.first + dx * t)),
      static_cast<int>(py_round(start.second + dy * t))};
    if (!traversable(grid, cell)) {
      return LETHAL;
    }
    highest = std::max(highest, grid.value(cell));
    const auto sx = cell.first - previous.first;
    const auto sy = cell.second - previous.second;
    if (sx != 0 && sy != 0) {
      for (const Cell corner : {Cell{previous.first + sx, previous.second},
          Cell{previous.first, previous.second + sy}})
      {
        if (!traversable(grid, corner)) {
          return LETHAL;
        }
        highest = std::max(highest, grid.value(corner));
      }
    }
    previous = cell;
  }
  return highest;
}

}  // namespace

bool line_is_clear(const CostGrid & grid, const Cell & start, const Cell & end)
{
  return rasterise_max_cost(grid, start, end) < LETHAL;
}

int line_max_cost(const CostGrid & grid, const Cell & start, const Cell & end)
{
  return rasterise_max_cost(grid, start, end);
}

double grid_lethal_radius(double required_clearance, double resolution)
{
  if (required_clearance < 0.0 || resolution <= 0.0) {
    throw std::invalid_argument("clearance must be non-negative and resolution positive");
  }
  return required_clearance + resolution / M_SQRT2;
}

std::vector<Cell> simplify_path(
  const CostGrid & grid, const std::vector<Cell> & cells, bool preserve_cost,
  const GridMap * source_grid, std::optional<double> required_clearance,
  int occupied_threshold)
{
  const auto clearance_ok = [&](const Cell & a, const Cell & b) {
      if (source_grid == nullptr || !required_clearance.has_value()) {
        return true;
      }
      return segment_has_clearance(
        *source_grid, grid.cell_center(a), grid.cell_center(b), *required_clearance,
        occupied_threshold);
    };
  if (cells.size() <= 2) {
    if (cells.size() == 2 && !clearance_ok(cells[0], cells[1])) {
      return {};
    }
    return cells;
  }
  std::vector<int> edge_costs;
  edge_costs.reserve(cells.size() - 1);
  for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
    edge_costs.push_back(line_max_cost(grid, cells[i], cells[i + 1]));
  }
  std::vector<Cell> simplified{cells.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    auto candidate = cells.size() - 1;
    std::optional<std::size_t> accepted;
    while (candidate > anchor) {
      const auto shortcut_cost = line_max_cost(grid, cells[anchor], cells[candidate]);
      auto valid = shortcut_cost < LETHAL;
      if (valid && preserve_cost) {
        valid = shortcut_cost <=
          *std::max_element(edge_costs.begin() + anchor, edge_costs.begin() + candidate);
      }
      if (valid) {
        valid = clearance_ok(cells[anchor], cells[candidate]);
      }
      if (valid) {
        accepted = candidate;
        break;
      }
      --candidate;
    }
    if (!accepted.has_value()) {
      return {};
    }
    simplified.push_back(cells[*accepted]);
    anchor = *accepted;
  }
  return simplified;
}

double path_length(const std::vector<Point2> & points)
{
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    total += point_distance(points[i], points[i + 1]);
  }
  return total;
}

std::optional<PathProjection> path_projection(
  const std::vector<Point2> & points, const Point2 & point)
{
  if (points.empty()) {
    return std::nullopt;
  }
  if (points.size() == 1) {
    return PathProjection{point_distance(point, points[0]), 0.0};
  }
  std::optional<std::pair<double, double>> best;
  double travelled = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & start = points[i];
    const auto & end = points[i + 1];
    const auto dx = end.first - start.first;
    const auto dy = end.second - start.second;
    const auto length = std::hypot(dx, dy);
    if (length <= 0.0) {
      continue;
    }
    const auto fraction = std::clamp(
      ((point.first - start.first) * dx + (point.second - start.second) * dy) / (length * length),
      0.0, 1.0);
    const Point2 projected{start.first + fraction * dx, start.second + fraction * dy};
    const auto distance = point_distance(point, projected);
    if (!best.has_value() || distance < best->first) {
      best = std::pair<double, double>{distance, travelled + fraction * length};
    }
    travelled += length;
  }
  if (!best.has_value()) {
    return std::nullopt;
  }
  return PathProjection{best->first, std::max(0.0, travelled - best->second)};
}

std::vector<Point2> trim_path_to(
  const std::vector<Point2> & points, const Point2 & point, double margin)
{
  if (points.size() < 2 || point_distance(point, points[0]) <= margin) {
    return points;
  }
  std::optional<std::tuple<double, std::size_t, double, Point2>> best;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & start = points[i];
    const auto & end = points[i + 1];
    const auto dx = end.first - start.first;
    const auto dy = end.second - start.second;
    const auto length_sq = dx * dx + dy * dy;
    if (length_sq <= 0.0) {
      continue;
    }
    const auto fraction = std::clamp(
      ((point.first - start.first) * dx + (point.second - start.second) * dy) / length_sq,
      0.0, 1.0);
    const Point2 projected{start.first + fraction * dx, start.second + fraction * dy};
    const auto distance = point_distance(point, projected);
    if (!best.has_value() || distance < std::get<0>(*best)) {
      best = std::make_tuple(distance, i, fraction, projected);
    }
  }
  if (!best.has_value()) {
    return points;
  }
  const auto index = std::get<1>(*best);
  const auto fraction = std::get<2>(*best);
  const auto projected = std::get<3>(*best);
  if (index == 0 && fraction <= 0.0) {
    // Lateral offset from the head, not progress along the path. Trimming
    // would only restate the same path under a new fingerprint.
    return points;
  }
  std::vector<Point2> trimmed{projected};
  trimmed.insert(trimmed.end(), points.begin() + index + 1, points.end());
  if (trimmed.size() < 2) {
    trimmed = {projected, points.back()};
  }
  return trimmed;
}

bool should_replace_path(
  const std::optional<PathProjection> & projection, double candidate_length,
  double retain_tolerance, double switch_improvement)
{
  if (!projection.has_value()) {
    return true;
  }
  if (projection->distance > retain_tolerance) {
    return true;
  }
  return candidate_length < projection->remaining * (1.0 - switch_improvement);
}


const char * to_string(GoalMode mode)
{
  switch (mode) {
    case GoalMode::PathValid:
      return "PATH_VALID";
    case GoalMode::SafeApproach:
      return "SAFE_APPROACH";
    case GoalMode::Exploring:
      break;
  }
  return "EXPLORING";
}

GoalMode goal_mode_from(bool exact, bool terminal)
{
  if (exact) {
    return GoalMode::PathValid;
  }
  return terminal ? GoalMode::SafeApproach : GoalMode::Exploring;
}

bool goal_mode_terminal(GoalMode mode)
{
  return mode != GoalMode::Exploring;
}

GoalModeHysteresis::GoalModeHysteresis(int confirmation_maps)
: confirmation_maps_(std::max(1, confirmation_maps))
{
}

void GoalModeHysteresis::reset()
{
  initialized_ = false;
  pending_.reset();
  pending_count_ = 0;
  last_counted_generation_.reset();
}

void GoalModeHysteresis::commit(GoalMode mode)
{
  stable_ = mode;
  initialized_ = true;
  pending_.reset();
  pending_count_ = 0;
  last_counted_generation_.reset();
}

ModeDecision GoalModeHysteresis::decision() const
{
  ModeDecision result;
  result.stable = stable_;
  result.confirmation_maps = confirmation_maps_;
  result.has_pending = pending_.has_value();
  result.pending = pending_.value_or(stable_);
  result.pending_count = pending_count_;
  return result;
}

ModeDecision GoalModeHysteresis::observe(GoalMode raw, std::int64_t map_generation)
{
  if (!initialized_) {
    // Nothing is committed yet, so there is no old meaning to protect.
    commit(raw);
    auto result = decision();
    result.committed = true;
    return result;
  }
  if (raw == stable_) {
    pending_.reset();
    pending_count_ = 0;
    last_counted_generation_.reset();
    return decision();
  }
  if (!pending_.has_value() || *pending_ != raw) {
    // A contradictory sample replaces the candidate rather than adding to it.
    pending_ = raw;
    pending_count_ = 1;
    last_counted_generation_ = map_generation;
  } else if (!last_counted_generation_.has_value() ||
    *last_counted_generation_ != map_generation)
  {
    // Only a genuinely new occupancy grid is new evidence. Repeated planner
    // ticks on one map must not confirm anything.
    ++pending_count_;
    last_counted_generation_ = map_generation;
  }
  if (pending_count_ >= confirmation_maps_) {
    commit(raw);
    auto result = decision();
    result.committed = true;
    return result;
  }
  return decision();
}

std::string GoalModeHysteresis::pending_suffix() const
{
  if (!pending_.has_value()) {
    return {};
  }
  return std::string(" MODE_PENDING ") + to_string(stable_) + "->" + to_string(*pending_) +
         " " + std::to_string(pending_count_) + "/" + std::to_string(confirmation_maps_);
}


PathReplacementDecision decide_path_replacement(const PathReplacementInputs & inputs)
{
  PathReplacementDecision decision;
  decision.off_corridor = !inputs.projection.has_value() ||
    inputs.projection->distance > inputs.retain_tolerance;
  decision.transition_hold = inputs.mode_transition_pending && inputs.retained_safe &&
    !decision.off_corridor && !inputs.goal_changed;
  decision.replace = !decision.transition_hold &&
    (inputs.goal_changed || inputs.effective_goal_changed ||
    should_replace_path(
      inputs.projection, inputs.candidate_length, inputs.retain_tolerance,
      inputs.switch_improvement));
  return decision;
}

}  // namespace px4_vio_bridge
