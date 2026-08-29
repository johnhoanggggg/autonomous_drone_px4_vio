#pragma once

// Exact C++ counterpart of px4_vio_bridge/grid_planner.py.
//
// Every function here is a line-for-line port of the Python original and is
// held to it by test/test_grid_planner_parity.py, which drives both through
// randomized maps and compares costs, expansions and cells. Where Python's
// semantics are load-bearing -- tuple ordering in the A* heap, round-half-to-
// even in the inflation kernel -- the port reproduces them deliberately rather
// than substituting the C++ idiom. Read the Python docstrings for the *why*;
// the comments here only flag where the two languages would otherwise diverge.

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "px4_vio_bridge/grid_clearance.hpp"

namespace px4_vio_bridge
{

inline constexpr int UNKNOWN = -1;
inline constexpr int LETHAL = 255;
inline constexpr int HARD_INFLATION_DISPLAY = 70;

using Cell = std::pair<int, int>;

// Inflated costs reach LETHAL (255), which does not fit the int8 occupancy of
// GridMap, so the costmap carries its own int16 storage. Geometry fields mirror
// GridMap exactly so the two index identically.
struct CostGrid
{
  std::size_t width{};
  std::size_t height{};
  double resolution{};
  double origin_x{};
  double origin_y{};
  std::vector<std::int16_t> data;

  [[nodiscard]] bool in_bounds(const Cell & cell) const;
  [[nodiscard]] int value(const Cell & cell) const;
  [[nodiscard]] std::size_t index(const Cell & cell) const;
  [[nodiscard]] std::optional<Cell> world_to_cell(const Point2 & point) const;
  [[nodiscard]] Point2 cell_center(const Cell & cell) const;
};

struct SearchResult
{
  std::vector<Cell> cells;
  double cost{};
  int expanded{};
  double elapsed_ms{};
  std::string reason;

  [[nodiscard]] bool found() const {return !cells.empty();}
};

struct GoalSelection
{
  Cell cell{};
  double distance{};
  int reachable_cells{};
};

struct StartRecovery
{
  Cell cell{};
  double distance{};
};

struct PathProjection
{
  double distance{};
  double remaining{};
};

struct InflationOffset
{
  int dx{};
  int dy{};
  int cost{};
};

[[nodiscard]] std::vector<InflationOffset> inflation_offsets(
  double resolution, double lethal_radius, double inflation_radius, double cost_scaling);

[[nodiscard]] CostGrid inflate_occupancy(
  const GridMap & grid,
  int occupied_threshold = 65,
  double lethal_radius = 0.40,
  double inflation_radius = 0.60,
  double cost_scaling = 3.0);

[[nodiscard]] std::vector<std::int8_t> inflation_display_data(
  const GridMap & source_grid, const CostGrid & inflated_grid, int occupied_threshold = 65);

[[nodiscard]] bool traversable(const CostGrid & grid, const Cell & cell);

[[nodiscard]] double octile_heuristic(const Cell & a, const Cell & b);

[[nodiscard]] std::optional<StartRecovery> recover_start(
  const GridMap & source_grid, const CostGrid & inflated_grid, const Cell & start,
  int occupied_threshold = 65, double max_radius = 0.0);

[[nodiscard]] std::optional<GoalSelection> closest_reachable_goal(
  const CostGrid & grid, const Cell & start, const Point2 & requested_world);

// Returns {exact, terminal}.
[[nodiscard]] std::pair<bool, bool> classify_goal(
  const GridMap & source_grid, const CostGrid & inflated_grid,
  const Point2 & requested_world, const Cell & selected_cell);

[[nodiscard]] SearchResult astar(
  const CostGrid & grid, const Cell & start, const Cell & goal,
  double heuristic_weight = 1.0, double cost_weight = 2.0, double timeout_ms = 100.0);

[[nodiscard]] bool line_is_clear(const CostGrid & grid, const Cell & start, const Cell & end);

[[nodiscard]] int line_max_cost(const CostGrid & grid, const Cell & start, const Cell & end);

[[nodiscard]] double grid_lethal_radius(double required_clearance, double resolution);

// An empty result means even an adjacent A* edge failed the continuous
// clearance check -- the caller must drop the route, not fall back to `cells`.
[[nodiscard]] std::vector<Cell> simplify_path(
  const CostGrid & grid, const std::vector<Cell> & cells,
  bool preserve_cost = false,
  const GridMap * source_grid = nullptr,
  std::optional<double> required_clearance = std::nullopt,
  int occupied_threshold = 65);

[[nodiscard]] double path_length(const std::vector<Point2> & points);

[[nodiscard]] std::optional<PathProjection> path_projection(
  const std::vector<Point2> & points, const Point2 & point);

[[nodiscard]] std::vector<Point2> trim_path_to(
  const std::vector<Point2> & points, const Point2 & point, double margin);

[[nodiscard]] bool should_replace_path(
  const std::optional<PathProjection> & projection, double candidate_length,
  double retain_tolerance, double switch_improvement);

}  // namespace px4_vio_bridge
