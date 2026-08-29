#pragma once

// The A* planner core. Everything down to should_replace_path() is a
// line-for-line port of the legacy px4_vio_bridge/grid_planner.py and is held
// to it by test/test_grid_planner_parity.py, which drives both through
// randomized maps and compares costs, expansions and cells. GoalModeHysteresis
// below has no Python counterpart: it was added on 2026-08-29, after the C++
// nodes became the flown implementation. Where Python's
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

// ---------------------------------------------------------------------------
// Goal-mode hysteresis
//
// One occupancy grid deciding the semantic mode outright let a single frame of
// connectivity change replace the endpoint, the path, the follower's heading
// and the goal-completion semantics. A mode change is therefore confirmed over
// `confirmation_maps` *distinct occupancy-grid generations* -- planner ticks
// are not independent evidence, and at ~1 Hz maps the default of 2 still
// commits well inside the flight adapter's 3 s planner-fault land timer.
//
// This debounces the *meaning* of a route, never its safety: the caller must
// still revalidate the retained route against every new map and drop it
// immediately when it is unsafe.
enum class GoalMode
{
  PathValid,
  SafeApproach,
  Exploring,
};

[[nodiscard]] const char * to_string(GoalMode mode);

// classify_goal()'s (exact, terminal) pair as a mode.
[[nodiscard]] GoalMode goal_mode_from(bool exact, bool terminal);

// Only PathValid and SafeApproach end at the requested goal; an exploration
// frontier is a temporary endpoint and must never report completion.
[[nodiscard]] bool goal_mode_terminal(GoalMode mode);

struct ModeDecision
{
  GoalMode stable{GoalMode::Exploring};
  bool has_pending{false};
  GoalMode pending{GoalMode::Exploring};
  int pending_count{};
  int confirmation_maps{};
  // This observation moved the stable mode.
  bool committed{false};
};

class GoalModeHysteresis
{
public:
  explicit GoalModeHysteresis(int confirmation_maps = 2);

  // A new requested goal carries no old semantic commitment.
  void reset();
  ModeDecision observe(GoalMode raw, std::int64_t map_generation);
  // Adopt `mode` at once. The caller uses this when it has just replaced the
  // accepted path, because no older commitment survives that.
  void commit(GoalMode mode);

  [[nodiscard]] bool initialized() const {return initialized_;}
  [[nodiscard]] GoalMode stable() const {return stable_;}
  [[nodiscard]] ModeDecision decision() const;
  // "" when settled, else " MODE_PENDING PATH_VALID->EXPLORING 1/2".
  [[nodiscard]] std::string pending_suffix() const;

private:
  int confirmation_maps_;
  bool initialized_{false};
  GoalMode stable_{GoalMode::Exploring};
  std::optional<GoalMode> pending_;
  int pending_count_{};
  std::optional<std::int64_t> last_counted_generation_;
};

// Inputs to the accepted-path decision, gathered in one frame: the map
// solution the current occupancy grid belongs to.
struct PathReplacementInputs
{
  // A semantic mode change is proposed but not yet confirmed.
  bool mode_transition_pending{};
  // The retained route revalidated against the newest raw grid.
  bool retained_safe{};
  bool goal_changed{};
  bool effective_goal_changed{};
  std::optional<PathProjection> projection;
  double candidate_length{};
  double retain_tolerance{};
  double switch_improvement{};
};

struct PathReplacementDecision
{
  bool replace{};
  // The retained route survived a pending transition unchanged.
  bool transition_hold{};
  // The vehicle has genuinely left the retained corridor.
  bool off_corridor{};
};

// Whether a fresh candidate should displace the accepted route.
//
// This is should_replace_path() plus the mode-transition rule. While a semantic
// transition is unconfirmed, the endpoint change and the length comparison both
// describe the *other* mode, so acting on them would defeat the debounce and
// re-create the PATH_VALID/EXPLORING oscillation. Two things still replace at
// once and are deliberately outside the hold: a route the newest grid proves
// unsafe, and a genuine physical deviation out of the corridor. Mode debounce
// is never collision debounce.
[[nodiscard]] PathReplacementDecision decide_path_replacement(
  const PathReplacementInputs & inputs);

[[nodiscard]] bool should_replace_path(
  const std::optional<PathProjection> & projection, double candidate_length,
  double retain_tolerance, double switch_improvement);

}  // namespace px4_vio_bridge
