/// Deterministic replay harness for the C++ grid planner.
///
/// Reads one scenario as JSON on stdin (grid geometry, cells, start, goal and
/// the planner's radii), runs the same inflate -> recover_start ->
/// closest_reachable_goal -> A* -> simplify pipeline the node runs, and writes
/// the result as JSON on stdout. test_grid_planner_parity.py drives
/// grid_planner.py over identical randomized maps and diffs the two, which is
/// what makes the C++ planner eligible to replace the Python one in flight.

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "px4_vio_bridge/grid_planner.hpp"

using nlohmann::json;
using namespace px4_vio_bridge;  // NOLINT(build/namespaces)

namespace
{

// Keeps the benchmark loop from being optimised away.
volatile long benchmark_sink = 0;

double now_ms()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

json cells_to_json(const std::vector<Cell> & cells)
{
  auto out = json::array();
  for (const auto & cell : cells) {
    out.push_back(json::array({cell.first, cell.second}));
  }
  return out;
}

}  // namespace

int main()
{
  json input;
  std::cin >> input;

  GridMap grid;
  grid.width = input.at("width").get<std::size_t>();
  grid.height = input.at("height").get<std::size_t>();
  grid.resolution = input.at("resolution").get<double>();
  grid.origin_x = input.at("origin_x").get<double>();
  grid.origin_y = input.at("origin_y").get<double>();
  for (const auto & value : input.at("data")) {
    grid.data.push_back(static_cast<std::int8_t>(value.get<int>()));
  }

  const auto occupied_threshold = input.at("occupied_threshold").get<int>();
  const auto lethal_radius = input.at("lethal_radius").get<double>();
  const auto inflation_radius = input.at("inflation_radius").get<double>();
  const auto cost_scaling = input.at("cost_scaling").get<double>();
  const auto heuristic_weight = input.at("heuristic_weight").get<double>();
  const auto cost_weight = input.at("cost_weight").get<double>();
  const auto start_recovery_radius = input.at("start_recovery_radius").get<double>();
  const Cell start{input.at("start").at(0).get<int>(), input.at("start").at(1).get<int>()};
  const Point2 goal_world{
    input.at("goal_world").at(0).get<double>(), input.at("goal_world").at(1).get<double>()};

  // Optional benchmark loop. The parity test never sets it; it exists so the
  // planner can be timed on a real recorded grid without the process-spawn and
  // JSON costs of this harness swamping the measurement.
  const auto repeat = input.value("repeat", 0);
  json timing = json::object();
  if (repeat > 0) {
    const auto inflate_begun = now_ms();
    for (int i = 0; i < repeat; ++i) {
      const auto scratch = inflate_occupancy(
        grid, occupied_threshold, grid_lethal_radius(lethal_radius, grid.resolution),
        grid_lethal_radius(inflation_radius, grid.resolution), cost_scaling);
      benchmark_sink += scratch.data.empty() ? 0 : scratch.data[0];
    }
    timing["inflate_ms"] = (now_ms() - inflate_begun) / repeat;
  }

  const auto inflated = inflate_occupancy(
    grid, occupied_threshold,
    grid_lethal_radius(lethal_radius, grid.resolution),
    grid_lethal_radius(inflation_radius, grid.resolution),
    cost_scaling);

  json output;
  output["inflated"] = json::array();
  for (const auto value : inflated.data) {
    output["inflated"].push_back(static_cast<int>(value));
  }
  output["display"] = json::array();
  for (const auto value : inflation_display_data(grid, inflated, occupied_threshold)) {
    output["display"].push_back(static_cast<int>(value));
  }

  auto search_start = start;
  const auto recovery = recover_start(
    grid, inflated, start, occupied_threshold, start_recovery_radius);
  if (recovery.has_value()) {
    output["recovered"] = json::array({recovery->cell.first, recovery->cell.second});
    output["recovery_distance"] = recovery->distance;
    search_start = recovery->cell;
  } else {
    output["recovered"] = nullptr;
    output["recovery_distance"] = nullptr;
  }

  if (!recovery.has_value()) {
    output["selection"] = nullptr;
    output["result"] = nullptr;
    output["simplified"] = nullptr;
    std::cout << output.dump() << std::endl;
    return 0;
  }

  if (repeat > 0) {
    const auto select_begun = now_ms();
    for (int i = 0; i < repeat; ++i) {
      const auto scratch = closest_reachable_goal(inflated, search_start, goal_world);
      benchmark_sink += scratch.has_value() ? scratch->reachable_cells : 0;
    }
    timing["select_ms"] = (now_ms() - select_begun) / repeat;
    const auto probe = closest_reachable_goal(inflated, search_start, goal_world);
    if (probe.has_value()) {
      const auto astar_begun = now_ms();
      for (int i = 0; i < repeat; ++i) {
        const auto scratch = astar(
          inflated, search_start, probe->cell, heuristic_weight, cost_weight, 0.0);
        benchmark_sink += scratch.expanded;
      }
      timing["astar_ms"] = (now_ms() - astar_begun) / repeat;
    }
    output["timing"] = timing;
  }
  const auto selection = closest_reachable_goal(inflated, search_start, goal_world);
  if (!selection.has_value()) {
    output["selection"] = nullptr;
    output["result"] = nullptr;
    output["simplified"] = nullptr;
    std::cout << output.dump() << std::endl;
    return 0;
  }
  output["selection"] = {
    {"cell", json::array({selection->cell.first, selection->cell.second})},
    {"distance", selection->distance},
    {"reachable_cells", selection->reachable_cells},
  };
  const auto [exact, terminal] = classify_goal(grid, inflated, goal_world, selection->cell);
  output["goal_exact"] = exact;
  output["goal_terminal"] = terminal;

  // Timeout disabled: a wall-clock cutoff would make the comparison
  // machine-dependent, and the parity test is about the search, not its clock.
  const auto result = astar(
    inflated, search_start, selection->cell, heuristic_weight, cost_weight, 0.0);
  output["result"] = {
    {"cells", cells_to_json(result.cells)},
    {"cost", result.found() ? json(result.cost) : json(nullptr)},
    {"expanded", result.expanded},
    {"reason", result.reason},
  };
  if (!result.found()) {
    output["simplified"] = nullptr;
    std::cout << output.dump() << std::endl;
    return 0;
  }
  output["simplified"] = cells_to_json(
    simplify_path(inflated, result.cells, true, &grid, lethal_radius, occupied_threshold));
  std::cout << output.dump() << std::endl;
  return 0;
}
