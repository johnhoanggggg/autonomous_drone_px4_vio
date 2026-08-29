/// Deterministic replay harness for the C++ command limiters.
///
/// Reads one scenario as JSON on stdin, applies each step to
/// PathCommandLimiter/HorizontalCommandLimiter, and writes the resulting state
/// per step as JSON on stdout. test_planner_flight_parity.py drives the Python
/// implementation over the identical step list and diffs the two, which is what
/// makes the C++ adapter eligible to replace the Python one in flight.

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "px4_vio_bridge/command_limiter.hpp"

using nlohmann::json;
using px4_vio_bridge::HorizontalCommandLimiter;
using px4_vio_bridge::PathCommandLimiter;
using px4_vio_bridge::Point2;

namespace
{

Point2 point_of(const json & value)
{
  return {value.at(0).get<double>(), value.at(1).get<double>()};
}

std::vector<Point2> points_of(const json & value)
{
  std::vector<Point2> points;
  points.reserve(value.size());
  for (const auto & entry : value) {
    points.push_back(point_of(entry));
  }
  return points;
}

json dump_path(const PathCommandLimiter & limiter)
{
  json state;
  state["position"] = limiter.position()
    ? json::array({limiter.position()->first, limiter.position()->second})
    : json(nullptr);
  state["velocity"] = json::array({limiter.velocity().first, limiter.velocity().second});
  state["waiting_vertex"] = limiter.waiting_vertex()
    ? json(*limiter.waiting_vertex()) : json(nullptr);
  state["has_path"] = limiter.path() != nullptr;
  return state;
}

json dump_horizontal(const HorizontalCommandLimiter & limiter)
{
  json state;
  state["position"] = limiter.position()
    ? json::array({limiter.position()->first, limiter.position()->second})
    : json(nullptr);
  state["velocity"] = json::array({limiter.velocity().first, limiter.velocity().second});
  return state;
}

}  // namespace

int main()
{
  json scenario;
  std::cin >> scenario;

  const auto & config = scenario.at("limiter");
  PathCommandLimiter path_limiter(
    config.at("max_speed").get<double>(),
    config.at("max_acceleration").get<double>(),
    config.at("max_projection_error").get<double>(),
    config.at("corner_tolerance").get<double>(),
    config.at("max_entry_error").get<double>(),
    config.at("max_connector_error").get<double>(),
    config.at("suffix_tolerance").get<double>(),
    config.at("corner_blending").get<bool>(),
    config.at("junction_deviation").get<double>());
  HorizontalCommandLimiter horizontal_limiter(
    config.at("max_speed").get<double>(),
    config.at("max_acceleration").get<double>());

  // A constant clearance verdict keeps the harness ROS-free while still
  // exercising both the accept and reject branches of the connector rejoin.
  const bool clearance = scenario.value("clearance", true);
  const PathCommandLimiter::ClearanceCheck clearance_check =
    [clearance](const Point2 &, const Point2 &) {return clearance;};

  json results = json::array();
  for (const auto & step : scenario.at("steps")) {
    const auto op = step.at("op").get<std::string>();
    json result;
    result["op"] = op;
    try {
      if (op == "set_path") {
        result["changed"] = path_limiter.set_path(
          points_of(step.at("points")), point_of(step.at("reference")), clearance_check);
      } else if (op == "update") {
        path_limiter.update(
          point_of(step.at("desired")),
          step.at("dt").get<double>(),
          step.value("advance", true),
          step.contains("reference") && !step.at("reference").is_null()
          ? std::optional<Point2>{point_of(step.at("reference"))}
          : std::nullopt);
      } else if (op == "clear") {
        path_limiter.clear();
      } else if (op == "horizontal_reset") {
        horizontal_limiter.reset(point_of(step.at("position")));
      } else if (op == "horizontal_update") {
        horizontal_limiter.update(
          point_of(step.at("target")), step.at("dt").get<double>());
      } else if (op == "horizontal_adopt") {
        horizontal_limiter.adopt(
          point_of(step.at("position")), point_of(step.at("velocity")));
      } else {
        throw std::invalid_argument("unknown op " + op);
      }
      result["ok"] = true;
    } catch (const std::exception & error) {
      result["ok"] = false;
      result["error"] = error.what();
    }
    result["path"] = dump_path(path_limiter);
    result["horizontal"] = dump_horizontal(horizontal_limiter);
    results.push_back(result);
  }
  std::cout << results.dump() << std::endl;
  return 0;
}
