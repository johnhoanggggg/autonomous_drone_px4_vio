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
  // Not compared by the Python parity test, which reads named fields only.
  // replay_command_heading.py needs it to know when a lane change is active.
  state["joining"] = limiter.joining();
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
  px4_vio_bridge::HeadingReference heading_reference;
  if (scenario.contains("initial_yaw")) {
    heading_reference.reset(scenario.at("initial_yaw").get<double>());
  }
  bool yaw_holding = false;

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
      } else if (op == "preview_direction") {
        const auto direction = path_limiter.preview_direction(
          point_of(step.at("desired")), step.at("dt").get<double>(),
          point_of(step.at("reference")), step.value("min_position_distance", 0.15));
        result["direction"] = direction
          ? json::array({direction->first, direction->second}) : json(nullptr);
      } else if (op == "heading_sample") {
        // Recorded-state comparison, not a vehicle/adapter simulation: the
        // caller supplies the accepted path and reconstructed command point.
        const px4_vio_bridge::Polyline path(points_of(step.at("points")));
        const auto projection = path.project(point_of(step.at("command")));
        const auto direction = px4_vio_bridge::route_heading_direction(
          path, projection.along, scenario.value("heading_lookahead", 0.35));
        std::optional<double> preview;
        if (direction) {
          preview = px4_vio_bridge::ned_track_heading(
            px4_vio_bridge::vio_enu_displacement_to_ned(
              px4_vio_bridge::map_displacement_to_vio(
                *direction, step.at("correction_yaw").get<double>())), 1e-6);
        }
        const auto target = heading_reference.update(
          preview, scenario.value("heading_deadband", 15.0 * M_PI / 180.0),
          scenario.value("heading_rate", 20.0 * M_PI / 180.0), step.value("dt", 0.05));
        const auto error = heading_reference.alignment_error(step.at("measured_heading"));
        if (error && std::abs(*error) > 40.0 * M_PI / 180.0) {
          yaw_holding = true;
        } else if (!error || std::abs(*error) <= 15.0 * M_PI / 180.0) {
          yaw_holding = false;
        }
        const auto cap = px4_vio_bridge::yaw_translation_speed_cap(
          error, 15.0 * M_PI / 180.0, 40.0 * M_PI / 180.0,
          config.at("max_speed"), {step.value("command_speed", 0.0), 0.0},
          config.at("max_acceleration"), step.value("dt", 0.05));
        result["preview"] = preview ? json(*preview) : json(nullptr);
        result["target"] = target ? json(*target) : json(nullptr);
        result["error"] = error ? json(*error) : json(nullptr);
        result["yaw_holding"] = yaw_holding;
        result["yaw_speed_cap"] = cap ? json(*cap) : json(nullptr);
        result["progress"] = projection.along;
        result["projection_error"] = projection.cross_track;
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
