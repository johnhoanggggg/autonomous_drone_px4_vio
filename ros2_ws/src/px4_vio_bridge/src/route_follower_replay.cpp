/// Deterministic JSON replay harness for PositionRouteFollower.
/// test_route_follower_parity.py drives the Python implementation through the
/// same operations and compares every published/result field.

#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "px4_vio_bridge/route_follower.hpp"

using nlohmann::json;
using px4_vio_bridge::FollowResult;
using px4_vio_bridge::Point2;
using px4_vio_bridge::PositionRouteFollower;

namespace
{

Point2 point_of(const json & value)
{
  return {value.at(0).get<double>(), value.at(1).get<double>()};
}

std::vector<Point2> points_of(const json & value)
{
  std::vector<Point2> points;
  for (const auto & entry : value) {
    points.push_back(point_of(entry));
  }
  return points;
}

json point_json(const Point2 & point)
{
  return json::array({point.first, point.second});
}

json result_json(const FollowResult & value)
{
  return {
    {"status", value.status}, {"valid", value.valid},
    {"desired_carrot", point_json(value.desired_carrot)},
    {"commanded_carrot", point_json(value.commanded_carrot)},
    {"commanded_displacement", point_json(value.commanded_displacement)},
    {"path_progress", value.path_progress}, {"progress", value.progress},
    {"remaining", value.remaining}, {"cross_track", value.cross_track},
    {"generation", value.generation}};
}

json state_json(const PositionRouteFollower & follower)
{
  return {
    {"has_path", follower.path() != nullptr}, {"generation", follower.generation()},
    {"progress", follower.progress()}, {"path_progress", follower.path_progress()},
    {"commanded_displacement", point_json(follower.commanded_displacement())},
    {"command_velocity", point_json(follower.command_velocity())},
    {"cross_track_latched", follower.cross_track_latched()},
    {"at_goal", follower.at_goal()}};
}

}  // namespace

int main()
{
  json scenario;
  std::cin >> scenario;
  const auto & config = scenario.at("follower");
  PositionRouteFollower follower(
    config.at("lookahead").get<double>(),
    config.at("max_carrot_speed").get<double>(),
    config.at("max_carrot_acceleration").get<double>(),
    config.at("max_cross_track").get<double>(),
    config.at("cross_track_resume").get<double>(),
    config.at("cross_track_recovery_time").get<double>(),
    config.at("arrival_tolerance").get<double>(),
    config.at("arrival_release_tolerance").get<double>());
  json output = json::array();
  for (const auto & step : scenario.at("steps")) {
    const auto op = step.at("op").get<std::string>();
    json item{{"op", op}};
    try {
      if (op == "set_path") {
        item["changed"] = follower.set_path(
          points_of(step.at("points")), point_of(step.at("pose")));
      } else if (op == "update") {
        const bool validator = step.value("validator", true);
        const auto lookahead = step.contains("lookahead") && !step.at("lookahead").is_null()
          ? std::optional<double>{step.at("lookahead").get<double>()} : std::nullopt;
        item["result"] = result_json(follower.update(
            point_of(step.at("pose")), step.at("dt").get<double>(), lookahead,
            [validator](const Point2 &) {return validator;}));
      } else if (op == "clear_path") {
        follower.clear_path();
      } else if (op == "reset_route_progress") {
        follower.reset_route_progress();
      } else if (op == "interrupt_cross_track_recovery") {
        follower.interrupt_cross_track_recovery();
      } else if (op == "hold_command") {
        follower.hold_command();
      } else {
        throw std::invalid_argument("unknown op " + op);
      }
      item["ok"] = true;
    } catch (const std::exception & error) {
      item["ok"] = false;
      item["error"] = error.what();
    }
    item["state"] = state_json(follower);
    output.push_back(item);
  }
  std::cout << output.dump() << std::endl;
  return 0;
}
