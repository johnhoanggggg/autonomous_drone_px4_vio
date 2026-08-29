/// Deterministic JSON replay/audit harness for the observation-only 3D mode.
///
/// A record contains one raw voxel generation, the start/goal/config used by
/// the planner, and optionally the accepted path and follower chords recorded
/// for that generation. The output both reruns planning and proves every
/// recorded segment with the production swept-sphere predicate.

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "px4_vio_bridge/grid_planner_3d.hpp"

using nlohmann::json;
using namespace px4_vio_bridge;  // NOLINT(build/namespaces)

namespace
{

Point3 point_from_json(const json & value)
{
  if (!value.is_array() || value.size() != 3) {
    throw std::invalid_argument("3D point must contain exactly three values");
  }
  return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}

json point_to_json(const Point3 & point)
{
  return json::array({point.x, point.y, point.z});
}

json voxel_to_json(const Voxel & voxel)
{
  return json::array({voxel.x, voxel.y, voxel.z});
}

std::vector<Point3> path_from_json(const json & value)
{
  std::vector<Point3> path;
  path.reserve(value.size());
  for (const auto & point : value) {
    path.push_back(point_from_json(point));
  }
  return path;
}

void audit_segments(
  const VoxelGrid & grid, const std::vector<Point3> & path, double radius,
  const std::string & kind, json & violations, std::size_t & checked)
{
  if (path.size() < 2) {
    violations.push_back({{"kind", kind + "_TOO_SHORT"}});
    return;
  }
  for (std::size_t index = 1; index < path.size(); ++index) {
    ++checked;
    if (!swept_sphere_clear(grid, path[index - 1], path[index], radius)) {
      violations.push_back({{"kind", kind + "_CLEARANCE"}, {"segment", index - 1}});
    }
  }
}

}  // namespace

int main()
{
  try {
    json input;
    std::cin >> input;
    const auto width = input.at("width").get<std::size_t>();
    const auto height = input.at("height").get<std::size_t>();
    const auto depth = input.at("depth").get<std::size_t>();
    VoxelGrid grid(
      width, height, depth, input.at("resolution").get<double>(),
      point_from_json(input.at("origin")));
    const auto & data = input.at("data");
    if (data.size() != grid.size()) {
      throw std::invalid_argument("voxel data length does not match geometry");
    }
    for (int z = 0; z < static_cast<int>(depth); ++z) {
      for (int y = 0; y < static_cast<int>(height); ++y) {
        for (int x = 0; x < static_cast<int>(width); ++x) {
          const Voxel voxel{x, y, z};
          const auto state = data.at(grid.index(voxel)).get<int>();
          if (state != -1 && state != 0 && state != 100) {
            throw std::invalid_argument("voxel state must be -1, 0 or 100");
          }
          grid.set(voxel, static_cast<VoxelState>(state));
        }
      }
    }

    const auto generation = input.at("generation").get<std::int64_t>();
    const auto & config_json = input.at("config");
    Planner3DConfig config;
    config.lethal_radius = config_json.at("lethal_radius").get<double>();
    config.inflation_radius = config_json.at("inflation_radius").get<double>();
    config.cost_scaling = config_json.value("cost_scaling", 3.0);
    config.heuristic_weight = config_json.value("heuristic_weight", 1.0);
    config.cost_weight = config_json.value("cost_weight", 2.0);
    config.timeout_ms = config_json.value("timeout_ms", 1000.0);
    config.start_recovery_radius = config_json.value("start_recovery_radius", 0.20);
    const auto result = plan_path_3d(
      grid, point_from_json(input.at("start")), point_from_json(input.at("goal")), config);

    json output;
    output["generation"] = generation;
    output["found"] = result.found();
    output["reason"] = result.reason;
    output["search_reason"] = result.search.reason;
    output["expanded"] = result.search.expanded;
    output["cost"] = result.search.found() ? json(result.search.cost) : json(nullptr);
    output["recovered_start"] = result.recovered_start ?
      voxel_to_json(*result.recovered_start) : json(nullptr);
    output["path"] = json::array();
    for (const auto & point : result.path) {
      output["path"].push_back(point_to_json(point));
    }
    if (result.goal) {
      output["goal"] = {
        {"voxel", voxel_to_json(result.goal->voxel)},
        {"exact", result.goal->exact},
        {"terminal", result.goal->terminal},
        {"requested_distance", result.goal->requested_distance},
        {"reachable_voxels", result.goal->reachable_voxels},
      };
    } else {
      output["goal"] = nullptr;
    }

    json violations = json::array();
    std::size_t checked_path_segments = 0;
    std::size_t checked_follower_chords = 0;
    if (result.path.size() >= 2) {
      audit_segments(
        grid, result.path, config.lethal_radius, "PLANNED_PATH", violations,
        checked_path_segments);
    }
    if (input.contains("accepted_path")) {
      if (input.at("path_map_generation").get<std::int64_t>() != generation) {
        violations.push_back({{"kind", "PATH_GENERATION_MISMATCH"}});
      } else {
        audit_segments(
          grid, path_from_json(input.at("accepted_path")), config.lethal_radius,
          "ACCEPTED_PATH", violations, checked_path_segments);
      }
    }
    for (const auto & chord : input.value("follower_chords", json::array())) {
      if (chord.at("map_generation").get<std::int64_t>() != generation) {
        violations.push_back({{"kind", "CHORD_GENERATION_MISMATCH"},
          {"chord", checked_follower_chords}});
      } else {
        const std::vector<Point3> segment{
          point_from_json(chord.at("start")), point_from_json(chord.at("end"))};
        audit_segments(
          grid, segment, config.lethal_radius, "FOLLOWER_CHORD", violations,
          checked_follower_chords);
      }
    }
    output["checked_path_segments"] = checked_path_segments;
    output["checked_follower_chords"] = checked_follower_chords;
    output["violations"] = violations;
    output["valid"] = violations.empty();
    std::cout << output.dump() << '\n';
    return violations.empty() ? 0 : 1;
  } catch (const std::exception & error) {
    std::cout << json({{"valid", false}, {"error", error.what()}}).dump() << '\n';
    return 2;
  }
}
