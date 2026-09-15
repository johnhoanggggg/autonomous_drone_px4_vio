// Streaming offline audit of recorded paths/chords against their exact OctoMap.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <iostream>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include "px4_vio_bridge/grid_planner_3d.hpp"

using namespace px4_vio_bridge;
using nlohmann::json;

int main()
{
  using Key = std::pair<std::int64_t, std::int64_t>;
  std::map<Key, std::unique_ptr<octomap::AbstractOcTree>> maps;
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      const auto value = json::parse(line);
      const Key key{value.at("generation"), value.at("stamp_ns")};
      if (value.at("kind") == "map") {
        octomap_msgs::msg::Octomap message;
        message.id = value.at("id"); message.binary = value.at("binary");
        message.resolution = value.at("resolution");
        message.data = value.at("data").get<std::vector<std::int8_t>>();
        auto tree = std::unique_ptr<octomap::AbstractOcTree>(octomap_msgs::msgToMap(message));
        if (!dynamic_cast<octomap::OcTree *>(tree.get()) &&
          !dynamic_cast<octomap::ColorOcTree *>(tree.get())) {throw std::runtime_error("map type");}
        maps[key] = std::move(tree);
        while (maps.size() > 16) {maps.erase(maps.begin());}
        std::cout << json{{"valid", true}, {"segments", 0}}.dump() << std::endl;
        continue;
      }
      if (maps.find(key) == maps.end()) {throw std::runtime_error("missing exact map generation");}
      auto & tree = *maps.at(key);
      const auto radius = value.at("radius").get<double>();
      if (!std::isfinite(radius) || radius <= 0.0) {throw std::runtime_error("radius");}
      std::vector<Point3> points;
      for (const auto & point : value.at("points")) {
        if (!point.is_array() || point.size() != 3) {throw std::runtime_error("point");}
        Point3 p{point[0], point[1], point[2]};
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
          throw std::runtime_error("nonfinite point");
        }
        points.push_back(p);
      }
      if (points.size() < 2) {throw std::runtime_error("short path");}
      const auto start_state = [&](const auto & typed_tree) {
          const auto & p = points.front();
          const auto * node = typed_tree.search(p.x, p.y, p.z);
          return node == nullptr ? "unknown" : (typed_tree.isNodeOccupied(node) ? "occupied" : "free");
        };
      const std::string first_state = dynamic_cast<const octomap::OcTree *>(&tree) ?
        start_state(*dynamic_cast<const octomap::OcTree *>(&tree)) :
        start_state(*dynamic_cast<const octomap::ColorOcTree *>(&tree));
      bool valid = true;
      for (std::size_t i = 1; i < points.size(); ++i) {
        const auto & a = points[i - 1]; const auto & b = points[i];
        const auto r = tree.getResolution(), pad = radius + 2 * r;
        const Point3 origin{std::floor((std::min(a.x, b.x) - pad) / r) * r,
          std::floor((std::min(a.y, b.y) - pad) / r) * r,
          std::floor((std::min(a.z, b.z) - pad) / r) * r};
        const auto width = static_cast<std::size_t>(std::ceil((std::max(a.x, b.x) + pad - origin.x) / r));
        const auto height = static_cast<std::size_t>(std::ceil((std::max(a.y, b.y) + pad - origin.y) / r));
        const auto depth = static_cast<std::size_t>(std::ceil((std::max(a.z, b.z) + pad - origin.z) / r));
        if (width > 512 || height > 512 || depth > 512 || width * height * depth > 2000000) {
          throw std::runtime_error("audit crop too large");
        }
        VoxelGrid grid(width, height, depth, r, origin);
        const auto fill = [&](const auto & typed_tree) {
            for (int z = 0; z < static_cast<int>(depth); ++z) {
              for (int y = 0; y < static_cast<int>(height); ++y) {
                for (int x = 0; x < static_cast<int>(width); ++x) {
                  const Voxel voxel{x, y, z}; const auto p = grid.voxel_center(voxel);
                  if (const auto * node = typed_tree.search(p.x, p.y, p.z)) {
                    grid.set(voxel, typed_tree.isNodeOccupied(node) ? VoxelState::Occupied : VoxelState::Free);
                  }
                }
              }
            }
          };
        if (const auto * typed = dynamic_cast<const octomap::OcTree *>(&tree)) {fill(*typed);}
        else {fill(*dynamic_cast<const octomap::ColorOcTree *>(&tree));}
        valid = valid && swept_sphere_clear(grid, a, b, radius);
      }
      std::cout << json{{"valid", valid}, {"segments", points.size() - 1}, {"start_state", first_state}}.dump() << std::endl;
    } catch (const std::exception & error) {
      std::cout << json{{"valid", false}, {"error", error.what()}}.dump() << std::endl;
    }
  }
}
