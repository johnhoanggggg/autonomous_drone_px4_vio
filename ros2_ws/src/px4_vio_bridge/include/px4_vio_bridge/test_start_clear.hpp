#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <octomap/OcTreeKey.h>
#include "px4_vio_bridge/voxel_grid.hpp"

namespace px4_vio_bridge
{
// Explicit observation-only test override. Clear only native voxel boxes wholly
// contained in the sphere; never extend the requested radius by half a voxel.
// Call on a publication copy, not the sensor evidence held by the assembler.
template<class Tree>
std::size_t clear_test_start_sphere(Tree & tree, const Point3 & center, double radius)
{
  if (!std::isfinite(radius) || radius <= 0.0 || radius > 0.5 ||
    !std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z))
  {
    throw std::invalid_argument("test start clearing requires a finite radius in (0, 0.5]");
  }
  const auto resolution = tree.getResolution();
  octomap::OcTreeKey key;
  if (!tree.coordToKeyChecked(center.x - radius, center.y - radius, center.z - radius, key) ||
    !tree.coordToKeyChecked(center.x + radius, center.y + radius, center.z + radius, key))
  {throw std::invalid_argument("test sphere outside OctoMap key range");}
  const auto half = resolution * 0.5;
  std::size_t count = 0;
  for (int z = static_cast<int>(std::floor((center.z - radius) / resolution));
    z <= static_cast<int>(std::floor((center.z + radius) / resolution)); ++z)
  {
    for (int y = static_cast<int>(std::floor((center.y - radius) / resolution));
      y <= static_cast<int>(std::floor((center.y + radius) / resolution)); ++y)
    {
      for (int x = static_cast<int>(std::floor((center.x - radius) / resolution));
        x <= static_cast<int>(std::floor((center.x + radius) / resolution)); ++x)
      {
        const Point3 p{(x + 0.5) * resolution, (y + 0.5) * resolution, (z + 0.5) * resolution};
        const auto dx = std::abs(p.x - center.x) + half;
        const auto dy = std::abs(p.y - center.y) + half;
        const auto dz = std::abs(p.z - center.z) + half;
        if (dx * dx + dy * dy + dz * dz > radius * radius) {continue;}
        auto * node = tree.setNodeValue(p.x, p.y, p.z, tree.getClampingThresMinLog(), true);
        if (node == nullptr) {throw std::runtime_error("test sphere outside OctoMap key range");}
        node->setOccupancyType(0);
        ++count;
      }
    }
  }
  tree.updateInnerOccupancy();
  return count;
}
}  // namespace px4_vio_bridge
