#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <octomap/OcTreeKey.h>
#include "px4_vio_bridge/voxel_grid.hpp"

namespace px4_vio_bridge
{
// Expand known leaves directly into a native, key-aligned crop. Coarse leaves
// represent all their native children; unknown space is never filled.
template<class Tree>
void fill_native_octomap_grid(const Tree & tree, VoxelGrid & grid)
{
  const auto r = tree.getResolution();
  const auto aligned = [r](double value) {
      return std::abs(value / r - std::round(value / r)) < 1e-6;
    };
  if (std::abs(grid.resolution() - r) > 1e-12 ||
    !aligned(grid.origin().x) || !aligned(grid.origin().y) || !aligned(grid.origin().z))
  {throw std::invalid_argument("OctoMap crop must use native key alignment");}
  if (grid.size() == 0) {return;}
  const auto first = grid.voxel_center({0, 0, 0});
  const auto last = grid.voxel_center({static_cast<int>(grid.width()) - 1,
    static_cast<int>(grid.height()) - 1, static_cast<int>(grid.depth()) - 1});
  octomap::OcTreeKey first_key, last_key;
  if (!tree.coordToKeyChecked(first.x, first.y, first.z, first_key) ||
    !tree.coordToKeyChecked(last.x, last.y, last.z, last_key))
  {throw std::invalid_argument("OctoMap crop outside key range");}
  const auto index = [r](double value, double origin, std::size_t size) {
      return static_cast<int>(std::clamp(std::round((value - origin) / r),
        0.0, static_cast<double>(size)));
    };
  for (auto it = tree.begin_leafs_bbx(first_key, last_key), end = tree.end_leafs_bbx(); it != end; ++it) {
    const auto half = it.getSize() * 0.5;
    const int lx = index(it.getX() - half, grid.origin().x, grid.width());
    const int hx = index(it.getX() + half, grid.origin().x, grid.width());
    const int ly = index(it.getY() - half, grid.origin().y, grid.height());
    const int hy = index(it.getY() + half, grid.origin().y, grid.height());
    const int lz = index(it.getZ() - half, grid.origin().z, grid.depth());
    const int hz = index(it.getZ() + half, grid.origin().z, grid.depth());
    const auto state = tree.isNodeOccupied(*it) ? VoxelState::Occupied : VoxelState::Free;
    for (int z = lz; z < hz; ++z) {
      for (int y = ly; y < hy; ++y) {
        for (int x = lx; x < hx; ++x) {grid.set({x, y, z}, state);}
      }
    }
  }
}
}  // namespace px4_vio_bridge
