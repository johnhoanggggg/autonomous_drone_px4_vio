#include "px4_vio_bridge/voxel_grid.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace px4_vio_bridge
{

VoxelGrid::VoxelGrid(
  std::size_t width, std::size_t height, std::size_t depth, double resolution,
  Point3 origin, VoxelState initial)
: width_(width), height_(height), depth_(depth), resolution_(resolution), origin_(origin)
{
  if (width == 0 || height == 0 || depth == 0 || !std::isfinite(resolution) ||
    resolution <= 0.0 || !std::isfinite(origin.x) || !std::isfinite(origin.y) ||
    !std::isfinite(origin.z))
  {
    throw std::invalid_argument("invalid voxel grid geometry");
  }
  if (width > std::numeric_limits<std::size_t>::max() / height ||
    width * height > std::numeric_limits<std::size_t>::max() / depth)
  {
    throw std::overflow_error("voxel grid is too large");
  }
  data_.assign(width * height * depth, initial);
}

bool VoxelGrid::in_bounds(const Voxel & voxel) const
{
  return voxel.x >= 0 && voxel.y >= 0 && voxel.z >= 0 &&
         static_cast<std::size_t>(voxel.x) < width_ &&
         static_cast<std::size_t>(voxel.y) < height_ &&
         static_cast<std::size_t>(voxel.z) < depth_;
}

std::size_t VoxelGrid::index(const Voxel & voxel) const
{
  if (!in_bounds(voxel)) {
    throw std::out_of_range("voxel outside grid");
  }
  return (static_cast<std::size_t>(voxel.z) * height_ +
         static_cast<std::size_t>(voxel.y)) * width_ + static_cast<std::size_t>(voxel.x);
}

VoxelState VoxelGrid::at(const Voxel & voxel) const {return data_.at(index(voxel));}

void VoxelGrid::set(const Voxel & voxel, VoxelState state) {data_.at(index(voxel)) = state;}

std::optional<Voxel> VoxelGrid::world_to_voxel(const Point3 & point) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return std::nullopt;
  }
  Voxel voxel{
    static_cast<int>(std::floor((point.x - origin_.x) / resolution_)),
    static_cast<int>(std::floor((point.y - origin_.y) / resolution_)),
    static_cast<int>(std::floor((point.z - origin_.z) / resolution_))};
  return in_bounds(voxel) ? std::optional<Voxel>(voxel) : std::nullopt;
}

Point3 VoxelGrid::voxel_center(const Voxel & voxel) const
{
  if (!in_bounds(voxel)) {
    throw std::out_of_range("voxel outside grid");
  }
  return {
    origin_.x + (static_cast<double>(voxel.x) + 0.5) * resolution_,
    origin_.y + (static_cast<double>(voxel.y) + 0.5) * resolution_,
    origin_.z + (static_cast<double>(voxel.z) + 0.5) * resolution_};
}

Point3 VoxelGrid::upper_bound() const
{
  return {
    origin_.x + static_cast<double>(width_) * resolution_,
    origin_.y + static_cast<double>(height_) * resolution_,
    origin_.z + static_cast<double>(depth_) * resolution_};
}

}  // namespace px4_vio_bridge
