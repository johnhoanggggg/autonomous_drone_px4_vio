#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace px4_vio_bridge
{

struct Point3
{
  double x{};
  double y{};
  double z{};
};

struct Voxel
{
  int x{};
  int y{};
  int z{};

  friend bool operator==(const Voxel & a, const Voxel & b)
  {
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }
  friend bool operator!=(const Voxel & a, const Voxel & b) {return !(a == b);}
};

enum class VoxelState : std::int8_t
{
  Unknown = -1,
  Free = 0,
  Occupied = 100,
};

// A bounded dense planning view. It is intentionally independent of ROS and
// OctoMap so all safety geometry can be unit-tested without middleware.
class VoxelGrid
{
public:
  VoxelGrid() = default;
  VoxelGrid(
    std::size_t width, std::size_t height, std::size_t depth, double resolution,
    Point3 origin, VoxelState initial = VoxelState::Unknown);

  [[nodiscard]] std::size_t width() const {return width_;}
  [[nodiscard]] std::size_t height() const {return height_;}
  [[nodiscard]] std::size_t depth() const {return depth_;}
  [[nodiscard]] double resolution() const {return resolution_;}
  [[nodiscard]] const Point3 & origin() const {return origin_;}
  [[nodiscard]] std::size_t size() const {return data_.size();}

  [[nodiscard]] bool in_bounds(const Voxel & voxel) const;
  [[nodiscard]] std::size_t index(const Voxel & voxel) const;
  [[nodiscard]] VoxelState at(const Voxel & voxel) const;
  void set(const Voxel & voxel, VoxelState state);
  [[nodiscard]] std::optional<Voxel> world_to_voxel(const Point3 & point) const;
  [[nodiscard]] Point3 voxel_center(const Voxel & voxel) const;
  [[nodiscard]] Point3 upper_bound() const;
  [[nodiscard]] const std::vector<VoxelState> & data() const {return data_;}

private:
  std::size_t width_{};
  std::size_t height_{};
  std::size_t depth_{};
  double resolution_{};
  Point3 origin_{};
  std::vector<VoxelState> data_;
};

}  // namespace px4_vio_bridge
