#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "px4_vio_bridge/voxel_grid.hpp"

namespace px4_vio_bridge
{

inline constexpr std::int16_t VOXEL_UNKNOWN_COST = -1;
inline constexpr std::int16_t VOXEL_LETHAL_COST = 255;

class CostVoxelGrid
{
public:
  explicit CostVoxelGrid(const VoxelGrid & geometry);

  [[nodiscard]] bool in_bounds(const Voxel & voxel) const;
  [[nodiscard]] std::size_t index(const Voxel & voxel) const;
  [[nodiscard]] std::int16_t at(const Voxel & voxel) const;
  void set(const Voxel & voxel, std::int16_t cost);
  [[nodiscard]] bool traversable(const Voxel & voxel) const;
  [[nodiscard]] std::size_t width() const {return width_;}
  [[nodiscard]] std::size_t height() const {return height_;}
  [[nodiscard]] std::size_t depth() const {return depth_;}
  [[nodiscard]] double resolution() const {return resolution_;}
  [[nodiscard]] Point3 voxel_center(const Voxel & voxel) const;
  [[nodiscard]] std::optional<Voxel> world_to_voxel(const Point3 & point) const;
  [[nodiscard]] const std::vector<std::int16_t> & data() const {return data_;}

private:
  std::size_t width_{};
  std::size_t height_{};
  std::size_t depth_{};
  double resolution_{};
  Point3 origin_{};
  std::vector<std::int16_t> data_;
};

struct Planner3DConfig
{
  double lethal_radius{0.35};
  double inflation_radius{0.55};
  double cost_scaling{3.0};
  double heuristic_weight{1.0};
  double cost_weight{2.0};
  double timeout_ms{150.0};
  double start_recovery_radius{0.20};
};

struct SearchResult3D
{
  std::vector<Voxel> voxels;
  double cost{};
  std::size_t expanded{};
  double elapsed_ms{};
  std::string reason;

  [[nodiscard]] bool found() const {return !voxels.empty();}
};

struct GoalSelection3D
{
  Voxel voxel{};
  bool exact{};
  // A blocked known goal may end at a safe approach. An unknown goal is an
  // exploration endpoint and must never be reported reached.
  bool terminal{};
  double requested_distance{};
  std::size_t reachable_voxels{};
};

struct PlanResult3D
{
  SearchResult3D search;
  std::vector<Point3> path;
  std::optional<GoalSelection3D> goal;
  std::optional<Voxel> recovered_start;
  std::string reason;

  [[nodiscard]] bool found() const {return !path.empty();}
};

[[nodiscard]] CostVoxelGrid inflate_voxels(
  const VoxelGrid & grid, double lethal_radius, double inflation_radius,
  double cost_scaling = 3.0);

// 26-connected neighbours. Every face/edge cell implicated in a diagonal is
// required to be free, preventing 2-axis and 3-axis corner cutting.
[[nodiscard]] std::vector<std::pair<Voxel, double>> traversable_neighbors_3d(
  const CostVoxelGrid & grid, const Voxel & voxel);

[[nodiscard]] std::optional<Voxel> recover_start_3d(
  const VoxelGrid & raw, const CostVoxelGrid & inflated, const Voxel & start,
  double max_radius);

[[nodiscard]] std::optional<GoalSelection3D> closest_reachable_goal_3d(
  const VoxelGrid & raw, const CostVoxelGrid & inflated, const Voxel & start,
  const Point3 & requested,
  double timeout_ms = 150.0);

[[nodiscard]] SearchResult3D astar_3d(
  const CostVoxelGrid & grid, const Voxel & start, const Voxel & goal,
  double heuristic_weight = 1.0, double cost_weight = 2.0,
  double timeout_ms = 150.0);

// Conservative continuous proof: the segment must remain in the map and may
// not intersect any non-free voxel expanded by the spherical envelope. The
// box expansion is conservative at voxel corners (safe, potentially rejecting
// a geometrically valid chord).
[[nodiscard]] bool swept_sphere_clear(
  const VoxelGrid & raw, const Point3 & start, const Point3 & end,
  double radius);

[[nodiscard]] std::vector<Point3> simplify_path_3d(
  const VoxelGrid & raw, const std::vector<Point3> & path, double radius);

[[nodiscard]] double path_length_3d(const std::vector<Point3> & path);

[[nodiscard]] PlanResult3D plan_path_3d(
  const VoxelGrid & raw, const Point3 & start, const Point3 & requested_goal,
  const Planner3DConfig & config = {});

}  // namespace px4_vio_bridge
