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
inline constexpr std::int16_t VOXEL_EXPLORATION_COST = 200;

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
  // When > 0, a start inside lethal_radius (e.g. after brushing a wall) plans
  // from the recovered voxel and publishes start -> recovered -> route, with
  // that first leg checked at this smaller radius. 0 keeps RECOVERY_REQUIRED.
  double recovery_connector_radius{0.0};
  // Clearance beyond lethal_radius that route shortcuts try to keep (see
  // simplify_path_3d). 0 keeps the historical lethal-radius shortcuts.
  double simplify_margin{0.0};
  bool project_blocked_goal{false};
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
  // True when path.front() -> path[1] is a recovery connector that was only
  // checked at recovery_connector_radius, not lethal_radius.
  bool recovery_connector{false};
  std::string reason;

  [[nodiscard]] bool found() const {return !path.empty();}
};

// Diagnostic only: native non-free boxes touched by a spherical envelope.
// This never changes occupancy or grants clearance to a path.
struct SphereBlockers3D
{
  bool inside_bounds{};
  std::vector<Voxel> unknown;
  std::vector<Voxel> occupied;
};

[[nodiscard]] SphereBlockers3D sphere_blockers_3d(
  const VoxelGrid & raw, const Point3 & center, double radius);

[[nodiscard]] CostVoxelGrid inflate_voxels(
  const VoxelGrid & grid, double lethal_radius, double inflation_radius,
  double cost_scaling = 3.0);

// Inflate the optimistic proposal grid once, then penalize cells which are
// unknown in the matching observed grid. Occupied cells and hard boundaries
// must already remain occupied in optimistic. Unknown is therefore usable for
// proposal generation, but A* prefers observed free space whenever practical.
[[nodiscard]] CostVoxelGrid inflate_exploration_voxels(
  const VoxelGrid & optimistic, const VoxelGrid & observed,
  double lethal_radius, double inflation_radius, double cost_scaling = 3.0,
  std::int16_t unknown_cost = VOXEL_EXPLORATION_COST);

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

// Conservative continuous proof: the segment must remain in the map and its
// minimum distance from every non-free voxel box must exceed the spherical
// envelope radius. Unknown voxels are treated as non-free.
[[nodiscard]] bool swept_sphere_clear(
  const VoxelGrid & raw, const Point3 & start, const Point3 & end,
  double radius);

// The exploration flight policy may traverse unknown space, but never known
// occupied voxels. Map bounds remain hard in both modes.
[[nodiscard]] bool swept_sphere_clear_with_unknown_policy(
  const VoxelGrid & raw, const Point3 & start, const Point3 & end,
  double radius, bool allow_unknown);

// A cylinder is convex, so keeping both segment endpoints inside its
// sphere-shrunken bounds proves the complete swept envelope remains inside.
[[nodiscard]] bool swept_sphere_inside_cylinder(
  const Point3 & start, const Point3 & end, const Point3 & center,
  double cylinder_radius, double min_z, double max_z, double sphere_radius);

// With margin > 0 a shortcut must also keep the clearance of the path it
// replaces, up to radius + margin: open-space routes keep the margin while a
// gap narrower than 2*(radius + margin) stays on the A* centre line.
[[nodiscard]] std::vector<Point3> simplify_path_3d(
  const VoxelGrid & raw, const std::vector<Point3> & path, double radius,
  double margin = 0.0);

[[nodiscard]] double path_length_3d(const std::vector<Point3> & path);

// Own one immutable map and its inflation together. Reuse for pose/goal changes;
// replace the whole object whenever the map generation or configuration changes.
class PreparedPlanner3D
{
public:
  PreparedPlanner3D(VoxelGrid raw, const Planner3DConfig & config,
    const VoxelGrid * observed_unknowns = nullptr);
  [[nodiscard]] const VoxelGrid & raw() const {return raw_;}
  [[nodiscard]] const CostVoxelGrid & costs() const {return inflated_;}
  [[nodiscard]] PlanResult3D plan(const Point3 & start, const Point3 & goal) const;

private:
  VoxelGrid raw_;
  Planner3DConfig config_;
  CostVoxelGrid inflated_;
};

// Longest continuously verified prefix of a proposed route. Unknown remains
// blocked here even when the proposal was planned optimistically.
[[nodiscard]] std::vector<Point3> verified_prefix_3d(
  const VoxelGrid & observed, const std::vector<Point3> & proposed, double radius);

[[nodiscard]] PlanResult3D plan_path_3d(
  const VoxelGrid & raw, const Point3 & start, const Point3 & requested_goal,
  const Planner3DConfig & config = {});

}  // namespace px4_vio_bridge
