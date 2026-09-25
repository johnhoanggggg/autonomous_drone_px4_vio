#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "px4_vio_bridge/grid_planner_3d.hpp"

namespace p = px4_vio_bridge;

TEST(VoxelGrid3D, ConvertsWorldCoordinatesAndChecksBounds)
{
  p::VoxelGrid grid(4, 5, 6, 0.1, {-0.2, -0.3, 0.4});
  const auto voxel = grid.world_to_voxel({0.05, -0.05, 0.75});
  ASSERT_TRUE(voxel.has_value());
  EXPECT_EQ(voxel->x, 2);
  EXPECT_EQ(voxel->y, 2);
  EXPECT_EQ(voxel->z, 3);
  EXPECT_FALSE(grid.world_to_voxel({1.0, 0.0, 0.0}).has_value());
}

TEST(GridPlanner3D, ForbidsThreeAxisCornerCutting)
{
  p::VoxelGrid raw(3, 3, 3, 0.1, {0.0, 0.0, 0.0});
  p::CostVoxelGrid costs(raw);
  costs.set({0, 0, 0}, 0);
  costs.set({1, 1, 1}, 0);
  const auto neighbors = p::traversable_neighbors_3d(costs, {0, 0, 0});
  EXPECT_TRUE(std::none_of(neighbors.begin(), neighbors.end(), [](const auto & edge) {
      return edge.first == p::Voxel{1, 1, 1};
  }));
  costs.set({1, 0, 0}, 0);
  costs.set({0, 1, 0}, 0);
  costs.set({0, 0, 1}, 0);
  costs.set({1, 1, 0}, 0);
  costs.set({1, 0, 1}, 0);
  costs.set({0, 1, 1}, 0);
  const auto clear_neighbors = p::traversable_neighbors_3d(costs, {0, 0, 0});
  EXPECT_TRUE(std::any_of(clear_neighbors.begin(), clear_neighbors.end(), [](const auto & edge) {
      return edge.first == p::Voxel{1, 1, 1};
  }));
}

TEST(GridPlanner3D, UnknownInflatesAsBlocked)
{
  p::VoxelGrid raw(12, 12, 12, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({6, 6, 6}, p::VoxelState::Unknown);
  const auto costs = p::inflate_voxels(raw, 0.10, 0.20);
  EXPECT_FALSE(costs.traversable({7, 6, 6}));
  EXPECT_TRUE(costs.traversable({9, 6, 6}));
}

TEST(GridPlanner3D, DiagonalNeedsEveryIntermediateVoxel)
{
  p::VoxelGrid raw(4, 4, 4, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  p::CostVoxelGrid costs(raw);
  for (int z = 0; z < 4; ++z) {
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        costs.set({x, y, z}, 0);
      }
    }
  }
  costs.set({2, 1, 1}, p::VOXEL_LETHAL_COST);
  const auto result = p::astar_3d(costs, {1, 1, 1}, {2, 2, 2});
  ASSERT_TRUE(result.found());
  EXPECT_GT(result.voxels.size(), 2U);
}

TEST(GridPlanner3D, SweptSphereRejectsUnknownAndOutsideVolume)
{
  p::VoxelGrid raw(20, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  EXPECT_TRUE(p::swept_sphere_clear(raw, {0.5, 0.5, 0.5}, {1.5, 0.5, 0.5}, 0.1));
  raw.set({10, 5, 5}, p::VoxelState::Unknown);
  EXPECT_FALSE(p::swept_sphere_clear(raw, {0.5, 0.55, 0.55}, {1.5, 0.55, 0.55}, 0.1));
  EXPECT_FALSE(p::swept_sphere_clear(raw, {0.05, 0.5, 0.5}, {0.5, 0.5, 0.5}, 0.1));
}

TEST(GridPlanner3D, ExactSphereClearanceAllowsVoxelizedHalfMetreBubble)
{
  constexpr double resolution = 0.05;
  constexpr double bubble_radius = 0.50;
  constexpr int extent = 25;
  p::VoxelGrid raw(extent, extent, extent, resolution, {-0.625, -0.625, -0.625});
  const auto half = resolution * 0.5;
  for (int z = 0; z < extent; ++z) {
    for (int y = 0; y < extent; ++y) {
      for (int x = 0; x < extent; ++x) {
        const auto center = raw.voxel_center({x, y, z});
        const auto outside_x = std::abs(center.x) + half;
        const auto outside_y = std::abs(center.y) + half;
        const auto outside_z = std::abs(center.z) + half;
        if (std::sqrt(outside_x * outside_x + outside_y * outside_y +
          outside_z * outside_z) <= bubble_radius + 1.0e-12)
        {
          raw.set({x, y, z}, p::VoxelState::Free);
        }
      }
    }
  }
  const auto costs = p::inflate_voxels(raw, 0.35, 0.45);
  const auto start = raw.voxel_center({12, 12, 12});
  const auto short_goal = raw.voxel_center({13, 12, 12});
  EXPECT_TRUE(costs.traversable({12, 12, 12}));
  EXPECT_TRUE(costs.traversable({13, 12, 12}));
  EXPECT_TRUE(p::swept_sphere_clear(raw, start, start, 0.35));
  EXPECT_TRUE(p::swept_sphere_clear(raw, start, short_goal, 0.35));
  p::Planner3DConfig config;
  config.lethal_radius = 0.35;
  config.inflation_radius = 0.45;
  config.timeout_ms = 500.0;
  const auto result = p::plan_path_3d(raw, start, short_goal, config);
  ASSERT_TRUE(result.found()) << result.reason;
  EXPECT_EQ(result.reason, "PATH_VALID");
}

TEST(GridPlanner3D, ExactSphereRejectsSideAndCornerContacts)
{
  p::VoxelGrid raw(20, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({5, 5, 5}, p::VoxelState::Occupied);
  EXPECT_FALSE(p::swept_sphere_clear(raw, {0.4, 0.55, 0.55},
    {0.4, 0.55, 0.55}, 0.1));
  const auto corner = 0.5 - 0.1 / std::sqrt(3.0);
  EXPECT_FALSE(p::swept_sphere_clear(raw, {corner, corner, corner},
    {corner, corner, corner}, 0.1));
}

TEST(GridPlanner3D, ExactSphereIncludesLowerEnumerationBoundaryTangency)
{
  p::VoxelGrid raw(20, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({4, 5, 5}, p::VoxelState::Occupied);
  // The sphere reaches x=0.5, exactly the occupied box's upper x face. This
  // voxel lies immediately below the old floor-derived candidate lower bound.
  EXPECT_FALSE(p::swept_sphere_clear(raw, {0.6, 0.55, 0.55},
    {0.6, 0.55, 0.55}, 0.1));
}

TEST(GridPlanner3D, ExactSphereAcceptsDiagonalNearMiss)
{
  p::VoxelGrid raw(20, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({5, 5, 5}, p::VoxelState::Occupied);
  EXPECT_TRUE(p::swept_sphere_clear(raw, {0.42, 0.42, 0.42},
    {0.44, 0.44, 0.44}, 0.1));
}

TEST(GridPlanner3D, ExactSphereFindsInteriorSegmentMinimum)
{
  p::VoxelGrid raw(20, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({5, 5, 5}, p::VoxelState::Occupied);
  const p::Point3 start{0.3, 0.55, 0.4};
  const p::Point3 end{0.8, 0.55, 0.4};
  EXPECT_FALSE(p::swept_sphere_clear(raw, start, end, 0.1));
  EXPECT_TRUE(p::swept_sphere_clear(raw, start, end, 0.099));
}

TEST(GridPlanner3D, EuclideanInflationIsConservativeAgainstBoxDistances)
{
  constexpr double resolution = 0.1;
  p::VoxelGrid raw(9, 9, 9, resolution, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  std::uint32_t state = 0x12345678U;
  for (int sample = 0; sample < 100; ++sample) {
    state = state * 1664525U + 1013904223U;
    const p::Voxel source{
      static_cast<int>((state >> 1U) % 9U), static_cast<int>((state >> 9U) % 9U),
      static_cast<int>((state >> 17U) % 9U)};
    state = state * 1664525U + 1013904223U;
    p::Voxel query{
      1 + static_cast<int>((state >> 1U) % 7U), 1 + static_cast<int>((state >> 9U) % 7U),
      1 + static_cast<int>((state >> 17U) % 7U)};
    if (query == source) {query.x = (query.x % 7) + 1;}
    raw.set(source, p::VoxelState::Occupied);
    const auto source_lower = raw.voxel_center(source);
    const auto half = resolution * 0.5;
    const auto query_center = raw.voxel_center(query);
    const auto dx = std::max(std::abs(query_center.x - source_lower.x) - half, 0.0);
    const auto dy = std::max(std::abs(query_center.y - source_lower.y) - half, 0.0);
    const auto dz = std::max(std::abs(query_center.z - source_lower.z) - half, 0.0);
    const auto box_distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const auto costs = p::inflate_voxels(raw, box_distance + 1.0e-8,
      box_distance + 0.25);
    EXPECT_EQ(costs.at(query), p::VOXEL_LETHAL_COST) << "sample " << sample;
    raw.set(source, p::VoxelState::Free);
  }
}

TEST(GridPlanner3D, EuclideanInflationMatchesMultiSourceCenterDistanceOracle)
{
  constexpr double resolution = 0.1;
  constexpr double lethal_radius = 0.01;
  constexpr double inflation_radius = 0.15;
  constexpr double cost_scaling = 2.3;
  constexpr int extent = 6;
  const auto half_diagonal = std::sqrt(3.0) * resolution * 0.5;
  std::uint32_t state = 0xC001D00DU;
  for (int sample = 0; sample < 100; ++sample) {
    p::VoxelGrid raw(extent, extent, extent, resolution, {0.0, 0.0, 0.0});
    for (int z = 0; z < extent; ++z) {
      for (int y = 0; y < extent; ++y) {
        for (int x = 0; x < extent; ++x) {
          state = state * 1664525U + 1013904223U;
          const auto draw = (state >> 28U) % 10U;
          raw.set({x, y, z}, draw < 2U ? p::VoxelState::Unknown :
            (draw < 4U ? p::VoxelState::Occupied : p::VoxelState::Free));
        }
      }
    }
    raw.set({0, 0, 0}, p::VoxelState::Occupied);
    const auto costs = p::inflate_voxels(raw, lethal_radius, inflation_radius, cost_scaling);
    const auto upper = raw.upper_bound();
    for (int z = 0; z < extent; ++z) {
      for (int y = 0; y < extent; ++y) {
        for (int x = 0; x < extent; ++x) {
          const p::Voxel query{x, y, z};
          if (raw.at(query) != p::VoxelState::Free) {continue;}
          const auto center = raw.voxel_center(query);
          double nearest_center_distance = std::numeric_limits<double>::infinity();
          for (int source_z = 0; source_z < extent; ++source_z) {
            for (int source_y = 0; source_y < extent; ++source_y) {
              for (int source_x = 0; source_x < extent; ++source_x) {
                const p::Voxel source{source_x, source_y, source_z};
                if (raw.at(source) == p::VoxelState::Free) {continue;}
                const auto source_center = raw.voxel_center(source);
                const auto dx = center.x - source_center.x;
                const auto dy = center.y - source_center.y;
                const auto dz = center.z - source_center.z;
                nearest_center_distance = std::min(nearest_center_distance,
                  std::sqrt(dx * dx + dy * dy + dz * dz));
              }
            }
          }
          const auto boundary_clearance = std::min({
              center.x - raw.origin().x, upper.x - center.x,
              center.y - raw.origin().y, upper.y - center.y,
              center.z - raw.origin().z, upper.z - center.z});
          const auto clearance = std::min(boundary_clearance,
            std::max(0.0, nearest_center_distance - half_diagonal));
          std::int16_t expected = 0;
          if (clearance <= lethal_radius + 1.0e-12) {
            expected = p::VOXEL_LETHAL_COST;
          } else if (clearance <= inflation_radius + 1.0e-12) {
            const auto span = std::max(resolution, inflation_radius - lethal_radius);
            const auto decay = std::exp(-cost_scaling *
                (clearance - lethal_radius) / span);
            expected = static_cast<std::int16_t>(std::clamp(
                static_cast<int>(std::nearbyint((p::VOXEL_LETHAL_COST - 1) * decay)),
                1, static_cast<int>(p::VOXEL_LETHAL_COST - 1)));
          }
          EXPECT_EQ(costs.at(query), expected) << "sample " << sample << " voxel " <<
            x << "," << y << "," << z;
        }
      }
    }
  }
}

TEST(GridPlanner3D, OpenVolumePreservesRequestedXYZGoal)
{
  p::VoxelGrid raw(24, 24, 16, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  p::Planner3DConfig config;
  config.lethal_radius = 0.10;
  config.inflation_radius = 0.20;
  config.timeout_ms = 500.0;
  const p::Point3 start{0.55, 0.55, 0.55};
  const p::Point3 goal{1.47, 1.33, 0.87};
  const auto result = p::plan_path_3d(raw, start, goal, config);
  ASSERT_TRUE(result.found()) << result.reason;
  ASSERT_TRUE(result.goal.has_value());
  EXPECT_TRUE(result.goal->exact);
  EXPECT_TRUE(result.goal->terminal);
  EXPECT_NEAR(result.path.front().x, start.x, 1.0e-12);
  EXPECT_NEAR(result.path.back().x, goal.x, 1.0e-12);
  EXPECT_NEAR(result.path.back().y, goal.y, 1.0e-12);
  EXPECT_NEAR(result.path.back().z, goal.z, 1.0e-12);
  EXPECT_EQ(result.reason, "PATH_VALID");
}

TEST(GridPlanner3D, UnknownGoalIsNeverTerminal)
{
  p::VoxelGrid raw(24, 24, 16, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({14, 13, 8}, p::VoxelState::Unknown);
  p::Planner3DConfig config;
  config.lethal_radius = 0.10;
  config.inflation_radius = 0.20;
  config.timeout_ms = 500.0;
  const auto result = p::plan_path_3d(
    raw, {0.55, 0.55, 0.55}, {1.45, 1.35, 0.85}, config);
  ASSERT_TRUE(result.found()) << result.reason;
  ASSERT_TRUE(result.goal.has_value());
  EXPECT_FALSE(result.goal->exact);
  EXPECT_FALSE(result.goal->terminal);
  EXPECT_EQ(result.reason, "EXPLORING");
}

TEST(GridPlanner3D, BlockedGoalProducesTerminalSafeApproach)
{
  p::VoxelGrid raw(24, 24, 16, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({14, 13, 8}, p::VoxelState::Occupied);
  p::Planner3DConfig config;
  config.lethal_radius = 0.10;
  config.inflation_radius = 0.20;
  config.timeout_ms = 500.0;
  const auto result = p::plan_path_3d(
    raw, {0.55, 0.55, 0.55}, {1.45, 1.35, 0.85}, config);
  ASSERT_TRUE(result.found()) << result.reason;
  ASSERT_TRUE(result.goal.has_value());
  EXPECT_FALSE(result.goal->exact);
  EXPECT_TRUE(result.goal->terminal);
  EXPECT_EQ(result.reason, "SAFE_APPROACH");
}

TEST(GridPlanner3D, FindsOnlyMonotonicallySaferRecoveryTarget)
{
  p::VoxelGrid raw(14, 14, 14, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({5, 5, 5}, p::VoxelState::Occupied);
  const auto costs = p::inflate_voxels(raw, 0.05, 0.25);
  ASSERT_FALSE(costs.traversable({6, 5, 5}));
  const auto recovered = p::recover_start_3d(raw, costs, {6, 5, 5}, 0.4);
  ASSERT_TRUE(recovered.has_value());
  EXPECT_TRUE(costs.traversable(*recovered));
  EXPECT_LT(costs.at(*recovered), costs.at({6, 5, 5}));
}

TEST(GridPlanner3D, ConnectsClearOffCenterBodyWithoutFalseRecovery)
{
  p::VoxelGrid raw(24, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  for (int z = 0; z < 20; ++z) {
    for (int y = 0; y < 20; ++y) {raw.set({5, y, z}, p::VoxelState::Occupied);}
  }
  p::Planner3DConfig config;
  config.lethal_radius = 0.075;
  config.inflation_radius = 0.2;
  const p::Point3 start{0.699, 1.05, 1.05}, goal{1.55, 1.05, 1.05};
  ASSERT_TRUE(p::swept_sphere_clear(raw, start, start, config.lethal_radius));
  const auto costs = p::inflate_voxels(raw, config.lethal_radius, config.inflation_radius);
  ASSERT_FALSE(costs.traversable(*raw.world_to_voxel(start)));
  const auto result = p::plan_path_3d(raw, start, goal, config);
  ASSERT_TRUE(result.found()) << result.reason;
  EXPECT_EQ(result.reason, "PATH_VALID");
  EXPECT_DOUBLE_EQ(result.path.front().x, start.x);
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    EXPECT_TRUE(p::swept_sphere_clear(raw, result.path[i - 1], result.path[i], config.lethal_radius));
  }
  // Nearby safe cells must never authorize escaping an actually blocked body.
  const auto blocked = p::plan_path_3d(raw, {0.65, 1.05, 1.05}, goal, config);
  EXPECT_FALSE(blocked.found());
  EXPECT_EQ(blocked.reason, "RECOVERY_REQUIRED");
}

TEST(GridPlanner3D, PreparedMapOwnsItsEvidenceAndReplacementChangesClearance)
{
  p::VoxelGrid raw(24, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  const p::Point3 start{0.65, 1.05, 1.05}, goal{1.55, 1.05, 1.05};
  const p::PreparedPlanner3D original(raw, {});
  ASSERT_EQ(original.plan(start, goal).reason, "PATH_VALID");
  raw.set(*raw.world_to_voxel(start), p::VoxelState::Occupied);
  EXPECT_EQ(original.plan(start, goal).reason, "PATH_VALID");
  const p::PreparedPlanner3D replacement(std::move(raw), {});
  EXPECT_FALSE(replacement.plan(start, goal).found());
}

TEST(GridPlanner3D, ReferenceRouteCrossesUnknownButItsVerifiedPrefixDoesNot)
{
  p::VoxelGrid observed(40, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  for (int z = 0; z < 20; ++z) {
    for (int y = 0; y < 20; ++y) {observed.set({15, y, z}, p::VoxelState::Unknown);}
  }
  auto proposed_map = observed;
  for (int z = 0; z < 20; ++z) {
    for (int y = 0; y < 20; ++y) {proposed_map.set({15, y, z}, p::VoxelState::Free);}
  }
  p::Planner3DConfig config;
  const p::PreparedPlanner3D reference(proposed_map, config, &observed);
  EXPECT_EQ(reference.costs().at({15, 10, 10}), p::VOXEL_EXPLORATION_COST);
  EXPECT_TRUE(reference.costs().traversable({15, 10, 10}));
  const auto route = reference.plan({0.65, 1.05, 1.05}, {3.05, 1.05, 1.05});
  ASSERT_TRUE(route.found());
  ASSERT_TRUE(route.goal->exact);
  EXPECT_FALSE(p::swept_sphere_clear(observed, route.path.front(), route.path.back(), 0.35));
  const auto prefix = p::verified_prefix_3d(observed, route.path, 0.35);
  ASSERT_EQ(prefix.size(), 2U);
  EXPECT_NEAR(prefix.back().x, 1.15, 1e-6);
  EXPECT_TRUE(p::swept_sphere_clear(observed, prefix.front(), prefix.back(), 0.35));
  EXPECT_GT(reference.costs().at({15, 10, 10}), reference.costs().at({6, 10, 10}));
  // An occupied wall cannot become an optimistic route.
  for (int z = 0; z < 20; ++z) {
    for (int y = 0; y < 20; ++y) {proposed_map.set({15, y, z}, p::VoxelState::Occupied);}
  }
  const p::PreparedPlanner3D walled(proposed_map, config, &observed);
  const auto blocked = walled.plan({0.65, 1.05, 1.05}, {3.05, 1.05, 1.05});
  ASSERT_TRUE(blocked.found());
  EXPECT_FALSE(blocked.goal->exact);
  EXPECT_LT(blocked.path.back().x, 1.5);
}

TEST(GridPlanner3D, VerifiedPrefixCannotSkipAnUnseenStartOrGap)
{
  p::VoxelGrid raw(40, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  const std::vector<p::Point3> route{{0.65, 1.05, 1.05}, {1.05, 1.05, 1.05}, {3.05, 1.05, 1.05}};
  EXPECT_EQ(p::verified_prefix_3d(raw, route, 0.35).size(), route.size());
  raw.set({6, 10, 10}, p::VoxelState::Unknown);
  EXPECT_TRUE(p::verified_prefix_3d(raw, route, 0.35).empty());
}

TEST(GridPlanner3D, ProjectsLowGoalToAReachableApproachAboveFloor)
{
  p::VoxelGrid raw(40, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  for (int y = 0; y < 20; ++y) {
    for (int x = 0; x < 40; ++x) {raw.set({x, y, 3}, p::VoxelState::Occupied);}
  }
  p::Planner3DConfig config;
  config.project_blocked_goal = true;
  const auto result = p::plan_path_3d(raw, {0.65, 1.05, 0.85}, {3.05, 1.05, 0.45}, config);
  ASSERT_TRUE(result.found()) << result.reason;
  ASSERT_TRUE(result.goal);
  EXPECT_FALSE(result.goal->exact);
  EXPECT_TRUE(result.goal->terminal);
  EXPECT_NEAR(result.path.back().x, 3.05, 1e-9);
  EXPECT_NEAR(result.path.back().z, 0.85, 1e-9);
  EXPECT_TRUE(p::swept_sphere_clear(raw, result.path.front(), result.path.back(), 0.35));
}

TEST(GridPlanner3D, DiagnosticSphereReportsBoxesAndKeepsMapUnchanged)
{
  p::VoxelGrid raw(20, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({14, 10, 10}, p::VoxelState::Unknown);  // Tangent at x=1.40.
  raw.set({13, 10, 10}, p::VoxelState::Occupied);
  raw.set({14, 14, 10}, p::VoxelState::Unknown);  // In AABB, outside sphere.
  const auto original = raw.data();
  const auto blockers = p::sphere_blockers_3d(raw, {1.05, 1.05, 1.05}, 0.35);
  EXPECT_TRUE(blockers.inside_bounds);
  ASSERT_EQ(blockers.unknown.size(), 1U);
  ASSERT_EQ(blockers.occupied.size(), 1U);
  EXPECT_EQ(blockers.unknown.front(), (p::Voxel{14, 10, 10}));
  EXPECT_EQ(blockers.occupied.front(), (p::Voxel{13, 10, 10}));
  EXPECT_EQ(raw.data(), original);
  EXPECT_FALSE(p::sphere_blockers_3d(raw, {0.05, 0.05, 0.05}, 0.35).inside_bounds);
  EXPECT_THROW(p::sphere_blockers_3d(raw, {1.0, 1.0, 1.0}, -1.0), std::invalid_argument);
  EXPECT_THROW(p::sphere_blockers_3d(p::VoxelGrid{}, {0.0, 0.0, 0.0}, 0.35), std::invalid_argument);
}

TEST(GridPlanner3D, SimplificationValidatesShortPaths)
{
  p::VoxelGrid raw(20, 20, 20, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  raw.set({10, 10, 10}, p::VoxelState::Occupied);
  EXPECT_TRUE(p::simplify_path_3d(raw, {{1.05, 1.05, 1.05}}, 0.1).empty());
  EXPECT_TRUE(p::simplify_path_3d(raw,
    {{0.55, 1.05, 1.05}, {1.55, 1.05, 1.05}}, 0.1).empty());
}

TEST(GridPlanner3D, UsesActualEndpointsWithoutGridCenterKinksInOpenSpace)
{
  p::VoxelGrid raw(30, 30, 30, 0.1, {0.0, 0.0, 0.0}, p::VoxelState::Free);
  const p::Point3 start{0.62, 0.67, 0.71}, goal{2.12, 1.03, 1.21};
  auto result = p::plan_path_3d(raw, start, goal);
  ASSERT_TRUE(result.found()) << result.reason;
  ASSERT_EQ(result.path.size(), 2U);
  EXPECT_DOUBLE_EQ(result.path.front().x, start.x);
  EXPECT_DOUBLE_EQ(result.path.front().y, start.y);
  EXPECT_DOUBLE_EQ(result.path.front().z, start.z);
  EXPECT_DOUBLE_EQ(result.path.back().x, goal.x);
  EXPECT_DOUBLE_EQ(result.path.back().y, goal.y);
  EXPECT_DOUBLE_EQ(result.path.back().z, goal.z);
  EXPECT_TRUE(p::swept_sphere_clear(raw, result.path.front(), result.path.back(), 0.35));
}
