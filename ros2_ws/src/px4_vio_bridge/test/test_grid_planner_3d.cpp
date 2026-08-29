#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

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
