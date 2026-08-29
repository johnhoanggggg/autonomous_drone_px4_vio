#include <gtest/gtest.h>

#include <map>
#include <vector>

#include <opencv2/core.hpp>
#include <rtabmap/core/Transform.h>

#include "px4_vio_bridge/grid_planner_3d.hpp"
#include "px4_vio_bridge/rtabmap_octomap.hpp"

namespace p = px4_vio_bridge;

namespace
{

cv::Mat points(std::initializer_list<cv::Point3f> values)
{
  cv::Mat result(1, static_cast<int>(values.size()), CV_32FC3);
  int column = 0;
  for (const auto & value : values) {
    result.at<cv::Point3f>(0, column++) = value;
  }
  return result;
}

cv::Mat points(const std::vector<cv::Point3f> & values)
{
  cv::Mat result(1, static_cast<int>(values.size()), CV_32FC3);
  for (std::size_t column = 0; column < values.size(); ++column) {
    result.at<cv::Point3f>(0, static_cast<int>(column)) = values[column];
  }
  return result;
}

bool occupied(const rtabmap::RtabmapColorOcTree & tree, float x, float y, float z)
{
  const auto * node = tree.search(x, y, z);
  return node != nullptr && tree.isNodeOccupied(node);
}

}  // namespace

TEST(RtabmapOctomap, PreservesObservedFreeAndTreatsGroundAsOccupied)
{
  p::RtabmapOctomapAssembler assembler;
  p::LocalGridObservation observation;
  observation.node_id = 1;
  observation.cell_size = 0.1F;
  observation.view_point = {0.0F, 0.0F, 0.5F};
  observation.ground = points({{0.0F, 0.0F, 0.0F}, {0.1F, 0.0F, 0.0F}});
  observation.obstacles = points({{0.4F, 0.0F, 0.5F}});
  observation.empty = points({{0.2F, 0.0F, 0.5F}});
  std::string error;
  ASSERT_TRUE(assembler.rebuild(
      {observation}, {{1, rtabmap::Transform::getIdentity()}}, &error)) << error;
  ASSERT_NE(assembler.tree(), nullptr);
  EXPECT_TRUE(occupied(*assembler.tree(), 0.0F, 0.0F, 0.0F));
  EXPECT_TRUE(occupied(*assembler.tree(), 0.1F, 0.0F, 0.0F));
  EXPECT_TRUE(occupied(*assembler.tree(), 0.4F, 0.0F, 0.5F));
  const auto * empty = assembler.tree()->search(0.2F, 0.0F, 0.5F);
  ASSERT_NE(empty, nullptr);
  EXPECT_FALSE(assembler.tree()->isNodeOccupied(empty));
  EXPECT_EQ(assembler.metadata().ground_cells, 2U);
  EXPECT_EQ(assembler.metadata().obstacle_cells, 1U);
  EXPECT_EQ(assembler.metadata().empty_cells, 1U);
}

TEST(RtabmapOctomap, LoopCorrectionRebuildRemovesOldVoxelLocation)
{
  p::RtabmapOctomapAssembler assembler;
  p::LocalGridObservation observation;
  observation.node_id = 7;
  observation.cell_size = 0.1F;
  observation.obstacles = points({{0.0F, 0.0F, 0.5F}});
  std::string error;
  const std::map<int, rtabmap::Transform> original{
    {7, rtabmap::Transform::getIdentity()}};
  ASSERT_TRUE(assembler.rebuild({observation}, original, &error)) << error;
  EXPECT_TRUE(occupied(*assembler.tree(), 0.0F, 0.0F, 0.5F));
  const auto original_generation = assembler.metadata().source_pose_generation;

  const std::map<int, rtabmap::Transform> corrected{
    {7, rtabmap::Transform(1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F)}};
  ASSERT_TRUE(assembler.rebuild({observation}, corrected, &error)) << error;
  EXPECT_EQ(assembler.tree()->search(0.0F, 0.0F, 0.5F), nullptr);
  EXPECT_TRUE(occupied(*assembler.tree(), 1.0F, 0.0F, 0.5F));
  EXPECT_NE(assembler.metadata().source_pose_generation, original_generation);
}

TEST(RtabmapOctomap, PoseGenerationIsDeterministicAndOrderIndependent)
{
  const std::map<int, rtabmap::Transform> poses_a{
    {1, rtabmap::Transform::getIdentity()},
    {2, rtabmap::Transform(0.2F, 0.1F, 0.3F, 0.0F, 0.0F, 0.1F)}};
  const std::map<int, rtabmap::Transform> poses_b{
    {2, rtabmap::Transform(0.2F, 0.1F, 0.3F, 0.0F, 0.0F, 0.1F)},
    {1, rtabmap::Transform::getIdentity()}};
  EXPECT_EQ(
    p::RtabmapOctomapAssembler::pose_generation(poses_a),
    p::RtabmapOctomapAssembler::pose_generation(poses_b));
}

TEST(RtabmapOctomap, RejectsMixedCellSizes)
{
  p::RtabmapOctomapAssembler assembler;
  p::LocalGridObservation first;
  first.node_id = 1;
  first.cell_size = 0.05F;
  first.empty = points({{0.0F, 0.0F, 0.0F}});
  p::LocalGridObservation second = first;
  second.node_id = 2;
  second.cell_size = 0.10F;
  std::string error;
  EXPECT_FALSE(assembler.rebuild(
      {first, second},
      {{1, rtabmap::Transform::getIdentity()}, {2, rtabmap::Transform::getIdentity()}},
      &error));
  EXPECT_EQ(error, "local grids have inconsistent cell sizes");
}

TEST(RtabmapOctomap, DenseObservedRoomSupportsAConservative3DPlan)
{
  std::vector<cv::Point3f> ground;
  std::vector<cv::Point3f> obstacles;
  std::vector<cv::Point3f> empty;
  constexpr float resolution = 0.1F;
  for (int iz = 0; iz < 16; ++iz) {
    const auto z = 0.25F + static_cast<float>(iz) * resolution;
    for (int iy = 0; iy < 40; ++iy) {
      const auto y = -1.95F + static_cast<float>(iy) * resolution;
      for (int ix = 0; ix < 40; ++ix) {
        const auto x = -1.95F + static_cast<float>(ix) * resolution;
        if (iz == 0) {
          ground.emplace_back(x, y, z);
        } else if (
          std::abs(x - 0.45F) < resolution * 0.5F && std::abs(y) <= 0.3F && z <= 1.5F)
        {
          obstacles.emplace_back(x, y, z);
        } else {
          empty.emplace_back(x, y, z);
        }
      }
    }
  }
  p::LocalGridObservation observation;
  observation.node_id = 1;
  observation.cell_size = resolution;
  observation.view_point = {-1.0F, 0.0F, 0.8F};
  observation.ground = points(ground);
  observation.obstacles = points(obstacles);
  observation.empty = points(empty);
  p::RtabmapOctomapAssembler assembler;
  std::string error;
  ASSERT_TRUE(assembler.rebuild(
      {observation}, {{1, rtabmap::Transform::getIdentity()}}, &error)) << error;
  const auto * start_cell_center = assembler.tree()->search(-0.95F, 0.05F, 0.85F);
  ASSERT_NE(start_cell_center, nullptr);
  EXPECT_FALSE(assembler.tree()->isNodeOccupied(start_cell_center));

  p::VoxelGrid grid(60, 60, 18, 0.1, {-4.0, -3.0, 0.2});
  for (int z = 0; z < 18; ++z) {
    for (int y = 0; y < 60; ++y) {
      for (int x = 0; x < 60; ++x) {
        const p::Voxel voxel{x, y, z};
        const auto center = grid.voxel_center(voxel);
        const auto * node = assembler.tree()->search(center.x, center.y, center.z);
        if (node != nullptr) {
          grid.set(
            voxel, assembler.tree()->isNodeOccupied(node) ?
            p::VoxelState::Occupied : p::VoxelState::Free);
        }
      }
    }
  }
  p::Planner3DConfig config;
  config.timeout_ms = 1000.0;
  const auto start_voxel = grid.world_to_voxel({-1.0, 0.0, 0.8});
  ASSERT_TRUE(start_voxel.has_value());
  EXPECT_EQ(grid.at(*start_voxel), p::VoxelState::Free);
  const auto inflated = p::inflate_voxels(
    grid, config.lethal_radius, config.inflation_radius, config.cost_scaling);
  EXPECT_TRUE(inflated.traversable(*start_voxel)) << inflated.at(*start_voxel);
  const auto result = p::plan_path_3d(grid, {-1.0, 0.0, 0.8}, {1.0, 0.0, 0.8}, config);
  EXPECT_TRUE(result.found()) << result.reason;
  EXPECT_EQ(result.reason, "PATH_VALID");
}
