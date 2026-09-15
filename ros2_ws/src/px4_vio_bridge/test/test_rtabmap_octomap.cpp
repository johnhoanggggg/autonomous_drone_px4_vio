#include <gtest/gtest.h>

#include <array>
#include <cmath>
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

cv::Mat colored_points(std::initializer_list<cv::Vec4f> values)
{
  cv::Mat result(1, static_cast<int>(values.size()), CV_32FC4);
  int column = 0;
  for (const auto & value : values) {
    result.at<cv::Vec4f>(0, column++) = value;
  }
  return result;
}

bool occupied(const rtabmap::RtabmapColorOcTree & tree, float x, float y, float z)
{
  const auto * node = tree.search(x, y, z);
  return node != nullptr && tree.isNodeOccupied(node);
}

struct LeafState
{
  std::array<unsigned short, 3> key{};
  unsigned depth{};
  float log_odds{};
  int node_ref{};
  int occupancy_type{};
  std::array<float, 3> point_ref{};
  std::array<unsigned char, 3> color{};

  bool operator==(const LeafState & other) const
  {
    return key == other.key && depth == other.depth && log_odds == other.log_odds &&
           node_ref == other.node_ref && occupancy_type == other.occupancy_type &&
           point_ref == other.point_ref && color == other.color;
  }
};

std::vector<LeafState> leaf_states(const rtabmap::RtabmapColorOcTree & tree)
{
  std::vector<LeafState> states;
  for (auto leaf = tree.begin_leafs(), end = tree.end_leafs(); leaf != end; ++leaf) {
    const auto point = leaf->getPointRef();
    const auto color = leaf->getColor();
    LeafState state;
    state.key = {leaf.getKey()[0], leaf.getKey()[1], leaf.getKey()[2]};
    state.depth = leaf.getDepth();
    state.log_odds = leaf->getLogOdds();
    state.node_ref = leaf->getNodeRefId();
    state.occupancy_type = leaf->getOccupancyType();
    state.point_ref = {point.x(), point.y(), point.z()};
    state.color = {color.r, color.g, color.b};
    states.push_back(state);
  }
  return states;
}

void expect_same_tree(
  const rtabmap::RtabmapColorOcTree & expected,
  const rtabmap::RtabmapColorOcTree & actual)
{
  const auto expected_leaves = leaf_states(expected);
  const auto actual_leaves = leaf_states(actual);
  ASSERT_EQ(actual_leaves.size(), expected_leaves.size());
  for (std::size_t index = 0; index < expected_leaves.size(); ++index) {
    EXPECT_EQ(actual_leaves[index].key, expected_leaves[index].key);
    EXPECT_EQ(actual_leaves[index].depth, expected_leaves[index].depth);
    EXPECT_FLOAT_EQ(actual_leaves[index].log_odds, expected_leaves[index].log_odds);
    EXPECT_EQ(actual_leaves[index].node_ref, expected_leaves[index].node_ref);
    EXPECT_EQ(actual_leaves[index].occupancy_type, expected_leaves[index].occupancy_type);
    EXPECT_EQ(actual_leaves[index].point_ref, expected_leaves[index].point_ref);
    EXPECT_EQ(actual_leaves[index].color, expected_leaves[index].color);
  }

  double expected_min[3];
  double expected_max[3];
  double actual_min[3];
  double actual_max[3];
  expected.getMetricMin(expected_min[0], expected_min[1], expected_min[2]);
  expected.getMetricMax(expected_max[0], expected_max[1], expected_max[2]);
  actual.getMetricMin(actual_min[0], actual_min[1], actual_min[2]);
  actual.getMetricMax(actual_max[0], actual_max[1], actual_max[2]);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_DOUBLE_EQ(actual_min[axis], expected_min[axis]);
    EXPECT_DOUBLE_EQ(actual_max[axis], expected_max[axis]);
  }
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

TEST(RtabmapOctomap, AcceptsColoredRosRtabmapCellsAndUsesOnlyXyz)
{
  p::RtabmapOctomapAssembler assembler;
  p::LocalGridObservation observation;
  observation.node_id = 1;
  observation.cell_size = 0.1F;
  observation.view_point = {0.0F, 0.0F, 0.5F};
  observation.obstacles = colored_points({
    {0.4F, 0.0F, 0.5F, 12345.0F},
    {0.5F, 0.0F, 0.5F, -9876.0F}});
  observation.empty = colored_points({{0.2F, 0.0F, 0.5F, 42.0F}});
  std::string error;
  ASSERT_TRUE(assembler.rebuild(
      {observation}, {{1, rtabmap::Transform::getIdentity()}}, &error)) << error;
  EXPECT_TRUE(occupied(*assembler.tree(), 0.4F, 0.0F, 0.5F));
  EXPECT_TRUE(occupied(*assembler.tree(), 0.5F, 0.0F, 0.5F));
  const auto * empty = assembler.tree()->search(0.2F, 0.0F, 0.5F);
  ASSERT_NE(empty, nullptr);
  EXPECT_FALSE(assembler.tree()->isNodeOccupied(empty));
  EXPECT_EQ(assembler.metadata().obstacle_cells, 2U);
  EXPECT_EQ(assembler.metadata().empty_cells, 1U);
}

TEST(RtabmapOctomap, RejectsHeightCollapsedCells)
{
  p::RtabmapOctomapAssembler assembler;
  p::LocalGridObservation observation;
  observation.node_id = 1;
  observation.cell_size = 0.1F;
  observation.obstacles = cv::Mat(1, 1, CV_32FC2, cv::Scalar(0.4F, 0.0F));
  std::string error;
  EXPECT_FALSE(assembler.rebuild(
      {observation}, {{1, rtabmap::Transform::getIdentity()}}, &error));
  EXPECT_NE(error.find("CV_32FC3 or CV_32FC4"), std::string::npos);
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

TEST(RtabmapOctomap, IncrementalUpdateAddsNodesAndRebuildsAfterLoopCorrection)
{
  p::RtabmapOctomapAssembler assembler;
  p::LocalGridObservation first;
  first.node_id = 1;
  first.cell_size = 0.1F;
  first.obstacles = points({{0.0F, 0.0F, 0.5F}});
  p::LocalGridObservation second = first;
  second.node_id = 2;
  std::map<int, rtabmap::Transform> poses = {
    {1, rtabmap::Transform::getIdentity()},
  };
  std::string error;
  ASSERT_TRUE(assembler.update({first}, poses, 0.01F, 0.01F, &error)) << error;
  EXPECT_TRUE(occupied(*assembler.tree(), 0.0F, 0.0F, 0.5F));

  poses.emplace(2, rtabmap::Transform(1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F));
  ASSERT_TRUE(assembler.update({first, second}, poses, 0.01F, 0.01F, &error)) << error;
  EXPECT_EQ(assembler.metadata().source_nodes, 2U);
  EXPECT_TRUE(occupied(*assembler.tree(), 1.0F, 0.0F, 0.5F));

  poses.at(1) = rtabmap::Transform(0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  ASSERT_TRUE(assembler.update({first, second}, poses, 0.01F, 0.01F, &error)) << error;
  EXPECT_EQ(assembler.tree()->search(0.0F, 0.0F, 0.5F), nullptr);
  EXPECT_TRUE(occupied(*assembler.tree(), 0.5F, 0.0F, 0.5F));
}

TEST(RtabmapOctomap, IncrementalUpdateRebuildsAfterPureRotationCorrection)
{
  p::RtabmapOctomapAssembler assembler;
  p::LocalGridObservation observation;
  observation.node_id = 1;
  observation.cell_size = 0.1F;
  observation.obstacles = points({{1.0F, 0.0F, 0.5F}});
  std::map<int, rtabmap::Transform> poses = {
    {1, rtabmap::Transform::getIdentity()},
  };
  std::string error;
  ASSERT_TRUE(assembler.update({observation}, poses, 0.01F, 0.01F, &error)) << error;
  EXPECT_TRUE(occupied(*assembler.tree(), 1.0F, 0.0F, 0.5F));

  poses.at(1) = rtabmap::Transform(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.5707963F);
  ASSERT_TRUE(assembler.update({observation}, poses, 0.01F, 0.01F, &error)) << error;
  EXPECT_FALSE(occupied(*assembler.tree(), 1.0F, 0.0F, 0.5F));
  EXPECT_TRUE(occupied(*assembler.tree(), 0.0F, 1.0F, 0.5F));
}

TEST(RtabmapOctomap, PureRollCorrectionRebuildsMap)
{
  p::RtabmapOctomapAssembler assembler;
  p::LocalGridObservation observation;
  observation.node_id = 1;
  observation.cell_size = 0.1F;
  observation.obstacles = points({{1.0F, 0.0F, 0.5F}});
  std::map<int, rtabmap::Transform> poses{{1, rtabmap::Transform::getIdentity()}};
  std::string error;
  ASSERT_TRUE(assembler.update({observation}, poses, 0.01F, 0.01F, &error)) << error;
  poses.at(1) = rtabmap::Transform(0.0F, 0.0F, 0.0F, 1.5707963F, 0.0F, 0.0F);
  ASSERT_TRUE(assembler.update({observation}, poses, 0.01F, 0.01F, &error)) << error;
  EXPECT_FALSE(occupied(*assembler.tree(), 1.0F, 0.0F, 0.5F));
  EXPECT_TRUE(occupied(*assembler.tree(), 1.0F, -0.5F, 0.0F));
  EXPECT_EQ(assembler.full_rebuilds(), 2U);
}

TEST(RtabmapOctomap, CheckpointRemovalMatchesCompleteRebuildIncludingBounds)
{
  p::LocalGridObservation first;
  first.node_id = 1;
  first.cell_size = 0.1F;
  first.view_point = {0.0F, 0.0F, 0.5F};
  first.obstacles = points({{0.0F, 0.0F, 0.5F}});
  first.empty = points({{0.2F, 0.0F, 0.5F}});
  p::LocalGridObservation second = first;
  second.node_id = 2;
  second.obstacles = points({{0.0F, 0.0F, 0.5F}});
  p::LocalGridObservation transient = first;
  transient.node_id = 3;
  transient.obstacles = points({{0.0F, 0.0F, 0.5F}});
  p::LocalGridObservation replacement = transient;
  replacement.node_id = 4;
  replacement.obstacles = points({{0.0F, 0.0F, 0.5F}});

  p::RtabmapOctomapAssembler optimized;
  p::RtabmapOctomapAssembler complete_rebuild;
  complete_rebuild.set_use_checkpoints(false);
  std::string optimized_error;
  std::string complete_error;
  const auto tolerance_m = 0.01F;
  const auto tolerance_rad = 0.01F;
  std::map<int, rtabmap::Transform> poses{{1, rtabmap::Transform::getIdentity()}};
  ASSERT_TRUE(optimized.update({first}, poses, tolerance_m, tolerance_rad, &optimized_error))
    << optimized_error;
  ASSERT_TRUE(complete_rebuild.update({first}, poses, tolerance_m, tolerance_rad, &complete_error))
    << complete_error;

  poses.emplace(2, rtabmap::Transform(1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F));
  ASSERT_TRUE(optimized.update({first, second}, poses, tolerance_m, tolerance_rad, &optimized_error))
    << optimized_error;
  ASSERT_TRUE(complete_rebuild.update({first, second}, poses, tolerance_m, tolerance_rad, &complete_error))
    << complete_error;
  poses.emplace(3, rtabmap::Transform(2.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F));
  ASSERT_TRUE(optimized.update({first, second, transient}, poses, tolerance_m, tolerance_rad, &optimized_error))
    << optimized_error;
  ASSERT_TRUE(complete_rebuild.update({first, second, transient}, poses, tolerance_m, tolerance_rad, &complete_error))
    << complete_error;

  // Node 3 disappears without a replacement observation. The checkpoint is
  // the exact prefix containing nodes 1 and 2, so this update must restore it
  // and leave no occupied or free evidence contributed only by node 3.
  poses.erase(3);
  ASSERT_TRUE(optimized.update({first, second, transient}, poses, tolerance_m, tolerance_rad, &optimized_error))
    << optimized_error;
  ASSERT_TRUE(complete_rebuild.update({first, second, transient}, poses, tolerance_m, tolerance_rad, &complete_error))
    << complete_error;
  ASSERT_NE(optimized.tree(), nullptr);
  ASSERT_NE(complete_rebuild.tree(), nullptr);
  expect_same_tree(*complete_rebuild.tree(), *optimized.tree());
  EXPECT_EQ(optimized.checkpoint_restores(), 1U);
  EXPECT_EQ(complete_rebuild.checkpoint_restores(), 0U);
  EXPECT_EQ(optimized.full_rebuilds(), 1U);
  EXPECT_EQ(complete_rebuild.full_rebuilds(), 2U);

  poses.emplace(4, rtabmap::Transform(3.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F));
  ASSERT_TRUE(optimized.update(
      {first, second, transient, replacement}, poses, tolerance_m, tolerance_rad,
      &optimized_error)) << optimized_error;
  ASSERT_TRUE(complete_rebuild.update(
      {first, second, transient, replacement}, poses, tolerance_m, tolerance_rad,
      &complete_error)) << complete_error;
  expect_same_tree(*complete_rebuild.tree(), *optimized.tree());
  EXPECT_TRUE(occupied(*optimized.tree(), 3.0F, 0.0F, 0.5F));
}

TEST(RtabmapOctomap, ReplacingNewestAndChangingStableGraphMatchesUncachedMap)
{
  p::RtabmapOctomapAssembler optimized, baseline;
  baseline.set_use_checkpoints(false);
  std::vector<p::LocalGridObservation> observations;
  std::map<int, rtabmap::Transform> poses;
  auto add = [&](int id, float x) {
      p::LocalGridObservation grid;
      grid.node_id = id; grid.cell_size = 0.1F; grid.view_point = {0.0F, 0.0F, 0.5F};
      grid.ground = points({{0.3F, 0.0F, 0.0F}});
      grid.obstacles = points({{0.6F, 0.0F, 0.5F}, {0.7F, 0.1F, 0.6F}});
      grid.empty = points({{0.2F, 0.0F, 0.5F}, {0.3F, 0.1F, 0.6F}});
      observations.push_back(grid);
      poses[id] = rtabmap::Transform(x, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    };
  auto compare = [&]() {
      std::string error;
      ASSERT_TRUE(optimized.update(observations, poses, 0.01F, 0.01F, &error)) << error;
      ASSERT_TRUE(baseline.update(observations, poses, 0.01F, 0.01F, &error)) << error;
      expect_same_tree(*baseline.tree(), *optimized.tree());
      p::RtabmapOctomapAssembler reference;
      ASSERT_TRUE(reference.rebuild(observations, poses, &error)) << error;
      expect_same_tree(*reference.tree(), *optimized.tree());
      EXPECT_EQ(optimized.metadata().source_nodes, baseline.metadata().source_nodes);
      EXPECT_EQ(optimized.metadata().source_pose_generation, baseline.metadata().source_pose_generation);
    };
  add(1, 0.0F); compare();
  add(2, 0.4F); compare();
  add(3, 0.8F); compare();
  for (int id = 4; id <= 7; ++id) {
    poses.erase(id - 1);
    add(id, static_cast<float>(id) * 0.4F);
    compare();
  }
  EXPECT_EQ(optimized.checkpoint_restores(), 4U);
  EXPECT_EQ(optimized.full_rebuilds(), 1U);
  EXPECT_EQ(baseline.full_rebuilds(), 5U);
  // Removing persistent evidence cannot use the old prefix checkpoint.
  poses.erase(1);
  compare();
  EXPECT_EQ(optimized.full_rebuilds(), 2U);
  poses[2] = rtabmap::Transform(0.4F, 0.0F, 0.0F, 0.0F, 0.0F, 0.3F);
  compare();
  EXPECT_EQ(optimized.full_rebuilds(), 3U);
  poses[2] = rtabmap::Transform(0.6F, 0.0F, 0.0F, 0.0F, 0.0F, 0.3F);
  compare();
  EXPECT_EQ(optimized.full_rebuilds(), 4U);
}

TEST(RtabmapOctomap, RebuildSeedsExactCheckpointAfterToleratedPoseChanges)
{
  p::LocalGridObservation observation;
  observation.cell_size = 0.05F;
  observation.obstacles = points({{1.0F, 0.0F, 0.5F}});
  observation.empty = points({{0.6F, 0.0F, 0.5F}});
  std::vector<p::LocalGridObservation> grids;
  std::map<int, rtabmap::Transform> poses;
  p::RtabmapOctomapAssembler optimized, baseline;
  baseline.set_use_checkpoints(false);
  std::string error;
  for (int id = 1; id <= 7; ++id) {
    observation.node_id = id;
    grids.push_back(observation);
    poses[id] = rtabmap::Transform(id * 0.2F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    if (id == 3) {
      poses.at(1) = rtabmap::Transform(0.203F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    }
    if (id >= 5) {poses.erase(id - 1);}
    ASSERT_TRUE(optimized.update(grids, poses, 0.01F, 0.01F, &error)) << error;
    ASSERT_TRUE(baseline.update(grids, poses, 0.01F, 0.01F, &error)) << error;
    expect_same_tree(*optimized.tree(), *baseline.tree());
  }
  EXPECT_EQ(optimized.full_rebuilds(), 2U);
  EXPECT_EQ(optimized.checkpoint_restores(), 2U);
  EXPECT_EQ(baseline.full_rebuilds(), 4U);
}

TEST(RtabmapOctomap, LowerIdRetrievalFallsBackInsteadOfChangingRayOrder)
{
  p::RtabmapOctomapAssembler optimized, baseline;
  baseline.set_use_checkpoints(false);
  std::vector<p::LocalGridObservation> observations;
  std::map<int, rtabmap::Transform> poses;
  auto add = [&](int id) {
      p::LocalGridObservation grid;
      grid.node_id = id; grid.cell_size = 0.1F; grid.view_point = {0.0F, 0.0F, 0.5F};
      grid.obstacles = points({{0.8F, 0.0F, 0.5F}});
      grid.empty = points({{0.4F, 0.0F, 0.5F}});
      observations.push_back(grid);
      poses[id] = rtabmap::Transform(static_cast<float>(id) * 0.03F,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
    };
  std::string error;
  for (const int id : {10, 20, 30}) {
    add(id);
    ASSERT_TRUE(optimized.update(observations, poses, 0.01F, 0.01F, &error)) << error;
    ASSERT_TRUE(baseline.update(observations, poses, 0.01F, 0.01F, &error)) << error;
  }
  poses.erase(30); add(5); add(31);
  ASSERT_TRUE(optimized.update(observations, poses, 0.01F, 0.01F, &error)) << error;
  ASSERT_TRUE(baseline.update(observations, poses, 0.01F, 0.01F, &error)) << error;
  expect_same_tree(*baseline.tree(), *optimized.tree());
  EXPECT_EQ(optimized.checkpoint_restores(), 0U);
  EXPECT_EQ(optimized.full_rebuilds(), 2U);
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
