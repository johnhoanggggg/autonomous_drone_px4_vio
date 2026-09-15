#include <gtest/gtest.h>
#include <rtabmap/core/global_map/OctoMap.h>
#include "px4_vio_bridge/test_start_clear.hpp"
#include "px4_vio_bridge/rtabmap_octomap.hpp"

namespace p = px4_vio_bridge;

TEST(TestStartClear, OverridesOnlyWholeVoxelsInsideSphereOnPublicationCopy)
{
  rtabmap::RtabmapColorOcTree source(0.05);
  source.updateNode(0.025, 0.025, 0.025, true);
  source.updateNode(0.525, 0.025, 0.025, true);
  auto cloned = p::clone_rtabmap_tree(source);
  auto & output = *cloned;
  EXPECT_GT(p::clear_test_start_sphere(output, {0.0, 0.0, 0.0}, 0.5), 0U);
  EXPECT_TRUE(source.isNodeOccupied(source.search(0.025, 0.025, 0.025)));
  ASSERT_NE(output.search(0.025, 0.025, 0.025), nullptr);
  EXPECT_FALSE(output.isNodeOccupied(output.search(0.025, 0.025, 0.025)));
  EXPECT_TRUE(output.isNodeOccupied(output.search(0.525, 0.025, 0.025)));
  EXPECT_EQ(source.search(-0.225, 0.025, 0.025), nullptr);
  EXPECT_NE(output.search(-0.225, 0.025, 0.025), nullptr);
  for (auto it = output.begin_leafs(); it != output.end_leafs(); ++it) {
    if (output.isNodeOccupied(*it)) {continue;}
    const auto half = it.getSize() * 0.5;
    const auto dx = std::abs(it.getX()) + half;
    const auto dy = std::abs(it.getY()) + half;
    const auto dz = std::abs(it.getZ()) + half;
    EXPECT_LE(dx * dx + dy * dy + dz * dz, 0.25 + 1e-10);
  }
}

TEST(TestStartClear, RejectsInvalidOrOversizedOverride)
{
  rtabmap::RtabmapColorOcTree tree(0.05);
  EXPECT_THROW(p::clear_test_start_sphere(tree, {}, 0.6), std::invalid_argument);
  EXPECT_THROW(p::clear_test_start_sphere(tree, {}, -0.1), std::invalid_argument);
  EXPECT_THROW(p::clear_test_start_sphere(tree, {NAN, 0.0, 0.0}, 0.5), std::invalid_argument);
}
