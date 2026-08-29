#include "px4_vio_bridge/grid_clearance.hpp"

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

namespace
{

px4_vio_bridge::GridMap open_grid()
{
  px4_vio_bridge::GridMap grid;
  grid.width = 20;
  grid.height = 20;
  grid.resolution = 0.05;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.data.assign(grid.width * grid.height, std::int8_t{0});
  return grid;
}

TEST(GridClearanceCpp, AcceptsKnownOpenSegment)
{
  const auto grid = open_grid();
  EXPECT_TRUE(px4_vio_bridge::segment_has_clearance(
      grid, {0.10, 0.10}, {0.80, 0.10}, 0.10));
}

TEST(GridClearanceCpp, RejectsUnknownSpace)
{
  auto grid = open_grid();
  grid.data[2 * grid.width + 8] = -1;
  EXPECT_FALSE(px4_vio_bridge::segment_has_clearance(
      grid, {0.10, 0.125}, {0.80, 0.125}, 0.05));
}

TEST(GridClearanceCpp, TreatsOccupiedCellAsFullSquare)
{
  auto grid = open_grid();
  grid.data[10 * grid.width + 10] = 100;
  // The segment lies 0.10 m below the occupied square's lower edge. Exactly
  // 0.10 m is accepted, while a slightly larger requirement is rejected.
  EXPECT_TRUE(px4_vio_bridge::segment_has_clearance(
      grid, {0.10, 0.40}, {0.90, 0.40}, 0.10));
  EXPECT_FALSE(px4_vio_bridge::segment_has_clearance(
      grid, {0.10, 0.40}, {0.90, 0.40}, 0.101));
}

TEST(GridClearanceCpp, RejectsPoseInsideClearance)
{
  auto grid = open_grid();
  grid.data[10 * grid.width + 10] = 100;
  EXPECT_FALSE(px4_vio_bridge::segment_has_clearance(
      grid, {0.495, 0.525}, {0.495, 0.525}, 0.01));
}

}  // namespace
