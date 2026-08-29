#include "px4_vio_bridge/grid_clearance.hpp"

#include <cmath>
#include <cstdint>
#include <utility>
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


TEST(GridClearanceCpp, PointClearanceIsExactToTheCellSquare)
{
  auto grid = open_grid();
  grid.data[10 * grid.width + 10] = 100;
  // The occupied square spans [0.50, 0.55] x [0.50, 0.55].
  const auto left = px4_vio_bridge::point_clearance(grid, {0.26, 0.525});
  ASSERT_TRUE(left.has_value());
  EXPECT_NEAR(*left, 0.24, 1e-12);
  // Diagonal: measured to the corner, not to the cell centre.
  const auto corner = px4_vio_bridge::point_clearance(grid, {0.20, 0.20});
  ASSERT_TRUE(corner.has_value());
  EXPECT_NEAR(*corner, std::hypot(0.30, 0.30), 1e-12);
  // Inside the square is zero: present, not absent, and never negative.
  const auto inside = px4_vio_bridge::point_clearance(grid, {0.52, 0.52});
  ASSERT_TRUE(inside.has_value());
  EXPECT_DOUBLE_EQ(*inside, 0.0);
}

TEST(GridClearanceCpp, SegmentMinimumClearanceMatchesTheClosestApproach)
{
  auto grid = open_grid();
  grid.data[10 * grid.width + 10] = 100;
  // Parallel, 0.10 m below the square's lower edge.
  const auto parallel = px4_vio_bridge::segment_minimum_clearance(
    grid, {0.10, 0.40}, {0.90, 0.40});
  ASSERT_TRUE(parallel.has_value());
  EXPECT_NEAR(*parallel, 0.10, 1e-12);
  // The minimum covers the whole chord, its start point included.
  const auto approaching = px4_vio_bridge::segment_minimum_clearance(
    grid, {0.30, 0.525}, {0.44, 0.525});
  ASSERT_TRUE(approaching.has_value());
  EXPECT_NEAR(*approaching, 0.06, 1e-12);
}

TEST(GridClearanceCpp, ObstacleFreeMapHasUnboundedClearance)
{
  const auto grid = open_grid();
  const auto clearance = px4_vio_bridge::point_clearance(grid, {0.50, 0.50});
  ASSERT_TRUE(clearance.has_value());
  EXPECT_TRUE(std::isinf(*clearance));
}

TEST(GridClearanceCpp, UnknownAndOutsideSpaceHaveNoClearance)
{
  auto grid = open_grid();
  grid.data[10 * grid.width + 10] = -1;
  EXPECT_FALSE(px4_vio_bridge::point_clearance(grid, {0.52, 0.52}).has_value());
  EXPECT_FALSE(
    px4_vio_bridge::segment_minimum_clearance(grid, {0.10, 0.525}, {0.90, 0.525})
    .has_value());
  EXPECT_FALSE(px4_vio_bridge::point_clearance(grid, {-0.10, 0.50}).has_value());
  EXPECT_FALSE(px4_vio_bridge::point_clearance(grid, {5.00, 0.50}).has_value());
}

TEST(GridClearanceCpp, ThresholdTestAgreesWithTheExactDistance)
{
  auto grid = open_grid();
  grid.data[10 * grid.width + 10] = 100;
  grid.data[4 * grid.width + 15] = 80;
  const std::vector<std::pair<px4_vio_bridge::Point2, px4_vio_bridge::Point2>> chords{
    {{0.10, 0.40}, {0.90, 0.40}},
    {{0.30, 0.525}, {0.44, 0.525}},
    {{0.70, 0.10}, {0.70, 0.90}},
    {{0.15, 0.15}, {0.85, 0.85}},
  };
  for (const auto & chord : chords) {
    const auto exact =
      px4_vio_bridge::segment_minimum_clearance(grid, chord.first, chord.second);
    ASSERT_TRUE(exact.has_value());
    for (const double required : {0.01, 0.05, 0.10, 0.20, 0.40}) {
      EXPECT_EQ(
        px4_vio_bridge::segment_has_clearance(grid, chord.first, chord.second, required),
        *exact + 1.0e-9 >= required) << "required=" << required;
    }
  }
}

}  // namespace
