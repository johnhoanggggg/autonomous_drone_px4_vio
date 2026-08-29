// Unit cover for the C++ grid planner. The exhaustive behavioural check is
// test_grid_planner_parity.py, which diffs this against grid_planner.py over
// randomized maps; these are the invariants worth failing fast on.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "px4_vio_bridge/grid_planner.hpp"

using px4_vio_bridge::Cell;
using px4_vio_bridge::CostGrid;
using px4_vio_bridge::GridMap;
using px4_vio_bridge::LETHAL;
using px4_vio_bridge::Point2;

namespace
{

// A 10x10 metre-ish room at 0.1 m: all free unless a test marks otherwise.
GridMap open_room(int width = 10, int height = 10)
{
  GridMap grid;
  grid.width = static_cast<std::size_t>(width);
  grid.height = static_cast<std::size_t>(height);
  grid.resolution = 0.1;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.data.assign(static_cast<std::size_t>(width * height), 0);
  return grid;
}

void set_cell(GridMap & grid, int x, int y, int value)
{
  grid.data[static_cast<std::size_t>(y) * grid.width + static_cast<std::size_t>(x)] =
    static_cast<std::int8_t>(value);
}

}  // namespace

TEST(GridPlannerCpp, InflationMarksLethalDiscAndPreservesUnknown)
{
  auto grid = open_room();
  set_cell(grid, 5, 5, 100);
  set_cell(grid, 0, 0, -1);
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.15, 0.3, 3.0);

  EXPECT_EQ(inflated.value({5, 5}), LETHAL);
  EXPECT_EQ(inflated.value({5, 6}), LETHAL);      // inside the lethal radius
  EXPECT_LT(inflated.value({5, 8}), LETHAL);      // outside it, still costed
  EXPECT_GT(inflated.value({5, 8}), 0);
  EXPECT_EQ(inflated.value({0, 0}), px4_vio_bridge::UNKNOWN);  // never inflated into
}

TEST(GridPlannerCpp, UnknownAndLethalAreNotTraversable)
{
  auto grid = open_room();
  set_cell(grid, 5, 5, 100);
  set_cell(grid, 0, 0, -1);
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.15, 0.3, 3.0);

  EXPECT_FALSE(px4_vio_bridge::traversable(inflated, {5, 5}));
  EXPECT_FALSE(px4_vio_bridge::traversable(inflated, {0, 0}));
  EXPECT_FALSE(px4_vio_bridge::traversable(inflated, {-1, 0}));
  EXPECT_TRUE(px4_vio_bridge::traversable(inflated, {9, 9}));
}

TEST(GridPlannerCpp, AstarFindsAStraightRunInAnOpenRoom)
{
  const auto grid = open_room();
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.05, 0.1, 3.0);
  const auto result = px4_vio_bridge::astar(inflated, {0, 0}, {9, 0}, 1.0, 2.0, 0.0);

  ASSERT_TRUE(result.found());
  EXPECT_EQ(result.reason, "PATH_VALID");
  EXPECT_EQ(result.cells.front(), (Cell{0, 0}));
  EXPECT_EQ(result.cells.back(), (Cell{9, 0}));
  EXPECT_EQ(result.cells.size(), 10u);
}

TEST(GridPlannerCpp, AstarRefusesBlockedEndpoints)
{
  auto grid = open_room();
  set_cell(grid, 9, 0, 100);
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.05, 0.1, 3.0);

  EXPECT_EQ(px4_vio_bridge::astar(inflated, {0, 0}, {9, 0}, 1.0, 2.0, 0.0).reason, "GOAL_BLOCKED");
  EXPECT_EQ(px4_vio_bridge::astar(inflated, {9, 0}, {0, 0}, 1.0, 2.0, 0.0).reason, "START_BLOCKED");
}

TEST(GridPlannerCpp, AstarReportsNoPathAcrossAFullWall)
{
  auto grid = open_room();
  for (int y = 0; y < 10; ++y) {
    set_cell(grid, 5, y, 100);
  }
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.05, 0.1, 3.0);
  const auto result = px4_vio_bridge::astar(inflated, {0, 0}, {9, 0}, 1.0, 2.0, 0.0);

  EXPECT_FALSE(result.found());
  EXPECT_EQ(result.reason, "NO_KNOWN_PATH");
}

TEST(GridPlannerCpp, RecoverStartEscapesItsOwnInflation)
{
  auto grid = open_room();
  set_cell(grid, 5, 5, 100);
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.25, 0.35, 3.0);
  ASSERT_FALSE(px4_vio_bridge::traversable(inflated, {5, 6}));

  const auto recovered = px4_vio_bridge::recover_start(grid, inflated, {5, 6}, 65, 0.5);
  ASSERT_TRUE(recovered.has_value());
  EXPECT_TRUE(px4_vio_bridge::traversable(inflated, recovered->cell));
  EXPECT_GT(recovered->distance, 0.0);

  // A start on the obstacle itself is not merely close to it -- no recovery.
  EXPECT_FALSE(px4_vio_bridge::recover_start(grid, inflated, {5, 5}, 65, 0.5).has_value());
  // Zero radius disables recovery entirely.
  EXPECT_FALSE(px4_vio_bridge::recover_start(grid, inflated, {5, 6}, 65, 0.0).has_value());
}

TEST(GridPlannerCpp, ClosestReachableGoalStopsAtTheKnownFrontier)
{
  auto grid = open_room();
  for (int y = 0; y < 10; ++y) {
    for (int x = 6; x < 10; ++x) {
      set_cell(grid, x, y, -1);   // unknown beyond x=5
    }
  }
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.05, 0.1, 3.0);
  // Ask for a point deep inside unknown space.
  const auto selection = px4_vio_bridge::closest_reachable_goal(inflated, {0, 0}, {0.95, 0.05});

  ASSERT_TRUE(selection.has_value());
  EXPECT_TRUE(px4_vio_bridge::traversable(inflated, selection->cell));
  EXPECT_LE(selection->cell.first, 5);   // never enters the unknown
  EXPECT_GT(selection->reachable_cells, 0);
}

TEST(GridPlannerCpp, SimplifyCollapsesACollinearRunAndKeepsTheCorner)
{
  const auto grid = open_room();
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.05, 0.1, 3.0);
  const std::vector<Cell> cells{{0, 0}, {1, 0}, {2, 0}, {3, 0}, {3, 1}, {3, 2}};
  const auto simplified = px4_vio_bridge::simplify_path(inflated, cells);

  EXPECT_EQ(simplified.front(), (Cell{0, 0}));
  EXPECT_EQ(simplified.back(), (Cell{3, 2}));
  EXPECT_LT(simplified.size(), cells.size());
}

TEST(GridPlannerCpp, SimplifyReturnsEmptyWhenClearanceCannotBeMet)
{
  auto grid = open_room();
  set_cell(grid, 1, 1, 100);
  const auto inflated = px4_vio_bridge::inflate_occupancy(grid, 65, 0.05, 0.1, 3.0);
  const std::vector<Cell> cells{{0, 0}, {0, 1}, {0, 2}};
  // A clearance wider than the room cannot be honoured by any segment.
  const auto simplified = px4_vio_bridge::simplify_path(inflated, cells, false, &grid, 5.0);

  EXPECT_TRUE(simplified.empty());
}

TEST(GridPlannerCpp, GridLethalRadiusCoversTheCellDiagonal)
{
  EXPECT_DOUBLE_EQ(px4_vio_bridge::grid_lethal_radius(0.30, 0.10), 0.30 + 0.10 / std::sqrt(2.0));
  EXPECT_THROW((void)px4_vio_bridge::grid_lethal_radius(-1.0, 0.10), std::invalid_argument);
  EXPECT_THROW((void)px4_vio_bridge::grid_lethal_radius(0.30, 0.0), std::invalid_argument);
}

TEST(GridPlannerCpp, PathProjectionAndReplacementPolicy)
{
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto projection = px4_vio_bridge::path_projection(path, {0.5, 0.25});
  ASSERT_TRUE(projection.has_value());
  EXPECT_NEAR(projection->distance, 0.25, 1.0e-12);
  EXPECT_NEAR(projection->remaining, 1.5, 1.0e-12);

  EXPECT_DOUBLE_EQ(px4_vio_bridge::path_length(path), 2.0);
  // No accepted path at all -- always replace.
  EXPECT_TRUE(px4_vio_bridge::should_replace_path(std::nullopt, 1.0, 0.35, 0.10));
  // Off-corridor -- replace.
  EXPECT_TRUE(px4_vio_bridge::should_replace_path(projection, 1.49, 0.20, 0.10));
  // On-corridor and no material improvement -- keep.
  EXPECT_FALSE(px4_vio_bridge::should_replace_path(projection, 1.49, 0.35, 0.10));
  // On-corridor but materially shorter -- replace.
  EXPECT_TRUE(px4_vio_bridge::should_replace_path(projection, 1.0, 0.35, 0.10));
}

TEST(GridPlannerCpp, TrimAdvancesTheHeadOnlyOnRealProgress)
{
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  // Within the margin -- untouched, so the follower's fingerprint is unchanged.
  EXPECT_EQ(px4_vio_bridge::trim_path_to(path, {0.1, 0.0}, 0.5), path);
  // Lateral offset from the head is not progress -- also untouched.
  EXPECT_EQ(px4_vio_bridge::trim_path_to(path, {-1.0, 1.0}, 0.5), path);
  // Genuine progress along the path moves the head onto the vehicle.
  const auto trimmed = px4_vio_bridge::trim_path_to(path, {1.5, 0.0}, 0.5);
  ASSERT_EQ(trimmed.size(), 2u);
  EXPECT_NEAR(trimmed.front().first, 1.5, 1.0e-12);
  EXPECT_EQ(trimmed.back(), (Point2{2.0, 0.0}));
}
