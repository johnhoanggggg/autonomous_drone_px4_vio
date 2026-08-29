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

// ---------------------------------------------------------------------------
// Change 3: goal-mode hysteresis.

namespace
{
using px4_vio_bridge::GoalMode;
using px4_vio_bridge::GoalModeHysteresis;
}  // namespace

TEST(GoalModeHysteresis, FirstResultCommitsImmediately)
{
  GoalModeHysteresis mode(2);
  EXPECT_FALSE(mode.initialized());
  const auto decision = mode.observe(GoalMode::PathValid, 1);
  EXPECT_TRUE(decision.committed);
  EXPECT_FALSE(decision.has_pending);
  EXPECT_EQ(decision.stable, GoalMode::PathValid);
  EXPECT_TRUE(mode.pending_suffix().empty());
}

TEST(GoalModeHysteresis, OneMapSpikeCommitsNothing)
{
  GoalModeHysteresis mode(2);
  mode.observe(GoalMode::PathValid, 1);
  const auto spike = mode.observe(GoalMode::Exploring, 2);
  EXPECT_FALSE(spike.committed);
  EXPECT_EQ(spike.stable, GoalMode::PathValid);
  EXPECT_TRUE(spike.has_pending);
  EXPECT_EQ(spike.pending, GoalMode::Exploring);
  EXPECT_EQ(spike.pending_count, 1);
  EXPECT_EQ(mode.pending_suffix(), " MODE_PENDING PATH_VALID->EXPLORING 1/2");
  // The map flips back: the candidate is dropped, not carried forward.
  const auto recovered = mode.observe(GoalMode::PathValid, 3);
  EXPECT_FALSE(recovered.has_pending);
  EXPECT_EQ(recovered.stable, GoalMode::PathValid);
  EXPECT_TRUE(mode.pending_suffix().empty());
}

TEST(GoalModeHysteresis, TwoConsecutiveMapsCommitTheChange)
{
  GoalModeHysteresis mode(2);
  mode.observe(GoalMode::PathValid, 1);
  EXPECT_FALSE(mode.observe(GoalMode::Exploring, 2).committed);
  const auto committed = mode.observe(GoalMode::Exploring, 3);
  EXPECT_TRUE(committed.committed);
  EXPECT_EQ(committed.stable, GoalMode::Exploring);
  EXPECT_FALSE(committed.has_pending);
}

TEST(GoalModeHysteresis, PlannerTicksOnOneMapConfirmNothing)
{
  GoalModeHysteresis mode(2);
  mode.observe(GoalMode::PathValid, 1);
  for (int tick = 0; tick < 20; ++tick) {
    const auto decision = mode.observe(GoalMode::Exploring, 2);
    EXPECT_FALSE(decision.committed) << "tick " << tick;
    EXPECT_EQ(decision.pending_count, 1);
  }
  EXPECT_TRUE(mode.observe(GoalMode::Exploring, 3).committed);
}

TEST(GoalModeHysteresis, ContradictorySampleReplacesThePendingCandidate)
{
  GoalModeHysteresis mode(3);
  mode.observe(GoalMode::PathValid, 1);
  EXPECT_EQ(mode.observe(GoalMode::Exploring, 2).pending_count, 1);
  EXPECT_EQ(mode.observe(GoalMode::Exploring, 3).pending_count, 2);
  const auto switched = mode.observe(GoalMode::SafeApproach, 4);
  EXPECT_EQ(switched.pending, GoalMode::SafeApproach);
  EXPECT_EQ(switched.pending_count, 1);
  EXPECT_EQ(switched.stable, GoalMode::PathValid);
}

TEST(GoalModeHysteresis, SafeApproachTransitionsAreDebouncedToo)
{
  GoalModeHysteresis mode(2);
  mode.observe(GoalMode::Exploring, 1);
  EXPECT_FALSE(mode.observe(GoalMode::SafeApproach, 2).committed);
  EXPECT_TRUE(mode.observe(GoalMode::SafeApproach, 3).committed);
  EXPECT_EQ(mode.stable(), GoalMode::SafeApproach);
  EXPECT_FALSE(mode.observe(GoalMode::PathValid, 4).committed);
  EXPECT_TRUE(mode.observe(GoalMode::PathValid, 5).committed);
}

TEST(GoalModeHysteresis, ReplacingThePathCommitsTheRawModeAtOnce)
{
  GoalModeHysteresis mode(2);
  mode.observe(GoalMode::PathValid, 1);
  EXPECT_TRUE(mode.observe(GoalMode::Exploring, 2).has_pending);
  // No old route survives a replacement, so no old meaning does either.
  mode.commit(GoalMode::Exploring);
  EXPECT_EQ(mode.stable(), GoalMode::Exploring);
  EXPECT_TRUE(mode.pending_suffix().empty());
}

TEST(GoalModeHysteresis, ANewGoalResetsStableAndPendingState)
{
  GoalModeHysteresis mode(2);
  mode.observe(GoalMode::PathValid, 1);
  mode.observe(GoalMode::Exploring, 2);
  mode.reset();
  EXPECT_FALSE(mode.initialized());
  EXPECT_TRUE(mode.pending_suffix().empty());
  const auto fresh = mode.observe(GoalMode::Exploring, 3);
  EXPECT_TRUE(fresh.committed);
  EXPECT_EQ(fresh.stable, GoalMode::Exploring);
}

TEST(GoalModeHysteresis, ExplorationIsNeverTerminalAndModesNameThemselves)
{
  EXPECT_EQ(px4_vio_bridge::goal_mode_from(true, true), GoalMode::PathValid);
  EXPECT_EQ(px4_vio_bridge::goal_mode_from(false, true), GoalMode::SafeApproach);
  EXPECT_EQ(px4_vio_bridge::goal_mode_from(false, false), GoalMode::Exploring);
  EXPECT_TRUE(px4_vio_bridge::goal_mode_terminal(GoalMode::PathValid));
  EXPECT_TRUE(px4_vio_bridge::goal_mode_terminal(GoalMode::SafeApproach));
  EXPECT_FALSE(px4_vio_bridge::goal_mode_terminal(GoalMode::Exploring));
  EXPECT_STREQ(px4_vio_bridge::to_string(GoalMode::PathValid), "PATH_VALID");
  EXPECT_STREQ(px4_vio_bridge::to_string(GoalMode::SafeApproach), "SAFE_APPROACH");
  EXPECT_STREQ(px4_vio_bridge::to_string(GoalMode::Exploring), "EXPLORING");
}

TEST(GoalModeHysteresis, ConfirmationCountOfOneCommitsEveryChange)
{
  GoalModeHysteresis mode(1);
  mode.observe(GoalMode::PathValid, 1);
  EXPECT_TRUE(mode.observe(GoalMode::Exploring, 2).committed);
  EXPECT_EQ(mode.stable(), GoalMode::Exploring);
}

// ---------------------------------------------------------------------------
// The accepted-path decision under a pending mode transition.

namespace
{

px4_vio_bridge::PathReplacementInputs on_corridor_inputs()
{
  px4_vio_bridge::PathReplacementInputs inputs;
  inputs.mode_transition_pending = false;
  inputs.retained_safe = true;
  inputs.goal_changed = false;
  inputs.effective_goal_changed = false;
  // Sitting 2 cm off a route with 2.0 m left to run.
  inputs.projection = px4_vio_bridge::PathProjection{0.02, 2.00};
  inputs.candidate_length = 1.95;
  inputs.retain_tolerance = 0.04;
  inputs.switch_improvement = 0.10;
  return inputs;
}

}  // namespace

TEST(PathReplacement, SettledModeStillReplacesOnAChangedEndpointOrABetterRoute)
{
  auto inputs = on_corridor_inputs();
  EXPECT_FALSE(px4_vio_bridge::decide_path_replacement(inputs).replace);
  // A genuinely advancing exploration frontier must still move the endpoint.
  inputs.effective_goal_changed = true;
  EXPECT_TRUE(px4_vio_bridge::decide_path_replacement(inputs).replace);
  inputs.effective_goal_changed = false;
  // A materially shorter candidate still wins.
  inputs.candidate_length = 1.50;
  EXPECT_TRUE(px4_vio_bridge::decide_path_replacement(inputs).replace);
}

TEST(PathReplacement, PendingTransitionHoldsASafeOnCorridorRoute)
{
  auto inputs = on_corridor_inputs();
  inputs.mode_transition_pending = true;
  // Both of the clauses that fired above belong to the *other* mode now.
  inputs.effective_goal_changed = true;
  inputs.candidate_length = 1.50;
  const auto decision = px4_vio_bridge::decide_path_replacement(inputs);
  EXPECT_FALSE(decision.replace);
  EXPECT_TRUE(decision.transition_hold);
  EXPECT_FALSE(decision.off_corridor);
}

TEST(PathReplacement, AnUnsafeRetainedRouteIsDroppedDespiteTheDebounce)
{
  auto inputs = on_corridor_inputs();
  inputs.mode_transition_pending = true;
  inputs.retained_safe = false;
  // path_valid() failing is what clears the projection at the call site.
  inputs.projection.reset();
  const auto decision = px4_vio_bridge::decide_path_replacement(inputs);
  EXPECT_TRUE(decision.replace);
  EXPECT_FALSE(decision.transition_hold);
}

TEST(PathReplacement, APhysicalDeviationIsNotDebounced)
{
  auto inputs = on_corridor_inputs();
  inputs.mode_transition_pending = true;
  // Well outside path_retain_tolerance: the vehicle really has left the
  // corridor, which is a geometry fact, not a semantic one.
  inputs.projection = px4_vio_bridge::PathProjection{0.30, 2.00};
  const auto decision = px4_vio_bridge::decide_path_replacement(inputs);
  EXPECT_TRUE(decision.off_corridor);
  EXPECT_TRUE(decision.replace);
  EXPECT_FALSE(decision.transition_hold);
}

TEST(PathReplacement, ANewGoalAlwaysReplaces)
{
  auto inputs = on_corridor_inputs();
  inputs.mode_transition_pending = true;
  inputs.goal_changed = true;
  const auto decision = px4_vio_bridge::decide_path_replacement(inputs);
  EXPECT_TRUE(decision.replace);
  EXPECT_FALSE(decision.transition_hold);
}

TEST(PathReplacement, ACorrectionSizedOffsetDoesNotTripRetainTolerance)
{
  // The 2026-08-29 bag saw 4.3-6.0 cm correction steps against a 4 cm
  // path_retain_tolerance. Re-expressing the route removes the offset before
  // it is measured, so what reaches this decision is near zero.
  auto inputs = on_corridor_inputs();
  inputs.projection = px4_vio_bridge::PathProjection{0.001, 2.00};
  EXPECT_FALSE(px4_vio_bridge::decide_path_replacement(inputs).replace);
  // Left un-re-expressed it would have looked like this, and rebuilt the route.
  inputs.projection = px4_vio_bridge::PathProjection{0.055, 2.00};
  EXPECT_TRUE(px4_vio_bridge::decide_path_replacement(inputs).replace);
}
