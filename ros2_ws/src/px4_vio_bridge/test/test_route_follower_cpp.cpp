#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>

#include <gtest/gtest.h>

#include "px4_vio_bridge/grid_clearance.hpp"
#include "px4_vio_bridge/route_follower.hpp"

using px4_vio_bridge::CorrectionReplanGate;
using px4_vio_bridge::PositionRouteFollower;

TEST(RouteFollower, LimitsPositionCarrotSpeedAndAcceleration)
{
  PositionRouteFollower follower(2.0, 1.0, 0.5);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0}, {5.0, 0.0}}, {0.0, 0.0}));
  const auto first = follower.update({0.0, 0.0}, 1.0);
  const auto second = follower.update({0.0, 0.0}, 1.0);
  EXPECT_NEAR(first.commanded_displacement.first, 0.5, 1e-12);
  EXPECT_NEAR(second.commanded_displacement.first, 1.5, 1e-12);
}

TEST(RouteFollower, CrossTrackFaultRequiresFreshPathAndStableRecovery)
{
  PositionRouteFollower follower(0.60, 0.10, 0.30, 0.10, 0.05, 0.30);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0}, {3.0, 0.0}}, {0.0, 0.0}));
  EXPECT_EQ(follower.update({1.0, 0.11}, 0.10).status, "CROSS_TRACK_EXCEEDED");
  EXPECT_EQ(
    follower.update({1.0, 0.04}, 0.10).status,
    "CROSS_TRACK_HOLD_WAITING_FOR_PATH");
  ASSERT_TRUE(follower.set_path({{1.0, 0.04}, {3.0, 0.0}}, {1.0, 0.04}));
  EXPECT_EQ(follower.update({1.0, 0.04}, 0.10).status, "CROSS_TRACK_RECOVERING");
  EXPECT_EQ(follower.update({1.0, 0.04}, 0.10).status, "CROSS_TRACK_RECOVERING");
  EXPECT_TRUE(follower.update({1.0, 0.04}, 0.10).valid);
}

TEST(RouteFollower, ArrivalUsesReleaseHysteresisAcrossReplans)
{
  PositionRouteFollower follower(0.60, 0.10, 0.30, 0.60, 0.05, 1.0, 0.12, 0.20);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0}, {1.0, 0.0}}, {0.0, 0.0}));
  EXPECT_EQ(follower.update({0.90, 0.0}, 0.1).status, "GOAL_REACHED");
  ASSERT_TRUE(follower.set_path({{0.0, 0.0}, {1.02, 0.0}}, {0.90, 0.0}));
  EXPECT_EQ(follower.update({0.90, 0.0}, 0.1).status, "GOAL_REACHED");
  ASSERT_TRUE(follower.set_path({{0.0, 0.0}, {1.25, 0.0}}, {0.90, 0.0}));
  EXPECT_EQ(follower.update({0.90, 0.0}, 0.1).status, "FOLLOWING");
}

TEST(RouteFollower, RejectedClearanceResetsRelativeCommand)
{
  PositionRouteFollower follower(0.60, 10.0, 100.0);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0}, {2.0, 0.0}}, {0.0, 0.0}));
  const auto result = follower.update(
    {0.0, 0.0}, 0.1, std::nullopt,
    [](const auto &) {return false;});
  EXPECT_EQ(result.status, "CLEARANCE_BLOCKED");
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.commanded_displacement.first, 0.0);
  EXPECT_DOUBLE_EQ(result.progress, 0.0);
}

TEST(CorrectionReplanGate, CoalescesAndRequiresFreshPath)
{
  const double radians = 3.14159265358979323846 / 180.0;
  CorrectionReplanGate gate(0.05, 1.5 * radians, 0.10, 0.02, 0.5 * radians, 0.20, 1.0);
  EXPECT_FALSE(gate.observe({0.0, 0.0, 0.0, 0.0}, 0.0));
  int triggers = 0;
  for (int index = 1; index <= 30; ++index) {
    triggers += gate.observe({0.12, 0.0, 0.0, 2.0 * radians}, index * 0.02);
  }
  EXPECT_EQ(triggers, 1);
  EXPECT_TRUE(gate.waiting(0.60));
  gate.path_received(0.61);
  EXPECT_FALSE(gate.waiting(0.70));
}

// ---------------------------------------------------------------------------
// Change 1: deterministic monotonic clearance escape.
//
// The rule is exercised through a synthetic probe where the chord and endpoint
// clearances are stated directly, and through a real occupancy grid where they
// are measured.

namespace
{

using px4_vio_bridge::ClearanceEscapeLimits;
using px4_vio_bridge::ClearanceProbe;
using px4_vio_bridge::Point2;
using px4_vio_bridge::Polyline;

constexpr double kRequired = 0.25;

ClearanceEscapeLimits escape_limits()
{
  ClearanceEscapeLimits limits;
  limits.required_clearance = kRequired;
  limits.minimum_improvement = 0.01;
  return limits;
}

// A probe whose answer depends only on how far the chord's endpoint is along
// +x: `clearance_at` gives the clearance of a point, and a chord's clearance is
// the minimum over samples of it.
ClearanceProbe probe_from(const std::function<double(const Point2 &)> & clearance_at)
{
  return [clearance_at](const Point2 & start, const Point2 & end)
         -> std::optional<double> {
           double best = clearance_at(start);
           for (int index = 1; index <= 64; ++index) {
             const double fraction = static_cast<double>(index) / 64.0;
             best = std::min(
               best,
               clearance_at(
                 {start.first + fraction * (end.first - start.first),
                   start.second + fraction * (end.second - start.second)}));
           }
           return best;
         };
}

px4_vio_bridge::GridMap wall_grid()
{
  // 2.0 x 2.0 m at 0.05 m, with a solid occupied column at x in [1.00, 1.05].
  px4_vio_bridge::GridMap grid;
  grid.width = 40;
  grid.height = 40;
  grid.resolution = 0.05;
  grid.origin_x = 0.0;
  grid.origin_y = 0.0;
  grid.data.assign(grid.width * grid.height, std::int8_t{0});
  for (std::size_t y = 0; y < grid.height; ++y) {
    grid.data[y * grid.width + 20] = 100;
  }
  return grid;
}

ClearanceProbe grid_probe(const px4_vio_bridge::GridMap & grid)
{
  return [&grid](const Point2 & start, const Point2 & end) {
           return px4_vio_bridge::segment_minimum_clearance(grid, start, end);
         };
}

}  // namespace

TEST(ClearanceEscape, FullClearanceKeepsTheNormalRuleExactly)
{
  const auto grid = wall_grid();
  const auto probe = grid_probe(grid);
  const Point2 pose{0.60, 1.00};   // 0.40 m from the wall: outside the envelope
  const auto start = probe(pose, pose);
  ASSERT_TRUE(start.has_value());
  for (const Point2 target : {Point2{0.30, 1.00}, Point2{0.74, 1.00}, Point2{0.90, 1.00}}) {
    EXPECT_EQ(
      px4_vio_bridge::command_chord_admissible(
        probe, pose, target, escape_limits(), *start),
      px4_vio_bridge::segment_has_clearance(grid, pose, target, kRequired))
      << "target=" << target.first;
  }
}

TEST(ClearanceEscape, SubClearancePoseMayCommandAStrictlyImprovingChord)
{
  const auto grid = wall_grid();
  const auto probe = grid_probe(grid);
  const Point2 pose{0.76, 1.00};   // 0.24 m from the wall, inside 0.25 m
  const auto start = probe(pose, pose);
  ASSERT_TRUE(start.has_value());
  EXPECT_NEAR(*start, 0.24, 1e-12);

  // Directly away: accepted, and no point of the chord is ever closer than the
  // pose already is.
  const Point2 away{0.40, 1.00};
  double end_clearance = 0.0;
  EXPECT_TRUE(
    px4_vio_bridge::command_chord_admissible(
      probe, pose, away, escape_limits(), *start,
      px4_vio_bridge::ChordRole::Target, &end_clearance));
  EXPECT_NEAR(end_clearance, 0.60, 1e-12);
  const auto chord = px4_vio_bridge::segment_minimum_clearance(grid, pose, away);
  ASSERT_TRUE(chord.has_value());
  EXPECT_GE(*chord + 1e-12, *start);

  // Towards the obstacle: rejected.
  EXPECT_FALSE(
    px4_vio_bridge::command_chord_admissible(
      probe, pose, {0.90, 1.00}, escape_limits(), *start));
}

TEST(ClearanceEscape, RejectsAChordThatDipsCloserBeforeGettingSafer)
{
  // Clearance falls to 0.18 m midway and only then rises well past the
  // requirement. The endpoint alone is safe; the chord is not.
  const auto probe = probe_from([](const Point2 & point) {
      const double x = point.first;
      if (x <= 0.5) {
        return 0.24 - 0.12 * x;          // 0.24 down to 0.18
      }
      return 0.18 + 0.84 * (x - 0.5);    // 0.18 up to 0.60
    });
  EXPECT_FALSE(
    px4_vio_bridge::command_chord_admissible(
      probe, {0.0, 0.0}, {1.0, 0.0}, escape_limits(), 0.24));
  // Stopping before the dip is not an escape either: no material improvement.
  EXPECT_FALSE(
    px4_vio_bridge::command_chord_admissible(
      probe, {0.0, 0.0}, {0.05, 0.0}, escape_limits(), 0.24));
}

TEST(ClearanceEscape, RejectsLateralMotionAtConstantClearance)
{
  const auto probe = probe_from([](const Point2 &) {return 0.24;});
  EXPECT_FALSE(
    px4_vio_bridge::command_chord_admissible(
      probe, {0.0, 0.0}, {1.0, 0.0}, escape_limits(), 0.24));
  // One centimetre of gain is exactly the configured minimum, and passes.
  const auto improving = probe_from([](const Point2 & point) {
      return 0.24 + 0.01 * point.first;
    });
  EXPECT_TRUE(
    px4_vio_bridge::command_chord_admissible(
      probe, {0.0, 0.0}, {1.0, 0.0}, escape_limits(), 0.24) == false);
  EXPECT_TRUE(
    px4_vio_bridge::command_chord_admissible(
      improving, {0.0, 0.0}, {1.0, 0.0}, escape_limits(), 0.24));
}

TEST(ClearanceEscape, UnknownSpaceIsNeverAnEscape)
{
  const ClearanceProbe blind = [](const Point2 &, const Point2 &) {
      return std::optional<double>{};
    };
  EXPECT_FALSE(
    px4_vio_bridge::command_chord_admissible(
      blind, {0.0, 0.0}, {1.0, 0.0}, escape_limits(), 0.24));
  const Polyline path({{0.0, 0.0}, {1.0, 0.0}});
  EXPECT_FALSE(
    px4_vio_bridge::select_safe_lookahead(
      path, {0.0, 0.0}, blind, escape_limits(), 0.25, 0.03, 0.05).has_value());
}

TEST(ClearanceEscape, SelectsTheFarthestEscapeAndReportsIt)
{
  const auto grid = wall_grid();
  const auto probe = grid_probe(grid);
  // A route leading away from the wall, the vehicle sitting 0.24 m from it.
  const Polyline path({{0.76, 1.00}, {0.20, 1.00}});
  const auto selection = px4_vio_bridge::select_safe_lookahead(
    path, {0.76, 1.00}, probe, escape_limits(), 0.25, 0.03, 0.05);
  ASSERT_TRUE(selection.has_value());
  EXPECT_TRUE(selection->escaping);
  EXPECT_NEAR(selection->start_clearance, 0.24, 1e-12);
  EXPECT_NEAR(selection->lookahead, 0.25, 1e-12);
  EXPECT_NEAR(selection->end_clearance, 0.49, 1e-12);

  // Once clear of the envelope the selection is an ordinary one again.
  const Polyline clear_path({{0.60, 1.00}, {0.20, 1.00}});
  const auto normal = px4_vio_bridge::select_safe_lookahead(
    clear_path, {0.60, 1.00}, probe, escape_limits(), 0.25, 0.03, 0.05);
  ASSERT_TRUE(normal.has_value());
  EXPECT_FALSE(normal->escaping);
}

TEST(ClearanceEscape, NoEscapeExistsWhenTheRouteLeadsIntoTheObstacle)
{
  const auto grid = wall_grid();
  const auto probe = grid_probe(grid);
  const Polyline path({{0.76, 1.00}, {0.95, 1.00}});
  EXPECT_FALSE(
    px4_vio_bridge::select_safe_lookahead(
      path, {0.76, 1.00}, probe, escape_limits(), 0.25, 0.03, 0.05).has_value());
}

TEST(ClearanceEscape, AccelerationLimitedCarrotObeysTheSameRule)
{
  const auto grid = wall_grid();
  const auto probe = grid_probe(grid);
  px4_vio_bridge::PositionRouteFollower follower(0.25, 0.10, 0.30, 0.60, 0.05, 1.0, 0.12, 0.20);
  ASSERT_TRUE(follower.set_path({{0.76, 1.00}, {0.20, 1.00}}, {0.76, 1.00}));
  const Point2 pose{0.76, 1.00};
  const auto start = probe(pose, pose);
  ASSERT_TRUE(start.has_value());
  follower.set_escape(true);

  const auto validator = [&](const Point2 & carrot) {
      return px4_vio_bridge::command_chord_admissible(
        probe, pose, carrot, escape_limits(), *start,
        px4_vio_bridge::ChordRole::IntermediateCarrot);
    };
  // The first acceleration-limited step is millimetres. It is admissible
  // because it never worsens clearance -- the centimetre of gain is demanded of
  // the target, which select_safe_lookahead already checked.
  const auto result = follower.update(pose, 0.10, 0.25, validator);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.status, "FOLLOWING");
  EXPECT_LT(result.commanded_carrot.first, pose.first);
  EXPECT_LT(pose.first - result.commanded_carrot.first, 0.01) << "a partial step";
  // As a Target the same millimetre step is refused: it improves nothing.
  EXPECT_FALSE(
    px4_vio_bridge::command_chord_admissible(
      probe, pose, result.commanded_carrot, escape_limits(), *start,
      px4_vio_bridge::ChordRole::Target));

  // A carrot that worsens clearance is refused in either role, and collapses
  // the command instead of being issued.
  px4_vio_bridge::PositionRouteFollower blocked(0.25, 0.10, 0.30, 0.60, 0.05, 1.0, 0.12, 0.20);
  ASSERT_TRUE(blocked.set_path({{0.76, 1.00}, {0.95, 1.00}}, pose));
  const auto refused = blocked.update(pose, 0.10, 0.25, validator);
  EXPECT_FALSE(refused.valid);
  EXPECT_EQ(refused.status, "CLEARANCE_BLOCKED");
  EXPECT_DOUBLE_EQ(refused.commanded_displacement.first, 0.0);
}

TEST(ClearanceEscape, ThresholdChatterDoesNotStarveTheCommand)
{
  // Regression for flight 20260829T084036Z. The takeoff pose measured
  // 0.2495-0.2505 m from an obstacle against a required 0.250 m, so `escaping`
  // re-crossed the threshold every tick. An unconditional edge-triggered wipe
  // zeroed the accumulating displacement roughly ten times a second: the
  // follower reported progress=0.00m for six seconds, the aircraft sagged
  // sideways, cross-track reached max_cross_track and the adapter landed it.
  const auto grid = wall_grid();
  const auto probe = grid_probe(grid);
  px4_vio_bridge::PositionRouteFollower follower(0.25, 0.20, 0.30, 0.60, 0.05, 1.0, 0.12, 0.20);
  ASSERT_TRUE(follower.set_path({{0.76, 1.00}, {0.20, 1.00}}, {0.76, 1.00}));
  const Point2 pose{0.76, 1.00};
  const auto start = probe(pose, pose);
  ASSERT_TRUE(start.has_value());

  double previous = 0.0;
  for (int tick = 0; tick < 20; ++tick) {
    // The measurement alternates across the threshold, as the flown one did.
    const bool escaping = tick % 2 == 0;
    const Point2 stale{
      pose.first + follower.commanded_displacement().first,
      pose.second + follower.commanded_displacement().second};
    follower.set_escape(
      escaping,
      px4_vio_bridge::command_chord_admissible(
        probe, pose, stale, escape_limits(), *start,
        px4_vio_bridge::ChordRole::IntermediateCarrot));
    const auto result = follower.update(
      pose, 0.10, 0.25, [&](const Point2 & carrot) {
        return px4_vio_bridge::command_chord_admissible(
          probe, pose, carrot, escape_limits(), *start,
          px4_vio_bridge::ChordRole::IntermediateCarrot);
      });
    ASSERT_TRUE(result.valid) << "tick " << tick;
    const double magnitude = std::hypot(
      result.commanded_displacement.first, result.commanded_displacement.second);
    EXPECT_GE(magnitude + 1e-12, previous) << "command collapsed on tick " << tick;
    previous = magnitude;
  }
  // Two seconds of chatter must still have built a real command, not 3 mm.
  EXPECT_GT(previous, 0.20);
}

TEST(ClearanceEscape, EnteringEscapeDropsTheStaleCommandButKeepsProgress)
{
  px4_vio_bridge::PositionRouteFollower follower(0.60, 0.10, 0.30, 0.60, 0.05, 1.0, 0.12, 0.20);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0}, {5.0, 0.0}}, {0.0, 0.0}));
  follower.update({0.0, 0.0}, 1.0);
  follower.update({0.5, 0.0}, 1.0);
  const auto before = follower.progress();
  EXPECT_GT(std::hypot(
      follower.commanded_displacement().first,
      follower.commanded_displacement().second), 0.0);

  // stale_command_admissible=false: the stored command no longer passes.
  follower.set_escape(true, false);
  EXPECT_TRUE(follower.escaping());
  EXPECT_DOUBLE_EQ(follower.commanded_displacement().first, 0.0);
  EXPECT_DOUBLE_EQ(follower.commanded_displacement().second, 0.0);
  EXPECT_DOUBLE_EQ(follower.progress(), before);

  // Staying in the escape does not keep re-clearing the command.
  follower.update({0.6, 0.0}, 1.0);
  follower.set_escape(true, false);
  EXPECT_GT(std::hypot(
      follower.commanded_displacement().first,
      follower.commanded_displacement().second), 0.0);
  // And leaving it preserves cumulative route progress.
  const auto during = follower.progress();
  follower.set_escape(false, false);
  EXPECT_FALSE(follower.escaping());
  EXPECT_DOUBLE_EQ(follower.progress(), during);
  EXPECT_EQ(follower.update({0.7, 0.0}, 1.0).status, "FOLLOWING");
}

// ---------------------------------------------------------------------------
// Change 2: correction-aware accepted paths.

namespace
{

using px4_vio_bridge::Correction4;

// The map-frame view of a physical route under a given correction: exactly
// what the planner would publish once its accepted path was re-expressed.
std::vector<Point2> in_map(
  const std::vector<Point2> & vio_points, const Correction4 & correction)
{
  std::vector<Point2> points;
  for (const auto & point : vio_points) {
    points.push_back(px4_vio_bridge::vio_point_to_map(point, correction));
  }
  return points;
}

}  // namespace

TEST(CorrectionGeometry, TransformsRoundTripAndComposeExactly)
{
  const Correction4 a{0.30, -0.20, 0.05, 0.4};
  const Correction4 b{0.35, -0.14, 0.05, 0.4};      // translation only
  const Correction4 c{0.30, -0.20, 0.05, 0.9};      // yaw only
  const Correction4 wrap{0.10, 0.20, 0.0, 3.10};
  const Correction4 wrapped{0.10, 0.20, 0.0, -3.10};

  const Point2 point{1.25, -0.75};
  for (const auto & correction : {a, b, c, wrap, wrapped}) {
    const auto round_trip = px4_vio_bridge::vio_point_to_map(
      px4_vio_bridge::map_point_to_vio(point, correction), correction);
    EXPECT_NEAR(round_trip.first, point.first, 1e-12);
    EXPECT_NEAR(round_trip.second, point.second, 1e-12);
  }

  // Translation only leaves a vector untouched and shifts a point by t.
  const auto shifted = px4_vio_bridge::reexpress_point(point, a, b);
  EXPECT_NEAR(shifted.first, point.first + 0.05, 1e-12);
  EXPECT_NEAR(shifted.second, point.second + 0.06, 1e-12);
  const auto same_vector = px4_vio_bridge::reexpress_vector({1.0, 0.0}, a, b);
  EXPECT_NEAR(same_vector.first, 1.0, 1e-12);
  EXPECT_NEAR(same_vector.second, 0.0, 1e-12);

  // Yaw only rotates a vector by the yaw difference.
  const auto rotated = px4_vio_bridge::reexpress_vector({1.0, 0.0}, a, c);
  EXPECT_NEAR(rotated.first, std::cos(0.5), 1e-12);
  EXPECT_NEAR(rotated.second, std::sin(0.5), 1e-12);

  // Re-expression is invertible, and wraps across +/-pi without a jump.
  const auto there = px4_vio_bridge::reexpress_point(point, a, c);
  const auto back = px4_vio_bridge::reexpress_point(there, c, a);
  EXPECT_NEAR(back.first, point.first, 1e-12);
  EXPECT_NEAR(back.second, point.second, 1e-12);
  // Crossing +/-pi rotates the short way (2*pi - 6.20 rad), not by 6.20.
  const double short_way = 2.0 * M_PI - 6.20;
  const auto across = px4_vio_bridge::reexpress_vector({1.0, 0.0}, wrap, wrapped);
  EXPECT_NEAR(std::atan2(across.second, across.first), short_way, 1e-12);
  const auto back_across = px4_vio_bridge::reexpress_vector(across, wrapped, wrap);
  EXPECT_NEAR(back_across.first, 1.0, 1e-12);
  EXPECT_NEAR(back_across.second, 0.0, 1e-12);
}

TEST(CorrectionAwareFollower, TranslationCorrectionIsNotCrossTrackOrANewPath)
{
  const Correction4 before{0.10, 0.00, 0.0, 0.0};
  const Correction4 after{0.15, 0.04, 0.0, 0.0};     // a 6.4 cm loop closure
  const std::vector<Point2> route_vio{{0.0, 0.0}, {3.0, 0.0}};

  PositionRouteFollower follower(0.60, 0.10, 0.30, 0.10, 0.03, 1.0, 0.12, 0.20);
  ASSERT_TRUE(follower.set_path(in_map(route_vio, before), {0.10, 0.00}, before));
  follower.update({0.60, 0.00}, 0.10, std::nullopt, {}, before);
  const auto generation = follower.generation();
  const auto progress = follower.progress();
  const auto path_progress = follower.path_progress();
  const auto vio_command = follower.vio_displacement();

  // Pose and route move together. Without re-expression this is a 6.4 cm
  // cross-track step, which max_cross_track 0.10 would be one bump from
  // faulting on.
  const Point2 moved_pose = px4_vio_bridge::reexpress_point({0.60, 0.00}, before, after);
  const auto result = follower.update(moved_pose, 0.10, std::nullopt, {}, after);
  EXPECT_NEAR(result.cross_track, 0.0, 1e-9);
  EXPECT_EQ(result.generation, generation);
  EXPECT_NEAR(result.progress, progress, 1e-9);
  EXPECT_NEAR(result.path_progress, path_progress, 1e-9);
  EXPECT_TRUE(result.valid);
  // A pure translation leaves the relative command untouched.
  EXPECT_NEAR(result.vio_displacement.first, vio_command.first + 0.0, 0.05);

  // Re-publishing the same physical route under the new correction is a
  // coordinate change, not a new semantic generation.
  EXPECT_FALSE(follower.set_path(in_map(route_vio, after), moved_pose, after));
  EXPECT_EQ(follower.generation(), generation);
  EXPECT_NEAR(follower.progress(), progress, 1e-9);
}

TEST(CorrectionAwareFollower, YawCorrectionRotatesRoutePoseAndCommandTogether)
{
  const Correction4 before{0.00, 0.00, 0.0, 0.0};
  const Correction4 after{0.00, 0.00, 0.0, 0.20};
  const std::vector<Point2> route_vio{{0.0, 0.0}, {3.0, 0.0}};

  PositionRouteFollower follower(0.60, 0.10, 0.30, 0.10, 0.03, 1.0, 0.12, 0.20);
  ASSERT_TRUE(follower.set_path(in_map(route_vio, before), {0.0, 0.0}, before));
  follower.update({0.50, 0.00}, 0.10, std::nullopt, {}, before);
  const auto before_command = follower.commanded_displacement();
  const auto before_vio = follower.vio_displacement();
  const auto generation = follower.generation();

  const Point2 moved_pose = px4_vio_bridge::reexpress_point({0.50, 0.00}, before, after);
  const auto result = follower.update(moved_pose, 0.10, std::nullopt, {}, after);
  EXPECT_NEAR(result.cross_track, 0.0, 1e-9);
  EXPECT_EQ(result.generation, generation);
  // The canonical command is untouched by the frame change; only its map-frame
  // rendering rotates.
  EXPECT_NEAR(
    std::hypot(
      result.vio_displacement.first - before_vio.first,
      result.vio_displacement.second - before_vio.second),
    0.0, 0.02);
  const auto expected = px4_vio_bridge::reexpress_vector(before_command, before, after);
  EXPECT_NEAR(
    std::atan2(result.commanded_displacement.second, result.commanded_displacement.first) -
    std::atan2(expected.second, expected.first), 0.0, 1e-6);
}

TEST(CorrectionAwareFollower, RealDeviationAfterACorrectionStillFaults)
{
  const Correction4 before{0.0, 0.0, 0.0, 0.0};
  const Correction4 after{0.05, 0.0, 0.0, 0.0};
  const std::vector<Point2> route_vio{{0.0, 0.0}, {3.0, 0.0}};
  PositionRouteFollower follower(0.60, 0.10, 0.30, 0.10, 0.03, 1.0, 0.12, 0.20);
  ASSERT_TRUE(follower.set_path(in_map(route_vio, before), {0.0, 0.0}, before));
  follower.update({0.50, 0.00}, 0.10, std::nullopt, {}, before);
  // Correction plus a genuine 0.20 m lateral excursion: the excursion survives
  // re-expression and still faults.
  const Point2 pose = px4_vio_bridge::reexpress_point({0.50, 0.20}, before, after);
  EXPECT_EQ(
    follower.update(pose, 0.10, std::nullopt, {}, after).status, "CROSS_TRACK_EXCEEDED");
}

// ---------------------------------------------------------------------------
// Correction episodes: settling, generation pairing, no blind cooldown.

TEST(CorrectionReplanGate, HoldIsNotReleasedByAPreCorrectionMapPath)
{
  CorrectionReplanGate gate(0.05, 0.03, 0.35, 0.03, 0.01, 0.40, 0.20);
  gate.map_received(7, 0.0);
  gate.observe({0.0, 0.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(gate.observe({0.30, 0.0, 0.0, 0.0}, 1.0));
  ASSERT_TRUE(gate.pending());

  // A path planned from the pre-correction grid arrives. It must not settle
  // the episode however long it is quiet afterwards.
  gate.path_map_generation(7, 1.05);
  gate.path_received(1.05);
  EXPECT_TRUE(gate.waiting(2.0));

  // The first grid published after the correction, then a path planned from it.
  gate.map_received(8, 2.1);
  EXPECT_EQ(gate.required_map_generation(), std::optional<std::int64_t>{8});
  gate.path_map_generation(8, 2.2);
  gate.path_received(2.2);
  EXPECT_FALSE(gate.waiting(2.6));
  EXPECT_EQ(gate.epoch(), 1);
}

TEST(CorrectionReplanGate, MaterialStepsOneSecondApartAreNeverHidden)
{
  CorrectionReplanGate gate(0.05, 0.03, 0.05, 0.03, 0.01, 0.40, 0.20);
  gate.observe({0.0, 0.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(gate.observe({0.30, 0.0, 0.0, 0.0}, 1.0));
  gate.map_received(1, 1.1);
  gate.path_map_generation(1, 1.2);
  gate.path_received(1.2);
  ASSERT_FALSE(gate.waiting(1.5));          // settled
  ASSERT_EQ(gate.epoch(), 1);

  // Another material step one second later. The old eight-second cooldown
  // swallowed exactly this; a sub-second re-arm guard cannot.
  gate.observe({0.30, 0.0, 0.0, 0.0}, 2.0);
  EXPECT_TRUE(gate.observe({0.60, 0.0, 0.0, 0.0}, 2.5));
  EXPECT_TRUE(gate.pending());
  EXPECT_EQ(gate.epoch(), 2);
}

TEST(CorrectionReplanGate, FurtherMaterialStepsExtendOneEpisode)
{
  CorrectionReplanGate gate(0.05, 0.03, 0.05, 0.03, 0.01, 0.40, 0.20);
  gate.observe({0.0, 0.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(gate.observe({0.30, 0.0, 0.0, 0.0}, 1.0));
  gate.map_received(1, 1.05);
  gate.path_map_generation(1, 1.1);
  gate.path_received(1.1);
  // A further material step restarts the quiet timer and voids the receipts.
  gate.observe({0.70, 0.0, 0.0, 0.0}, 1.2);
  EXPECT_TRUE(gate.waiting(1.7));
  EXPECT_EQ(gate.epoch(), 1) << "one episode, not two";
  EXPECT_FALSE(gate.required_map_generation().has_value());
  gate.map_received(2, 1.8);
  gate.path_map_generation(2, 1.9);
  gate.path_received(1.9);
  EXPECT_FALSE(gate.waiting(2.3));
}

TEST(CorrectionReplanGate, SubThresholdJitterNeverOpensAnEpisode)
{
  CorrectionReplanGate gate(0.05, 0.03, 0.35, 0.03, 0.01, 0.40, 0.20);
  gate.observe({0.0, 0.0, 0.0, 0.0}, 0.0);
  for (int index = 1; index <= 40; ++index) {
    const double sign = index % 2 == 0 ? 1.0 : -1.0;
    EXPECT_FALSE(gate.observe({0.01 * sign, 0.0, 0.0, 0.0}, 0.1 * index));
  }
  EXPECT_FALSE(gate.pending());
  EXPECT_EQ(gate.epoch(), 0);
}

TEST(CorrectionReplanGate, WithoutGenerationTelemetryTheTimeRuleStillReleases)
{
  // A planner that publishes no generation topics must not deadlock the
  // follower: the receipt-time rule alone still settles the episode.
  CorrectionReplanGate gate(0.05, 0.03, 0.35, 0.03, 0.01, 0.40, 0.20);
  gate.observe({0.0, 0.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(gate.observe({0.30, 0.0, 0.0, 0.0}, 1.0));
  gate.path_received(1.2);
  EXPECT_FALSE(gate.waiting(1.5));
}

TEST(CorrectionReplanGate, TheFirstPathIsNeverDeferred)
{
  // Regression for the 20260829T102157Z / 102221Z aborts. A correction episode
  // that opens before the follower owns a route must not defer the path that
  // would give it one: the follower would report WAITING_FOR_PATH for ever, and
  // because waiting() is what settles an episode, the episode could never
  // clear either. The planner published PATH_VALID for eleven seconds while
  // every path was deferred, and the adapter aborted on engage timeout.
  EXPECT_FALSE(px4_vio_bridge::defer_path_for_correction(true, false));
  EXPECT_FALSE(px4_vio_bridge::defer_path_for_correction(false, false));
  EXPECT_FALSE(px4_vio_bridge::defer_path_for_correction(false, true));
  // With a route already installed there is something to protect, so a path
  // that may have been planned from the pre-correction grid does wait.
  EXPECT_TRUE(px4_vio_bridge::defer_path_for_correction(true, true));
}

TEST(CorrectionReplanGate, SettlesOnReceiptsAndTimeWithoutAnyRoute)
{
  // The other half of that latch: the episode state machine must be advanced
  // from the tick unconditionally. It depends on receipts and elapsed time
  // alone, so a follower that owns no route -- and therefore never reaches the
  // command path -- must still be able to settle an episode.
  CorrectionReplanGate gate(0.05, 0.03, 0.35, 0.03, 0.01, 0.40, 0.20);
  gate.observe({0.0, 0.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(gate.observe({0.30, 0.0, 0.0, 0.0}, 1.0));
  gate.map_received(51, 1.1);
  gate.path_map_generation(51, 1.2);
  gate.path_received(1.2);
  EXPECT_FALSE(gate.waiting(1.7)) << "settling must not depend on owning a route";
}

TEST(CorrectionReplanGate, FilterConvergenceIsNotAMaterialChange)
{
  // Regression for the 20260829T103521Z stall. The native correction is a
  // staircase: constant, then one loop-closure step. Replayed here at the flown
  // 13 Hz with the flown 333 mm step, with a fresh map and a path planned from
  // it offered on every tick -- so the ONLY thing that can hold the episode
  // open is the "still moving" test.
  //
  // Asking the *filtered* value whether the correction is still moving answers
  // "yes" for as long as the filter takes to converge, turning one step into a
  // chain of self-inflicted material changes that each restart the quiet timer.
  // In flight that made 5 real loop closures look like 20 events with a median
  // gap of 0.14 s: episodes ran ~2 s instead of ~0.4 s, path acceptance stalled
  // and the route aged until cross-track faulted.
  CorrectionReplanGate gate(0.05, 0.03, 0.35, 0.03, 0.013, 0.40, 0.20);
  const double period = 1.0 / 13.0;
  double now = 0.0;
  for (int i = 0; i < 13; ++i, now += period) {
    ASSERT_FALSE(gate.observe({0.0, 0.0, 0.0, 0.0}, now));
  }

  const double step_time = now;
  const Correction4 stepped{0.333, 0.0, 0.0, 0.0};
  std::int64_t generation = 100;
  double settled = -1.0;
  for (int i = 0; i < 60; ++i, now += period) {
    gate.observe(stepped, now);
    // A new grid and a path planned from it, every tick: receipts are never
    // what is missing here.
    ++generation;
    gate.map_received(generation, now);
    gate.path_map_generation(generation, now);
    gate.path_received(now);
    if (!gate.waiting(now) && settled < 0.0) {
      settled = now - step_time;
    }
  }
  ASSERT_GE(settled, 0.0) << "the episode never settled";
  EXPECT_EQ(gate.epoch(), 1);
  // quiet_time is 0.40 s. Settling must follow the correction going still, not
  // the filter catching up: the flown behaviour took about 2 s.
  EXPECT_LT(settled, 0.40 + 4.0 * period)
    << "episode stayed open " << settled << "s after a single step";

  // A genuine second step still opens a new episode.
  now += 0.5;
  for (int i = 0; i < 20; ++i, now += period) {
    gate.observe({0.666, 0.0, 0.0, 0.0}, now);
  }
  EXPECT_EQ(gate.epoch(), 2);
}
