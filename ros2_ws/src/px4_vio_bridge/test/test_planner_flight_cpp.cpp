#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "px4_vio_bridge/command_limiter.hpp"
#include "px4_vio_bridge/path_geometry.hpp"

using px4_vio_bridge::HorizontalCommandLimiter;
using px4_vio_bridge::PathCommandLimiter;
using px4_vio_bridge::Point2;
using px4_vio_bridge::Polyline;

TEST(Polyline, DropsConsecutiveDuplicatesAndMeasuresArcLength)
{
  const Polyline path({{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  EXPECT_EQ(path.points().size(), 3u);
  EXPECT_DOUBLE_EQ(path.length(), 2.0);
  EXPECT_NEAR(path.point_at(1.5).first, 1.0, 1e-12);
  EXPECT_NEAR(path.point_at(1.5).second, 0.5, 1e-12);
}

TEST(Polyline, ProjectionTieBreaksToTheFurtherArcLength)
{
  // Python compares the tuple (cross_track, -along), so an equally-distant
  // candidate wins only when its arc length is strictly greater. On a path that
  // doubles back, the return leg is the later projection at the same distance.
  // Verified against path_follower.Polyline: along=1.5, segment=1.
  const Polyline doubling_back({{0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}});
  const auto later = doubling_back.project({0.5, 1.0});
  EXPECT_NEAR(later.cross_track, 1.0, 1e-12);
  EXPECT_NEAR(later.along, 1.5, 1e-12);
  EXPECT_EQ(later.segment, 1u);

  // At a shared vertex both candidates have the same arc length, so the strict
  // comparison keeps the earlier segment. Python gives along=1.0, segment=0.
  const Polyline shared_vertex({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  const auto tied = shared_vertex.project({1.0, 1.0});
  EXPECT_NEAR(tied.cross_track, 1.0, 1e-12);
  EXPECT_NEAR(tied.along, 1.0, 1e-12);
  EXPECT_EQ(tied.segment, 0u);
}

TEST(PathFingerprint, RoundsToFourDecimals)
{
  const auto first = px4_vio_bridge::path_fingerprint({{1.00001, 2.00001}});
  const auto second = px4_vio_bridge::path_fingerprint({{1.00002, 2.00002}});
  EXPECT_EQ(first, second);
  const auto third = px4_vio_bridge::path_fingerprint({{1.0002, 2.0}});
  EXPECT_NE(first, third);
}

TEST(Transforms, VioEnuToNedSwapsAxes)
{
  const auto ned = px4_vio_bridge::vio_enu_displacement_to_ned({1.0, 2.0});
  EXPECT_DOUBLE_EQ(ned.first, 2.0);
  EXPECT_DOUBLE_EQ(ned.second, 1.0);
}

TEST(Transforms, MapAndVioRotationsAreInverses)
{
  const Point2 original{0.4, -0.3};
  const double yaw = 0.7;
  const auto mapped = px4_vio_bridge::vio_displacement_to_map(original, yaw);
  const auto back = px4_vio_bridge::map_displacement_to_vio(mapped, yaw);
  EXPECT_NEAR(back.first, original.first, 1e-12);
  EXPECT_NEAR(back.second, original.second, 1e-12);
}

TEST(CorrectionGate, RejectsTranslationAndYawBeyondLimits)
{
  EXPECT_FALSE(
    px4_vio_bridge::correction_rejection_reason({0.1, 0.0, 0.0, 0.0}, 0.25, 0.1)
    .has_value());
  EXPECT_TRUE(
    px4_vio_bridge::correction_rejection_reason({0.3, 0.0, 0.0, 0.0}, 0.25, 0.1)
    .has_value());
  EXPECT_TRUE(
    px4_vio_bridge::correction_rejection_reason({0.0, 0.0, 0.0, 0.2}, 0.25, 0.1)
    .has_value());
  EXPECT_TRUE(
    px4_vio_bridge::correction_rejection_reason(
      {std::nan(""), 0.0, 0.0, 0.0}, 0.25, 0.1).has_value());
}

TEST(HorizontalCommandLimiter, RespectsAccelerationAndSpeedLimits)
{
  HorizontalCommandLimiter limiter(0.20, 0.30);
  limiter.reset({0.0, 0.0});
  const double dt = 0.05;
  for (int step = 0; step < 200; ++step) {
    limiter.update({10.0, 0.0}, dt);
    EXPECT_LE(std::hypot(limiter.velocity().first, limiter.velocity().second),
      0.20 + 1e-9);
  }
  EXPECT_NEAR(limiter.velocity().first, 0.20, 1e-9);
}

TEST(HorizontalCommandLimiter, AdoptRejectsVelocityAboveMaxSpeed)
{
  HorizontalCommandLimiter limiter(0.20, 0.30);
  EXPECT_THROW(limiter.adopt({0.0, 0.0}, {0.5, 0.0}), std::invalid_argument);
}

TEST(PathCommandLimiter, StopsAtEveryBendWithoutBlending)
{
  PathCommandLimiter limiter(0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.01, false, 0.05);
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  ASSERT_TRUE(limiter.set_path(path, {0.0, 0.0}));
  for (int step = 0; step < 400; ++step) {
    limiter.update({1.0, 1.0}, 0.05, true, Point2{0.0, 0.0});
  }
  // The vehicle reference never reaches the vertex, so the command waits there.
  ASSERT_TRUE(limiter.waiting_vertex().has_value());
  EXPECT_NEAR(*limiter.waiting_vertex(), 1.0, 1e-9);
  EXPECT_NEAR(limiter.position()->first, 1.0, 1e-9);
  EXPECT_NEAR(limiter.position()->second, 0.0, 1e-9);
}

TEST(PathCommandLimiter, CornerBlendingCarriesSpeedThroughAShallowBend)
{
  PathCommandLimiter limiter(0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.01, true, 0.05);
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.2}};
  ASSERT_TRUE(limiter.set_path(path, {0.0, 0.0}));
  for (int step = 0; step < 400; ++step) {
    limiter.update({2.0, 0.2}, 0.05, true, Point2{0.0, 0.0});
  }
  // No stop is latched at the bend; the command runs to the path end.
  EXPECT_FALSE(limiter.waiting_vertex().has_value());
  EXPECT_NEAR(limiter.position()->first, 2.0, 1e-6);
  EXPECT_NEAR(limiter.position()->second, 0.2, 1e-6);
}

TEST(PathCommandLimiter, RejectsRouteEntryBeyondTheEntryTolerance)
{
  PathCommandLimiter limiter(0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.01, false, 0.05);
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_THROW(limiter.set_path(path, {0.0, 5.0}), std::invalid_argument);
}

TEST(PathCommandLimiter, SharedSuffixKeepsTheCommandPointExactly)
{
  PathCommandLimiter limiter(0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.01, false, 0.05);
  ASSERT_TRUE(limiter.set_path({{0.0, 0.0}, {2.0, 0.0}}, {0.0, 0.0}));
  for (int step = 0; step < 10; ++step) {
    limiter.update({2.0, 0.0}, 0.05, true, Point2{0.0, 0.0});
  }
  const auto before = *limiter.position();
  // Replanning rewrites only the head; the tail the command sits on survives.
  ASSERT_TRUE(limiter.set_path({{-1.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}}, {0.0, 0.0}));
  EXPECT_NEAR(limiter.position()->first, before.first, 1e-9);
  EXPECT_NEAR(limiter.position()->second, before.second, 1e-9);
}

TEST(PathCommandLimiter, SharedSuffixKeepsSpeedAcrossSmallMapCorrectionShift)
{
  // Tunnel flight 20260829T114405Z moved retained map-frame paths by
  // 1.1-2.4 cm while the correction filter converged. With the operational
  // 3 cm suffix tolerance that is a coordinate update, not a reason to stop
  // and start a connector rejoin.
  PathCommandLimiter limiter(0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.03, true, 0.05);
  ASSERT_TRUE(limiter.set_path({{0.0, 0.0}, {2.0, 0.0}}, {0.0, 0.0}));
  for (int step = 0; step < 10; ++step) {
    limiter.update({2.0, 0.0}, 0.05, true, Point2{0.0, 0.0});
  }
  const auto before = limiter.snapshot();
  ASSERT_GT(before.speed, 0.0);

  ASSERT_TRUE(limiter.set_path({{0.0, 0.02}, {2.0, 0.02}}, {0.0, 0.02}));
  const auto after = limiter.snapshot();
  EXPECT_DOUBLE_EQ(after.speed, before.speed);
  EXPECT_EQ(after.velocity, before.velocity);
  EXPECT_FALSE(after.join_target.has_value());
  ASSERT_TRUE(after.position.has_value());
  EXPECT_NEAR(after.position->first, before.position->first, 1e-12);
  EXPECT_NEAR(after.position->second, before.position->second + 0.02, 1e-12);
}

TEST(PathCommandLimiter, SnapshotRestoreUndoesARejectedTick)
{
  PathCommandLimiter limiter(0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.01, false, 0.05);
  ASSERT_TRUE(limiter.set_path({{0.0, 0.0}, {2.0, 0.0}}, {0.0, 0.0}));
  for (int step = 0; step < 5; ++step) {
    limiter.update({2.0, 0.0}, 0.05, true, Point2{0.0, 0.0});
  }
  const auto snapshot = limiter.snapshot();
  const auto position = *limiter.position();
  limiter.update({2.0, 0.0}, 0.05, true, Point2{0.0, 0.0});
  ASSERT_NE(limiter.position()->first, position.first);
  limiter.restore(snapshot);
  EXPECT_DOUBLE_EQ(limiter.position()->first, position.first);
  EXPECT_DOUBLE_EQ(limiter.position()->second, position.second);
}

TEST(PathCommandLimiter, NoAdvanceBrakesForwardOnly)
{
  PathCommandLimiter limiter(0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.01, false, 0.05);
  ASSERT_TRUE(limiter.set_path({{0.0, 0.0}, {2.0, 0.0}}, {0.0, 0.0}));
  for (int step = 0; step < 10; ++step) {
    limiter.update({2.0, 0.0}, 0.05, true, Point2{0.0, 0.0});
  }
  double previous = limiter.position()->first;
  for (int step = 0; step < 40; ++step) {
    limiter.update({2.0, 0.0}, 0.05, false, Point2{0.0, 0.0});
    EXPECT_GE(limiter.position()->first, previous - 1e-12);
    previous = limiter.position()->first;
  }
}

namespace
{
// A limiter already running at speed along a straight route, with the command
// point `offset` metres to the side of the replacement that is about to arrive.
px4_vio_bridge::PathCommandLimiter running_limiter()
{
  px4_vio_bridge::PathCommandLimiter limiter(
    0.20, 0.30, 0.10, 0.05, 0.30, 0.20, 0.01, true, 0.05);
  limiter.set_path({{0.0, 0.0}, {3.0, 0.0}}, {0.0, 0.0});
  for (int tick = 0; tick < 40; ++tick) {
    limiter.update({3.0, 0.0}, 0.05, true, limiter.position());
  }
  return limiter;
}

double speed_of(const px4_vio_bridge::PathCommandLimiter & limiter)
{
  return std::hypot(limiter.velocity().first, limiter.velocity().second);
}
}  // namespace

TEST(PathCommandLimiter, JoinKeepsMovingThroughAReplacementPath)
{
  // Regression for the stop-start route measured across 20260831T064152Z,
  // T064236Z and T064828Z. The planner replaced the accepted path 1.06-1.19
  // times a second; the old frozen-target join decelerated the command point to
  // a standstill on each one, so 20 of 27 stops began within 0.6 s of a
  // replacement and commanded speed only reached the 0.20 m/s ceiling in
  // samples where a path had gone untouched for over 1.5 s.
  auto limiter = running_limiter();
  const double before = speed_of(limiter);
  ASSERT_GT(before, 0.15) << "fixture failed to reach cruise";

  // A replacement offset by 30 mm -- the median command-point offset measured
  // at a rebuild on those flights, and one grid cell.
  ASSERT_TRUE(limiter.set_path({{0.0, 0.030}, {3.0, 0.030}}, {0.0, 0.030}));

  double slowest = before;
  for (int tick = 0; tick < 12; ++tick) {
    limiter.update({3.0, 0.030}, 0.05, true, limiter.position());
    slowest = std::min(slowest, speed_of(limiter));
  }
  EXPECT_GT(slowest, 0.5 * before)
    << "the command point stalled rejoining a 30 mm offset";
  EXPECT_GT(speed_of(limiter), 0.15) << "did not return to cruise after the join";
}

TEST(PathCommandLimiter, JoinStillRespectsTheProjectionBand)
{
  // The band is the safety property of a join and is not relaxed by rolling it.
  // max_projection_error 0.10 with a 0.16 m offset must be refused outright.
  auto limiter = running_limiter();
  EXPECT_THROW(
    limiter.set_path({{0.0, 0.160}, {3.0, 0.160}}, {0.0, 0.160}),
    std::invalid_argument);
}

TEST(PathCommandLimiter, JoinConvergesOntoTheReplacementPath)
{
  auto limiter = running_limiter();
  ASSERT_TRUE(limiter.set_path({{0.0, 0.060}, {3.0, 0.060}}, {0.0, 0.060}));
  for (int tick = 0; tick < 40; ++tick) {
    limiter.update({3.0, 0.060}, 0.05, true, limiter.position());
  }
  ASSERT_TRUE(limiter.position().has_value());
  EXPECT_NEAR(limiter.position()->second, 0.060, 5.0e-3)
    << "the join never actually reached the new path";
}

TEST(PathCommandLimiter, JoinBrakesForwardWhenTranslationIsPaused)
{
  // A yaw-alignment pause must still brake a join to a stop; the rolling
  // along-path term is gated on `advance` exactly as ordinary motion is.
  auto limiter = running_limiter();
  ASSERT_TRUE(limiter.set_path({{0.0, 0.030}, {3.0, 0.030}}, {0.0, 0.030}));
  for (int tick = 0; tick < 40; ++tick) {
    limiter.update({3.0, 0.030}, 0.05, false, limiter.position());
  }
  EXPECT_LT(speed_of(limiter), 0.02) << "advance=false did not brake the join";
}

TEST(PathCommandLimiter, JoinBrakesToTheEndOfAShortReplacement)
{
  // End-of-path braking applies during a join: a replacement with almost no
  // route left must not be entered at cruise.
  auto limiter = running_limiter();
  ASSERT_TRUE(limiter.position().has_value());
  const double x = limiter.position()->first;
  ASSERT_TRUE(limiter.set_path({{x - 0.05, 0.030}, {x + 0.10, 0.030}}, {x, 0.030}));
  for (int tick = 0; tick < 40; ++tick) {
    limiter.update({x + 0.10, 0.030}, 0.05, true, limiter.position());
  }
  EXPECT_LT(speed_of(limiter), 0.02) << "did not stop at the end of the route";
  EXPECT_LT(limiter.position()->first, x + 0.10 + 1.0e-6);
}
