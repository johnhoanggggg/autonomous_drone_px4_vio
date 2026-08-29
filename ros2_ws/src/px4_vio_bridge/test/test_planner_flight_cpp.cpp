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
