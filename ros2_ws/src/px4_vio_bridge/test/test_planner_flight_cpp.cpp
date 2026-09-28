#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "px4_vio_bridge/brake_damping.hpp"
#include "px4_vio_bridge/command_limiter.hpp"
#include "px4_vio_bridge/path_geometry.hpp"

using px4_vio_bridge::HorizontalCommandLimiter;
using px4_vio_bridge::PathCommandLimiter;
using px4_vio_bridge::Point2;
using px4_vio_bridge::Polyline;

TEST(BrakeDamping, DisabledIsAnExactPassThrough)
{
  px4_vio_bridge::BrakeDamping damping(false, 0.75, 0.10, 0.10, 0.30);
  const auto result = damping.update(
    {0.0, 0.0}, {-0.30, 0.0}, Point2{0.20, 0.0}, 1.0, true, true);
  EXPECT_EQ(result.acceleration, Point2(-0.30, 0.0));
  EXPECT_EQ(result.applied_correction, Point2(0.0, 0.0));
  EXPECT_EQ(result.reason, "disabled");
}

TEST(BrakeDamping, FiltersCapsAndPreservesTheTotalAccelerationLimit)
{
  px4_vio_bridge::BrakeDamping damping(true, 0.75, 0.10, 0.10, 0.30);
  damping.update({0.0, 0.0}, {0.0, 0.0}, Point2{1.0, 0.0}, 1.00, true, true);
  const auto filtered = damping.update(
    {0.0, 0.0}, {0.0, 0.0}, Point2{1.0, 0.0}, 1.10, true, true);
  EXPECT_NEAR(filtered.raw_correction.first, -0.75, 1e-12);
  EXPECT_NEAR(filtered.filtered_correction.first, -0.10 * (1.0 - std::exp(-1.0)), 1e-12);
  const auto capped = damping.update(
    {0.0, 0.0}, {-0.29, 0.0}, Point2{1.0, 0.0}, 1.20, true, true);
  EXPECT_NEAR(std::hypot(capped.acceleration.first, capped.acceleration.second), 0.30, 1e-12);
}

TEST(BrakeDamping, UnsafeExitAndCallbackGapClearStoredCorrection)
{
  px4_vio_bridge::BrakeDamping damping(true, 0.75, 0.10, 0.10, 0.30);
  damping.update({0.0, 0.0}, {0.0, 0.0}, Point2{1.0, 0.0}, 1.00, true, true);
  EXPECT_LT(damping.update(
      {0.0, 0.0}, {0.0, 0.0}, Point2{1.0, 0.0}, 1.10, true, true)
    .acceleration.first, 0.0);
  EXPECT_EQ(damping.update(
      {0.0, 0.0}, {0.0, 0.0}, Point2{1.0, 0.0}, 1.15, false, true)
    .reason, "inactive_state");
  EXPECT_DOUBLE_EQ(damping.update(
      {0.0, 0.0}, {0.0, 0.0}, Point2{1.0, 0.0}, 1.20, true, true)
    .acceleration.first, 0.0);
  damping.update({0.0, 0.0}, {0.0, 0.0}, Point2{1.0, 0.0}, 1.25, true, true);
  EXPECT_EQ(damping.update(
      {0.0, 0.0}, {0.0, 0.0}, Point2{1.0, 0.0}, 1.50, true, true)
    .reason, "callback_gap");
}

TEST(BrakeDamping, InvalidOrMissingVelocityNeverAppliesFeedback)
{
  px4_vio_bridge::BrakeDamping damping(true, 0.75, 0.10, 0.10, 0.30);
  const auto missing = damping.update(
    {0.0, 0.0}, {-0.20, 0.0}, std::nullopt, 1.0, true, true);
  EXPECT_EQ(missing.acceleration, Point2(-0.20, 0.0));
  EXPECT_EQ(missing.reason, "velocity_unavailable");
  const auto invalid = damping.update(
    {0.0, 0.0}, {-0.20, 0.0}, Point2{std::nan(""), 0.0}, 1.1, true, true);
  EXPECT_EQ(invalid.acceleration, Point2(-0.20, 0.0));
  EXPECT_EQ(invalid.reason, "velocity_unavailable");
}

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

px4_vio_bridge::PathCommandLimiter strict_limiter()
{
  px4_vio_bridge::PathCommandLimiter limiter(
    0.20, 0.30, 0.10, 0.05, 0.30, 0.20, 0.01, true, 0.05, true);
  limiter.set_path({{0.0, 0.0}, {3.0, 0.0}}, {0.0, 0.0});
  for (int tick = 0; tick < 40; ++tick) {
    limiter.update({3.0, 0.0}, 0.05, true, limiter.position());
  }
  return limiter;
}
}  // namespace

TEST(PathCommandLimiter, StrictModeBoundsSecantsAtSharpAndShallowCorners)
{
  for (const auto & path : {
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}},
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {1.98, 0.17}}})
  {
    PathCommandLimiter limiter(
      0.20, 0.30, 0.10, 0.05, 0.30, 0.20, 0.01, true, 0.05, true);
    ASSERT_TRUE(limiter.set_path(path, {0.0, 0.0}));
    Point2 previous_velocity{0.0, 0.0};
    bool saw_corner = false;
    bool reached_end = false;
    for (int tick = 0; tick < 1200; ++tick) {
      const double dt = tick % 3 == 0 ? 0.02 : (tick % 3 == 1 ? 0.05 : 0.08);
      limiter.update(path.back(), dt, true, limiter.position());
      const auto velocity = limiter.velocity();
      EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
        velocity.second - previous_velocity.second), 0.30 * dt + 1.0e-8);
      previous_velocity = velocity;
      if (limiter.waiting_vertex()) {
        saw_corner = true;
      }
      if (limiter.snapshot().progress >= limiter.path()->length() - 1.0e-8) {
        if (reached_end) {
          break;
        }
        reached_end = true;
      }
      if (reached_end && speed_of(limiter) <= 1.0e-8) {
        break;
      }
    }
    EXPECT_TRUE(saw_corner);
    EXPECT_GE(limiter.snapshot().progress, limiter.path()->length() - 1.0e-8);
    EXPECT_LT(speed_of(limiter), 1.0e-8);
  }
}

TEST(PathCommandLimiter, StrictModeDoesNotSnapAReplacementJoin)
{
  auto limiter = strict_limiter();
  const auto before = *limiter.position();
  ASSERT_TRUE(limiter.set_path(
    {{0.0, 0.002}, {3.0, 0.002}}, {0.0, 0.002}));
  EXPECT_TRUE(limiter.joining());
  EXPECT_NEAR(limiter.position()->first, before.first, 1.0e-12);
  EXPECT_NEAR(limiter.position()->second, before.second, 1.0e-12);
}

TEST(PathCommandLimiter, StrictModeTransitionsAChangedJoinTangent)
{
  auto limiter = strict_limiter();
  const Point2 before = *limiter.position();
  ASSERT_TRUE(limiter.set_path(
    {{before.first, 0.0}, {before.first, 3.0}}, {before.first, 0.0}));
  ASSERT_TRUE(limiter.joining()) << "a tangent change must use the bounded join";
  Point2 previous_velocity = limiter.velocity();
  for (int tick = 0; tick < 300 && limiter.joining(); ++tick) {
    const double dt = tick % 2 == 0 ? 0.05 : 0.08;
    limiter.update({before.first, 3.0}, dt, true, limiter.position());
    const auto velocity = limiter.velocity();
    EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
      velocity.second - previous_velocity.second), 0.30 * dt + 1.0e-8);
    previous_velocity = velocity;
  }
  EXPECT_FALSE(limiter.joining());
}

TEST(PathCommandLimiter, StrictModeBoundsJoinThroughCompletionAndPause)
{
  auto limiter = strict_limiter();
  ASSERT_TRUE(limiter.set_path(
    {{0.0, 0.030}, {3.0, 0.030}}, {0.0, 0.030}));
  Point2 previous_velocity = limiter.velocity();
  Point2 previous_position = *limiter.position();
  for (int tick = 0; tick < 300; ++tick) {
    const double dt = tick % 2 == 0 ? 0.05 : 0.08;
    limiter.update({3.0, 0.030}, dt, tick < 80, limiter.position());
    const auto velocity = limiter.velocity();
    EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
      velocity.second - previous_velocity.second), 0.30 * dt + 1.0e-8);
    EXPECT_NEAR((limiter.position()->first - previous_position.first) / dt,
      velocity.first, 1.0e-8);
    EXPECT_NEAR((limiter.position()->second - previous_position.second) / dt,
      velocity.second, 1.0e-8);
    previous_velocity = velocity;
    previous_position = *limiter.position();
    if (!limiter.joining() && tick > 80) {
      break;
    }
  }
  EXPECT_FALSE(limiter.joining());
}

TEST(PathCommandLimiter, StrictModeBrakesOnPathWhenTheRequestedPointRetreats)
{
  auto limiter = strict_limiter();
  const double speed_before = speed_of(limiter);
  const double progress_before = limiter.snapshot().progress;
  ASSERT_GT(speed_before, 0.15);
  Point2 previous_velocity = limiter.velocity();
  Point2 previous_position = *limiter.position();
  for (int tick = 0; tick < 30; ++tick) {
    // First put the carrot just inside the stopping distance, then retreat it
    // behind the command. Both requests must brake on the exact path rather
    // than forcing a zero secant or allowing the join to cruise onward.
    const Point2 target = tick == 0 ?
      Point2{previous_position.first + 0.01, previous_position.second} :
      Point2{0.0, 0.0};
    ASSERT_NO_THROW(limiter.update(target, 0.05, true, limiter.position(), 0.03));
    const auto velocity = limiter.velocity();
    EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
      velocity.second - previous_velocity.second), 0.30 * 0.05 + 1.0e-8);
    EXPECT_NEAR((limiter.position()->first - previous_position.first) / 0.05,
      velocity.first, 1.0e-8);
    EXPECT_NEAR((limiter.position()->second - previous_position.second) / 0.05,
      velocity.second, 1.0e-8);
    EXPECT_GE(limiter.snapshot().progress, 0.0);
    previous_velocity = velocity;
    previous_position = *limiter.position();
    if (speed_of(limiter) <= 1.0e-8) {
      break;
    }
  }
  EXPECT_LE(speed_of(limiter), 1.0e-8);
  EXPECT_LE(
    limiter.snapshot().progress - progress_before,
    speed_before * speed_before / (2.0 * 0.30) + speed_before * 0.05 + 1.0e-6);
}

TEST(PathCommandLimiter, StrictModeJoinHonorsARetreatedTarget)
{
  auto limiter = strict_limiter();
  const double progress_before = limiter.snapshot().progress;
  ASSERT_TRUE(limiter.set_path(
    {{0.0, 0.030}, {3.0, 0.030}}, {0.0, 0.030}));
  ASSERT_TRUE(limiter.joining());
  Point2 previous_velocity = limiter.velocity();
  for (int tick = 0; tick < 100; ++tick) {
    ASSERT_NO_THROW(limiter.update({0.0, 0.0}, 0.05, true, limiter.position(), 0.03));
    const auto velocity = limiter.velocity();
    EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
      velocity.second - previous_velocity.second), 0.30 * 0.05 + 1.0e-8);
    previous_velocity = velocity;
  }
  EXPECT_LE(speed_of(limiter), 1.0e-8);
  EXPECT_LE(
    limiter.snapshot().progress - progress_before,
    0.20 * 0.20 / (2.0 * 0.30) + 0.20 * 0.05 + 1.0e-6);
}

TEST(PathCommandLimiter, StrictModeBrakesPastASoftTargetAcrossCollinearWaypoints)
{
  PathCommandLimiter limiter(
    0.20, 0.30, 0.10, 0.05, 0.30, 0.20, 0.01, true, 0.05, true);
  ASSERT_TRUE(limiter.set_path({{0.0, 0.0}, {0.50, 0.0}, {2.0, 0.0}}, {0.0, 0.0}));
  for (int tick = 0; tick < 51; ++tick) {
    limiter.update({2.0, 0.0}, 0.05, true, limiter.position());
  }
  ASSERT_GT(speed_of(limiter), 0.19);
  const auto start = *limiter.position();
  // The soft carrot is only 1 mm ahead; the 0.50 m waypoint is straight,
  // so braking must remain continuous through it rather than freeze there.
  const Point2 target{start.first + 0.001, 0.0};
  Point2 previous_velocity = limiter.velocity();
  Point2 previous_position = start;
  for (int tick = 0; tick < 30; ++tick) {
    ASSERT_NO_THROW(limiter.update(target, 0.05, true, limiter.position()));
    const auto velocity = limiter.velocity();
    EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
      velocity.second - previous_velocity.second), 0.015 + 1.0e-8);
    EXPECT_NEAR((limiter.position()->first - previous_position.first) / 0.05,
      velocity.first, 1.0e-8);
    EXPECT_NEAR(limiter.position()->second, 0.0, 1.0e-10);
    previous_velocity = velocity;
    previous_position = *limiter.position();
  }
  EXPECT_LE(speed_of(limiter), 1.0e-8);
  EXPECT_GT(limiter.position()->first, 0.50);
  EXPECT_LE(limiter.position()->first - start.first, 0.077);
}

TEST(PathCommandLimiter, StrictModeJitteredDtAndReplansReachTheRouteEnd)
{
  PathCommandLimiter limiter(
    0.20, 0.30, 0.10, 0.05, 0.30, 0.20, 0.01, true, 0.05, true);
  std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  ASSERT_TRUE(limiter.set_path(path, {0.0, 0.0}));
  Point2 previous_velocity{0.0, 0.0};
  for (int tick = 0; tick < 1800; ++tick) {
    // Replan a coordinate-shifted copy a few times, as the flight adapter does.
    if (tick == 140 || tick == 520) {
      const double shift = tick == 140 ? 0.002 : -0.002;
      path = {{0.0, shift}, {1.0, shift}, {1.0, 1.0 + shift}};
      ASSERT_TRUE(limiter.set_path(path, *limiter.position()));
    }
    const double dt = 0.02 + 0.01 * ((37 * tick + 11) % 7);
    const Point2 reference = limiter.waiting_vertex() && tick % 5 < 3 ?
      Point2{0.0, 2.0} : *limiter.position();
    ASSERT_NO_THROW(limiter.update(path.back(), dt, true, reference));
    const auto velocity = limiter.velocity();
    EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
      velocity.second - previous_velocity.second), 0.30 * dt + 1.0e-8);
    previous_velocity = velocity;
    if (limiter.snapshot().progress >= limiter.path()->length() - 1.0e-8 &&
      speed_of(limiter) <= 1.0e-8)
    {
      break;
    }
  }
  ASSERT_GE(limiter.snapshot().progress, limiter.path()->length() - 1.0e-8);
  EXPECT_LT(speed_of(limiter), 1.0e-8);
}

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

TEST(CommandHeading, PreviewStartsFromRestWithoutAdvancingTheCommand)
{
  PathCommandLimiter limiter;
  limiter.set_path({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, {0.0, 0.0});
  const auto before = limiter.snapshot();
  // Follower aims diagonally across the corner; the accepted command goes east.
  const auto direction = limiter.preview_direction({1.0, 1.0}, 0.05, {0.0, 0.0}, 0.15);
  ASSERT_TRUE(direction);
  EXPECT_NEAR(direction->first, 1.0, 1e-12);
  EXPECT_NEAR(direction->second, 0.0, 1e-12);
  EXPECT_EQ(limiter.snapshot().position, before.position);
  EXPECT_EQ(limiter.snapshot().velocity, before.velocity);
  EXPECT_EQ(limiter.snapshot().progress, before.progress);
  EXPECT_EQ(limiter.snapshot().speed, before.speed);
  // A yaw hold does not hide the heading needed to release the hold.
  for (int i = 0; i < 10; ++i) {
    limiter.update({1.0, 1.0}, 0.05, false, Point2{0.0, 0.0});
    EXPECT_EQ(limiter.preview_direction({1.0, 1.0}, 0.05, {0.0, 0.0}, 0.15), direction);
  }
  EXPECT_EQ(limiter.position(), before.position);
  limiter.update({1.0, 1.0}, 0.05, true, Point2{0.0, 0.0});
  EXPECT_GT(limiter.position()->first, 0.0);
}

TEST(PathCommandLimiter, JoinMergesInsideTheCappedAngleAndStillFinishes)
{
  // Regression for the corner weave on 20260906T120129Z / T120238Z. The three
  // original bounds let the lateral term reach cruise speed, so a replacement a
  // few centimetres to the side was commanded as a 45-degree dart. Every
  // commanded velocity must now stay within the merge cap of the route.
  for (const double offset : {0.02, 0.035, 0.05}) {
    auto limiter = running_limiter();
    ASSERT_TRUE(limiter.set_path({{0.0, offset}, {3.0, offset}}, {0.0, offset}))
      << "offset " << offset;
    ASSERT_TRUE(limiter.joining()) << "offset " << offset;
    double worst = 0.0;
    int ticks = 0;
    for (; ticks < 200 && limiter.joining(); ++ticks) {
      limiter.update({3.0, offset}, 0.05, true, limiter.position());
      const auto v = limiter.velocity();
      if (std::hypot(v.first, v.second) > 1.0e-6) {
        worst = std::max(worst, std::abs(std::atan2(v.second, v.first)));
      }
    }
    EXPECT_LT(worst, 16.0 * M_PI / 180.0)
      << "offset " << offset << " commanded " << worst * 180.0 / M_PI << " deg off route";
    EXPECT_FALSE(limiter.joining()) << "offset " << offset << " never finished merging";
    EXPECT_LT(ticks * 0.05, 4.0) << "offset " << offset << " merge took too long";
  }
}

TEST(PathCommandLimiter, JoinEscalatesToCloseALargeOffsetInBoundedTime)
{
  // 20260906T124409Z: a hard replan left the command 0.168 m off the new path.
  // The angle bound alone needed about three seconds, the follower tripped at
  // 0.9 s, and the fault froze the merge with the offset standing. An offset
  // this large is a real replan, not a re-anchor, and has to close promptly.
  // The flown projection tolerance is 0.05 m, and it is what the escalation is
  // measured against, so this runs the launch configuration rather than the
  // looser 0.10 m the shared fixture uses.
  px4_vio_bridge::PathCommandLimiter limiter(
    0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.01, true, 0.05);
  limiter.set_path({{0.0, 0.0}, {3.0, 0.0}}, {0.0, 0.0});
  for (int tick = 0; tick < 40; ++tick) {
    limiter.update({3.0, 0.0}, 0.05, true, limiter.position());
  }
  const auto clearance = [](const Point2 &, const Point2 &) {return true;};
  ASSERT_TRUE(limiter.set_path({{0.0, 0.170}, {3.0, 0.170}}, {0.0, 0.170}, clearance));
  ASSERT_TRUE(limiter.joining());
  double worst = 0.0;
  double under_follower_limit = -1.0;
  int ticks = 0;
  for (; ticks < 200 && limiter.joining(); ++ticks) {
    limiter.update({3.0, 0.170}, 0.05, true, limiter.position());
    const auto v = limiter.velocity();
    if (std::hypot(v.first, v.second) > 1.0e-6) {
      worst = std::max(worst, std::abs(std::atan2(v.second, v.first)));
    }
    // The follower's own max_cross_track is 0.08 m, and tripping it is what
    // latched the fault that froze the merge and landed the aircraft. Getting
    // back inside it quickly matters more than when the last 3 mm closes.
    if (under_follower_limit < 0.0 &&
      std::abs(limiter.position()->second - 0.170) < 0.08)
    {
      under_follower_limit = (ticks + 1) * 0.05;
    }
  }
  EXPECT_FALSE(limiter.joining()) << "a large offset never closed";
  EXPECT_GT(under_follower_limit, 0.0);
  // The along-path and lateral terms share one `max_speed_` budget, so the norm
  // limit shrinks both, and the velocity still has to rotate under the
  // acceleration limit -- the merge is slower than the bound alone implies:
  // 1.05 s measured here, against about 3 s from the flat bound.
  // The flight tripped at 0.9 s, so this shortens the exposure rather than
  // removing it: what turned that trip into a landing is that a latched fault
  // stops the limiter being ticked at all, which this does not address.
  EXPECT_LT(under_follower_limit, 1.2)
    << "still outside the follower's cross-track limit after "
    << under_follower_limit << " s";
  EXPECT_LT(ticks * 0.05, 3.0) << "took " << ticks * 0.05 << " s to merge";
  EXPECT_GT(worst, 16.0 * M_PI / 180.0) << "the time bound never escalated past the angle bound";
  EXPECT_LT(worst, 50.0 * M_PI / 180.0) << "escalated further than the old uncapped merge";
}

TEST(PathCommandLimiter, JoinFinishesWithNoAlongPathMotionLeft)
{
  // The cap scales with the along-path rate, so the floor is what closes a
  // rejoin opened where there is no route left to travel.
  auto limiter = running_limiter();
  ASSERT_TRUE(limiter.position().has_value());
  const double x = limiter.position()->first;
  ASSERT_TRUE(limiter.set_path({{x - 0.05, 0.030}, {x + 0.02, 0.030}}, {x, 0.030}));
  ASSERT_TRUE(limiter.joining());
  for (int tick = 0; tick < 400 && limiter.joining(); ++tick) {
    limiter.update({x + 0.02, 0.030}, 0.05, true, limiter.position());
  }
  EXPECT_FALSE(limiter.joining()) << "a rejoin with no along-path motion never completed";
}

TEST(CommandHeading, PreviewFollowsTheRouteNotTheJoinAndLeavesAllJoinStateIntact)
{
  auto limiter = running_limiter();
  limiter.set_path({{0.0, 0.060}, {3.0, 0.060}}, {0.0, 0.060});
  ASSERT_TRUE(limiter.joining());
  const auto before = limiter.snapshot();
  const auto direction = limiter.preview_direction({3.0, 0.060}, 0.05, *limiter.position(), 0.15);
  ASSERT_TRUE(direction);
  const auto after = limiter.snapshot();
  EXPECT_EQ(after.installation, before.installation);
  EXPECT_EQ(after.position, before.position);
  EXPECT_EQ(after.velocity, before.velocity);
  EXPECT_EQ(after.progress, before.progress);
  EXPECT_EQ(after.speed, before.speed);
  EXPECT_EQ(after.join_target, before.join_target);
  EXPECT_EQ(after.join_limit, before.join_limit);
  EXPECT_EQ(after.waiting_vertex, before.waiting_vertex);
  // The route is east. The lane change north is a 6 cm correction the command
  // point makes on its way, and it must not be flown as a heading.
  EXPECT_NEAR(direction->first, 1.0, 1e-12);
  EXPECT_NEAR(direction->second, 0.0, 1e-12);
  limiter.update({3.0, 0.060}, 0.05, true, before.position);
  EXPECT_GT(limiter.velocity().second, 1.0e-3)
    << "this tick is supposed to contain the lateral term the heading ignores";
}

TEST(CommandHeading, HeadingSurvivesBrakingThenAReplacementDuringAlignment)
{
  // The 2026-09-06 stall: 20260906T113239Z spent 39.5 s of a 42.5 s route in
  // YAW_ALIGN and netted 0.295 m. Alignment brakes the command point; the next
  // replacement opens a join; the one-tick velocity out of rest is then the
  // acceleration direction, which the lateral term dominates. Heading swung to
  // ~85 degrees off the route, which restarted the alignment that had just
  // released, on a route that never turned.
  // Above `suffix_tolerance` (0.01 m here), or the replacement is treated as
  // the same geometry renumbered and never opens a join at all.
  for (const double offset : {0.02, 0.03, 0.10}) {
    auto limiter = running_limiter();
    const auto moving = limiter.preview_direction({3.0, 0.0}, 0.05, *limiter.position(), 0.15);
    ASSERT_TRUE(moving) << "offset " << offset;
    EXPECT_NEAR(moving->first, 1.0, 1e-12) << "offset " << offset;

    // Alignment pauses translation. Brake to a standstill on the same route.
    for (int tick = 0; tick < 40; ++tick) {
      limiter.update({3.0, 0.0}, 0.05, false, limiter.position());
    }
    ASSERT_LT(speed_of(limiter), 1.0e-6) << "offset " << offset;

    // The planner republishes, shifted sideways by less than the drone is wide.
    ASSERT_TRUE(limiter.set_path({{0.0, offset}, {3.0, offset}}, {0.0, offset}));
    ASSERT_TRUE(limiter.joining()) << "offset " << offset;
    const auto stopped = limiter.preview_direction(
      {3.0, offset}, 0.05, *limiter.position(), 0.15);
    ASSERT_TRUE(stopped) << "offset " << offset;
    EXPECT_NEAR(stopped->first, 1.0, 1e-12) << "offset " << offset;
    EXPECT_NEAR(stopped->second, 0.0, 1e-12) << "offset " << offset;
  }
}

TEST(CommandHeading, RepeatedReplacementsWhileHeldCannotWalkTheHeading)
{
  // The alignment loop ran for 12 entries over 42 s, so one tick is not enough
  // coverage: hold translation and keep replacing the path, as the planner does
  // at about 1 Hz, and the commanded heading must stay on the route.
  auto limiter = running_limiter();
  double worst = 0.0;
  for (int cycle = 0; cycle < 20; ++cycle) {
    const double offset = (cycle % 2 == 0 ? 1.0 : -1.0) * 0.02 * (1 + cycle % 3);
    ASSERT_TRUE(limiter.set_path({{0.0, offset}, {3.0, offset}}, {0.0, offset}))
      << "cycle " << cycle;
    for (int tick = 0; tick < 5; ++tick) {
      const auto reference = *limiter.position();
      const auto direction = limiter.preview_direction({3.0, offset}, 0.05, reference, 0.15);
      ASSERT_TRUE(direction) << "cycle " << cycle;
      worst = std::max(worst, std::abs(std::atan2(direction->second, direction->first)));
      limiter.update({3.0, offset}, 0.05, false, reference);
    }
  }
  EXPECT_LT(worst, 1.0e-9) << "heading walked off an unchanged eastbound route";
}

TEST(CommandHeading, RejectedReplacementKeepsHeadingOnAcceptedPath)
{
  auto limiter = running_limiter();
  EXPECT_THROW(limiter.set_path({{0.0, 1.0}, {0.0, 3.0}}, {0.0, 1.0}), std::invalid_argument);
  const auto direction = limiter.preview_direction({3.0, 1.0}, 0.05, *limiter.position(), 0.15);
  ASSERT_TRUE(direction);
  EXPECT_NEAR(direction->first, 1.0, 1e-12);
  EXPECT_NEAR(direction->second, 0.0, 1e-12);
}

TEST(CommandHeading, PreviewDoesNotPreventBrakingDuringYawHold)
{
  auto limiter = running_limiter();
  limiter.set_path({{0.0, 0.030}, {3.0, 0.030}}, {0.0, 0.030});
  auto without_preview = limiter;
  for (int tick = 0; tick < 40; ++tick) {
    const auto reference = *limiter.position();
    (void)limiter.preview_direction({3.0, 0.030}, 0.05, reference, 0.15);
    limiter.update({3.0, 0.030}, 0.05, false, reference);
    without_preview.update({3.0, 0.030}, 0.05, false, reference);
    EXPECT_EQ(limiter.position(), without_preview.position());
    EXPECT_EQ(limiter.velocity(), without_preview.velocity());
  }
  EXPECT_LT(speed_of(limiter), 0.02);
}

TEST(CommandHeading, CornerWaitFacesIncomingThenOutgoingLegWithoutConsumingWait)
{
  PathCommandLimiter limiter;
  limiter.set_path({{0.0, 0.0}, {0.2, 0.0}, {0.2, 0.5}}, {0.0, 0.0});
  for (int i = 0; i < 200 && !limiter.waiting_vertex(); ++i) {
    limiter.update({0.2, 0.5}, 0.05, true, Point2{0.0, 0.0});
  }
  ASSERT_TRUE(limiter.waiting_vertex());
  auto direction = limiter.preview_direction({0.2, 0.5}, 0.05, {0.0, 0.0}, 0.15);
  ASSERT_TRUE(direction);
  EXPECT_NEAR(direction->first, 1.0, 1e-12);
  direction = limiter.preview_direction({0.2, 0.5}, 0.05, {0.2, 0.0}, 0.15);
  ASSERT_TRUE(direction);
  EXPECT_NEAR(direction->first, 0.0, 1e-12);
  EXPECT_NEAR(direction->second, 1.0, 1e-12);
  EXPECT_TRUE(limiter.waiting_vertex());
  EXPECT_NEAR(limiter.position()->second, 0.0, 1e-12);
}

TEST(CommandHeading, AtGoalRetainsHeadingAndFailedPreviewCannotMutateState)
{
  PathCommandLimiter limiter;
  limiter.set_path({{0.0, 0.0}, {1.0, 0.0}}, {0.0, 0.0});
  const auto before = limiter.snapshot();
  EXPECT_FALSE(limiter.preview_direction({0.0, 0.0}, 0.05, {0.0, 0.0}, 0.15));
  EXPECT_THROW(
    (void)limiter.preview_direction({1.0, 0.0}, -0.05, {0.0, 0.0}, 0.15), std::invalid_argument);
  EXPECT_THROW(
    (void)limiter.preview_direction({1.0, 0.0}, 0.05, {0.0, 0.0}, 0.0), std::invalid_argument);
  EXPECT_EQ(limiter.snapshot().position, before.position);
  EXPECT_EQ(limiter.snapshot().progress, before.progress);
}

TEST(CommandHeading, MapCorrectionIsRemovedBeforeNedHeading)
{
  PathCommandLimiter limiter;
  // East in VIO has been rotated north in the map by a +90 degree correction.
  limiter.set_path({{0.0, 0.0}, {0.0, 1.0}}, {0.0, 0.0});
  const auto direction = limiter.preview_direction({0.0, 1.0}, 0.05, {0.0, 0.0}, 0.15);
  ASSERT_TRUE(direction);
  const auto vio = px4_vio_bridge::map_displacement_to_vio(*direction, M_PI / 2.0);
  const auto heading = px4_vio_bridge::ned_track_heading(
    px4_vio_bridge::vio_enu_displacement_to_ned(vio), 1.0e-6);
  ASSERT_TRUE(heading);
  EXPECT_NEAR(*heading, M_PI / 2.0, 1e-12);
}

TEST(CommandHeading, RecordedShortDiagonalDoesNotBecomeAFullYawExcursion)
{
  // 095742Z, path received at 13.171 s: 12.7 cm at 135.586 degrees NED,
  // followed by 87 cm at 90.586 degrees. The old preview switches by 45 deg.
  const Polyline path({{0.4720230894, 0.0197502850}, {0.5620230874, -0.0702497130},
    {1.4320230680, -0.0702497130}, {1.7620230606, 0.2597502796}});
  const auto heading = [&](double progress, double lookahead) {
      const auto direction = px4_vio_bridge::route_heading_direction(path, progress, lookahead);
      return *px4_vio_bridge::ned_track_heading(px4_vio_bridge::vio_enu_displacement_to_ned(
          px4_vio_bridge::map_displacement_to_vio(*direction, 0.5861301866 * M_PI / 180)), 1e-6);
    };
  const double corner = path.cumulative()[1];
  EXPECT_NEAR((heading(corner - 0.001, 0) - heading(corner, 0)) * 180 / M_PI, 45, 1e-5);
  EXPECT_LT(std::abs(heading(corner - 0.001, 0.35) - heading(corner, 0.35)), 0.003);
  EXPECT_LT(std::abs(heading(0, 0.35) - 95 * M_PI / 180), 15 * M_PI / 180);
}

TEST(CommandHeading, PreviewDoesNotAverageAcrossAHairpinOrInventAGoalDirection)
{
  const Polyline path({{0, 0}, {1, 0}, {0, 0}});
  const auto incoming = px4_vio_bridge::route_heading_direction(path, 0.9, 0.35);
  const auto outgoing = px4_vio_bridge::route_heading_direction(path, 1.0, 0.35);
  ASSERT_TRUE(incoming);
  ASSERT_TRUE(outgoing);
  EXPECT_NEAR(incoming->first, 1, 1e-12);
  EXPECT_NEAR(outgoing->first, -1, 1e-12);
  EXPECT_FALSE(px4_vio_bridge::route_heading_direction(Polyline({{0, 0}}), 0, 0.35));
  auto limiter = running_limiter();
  const auto before = limiter.snapshot();
  EXPECT_THROW((void)limiter.preview_direction({3, 0}, 0.05, {0, 0}, 0.15, -1),
    std::invalid_argument);
  EXPECT_EQ(limiter.snapshot().progress, before.progress);
}

TEST(CommandHeading, AveragedPreviewCannotChangeCommandOrConsumeACornerWait)
{
  PathCommandLimiter limiter;
  limiter.set_path({{0, 0}, {0.2, 0}, {0.2, 0.5}}, {0, 0});
  auto original = limiter;
  for (int i = 0; i < 200; ++i) {
    (void)limiter.preview_direction({0.2, 0.5}, 0.05, {0, 0}, 0.15, 0.35);
    limiter.update({0.2, 0.5}, 0.05, true, Point2{0, 0});
    original.update({0.2, 0.5}, 0.05, true, Point2{0, 0});
    EXPECT_EQ(limiter.position(), original.position());
    EXPECT_EQ(limiter.velocity(), original.velocity());
    EXPECT_EQ(limiter.waiting_vertex(), original.waiting_vertex());
  }
  ASSERT_TRUE(limiter.waiting_vertex());
  const auto direction = limiter.preview_direction({0.2, 0.5}, 0.05, {0, 0}, 0.15, 0.35);
  ASSERT_TRUE(direction);
  EXPECT_NEAR(direction->first, 1, 1e-12);
}

TEST(HeadingReference, ReplansStayBoundedButGenuineTurnsStillTriggerAlignment)
{
  px4_vio_bridge::HeadingReference heading;
  heading.reset(0.0);
  double previous = 0;
  for (int i = 0; i < 40; ++i) {
    const double goal = i < 10 ? M_PI / 2 : -M_PI / 2;
    const double target = *heading.update(goal, 0.1, 0.4, 0.05);
    EXPECT_LE(std::abs(px4_vio_bridge::wrap_pi(target - previous)), 0.020000001);
    EXPECT_GE(std::abs(*heading.alignment_error(0)), M_PI / 2);
    previous = target;
  }
  // An accepted reference must survive a rejected proposal verbatim.
  const auto accepted = heading;
  heading.update(M_PI, 0.1, 0.4, 0.05);
  heading = accepted;
  EXPECT_EQ(heading.target(), accepted.target());
  EXPECT_EQ(heading.goal(), accepted.goal());
  EXPECT_EQ(heading.preview(), accepted.preview());
}

TEST(HeadingReference, DeadbandDoesNotStallTheSlewAndWrapTakesTheShortWay)
{
  px4_vio_bridge::HeadingReference heading;
  heading.reset(179 * M_PI / 180);
  const double goal = -160 * M_PI / 180;
  double previous = *heading.target();
  for (int i = 0; i < 100; ++i) {
    const double target = *heading.update(goal, 15 * M_PI / 180, 0.4, 0.05);
    const double delta = px4_vio_bridge::wrap_pi(target - previous);
    EXPECT_GE(delta, -1e-12);
    EXPECT_LE(delta, 0.020000001);
    previous = target;
  }
  EXPECT_NEAR(*heading.target(), goal, 1e-12);
  heading.update(std::nullopt, 0.1, 0.4, 0.05);
  EXPECT_NEAR(*heading.target(), goal, 1e-12);
  EXPECT_THROW(heading.update(0.0, 0.1, -1, 0.05), std::invalid_argument);
  EXPECT_NEAR(*heading.target(), goal, 1e-12);
  heading.reset(0.3);
  EXPECT_EQ(heading.goal(), heading.target());
  EXPECT_FALSE(heading.preview());
}

TEST(HeadingReference, ZeroRateRetainsLegacyDeadbandAndGateBehavior)
{
  px4_vio_bridge::HeadingReference heading;
  std::optional<double> legacy;
  for (double goal : {0.2, 0.25, 1.0, -2.0, -2.05}) {
    legacy = px4_vio_bridge::track_yaw_target(legacy, goal, 0.1);
    EXPECT_EQ(heading.update(goal, 0.1, 0, 0.05), legacy);
    EXPECT_NEAR(*heading.alignment_error(0.1), px4_vio_bridge::wrap_pi(*legacy - 0.1), 1e-12);
  }
}

TEST(PathCommandLimiter, SpeedCapBrakesToTheCeilingWithoutSteppingVelocity)
{
  // Flights 20260908T032454Z and T032636Z stepped from cruise to the
  // follower ride-out cap in one tick. At 20 Hz this demanded 2.8 m/s^2,
  // despite the configured 0.3 m/s^2 limit and clipped acceleration FF.
  auto capped = running_limiter();
  auto normal = running_limiter();
  for (int tick = 0; tick < 40; ++tick) {
    const auto before = capped.velocity();
    const auto position = *capped.position();
    capped.update({3.0, 0.0}, 0.05, true, capped.position(), 0.06);
    normal.update({3.0, 0.0}, 0.05, true, normal.position());
    EXPECT_NEAR(speed_of(capped), std::max(0.06, 0.20 - 0.015 * (tick + 1)), 1e-9);
    EXPECT_LE(std::hypot(capped.velocity().first - before.first,
      capped.velocity().second - before.second), 0.015 + 1e-9);
    EXPECT_NEAR((capped.position()->first - position.first) / 0.05,
      capped.velocity().first, 1e-9);
    EXPECT_NEAR(capped.position()->second, 0.0, 1.0e-9) << "left the route";
  }
  EXPECT_LT(capped.position()->first, normal.position()->first)
    << "the capped command should be behind the uncapped one";
  EXPECT_GT(capped.position()->first, 0.0) << "the cap stopped the command instead of slowing it";
  // Releasing the cap returns to cruise rather than restarting from rest.
  const double resumed_from = speed_of(capped);
  capped.update({3.0, 0.0}, 0.05, true, capped.position());
  EXPECT_GT(speed_of(capped), resumed_from);
}

TEST(PathCommandLimiter, SpeedCapBrakesARejoinWithoutSteppingTheVelocityVector)
{
  auto limiter = running_limiter();
  ASSERT_TRUE(limiter.set_path({{0.0, 0.030}, {3.0, 0.030}}, {0.0, 0.030}));
  ASSERT_TRUE(limiter.joining());
  for (int tick = 0; tick < 40 && limiter.joining(); ++tick) {
    const auto before = limiter.velocity();
    limiter.update({3.0, 0.030}, 0.05, true, limiter.position(), 0.06);
    EXPECT_LE(std::hypot(limiter.velocity().first - before.first,
      limiter.velocity().second - before.second), 0.015 + 1e-9);
    if (tick >= 10) {
      EXPECT_LE(speed_of(limiter), 0.06 + 1.0e-9);
    }
  }
}

TEST(PathCommandLimiter, ZeroCapBrakesToRestOnPathAndDuringRejoin)
{
  for (const bool join : {false, true}) {
    auto limiter = running_limiter();
    if (join) {
      ASSERT_TRUE(limiter.set_path({{0.0, 0.030}, {3.0, 0.030}}, {0.0, 0.030}));
    }
    for (const double dt : {0.02, 0.08, 0.04, 0.06, 0.10, 0.10, 0.10, 0.10, 0.10}) {
      const auto before = limiter.velocity();
      limiter.update({3.0, 0.030}, dt, true, limiter.position(), 0.0);
      EXPECT_LE(std::hypot(limiter.velocity().first - before.first,
        limiter.velocity().second - before.second), 0.30 * dt + 1e-9);
    }
    EXPECT_NEAR(speed_of(limiter), 0.0, 1e-9);
    limiter.update({3.0, 0.030}, 0.05, true, limiter.position());
    EXPECT_GT(speed_of(limiter), 0.0);
    EXPECT_LE(speed_of(limiter), 0.015 + 1e-9);
  }
}

TEST(PathCommandLimiter, SpeedCapRejectsNonsense)
{
  auto limiter = running_limiter();
  EXPECT_THROW(
    limiter.update({3.0, 0.0}, 0.05, true, limiter.position(), -0.01), std::invalid_argument);
  EXPECT_THROW(
    limiter.update(
      {3.0, 0.0}, 0.05, true, limiter.position(),
      std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(YawTranslation, BeginsBrakingBeforeTheHoldWithoutSteppingSpeed)
{
  auto limiter = running_limiter();
  const double resume = M_PI / 12.0, hold = M_PI * 40.0 / 180.0;
  for (int tick = 0; tick < 20; ++tick) {
    const auto previous = limiter.velocity();
    const auto cap = px4_vio_bridge::yaw_translation_speed_cap(
      M_PI * 30.0 / 180.0, resume, hold, 0.2, previous, 0.3, 0.05);
    ASSERT_TRUE(cap);
    limiter.update({3.0, 0.0}, 0.05, true, limiter.position(), cap);
    EXPECT_LE(std::hypot(limiter.velocity().first - previous.first,
      limiter.velocity().second - previous.second), 0.015 + 1e-9);
    EXPECT_NEAR(limiter.position()->second, 0.0, 1e-12);
  }
  // 30 degrees lies 60% of the way from 15 to 40: 40% cruise remains.
  EXPECT_NEAR(speed_of(limiter), 0.08, 1e-9);
  const auto cap = px4_vio_bridge::yaw_translation_speed_cap(
    resume, resume, hold, 0.2, limiter.velocity(), 0.3, 0.05);
  EXPECT_FALSE(cap);
  limiter.update({3.0, 0.0}, 0.05, true, limiter.position(), cap);
  EXPECT_NEAR(speed_of(limiter), 0.095, 1e-9);
}

TEST(YawTranslation, SymmetricAndDisabledWhenHeadingOrAlignmentIsUnavailable)
{
  using px4_vio_bridge::yaw_translation_speed_cap;
  const Point2 stopped{0.0, 0.0};
  EXPECT_EQ(yaw_translation_speed_cap(0.5, 0.2, 0.7, 0.2, stopped, 0.3, 0.05),
    yaw_translation_speed_cap(-0.5, 0.2, 0.7, 0.2, stopped, 0.3, 0.05));
  EXPECT_FALSE(yaw_translation_speed_cap(std::nullopt, 0.2, 0.7, 0.2, stopped, 0.3, 0.05));
  EXPECT_FALSE(yaw_translation_speed_cap(0.5, 0.0, 0.0, 0.2, stopped, 0.3, 0.05));
  EXPECT_FALSE(yaw_translation_speed_cap(0.5, 0.2, 0.2, 0.2, stopped, 0.3, 0.05));
  EXPECT_THROW(yaw_translation_speed_cap(std::nan(""), 0.2, 0.7, 0.2, stopped, 0.3, 0.05),
    std::invalid_argument);
}

TEST(YawTranslation, KeepsExistingHoldAndStricterFollowerCap)
{
  auto limiter = running_limiter();
  for (int tick = 0; tick < 20; ++tick) {
    auto cap = px4_vio_bridge::yaw_translation_speed_cap(
      0.6, 0.2, 0.7, 0.2, limiter.velocity(), 0.3, 0.05);
    ASSERT_TRUE(cap);
    limiter.update({3.0, 0.0}, 0.05, true, limiter.position(), std::min(*cap, 0.06));
    // The yaw taper eventually requests 0.04, stricter than the 0.06 crawl;
    // both requests must preserve deceleration from the standing velocity.
    EXPECT_NEAR(speed_of(limiter), std::max(0.04, 0.20 - 0.015 * (tick + 1)), 1e-9);
  }
  for (int tick = 0; tick < 20; ++tick) {
    const auto cap = px4_vio_bridge::yaw_translation_speed_cap(
      0.8, 0.2, 0.7, 0.2, limiter.velocity(), 0.3, 0.05);
    limiter.update({3.0, 0.0}, 0.05, false, limiter.position(), cap);
  }
  EXPECT_NEAR(speed_of(limiter), 0.0, 1e-9);
}

// --- carrying the command across a map correction ---------------------------
//
// 20260908T052413Z, t=19.29: RTAB-Map published a 0.396 m / 5.44 deg loop
// closure in one message. The vehicle's map pose moved into the new solution
// immediately; the limiter's command point, velocity and path stayed in the old
// one, because nothing re-expressed them. The adapter publishes
// `route_limiter_->position() - map_pose_` as the PX4 setpoint displacement, so
// that difference picked up the whole correction delta in a single 50 ms tick:
// the setpoint's lead ahead of the vehicle went 0.073 m -> 0.365 m and PX4
// closed it at 0.30 m/s while the commanded crawl was 0.06 m/s. The limiter's
// own speed cap cannot bind this -- it bounds the command point, and it was the
// reference being subtracted that moved.

namespace
{
/// The correction either side of that closure, radians for the yaw.
constexpr px4_vio_bridge::Correction4 kBeforeClosure{-0.0306, -0.0080, 0.0, -0.00436};
constexpr px4_vio_bridge::Correction4 kAfterClosure{-0.3072, -0.2912, 0.0, 0.09495};

/// Wind a fresh limiter a little way along a straight route.
PathCommandLimiter cruising_limiter()
{
  PathCommandLimiter limiter(0.20, 0.30, 0.05, 0.05, 0.30, 0.20, 0.01, false, 0.05);
  limiter.set_path({{0.0, 0.0}, {2.0, 0.0}}, {0.0, 0.0});
  for (int tick = 0; tick < 10; ++tick) {
    limiter.update({2.0, 0.0}, 0.05);
  }
  return limiter;
}
}  // namespace

TEST(PathCommandLimiter, ReexpressKeepsTheCommandedDisplacementContinuous)
{
  auto limiter = cruising_limiter();
  const Point2 pose_before{0.05, 0.0};
  const Point2 command_before = *limiter.position();
  const Point2 lead_before{
    command_before.first - pose_before.first, command_before.second - pose_before.second};

  // The caller re-derives the pose through the new correction; the limiter has
  // to follow it into the same solution.
  const Point2 pose_after =
    px4_vio_bridge::reexpress_point(pose_before, kBeforeClosure, kAfterClosure);
  limiter.reexpress(kBeforeClosure, kAfterClosure);
  const Point2 command_after = *limiter.position();
  const Point2 lead_after{
    command_after.first - pose_after.first, command_after.second - pose_after.second};

  // What actually reaches PX4 is the displacement rotated out of the map frame,
  // so that is the quantity which must not move.
  const Point2 vio_before =
    px4_vio_bridge::map_displacement_to_vio(lead_before, kBeforeClosure[3]);
  const Point2 vio_after =
    px4_vio_bridge::map_displacement_to_vio(lead_after, kAfterClosure[3]);
  EXPECT_NEAR(vio_after.first, vio_before.first, 1e-9);
  EXPECT_NEAR(vio_after.second, vio_before.second, 1e-9);
}

TEST(PathCommandLimiter, WithoutReexpressTheDisplacementStepsByTheCorrection)
{
  // The failure this guards against, stated as the size of the step: leaving
  // the command point in the old solution while the pose moves into the new one
  // is worth about the correction's own translation, in one tick.
  auto limiter = cruising_limiter();
  const Point2 pose_before{0.05, 0.0};
  const Point2 stale_command = *limiter.position();
  const Point2 pose_after =
    px4_vio_bridge::reexpress_point(pose_before, kBeforeClosure, kAfterClosure);

  const Point2 lead_before{
    stale_command.first - pose_before.first, stale_command.second - pose_before.second};
  const Point2 lead_stale{
    stale_command.first - pose_after.first, stale_command.second - pose_after.second};
  const double step = std::hypot(
    lead_stale.first - lead_before.first, lead_stale.second - lead_before.second);
  EXPECT_GT(step, 0.25);
}

TEST(PathCommandLimiter, ReexpressPreservesArcLengthSpeedAndGeometry)
{
  auto limiter = cruising_limiter();
  const auto snapshot = limiter.snapshot();
  const double length_before = limiter.path()->length();

  limiter.reexpress(kBeforeClosure, kAfterClosure);

  // A rigid transform changes neither how far along the route the command is
  // nor how fast it is going, so the speed machinery carries on uninterrupted.
  EXPECT_DOUBLE_EQ(limiter.snapshot().progress, snapshot.progress);
  EXPECT_DOUBLE_EQ(limiter.snapshot().speed, snapshot.speed);
  EXPECT_NEAR(limiter.path()->length(), length_before, 1e-12);
  EXPECT_NEAR(
    std::hypot(limiter.velocity().first, limiter.velocity().second),
    std::hypot(snapshot.velocity.first, snapshot.velocity.second), 1e-12);
  // The command point stays on its own path.
  EXPECT_NEAR(limiter.path()->project(*limiter.position()).cross_track, 0.0, 1e-9);
}

TEST(PathCommandLimiter, ReexpressIsANoOpForAnUnchangedCorrectionAndRoundTrips)
{
  auto limiter = cruising_limiter();
  const Point2 command = *limiter.position();

  limiter.reexpress(kBeforeClosure, kBeforeClosure);
  EXPECT_DOUBLE_EQ(limiter.position()->first, command.first);
  EXPECT_DOUBLE_EQ(limiter.position()->second, command.second);

  limiter.reexpress(kBeforeClosure, kAfterClosure);
  limiter.reexpress(kAfterClosure, kBeforeClosure);
  EXPECT_NEAR(limiter.position()->first, command.first, 1e-12);
  EXPECT_NEAR(limiter.position()->second, command.second, 1e-12);
}

TEST(PathCommandLimiter, ReexpressRejectsANonFiniteCorrection)
{
  auto limiter = cruising_limiter();
  const px4_vio_bridge::Correction4 broken{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0};
  EXPECT_THROW(limiter.reexpress(kBeforeClosure, broken), std::invalid_argument);
}

// --- corner fillets (experimental) -----------------------------------------

namespace
{
PathCommandLimiter fillet_limiter()
{
  return PathCommandLimiter(
    0.20, 0.30, 0.10, 0.05, 0.30, 0.20, 0.01, true, 0.05, true, true);
}

const PathCommandLimiter::FilletCheck kAcceptFillet =
  [](const std::vector<Point2> &, const std::vector<Point2> &) {return true;};
const PathCommandLimiter::FilletCheck kRejectFillet =
  [](const std::vector<Point2> &, const std::vector<Point2> &) {return false;};

struct FilletRun
{
  bool stopped_at_a_vertex{false};
  bool reached_end{false};
  double slowest_before_end{1.0e9};
  double worst_step{0.0};        ///< largest |dv| / (a * dt)
  double worst_secant_error{0.0};
  double farthest_from_corner{0.0};
};

// Fly a path to its end with the vehicle exactly on the command, checking the
// strict invariants on every tick.
FilletRun fly_filleted(
  PathCommandLimiter & limiter, const std::vector<Point2> & path, bool jitter)
{
  FilletRun run;
  const Polyline sharp(path);
  Point2 previous_velocity = limiter.velocity();
  Point2 previous_position = *limiter.position();
  for (int tick = 0; tick < 2000; ++tick) {
    // The adapter's dt is fixed; allow the modest jitter the arc speed is sized
    // for (a tick up to 1.5x the next).
    const double dt = jitter ? 0.04 + 0.005 * ((37 * tick + 11) % 5) : 0.05;
    limiter.update(path.back(), dt, true, limiter.position());
    const auto velocity = limiter.velocity();
    run.worst_step = std::max(
      run.worst_step,
      std::hypot(velocity.first - previous_velocity.first,
        velocity.second - previous_velocity.second) / (0.30 * dt));
    run.worst_secant_error = std::max(
      {run.worst_secant_error,
        std::abs((limiter.position()->first - previous_position.first) / dt - velocity.first),
        std::abs((limiter.position()->second - previous_position.second) / dt - velocity.second)});
    run.farthest_from_corner = std::max(
      run.farthest_from_corner, sharp.project(*limiter.position()).cross_track);
    if (limiter.waiting_vertex()) {
      run.stopped_at_a_vertex = true;
    }
    const double progress = limiter.snapshot().progress;
    const double speed = std::hypot(velocity.first, velocity.second);
    // Once off the starting ramp and before the end brake.
    if (progress > 0.15 && progress < limiter.path()->length() - 0.15) {
      run.slowest_before_end = std::min(run.slowest_before_end, speed);
    }
    previous_velocity = velocity;
    previous_position = *limiter.position();
    if (progress >= limiter.path()->length() - 1.0e-8 && speed <= 1.0e-8) {
      run.reached_end = true;
      break;
    }
  }
  return run;
}
}  // namespace

TEST(PathCommandLimiter, CornerFilletsRequireStrictMode)
{
  EXPECT_THROW(
    PathCommandLimiter(0.20, 0.30, 0.10, 0.05, 0.30, 0.20, 0.01, true, 0.05, false, true),
    std::invalid_argument);
}

TEST(PathCommandLimiter, FilletCarriesSpeedThroughBendsInsideEveryStrictBound)
{
  // A right angle has to slow (the arc radius is 0.12 m); a 30 degree bend
  // with room for its arc does not slow at all.
  struct Case
  {
    std::vector<Point2> path;
    double minimum_speed;
  };
  for (const auto & [path, minimum_speed] : {
      Case{{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, 0.12},
      Case{{{0.0, 0.0}, {1.0, 0.0}, {1.0 + 0.866, 0.5}}, 0.19},
      Case{{{0.0, 0.0}, {0.6, 0.0}, {0.9, -0.4}, {1.5, -0.4}, {1.5, 0.3}}, 0.10}})
  {
    for (const bool jitter : {false, true}) {
      auto limiter = fillet_limiter();
      ASSERT_TRUE(limiter.set_path(path, {0.0, 0.0}, nullptr, kAcceptFillet));
      EXPECT_TRUE(limiter.installation()->bends.empty());
      FilletRun run;
      ASSERT_NO_THROW(run = fly_filleted(limiter, path, jitter));
      EXPECT_FALSE(run.stopped_at_a_vertex);
      EXPECT_TRUE(run.reached_end);
      EXPECT_LE(run.worst_step, 1.0 + 1.0e-6);
      EXPECT_LE(run.worst_secant_error, 1.0e-8);
      // The command never leaves the planner's corner by more than the
      // junction deviation.
      EXPECT_LE(run.farthest_from_corner, 0.05 + 1.0e-6);
      if (!jitter) {
        EXPECT_GE(run.slowest_before_end, minimum_speed);
      }
    }
  }
}

TEST(PathCommandLimiter, RejectedOrMissingFilletCheckKeepsTheStrictStop)
{
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  for (const auto & check : {kRejectFillet, PathCommandLimiter::FilletCheck{}}) {
    auto limiter = fillet_limiter();
    ASSERT_TRUE(limiter.set_path(path, {0.0, 0.0}, nullptr, check));
    ASSERT_EQ(limiter.installation()->bends.size(), 1u);
    EXPECT_EQ(limiter.path()->points().size(), 3u);
    for (int tick = 0; tick < 400; ++tick) {
      ASSERT_NO_THROW(limiter.update(path.back(), 0.05, true, Point2{0.0, 0.0}));
    }
    // The vehicle reference never reaches the vertex, so the command waits there.
    ASSERT_TRUE(limiter.waiting_vertex().has_value());
    EXPECT_NEAR(limiter.position()->first, 1.0, 1e-9);
    EXPECT_NEAR(limiter.position()->second, 0.0, 1e-9);
  }
}

TEST(PathCommandLimiter, FilletCheckSeesTheArcAndTheCornerItReplaces)
{
  std::vector<Point2> seen_arc;
  std::vector<Point2> seen_corner;
  const PathCommandLimiter::FilletCheck record =
    [&](const std::vector<Point2> & arc, const std::vector<Point2> & corner) {
      seen_arc = arc;
      seen_corner = corner;
      return true;
    };
  auto limiter = fillet_limiter();
  ASSERT_TRUE(limiter.set_path({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, {0.0, 0.0}, nullptr, record));
  ASSERT_EQ(seen_corner.size(), 3u);
  EXPECT_NEAR(seen_corner[1].first, 1.0, 1e-12);
  EXPECT_NEAR(seen_corner[1].second, 0.0, 1e-12);
  // The arc starts and ends on the legs, at the corner's tangent points.
  ASSERT_GE(seen_arc.size(), 3u);
  EXPECT_NEAR(seen_arc.front().first, seen_corner.front().first, 1e-12);
  EXPECT_NEAR(seen_arc.back().second, seen_corner.back().second, 1e-12);
  // Its midpoint is the junction deviation inside the vertex.
  const auto middle = seen_arc[seen_arc.size() / 2];
  EXPECT_NEAR(std::hypot(middle.first - 1.0, middle.second), 0.05, 2e-3);
}

TEST(PathCommandLimiter, FilletShrinksWhenTheFullArcIsRejected)
{
  // Only an arc within 3 cm of the vertex is acceptable: the first 5 cm trial
  // is rejected and the 2.5 cm one is installed rather than keeping the stop.
  const PathCommandLimiter::FilletCheck near_vertex =
    [](const std::vector<Point2> & arc, const std::vector<Point2> & corner) {
      const auto middle = arc[arc.size() / 2];
      return std::hypot(middle.first - corner[1].first, middle.second - corner[1].second) < 0.03;
    };
  auto limiter = fillet_limiter();
  ASSERT_TRUE(limiter.set_path(
      {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, {0.0, 0.0}, nullptr, near_vertex));
  EXPECT_TRUE(limiter.installation()->bends.empty());
  const auto & points = limiter.path()->points();
  ASSERT_GT(points.size(), 3u);
  // The installed arc's midpoint is the halved deviation inside the vertex.
  const auto middle = points[points.size() / 2];
  EXPECT_NEAR(std::hypot(middle.first - 1.0, middle.second), 0.025, 2e-3);
}

TEST(PathCommandLimiter, FilletLeavesAHairpinAsAStop)
{
  auto limiter = fillet_limiter();
  ASSERT_TRUE(limiter.set_path(
      {{0.0, 0.0}, {1.0, 0.0}, {0.0, 0.05}}, {0.0, 0.0}, nullptr, kAcceptFillet));
  EXPECT_EQ(limiter.installation()->bends.size(), 1u);
}

TEST(PathCommandLimiter, FilletBrakesOnTheArcAndResumesWithinTheBound)
{
  const std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  auto limiter = fillet_limiter();
  ASSERT_TRUE(limiter.set_path(path, {0.0, 0.0}, nullptr, kAcceptFillet));
  Point2 previous_velocity{0.0, 0.0};
  Point2 previous_position = *limiter.position();
  bool paused_on_arc = false;
  for (int tick = 0; tick < 1500; ++tick) {
    const auto position = *limiter.position();
    // Pause translation (yaw alignment) and then cap it (ride-out) while the
    // command is on the arc.
    const bool on_arc = position.first > 0.9 && position.second < 0.1 && position.second > 0.0;
    const bool advance = !(on_arc && tick % 60 < 25);
    const std::optional<double> cap = on_arc && tick % 60 >= 40 ?
      std::optional<double>{0.06} : std::nullopt;
    paused_on_arc = paused_on_arc || !advance;
    ASSERT_NO_THROW(limiter.update(path.back(), 0.05, advance, limiter.position(), cap));
    const auto velocity = limiter.velocity();
    EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
      velocity.second - previous_velocity.second), 0.30 * 0.05 + 1.0e-8);
    EXPECT_NEAR((limiter.position()->first - previous_position.first) / 0.05,
      velocity.first, 1.0e-8);
    EXPECT_NEAR((limiter.position()->second - previous_position.second) / 0.05,
      velocity.second, 1.0e-8);
    previous_velocity = velocity;
    previous_position = *limiter.position();
    if (limiter.snapshot().progress >= limiter.path()->length() - 1.0e-8 &&
      std::hypot(velocity.first, velocity.second) <= 1.0e-8)
    {
      break;
    }
  }
  EXPECT_TRUE(paused_on_arc);
  EXPECT_GE(limiter.snapshot().progress, limiter.path()->length() - 1.0e-8);
}

TEST(PathCommandLimiter, FilletReplacementsAndReexpressKeepTheArc)
{
  std::vector<Point2> path{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  auto limiter = fillet_limiter();
  ASSERT_TRUE(limiter.set_path(path, {0.0, 0.0}, nullptr, kAcceptFillet));
  const auto soft_before = limiter.installation()->soft.size();
  ASSERT_GT(soft_before, 10u);
  // The same planner points again are recognised even though the installed
  // geometry is rounded.
  EXPECT_FALSE(limiter.set_path(path, {0.0, 0.0}, nullptr, kAcceptFillet));

  Point2 previous_velocity{0.0, 0.0};
  for (int tick = 0; tick < 1500; ++tick) {
    if (tick == 60 || tick == 110) {
      const double shift = tick == 60 ? 0.003 : -0.002;
      path = {{0.0, shift}, {1.0, shift}, {1.0, 1.0 + shift}};
      ASSERT_TRUE(limiter.set_path(path, *limiter.position(), nullptr, kAcceptFillet));
    }
    if (tick == 90) {
      limiter.reexpress(kBeforeClosure, kAfterClosure);
      EXPECT_EQ(limiter.installation()->soft.size(), soft_before);
      EXPECT_TRUE(limiter.installation()->bends.empty());
      limiter.reexpress(kAfterClosure, kBeforeClosure);
      previous_velocity = limiter.velocity();
    }
    ASSERT_NO_THROW(limiter.update(path.back(), 0.05, true, limiter.position()));
    const auto velocity = limiter.velocity();
    EXPECT_LE(std::hypot(velocity.first - previous_velocity.first,
      velocity.second - previous_velocity.second), 0.30 * 0.05 + 1.0e-8);
    previous_velocity = velocity;
    EXPECT_FALSE(limiter.waiting_vertex().has_value());
    if (limiter.snapshot().progress >= limiter.path()->length() - 1.0e-8 &&
      std::hypot(velocity.first, velocity.second) <= 1.0e-8)
    {
      break;
    }
  }
  EXPECT_GE(limiter.snapshot().progress, limiter.path()->length() - 1.0e-8);
}
