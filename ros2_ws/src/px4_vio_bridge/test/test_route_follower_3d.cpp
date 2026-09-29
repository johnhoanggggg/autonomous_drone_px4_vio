#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "px4_vio_bridge/route_follower_3d.hpp"

namespace p = px4_vio_bridge;

TEST(RouteFollower3D, TiltedMapLimitsRespectContinuousFrameSpeedAndAcceleration)
{
  const p::Follower3DConfig original;
  for (const double angle : {0.0, 0.1073, 0.5, 1.5707963267948966, 3.141592653589793}) {
    const double c = std::cos(angle), s = std::sin(angle);
    const auto adjusted = p::limits_for_map_tilt(original, c);
    for (int i = 0; i < 32; ++i) {
      for (const double sign : {-1.0, 1.0}) {
        const double theta = i * 6.283185307179586 / 32;
        for (const bool acceleration : {false, true}) {
          const double h = acceleration ? adjusted.max_horizontal_acceleration : adjusted.max_horizontal_speed;
          const double v = acceleration ? adjusted.max_vertical_acceleration : adjusted.max_vertical_speed;
          const double x = h * std::cos(theta), y = h * std::sin(theta), z = sign * v;
          const double raw_x = c * x - s * z, raw_y = y, raw_z = s * x + c * z;
          EXPECT_LE(std::hypot(raw_x, raw_y), (acceleration ? original.max_horizontal_acceleration :
            original.max_horizontal_speed) + 1e-12);
          EXPECT_LE(std::abs(raw_z), (acceleration ? original.max_vertical_acceleration :
            original.max_vertical_speed) + 1e-12);
        }
      }
    }
  }
}

TEST(PathGeometry3D, ProjectionAndArcLengthUseXYZ)
{
  p::Polyline3D path({{0.0, 0.0, 0.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 1.0}});
  EXPECT_NEAR(path.length(), std::sqrt(2.0) + 1.0, 1.0e-12);
  const auto projection = path.project({0.5, 0.2, 0.5});
  EXPECT_NEAR(projection.point.x, 0.5, 1.0e-12);
  EXPECT_NEAR(projection.point.z, 0.5, 1.0e-12);
  EXPECT_NEAR(projection.horizontal_distance, 0.2, 1.0e-12);
  EXPECT_NEAR(projection.vertical_distance, 0.0, 1.0e-12);
}

TEST(RouteFollower3D, LimitsHorizontalAndVerticalVelocitySeparately)
{
  p::Follower3DConfig config;
  config.lookahead = 1.0;
  config.max_horizontal_speed = 0.20;
  config.max_vertical_speed = 0.05;
  config.max_horizontal_acceleration = 1.0;
  config.max_vertical_acceleration = 1.0;
  config.max_cross_track = 0.20;
  config.max_vertical_track = 0.20;
  p::RouteFollower3D follower(config);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {2.0, 0.0, 1.5}}, {0.0, 0.0, 0.5}));
  const auto result = follower.update(
    {0.0, 0.0, 0.5}, 0.1, [](const auto &, const auto &) {return true;});
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LE(std::hypot(result.velocity.x, result.velocity.y), 0.20 + 1.0e-12);
  EXPECT_LE(std::abs(result.velocity.z), 0.05 + 1.0e-12);
  EXPECT_GT(result.displacement.x, 0.0);
  EXPECT_GT(result.displacement.z, 0.0);
}

TEST(RouteFollower3D, RejectsHorizontalAndVerticalTrackingFaults)
{
  p::Follower3DConfig config;
  config.max_cross_track = 0.10;
  config.max_vertical_track = 0.05;
  p::RouteFollower3D follower(config);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  EXPECT_EQ(
    follower.update({0.0, 0.11, 0.5}, 0.1, [](const auto &, const auto &) {return true;}).reason,
    "CROSS_TRACK");
  EXPECT_EQ(
    follower.update({0.0, 0.0, 0.56}, 0.1, [](const auto &, const auto &) {return true;}).reason,
    "VERTICAL_TRACK");
}

TEST(RouteFollower3D, ValidatesLookaheadAndCommandChords)
{
  p::Follower3DConfig config;
  config.max_cross_track = 0.20;
  config.max_vertical_track = 0.20;
  p::RouteFollower3D follower(config);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  int calls = 0;
  const auto lookahead_rejected = follower.update(
    {0.0, 0.0, 0.5}, 0.1,
    [&calls](const auto &, const auto &) {++calls; return false;});
  EXPECT_FALSE(lookahead_rejected.valid);
  EXPECT_EQ(lookahead_rejected.reason, "LOOKAHEAD_CHORD_BLOCKED");
  EXPECT_EQ(calls, 1);

  calls = 0;
  const auto carrot_rejected = follower.update(
    {0.0, 0.0, 0.5}, 0.1,
    [&calls](const auto &, const auto &) {return ++calls == 1;});
  EXPECT_FALSE(carrot_rejected.valid);
  EXPECT_EQ(carrot_rejected.reason, "CARROT_CHORD_BLOCKED");
  EXPECT_EQ(calls, 2);
}

TEST(RouteFollower3D, RouteAheadChordIgnoresLateralErrorButTurnsBeforeCorners)
{
  p::Follower3DConfig config;
  config.lookahead = 0.25;
  config.max_cross_track = 0.20;
  config.max_vertical_track = 0.20;
  const auto clear = [](const auto &, const auto &) {return true;};
  p::RouteFollower3D straight(config);
  ASSERT_TRUE(straight.set_path({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}}, {0.5, 0.10, 0.5}));
  const auto offset = straight.update({0.5, 0.10, 0.5}, 0.1, clear);
  ASSERT_TRUE(offset.valid);
  // 10 cm left of a straight route: the chord stays exactly along the route.
  EXPECT_NEAR(offset.route_ahead.x - offset.projection.point.x, 0.25, 1e-9);
  EXPECT_NEAR(offset.route_ahead.y - offset.projection.point.y, 0.0, 1e-9);

  p::RouteFollower3D corner(config);
  ASSERT_TRUE(corner.set_path({{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}, {1.0, 1.0, 0.5}}, {0.9, 0.0, 0.5}));
  const auto turning = corner.update({0.9, 0.0, 0.5}, 0.1, clear);
  ASSERT_TRUE(turning.valid);
  // 0.10 m before the corner the chord already bends onto the next leg.
  EXPECT_NEAR(turning.route_ahead.x, 1.0, 1e-9);
  EXPECT_NEAR(turning.route_ahead.y, 0.15, 1e-9);
}

TEST(RouteFollower3D, LongerHeadingChordDilutesAShortSidewaysConnector)
{
  p::Follower3DConfig config;
  config.lookahead = 0.25;
  config.max_cross_track = 0.20;
  config.max_vertical_track = 0.20;
  const auto clear = [](const auto &, const auto &) {return true;};
  // A recovery route: a 6 cm sideways connector, then straight along +x.
  const std::vector<p::Point3> route{{0.0, 0.0, 0.5}, {0.0, 0.06, 0.5}, {2.0, 0.06, 0.5}};
  const auto heading = [](const p::FollowResult3D & r) {
      return std::atan2(r.route_ahead.y - r.projection.point.y,
        r.route_ahead.x - r.projection.point.x) * 180.0 / M_PI;
    };
  p::RouteFollower3D short_chord(config);
  ASSERT_TRUE(short_chord.set_path(route, {0.0, 0.0, 0.5}));
  const auto short_result = short_chord.update({0.0, 0.0, 0.5}, 0.1, clear);
  ASSERT_TRUE(short_result.valid);
  config.heading_lookahead = 0.60;
  p::RouteFollower3D long_chord(config);
  ASSERT_TRUE(long_chord.set_path(route, {0.0, 0.0, 0.5}));
  const auto long_result = long_chord.update({0.0, 0.0, 0.5}, 0.1, clear);
  ASSERT_TRUE(long_result.valid);
  EXPECT_GT(heading(short_result), 15.0);  // 0.25 m chord: ~17 deg off the route
  EXPECT_LT(heading(long_result), 7.0);    // 0.60 m chord: ~6 deg, inside a 10 deg deadband
}

TEST(RouteFollower3D, ArrivalUsesActualXYZEndpoint)
{
  p::Follower3DConfig config;
  config.max_cross_track = 0.20;
  config.max_vertical_track = 0.20;
  p::RouteFollower3D follower(config);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {0.05, 0.0, 0.55}}, {0.0, 0.0, 0.5}));
  const auto result = follower.update(
    {0.05, 0.0, 0.55}, 0.1, [](const auto &, const auto &) {return true;});
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.reached);
}

TEST(RouteFollower3D, BrakesAtShortEndpointWithoutAccelerationSpike)
{
  const p::Follower3DConfig config;
  for (const double sign : {-1.0, 1.0}) {
    p::RouteFollower3D follower(config);
    const p::Point3 end{sign * 0.11, sign * 0.035, sign * 0.065};
    ASSERT_TRUE(follower.set_path({{}, end}, {}));
    p::Point3 previous_velocity{}, previous_carrot{};
    for (int i = 0; i < 300; ++i) {
      // Variable timer intervals must preserve the braking guarantee.
      const double dt = i % 3 == 0 ? 0.03 : (i % 3 == 1 ? 0.07 : 0.05);
      const auto r = follower.update({}, dt, [](const auto &, const auto &) {return true;});
      ASSERT_TRUE(r.valid) << r.reason;
      EXPECT_LE(std::hypot(r.velocity.x, r.velocity.y), config.max_horizontal_speed + 1e-9);
      EXPECT_LE(std::abs(r.velocity.z), config.max_vertical_speed + 1e-9);
      EXPECT_LE(std::hypot(r.acceleration.x, r.acceleration.y),
        config.max_horizontal_acceleration + 1e-9);
      EXPECT_LE(std::abs(r.acceleration.z), config.max_vertical_acceleration + 1e-9);
      EXPECT_NEAR((r.velocity.z - previous_velocity.z) / dt, r.acceleration.z, 1e-9);
      EXPECT_NEAR((r.carrot.z - previous_carrot.z) / dt, r.velocity.z, 1e-9);
      EXPECT_GE(sign * r.carrot.x, -1e-9);
      EXPECT_LE(sign * r.carrot.x, sign * end.x + 1e-9);
      EXPECT_LE(sign * r.carrot.y, sign * end.y + 1e-9);
      EXPECT_LE(sign * r.carrot.z, sign * end.z + 1e-9);
      previous_velocity = r.velocity; previous_carrot = r.carrot;
    }
    EXPECT_NEAR(previous_carrot.x, end.x, 1e-6);
    EXPECT_NEAR(previous_carrot.y, end.y, 1e-6);
    EXPECT_NEAR(previous_carrot.z, end.z, 1e-6);
    EXPECT_NEAR(p::distance_3d(previous_velocity, {}), 0.0, 1e-6);
  }
}

TEST(RouteFollower3D, EquivalentReplacementPreservesVelocityAndUsesVerifiedRejoin)
{
  p::Follower3DConfig config;
  config.lookahead = 0.50;
  config.max_horizontal_speed = 0.20;
  config.max_horizontal_acceleration = 0.50;
  config.max_cross_track = 0.20;
  p::RouteFollower3D follower(config);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  p::FollowResult3D before;
  for (int i = 0; i < 8; ++i) {
    before = follower.update({0.05, 0.0, 0.5}, 0.1,
      [](const auto &, const auto &) {return true;});
    ASSERT_TRUE(before.valid) << before.reason;
  }
  ASSERT_GT(before.velocity.x, 0.10);
  int rejoin_checks = 0;
  ASSERT_TRUE(follower.replace_equivalent_path(
    {{0.05, 0.02, 0.5}, {2.0, 0.02, 0.5}}, {0.05, 0.0, 0.5}, 0.10,
    [&rejoin_checks](const auto &, const auto &) {++rejoin_checks; return true;}));
  EXPECT_EQ(rejoin_checks, 1);
  EXPECT_NEAR(follower.velocity().x, before.velocity.x, 1.0e-12);
  const auto after = follower.update({0.05, 0.0, 0.5}, 0.1,
    [](const auto &, const auto &) {return true;});
  ASSERT_TRUE(after.valid) << after.reason;
  EXPECT_GT(after.velocity.x, 0.10);
}

TEST(RouteFollower3D, ReplacementRejectsDifferentRouteOrBlockedRejoin)
{
  p::Follower3DConfig config;
  config.max_cross_track = 0.20;
  p::RouteFollower3D follower(config);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {2.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  EXPECT_FALSE(follower.replace_equivalent_path(
    {{0.0, 0.25, 0.5}, {2.0, 0.25, 0.5}}, {0.0, 0.0, 0.5}, 0.10,
    [](const auto &, const auto &) {return true;}));
  EXPECT_FALSE(follower.replace_equivalent_path(
    {{0.0, 0.02, 0.5}, {2.0, 0.02, 0.5}}, {0.0, 0.0, 0.5}, 0.10,
    [](const auto &, const auto &) {return false;}));
  const auto unchanged = follower.update({0.0, 0.0, 0.5}, 0.1,
    [](const auto &, const auto &) {return true;});
  EXPECT_TRUE(unchanged.valid);
  EXPECT_NEAR(unchanged.lookahead.y, 0.0, 1.0e-12);
}

namespace
{
p::Follower3DConfig hold_line_config()
{
  p::Follower3DConfig config;
  config.stop_at_vertices = true;  // as flown by the 3D adapter
  config.max_cross_track = 0.15;
  config.max_vertical_track = 0.15;
  config.hold_line_skip = 0.35;
  config.hold_line_max_offset = 0.10;
  return config;
}
const auto always_clear = [](const p::Point3 &, const p::Point3 &) {return true;};
}  // namespace

TEST(RouteFollower3D, HoldLineKeepsRouteWhenReplanOnlyRestartsAtTheDrone)
{
  p::RouteFollower3D follower(hold_line_config());
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  for (int i = 0; i < 20; ++i) {ASSERT_TRUE(follower.update({0.1 * i / 20, 0.0, 0.5}, 0.1, always_clear).valid);}
  // The drone drifted 8 cm left; the planner's replan starts there and rejoins.
  const p::Point3 drone{0.25, 0.08, 0.5};
  const std::vector<p::Point3> replan{drone, {0.30, 0.07, 0.5}, {0.9, 0.0, 0.5}, {3.0, 0.0, 0.5}};
  int checks = 0;
  ASSERT_TRUE(follower.hold_equivalent_path(replan, drone, 0.10,
    [&checks](const auto &, const auto &) {++checks; return true;}));
  EXPECT_GE(checks, 1);  // the held route was re-validated
  const auto held = follower.update(drone, 0.1, always_clear);
  ASSERT_TRUE(held.valid) << held.reason;
  // Command and route stay on the original line, not on the drifted start.
  EXPECT_NEAR(held.carrot.y, 0.0, 1e-9);
  EXPECT_NEAR(held.projection.point.y, 0.0, 1e-9);
}

TEST(RouteFollower3D, HoldLineAdoptsTheReplanWhenFarOffDifferentOrBlocked)
{
  p::RouteFollower3D follower(hold_line_config());
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  // Drone beyond hold_line_max_offset: take the replan.
  EXPECT_FALSE(follower.hold_equivalent_path(
    {{0.3, 0.14, 0.5}, {0.9, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.3, 0.14, 0.5}, 0.10, always_clear));
  // A genuinely different route (around something new): take it.
  EXPECT_FALSE(follower.hold_equivalent_path(
    {{0.3, 0.02, 0.5}, {1.0, 0.4, 0.5}, {3.0, 0.0, 0.5}}, {0.3, 0.02, 0.5}, 0.10, always_clear));
  // The held route no longer clears the map: take the replan.
  EXPECT_FALSE(follower.hold_equivalent_path(
    {{0.3, 0.02, 0.5}, {3.0, 0.0, 0.5}}, {0.3, 0.02, 0.5}, 0.10,
    [](const auto &, const auto &) {return false;}));
  // Disabled by default.
  p::RouteFollower3D plain;
  ASSERT_TRUE(plain.set_path({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  EXPECT_FALSE(plain.hold_equivalent_path(
    {{0.3, 0.02, 0.5}, {3.0, 0.0, 0.5}}, {0.3, 0.02, 0.5}, 0.10, always_clear));
}

TEST(RouteFollower3D, CrossTrackCorrectionPullsBackProportionallyAndCapped)
{
  auto config = hold_line_config();
  config.cross_track_gain = 0.5;
  config.cross_track_correction_max = 0.04;
  p::RouteFollower3D follower(config);
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  // 6 cm left and 2 cm high -> 0.5 x (0.06, 0.02) = 3.2 cm toward the line, under the cap.
  const auto small = follower.update({0.05, 0.06, 0.52}, 0.1, always_clear);
  ASSERT_TRUE(small.valid) << small.reason;
  EXPECT_NEAR(small.correction.x, 0.0, 1e-9);  // no along-track component
  EXPECT_NEAR(small.correction.y, -0.03, 1e-9);
  EXPECT_NEAR(small.correction.z, -0.01, 1e-9);
  EXPECT_NEAR(small.displacement.y, small.carrot.y + small.correction.y - 0.06, 1e-9);
  // 12 cm off -> 6 cm wanted, capped at 4 cm.
  const auto big = follower.update({0.06, 0.12, 0.5}, 0.1, always_clear);
  ASSERT_TRUE(big.valid) << big.reason;
  EXPECT_NEAR(std::hypot(big.correction.y, big.correction.z), 0.04, 1e-9);
  // A blocked corrected command falls back to the plain carrot.
  int calls = 0;
  const auto blocked = follower.update({0.07, 0.06, 0.5}, 0.1, [&calls](const auto &, const auto &) {
      return ++calls < 3;  // lookahead + carrot chords pass, corrected chord fails
    });
  ASSERT_TRUE(blocked.valid) << blocked.reason;
  EXPECT_NEAR(blocked.correction.y, 0.0, 1e-12);
}

namespace
{
p::Follower3DConfig carrot_config()
{
  auto config = hold_line_config();
  config.path_carrot = true;
  config.max_horizontal_speed = 0.15;
  config.max_vertical_speed = 0.10;
  config.max_cross_track = 0.30;
  config.max_vertical_track = 0.30;
  return config;
}
// Drone tracks the command one tick late; returns the speed profile.
std::vector<double> fly(p::RouteFollower3D & follower, p::Point3 pose, int ticks, bool * reached = nullptr)
{
  std::vector<double> speeds;
  for (int i = 0; i < ticks; ++i) {
    const auto r = follower.update(pose, 0.1, always_clear);
    EXPECT_TRUE(r.valid) << r.reason;
    if (!r.valid) {break;}
    speeds.push_back(std::hypot(std::hypot(r.velocity.x, r.velocity.y), r.velocity.z));
    if (reached && r.reached) {*reached = true;}
    pose = r.carrot;
  }
  return speeds;
}
}  // namespace

TEST(RouteFollower3D, PathCarrotSlowsForCornersWithoutStopping)
{
  p::RouteFollower3D follower(carrot_config());
  // 1 m east, then a 45 degree turn, then 1 m north-east.
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}, {1.7071, 0.7071, 0.5}}, {0.0, 0.0, 0.5}));
  bool reached = false;
  const auto speeds = fly(follower, {0.0, 0.0, 0.5}, 250, &reached);
  EXPECT_TRUE(reached);
  // Cruise at the limit, never stop before the end, and cross the corner no faster than
  // 0.15 cos 45 (the discrete braking bound undershoots it by up to one tick of braking).
  const auto peak = *std::max_element(speeds.begin(), speeds.end());
  EXPECT_NEAR(peak, 0.15, 1e-6);
  std::size_t first = 0; while (first < speeds.size() && speeds[first] < 0.14) {++first;}
  std::size_t last = speeds.size() - 1; while (last > 0 && speeds[last] < 0.14) {--last;}
  ASSERT_LT(first, last);
  const auto dip = *std::min_element(speeds.begin() + first, speeds.begin() + last);
  EXPECT_GT(dip, 0.15 * std::cos(M_PI / 4) - 0.3 * 0.1);
  EXPECT_LE(dip, 0.15 * std::cos(M_PI / 4) + 1e-6);
  // The same route with vertex stops comes to rest at the corner.
  auto stops = carrot_config();
  stops.path_carrot = false;
  p::RouteFollower3D stopper(stops);
  ASSERT_TRUE(stopper.set_path({{0.0, 0.0, 0.5}, {1.0, 0.0, 0.5}, {1.7071, 0.7071, 0.5}}, {0.0, 0.0, 0.5}));
  const auto stop_speeds = fly(stopper, {0.0, 0.0, 0.5}, 250);
  std::size_t moving = 0; while (moving < stop_speeds.size() && stop_speeds[moving] < 0.05) {++moving;}
  EXPECT_LT(*std::min_element(stop_speeds.begin() + moving, stop_speeds.begin() + moving + 100), 1e-3);
}

TEST(RouteFollower3D, PathCarrotRespectsAccelerationAndVerticalLimits)
{
  p::RouteFollower3D follower(carrot_config());
  // 45 degree climb: the vertical limit (0.10) caps the along speed at 0.1414.
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.3}, {1.0, 0.0, 1.3}}, {0.0, 0.0, 0.3}));
  const auto speeds = fly(follower, {0.0, 0.0, 0.3}, 200);
  const double along_accel = 0.2 * std::sqrt(2.0);  // vertical 0.2 m/s2 along a 45 degree climb
  for (std::size_t i = 1; i < speeds.size(); ++i) {
    EXPECT_LE(std::abs(speeds[i] - speeds[i - 1]) / 0.1, along_accel + 1e-6) << i;
    EXPECT_LE(speeds[i], 0.10 * std::sqrt(2.0) + 1e-6);
  }
  EXPECT_NEAR(*std::max_element(speeds.begin(), speeds.end()), 0.10 * std::sqrt(2.0), 1e-6);
  EXPECT_NEAR(speeds.back(), 0.0, 1e-3);  // stops at the end
}

TEST(RouteFollower3D, PathCarrotNeverTrailsAndLimitsItsLead)
{
  p::RouteFollower3D follower(carrot_config());
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  // The drone does not move: the command stops within max_carrot_lead ahead.
  p::FollowResult3D r;
  for (int i = 0; i < 100; ++i) {r = follower.update({0.0, 0.0, 0.5}, 0.1, always_clear);}
  ASSERT_TRUE(r.valid) << r.reason;
  EXPECT_LE(r.carrot.x, 0.35 + 1e-9);
  EXPECT_GT(r.carrot.x, 0.2);
  EXPECT_NEAR(std::hypot(r.velocity.x, r.velocity.y), 0.0, 0.02);
  // The drone is pushed well past the command: it restarts from the drone, not behind it.
  r = follower.update({1.0, 0.05, 0.5}, 0.1, always_clear);
  ASSERT_TRUE(r.valid) << r.reason;
  EXPECT_GE(r.carrot.x, 1.0);
  EXPECT_NEAR(r.carrot.y, 0.0, 1e-9);  // the command stays on the route
}

TEST(RouteFollower3D, PathCarrotSeedKeepsSpeedAcrossANewRoute)
{
  p::RouteFollower3D follower(carrot_config());
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  follower.seed_velocity({0.15, 0.02, 0.0});
  const auto r = follower.update({0.0, 0.0, 0.5}, 0.1, always_clear);
  ASSERT_TRUE(r.valid) << r.reason;
  EXPECT_NEAR(r.velocity.x, 0.15, 1e-6);  // along-route part kept, no restart from zero
  EXPECT_NEAR(r.velocity.y, 0.0, 1e-12);
  // Seeding against the route direction does not command a reverse.
  p::RouteFollower3D reverse(carrot_config());
  ASSERT_TRUE(reverse.set_path({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  reverse.seed_velocity({-0.15, 0.0, 0.0});
  const auto back = reverse.update({0.0, 0.0, 0.5}, 0.1, always_clear);
  ASSERT_TRUE(back.valid);
  EXPECT_GE(back.velocity.x, 0.0);
  EXPECT_LE(back.velocity.x, 0.03 + 1e-9);
}

TEST(RouteFollower3D, PathCarrotHoldLineKeepsTheCommandMoving)
{
  p::RouteFollower3D follower(carrot_config());
  ASSERT_TRUE(follower.set_path({{0.0, 0.0, 0.5}, {3.0, 0.0, 0.5}}, {0.0, 0.0, 0.5}));
  const auto speeds = fly(follower, {0.0, 0.0, 0.5}, 20);
  ASSERT_GT(speeds.back(), 0.14);
  const p::Point3 drone{0.25, 0.06, 0.5};
  ASSERT_TRUE(follower.hold_equivalent_path(
    {drone, {0.35, 0.03, 0.5}, {0.9, 0.0, 0.5}, {3.0, 0.0, 0.5}}, drone, 0.10, always_clear));
  const auto r = follower.update(drone, 0.1, always_clear);
  ASSERT_TRUE(r.valid) << r.reason;
  EXPECT_NEAR(r.carrot.y, 0.0, 1e-9);
  EXPECT_GT(r.velocity.x, 0.13);  // no restart from standstill
}
