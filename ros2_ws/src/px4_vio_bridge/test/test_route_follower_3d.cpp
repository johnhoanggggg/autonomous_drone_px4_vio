#include <gtest/gtest.h>

#include <cmath>

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
