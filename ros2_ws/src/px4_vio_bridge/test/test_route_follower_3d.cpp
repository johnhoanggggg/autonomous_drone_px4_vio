#include <gtest/gtest.h>

#include <cmath>

#include "px4_vio_bridge/route_follower_3d.hpp"

namespace p = px4_vio_bridge;

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
