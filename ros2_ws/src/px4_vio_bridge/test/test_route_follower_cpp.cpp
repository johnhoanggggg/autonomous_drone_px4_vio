#include <cmath>

#include <gtest/gtest.h>

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
