#include <gtest/gtest.h>

#include <cmath>
#include <optional>

#include "px4_vio_bridge/climb_feedforward.hpp"

using px4_vio_bridge::ClimbRampConfig;
using px4_vio_bridge::ClimbRampState;
using px4_vio_bridge::climb_ramp_step;

namespace
{
ClimbRampConfig config(bool latch)
{
  return {0.25, 0.12, 0.05, true, latch, 0.02};
}
}  // namespace

TEST(ClimbFeedforward, PullsAtFullRateWhileClimbing)
{
  ClimbRampState state;
  const auto [z, vz] = climb_ramp_step(config(true), state, 0.50, 0.10);
  EXPECT_NEAR(z, 0.22, 1e-9);  // leashed 0.12 above the vehicle
  EXPECT_DOUBLE_EQ(vz, -0.25);
}

TEST(ClimbFeedforward, TapersInsideTheReleaseBand)
{
  ClimbRampState state{0.50, std::nullopt};
  EXPECT_NEAR(climb_ramp_step(config(true), state, 0.50, 0.475).second, -0.125, 1e-9);
}

TEST(ClimbFeedforward, UnlatchedTaperActsAsAltitudeGainAtHover)
{
  // The 20261004T060900Z bob: +/-3 cm of hover error commanded +/-0.15 m/s.
  ClimbRampState state{0.50, std::nullopt};
  EXPECT_NEAR(climb_ramp_step(config(false), state, 0.50, 0.53).second, 0.15, 1e-9);
  EXPECT_NEAR(climb_ramp_step(config(false), state, 0.50, 0.47).second, -0.15, 1e-9);
}

TEST(ClimbFeedforward, LatchEndsTheFeedforwardOnceTheClimbIsDone)
{
  ClimbRampState state{0.50, std::nullopt};
  // Vehicle arrives within 1 cm with the ramp finished: latch.
  EXPECT_DOUBLE_EQ(climb_ramp_step(config(true), state, 0.50, 0.495).second, 0.0);
  ASSERT_TRUE(state.ff_done_target.has_value());
  // Hover error afterwards no longer drives the velocity setpoint.
  EXPECT_DOUBLE_EQ(climb_ramp_step(config(true), state, 0.50, 0.53).second, 0.0);
  EXPECT_DOUBLE_EQ(climb_ramp_step(config(true), state, 0.50, 0.47).second, 0.0);
  EXPECT_DOUBLE_EQ(climb_ramp_step(config(true), state, 0.50, 0.53).first, 0.50);
}

TEST(ClimbFeedforward, LatchWaitsForTheVehicleNotTheRamp)
{
  // Ramp converged but the vehicle is still 3 cm low: keep pulling.
  ClimbRampState state{0.50, std::nullopt};
  EXPECT_LT(climb_ramp_step(config(true), state, 0.50, 0.47).second, 0.0);
  EXPECT_FALSE(state.ff_done_target.has_value());
}

TEST(ClimbFeedforward, NewTargetReArmsTheFeedforward)
{
  ClimbRampState state{0.50, 0.50};
  // A descent target releases the latch and pushes down (NED +vz).
  const auto [z, vz] = climb_ramp_step(config(true), state, 0.20, 0.50);
  EXPECT_LT(z, 0.50);
  EXPECT_DOUBLE_EQ(vz, 0.25);
  EXPECT_FALSE(state.ff_done_target.has_value());
}

TEST(ClimbFeedforward, FullClimbLatchesAndThenHolds)
{
  // Simulate a first-order vehicle following the ramp; it must latch and the
  // feedforward must stay zero through a later +/-3 cm disturbance.
  ClimbRampState state;
  double z_now = 0.0;
  bool latched = false;
  for (int i = 0; i < 2000 && !latched; ++i) {
    const auto [z_sp, vz] = climb_ramp_step(config(true), state, 0.50, z_now);
    z_now += 0.2 * (z_sp - z_now);
    latched = state.ff_done_target.has_value();
    (void)vz;
  }
  ASSERT_TRUE(latched);
  EXPECT_DOUBLE_EQ(climb_ramp_step(config(true), state, 0.50, 0.53).second, 0.0);
}

TEST(ClimbFeedforward, DisabledFeedforwardAndRawStepPublishNoVelocity)
{
  auto no_ff = config(true);
  no_ff.feedforward = false;
  ClimbRampState state;
  EXPECT_TRUE(std::isnan(climb_ramp_step(no_ff, state, 0.50, 0.0).second));
  auto raw = config(true);
  raw.rate = 0.0;
  ClimbRampState raw_state;
  const auto [z, vz] = climb_ramp_step(raw, raw_state, 0.50, 0.0);
  EXPECT_DOUBLE_EQ(z, 0.50);
  EXPECT_TRUE(std::isnan(vz));
}
