#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace px4_vio_bridge
{

// Vehicle within this of the altitude target (with the ramp finished) ends the
// climb feedforward when latching is on. Same value as offboard_hover.py.
constexpr double kClimbFeedforwardLatchTol = 0.01;

struct ClimbRampConfig
{
  double rate{};         // commanded climb rate, m/s; <= 0 is the raw step
  double leash{};        // max setpoint lead over the vehicle, m; <= 0 is off
  double release{};      // feedforward taper band, m
  bool feedforward{};    // publish vz alongside the ramped position
  bool latch{};          // end the feedforward once the climb has finished
  double dt{};           // control period, s
};

struct ClimbRampState
{
  std::optional<double> z_cmd;           // ramped altitude setpoint, up-positive
  std::optional<double> ff_done_target;  // target whose climb feedforward ended
};

// One control tick of the altitude ramp. Returns {z setpoint (up), vz setpoint
// (NED, NaN = none)}. `z_now` is the vehicle altitude (up) when known.
//
// PX4's takeoff ramp hands the position controller a collective ~0.08 below
// hover, so a hard z step climbs on the integrator alone (20.4 s for 0.30 m,
// ULog 209). The feedforward puts the full commanded rate into the velocity
// loop; the ramp keeps the position setpoint honest while that happens.
//
// Without the latch the taper never ends: at hover it is rate/release (5 /s
// at the defaults) of extra position gain stacked on MPC_Z_P, which flew as a
// 0.8 Hz, +/-2.5 cm height bob on 20261004T060900Z.
inline std::pair<double, double> climb_ramp_step(
  const ClimbRampConfig & cfg, ClimbRampState & state, double target_up,
  std::optional<double> z_now)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (cfg.rate <= 0.0) {
    state.z_cmd = target_up;
    return {target_up, nan};
  }
  if (!state.z_cmd) {
    state.z_cmd = target_up;
  }
  // Advance toward the target, clamped to the remaining distance so the ramp
  // lands exactly on it and never overshoots.
  const double step = cfg.rate * cfg.dt;
  *state.z_cmd += std::clamp(target_up - *state.z_cmd, -step, step);
  // The leash is applied on every path, including the first call.
  if (cfg.leash > 0.0 && z_now) {
    state.z_cmd = std::clamp(*state.z_cmd, *z_now - cfg.leash, *z_now + cfg.leash);
  }
  if (!cfg.feedforward) {
    return {*state.z_cmd, nan};
  }
  if (cfg.latch && state.ff_done_target) {
    if (std::abs(*state.ff_done_target - target_up) <= 1.0e-6) {
      // Climb finished: PX4's own altitude loop holds from here.
      return {*state.z_cmd, 0.0};
    }
    state.ff_done_target.reset();  // a new target re-arms the feedforward
  }
  // Release on the VEHICLE's remaining distance, not the ramp's: the ramp can
  // reach the target while the vehicle is still low but inside the leash.
  const double z_ref = z_now ? *z_now : *state.z_cmd;
  const double remaining = target_up - z_ref;
  if (cfg.latch && std::abs(*state.z_cmd - target_up) <= 1.0e-9 &&
    std::abs(remaining) <= kClimbFeedforwardLatchTol)
  {
    state.ff_done_target = target_up;
    return {*state.z_cmd, 0.0};
  }
  // Taper inside the release band rather than switching off, so there is no
  // discontinuity and noise near the target cannot chatter the feedforward.
  const double span = std::max(cfg.release, 1.0e-6);
  const double rate = cfg.rate * std::min(1.0, std::abs(remaining) / span);
  if (rate < 1.0e-3) {
    return {*state.z_cmd, 0.0};
  }
  // Deliberately the commanded rate, not d(z_cmd)/dt: while the leash holds the
  // setpoint back the vehicle is failing to keep up, which is exactly when the
  // velocity loop needs a real error to wind against. NED, so climbing is -vz.
  return {*state.z_cmd, -std::copysign(rate, remaining)};
}

}  // namespace px4_vio_bridge
