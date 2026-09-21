#pragma once

/// ROS-free bounded velocity-error feedback for route braking and holds.

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include "px4_vio_bridge/path_geometry.hpp"

namespace px4_vio_bridge
{

struct BrakeDampingResult
{
  Point2 acceleration{};
  Point2 raw_correction{};
  Point2 filtered_correction{};
  Point2 applied_correction{};
  std::string reason;
};

class BrakeDamping
{
public:
  BrakeDamping(
    bool enabled, double gain, double correction_limit,
    double time_constant, double acceleration_limit)
  : enabled_(enabled), gain_(gain), correction_limit_(correction_limit),
    time_constant_(time_constant), acceleration_limit_(acceleration_limit)
  {
    if (!std::isfinite(gain_) || gain_ < 0.0) {
      throw std::invalid_argument("damping gain must be finite and nonnegative");
    }
    if (!std::isfinite(correction_limit_) || correction_limit_ <= 0.0 ||
      !std::isfinite(time_constant_) || time_constant_ <= 0.0 ||
      !std::isfinite(acceleration_limit_) || acceleration_limit_ <= 0.0)
    {
      throw std::invalid_argument("damping limits and time constant must be finite and positive");
    }
    if (enabled_ && gain_ > 0.0 && correction_limit_ > acceleration_limit_) {
      throw std::invalid_argument("damping limit must not exceed total acceleration limit");
    }
  }

  void reset()
  {
    previous_time_.reset();
    correction_ = {0.0, 0.0};
  }

  [[nodiscard]] bool enabled() const {return enabled_;}
  [[nodiscard]] double gain() const {return gain_;}
  [[nodiscard]] double correction_limit() const {return correction_limit_;}
  [[nodiscard]] double time_constant() const {return time_constant_;}

  BrakeDampingResult update(
    const Point2 & reference_velocity, const Point2 & reference_acceleration,
    const std::optional<Point2> & measured_velocity, double now,
    bool active, bool braking)
  {
    const auto unchanged = [this, &reference_acceleration](const char * reason) {
        reset();
        return BrakeDampingResult{
          reference_acceleration, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, reason};
      };
    if (!enabled_ || gain_ == 0.0) {
      return unchanged(enabled_ ? "zero_gain" : "disabled");
    }
    if (!active) {
      return unchanged("inactive_state");
    }
    if (!finite(reference_velocity) || !finite(reference_acceleration)) {
      return unchanged("reference_unavailable");
    }
    if (!measured_velocity || !finite(*measured_velocity)) {
      return unchanged("velocity_unavailable");
    }
    if (!std::isfinite(now)) {
      return unchanged("invalid_time");
    }
    const double dt = previous_time_ ? now - *previous_time_ : 0.0;
    if (previous_time_ && !(dt > 0.0 && dt <= 0.15)) {
      return unchanged("callback_gap");
    }
    previous_time_ = now;
    const Point2 raw = braking ? Point2{
      gain_ * (reference_velocity.first - measured_velocity->first),
      gain_ * (reference_velocity.second - measured_velocity->second)} : Point2{0.0, 0.0};
    const Point2 target = limit_norm(raw, correction_limit_);
    const double alpha = -std::expm1(-dt / time_constant_);
    correction_ = {
      correction_.first + alpha * (target.first - correction_.first),
      correction_.second + alpha * (target.second - correction_.second)};
    const Point2 acceleration = limit_norm(
      {reference_acceleration.first + correction_.first,
        reference_acceleration.second + correction_.second},
      acceleration_limit_);
    const Point2 applied{
      acceleration.first - reference_acceleration.first,
      acceleration.second - reference_acceleration.second};
    return {
      acceleration, raw, correction_, applied,
      braking ? "braking_or_hold" : "phase_release"};
  }

private:
  bool enabled_{};
  double gain_{}, correction_limit_{}, time_constant_{}, acceleration_limit_{};
  std::optional<double> previous_time_;
  Point2 correction_{0.0, 0.0};
};

}  // namespace px4_vio_bridge
