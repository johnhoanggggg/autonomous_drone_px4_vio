#include "px4_vio_bridge/route_follower_3d.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_vio_bridge
{
namespace
{

Point3 limit_velocity(const Point3 & target, const Follower3DConfig & config)
{
  Point3 limited = target;
  const auto horizontal = std::hypot(target.x, target.y);
  if (horizontal > config.max_horizontal_speed && horizontal > 0.0) {
    const auto scale = config.max_horizontal_speed / horizontal;
    limited.x *= scale;
    limited.y *= scale;
  }
  limited.z = std::clamp(target.z, -config.max_vertical_speed, config.max_vertical_speed);
  return limited;
}

Point3 limit_acceleration(
  const Point3 & current, const Point3 & desired, double dt,
  const Follower3DConfig & config)
{
  auto delta = subtract_3d(desired, current);
  const auto horizontal = std::hypot(delta.x, delta.y);
  const auto max_horizontal_delta = config.max_horizontal_acceleration * dt;
  if (horizontal > max_horizontal_delta && horizontal > 0.0) {
    const auto scale = max_horizontal_delta / horizontal;
    delta.x *= scale;
    delta.y *= scale;
  }
  delta.z = std::clamp(
    delta.z, -config.max_vertical_acceleration * dt,
    config.max_vertical_acceleration * dt);
  return add_3d(current, delta);
}

}  // namespace

RouteFollower3D::RouteFollower3D(Follower3DConfig config)
: config_(config)
{
  if (config_.lookahead <= 0.0 || config_.max_horizontal_speed <= 0.0 ||
    config_.max_vertical_speed <= 0.0 || config_.max_horizontal_acceleration <= 0.0 ||
    config_.max_vertical_acceleration <= 0.0 || config_.max_cross_track <= 0.0 ||
    config_.max_vertical_track <= 0.0 || config_.arrival_tolerance <= 0.0)
  {
    throw std::invalid_argument("invalid 3D follower configuration");
  }
}

void RouteFollower3D::clear()
{
  path_.reset();
  velocity_ = {};
  initialized_ = false;
}

bool RouteFollower3D::set_path(const std::vector<Point3> & points, const Point3 & pose)
{
  try {
    path_ = std::make_unique<Polyline3D>(points);
  } catch (const std::invalid_argument &) {
    clear();
    return false;
  }
  carrot_ = pose;
  velocity_ = {};
  initialized_ = true;
  return true;
}

FollowResult3D RouteFollower3D::update(
  const Point3 & pose, double dt, const ChordValidator & validator)
{
  FollowResult3D result;
  result.carrot = pose;
  result.lookahead = pose;
  if (!path_) {result.reason = "NO_PATH"; return result;}
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5) {result.reason = "INVALID_DT"; return result;}
  result.projection = path_->project(pose);
  result.remaining = std::max(0.0, path_->length() - result.projection.along);
  if (result.projection.horizontal_distance > config_.max_cross_track) {
    result.reason = "CROSS_TRACK";
    return result;
  }
  if (result.projection.vertical_distance > config_.max_vertical_track) {
    result.reason = "VERTICAL_TRACK";
    return result;
  }
  result.lookahead = path_->point_at(
    std::min(path_->length(), result.projection.along + config_.lookahead));
  if (!validator || !validator(pose, result.lookahead)) {
    result.reason = "LOOKAHEAD_CHORD_BLOCKED";
    return result;
  }
  if (!initialized_) {carrot_ = pose; initialized_ = true;}
  const auto to_target = subtract_3d(result.lookahead, carrot_);
  const Point3 unconstrained{
    to_target.x / dt, to_target.y / dt, to_target.z / dt};
  const auto desired_velocity = limit_velocity(unconstrained, config_);
  const auto next_velocity = limit_acceleration(velocity_, desired_velocity, dt, config_);
  auto candidate = add_3d(carrot_, scale_3d(next_velocity, dt));
  // Do not overshoot the selected finite lookahead along any controlled axis.
  const auto clamp_axis = [](double current, double target, double value) {
      return target >= current ? std::min(value, target) : std::max(value, target);
    };
  candidate.x = clamp_axis(carrot_.x, result.lookahead.x, candidate.x);
  candidate.y = clamp_axis(carrot_.y, result.lookahead.y, candidate.y);
  candidate.z = clamp_axis(carrot_.z, result.lookahead.z, candidate.z);
  if (!validator(pose, candidate)) {
    velocity_ = {};
    initialized_ = false;
    result.reason = "CARROT_CHORD_BLOCKED";
    return result;
  }
  const auto actual_velocity = scale_3d(subtract_3d(candidate, carrot_), 1.0 / dt);
  result.acceleration = scale_3d(subtract_3d(actual_velocity, velocity_), 1.0 / dt);
  velocity_ = actual_velocity;
  carrot_ = candidate;
  result.carrot = carrot_;
  result.displacement = subtract_3d(carrot_, pose);
  result.velocity = velocity_;
  result.reached = result.remaining <= config_.arrival_tolerance &&
    distance_3d(pose, path_->points().back()) <= config_.arrival_tolerance;
  result.valid = true;
  result.reason = result.reached ? "GOAL_REACHED" : "FOLLOWING";
  return result;
}

}  // namespace px4_vio_bridge
