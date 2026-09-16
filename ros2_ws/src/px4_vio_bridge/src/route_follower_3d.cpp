#include "px4_vio_bridge/route_follower_3d.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_vio_bridge
{
Follower3DConfig limits_for_map_tilt(const Follower3DConfig & config, double cosine)
{
  if (!std::isfinite(cosine) || std::abs(cosine) > 1.0 + 1e-9) {
    throw std::invalid_argument("invalid map rotation");
  }
  cosine = std::abs(std::clamp(cosine, -1.0, 1.0));
  const double sine = std::sqrt(std::max(0.0, 1.0 - cosine * cosine));
  const auto scale = [sine, cosine](double horizontal, double vertical) {
      if (!std::isfinite(horizontal) || !std::isfinite(vertical) || horizontal <= 0 || vertical <= 0) {
        throw std::invalid_argument("invalid rotated motion limits");
      }
      return std::min({1.0, horizontal / (horizontal + vertical * sine),
        vertical / (horizontal * sine + vertical * cosine)});
    };
  auto result = config;
  const auto speed = scale(config.max_horizontal_speed, config.max_vertical_speed);
  const auto acceleration = scale(config.max_horizontal_acceleration, config.max_vertical_acceleration);
  result.max_horizontal_speed *= speed; result.max_vertical_speed *= speed;
  result.max_horizontal_acceleration *= acceleration; result.max_vertical_acceleration *= acceleration;
  return result;
}

Point3 segment_velocity_3d(const Point3 & displacement, const Point3 & velocity,
  double dt, const Follower3DConfig & config)
{
  const double length=distance_3d(displacement,{});
  Point3 desired{};
  if (length>1e-12) {
    const auto direction=scale_3d(displacement,1.0/length);
    const double h=std::hypot(direction.x,direction.y),v=std::abs(direction.z);
    const double speed=std::min(config.max_horizontal_speed/std::max(h,1e-12),
      config.max_vertical_speed/std::max(v,1e-12));
    const double acceleration=std::min(config.max_horizontal_acceleration/std::max(h,1e-12),
      config.max_vertical_acceleration/std::max(v,1e-12));
    const double adt=acceleration*dt;
    const double braking=2*acceleration*length/(std::sqrt(adt*adt+2*acceleration*length)+adt);
    desired=scale_3d(direction,std::min(speed,braking));
  }
  const auto delta=subtract_3d(desired,velocity);
  const double scale=std::min({1.0,
    config.max_horizontal_acceleration*dt/std::max(std::hypot(delta.x,delta.y),1e-12),
    config.max_vertical_acceleration*dt/std::max(std::abs(delta.z),1e-12)});
  return add_3d(velocity,scale_3d(delta,scale));
}

namespace
{

Point3 limit_velocity(const Point3 & target, const Follower3DConfig & config)
{
  Point3 limited = target;
  const auto axis_speed = config.max_horizontal_speed / std::sqrt(2.0);
  limited.x = std::clamp(target.x, -axis_speed, axis_speed);
  limited.y = std::clamp(target.y, -axis_speed, axis_speed);
  limited.z = std::clamp(target.z, -config.max_vertical_speed, config.max_vertical_speed);
  return limited;
}

Point3 limit_acceleration(
  const Point3 & current, const Point3 & desired, double dt,
  const Follower3DConfig & config)
{
  auto delta = subtract_3d(desired, current);
  // Reserve an independent braking budget on each horizontal axis. Their
  // combined magnitude still stays within the configured horizontal bound.
  const auto axis_delta = config.max_horizontal_acceleration * dt / std::sqrt(2.0);
  delta.x = std::clamp(delta.x, -axis_delta, axis_delta);
  delta.y = std::clamp(delta.y, -axis_delta, axis_delta);
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
  target_vertex_=path_->project(pose).segment+1;
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
  if (config_.stop_at_vertices) {
    // Brake at each vertex: a long lookahead can cut a valid route's corner.
    const auto & points=path_->points();
    if (target_vertex_+1<points.size() && distance_3d(carrot_,points[target_vertex_])<1e-5 &&
      distance_3d(pose,points[target_vertex_])<std::min(.05,config_.arrival_tolerance) && distance_3d(velocity_,{})<1e-4) {
      ++target_vertex_;
    }
    result.lookahead=points[target_vertex_];
  } else {
    result.lookahead = path_->point_at(
      std::min(path_->length(), result.projection.along + config_.lookahead));
  }
  const auto motion_target=result.lookahead;
  if (config_.stop_at_vertices) {
    const auto delta=subtract_3d(result.lookahead,pose);
    const double length=distance_3d(result.lookahead,pose);
    if (length>config_.lookahead) {
      result.lookahead=add_3d(pose,scale_3d(delta,config_.lookahead/length));
    }
  }
  if (!validator || !validator(pose, result.lookahead)) {
    result.reason = "LOOKAHEAD_CHORD_BLOCKED";
    return result;
  }
  if (!initialized_) {carrot_ = pose; initialized_ = true;}
  const auto to_target = subtract_3d(motion_target, carrot_);
  // Reserve both this tick's travel and the subsequent braking distance:
  // v*dt + v*v/(2*a) <= distance. Clamping position after integration would
  // abruptly change velocity at a short endpoint and violate acceleration.
  const auto braking_velocity = [dt](double distance, double acceleration) {
      const auto d = std::abs(distance);
      const auto adt = acceleration * dt;
      const auto speed = (2.0 * acceleration * d) /
        (std::sqrt(adt * adt + 2.0 * acceleration * d) + adt);
      return std::copysign(speed, distance);
    };
  const auto horizontal_axis_acceleration = config_.max_horizontal_acceleration / std::sqrt(2.0);
  const Point3 unconstrained{
    braking_velocity(to_target.x, horizontal_axis_acceleration),
    braking_velocity(to_target.y, horizontal_axis_acceleration),
    braking_velocity(to_target.z, config_.max_vertical_acceleration)};
  const auto desired_velocity = limit_velocity(unconstrained, config_);
  const auto next_velocity = config_.stop_at_vertices ?
    segment_velocity_3d(to_target,velocity_,dt,config_) :
    limit_acceleration(velocity_, desired_velocity, dt, config_);
  const auto candidate = add_3d(carrot_, scale_3d(next_velocity, dt));
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
