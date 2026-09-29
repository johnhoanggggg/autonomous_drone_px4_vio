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

bool suffix_near(
  const Polyline3D & source, double source_along,
  const Polyline3D & target, double target_along, double tolerance)
{
  const auto remaining = std::max(0.0, source.length() - source_along);
  // Uniform arc-length samples catch equivalent routes with different vertex
  // spacing. Bound the work for malformed or unusually dense input paths.
  const auto samples = std::clamp<std::size_t>(
    static_cast<std::size_t>(std::ceil(remaining / std::max(tolerance * 0.5, 0.01))),
    1, 512);
  for (std::size_t index = 0; index <= samples; ++index) {
    const auto along = source_along + remaining * static_cast<double>(index) /
      static_cast<double>(samples);
    const auto projection = target.project(source.point_at(along));
    if (projection.distance > tolerance + 1.0e-12 ||
      projection.along + tolerance < target_along)
    {
      return false;
    }
  }
  return true;
}

bool equivalent_suffixes(
  const Polyline3D & first, const Polyline3D & second,
  const Point3 & pose, double tolerance)
{
  const auto first_pose = first.project(pose);
  const auto second_pose = second.project(pose);
  if (first_pose.distance > tolerance || second_pose.distance > tolerance ||
    distance_3d(first.points().back(), second.points().back()) > tolerance)
  {
    return false;
  }
  const auto first_remaining = first.length() - first_pose.along;
  const auto second_remaining = second.length() - second_pose.along;
  if (std::abs(first_remaining - second_remaining) > 2.0 * tolerance) {return false;}
  return suffix_near(first, first_pose.along, second, second_pose.along, tolerance) &&
         suffix_near(second, second_pose.along, first, first_pose.along, tolerance);
}

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
  if (config_.path_carrot) {
    s_ = path_->project(pose).along;
    carrot_ = path_->point_at(s_);
    speed_ = 0.0;
  }
  initialized_ = true;
  return true;
}

void RouteFollower3D::seed_velocity(const Point3 & velocity)
{
  velocity_ = velocity;
  if (config_.path_carrot && path_) {
    const auto seg = segment_at(s_);
    const auto & p = path_->points();
    const auto d = subtract_3d(p[seg + 1], p[seg]);
    const double length = distance_3d(d, {});
    speed_ = length > 1e-9 ? std::max(0.0, (velocity.x * d.x + velocity.y * d.y + velocity.z * d.z) / length) : 0.0;
    speed_ = std::min(speed_, segment_speed(seg));
    velocity_ = length > 1e-9 ? scale_3d(d, speed_ / length) : Point3{};
  }
}

std::size_t RouteFollower3D::segment_at(double along) const
{
  const auto & c = path_->cumulative();
  std::size_t seg = 0;
  while (seg + 2 < c.size() && c[seg + 1] <= along) {++seg;}
  return seg;
}

double RouteFollower3D::segment_speed(std::size_t segment) const
{
  const auto & p = path_->points();
  const auto d = subtract_3d(p[segment + 1], p[segment]);
  const double length = distance_3d(d, {});
  if (length < 1e-9) {return config_.max_horizontal_speed;}
  const double h = std::hypot(d.x, d.y) / length, v = std::abs(d.z) / length;
  return std::min(h > 1e-9 ? config_.max_horizontal_speed / h : 1e9,
    v > 1e-9 ? config_.max_vertical_speed / v : 1e9);
}

double RouteFollower3D::segment_acceleration(std::size_t segment) const
{
  const auto & p = path_->points();
  const auto d = subtract_3d(p[segment + 1], p[segment]);
  const double length = distance_3d(d, {});
  if (length < 1e-9) {return config_.max_horizontal_acceleration;}
  const double h = std::hypot(d.x, d.y) / length, v = std::abs(d.z) / length;
  return std::min(h > 1e-9 ? config_.max_horizontal_acceleration / h : 1e9,
    v > 1e-9 ? config_.max_vertical_acceleration / v : 1e9);
}

// Highest along-route speed at `along` that, after this tick's travel, can still
// slow to every upcoming corner's speed (and to zero at the end):
// v*dt + (v*v - corner*corner) / (2*a) <= distance.
double RouteFollower3D::allowed_speed(double along, double dt) const
{
  const auto & p = path_->points();
  const auto & c = path_->cumulative();
  const auto seg = segment_at(along);
  double allowed = segment_speed(seg);
  const double brake = std::max(1e-3, segment_acceleration(seg));
  for (std::size_t k = seg + 1; k < p.size(); ++k) {
    const double distance = std::max(0.0, c[k] - along);
    double corner = 0.0;  // the end of the route
    if (k + 1 < p.size()) {
      const auto a = subtract_3d(p[k], p[k - 1]), b = subtract_3d(p[k + 1], p[k]);
      const double la = distance_3d(a, {}), lb = distance_3d(b, {});
      const double cosine = (la > 1e-9 && lb > 1e-9) ?
        std::clamp((a.x * b.x + a.y * b.y + a.z * b.z) / (la * lb), -1.0, 1.0) : 1.0;
      corner = std::min(segment_speed(k - 1), segment_speed(k));
      corner = std::max(config_.corner_speed_min, corner * std::max(0.0, cosine));
    }
    const double adt = brake * dt;
    allowed = std::min(allowed, std::max(0.0,
      std::sqrt(adt * adt + corner * corner + 2.0 * brake * distance) - adt));
    if (distance > allowed * allowed / (2.0 * brake) + 1.0) {break;}
  }
  return allowed;
}

bool RouteFollower3D::replace_equivalent_path(
  const std::vector<Point3> & points, const Point3 & pose, double tolerance,
  const ChordValidator & validator)
{
  if (!path_ || !initialized_ || !std::isfinite(tolerance) || tolerance <= 0.0) {
    return false;
  }
  std::unique_ptr<Polyline3D> replacement;
  try {
    replacement = std::make_unique<Polyline3D>(points);
  } catch (const std::invalid_argument &) {
    return false;
  }
  if (!equivalent_suffixes(*path_, *replacement, pose, tolerance)) {return false;}

  const auto pose_projection = replacement->project(pose);
  const auto carrot_projection = replacement->project(carrot_);
  if (carrot_projection.distance > tolerance) {return false;}
  // Never carry route progress behind the aircraft. This is the bounded rejoin
  // used after a short hold or a slightly shifted replacement route.
  const auto carried_along = std::max(pose_projection.along, carrot_projection.along);
  const auto carried_carrot = replacement->point_at(carried_along);
  if (!validator || !validator(pose, carried_carrot)) {return false;}

  path_ = std::move(replacement);
  carrot_ = carried_carrot;
  if (config_.path_carrot) {s_ = carried_along;}
  target_vertex_ = std::min(path_->points().size() - 1,
    path_->project(carrot_).segment + 1);
  // velocity_ deliberately survives. The normal update path still applies its
  // acceleration limits and validates both lookahead and command chords.
  return true;
}

bool RouteFollower3D::hold_equivalent_path(
  const std::vector<Point3> & points, const Point3 & pose, double tolerance,
  const ChordValidator & validator)
{
  if (!path_ || !initialized_ || !validator || !(config_.hold_line_skip > 0.0) ||
    !std::isfinite(tolerance) || tolerance <= 0.0)
  {
    return false;
  }
  std::unique_ptr<Polyline3D> replacement;
  try {
    replacement = std::make_unique<Polyline3D>(points);
  } catch (const std::invalid_argument &) {
    return false;
  }
  if (distance_3d(path_->points().back(), replacement->points().back()) > tolerance) {return false;}
  const auto pose_projection = path_->project(pose);
  if (pose_projection.distance > config_.hold_line_max_offset) {return false;}
  // The replacement beyond its connector must lie on the current route, and the
  // current route ahead of the command point must lie on the replacement.
  const double skip = std::min(config_.hold_line_skip, 0.5 * replacement->length());
  if (!suffix_near(*replacement, skip, *path_, 0.0, tolerance)) {return false;}
  const auto carrot_projection = path_->project(carrot_);
  const double from = std::max(pose_projection.along, carrot_projection.along) + skip;
  if (from < path_->length() && !suffix_near(*path_, from, *replacement, 0.0, tolerance)) {
    return false;
  }
  // Never carry progress behind the aircraft (as replace_equivalent_path does),
  // but move the command point ALONG the held route, not onto the drone's line.
  // Re-anchored replans used to reset the vertex stop; holding must too, or the
  // drone would have to reach every vertex within 5 cm before moving on.
  auto carrot = carrot_;
  if (pose_projection.along > carrot_projection.along) {
    carrot = path_->point_at(pose_projection.along);
    if (!validator(pose, carrot)) {return false;}
  }
  // The held route must still clear the current map from the command point on.
  const auto & held = path_->points();
  const auto from_projection = path_->project(carrot);
  Point3 previous = carrot;
  for (std::size_t index = from_projection.segment + 1; index < held.size(); ++index) {
    if (!validator(previous, held[index])) {return false;}
    previous = held[index];
  }
  carrot_ = carrot;
  target_vertex_ = std::max(target_vertex_,
    std::min(held.size() - 1, from_projection.segment + 1));
  if (config_.path_carrot) {s_ = std::max(s_, from_projection.along);}
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
  const double heading_arc =
    config_.heading_lookahead > 0.0 ? config_.heading_lookahead : config_.lookahead;
  result.route_ahead = path_->point_at(
    std::min(path_->length(), result.projection.along + heading_arc));
  result.remaining = std::max(0.0, path_->length() - result.projection.along);
  if (result.projection.horizontal_distance > config_.max_cross_track) {
    result.reason = "CROSS_TRACK";
    return result;
  }
  if (result.projection.vertical_distance > config_.max_vertical_track) {
    result.reason = "VERTICAL_TRACK";
    return result;
  }
  const bool vertex_stops = config_.stop_at_vertices && !config_.path_carrot;
  if (vertex_stops) {
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
  if (vertex_stops) {
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
  if (config_.path_carrot) {
    // Never trail the aircraft; lead it by at most max_carrot_lead.
    s_ = std::clamp(std::max(s_, result.projection.along), 0.0, path_->length());
    const double lead = s_ - result.projection.along;
    double desired = allowed_speed(s_, dt);
    if (config_.max_carrot_lead > 0.0) {
      desired *= std::clamp((config_.max_carrot_lead - lead) / std::max(0.05, 0.3 * config_.max_carrot_lead), 0.0, 1.0);
    }
    const double accel = segment_acceleration(segment_at(s_));
    speed_ = std::clamp(desired, std::max(0.0, speed_ - accel * dt), speed_ + accel * dt);
    double next_s = std::min(path_->length(), s_ + speed_ * dt);
    if (path_->length() - next_s < 1e-4) {next_s = path_->length();}
    const auto candidate = path_->point_at(next_s);
    if (!validator(pose, candidate)) {
      velocity_ = {};
      speed_ = 0.0;
      result.reason = "CARROT_CHORD_BLOCKED";
      return result;
    }
    const auto actual_velocity = scale_3d(subtract_3d(candidate, carrot_), 1.0 / dt);
    result.acceleration = scale_3d(subtract_3d(actual_velocity, velocity_), 1.0 / dt);
    velocity_ = scale_3d(subtract_3d(path_->point_at(std::min(path_->length(), next_s + 1e-3)),
      candidate), speed_ / 1e-3);
    if (next_s >= path_->length() - 1e-9) {velocity_ = {};}
    s_ = next_s;
    carrot_ = candidate;
  } else {
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
  }
  result.carrot = carrot_;
  result.displacement = subtract_3d(carrot_, pose);
  if (config_.cross_track_gain > 0.0) {
    // Perpendicular offset from the drone back to the route at its projection.
    const auto & points = path_->points();
    const auto segment = std::min(result.projection.segment, points.size() - 2);
    const auto tangent_raw = subtract_3d(points[segment + 1], points[segment]);
    const double tangent_length = distance_3d(tangent_raw, {});
    auto back = subtract_3d(result.projection.point, pose);
    if (tangent_length > 1e-9) {
      const auto tangent = scale_3d(tangent_raw, 1.0 / tangent_length);
      const double along = back.x * tangent.x + back.y * tangent.y + back.z * tangent.z;
      back = subtract_3d(back, scale_3d(tangent, along));
    }
    auto correction = scale_3d(back, config_.cross_track_gain);
    const double size = distance_3d(correction, {});
    if (size > config_.cross_track_correction_max) {
      correction = scale_3d(correction, config_.cross_track_correction_max / size);
    }
    const auto commanded = add_3d(carrot_, correction);
    if (size > 1e-9 && validator(pose, commanded)) {
      result.correction = correction;
      result.displacement = subtract_3d(commanded, pose);
    }
  }
  result.velocity = velocity_;
  result.reached = result.remaining <= config_.arrival_tolerance &&
    distance_3d(pose, path_->points().back()) <= config_.arrival_tolerance;
  result.valid = true;
  result.reason = result.reached ? "GOAL_REACHED" : "FOLLOWING";
  return result;
}

}  // namespace px4_vio_bridge
