#include "px4_vio_bridge/command_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace px4_vio_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;
}  // namespace

// --- HorizontalCommandLimiter ---------------------------------------------

HorizontalCommandLimiter::HorizontalCommandLimiter(
  double max_speed, double max_acceleration)
: max_speed_(max_speed), max_acceleration_(max_acceleration)
{
  if (!std::isfinite(max_speed) || max_speed <= 0.0) {
    throw std::invalid_argument("max_speed must be finite and positive");
  }
  if (!std::isfinite(max_acceleration) || max_acceleration <= 0.0) {
    throw std::invalid_argument("max_acceleration must be finite and positive");
  }
}

void HorizontalCommandLimiter::reset(const Point2 & position)
{
  if (!finite(position)) {
    throw std::invalid_argument("reset position must be finite");
  }
  position_ = position;
  velocity_ = {0.0, 0.0};
}

Point2 HorizontalCommandLimiter::update(const Point2 & target, double dt)
{
  if (!finite(target) || !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("target and dt must be finite, with positive dt");
  }
  if (!position_) {
    reset(target);
    return *position_;
  }

  const Point2 offset{target.first - position_->first, target.second - position_->second};
  const double distance_to_target = std::hypot(offset.first, offset.second);
  Point2 desired_velocity{0.0, 0.0};
  if (distance_to_target > 1.0e-9) {
    // The stopping-speed term makes the command decelerate as it closes on a
    // stationary target instead of arriving with a velocity step.
    const double speed = std::min(
      {max_speed_, distance_to_target / dt,
        std::sqrt(2.0 * max_acceleration_ * distance_to_target)});
    desired_velocity = {
      offset.first / distance_to_target * speed,
      offset.second / distance_to_target * speed};
  }

  const Point2 velocity_change = limit_norm(
    {desired_velocity.first - velocity_.first, desired_velocity.second - velocity_.second},
    max_acceleration_ * dt);
  velocity_ = limit_norm(
    {velocity_.first + velocity_change.first, velocity_.second + velocity_change.second},
    max_speed_);
  // Do not snap to a nearby target: it is rebased from the latest PX4 position
  // and moves even while the airframe is stationary. A small bounded overshoot
  // is preferable to bypassing the acceleration limit.
  position_ = Point2{
    position_->first + velocity_.first * dt,
    position_->second + velocity_.second * dt};
  return *position_;
}

Point2 HorizontalCommandLimiter::adopt(const Point2 & position, const Point2 & velocity)
{
  if (!finite(position) || !finite(velocity)) {
    throw std::invalid_argument("adopted position and velocity must be finite");
  }
  if (std::hypot(velocity.first, velocity.second) > max_speed_ + 1.0e-9) {
    throw std::invalid_argument("adopted velocity exceeds max_speed");
  }
  position_ = position;
  velocity_ = velocity;
  return *position_;
}

// --- PathCommandLimiter ----------------------------------------------------

PathCommandLimiter::PathCommandLimiter(
  double max_speed, double max_acceleration, double max_projection_error,
  double corner_tolerance, double max_entry_error, double max_connector_error,
  double suffix_tolerance, bool corner_blending, double junction_deviation)
: max_speed_(max_speed),
  max_acceleration_(max_acceleration),
  max_projection_error_(max_projection_error),
  corner_tolerance_(corner_tolerance),
  max_entry_error_(max_entry_error),
  max_connector_error_(max_connector_error),
  suffix_tolerance_(suffix_tolerance),
  corner_blending_(corner_blending),
  junction_deviation_(junction_deviation)
{
  for (const double value : {max_speed, max_acceleration, max_projection_error,
      corner_tolerance, max_entry_error, max_connector_error, suffix_tolerance,
      junction_deviation})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("path command limits must be finite and positive");
    }
  }
  if (max_connector_error < max_projection_error) {
    throw std::invalid_argument(
            "connector tolerance must be at least the projection tolerance");
  }
}

void PathCommandLimiter::clear()
{
  installation_.reset();
  progress_ = 0.0;
  speed_ = 0.0;
  position_.reset();
  velocity_ = {0.0, 0.0};
  join_target_.reset();
  join_limit_ = 0.0;
  waiting_vertex_.reset();
}

PathCommandLimiter::Snapshot PathCommandLimiter::snapshot() const
{
  return Snapshot{
    installation_, progress_, speed_, position_, velocity_,
    join_target_, join_limit_, waiting_vertex_};
}

void PathCommandLimiter::restore(const Snapshot & state)
{
  installation_ = state.installation;
  progress_ = state.progress;
  speed_ = state.speed;
  position_ = state.position;
  velocity_ = state.velocity;
  join_target_ = state.join_target;
  join_limit_ = state.join_limit;
  waiting_vertex_ = state.waiting_vertex;
}

std::shared_ptr<const PathInstallation> PathCommandLimiter::build_installation(
  std::shared_ptr<const Polyline> path, PathFingerprint fingerprint)
{
  auto installation = std::make_shared<PathInstallation>();
  installation->path = std::move(path);
  installation->fingerprint = std::move(fingerprint);
  // Arc lengths of the vertices the command must stop at.
  const auto & points = installation->path->points();
  const auto & cumulative = installation->path->cumulative();
  for (std::size_t index = 1; index + 1 < points.size(); ++index) {
    const Point2 first{
      points[index].first - points[index - 1].first,
      points[index].second - points[index - 1].second};
    const Point2 second{
      points[index + 1].first - points[index].first,
      points[index + 1].second - points[index].second};
    const double cross = first.first * second.second - first.second * second.first;
    const double dot = first.first * second.first + first.second * second.second;
    if (std::abs(cross) > kEpsilon || dot <= 0.0) {
      installation->bends.push_back(cumulative[index]);
      installation->bend_turns.push_back(std::atan2(std::abs(cross), dot));
    }
  }
  return installation;
}

std::optional<double> PathCommandLimiter::shared_suffix_offset(
  const Polyline & new_path) const
{
  if (!installation_) {
    return std::nullopt;
  }
  const auto & old_points = installation_->path->points();
  const auto & new_points = new_path.points();
  std::size_t shared = 0;
  while (shared < old_points.size() && shared < new_points.size() &&
    distance(
      old_points[old_points.size() - 1 - shared],
      new_points[new_points.size() - 1 - shared]) <= suffix_tolerance_)
  {
    ++shared;
  }
  // One common endpoint is a shared point, not a shared segment.
  if (shared < 2) {
    return std::nullopt;
  }
  const double old_start = installation_->path->cumulative()[old_points.size() - shared];
  if (progress_ < old_start - kEpsilon) {
    return std::nullopt;
  }
  const double new_start = new_path.cumulative()[new_points.size() - shared];
  return new_start - old_start;
}

bool PathCommandLimiter::set_path(
  const std::vector<Point2> & points,
  const Point2 & reference,
  const ClearanceCheck & clearance_check)
{
  auto fingerprint = path_fingerprint(points);
  if (installation_ && fingerprint == installation_->fingerprint) {
    return false;
  }
  auto new_path = std::make_shared<const Polyline>(points);

  Point2 anchor{};
  Projection projection{};
  double limit = 0.0;

  if (!position_) {
    // Route entry. The anchor is the vehicle, whose own cross-track is far
    // larger than any command-to-command offset.
    anchor = reference;
    if (!finite(anchor)) {
      throw std::invalid_argument("path command reference must be finite");
    }
    projection = new_path->project(anchor);
    if (projection.cross_track > max_entry_error_ + kEpsilon) {
      char buffer[160];
      std::snprintf(
        buffer, sizeof(buffer),
        "route entry is %.3fm from the path (limit %.3fm)",
        projection.cross_track, max_entry_error_);
      throw std::invalid_argument(buffer);
    }
    limit = max_entry_error_;
  } else {
    anchor = *position_;
    const auto offset = shared_suffix_offset(*new_path);
    if (offset) {
      // Identical geometry ahead: keep the command point and its speed and
      // only renumber the arc length it is measured against.
      std::optional<double> waiting;
      if (waiting_vertex_ && *waiting_vertex_ + *offset >= -kEpsilon) {
        waiting = *waiting_vertex_ + *offset;
      }
      install(build_installation(new_path, std::move(fingerprint)), progress_ + *offset);
      position_ = new_path->point_at(progress_);
      waiting_vertex_ = waiting;
      return true;
    }

    projection = new_path->project(anchor);
    limit = max_projection_error_;
    if (projection.cross_track > limit + kEpsilon) {
      const bool connector_ok =
        clearance_check != nullptr &&
        projection.cross_track <= max_connector_error_ + kEpsilon &&
        clearance_check(anchor, projection.point);
      if (!connector_ok) {
        char buffer[200];
        std::snprintf(
          buffer, sizeof(buffer),
          "path is %.3fm from final command (limit %.3fm, connector limit %.3fm)",
          projection.cross_track, max_projection_error_, max_connector_error_);
        throw std::invalid_argument(buffer);
      }
      limit = max_connector_error_;
    }
  }

  install(build_installation(new_path, std::move(fingerprint)), projection.along);
  // Keep the exact previously-published point. If the path is offset, rejoin it
  // through the bounded band; snapping directly to projection.point would
  // bypass the speed limit.
  position_ = anchor;
  if (projection.cross_track > join_tolerance_) {
    join_target_ = projection.point;
    join_limit_ = limit;
    // Carry the standing rate into the join. update_join() runs the same
    // along-path speed machinery, so this is the speed it continues from rather
    // than a rate it is permitted to jump to.
    speed_ = std::min(max_speed_, std::hypot(velocity_.first, velocity_.second));
  } else {
    position_ = projection.point;
    speed_ = std::min(max_speed_, std::hypot(velocity_.first, velocity_.second));
  }
  return true;
}

void PathCommandLimiter::install(
  std::shared_ptr<const PathInstallation> installation, double progress)
{
  installation_ = std::move(installation);
  progress_ = std::clamp(progress, 0.0, installation_->path->length());
  join_target_.reset();
  join_limit_ = 0.0;
  waiting_vertex_.reset();
}

double PathCommandLimiter::along_path_speed(double along, double dt, bool advance)
{
  // The same speed machinery the on-path advance uses, factored out so a join
  // is braked by corners and by the path end exactly as ordinary motion is.
  if (!advance) {
    speed_ = std::max(0.0, speed_ - max_acceleration_ * dt);
    return speed_;
  }
  const double target_progress = next_motion_target(installation_->path->length());
  const double remaining = std::max(0.0, target_progress - along);
  if (remaining <= kEpsilon) {
    speed_ = 0.0;
    return 0.0;
  }
  const double braking_speed = std::max(
    0.0,
    -max_acceleration_ * dt +
    std::sqrt(
      (max_acceleration_ * dt) * (max_acceleration_ * dt) +
      2.0 * max_acceleration_ * remaining));
  double desired_speed = std::min({max_speed_, remaining / dt, braking_speed});
  if (corner_blending_) {
    desired_speed = std::min(desired_speed, corner_speed_limit());
  }
  const double delta = std::clamp(
    desired_speed - speed_, -max_acceleration_ * dt, max_acceleration_ * dt);
  speed_ = std::clamp(speed_ + delta, 0.0, max_speed_);
  return speed_;
}

Point2 PathCommandLimiter::path_tangent(double along) const
{
  // Finite difference rather than a Polyline accessor: the step is a hundredth
  // of a grid cell, and a vertex is the only place the two would disagree.
  const double length = installation_->path->length();
  const double step = std::min(1.0e-4, std::max(length, 1.0e-6));
  const double back = std::clamp(along - step, 0.0, length);
  const double forward = std::clamp(along + step, 0.0, length);
  const Point2 a = installation_->path->point_at(back);
  const Point2 b = installation_->path->point_at(forward);
  const Point2 delta{b.first - a.first, b.second - a.second};
  const double norm = std::hypot(delta.first, delta.second);
  if (norm <= kEpsilon) {
    return {0.0, 0.0};
  }
  return {delta.first / norm, delta.second / norm};
}

Point2 PathCommandLimiter::update_join(double dt, bool advance)
{
  // A join is a lane change, not a stop.
  //
  // This used to steer the command point at a FROZEN target -- the projection
  // of the old command onto the new path -- and complete only once the velocity
  // had decayed to within one tick of zero. Arriving at a fixed point at speed
  // would overshoot it, so that formulation could not end any other way: every
  // path replacement decelerated the command to a standstill and then
  // re-accelerated along the route from rest.
  //
  // That is what made the flown route stop-start. Measured over the three
  // flights of 20260831T064x, the planner replaced the accepted path 1.06-1.19
  // times a second while the command point needed 0.67 s to get back to
  // 0.20 m/s, and 20 of 27 stops began within 0.6 s of a replacement. Commanded
  // speed only reached the ceiling in samples where a path had been left alone
  // for more than 1.5 s.
  //
  // So the target rolls forward instead. The along-path component runs the
  // ordinary speed machinery -- corner limits, end braking, the acceleration
  // limit -- and a bounded lateral component closes the offset alongside it.
  // Both safety properties are unchanged: every tick is still rejected if it
  // would leave `join_limit_`, and the caller still clearance-checks the
  // resulting chord and rolls the tick back if it fails.
  const Point2 old_position = *position_;
  const auto start = installation_->path->project(*position_);

  const double along_speed = along_path_speed(start.along, dt, advance);
  const Point2 tangent = path_tangent(start.along);

  Point2 desired_velocity{tangent.first * along_speed, tangent.second * along_speed};

  const Point2 offset{
    start.point.first - position_->first, start.point.second - position_->second};
  const double offset_distance = std::hypot(offset.first, offset.second);
  if (advance && offset_distance > kEpsilon) {
    // Close the offset no faster than it can be stopped, and never faster than
    // it can be closed this tick, so the lateral term cannot overshoot the path.
    //
    // Then cap it against the along-path rate, because those three bounds alone
    // let the lateral term reach cruise speed: at 0.30 m/s^2 anything past
    // ~7 cm saturates `sqrt(2*a*offset)` at the full 0.20 m/s, and an equal
    // along-path term puts the commanded velocity 45 degrees off the route at
    // full speed. The planner republishes about twice a second anchored on the
    // vehicle, so a rejoin is not the occasional lane change this was written
    // for -- it was open for 36-57% of the route on 20260906T120129Z and
    // T120238Z, and the drone spent that time crabbing to the outside of every
    // bend (median 18 and 27 degrees off route, worst 85) until cross-track
    // tripped the follower and translation was held.
    //
    // `join_lateral_ratio_` bounds the merge angle instead. The floor is only
    // for the terminal case -- a rejoin opened where the route has no travel
    // left to scale against -- because a floor applied whenever the along-path
    // rate is merely small would dominate the direction coming out of a brake,
    // where one tick of acceleration is all the along-path rate there is.
    // One flat angle is not enough either. 20260906T124409Z auto-landed on it:
    // a hard replan left the command 0.168 m off the new path, 15 degrees of a
    // 0.20 m/s cruise needed about three seconds to close that, the follower's
    // 0.08 m cross-track limit tripped after 0.9 s, and a latched fault stops
    // update_route_command() being called at all -- so the merge froze with the
    // offset standing. Six seconds later the adapter landed. T124502Z reached
    // 0.211 m, past the connector limit, where no replacement joins at all.
    //
    // So widen the angle in proportion to how far past `max_projection_error_`
    // the offset has got. Inside that tolerance the offset is the few
    // centimetres of re-anchoring the bound exists for and the merge stays at
    // `join_lateral_ratio_`; beyond it the command is somewhere a replacement
    // can no longer be joined from, and closing that is worth a wider angle.
    //
    // The escalation stays proportional to the along-path rate, deliberately.
    // An absolute rate -- a merge-time bound -- was tried first and put the
    // replayed 90th percentile back at 22-75 degrees, because a rate that does
    // not scale with travel dominates the direction exactly where the
    // along-path rate is small, which is the failure this whole bound exists to
    // prevent.
    const double escalation = max_projection_error_ > kEpsilon
      ? std::max(1.0, offset_distance / max_projection_error_)
      : 1.0;
    const double lateral_limit = along_speed > kEpsilon
      ? along_speed * join_lateral_ratio_ * escalation
      : join_lateral_floor_;
    const double lateral_speed = std::min(
      {max_speed_, offset_distance / dt,
        std::sqrt(2.0 * max_acceleration_ * offset_distance), lateral_limit});
    desired_velocity = {
      desired_velocity.first + offset.first / offset_distance * lateral_speed,
      desired_velocity.second + offset.second / offset_distance * lateral_speed};
  }
  desired_velocity = limit_norm(desired_velocity, max_speed_);

  const Point2 change = limit_norm(
    {desired_velocity.first - velocity_.first, desired_velocity.second - velocity_.second},
    max_acceleration_ * dt);
  velocity_ = limit_norm(
    {velocity_.first + change.first, velocity_.second + change.second}, max_speed_);
  position_ = Point2{
    position_->first + velocity_.first * dt,
    position_->second + velocity_.second * dt};
  const auto projection = installation_->path->project(*position_);
  if (projection.cross_track > join_limit_ + kEpsilon) {
    position_ = old_position;
    throw std::invalid_argument("path rejoin would leave the projection tolerance band");
  }
  progress_ = projection.along;
  // Complete on geometry alone. The old velocity condition is what forced the
  // stop; the command point is already on the path here, and `speed_` carries
  // the along-path rate straight into the ordinary advance.
  if (projection.cross_track <= join_tolerance_) {
    position_ = projection.point;
    progress_ = projection.along;
    join_target_.reset();
  }
  return *position_;
}

double PathCommandLimiter::next_motion_target(double desired_progress) const
{
  // With corner blending on, only the path end is a stop; bends become speed
  // limits in corner_speed_limit() instead.
  const double length = installation_->path->length();
  if (!corner_blending_) {
    for (const double vertex : installation_->bends) {
      if (progress_ + kEpsilon < vertex && vertex < desired_progress - kEpsilon) {
        return vertex;
      }
    }
  }
  if (progress_ + kEpsilon < length && length < desired_progress - kEpsilon) {
    return length;
  }
  return desired_progress;
}

double PathCommandLimiter::corner_speed(double turn) const
{
  // Junction-deviation model: rounding the corner inside a circle of deviation
  // d from the vertex needs v <= sqrt(a * d * cos(t/2) / (1 - cos(t/2))), where
  // t is the turn away from straight. The published command still rides the
  // polyline exactly -- only the vehicle rounds the corner -- so every existing
  // projection and clearance check applies unchanged to the point sent to PX4.
  const double half = 0.5 * std::clamp(turn, 0.0, kPi);
  const double cos_half = std::cos(half);
  if (cos_half >= 1.0 - kEpsilon) {
    return max_speed_;    // straight through; no limit
  }
  if (cos_half <= kEpsilon) {
    return 0.0;           // a full reversal genuinely has to stop
  }
  return std::min(
    max_speed_,
    std::sqrt(max_acceleration_ * junction_deviation_ * cos_half / (1.0 - cos_half)));
}

double PathCommandLimiter::corner_speed_limit() const
{
  double limit = max_speed_;
  for (std::size_t index = 0; index < installation_->bends.size(); ++index) {
    const double vertex = installation_->bends[index];
    const double vertex_distance = vertex - progress_;
    if (vertex_distance <= kEpsilon) {
      continue;
    }
    const double corner = corner_speed(installation_->bend_turns[index]);
    // Fastest we may go now and still brake to `corner` by the vertex.
    const double reachable =
      std::sqrt(corner * corner + 2.0 * max_acceleration_ * vertex_distance);
    if (reachable < limit) {
      limit = reachable;
    }
    if (corner >= max_speed_ &&
      vertex_distance > max_speed_ * max_speed_ / (2.0 * max_acceleration_))
    {
      break;
    }
  }
  return limit;
}

Point2 PathCommandLimiter::update(
  const Point2 & desired_point, double dt, bool advance,
  const std::optional<Point2> & reference_point)
{
  if (!installation_ || !position_) {
    throw std::runtime_error("no path command has been initialized");
  }
  if (!finite(desired_point) || !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("desired point and dt must be finite, with positive dt");
  }

  const Point2 old_position = *position_;
  if (join_target_) {
    return update_join(dt, advance);
  }
  if (waiting_vertex_) {
    const Point2 vertex = installation_->path->point_at(*waiting_vertex_);
    if (!reference_point || distance(*reference_point, vertex) > corner_tolerance_) {
      speed_ = 0.0;
      velocity_ = {0.0, 0.0};
      return *position_;
    }
    waiting_vertex_.reset();
  }

  if (!advance) {
    // A yaw-alignment pause brakes forward along the route; it cannot produce a
    // shortcut or reverse through a corner.
    double next_speed = std::max(0.0, speed_ - max_acceleration_ * dt);
    const double step = next_speed * dt;
    double next_vertex = installation_->path->length();
    for (std::size_t index = 1; index < installation_->path->cumulative().size(); ++index) {
      const double vertex = installation_->path->cumulative()[index];
      if (vertex > progress_ + kEpsilon) {
        next_vertex = vertex;
        break;
      }
    }
    progress_ = std::min(progress_ + step, next_vertex);
    if (progress_ >= next_vertex - kEpsilon) {
      next_speed = 0.0;
    }
    speed_ = next_speed;
  } else {
    const auto projection = installation_->path->project(desired_point);
    const double desired_progress = std::max(progress_, projection.along);
    const double target_progress = next_motion_target(desired_progress);
    const double remaining = std::max(0.0, target_progress - progress_);
    if (remaining <= kEpsilon) {
      speed_ = 0.0;
    } else {
      // Bound this step and the remaining braking distance. The discrete form
      // prevents overshooting a corner at finite dt.
      const double braking_speed = std::max(
        0.0,
        -max_acceleration_ * dt +
        std::sqrt(
          (max_acceleration_ * dt) * (max_acceleration_ * dt) +
          2.0 * max_acceleration_ * remaining));
      double desired_speed = std::min({max_speed_, remaining / dt, braking_speed});
      if (corner_blending_) {
        desired_speed = std::min(desired_speed, corner_speed_limit());
      }
      const double delta = std::clamp(
        desired_speed - speed_, -max_acceleration_ * dt, max_acceleration_ * dt);
      speed_ = std::clamp(speed_ + delta, 0.0, max_speed_);
      const double step = std::min(remaining, speed_ * dt);
      progress_ += step;
      if (remaining - step <= kEpsilon) {
        progress_ = target_progress;
        speed_ = 0.0;
        for (const double vertex : installation_->bends) {
          if (std::abs(target_progress - vertex) <= kEpsilon) {
            waiting_vertex_ = target_progress;
            break;
          }
        }
      }
    }
  }

  position_ = installation_->path->point_at(progress_);
  velocity_ = {
    (position_->first - old_position.first) / dt,
    (position_->second - old_position.second) / dt};
  return *position_;
}

std::optional<Point2> PathCommandLimiter::advance_tangent() const
{
  if (!installation_) {
    return std::nullopt;
  }
  const double length = installation_->path->length();
  const double step = std::min(1.0e-4, length);
  if (step <= kEpsilon) {
    return std::nullopt;
  }
  // Forward difference from the previewed progress. A central difference would
  // straddle a vertex and return the bisector; the command point is about to
  // travel the outgoing leg, so that is the leg the nose has to face.
  const double back = std::clamp(progress_, 0.0, length - step);
  const Point2 a = installation_->path->point_at(back);
  const Point2 b = installation_->path->point_at(back + step);
  const Point2 delta{b.first - a.first, b.second - a.second};
  const double norm = std::hypot(delta.first, delta.second);
  if (norm <= kEpsilon) {
    return std::nullopt;
  }
  return Point2{delta.first / norm, delta.second / norm};
}

std::optional<Point2> PathCommandLimiter::preview_direction(
  const Point2 & desired_point, double dt, const Point2 & reference_point,
  double min_position_distance) const
{
  if (!finite(reference_point) || !std::isfinite(min_position_distance) ||
    min_position_distance <= 0.0)
  {
    throw std::invalid_argument("heading reference and positive distance must be finite");
  }
  // The installation is immutable and shared. The copy owns all mutable
  // progress/join/corner state, including changes made before an exception.
  auto preview = *this;
  const auto next = preview.update(desired_point, dt, true, reference_point);

  // Heading comes from the route, not from this tick's command velocity.
  //
  // The velocity of a single advancing tick is only the route direction while
  // the command point is already moving along the path. Out of a yaw-alignment
  // brake it is the ACCELERATION direction: along-path gets `a*dt` (0.015 m/s
  // at the flown 0.30 m/s^2 and 20 Hz) while update_join()'s lateral term gets
  // up to `sqrt(2*a*offset)` (0.134 m/s at 3 cm), so a lane change narrower
  // than the drone reads as a right-angle turn. `join_tolerance_` is 3 mm and
  // the planner replaces the path about once a second, so on 2026-09-06 nearly
  // every replacement during an alignment re-pointed yaw sideways and the
  // alignment kept restarting: 39.5 s of a 42.5 s route, 0.295 m flown.
  //
  // The tangent ahead of the previewed progress is where translation actually
  // resumes, and it does not depend on the current speed, so braking, joining
  // and releasing all report the same heading.
  const bool advancing =
    preview.speed_ > kEpsilon || preview.progress_ > progress_ + kEpsilon;
  if (advancing) {
    if (const auto tangent = preview.advance_tangent()) {
      return tangent;
    }
  }
  // Nothing along-path is being commanded -- at the goal, or held off a corner
  // the vehicle has not reached. Face a standing position target if it is far
  // enough to be meaningful, otherwise keep the caller's previous heading.
  const Point2 offset{
    next.first - reference_point.first, next.second - reference_point.second};
  const double magnitude = std::hypot(offset.first, offset.second);
  if (magnitude < min_position_distance) {
    return std::nullopt;
  }
  return Point2{offset.first / magnitude, offset.second / magnitude};
}

Point2 clamp_to_disc(const Point2 & point, const Point2 & center, double radius)
{
  if (!std::isfinite(radius) || radius <= 0.0) {
    throw std::invalid_argument("geofence radius must be finite and positive");
  }
  const Point2 offset{point.first - center.first, point.second - center.second};
  const double offset_distance = std::hypot(offset.first, offset.second);
  if (offset_distance <= radius || offset_distance <= 0.0) {
    return point;
  }
  const double scale = radius / offset_distance;
  return {center.first + offset.first * scale, center.second + offset.second * scale};
}

}  // namespace px4_vio_bridge
