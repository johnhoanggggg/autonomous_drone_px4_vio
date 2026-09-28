#include "px4_vio_bridge/command_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

namespace px4_vio_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

// --- corner fillets (experimental) ------------------------------------------
//
// Strict mode stops the command at every polyline vertex, because carrying
// speed through a sharp vertex steps the velocity vector. On the 2026-09-28
// global flights the planner's paths had 1-2 such bends per metre (median turn
// 27-42 degrees), and the resulting brake/accelerate cycles drove the
// acceleration feedforward to its +-0.30 m/s^2 clip 15-32% of the time.
//
// A fillet replaces a bend with a circular arc drawn as a fine polyline. Each
// arc vertex turns by at most kArcStep, so crossing one at cruise changes the
// velocity secant by v * kArcStep = 0.0017 m/s at 0.20 m/s -- a small fraction
// of the 0.30 m/s^2 * dt budget even at 50 Hz.
constexpr double kArcStep = 0.5 * kPi / 180.0;
/// A vertex at or below this turn is crossed at speed, not stopped at.
constexpr double kSoftTurn = kArcStep + 1.0e-6;
/// Near-reversals keep their stop: a rounded hairpin is a U-turn at a crawl.
constexpr double kMaxFilletTurn = 150.0 * kPi / 180.0;
/// Below this radius the arc speed would be a crawl; keep the stop instead.
constexpr double kMinFilletRadius = 0.01;
/// Shares of the vector acceleration limit. On an arc the velocity turns at
/// v^2/R plus up to v*step/dt for the vertex a tick may straddle; speed
/// changes use the along-path share. sqrt(0.6^2 + 0.6^2) = 0.85 leaves room
/// for the difference between the polyline secant and the ideal arc.
constexpr double kNormalShare = 0.6;
constexpr double kTangentShare = 0.6;
/// The turn between consecutive secants on an arc is v * (dt_prev + dt) / (2R),
/// so a tick shorter than the one before needs more of this tick's budget. The
/// flight adapter's dt is fixed; the arc speed allows for a previous tick up
/// to this much longer.
constexpr double kDtRatioAllowance = 1.5;

Point2 unit(const Point2 & vector)
{
  const double norm = std::hypot(vector.first, vector.second);
  return {vector.first / norm, vector.second / norm};
}

/// Round every roundable bend of `points`; see PathCommandLimiter::FilletCheck.
std::vector<Point2> fillet_corners(
  const std::vector<Point2> & raw, double deviation,
  const PathCommandLimiter::FilletCheck & fillet_check)
{
  // Work on the de-duplicated vertices, exactly as Polyline stores them.
  const auto points = Polyline(raw).points();
  const std::size_t count = points.size();
  if (count < 3) {
    return points;
  }
  std::vector<double> turns(count, 0.0);
  std::vector<bool> candidate(count, false);
  for (std::size_t index = 1; index + 1 < count; ++index) {
    const Point2 in{
      points[index].first - points[index - 1].first,
      points[index].second - points[index - 1].second};
    const Point2 out{
      points[index + 1].first - points[index].first,
      points[index + 1].second - points[index].second};
    turns[index] = std::atan2(
      std::abs(in.first * out.second - in.second * out.first),
      in.first * out.first + in.second * out.second);
    candidate[index] = turns[index] > kSoftTurn && turns[index] < kMaxFilletTurn;
  }

  std::vector<Point2> result{points.front()};
  for (std::size_t index = 1; index + 1 < count; ++index) {
    const Point2 & vertex = points[index];
    if (!candidate[index]) {
      result.push_back(vertex);
      continue;
    }
    const Point2 in_leg{
      vertex.first - points[index - 1].first, vertex.second - points[index - 1].second};
    const Point2 out_leg{
      points[index + 1].first - vertex.first, points[index + 1].second - vertex.second};
    const Point2 in_dir = unit(in_leg);
    const Point2 out_dir = unit(out_leg);
    // A leg shared with another rounded bend is split between them; a leg that
    // ends at the path end or at a kept vertex can be used almost entirely.
    const double in_share = candidate[index - 1] ? 0.5 : 0.95;
    const double out_share = candidate[index + 1] ? 0.5 : 0.95;
    const double max_tangent = std::min(
      in_share * std::hypot(in_leg.first, in_leg.second),
      out_share * std::hypot(out_leg.first, out_leg.second));
    const double half = 0.5 * turns[index];
    const double side =
      in_dir.first * out_dir.second - in_dir.second * out_dir.first > 0.0 ? 1.0 : -1.0;

    bool rounded = false;
    // Halving the deviation is a smaller arc that stays nearer the planner's
    // corner, for when the full one cuts too close to an obstacle.
    for (double trial = deviation; trial >= 0.25 * deviation - kEpsilon; trial *= 0.5) {
      // Junction-deviation geometry: an arc tangent to both legs whose midpoint
      // is `trial` from the vertex.
      double radius = trial * std::cos(half) / (1.0 - std::cos(half));
      double tangent = radius * std::tan(half);
      if (tangent > max_tangent) {
        tangent = max_tangent;
        radius = tangent / std::tan(half);
      }
      if (radius < kMinFilletRadius) {
        break;
      }
      const Point2 start{
        vertex.first - in_dir.first * tangent, vertex.second - in_dir.second * tangent};
      const Point2 end{
        vertex.first + out_dir.first * tangent, vertex.second + out_dir.second * tangent};
      const Point2 normal{-side * in_dir.second, side * in_dir.first};
      const Point2 center{
        start.first + normal.first * radius, start.second + normal.second * radius};
      const double start_angle =
        std::atan2(start.second - center.second, start.first - center.first);
      const int steps = std::max(1, static_cast<int>(std::ceil(turns[index] / kArcStep)));
      std::vector<Point2> arc;
      arc.reserve(static_cast<std::size_t>(steps) + 1);
      arc.push_back(start);
      for (int step = 1; step < steps; ++step) {
        const double angle = start_angle + side * turns[index] * step / steps;
        arc.push_back({center.first + radius * std::cos(angle),
            center.second + radius * std::sin(angle)});
      }
      arc.push_back(end);
      if (fillet_check && fillet_check(arc, {start, vertex, end})) {
        result.insert(result.end(), arc.begin(), arc.end());
        rounded = true;
        break;
      }
    }
    if (!rounded) {
      result.push_back(vertex);
    }
  }
  result.push_back(points.back());
  return result;
}
}  // namespace

std::optional<double> yaw_translation_speed_cap(
  std::optional<double> error, double resume_error, double hold_error,
  double max_speed, const Point2 & velocity, double acceleration, double dt)
{
  if (!error || hold_error <= 0.0 || hold_error <= resume_error) {
    return std::nullopt;
  }
  if (!std::isfinite(*error) || !std::isfinite(resume_error) ||
    !std::isfinite(hold_error) || resume_error < 0.0 ||
    !std::isfinite(max_speed) || max_speed <= 0.0 || !finite(velocity) ||
    !std::isfinite(acceleration) || acceleration <= 0.0 ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    throw std::invalid_argument("invalid yaw translation speed inputs");
  }
  if (std::abs(*error) <= resume_error) {
    return std::nullopt;
  }
  const double scale = std::clamp(
    (hold_error - std::abs(*error)) / (hold_error - resume_error), 0.0, 1.0);
  const double braking_ceiling = std::max(
    0.0, std::hypot(velocity.first, velocity.second) - acceleration * dt);
  return std::min(max_speed, std::max(max_speed * scale, braking_ceiling));
}

std::optional<Point2> route_heading_direction(
  const Polyline & path, double progress, double lookahead)
{
  if (!std::isfinite(progress) || !std::isfinite(lookahead) || lookahead < 0.0) {
    throw std::invalid_argument("heading progress and nonnegative lookahead must be finite");
  }
  const double length = path.length();
  const double step = std::min(1.0e-4, length);
  if (step <= kEpsilon) {
    return std::nullopt;
  }
  const double begin = std::clamp(progress, 0.0, length - step);
  double end = std::min(length, begin + std::max(step, lookahead));
  const auto & points = path.points();
  const auto & cumulative = path.cumulative();
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    if (cumulative[i] <= begin + kEpsilon || cumulative[i] >= end) {
      continue;
    }
    const Point2 a{points[i].first - points[i - 1].first,
      points[i].second - points[i - 1].second};
    const Point2 b{points[i + 1].first - points[i].first,
      points[i + 1].second - points[i].second};
    const double cosine = (a.first * b.first + a.second * b.second) /
      (std::hypot(a.first, a.second) * std::hypot(b.first, b.second));
    if (cosine <= -std::sqrt(0.5)) {
      end = cumulative[i];
      break;
    }
  }
  const auto a = path.point_at(begin);
  const auto b = path.point_at(end);
  Point2 delta{b.first - a.first, b.second - a.second};
  double norm = std::hypot(delta.first, delta.second);
  // A tightly folded path can cancel without a single hairpin vertex.
  if (norm < 0.1 * (end - begin)) {
    const auto near = path.point_at(begin + step);
    delta = {near.first - a.first, near.second - a.second};
    norm = std::hypot(delta.first, delta.second);
  }
  if (norm <= kEpsilon) {
    return std::nullopt;
  }
  return Point2{delta.first / norm, delta.second / norm};
}

void HeadingReference::reset(std::optional<double> heading)
{
  if (heading && !std::isfinite(*heading)) {
    throw std::invalid_argument("heading seed must be finite");
  }
  preview_.reset();
  goal_ = target_ = heading;
}

std::optional<double> HeadingReference::update(
  std::optional<double> preview, double deadband, double max_rate, double dt)
{
  if (!std::isfinite(deadband) || deadband < 0.0 ||
    !std::isfinite(max_rate) || max_rate < 0.0 || !std::isfinite(dt) || dt <= 0.0 ||
    (preview && !std::isfinite(*preview)))
  {
    throw std::invalid_argument("invalid heading reference inputs");
  }
  preview_ = preview;
  goal_ = track_yaw_target(goal_, preview, deadband);
  slew_enabled_ = max_rate > 0.0;
  if (!target_ || !slew_enabled_) {
    target_ = goal_;
  } else if (goal_) {
    target_ = wrap_pi(*target_ + std::clamp(
        wrap_pi(*goal_ - *target_), -max_rate * dt, max_rate * dt));
  }
  return target_;
}

std::optional<double> HeadingReference::alignment_error(double measured) const
{
  if (!std::isfinite(measured) || !target_) {
    return std::nullopt;
  }
  double error = wrap_pi(*target_ - measured);
  if (slew_enabled_) {
    for (const auto heading : {preview_, goal_}) {
      if (heading && std::abs(wrap_pi(*heading - measured)) > std::abs(error)) {
        error = wrap_pi(*heading - measured);
      }
    }
  }
  return error;
}

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
  double suffix_tolerance, bool corner_blending, double junction_deviation,
  bool strict_vector_limits, bool corner_fillets)
: max_speed_(max_speed),
  max_acceleration_(max_acceleration),
  max_projection_error_(max_projection_error),
  corner_tolerance_(corner_tolerance),
  max_entry_error_(max_entry_error),
  max_connector_error_(max_connector_error),
  suffix_tolerance_(suffix_tolerance),
  corner_blending_(corner_blending),
  junction_deviation_(junction_deviation),
  strict_vector_limits_(strict_vector_limits),
  corner_fillets_(corner_fillets)
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
  if (corner_fillets && !strict_vector_limits) {
    throw std::invalid_argument("corner fillets require strict vector limits");
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

void PathCommandLimiter::reexpress(const Correction4 & from, const Correction4 & to)
{
  if (!finite(from) || !finite(to)) {
    throw std::invalid_argument("corrections must be finite");
  }
  if (from == to) {
    return;
  }
  if (installation_) {
    const auto & old_points = installation_->path->points();
    std::vector<Point2> points;
    points.reserve(old_points.size());
    for (const auto & point : old_points) {
      points.push_back(reexpress_point(point, from, to));
    }
    // Deliberately not install(): that resets the join and corner state, which
    // a frame change does not invalidate. Arc length is preserved by the rigid
    // transform, so progress_ carries over untouched.
    auto fingerprint = path_fingerprint(points);
    installation_ = build_installation(
      std::make_shared<const Polyline>(points), std::move(fingerprint), corner_fillets_);
  }
  if (position_) {
    position_ = reexpress_point(*position_, from, to);
  }
  velocity_ = reexpress_vector(velocity_, from, to);
  if (join_target_) {
    join_target_ = reexpress_point(*join_target_, from, to);
  }
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
  std::shared_ptr<const Polyline> path, PathFingerprint fingerprint, bool corner_fillets)
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
      const double turn = std::atan2(std::abs(cross), dot);
      if (corner_fillets && dot > 0.0 && turn <= kSoftTurn) {
        // Crossable at speed. The local radius is that of the circle through
        // the vertex and its shorter leg: exact on a fillet arc, effectively
        // infinite for a single kink between long legs.
        const double shorter = std::min(
          std::hypot(first.first, first.second), std::hypot(second.first, second.second));
        const double chord_turn = 2.0 * std::sin(0.5 * turn);
        installation->soft.push_back(cumulative[index]);
        installation->soft_turns.push_back(turn);
        installation->soft_radii.push_back(
          chord_turn > kEpsilon ? shorter / chord_turn :
          std::numeric_limits<double>::infinity());
        continue;
      }
      installation->bends.push_back(cumulative[index]);
      installation->bend_turns.push_back(turn);
    }
  }
  return installation;
}

double PathCommandLimiter::tangential_acceleration() const
{
  return corner_fillets_ ? kTangentShare * max_acceleration_ : max_acceleration_;
}

double PathCommandLimiter::soft_vertex_speed_limit(double dt) const
{
  if (!corner_fillets_ || installation_->soft.empty()) {
    return max_speed_;
  }
  // Per tick the velocity secant turns by v^2 dt / R on an arc (times the
  // dt-ratio allowance), plus up to v * turn for the vertex a tick may
  // straddle. Keep their sum inside the normal share:
  //   c * v^2 / R + v * turn / dt <= kNormalShare * a.
  const double normal = kNormalShare * max_acceleration_;
  const double along = tangential_acceleration();
  const auto & soft = installation_->soft;
  // A vertex crossed during the previous tick still shapes this tick's secant.
  const double behind = max_speed_ * dt + kEpsilon;
  const double horizon = max_speed_ * max_speed_ / (2.0 * along) + 2.0 * max_speed_ * dt;
  auto index = static_cast<std::size_t>(
    std::lower_bound(soft.begin(), soft.end(), progress_ - behind) - soft.begin());
  double limit = max_speed_;
  for (; index < soft.size(); ++index) {
    const double vertex_distance = soft[index] - progress_;
    if (vertex_distance > horizon) {
      break;
    }
    const double turn = installation_->soft_turns[index];
    const double radius = installation_->soft_radii[index];
    const double rate = turn / dt;
    double vertex_speed = max_speed_;
    if (std::isinf(radius)) {
      if (turn > kEpsilon) {
        vertex_speed = normal / rate;
      }
    } else {
      const double c = 0.5 * (1.0 + kDtRatioAllowance);
      vertex_speed = 0.5 * radius / c *
        (-rate + std::sqrt(rate * rate + 4.0 * c * normal / radius));
    }
    limit = std::min(
      limit,
      std::sqrt(vertex_speed * vertex_speed + 2.0 * along * std::max(0.0, vertex_distance)));
  }
  return limit;
}

double PathCommandLimiter::fillet_progress(
  double target_progress, double dt, const Point2 & old_position) const
{
  // The same contract as strict_progress(): the reachable progress nearest the
  // request, never past the next stop. Across an arc's vertices the secant is
  // not affine in arc length, so search the reachable set numerically. The
  // secant speed can only lie within [|v| - a dt, |v| + a dt], which bounds the
  // interval that can possibly be reachable.
  const double start = progress_;
  const double length = installation_->path->length();
  double stop = length;
  for (const double vertex : installation_->bends) {
    if (vertex > start + kEpsilon) {
      stop = vertex;
      break;
    }
  }
  const double maximum_delta = max_acceleration_ * dt;
  const double speed = std::hypot(velocity_.first, velocity_.second);
  const double upper = std::min(stop, start + (speed + maximum_delta) * dt + 1.0e-9);
  const double target = std::clamp(target_progress, start, upper);
  const auto reachable = [&](double progress) {
      const auto point = installation_->path->point_at(progress);
      const Point2 secant{
        (point.first - old_position.first) / dt, (point.second - old_position.second) / dt};
      return std::hypot(secant.first - velocity_.first, secant.second - velocity_.second) <=
             maximum_delta + 1.0e-10;
    };
  if (reachable(target)) {
    return target;
  }
  constexpr int kSamples = 64;
  std::optional<double> best;
  for (int sample = 0; sample <= kSamples; ++sample) {
    const double progress = start + (upper - start) * sample / kSamples;
    if (reachable(progress) &&
      (!best || std::abs(progress - target) < std::abs(*best - target)))
    {
      best = progress;
    }
  }
  if (!best) {
    throw std::invalid_argument("strict vector limit cannot reach the filleted path");
  }
  // Close the gap to the request: bisect between the reachable sample and the
  // request, keeping the reachable end.
  double good = *best;
  double bad = target;
  for (int iteration = 0; iteration < 40 && std::abs(bad - good) > 1.0e-12; ++iteration) {
    const double middle = 0.5 * (good + bad);
    if (reachable(middle)) {
      good = middle;
    } else {
      bad = middle;
    }
  }
  return good;
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
  const ClearanceCheck & clearance_check,
  const FilletCheck & fillet_check)
{
  // The fingerprint is always of the planner's own points, so an unchanged
  // republish is still recognised when the installed geometry is rounded.
  auto fingerprint = path_fingerprint(points);
  if (installation_ && fingerprint == installation_->fingerprint) {
    return false;
  }
  auto new_path = std::make_shared<const Polyline>(
    corner_fillets_ && fillet_check ?
    fillet_corners(points, junction_deviation_, fillet_check) : points);

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
    if (offset && !strict_vector_limits_) {
      // Identical geometry ahead: keep the command point and its speed and
      // only renumber the arc length it is measured against.
      std::optional<double> waiting;
      if (waiting_vertex_ && *waiting_vertex_ + *offset >= -kEpsilon) {
        waiting = *waiting_vertex_ + *offset;
      }
      install(
        build_installation(new_path, std::move(fingerprint), corner_fillets_),
        progress_ + *offset);
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

  install(build_installation(new_path, std::move(fingerprint), corner_fillets_), projection.along);
  // Keep the exact previously-published point. If the path is offset, rejoin it
  // through the bounded band; snapping directly to projection.point would
  // bypass the speed limit.
  position_ = anchor;
  bool tangent_requires_join = false;
  if (strict_vector_limits_ && projection.cross_track <= kEpsilon) {
    const Point2 tangent = segment_tangent(projection.along);
    const double tangent_speed =
      velocity_.first * tangent.first + velocity_.second * tangent.second;
    const Point2 tangent_velocity{
      tangent.first * std::max(0.0, tangent_speed),
      tangent.second * std::max(0.0, tangent_speed)};
    tangent_requires_join = std::hypot(
      velocity_.first - tangent_velocity.first,
      velocity_.second - tangent_velocity.second) > kEpsilon;
  }
  // Strict mode never snaps even a sub-tolerance replacement onto its new
  // path. It must either already be the same point or pass through the bounded
  // join state, where the caller's swept-chord clearance gate still applies.
  const bool open_join = projection.cross_track > join_tolerance_ ||
    (strict_vector_limits_ &&
    (projection.cross_track > kEpsilon || tangent_requires_join));
  if (open_join) {
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

double PathCommandLimiter::speed_ceiling(std::optional<double> speed_cap) const
{
  if (!speed_cap) {
    return max_speed_;
  }
  if (!std::isfinite(*speed_cap) || *speed_cap < 0.0) {
    throw std::invalid_argument("speed cap must be finite and non-negative");
  }
  return std::min(max_speed_, *speed_cap);
}

double PathCommandLimiter::along_path_speed(
  double along, double desired_progress, double dt, bool advance, double ceiling)
{
  // The same speed machinery the on-path advance uses, factored out so a join
  // is braked by corners and by the path end exactly as ordinary motion is.
  const double acceleration = tangential_acceleration();
  if (!advance) {
    speed_ = std::max(0.0, speed_ - acceleration * dt);
    return speed_;
  }
  const double target_progress = next_motion_target(
    std::max(along, std::min(desired_progress, installation_->path->length())));
  const double remaining = std::max(0.0, target_progress - along);
  if (remaining <= kEpsilon) {
    speed_ = 0.0;
    return 0.0;
  }
  const double braking_speed = std::max(
    0.0,
    -acceleration * dt +
    std::sqrt(
      (acceleration * dt) * (acceleration * dt) +
      2.0 * acceleration * remaining));
  double desired_speed = std::min({ceiling, remaining / dt, braking_speed});
  if (corner_blending_) {
    desired_speed = std::min(desired_speed, corner_speed_limit());
  }
  desired_speed = std::min(desired_speed, soft_vertex_speed_limit(dt));
  const double delta = std::clamp(
    desired_speed - speed_, -acceleration * dt, acceleration * dt);
  // The cap is a desired rate. Clamping the state to a newly lowered cap
  // would undo the acceleration bound in delta.
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

Point2 PathCommandLimiter::segment_tangent(double along) const
{
  const auto & points = installation_->path->points();
  const auto & cumulative = installation_->path->cumulative();
  if (points.size() < 2) {
    return {0.0, 0.0};
  }
  const double clamped = std::clamp(along, 0.0, installation_->path->length());
  for (std::size_t index = 0; index + 1 < points.size(); ++index) {
    if (clamped < cumulative[index + 1] - kEpsilon || index + 2 == points.size()) {
      const Point2 delta{
        points[index + 1].first - points[index].first,
        points[index + 1].second - points[index].second};
      const double norm = std::hypot(delta.first, delta.second);
      if (norm > kEpsilon) {
        return {delta.first / norm, delta.second / norm};
      }
    }
  }
  return {0.0, 0.0};
}

double PathCommandLimiter::strict_progress(
  double target_progress, double dt, const Point2 & old_position) const
{
  if (corner_fillets_) {
    return fillet_progress(target_progress, dt, old_position);
  }
  const double start = progress_;
  const double target = std::clamp(
    target_progress, start, installation_->path->length());
  const double requested = target - start;
  double planned = requested;
  const double maximum_delta = max_acceleration_ * dt;
  double segment_end = installation_->path->length();
  // Collinear waypoint subdivisions are not braking boundaries.
  for (const double vertex : installation_->bends) {
    if (vertex > start + kEpsilon) {
      segment_end = vertex;
      break;
    }
  }
  const double segment_remaining = std::max(0.0, segment_end - start);
  const auto candidate_velocity = [&](double step) {
      const auto point = installation_->path->point_at(start + step);
      return Point2{
        (point.first - old_position.first) / dt,
        (point.second - old_position.second) / dt};
    };
  const auto reachable = [&](const Point2 & candidate) {
      return std::hypot(
        candidate.first - velocity_.first, candidate.second - velocity_.second) <=
        maximum_delta + 1.0e-10;
    };

  bool brake_without_target = false;
  if (planned <= kEpsilon) {
    if (reachable(candidate_velocity(0.0))) {
      return start;
    }
    // A newly lowered cap or a follower pause can ask for zero path progress
    // while the current vector still needs a braking tick.  Holding the exact
    // path point would require an acceleration jump; use the nearest reachable
    // secant on this segment instead.  Crossing a bend here is impossible to
    // validate without a fillet, so that case remains fail-closed below.
    planned = segment_remaining;
    brake_without_target = true;
  }

  const auto candidate = candidate_velocity(planned);
  if (!brake_without_target && reachable(candidate)) {
    return target;
  }

  // On one segment the velocity secant is affine in arc length, so the
  // reachable set is the interval cut out by one quadratic inequality. Strict
  // mode never intentionally crosses a bend: next_motion_target() has already
  // stopped the requested interval at the next vertex.
  const Point2 base = candidate_velocity(0.0);
  const Point2 tangent = segment_tangent(start);
  const Point2 direction{tangent.first / dt, tangent.second / dt};
  const Point2 relative{
    base.first - velocity_.first, base.second - velocity_.second};
  const double aa = direction.first * direction.first + direction.second * direction.second;
  const double bb = 2.0 * (relative.first * direction.first + relative.second * direction.second);
  const double cc = relative.first * relative.first + relative.second * relative.second -
    maximum_delta * maximum_delta;
  if (aa <= kEpsilon) {
    throw std::invalid_argument("strict vector limit has no path tangent");
  }
  const double discriminant = bb * bb - 4.0 * aa * cc;
  if (discriminant < -1.0e-12) {
    throw std::invalid_argument("strict vector limit cannot reach the path");
  }
  const double root = std::sqrt(std::max(0.0, discriminant));
  const double low = std::max(0.0, (-bb - root) / (2.0 * aa));
  const double high = std::min(planned, (-bb + root) / (2.0 * aa));
  if (high < low - 1.0e-10 || high < -1.0e-10 ||
    !reachable(candidate_velocity(std::max(0.0, high))))
  {
    if (!brake_without_target && requested < low - 1.0e-10) {
      const bool target_is_geometry_boundary =
        std::abs(target - segment_end) <= 1.0e-8;
      if (!target_is_geometry_boundary && low <= segment_remaining + 1.0e-10) {
        return start + low;
      }
    }
    throw std::invalid_argument("strict vector limit cannot reach the requested path step");
  }
  if (brake_without_target) {
    if (low > segment_remaining + 1.0e-10) {
      throw std::invalid_argument("strict vector limit cannot brake before the path bend");
    }
    return start + std::max(0.0, low);
  }
  return start + std::max(0.0, high);
}

Point2 PathCommandLimiter::update_join(
  const Point2 & desired_point, double dt, bool advance, double ceiling)
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
  const Point2 old_velocity = velocity_;
  const auto start = installation_->path->project(*position_);

  const auto desired_projection = installation_->path->project(desired_point);
  const double along_speed = along_path_speed(
    start.along,
    strict_vector_limits_ ? desired_projection.along : installation_->path->length(),
    dt, advance, ceiling);
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
      {ceiling, offset_distance / dt,
        std::sqrt(2.0 * max_acceleration_ * offset_distance), lateral_limit});
    desired_velocity = {
      desired_velocity.first + offset.first / offset_distance * lateral_speed,
      desired_velocity.second + offset.second / offset_distance * lateral_speed};
  }
  desired_velocity = limit_norm(desired_velocity, ceiling);

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
  // In strict mode a geometric crossing is not enough: snapping the point to
  // the path would make the published velocity differ from its position
  // secant, and carrying a residual normal rate into the ordinary path update
  // would step the full velocity vector. Complete only when the projection
  // handoff itself is reachable and the next tangent is compatible. Otherwise
  // remain in the bounded join; the caller's swept-chord clearance gate still
  // validates every such tick.
  if (projection.cross_track <= join_tolerance_) {
    const Point2 tangent = segment_tangent(projection.along);
    const Point2 handoff_delta{
      projection.point.first - old_position.first,
      projection.point.second - old_position.second};
    const Point2 handoff_velocity{handoff_delta.first / dt, handoff_delta.second / dt};
    const double handoff_speed = std::max(
      0.0, handoff_velocity.first * tangent.first + handoff_velocity.second * tangent.second);
    const Point2 tangent_velocity{
      tangent.first * handoff_speed, tangent.second * handoff_speed};
    const bool handoff_reachable = std::hypot(
      handoff_velocity.first - old_velocity.first,
      handoff_velocity.second - old_velocity.second) <= max_acceleration_ * dt + 1.0e-10;
    const bool tangent_compatible = std::hypot(
      handoff_velocity.first - tangent_velocity.first,
      handoff_velocity.second - tangent_velocity.second) <= max_acceleration_ * dt + 1.0e-10;
    const bool handoff_within_speed =
      std::hypot(handoff_velocity.first, handoff_velocity.second) <= max_speed_ + kEpsilon;
    bool ordinary_handoff_reachable = true;
    if (strict_vector_limits_ && handoff_reachable && tangent_compatible && handoff_within_speed) {
      // The handoff tick itself is valid, but the next on-path secant may not
      // be.  In particular, a positive residual normal component consumes part
      // of the next tick's acceleration disk while the ordinary path update
      // also tries to change speed.  Preview that exact next update from the
      // handoff state; leave the bounded join open when it would reject.  The
      // next join tick can remove the residual at the same vector limit.
      auto next = *this;
      next.position_ = projection.point;
      next.progress_ = projection.along;
      next.velocity_ = handoff_velocity;
      next.speed_ = handoff_speed;
      next.join_target_.reset();
      next.waiting_vertex_.reset();
      try {
        (void)next.update(desired_point, dt, advance, std::nullopt, ceiling);
      } catch (const std::invalid_argument &) {
        ordinary_handoff_reachable = false;
      }
    }
    if (!strict_vector_limits_ ||
      (handoff_reachable && tangent_compatible && handoff_within_speed &&
      ordinary_handoff_reachable))
    {
      position_ = projection.point;
      progress_ = projection.along;
      if (strict_vector_limits_) {
        velocity_ = handoff_velocity;
        speed_ = handoff_speed;
      }
      join_target_.reset();
    }
  }
  return *position_;
}

double PathCommandLimiter::next_motion_target(double desired_progress) const
{
  // With corner blending on, only the path end is a stop; bends become speed
  // limits in corner_speed_limit() instead.
  const double length = installation_->path->length();
  // Strict vector mode deliberately stops at every sharp polyline vertex.
  // Carrying speed through an unfilleted bend would require a velocity vector
  // that is not the derivative of the exact, clearance-checked command path.
  if (!corner_blending_ || strict_vector_limits_) {
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
  const std::optional<Point2> & reference_point, std::optional<double> speed_cap)
{
  if (!installation_ || !position_) {
    throw std::runtime_error("no path command has been initialized");
  }
  if (!finite(desired_point) || !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("desired point and dt must be finite, with positive dt");
  }
  const double ceiling = speed_ceiling(speed_cap);

  const Point2 old_position = *position_;
  const Point2 old_velocity = velocity_;
  if (join_target_) {
    return update_join(desired_point, dt, advance, ceiling);
  }
  if (waiting_vertex_) {
    const Point2 vertex = installation_->path->point_at(*waiting_vertex_);
    if (!reference_point || distance(*reference_point, vertex) > corner_tolerance_) {
      speed_ = 0.0;
      if (strict_vector_limits_ && std::hypot(velocity_.first, velocity_.second) >
        max_acceleration_ * dt + 1.0e-10)
      {
        throw std::invalid_argument("strict vector limit cannot enter a corner hold");
      }
      velocity_ = {0.0, 0.0};
      return *position_;
    }
    waiting_vertex_.reset();
  }

  const double acceleration = tangential_acceleration();
  if (!advance) {
    // A yaw-alignment pause brakes forward along the route; it cannot produce a
    // shortcut or reverse through a corner.
    double next_speed = std::max(0.0, speed_ - acceleration * dt);
    const double step = next_speed * dt;
    double next_vertex = installation_->path->length();
    // A filleted arc is crossable while braking; only a remaining stop is not.
    const auto & vertices = corner_fillets_ ?
      installation_->bends : installation_->path->cumulative();
    for (const double vertex : vertices) {
      if (vertex > progress_ + kEpsilon) {
        next_vertex = vertex;
        break;
      }
    }
    const double planned_progress = std::min(progress_ + step, next_vertex);
    progress_ = strict_vector_limits_ ?
      strict_progress(planned_progress, dt, old_position) : planned_progress;
    if (progress_ >= next_vertex - kEpsilon) {
      next_speed = 0.0;
    }
    if (strict_vector_limits_) {
      next_speed = std::max(0.0, progress_ - (planned_progress - step)) / dt;
    }
    speed_ = next_speed;
  } else {
    const auto projection = installation_->path->project(desired_point);
    const double desired_progress = std::max(progress_, projection.along);
    const double target_progress = next_motion_target(desired_progress);
    const double remaining = std::max(0.0, target_progress - progress_);
    if (remaining <= kEpsilon) {
      speed_ = 0.0;
      if (strict_vector_limits_) {
        // A scalar target can fall behind a still-moving command (for example
        // when a carrot retreats during recovery).  Let strict_progress()
        // choose the nearest acceleration-reachable on-path braking secant
        // instead of forcing an instantaneous zero vector.
        const double previous_progress = progress_;
        progress_ = strict_progress(progress_, dt, old_position);
        if (progress_ > previous_progress + kEpsilon) {
          speed_ = (progress_ - previous_progress) / dt;
        }
      }
    } else {
      // Bound this step and the remaining braking distance. The discrete form
      // prevents overshooting a corner at finite dt.
      const double braking_speed = std::max(
        0.0,
        -acceleration * dt +
        std::sqrt(
          (acceleration * dt) * (acceleration * dt) +
          2.0 * acceleration * remaining));
      double desired_speed = std::min({ceiling, remaining / dt, braking_speed});
      if (corner_blending_) {
        desired_speed = std::min(desired_speed, corner_speed_limit());
      }
      desired_speed = std::min(desired_speed, soft_vertex_speed_limit(dt));
      const double delta = std::clamp(
        desired_speed - speed_, -acceleration * dt, acceleration * dt);
      // A follower ride-out requests 0.06 m/s from a 0.20 m/s cruise.
      // Reach that rate through the acceleration-limited delta, rather than
      // stepping the velocity and then clipping only its acceleration FF.
      speed_ = std::clamp(speed_ + delta, 0.0, max_speed_);
      const double step = std::min(remaining, speed_ * dt);
      const double planned_progress = progress_ + step;
      const double previous_progress = progress_;
      progress_ = strict_vector_limits_ ?
        strict_progress(planned_progress, dt, old_position) : planned_progress;
      if (strict_vector_limits_) {
        speed_ = std::max(0.0, progress_ - previous_progress) / dt;
      }
      if (progress_ <= planned_progress + kEpsilon &&
        remaining - (progress_ - (planned_progress - step)) <= kEpsilon &&
        progress_ >= target_progress - kEpsilon)
      {
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
  if (strict_vector_limits_) {
    if (std::hypot(velocity_.first - old_velocity.first,
      velocity_.second - old_velocity.second) > max_acceleration_ * dt + 1.0e-9)
    {
      throw std::invalid_argument("strict vector limit was exceeded by the path secant");
    }
  }
  return *position_;
}

std::optional<Point2> PathCommandLimiter::advance_tangent(double lookahead) const
{
  if (!installation_) {
    return std::nullopt;
  }
  return route_heading_direction(*installation_->path, progress_, lookahead);
}

std::optional<Point2> PathCommandLimiter::preview_direction(
  const Point2 & desired_point, double dt, const Point2 & reference_point,
  double min_position_distance, double heading_lookahead) const
{
  if (!finite(reference_point) || !std::isfinite(min_position_distance) ||
    min_position_distance <= 0.0 || !std::isfinite(heading_lookahead) ||
    heading_lookahead < 0.0)
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
    if (const auto tangent = preview.advance_tangent(heading_lookahead)) {
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
