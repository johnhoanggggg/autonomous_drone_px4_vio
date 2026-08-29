#include "px4_vio_bridge/route_follower.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>


namespace px4_vio_bridge
{
namespace
{

// segment_has_clearance()'s acceptance epsilon. Reused verbatim so the normal
// (non-escaping) branch of the escape validator is the same predicate.
constexpr double kClearanceEpsilon = 1.0e-9;

Point2 correction_delta(const Correction4 & first, const Correction4 & second)
{
  const double dx = second[0] - first[0];
  const double dy = second[1] - first[1];
  const double dz = second[2] - first[2];
  return {
    std::sqrt(dx * dx + dy * dy + dz * dz),
    std::abs(wrap_pi_mod(second[3] - first[3]))};
}

Correction4 blend_correction(
  const Correction4 & first, const Correction4 & second, double fraction)
{
  return {
    first[0] + fraction * (second[0] - first[0]),
    first[1] + fraction * (second[1] - first[1]),
    first[2] + fraction * (second[2] - first[2]),
    wrap_pi_mod(first[3] + fraction * wrap_pi_mod(second[3] - first[3]))};
}

}  // namespace

bool requested_goal_reached(const std::string & follower_status, bool goal_terminal)
{
  return follower_status == "GOAL_REACHED" && goal_terminal;
}

bool command_chord_admissible(
  const ClearanceProbe & probe, const Point2 & pose, const Point2 & target,
  const ClearanceEscapeLimits & limits, double start_clearance,
  ChordRole role, double * end_clearance)
{
  if (!probe || !finite(pose) || !finite(target) ||
    !std::isfinite(limits.required_clearance) || limits.required_clearance < 0.0 ||
    !std::isfinite(start_clearance))
  {
    return false;
  }
  const auto chord = probe(pose, target);
  if (!chord.has_value()) {
    return false;
  }
  if (start_clearance + kClearanceEpsilon >= limits.required_clearance) {
    // Full envelope: identical to segment_has_clearance(pose, target, required).
    if (end_clearance != nullptr) {
      *end_clearance = *chord;
    }
    return *chord + kClearanceEpsilon >= limits.required_clearance;
  }
  // Escape. The chord may never take the vehicle closer to an occupied cell
  // than it already is -- a target that is safer only at its endpoint but
  // brushes the obstacle on the way is rejected here, not smoothed over.
  if (*chord + limits.tolerance < start_clearance) {
    return false;
  }
  const auto endpoint = probe(target, target);
  if (!endpoint.has_value()) {
    return false;
  }
  if (end_clearance != nullptr) {
    *end_clearance = *endpoint;
  }
  if (role == ChordRole::IntermediateCarrot) {
    // A partial step towards a target that already proved it improves. The
    // non-worsening test above is the whole safety requirement; asking a
    // millimetre-scale first step for a centimetre of gain would forbid every
    // escape from starting.
    return true;
  }
  // Material progress, so following the same unsafe contour sideways for ever
  // cannot pass as recovery. Full clearance is always enough.
  const auto goal = std::min(
    limits.required_clearance, start_clearance + limits.minimum_improvement);
  return *endpoint + limits.tolerance >= goal;
}

std::optional<LookaheadSelection> select_safe_lookahead(
  const Polyline & path, const Point2 & pose, const ClearanceProbe & probe,
  const ClearanceEscapeLimits & limits,
  double lookahead, double lookahead_step, double min_lookahead)
{
  if (!probe || !finite(pose) || !std::isfinite(lookahead) ||
    !std::isfinite(lookahead_step) || lookahead_step <= 0.0 ||
    !std::isfinite(min_lookahead))
  {
    return std::nullopt;
  }
  const auto start = probe(pose, pose);
  if (!start.has_value()) {
    // Unknown or outside-map space is blocked, not "far from an obstacle".
    return std::nullopt;
  }
  const bool escaping = *start + kClearanceEpsilon < limits.required_clearance;
  const auto projection = path.project(pose);
  for (double candidate = lookahead;
    candidate + 1.0e-12 >= min_lookahead; candidate -= lookahead_step)
  {
    const auto target = path.point_at(projection.along + candidate);
    double end_clearance = 0.0;
    if (command_chord_admissible(
        probe, pose, target, limits, *start, ChordRole::Target, &end_clearance))
    {
      return LookaheadSelection{candidate, escaping, *start, end_clearance};
    }
  }
  return std::nullopt;
}

bool defer_path_for_correction(bool episode_pending, bool have_route)
{
  return episode_pending && have_route;
}

CorrectionReplanGate::CorrectionReplanGate(
  double translation_trigger, double yaw_trigger, double filter_time_constant,
  double material_translation, double material_yaw, double quiet_time, double rearm_guard)
: translation_trigger_(translation_trigger),
  yaw_trigger_(yaw_trigger),
  filter_time_constant_(filter_time_constant),
  material_translation_(material_translation),
  material_yaw_(material_yaw),
  quiet_time_(quiet_time),
  rearm_guard_(rearm_guard)
{
  const std::array<double, 6> values{
    translation_trigger_, yaw_trigger_, filter_time_constant_, material_translation_,
    material_yaw_, quiet_time_};
  if (std::any_of(values.begin(), values.end(), [](double value) {
      return !std::isfinite(value) || value <= 0.0;
    }))
  {
    throw std::invalid_argument("correction gate limits must be finite and positive");
  }
  // The guard exists only to stop one settling episode from immediately
  // reopening on its own residual, so zero -- re-arm at once -- is legal.
  if (!std::isfinite(rearm_guard_) || rearm_guard_ < 0.0) {
    throw std::invalid_argument("correction rearm guard must be finite and non-negative");
  }
}

void CorrectionReplanGate::restart_receipts()
{
  path_after_change_ = false;
  path_generation_after_change_ = false;
  required_map_generation_.reset();
}

bool CorrectionReplanGate::observe(const Correction4 & correction, double now)
{
  if (!finite(correction) || !std::isfinite(now)) {
    return false;
  }
  if (!filtered_ || !last_observation_ || now < *last_observation_) {
    filtered_ = correction;
    baseline_ = correction;
    material_reference_ = correction;
    last_raw_ = correction;
    last_observation_ = now;
    return false;
  }

  const double dt = now - *last_observation_;
  last_observation_ = now;
  last_raw_ = correction;
  const double fraction = 1.0 - std::exp(-dt / filter_time_constant_);
  filtered_ = blend_correction(*filtered_, correction, fraction);

  if (pending_) {
    // Every further material step extends the same episode. This is what a
    // blind cooldown could not do: a one-second run of alternating 5 cm steps
    // keeps restarting the quiet timer instead of passing through unobserved.
    //
    // Raw against raw, deliberately. Asking the filtered value whether the
    // correction is still moving answers "yes" for as long as the filter takes
    // to converge, which after a 333 mm loop closure is well over a second of
    // events the correction never actually produced.
    const auto [translation, yaw] = correction_delta(*material_reference_, correction);
    if (translation > material_translation_ || yaw > material_yaw_) {
      material_reference_ = correction;
      last_material_change_ = now;
      restart_receipts();
    }
    return false;
  }

  if (now < rearm_until_) {
    return false;
  }
  const auto [translation, yaw] = correction_delta(*baseline_, *filtered_);
  if (translation <= translation_trigger_ && yaw <= yaw_trigger_) {
    return false;
  }
  pending_ = true;
  ++epoch_;
  material_reference_ = correction;
  last_material_change_ = now;
  restart_receipts();
  last_trigger_delta_ = {translation, yaw};
  return true;
}

void CorrectionReplanGate::map_received(std::int64_t map_generation, double now)
{
  if (!std::isfinite(now)) {
    return;
  }
  if (pending_ && !required_map_generation_.has_value() && now >= last_material_change_) {
    required_map_generation_ = map_generation;
  }
}

void CorrectionReplanGate::path_received(double now)
{
  if (pending_ && std::isfinite(now) && now >= last_material_change_) {
    path_after_change_ = true;
  }
}

void CorrectionReplanGate::path_map_generation(std::int64_t map_generation, double now)
{
  if (!std::isfinite(now)) {
    return;
  }
  generation_pairing_seen_ = true;
  if (pending_ && required_map_generation_.has_value() &&
    map_generation >= *required_map_generation_)
  {
    path_generation_after_change_ = true;
  }
}

bool CorrectionReplanGate::waiting(double now)
{
  // A planner that publishes no generation telemetry (the legacy Python node)
  // leaves only the receipt-time rule. Never deadlock the follower on a
  // pairing the producer cannot supply.
  const bool paired = path_generation_after_change_ || !generation_pairing_seen_;
  if (pending_ && path_after_change_ && paired &&
    now - last_material_change_ >= quiet_time_)
  {
    pending_ = false;
    // Snap the filter to the settled value rather than baselining on it
    // mid-convergence. The episode only reaches here because the raw
    // correction has been still for quiet_time, so the raw value IS the
    // settled one -- while `filtered_` may still be most of a time constant
    // behind it. Baselining on the lagging value made the filter's own
    // catch-up cross the trigger threshold and open a second, phantom episode
    // immediately after the first.
    if (last_raw_) {
      filtered_ = last_raw_;
      baseline_ = last_raw_;
    } else {
      baseline_ = filtered_;
    }
    rearm_until_ = now + rearm_guard_;
  }
  return pending_;
}

std::string CorrectionReplanGate::pending_detail(double now) const
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(2);
  stream << "epoch=" << epoch_ << " quiet=" << (now - last_material_change_) << "/"
         << quiet_time_ << "s path=" << (path_after_change_ ? 1 : 0);
  if (generation_pairing_seen_) {
    stream << " map_generation=";
    if (required_map_generation_.has_value()) {
      stream << *required_map_generation_;
    } else {
      stream << "none";
    }
    stream << " path_generation=" << (path_generation_after_change_ ? 1 : 0);
  }
  return stream.str();
}

PositionRouteFollower::PositionRouteFollower(
  double lookahead, double max_carrot_speed, double max_carrot_acceleration,
  double max_cross_track, double cross_track_resume, double cross_track_recovery_time,
  double arrival_tolerance, double arrival_release_tolerance)
: lookahead_(lookahead),
  max_carrot_speed_(max_carrot_speed),
  max_carrot_acceleration_(max_carrot_acceleration),
  max_cross_track_(max_cross_track),
  cross_track_resume_(cross_track_resume),
  cross_track_recovery_time_(cross_track_recovery_time),
  arrival_tolerance_(arrival_tolerance),
  arrival_release_tolerance_(arrival_release_tolerance)
{
  const std::array<double, 8> limits{
    lookahead_, max_carrot_speed_, max_carrot_acceleration_, max_cross_track_,
    cross_track_resume_, cross_track_recovery_time_, arrival_tolerance_,
    arrival_release_tolerance_};
  if (std::any_of(limits.begin(), limits.end(), [](double value) {
      return !std::isfinite(value) || value <= 0.0;
    }))
  {
    throw std::invalid_argument("follower limits must be positive");
  }
  if (cross_track_resume_ >= max_cross_track_) {
    throw std::invalid_argument("cross_track_resume must be below max_cross_track");
  }
  if (arrival_release_tolerance_ < arrival_tolerance_) {
    throw std::invalid_argument(
            "arrival_release_tolerance must be at least arrival_tolerance");
  }
}

Point2 PositionRouteFollower::commanded_displacement() const
{
  return vio_displacement_to_map(commanded_displacement_vio_, correction_[3]);
}

Point2 PositionRouteFollower::command_velocity() const
{
  return vio_displacement_to_map(command_velocity_vio_, correction_[3]);
}

void PositionRouteFollower::render(const Correction4 & correction)
{
  if (!finite(correction)) {
    throw std::invalid_argument("correction must be finite");
  }
  const bool same = correction_ == correction;
  correction_ = correction;
  if (vio_points_.empty()) {
    path_.reset();
    return;
  }
  if (same && path_) {
    return;
  }
  std::vector<Point2> map_points;
  map_points.reserve(vio_points_.size());
  for (const auto & point : vio_points_) {
    map_points.push_back(vio_point_to_map(point, correction));
  }
  path_ = std::make_unique<Polyline>(map_points);
}

void PositionRouteFollower::clear_path()
{
  path_.reset();
  vio_points_.clear();
  fingerprint_.clear();
  path_progress_ = 0.0;
  command_velocity_vio_ = {0.0, 0.0};
  at_goal_ = false;
}

void PositionRouteFollower::reset_route_progress()
{
  progress_ = 0.0;
  cross_track_latched_ = false;
  cross_track_fault_generation_ = 0;
  cross_track_recovery_elapsed_ = 0.0;
  at_goal_ = false;
  if (path_) {
    path_progress_ = 0.0;
  }
}

void PositionRouteFollower::interrupt_cross_track_recovery()
{
  if (cross_track_latched_) {
    cross_track_recovery_elapsed_ = 0.0;
    command_velocity_vio_ = {0.0, 0.0};
  }
}

void PositionRouteFollower::hold_command()
{
  commanded_displacement_vio_ = {0.0, 0.0};
  command_velocity_vio_ = {0.0, 0.0};
  at_goal_ = false;
}

void PositionRouteFollower::set_escape(bool escaping, bool stale_command_admissible)
{
  if (escaping && !escaping_ && !stale_command_admissible) {
    // Entering the escape with a command that no longer passes the escape
    // predicate: it must not survive. Route progress is deliberately untouched.
    commanded_displacement_vio_ = {0.0, 0.0};
    command_velocity_vio_ = {0.0, 0.0};
    at_goal_ = false;
  }
  escaping_ = escaping;
}

bool PositionRouteFollower::set_path(
  const std::vector<Point2> & points, const Point2 & pose, const Correction4 & correction)
{
  if (!finite(correction)) {
    throw std::invalid_argument("correction must be finite");
  }
  std::vector<Point2> vio_points;
  vio_points.reserve(points.size());
  for (const auto & point : points) {
    if (!finite(point)) {
      throw std::invalid_argument("path contains a non-finite point");
    }
    vio_points.push_back(map_point_to_vio(point, correction));
  }
  // Fingerprinting the canonical VIO coordinates is what makes re-publishing
  // the same physical route under a new correction a no-op rather than a new
  // semantic generation.
  const auto fingerprint = path_fingerprint(vio_points);
  if (path_ && fingerprint == fingerprint_) {
    render(correction);
    return false;
  }
  std::vector<Point2> map_points;
  map_points.reserve(vio_points.size());
  for (const auto & point : vio_points) {
    map_points.push_back(vio_point_to_map(point, correction));
  }
  auto path = std::make_unique<Polyline>(map_points);
  path_progress_ = path->project(pose).along;
  vio_points_ = std::move(vio_points);
  correction_ = correction;
  path_ = std::move(path);
  fingerprint_ = fingerprint;
  ++generation_;
  return true;
}

FollowResult PositionRouteFollower::update(
  const Point2 & pose, double dt, std::optional<double> lookahead,
  const CommandValidator & command_validator, const Correction4 & correction)
{
  if (vio_points_.empty()) {
    throw std::runtime_error("no path");
  }
  if (!finite(pose) || !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("pose and dt must be finite, and dt positive");
  }
  const double effective_lookahead = lookahead.value_or(lookahead_);
  if (!std::isfinite(effective_lookahead) || effective_lookahead < 0.0) {
    throw std::invalid_argument("lookahead must be finite and non-negative");
  }
  // Re-express the route in the map solution the caller's pose belongs to. A
  // correction-only change moves route and pose together, so it cannot show up
  // as cross-track, a new generation or a progress jump.
  render(correction);

  Point2 commanded_displacement = this->commanded_displacement();
  Point2 command_velocity = this->command_velocity();
  const auto store = [this](const Point2 & displacement, const Point2 & velocity) {
      commanded_displacement_vio_ = map_displacement_to_vio(displacement, correction_[3]);
      command_velocity_vio_ = map_displacement_to_vio(velocity, correction_[3]);
    };

  const auto projection = path_->project(pose);
  const double candidate_progress = std::max(path_progress_, projection.along);
  double remaining = std::max(0.0, path_->length() - candidate_progress);
  const Point2 desired = path_->point_at(candidate_progress + effective_lookahead);
  const Point2 desired_offset{desired.first - pose.first, desired.second - pose.second};

  if (projection.cross_track > max_cross_track_) {
    if (!cross_track_latched_) {
      cross_track_latched_ = true;
      cross_track_fault_generation_ = generation_;
    }
    cross_track_recovery_elapsed_ = 0.0;
    command_velocity = {0.0, 0.0};
    store(commanded_displacement, command_velocity);
    const Point2 carrot{
      pose.first + commanded_displacement.first,
      pose.second + commanded_displacement.second};
    return {
      "CROSS_TRACK_EXCEEDED", false, desired, carrot, commanded_displacement,
      commanded_displacement_vio_, path_progress_, progress_, remaining,
      projection.cross_track, generation_};
  }

  if (cross_track_latched_) {
    command_velocity = {0.0, 0.0};
    const bool fresh_path = generation_ > cross_track_fault_generation_;
    std::string status;
    if (!fresh_path || projection.cross_track > cross_track_resume_) {
      cross_track_recovery_elapsed_ = 0.0;
      status = fresh_path ? "CROSS_TRACK_HOLD" : "CROSS_TRACK_HOLD_WAITING_FOR_PATH";
    } else {
      cross_track_recovery_elapsed_ += dt;
      status = "CROSS_TRACK_RECOVERING";
    }
    const Point2 carrot{
      pose.first + commanded_displacement.first,
      pose.second + commanded_displacement.second};
    if (cross_track_recovery_elapsed_ + 1.0e-12 < cross_track_recovery_time_) {
      store(commanded_displacement, command_velocity);
      return {
        status, false, desired, carrot, commanded_displacement,
        commanded_displacement_vio_, path_progress_, progress_, remaining,
        projection.cross_track, generation_};
    }
    cross_track_latched_ = false;
    cross_track_fault_generation_ = 0;
    cross_track_recovery_elapsed_ = 0.0;
  }

  const Point2 error{
    desired_offset.first - commanded_displacement.first,
    desired_offset.second - commanded_displacement.second};
  const auto desired_velocity = limit_norm(
    {error.first / dt, error.second / dt}, max_carrot_speed_);
  const auto velocity_change = limit_norm(
    {desired_velocity.first - command_velocity.first,
      desired_velocity.second - command_velocity.second},
    max_carrot_acceleration_ * dt);
  const auto velocity = limit_norm(
    {command_velocity.first + velocity_change.first,
      command_velocity.second + velocity_change.second},
    max_carrot_speed_);
  const Point2 step{velocity.first * dt, velocity.second * dt};
  Point2 next_displacement;
  Point2 next_velocity;
  if (std::hypot(step.first, step.second) >= std::hypot(error.first, error.second)) {
    next_displacement = desired_offset;
    next_velocity = {0.0, 0.0};
  } else {
    next_displacement = {
      commanded_displacement.first + step.first,
      commanded_displacement.second + step.second};
    next_velocity = velocity;
  }

  const Point2 next_carrot{
    pose.first + next_displacement.first, pose.second + next_displacement.second};
  if (command_validator && !command_validator(next_carrot)) {
    // The adapter treats invalidity as a stationary hold. Reset the relative
    // proposal too, so a future safe recovery accelerates from the held pose
    // instead of reviving a stale direction.
    store({0.0, 0.0}, {0.0, 0.0});
    at_goal_ = false;
    return {
      "CLEARANCE_BLOCKED", false, desired, pose, {0.0, 0.0}, {0.0, 0.0},
      path_progress_, progress_, remaining, projection.cross_track, generation_};
  }

  progress_ += candidate_progress - path_progress_;
  path_progress_ = candidate_progress;
  remaining = std::max(0.0, path_->length() - path_progress_);
  store(next_displacement, next_velocity);
  const Point2 carrot{
    pose.first + next_displacement.first, pose.second + next_displacement.second};
  const double tolerance = at_goal_ ? arrival_release_tolerance_ : arrival_tolerance_;
  at_goal_ = remaining <= tolerance && distance(pose, path_->points().back()) <= tolerance;
  return {
    at_goal_ ? "GOAL_REACHED" : "FOLLOWING", true, desired, carrot,
    next_displacement, commanded_displacement_vio_, path_progress_, progress_,
    remaining, projection.cross_track, generation_};
}

}  // namespace px4_vio_bridge
