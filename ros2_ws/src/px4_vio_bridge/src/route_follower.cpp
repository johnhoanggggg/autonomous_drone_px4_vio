#include "px4_vio_bridge/route_follower.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace px4_vio_bridge
{
namespace
{

bool finite(const Correction4 & correction)
{
  return std::all_of(
    correction.begin(), correction.end(), [](double value) {return std::isfinite(value);});
}

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

CorrectionReplanGate::CorrectionReplanGate(
  double translation_trigger, double yaw_trigger, double filter_time_constant,
  double material_translation, double material_yaw, double quiet_time, double cooldown)
: translation_trigger_(translation_trigger),
  yaw_trigger_(yaw_trigger),
  filter_time_constant_(filter_time_constant),
  material_translation_(material_translation),
  material_yaw_(material_yaw),
  quiet_time_(quiet_time),
  cooldown_(cooldown)
{
  const std::array<double, 7> values{
    translation_trigger_, yaw_trigger_, filter_time_constant_, material_translation_,
    material_yaw_, quiet_time_, cooldown_};
  if (std::any_of(values.begin(), values.end(), [](double value) {
      return !std::isfinite(value) || value <= 0.0;
    }))
  {
    throw std::invalid_argument("correction gate limits must be finite and positive");
  }
}

bool CorrectionReplanGate::observe(const Correction4 & correction, double now)
{
  if (!finite(correction) || !std::isfinite(now)) {
    return false;
  }
  if (!filtered_ || !last_observation_ || now < *last_observation_) {
    filtered_ = correction;
    baseline_ = correction;
    event_reference_ = correction;
    last_observation_ = now;
    return false;
  }

  const double dt = now - *last_observation_;
  last_observation_ = now;
  const double fraction = 1.0 - std::exp(-dt / filter_time_constant_);
  filtered_ = blend_correction(*filtered_, correction, fraction);

  if (pending_) {
    const auto [translation, yaw] = correction_delta(*event_reference_, *filtered_);
    if (translation > material_translation_ || yaw > material_yaw_) {
      event_reference_ = filtered_;
      last_material_change_ = now;
      path_after_change_ = false;
    }
    return false;
  }

  if (now < cooldown_until_) {
    return false;
  }
  const auto [translation, yaw] = correction_delta(*baseline_, *filtered_);
  if (translation <= translation_trigger_ && yaw <= yaw_trigger_) {
    return false;
  }
  pending_ = true;
  event_reference_ = filtered_;
  last_material_change_ = now;
  path_after_change_ = false;
  last_trigger_delta_ = {translation, yaw};
  return true;
}

void CorrectionReplanGate::path_received(double now)
{
  if (pending_ && std::isfinite(now) && now >= last_material_change_) {
    path_after_change_ = true;
  }
}

bool CorrectionReplanGate::waiting(double now)
{
  if (pending_ && path_after_change_ && now - last_material_change_ >= quiet_time_) {
    pending_ = false;
    baseline_ = filtered_;
    cooldown_until_ = now + cooldown_;
  }
  return pending_;
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

void PositionRouteFollower::clear_path()
{
  path_.reset();
  fingerprint_.clear();
  path_progress_ = 0.0;
  command_velocity_ = {0.0, 0.0};
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
    command_velocity_ = {0.0, 0.0};
  }
}

void PositionRouteFollower::hold_command()
{
  commanded_displacement_ = {0.0, 0.0};
  command_velocity_ = {0.0, 0.0};
  at_goal_ = false;
}

bool PositionRouteFollower::set_path(
  const std::vector<Point2> & points, const Point2 & pose)
{
  const auto fingerprint = path_fingerprint(points);
  if (path_ && fingerprint == fingerprint_) {
    return false;
  }
  auto path = std::make_unique<Polyline>(points);
  path_progress_ = path->project(pose).along;
  path_ = std::move(path);
  fingerprint_ = fingerprint;
  ++generation_;
  return true;
}

FollowResult PositionRouteFollower::update(
  const Point2 & pose, double dt, std::optional<double> lookahead,
  const CommandValidator & command_validator)
{
  if (!path_) {
    throw std::runtime_error("no path");
  }
  if (!finite(pose) || !std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("pose and dt must be finite, and dt positive");
  }
  const double effective_lookahead = lookahead.value_or(lookahead_);
  if (!std::isfinite(effective_lookahead) || effective_lookahead < 0.0) {
    throw std::invalid_argument("lookahead must be finite and non-negative");
  }

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
    command_velocity_ = {0.0, 0.0};
    const Point2 carrot{
      pose.first + commanded_displacement_.first,
      pose.second + commanded_displacement_.second};
    return {
      "CROSS_TRACK_EXCEEDED", false, desired, carrot, commanded_displacement_,
      path_progress_, progress_, remaining, projection.cross_track, generation_};
  }

  if (cross_track_latched_) {
    command_velocity_ = {0.0, 0.0};
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
      pose.first + commanded_displacement_.first,
      pose.second + commanded_displacement_.second};
    if (cross_track_recovery_elapsed_ + 1.0e-12 < cross_track_recovery_time_) {
      return {
        status, false, desired, carrot, commanded_displacement_, path_progress_, progress_,
        remaining, projection.cross_track, generation_};
    }
    cross_track_latched_ = false;
    cross_track_fault_generation_ = 0;
    cross_track_recovery_elapsed_ = 0.0;
  }

  const Point2 error{
    desired_offset.first - commanded_displacement_.first,
    desired_offset.second - commanded_displacement_.second};
  const auto desired_velocity = limit_norm(
    {error.first / dt, error.second / dt}, max_carrot_speed_);
  const auto velocity_change = limit_norm(
    {desired_velocity.first - command_velocity_.first,
      desired_velocity.second - command_velocity_.second},
    max_carrot_acceleration_ * dt);
  const auto velocity = limit_norm(
    {command_velocity_.first + velocity_change.first,
      command_velocity_.second + velocity_change.second},
    max_carrot_speed_);
  const Point2 step{velocity.first * dt, velocity.second * dt};
  Point2 next_displacement;
  Point2 next_velocity;
  if (std::hypot(step.first, step.second) >= std::hypot(error.first, error.second)) {
    next_displacement = desired_offset;
    next_velocity = {0.0, 0.0};
  } else {
    next_displacement = {
      commanded_displacement_.first + step.first,
      commanded_displacement_.second + step.second};
    next_velocity = velocity;
  }

  const Point2 next_carrot{
    pose.first + next_displacement.first, pose.second + next_displacement.second};
  if (command_validator && !command_validator(next_carrot)) {
    commanded_displacement_ = {0.0, 0.0};
    command_velocity_ = {0.0, 0.0};
    at_goal_ = false;
    return {
      "CLEARANCE_BLOCKED", false, desired, pose, commanded_displacement_, path_progress_,
      progress_, remaining, projection.cross_track, generation_};
  }

  progress_ += candidate_progress - path_progress_;
  path_progress_ = candidate_progress;
  remaining = std::max(0.0, path_->length() - path_progress_);
  commanded_displacement_ = next_displacement;
  command_velocity_ = next_velocity;
  const Point2 carrot{
    pose.first + commanded_displacement_.first,
    pose.second + commanded_displacement_.second};
  const double tolerance = at_goal_ ? arrival_release_tolerance_ : arrival_tolerance_;
  at_goal_ = remaining <= tolerance && distance(pose, path_->points().back()) <= tolerance;
  return {
    at_goal_ ? "GOAL_REACHED" : "FOLLOWING", true, desired, carrot,
    commanded_displacement_, path_progress_, progress_, remaining,
    projection.cross_track, generation_};
}

}  // namespace px4_vio_bridge
