#include "px4_vio_bridge/path_geometry.hpp"

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
}  // namespace

double wrap_pi_mod(double angle)
{
  // Python's `%` yields a result with the sign of the divisor; std::fmod does
  // not, so the negative branch is corrected explicitly.
  double value = std::fmod(angle + kPi, 2.0 * kPi);
  if (value < 0.0) {
    value += 2.0 * kPi;
  }
  return value - kPi;
}

double wrap_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_quaternion(double w, double x, double y, double z)
{
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-3) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double px4_yaw_from_quaternion(double w, double x, double y, double z)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

bool finite(const Point2 & point)
{
  return std::isfinite(point.first) && std::isfinite(point.second);
}

double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}

Polyline::Polyline(const std::vector<Point2> & points)
{
  for (const auto & point : points) {
    if (!finite(point)) {
      throw std::invalid_argument("path contains a non-finite point");
    }
    if (points_.empty() || distance(points_.back(), point) > 1.0e-9) {
      points_.push_back(point);
    }
  }
  if (points_.empty()) {
    throw std::invalid_argument("path needs at least one point");
  }
  cumulative_.reserve(points_.size());
  cumulative_.push_back(0.0);
  for (std::size_t index = 1; index < points_.size(); ++index) {
    cumulative_.push_back(
      cumulative_.back() + distance(points_[index - 1], points_[index]));
  }
  length_ = cumulative_.back();
}

Projection Polyline::project(const Point2 & point) const
{
  if (points_.size() == 1) {
    return Projection{points_[0], 0.0, distance(point, points_[0]), 0};
  }
  std::optional<Projection> best;
  for (std::size_t index = 0; index + 1 < points_.size(); ++index) {
    const auto & start = points_[index];
    const auto & end = points_[index + 1];
    const double dx = end.first - start.first;
    const double dy = end.second - start.second;
    const double length_sq = dx * dx + dy * dy;
    const double fraction = std::clamp(
      ((point.first - start.first) * dx + (point.second - start.second) * dy) / length_sq,
      0.0, 1.0);
    const Point2 projected{start.first + fraction * dx, start.second + fraction * dy};
    const Projection candidate{
      projected,
      cumulative_[index] + fraction * std::sqrt(length_sq),
      distance(point, projected),
      index};
    // Python compares the tuple (cross_track, -along), so a tie in distance
    // resolves to the projection that is FURTHER along the path.
    if (!best ||
      candidate.cross_track < best->cross_track ||
      (candidate.cross_track == best->cross_track && -candidate.along < -best->along))
    {
      best = candidate;
    }
  }
  return *best;
}

Point2 Polyline::point_at(double along) const
{
  if (points_.size() == 1) {
    return points_[0];
  }
  const double target = std::clamp(along, 0.0, length_);
  for (std::size_t index = 0; index + 1 < points_.size(); ++index) {
    const double start_distance = cumulative_[index];
    const double end_distance = cumulative_[index + 1];
    if (target <= end_distance) {
      const double fraction =
        (target - start_distance) / (end_distance - start_distance);
      const auto & start = points_[index];
      const auto & end = points_[index + 1];
      return {
        start.first + fraction * (end.first - start.first),
        start.second + fraction * (end.second - start.second)};
    }
  }
  return points_.back();
}

PathFingerprint path_fingerprint(const std::vector<Point2> & points, int precision)
{
  const double scale = std::pow(10.0, precision);
  PathFingerprint fingerprint;
  fingerprint.reserve(points.size());
  for (const auto & point : points) {
    fingerprint.emplace_back(
      static_cast<std::int64_t>(std::llround(point.first * scale)),
      static_cast<std::int64_t>(std::llround(point.second * scale)));
  }
  return fingerprint;
}

std::optional<std::string> correction_rejection_reason(
  const std::array<double, 4> & correction, double max_translation, double max_yaw)
{
  for (const double value : correction) {
    if (!std::isfinite(value)) {
      return std::string("correction contains a non-finite value");
    }
  }
  const double translation = std::sqrt(
    correction[0] * correction[0] + correction[1] * correction[1] +
    correction[2] * correction[2]);
  char buffer[160];
  if (translation > max_translation) {
    std::snprintf(
      buffer, sizeof(buffer),
      "translation %.2fm exceeds max_correction_m %.2f", translation, max_translation);
    return std::string(buffer);
  }
  const double yaw = std::abs(wrap_pi_mod(correction[3]));
  if (yaw > max_yaw) {
    std::snprintf(
      buffer, sizeof(buffer),
      "yaw %.1fdeg exceeds max_correction_yaw_deg %.1f",
      yaw * 180.0 / kPi, max_yaw * 180.0 / kPi);
    return std::string(buffer);
  }
  return std::nullopt;
}

bool finite(const Correction4 & correction)
{
  return std::all_of(
    correction.begin(), correction.end(), [](double value) {return std::isfinite(value);});
}

Point2 vio_point_to_map(const Point2 & point, const Correction4 & correction)
{
  const auto cosine = std::cos(correction[3]);
  const auto sine = std::sin(correction[3]);
  return {
    cosine * point.first - sine * point.second + correction[0],
    sine * point.first + cosine * point.second + correction[1]};
}

Point2 map_point_to_vio(const Point2 & point, const Correction4 & correction)
{
  const auto cosine = std::cos(correction[3]);
  const auto sine = std::sin(correction[3]);
  const auto dx = point.first - correction[0];
  const auto dy = point.second - correction[1];
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

Point2 reexpress_point(const Point2 & point, const Correction4 & from, const Correction4 & to)
{
  return vio_point_to_map(map_point_to_vio(point, from), to);
}

Point2 reexpress_vector(const Point2 & vector, const Correction4 & from, const Correction4 & to)
{
  const auto delta = wrap_pi_mod(to[3] - from[3]);
  const auto cosine = std::cos(delta);
  const auto sine = std::sin(delta);
  return {
    cosine * vector.first - sine * vector.second,
    sine * vector.first + cosine * vector.second};
}

Point2 map_displacement_to_vio(const Point2 & displacement, double correction_yaw)
{
  if (!finite(displacement) || !std::isfinite(correction_yaw)) {
    throw std::invalid_argument("displacement and correction yaw must be finite");
  }
  const double cosine = std::cos(correction_yaw);
  const double sine = std::sin(correction_yaw);
  // R(-yaw) * d_map. Translation cancels because this is a vector.
  return {
    cosine * displacement.first + sine * displacement.second,
    -sine * displacement.first + cosine * displacement.second};
}

Point2 vio_displacement_to_map(const Point2 & displacement, double correction_yaw)
{
  if (!finite(displacement) || !std::isfinite(correction_yaw)) {
    throw std::invalid_argument("displacement and correction yaw must be finite");
  }
  const double cosine = std::cos(correction_yaw);
  const double sine = std::sin(correction_yaw);
  return {
    cosine * displacement.first - sine * displacement.second,
    sine * displacement.first + cosine * displacement.second};
}

Point2 vio_enu_displacement_to_ned(const Point2 & displacement)
{
  if (!finite(displacement)) {
    throw std::invalid_argument("VIO displacement must be finite");
  }
  return {displacement.second, displacement.first};
}

std::optional<double> ned_track_heading(const Point2 & displacement, double min_distance)
{
  if (!finite(displacement)) {
    throw std::invalid_argument("displacement must be finite");
  }
  if (!std::isfinite(min_distance) || min_distance <= 0.0) {
    throw std::invalid_argument("min_distance must be finite and positive");
  }
  if (std::hypot(displacement.first, displacement.second) < min_distance) {
    return std::nullopt;
  }
  // NED yaw is measured from +x (north) toward +y (east).
  return std::atan2(displacement.second, displacement.first);
}

std::optional<double> track_yaw_target(
  std::optional<double> current, std::optional<double> heading, double deadband)
{
  if (!heading || !std::isfinite(*heading)) {
    return current;
  }
  if (!std::isfinite(deadband) || deadband < 0.0) {
    throw std::invalid_argument("deadband must be finite and non-negative");
  }
  if (!current || std::abs(wrap_pi(*heading - *current)) > deadband) {
    return heading;
  }
  return current;
}

Point2 limit_norm(const Point2 & vector, double maximum)
{
  const double magnitude = std::hypot(vector.first, vector.second);
  if (magnitude <= maximum || magnitude <= 0.0) {
    return vector;
  }
  const double scale = maximum / magnitude;
  return {vector.first * scale, vector.second * scale};
}

}  // namespace px4_vio_bridge
