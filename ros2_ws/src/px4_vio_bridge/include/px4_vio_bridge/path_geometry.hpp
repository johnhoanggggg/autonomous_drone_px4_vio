#pragma once

/// ROS-free geometry shared by the C++ flight adapter.
///
/// Exact counterpart of the pure-Python helpers in path_follower.py and the
/// transform/heading helpers in planner_flight.py. Kept ROS-free so the
/// parity tests can exercise it without a node.

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace px4_vio_bridge
{

using Point2 = std::pair<double, double>;

/// path_follower._wrap_pi: Python's modulo, whose result takes the sign of the
/// divisor. std::fmod takes the sign of the dividend, so it cannot be used raw.
double wrap_pi_mod(double angle);

/// offboard_hover.wrap_pi / planner_flight._wrap_pi.
double wrap_pi(double angle);

/// Yaw about +Z from a quaternion supplied as (w, x, y, z). NaN when the
/// quaternion is not normalized, matching path_follower.yaw_from_quaternion.
double yaw_from_quaternion(double w, double x, double y, double z);

/// PX4 [w, x, y, z] yaw without the normalization gate
/// (offboard_hover.yaw_from_quaternion).
double px4_yaw_from_quaternion(double w, double x, double y, double z);

struct Projection
{
  Point2 point{};
  double along{};
  double cross_track{};
  std::size_t segment{};
};

/// path_follower.Polyline. Consecutive duplicate points are dropped on
/// construction; throws std::invalid_argument on a non-finite or empty path.
class Polyline
{
public:
  explicit Polyline(const std::vector<Point2> & points);

  [[nodiscard]] const std::vector<Point2> & points() const {return points_;}
  [[nodiscard]] const std::vector<double> & cumulative() const {return cumulative_;}
  [[nodiscard]] double length() const {return length_;}

  [[nodiscard]] Projection project(const Point2 & point) const;
  [[nodiscard]] Point2 point_at(double along) const;

private:
  std::vector<Point2> points_;
  std::vector<double> cumulative_;
  double length_{};
};

/// path_follower.path_fingerprint at the default precision of 4 decimals,
/// as fixed-point integers so equality is exact.
using PathFingerprint = std::vector<std::pair<std::int64_t, std::int64_t>>;
PathFingerprint path_fingerprint(const std::vector<Point2> & points, int precision = 4);

/// Why a native map-to-odom correction is unsafe, if anything.
/// correction is (x, y, z, yaw); mirrors path_follower.correction_rejection_reason.
std::optional<std::string> correction_rejection_reason(
  const std::array<double, 4> & correction, double max_translation, double max_yaw);

/// Rotate a map-frame vector into the continuous VIO/odometry frame: R(-yaw).
Point2 map_displacement_to_vio(const Point2 & displacement, double correction_yaw);

/// Rotate a continuous-VIO vector into the SLAM map frame: R(+yaw).
Point2 vio_displacement_to_map(const Point2 & displacement, double correction_yaw);

/// Convert a continuous-VIO ENU vector to PX4 NED horizontal axes.
Point2 vio_enu_displacement_to_ned(const Point2 & displacement);

/// PX4 NED yaw pointing along a horizontal NED displacement, or nullopt when
/// the vector is shorter than min_distance.
std::optional<double> ned_track_heading(const Point2 & displacement, double min_distance);

/// Latch a new yaw target only once the path heading leaves the deadband.
std::optional<double> track_yaw_target(
  std::optional<double> current, std::optional<double> heading, double deadband);

/// Scale a vector down to `maximum` if it is longer (planner_flight._limit).
Point2 limit_norm(const Point2 & vector, double maximum);

double distance(const Point2 & a, const Point2 & b);

bool finite(const Point2 & point);

}  // namespace px4_vio_bridge
