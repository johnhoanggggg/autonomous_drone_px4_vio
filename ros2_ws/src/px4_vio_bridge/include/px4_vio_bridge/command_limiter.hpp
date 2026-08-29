#pragma once

/// ROS-free position-command limiting for global-planner flight.
/// Exact counterpart of planner_flight.py.

#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "px4_vio_bridge/path_geometry.hpp"

namespace px4_vio_bridge
{

/// Speed- and acceleration-limit the final PX4 horizontal setpoint.
class HorizontalCommandLimiter
{
public:
  HorizontalCommandLimiter(double max_speed = 0.20, double max_acceleration = 0.40);

  void reset(const Point2 & position);
  Point2 update(const Point2 & target, double dt);
  /// Adopt an already-limited command without applying another free chord.
  Point2 adopt(const Point2 & position, const Point2 & velocity = {0.0, 0.0});

  [[nodiscard]] const std::optional<Point2> & position() const {return position_;}
  [[nodiscard]] const Point2 & velocity() const {return velocity_;}
  [[nodiscard]] double max_speed() const {return max_speed_;}
  [[nodiscard]] double max_acceleration() const {return max_acceleration_;}

private:
  double max_speed_;
  double max_acceleration_;
  std::optional<Point2> position_;
  Point2 velocity_{0.0, 0.0};
};

/// One installed path plus everything derived from its geometry. Immutable and
/// shared, so a snapshot/restore pair can never leave the bend table
/// disagreeing with the polyline it was computed from.
struct PathInstallation
{
  std::shared_ptr<const Polyline> path;
  PathFingerprint fingerprint;
  std::vector<double> bends;       ///< arc length of each vertex that is a stop
  std::vector<double> bend_turns;  ///< turn away from straight, parallel to bends
};

/// Advance the final command on a polyline or its bounded rejoin band.
///
/// Progress is one-dimensional arc length. Each bend is treated as a stop
/// point unless corner blending is on, in which case bends become speed limits.
class PathCommandLimiter
{
public:
  struct Snapshot
  {
    std::shared_ptr<const PathInstallation> installation;
    double progress{};
    double speed{};
    std::optional<Point2> position;
    Point2 velocity{0.0, 0.0};
    std::optional<Point2> join_target;
    double join_limit{};
    std::optional<double> waiting_vertex;
  };

  /// `clearance_check(start, end)` decides whether the wider connector rejoin
  /// is permitted; it is the caller's occupancy test.
  using ClearanceCheck = std::function<bool (const Point2 &, const Point2 &)>;

  PathCommandLimiter(
    double max_speed = 0.10,
    double max_acceleration = 0.30,
    double max_projection_error = 0.10,
    double corner_tolerance = 0.05,
    double max_entry_error = 0.30,
    double max_connector_error = 0.20,
    double suffix_tolerance = 0.01,
    bool corner_blending = false,
    double junction_deviation = 0.05);

  void clear();
  [[nodiscard]] Snapshot snapshot() const;
  void restore(const Snapshot & state);

  /// Install a path, carrying the command onto it without a jump.
  /// Throws std::invalid_argument when the replacement cannot be joined.
  bool set_path(
    const std::vector<Point2> & points,
    const Point2 & reference,
    const ClearanceCheck & clearance_check = nullptr);

  /// Throws std::runtime_error with no path, std::invalid_argument on a
  /// rejected command.
  Point2 update(
    const Point2 & desired_point,
    double dt,
    bool advance = true,
    const std::optional<Point2> & reference_point = std::nullopt);

  [[nodiscard]] const std::shared_ptr<const PathInstallation> & installation() const
  {
    return installation_;
  }
  [[nodiscard]] const Polyline * path() const
  {
    return installation_ ? installation_->path.get() : nullptr;
  }
  [[nodiscard]] const std::optional<Point2> & position() const {return position_;}
  [[nodiscard]] const Point2 & velocity() const {return velocity_;}
  [[nodiscard]] const std::optional<double> & waiting_vertex() const {return waiting_vertex_;}
  [[nodiscard]] double max_speed() const {return max_speed_;}
  [[nodiscard]] double max_acceleration() const {return max_acceleration_;}

private:
  static std::shared_ptr<const PathInstallation> build_installation(
    std::shared_ptr<const Polyline> path, PathFingerprint fingerprint);
  [[nodiscard]] std::optional<double> shared_suffix_offset(const Polyline & new_path) const;
  void install(std::shared_ptr<const PathInstallation> installation, double progress);
  Point2 update_join(double dt, bool advance);
  [[nodiscard]] double next_motion_target(double desired_progress) const;
  [[nodiscard]] double corner_speed(double turn) const;
  [[nodiscard]] double corner_speed_limit() const;

  double max_speed_;
  double max_acceleration_;
  double max_projection_error_;
  double corner_tolerance_;
  double max_entry_error_;
  double max_connector_error_;
  double suffix_tolerance_;
  bool corner_blending_;
  double junction_deviation_;

  std::shared_ptr<const PathInstallation> installation_;
  double progress_{0.0};
  double speed_{0.0};
  std::optional<Point2> position_;
  Point2 velocity_{0.0, 0.0};
  std::optional<Point2> join_target_;
  double join_limit_{0.0};
  std::optional<double> waiting_vertex_;
};

/// Clamp a point into a disc (planner_flight.clamp_to_disc).
Point2 clamp_to_disc(const Point2 & point, const Point2 & center, double radius);

}  // namespace px4_vio_bridge
