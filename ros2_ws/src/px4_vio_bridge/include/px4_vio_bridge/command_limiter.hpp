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

/// Taper translation between the yaw resume and hold errors (radians).
/// The ceiling itself falls by at most acceleration * dt. The existing yaw
/// hold and any stricter fault cap remain the caller's responsibility.
std::optional<double> yaw_translation_speed_cap(
  std::optional<double> error, double resume_error, double hold_error,
  double max_speed, const Point2 & velocity, double acceleration, double dt);

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
  ///
  /// `speed_cap` lowers `max_speed_` for this tick only, for a state that wants
  /// to keep moving slowly rather than stop -- the grace a brief follower
  /// dropout gets before it becomes a hold. It bounds
  /// the along-path rate, the rejoin's lateral rate and the combined velocity,
  /// so a capped tick is the ordinary tick at a lower ceiling -- not a
  /// different trajectory. Everything else, corner and end braking included,
  /// is unchanged.
  Point2 update(
    const Point2 & desired_point,
    double dt,
    bool advance = true,
    const std::optional<Point2> & reference_point = std::nullopt,
    std::optional<double> speed_cap = std::nullopt);

  /// Direction the accepted route is travelled in next, without committing any
  /// progress, join or corner state. Unit vector in the path frame.
  ///
  /// This is path geometry, not the one-tick command velocity. From rest that
  /// velocity is acceleration-limited, so update_join()'s lateral term
  /// dominates its along-path term and a few centimetres of lane change would
  /// be flown as a right-angle turn -- the 2026-09-06 stall.
  /// When the previewed tick commands no along-path motion at all, face an
  /// existing position target only if it is far enough from the vehicle;
  /// otherwise retain the caller's previous heading.
  [[nodiscard]] std::optional<Point2> preview_direction(
    const Point2 & desired_point, double dt, const Point2 & reference_point,
    double min_position_distance) const;

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
  /// True while the command point is rejoining a replacement path. Published so
  /// a bag shows directly what used to have to be inferred from a speed dip.
  [[nodiscard]] bool joining() const {return join_target_.has_value();}
  [[nodiscard]] double max_speed() const {return max_speed_;}
  [[nodiscard]] double max_acceleration() const {return max_acceleration_;}

private:
  static std::shared_ptr<const PathInstallation> build_installation(
    std::shared_ptr<const Polyline> path, PathFingerprint fingerprint);
  [[nodiscard]] std::optional<double> shared_suffix_offset(const Polyline & new_path) const;
  void install(std::shared_ptr<const PathInstallation> installation, double progress);
  Point2 update_join(double dt, bool advance, double ceiling);
  /// The on-path speed for this tick, shared by the advance and the join so a
  /// lane change is braked by corners and the path end exactly as motion is.
  double along_path_speed(double along, double dt, bool advance, double ceiling);
  /// `max_speed_`, or the caller's lower ceiling for this tick.
  [[nodiscard]] double speed_ceiling(std::optional<double> speed_cap) const;
  [[nodiscard]] Point2 path_tangent(double along) const;
  /// Unit tangent of the leg the command point is about to travel. Forward
  /// difference, so at a vertex this is the outgoing leg rather than the
  /// bisector a central difference would return.
  [[nodiscard]] std::optional<Point2> advance_tangent() const;
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
  /// Cross-track at or below which a join is finished and the command point is
  /// placed on the path. A tenth of a grid cell: small enough that snapping is
  /// below the map's own resolution, large enough that the join terminates.
  double join_tolerance_{0.003};
  /// Largest lateral closing rate a rejoin may add, as a multiple of the
  /// along-path rate: tan(15 degrees), so the commanded velocity stays within
  /// 15 degrees of the route while a replacement is being merged onto.
  double join_lateral_ratio_{0.2679};
  /// Lateral rate allowed regardless of the ratio, so a rejoin still completes
  /// when the along-path rate is zero. A crawl next to the 0.20 m/s cruise.
  double join_lateral_floor_{0.02};
  /// Beyond `max_projection_error_` of offset the merge angle widens in
  /// proportion, so an offset too large for the flat bound to clear before the
  /// follower's cross-track limit trips cannot stand. Inside that tolerance --
  /// which covers the re-anchoring offsets the bound exists for, median 0.022 m
  /// and 0.048 m on 20260906T120129Z and T120238Z -- nothing changes.
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
