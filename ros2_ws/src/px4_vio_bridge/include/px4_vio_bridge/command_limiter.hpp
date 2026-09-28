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

/// Arc-length averaged route direction. Zero lookahead retains the point
/// tangent. Hairpins are not averaged across: opposite legs can cancel.
std::optional<Point2> route_heading_direction(
  const Polyline & path, double progress, double lookahead);

/// Persistent heading reference in the continuous flight frame. The geometric
/// goal has a deadband; the reference slews to that goal even inside it.
/// Copyable so rejected route commands can roll back all heading state.
class HeadingReference
{
public:
  void reset(std::optional<double> heading);
  std::optional<double> update(
    std::optional<double> preview, double deadband, double max_rate, double dt);
  /// Use the largest error to preview, goal or reference for braking/alignment.
  /// Smoothing must not hide a genuine large change in route direction.
  [[nodiscard]] std::optional<double> alignment_error(double measured) const;
  [[nodiscard]] std::optional<double> preview() const {return preview_;}
  [[nodiscard]] std::optional<double> goal() const {return goal_;}
  [[nodiscard]] std::optional<double> target() const {return target_;}

private:
  std::optional<double> preview_, goal_, target_;
  bool slew_enabled_{false};
};

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
  /// Corner-fillet mode only: vertices shallow enough to be crossed at speed
  /// (the rounded arcs and any near-straight planner kink). Sorted by arc
  /// length; parallel arrays. Empty in every other mode.
  std::vector<double> soft;          ///< arc length of each crossable vertex
  std::vector<double> soft_turns;    ///< turn away from straight at that vertex
  std::vector<double> soft_radii;    ///< local radius; infinity for an isolated kink
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

  /// `fillet_check(replacement, original)` decides whether a rounded corner may
  /// replace the sharp one: `original` is the tangent point, the vertex and the
  /// other tangent point; `replacement` is the arc polyline between the same
  /// tangent points. It is the caller's occupancy test.
  using FilletCheck = std::function<bool (
        const std::vector<Point2> & replacement, const std::vector<Point2> & original)>;

  PathCommandLimiter(
    double max_speed = 0.10,
    double max_acceleration = 0.30,
    double max_projection_error = 0.10,
    double corner_tolerance = 0.05,
    double max_entry_error = 0.30,
    double max_connector_error = 0.20,
    double suffix_tolerance = 0.01,
    bool corner_blending = false,
    double junction_deviation = 0.05,
    /// When true, every published velocity is the exact command-point secant
    /// and its full vector change is acceleration bounded. Polyline corners
    /// become stops; future fillets must be clearance checked before use.
    bool strict_vector_limits = false,
    /// EXPERIMENTAL, strict mode only. Each bend of an installed path is
    /// replaced by a circular arc that cuts the corner by at most
    /// `junction_deviation`, provided set_path()'s fillet check accepts it, so
    /// the command keeps moving instead of stopping at the vertex. Every
    /// strict invariant still holds on the arc; the along-path acceleration is
    /// lowered to leave room for the turn. Bends that are not rounded still stop.
    bool corner_fillets = false);

  void clear();
  [[nodiscard]] Snapshot snapshot() const;
  void restore(const Snapshot & state);

  /// Re-express every map-frame quantity this limiter holds in a new SLAM map
  /// solution.
  ///
  /// The command point, its velocity and the installed path live in the map
  /// frame, but the limiter integrates them across ticks and nothing else
  /// moves them. When a loop closure changes the correction, the caller's pose
  /// jumps into the new solution while this state stays in the old one, and
  /// the difference the caller takes between them -- the displacement that
  /// becomes the PX4 setpoint -- picks up the whole correction delta in a
  /// single tick. On 20260908T052413Z a 0.396 m closure moved that setpoint
  /// 0.293 m while the limiter's own speed cap was 0.06 m/s, because the cap
  /// bounds this state and it was the *reference* that moved.
  ///
  /// A correction change is a rigid transform, so arc lengths, speeds, the
  /// bend table and the corner/join bookkeeping all survive it unchanged;
  /// only points and vectors are carried across.
  void reexpress(const Correction4 & from, const Correction4 & to);

  /// Install a path, carrying the command onto it without a jump.
  /// Throws std::invalid_argument when the replacement cannot be joined.
  /// With corner fillets on, a bend is rounded only when `fillet_check` is
  /// given and accepts the arc; without it every bend stays a stop.
  bool set_path(
    const std::vector<Point2> & points,
    const Point2 & reference,
    const ClearanceCheck & clearance_check = nullptr,
    const FilletCheck & fillet_check = nullptr);

  /// Throws std::runtime_error with no path, std::invalid_argument on a
  /// rejected command.
  ///
  /// `speed_cap` lowers the desired speed for this tick, including the
  /// rejoin's combined velocity. A newly lowered cap is approached through
  /// the acceleration limit; it is not an instantaneous bound on the state.
  /// For the follower's degraded ride-out, 0.20 -> 0.06 m/s at 0.30 m/s^2
  /// therefore takes ten 50 ms updates. Path geometry, corner/end handling
  /// and the caller's clearance checks still apply.
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
  /// A positive heading_lookahead averages the route tangent over that arc
  /// length. This changes heading only; the position command stays on the path.
  [[nodiscard]] std::optional<Point2> preview_direction(
    const Point2 & desired_point, double dt, const Point2 & reference_point,
    double min_position_distance, double heading_lookahead = 0.0) const;

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
  [[nodiscard]] bool corner_fillets() const {return corner_fillets_;}

private:
  static std::shared_ptr<const PathInstallation> build_installation(
    std::shared_ptr<const Polyline> path, PathFingerprint fingerprint, bool corner_fillets);
  /// The along-path acceleration budget: all of it, or with corner fillets
  /// the share that leaves room for the turn inside the vector limit.
  [[nodiscard]] double tangential_acceleration() const;
  /// Fastest speed now that can still slow to every crossable vertex ahead.
  [[nodiscard]] double soft_vertex_speed_limit(double dt) const;
  /// strict_progress() for filleted paths, where the secant is not affine
  /// across the crossable vertices of an arc.
  [[nodiscard]] double fillet_progress(
    double target_progress, double dt, const Point2 & old_position) const;
  [[nodiscard]] std::optional<double> shared_suffix_offset(const Polyline & new_path) const;
  void install(std::shared_ptr<const PathInstallation> installation, double progress);
  Point2 update_join(
    const Point2 & desired_point, double dt, bool advance, double ceiling);
  /// The on-path speed for this tick, shared by the advance and the join so a
  /// lane change is braked by corners and the path end exactly as motion is.
  double along_path_speed(
    double along, double desired_progress, double dt, bool advance, double ceiling);
  /// `max_speed_`, or the caller's lower ceiling for this tick.
  [[nodiscard]] double speed_ceiling(std::optional<double> speed_cap) const;
  [[nodiscard]] Point2 path_tangent(double along) const;
  [[nodiscard]] Point2 segment_tangent(double along) const;
  [[nodiscard]] double strict_progress(
    double target_progress, double dt, const Point2 & old_position) const;
  /// Direction over the upcoming route horizon (point tangent at zero).
  [[nodiscard]] std::optional<Point2> advance_tangent(double lookahead) const;
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
  /// Preserve exact path secants and bound the full velocity vector. Sharp
  /// corners stop in this mode; rounded, clearance-checked geometry can be
  /// added later without weakening this invariant.
  bool strict_vector_limits_;
  bool corner_fillets_;

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
