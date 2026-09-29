#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "px4_vio_bridge/path_geometry_3d.hpp"

namespace px4_vio_bridge
{

struct Follower3DConfig
{
  double lookahead{0.35};
  // Arc length for route_ahead (the heading chord). 0 uses `lookahead`.
  double heading_lookahead{0.0};
  double max_horizontal_speed{0.10};
  double max_vertical_speed{0.05};
  double max_horizontal_acceleration{0.30};
  double max_vertical_acceleration{0.20};
  double max_cross_track{0.05};
  double max_vertical_track{0.05};
  double arrival_tolerance{0.10};
  bool stop_at_vertices{false};
  // Hold the line. The planner starts every replan at the drone, so adopting it
  // bakes the drone's drift into the route. When > 0, a replacement that is the
  // same route apart from its first `hold_line_skip` metres keeps the current
  // route and command point instead (see hold_equivalent_path). 0 disables.
  double hold_line_skip{0.0};
  // Hold only while the drone is within this distance of the current route.
  double hold_line_max_offset{0.10};
  // Pull back toward the route: the command is offset toward it by this fraction
  // of the drone's perpendicular (cross-track and vertical) offset, capped.
  double cross_track_gain{0.0};
  double cross_track_correction_max{0.08};
  // Path carrot: the command point moves ALONG the route (arc length) instead of
  // stopping at every vertex (stop_at_vertices is then ignored). It slows before
  // corners (full speed straight, corner_speed_min at >= 90 deg), respects the
  // per-segment horizontal/vertical limits, stops at the end, and never leads
  // the drone's projection by more than max_carrot_lead or trails it.
  bool path_carrot{false};
  double corner_speed_min{0.03};
  double max_carrot_lead{0.35};
};

// Direction-preserving braking and acceleration for 3D flight segments.
[[nodiscard]] Point3 segment_velocity_3d(const Point3 & displacement, const Point3 & velocity,
  double dt, const Follower3DConfig & config);

// Conservative map-frame limits whose inverse rotation fits the configured
// continuous-odometry horizontal/vertical limits. cosine is R(map<-odom)[2,2].
[[nodiscard]] Follower3DConfig limits_for_map_tilt(
  const Follower3DConfig & config, double cosine);

struct FollowResult3D
{
  bool valid{};
  bool reached{};
  Point3 carrot{};
  Point3 lookahead{};
  // On-route point `heading_lookahead` (else `lookahead`) beyond the projection. Together with
  // projection.point it gives a heading chord that lateral error cannot turn.
  Point3 route_ahead{};
  Point3 displacement{};
  // Pull-back added to the command (zero when cross_track_gain is 0 or blocked).
  Point3 correction{};
  Point3 velocity{};
  Point3 acceleration{};
  Projection3D projection{};
  double remaining{};
  std::string reason;
};

class RouteFollower3D
{
public:
  using ChordValidator = std::function<bool(const Point3 &, const Point3 &)>;

  explicit RouteFollower3D(Follower3DConfig config = {});
  void clear();
  bool set_path(const std::vector<Point3> & points, const Point3 & pose);
  // Replace a re-published version of the same route without discarding the
  // command point or its velocity. The forward suffixes from `pose` must stay
  // within `tolerance`, and the connector to the carried command point must be
  // clearance checked before any state is changed.
  bool replace_equivalent_path(
    const std::vector<Point3> & points, const Point3 & pose, double tolerance,
    const ChordValidator & validator);
  // Hold the line: keep the current route, command point and velocity when the
  // replacement matches it beyond its drone-anchored start (both ways, within
  // `tolerance`), ends at the same goal, the drone is within
  // hold_line_max_offset of the current route, and the current route's
  // remaining chords from the command point still validate. Returns false
  // (nothing changed) when any condition fails or hold_line_skip is 0.
  bool hold_equivalent_path(
    const std::vector<Point3> & points, const Point3 & pose, double tolerance,
    const ChordValidator & validator);
  [[nodiscard]] FollowResult3D update(
    const Point3 & pose, double dt, const ChordValidator & validator);

  [[nodiscard]] Point3 velocity() const {return velocity_;}
  // Carry the previous follower's velocity into a new route, so adopting a
  // route does not restart from standstill (path_carrot keeps its along speed).
  void seed_velocity(const Point3 & velocity);

private:
  Follower3DConfig config_;
  std::unique_ptr<Polyline3D> path_;
  Point3 carrot_{};
  Point3 velocity_{};
  bool initialized_{};
  std::size_t target_vertex_{1};
  double s_{};        // path_carrot: carrot arc length
  double speed_{};    // path_carrot: carrot speed along the route
  [[nodiscard]] double segment_speed(std::size_t segment) const;
  [[nodiscard]] double segment_acceleration(std::size_t segment) const;
  [[nodiscard]] double allowed_speed(double along, double dt) const;
  [[nodiscard]] std::size_t segment_at(double along) const;
};

}  // namespace px4_vio_bridge
