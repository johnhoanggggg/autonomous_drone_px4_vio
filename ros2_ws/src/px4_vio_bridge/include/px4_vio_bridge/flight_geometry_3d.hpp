#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>
#include "px4_vio_bridge/grid_planner_3d.hpp"
#include "px4_vio_bridge/navigation_snapshot.hpp"
#include "px4_vio_bridge/octomap_grid.hpp"
#include "px4_vio_bridge/route_follower_3d.hpp"
namespace px4_vio_bridge
{
inline bool finite_flight_point(const Point3 & p)
{return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);}
inline Point3 point3(const Vec3 & p) {return {p.x, p.y, p.z};}
inline Vec3 vec3(const Point3 & p) {return {p.x, p.y, p.z};}
inline Quat inverse_rotation(const Quat & q) {return {-q.x, -q.y, -q.z, q.w};}
inline Point3 odom_point_to_map_3d(const Point3 & p, const Pose3 & map_from_odom)
{
  const auto rotated = rotate(map_from_odom.orientation, vec3(p));
  return {
    map_from_odom.position.x + rotated.x,
    map_from_odom.position.y + rotated.y,
    map_from_odom.position.z + rotated.z};
}
inline Point3 map_point_to_odom_3d(const Point3 & p, const Pose3 & map_from_odom)
{
  return point3(rotate(inverse_rotation(map_from_odom.orientation), {
    p.x - map_from_odom.position.x,
    p.y - map_from_odom.position.y,
    p.z - map_from_odom.position.z}));
}
inline Point3 reframe_map_point_3d(
  const Point3 & p, const Pose3 & from_map_from_odom, const Pose3 & to_map_from_odom)
{
  return odom_point_to_map_3d(map_point_to_odom_3d(p, from_map_from_odom), to_map_from_odom);
}
inline Point3 map_delta_to_ned(const Point3 & d, const Quat & map_from_odom, double yaw_offset)
{
  auto raw = rotate(inverse_rotation(map_from_odom), vec3(d));
  const auto c = std::cos(yaw_offset), s = std::sin(yaw_offset);
  return {s * raw.x + c * raw.y, c * raw.x - s * raw.y, -raw.z};
}
inline Point3 ned_delta_to_map(const Point3 & d, const Quat & map_from_odom, double yaw_offset)
{
  const auto c = std::cos(yaw_offset), s = std::sin(yaw_offset);
  return point3(rotate(map_from_odom, {c * d.y + s * d.x, -s * d.y + c * d.x, -d.z}));
}
inline bool within_flight_bounds(
  const Point3 & p, const Point3 & origin, double radius, double low, double high)
{
  const double height = origin.z - p.z;
  return finite_flight_point(p) && std::hypot(p.x-origin.x, p.y-origin.y) <= radius &&
    height >= low && height <= high;
}
// The policy applies only to this crop. The source OctoMap is never modified.
template<class Tree>
bool flight_chord_clear(const Tree & tree, const Point3 & a, const Point3 & b,
  double radius, bool allow_unknown)
{
  if (!finite_flight_point(a) || !finite_flight_point(b) || !std::isfinite(radius) || radius <= 0) {return false;}
  const auto r = tree.getResolution();
  if (!std::isfinite(r) || r < .01 || r > .5) {return false;}
  const double pad = radius + r;
  const Point3 origin{std::floor((std::min(a.x,b.x)-pad)/r)*r,
    std::floor((std::min(a.y,b.y)-pad)/r)*r, std::floor((std::min(a.z,b.z)-pad)/r)*r};
  const auto dim = [r](double lo, double hi) {return std::ceil((hi-lo)/r)+1.0;};
  const double w=dim(origin.x,std::max(a.x,b.x)+pad), h=dim(origin.y,std::max(a.y,b.y)+pad),
    d=dim(origin.z,std::max(a.z,b.z)+pad);
  if (!std::isfinite(w*h*d) || w*h*d > 2000000 || w<1 || h<1 || d<1) {return false;}
  VoxelGrid crop(static_cast<std::size_t>(w), static_cast<std::size_t>(h), static_cast<std::size_t>(d),
    r, origin, allow_unknown ? VoxelState::Free : VoxelState::Unknown);
  fill_native_octomap_grid(tree, crop);
  return swept_sphere_clear(crop,a,b,radius);
}
// Final PX4 position limiter: applied AFTER map->odom->NED conversion/rebasing.
class FlightPositionLimiter3D
{
public:
  explicit FlightPositionLimiter3D(Follower3DConfig config = {}) : config_(config) {}
  void reset(const Point3 & p) {position_=p; velocity_={};}
  Point3 update(const Point3 & target, double dt)
  {
    if (!finite_flight_point(target) || !std::isfinite(dt) || dt <= 0 || dt > .5) {
      throw std::invalid_argument("invalid flight limiter input");
    }
    const auto axis = [dt](double p, double v, double target_value, double speed, double acceleration) {
        const double d=target_value-p, adt=acceleration*dt;
        const double braking=(2*acceleration*std::abs(d))/(std::sqrt(adt*adt+2*acceleration*std::abs(d))+adt);
        const double desired=std::copysign(std::min(speed,braking),d);
        return v+std::clamp(desired-v,-adt,adt);
      };
    if (config_.stop_at_vertices) {
      velocity_=segment_velocity_3d(subtract_3d(target,position_),velocity_,dt,config_);
    } else {velocity_={axis(position_.x,velocity_.x,target.x,config_.max_horizontal_speed/std::sqrt(2.0),config_.max_horizontal_acceleration/std::sqrt(2.0)),
      axis(position_.y,velocity_.y,target.y,config_.max_horizontal_speed/std::sqrt(2.0),config_.max_horizontal_acceleration/std::sqrt(2.0)),
      axis(position_.z,velocity_.z,target.z,config_.max_vertical_speed,config_.max_vertical_acceleration)};}
    position_=add_3d(position_,scale_3d(velocity_,dt)); return position_;
  }
  Point3 position() const {return position_;}
  Point3 velocity() const {return velocity_;}
private:
  Follower3DConfig config_;
  Point3 position_{}, velocity_{};
};

/// Blocking voxel boxes near a point, for clearance-escape decisions.
///
/// clearance() is the box distance to the nearest blocking voxel, capped at
/// `cap` when none lies within the collected crop (so the cap must not exceed
/// the crop radius used to build it).
struct BlockingBoxes3D
{
  std::vector<Point3> centers;
  double half_size{};
  double cap{};
  [[nodiscard]] double clearance(const Point3 & p) const
  {
    double best = cap;
    for (const auto & c : centers) {
      const double qx = std::max(std::abs(p.x - c.x) - half_size, 0.0);
      const double qy = std::max(std::abs(p.y - c.y) - half_size, 0.0);
      const double qz = std::max(std::abs(p.z - c.z) - half_size, 0.0);
      best = std::min(best, std::sqrt(qx * qx + qy * qy + qz * qz));
    }
    return best;
  }
  /// Minimum clearance along a chord, sampled at a quarter voxel.
  [[nodiscard]] double chord_clearance(const Point3 & a, const Point3 & b) const
  {
    const double length = distance_3d(a, b);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / std::max(half_size * 0.5, 1e-3))));
    double best = cap;
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      best = std::min(best, clearance({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)}));
    }
    return best;
  }
};

/// Collect occupied (and, unless allow_unknown, unknown) native voxels within
/// `radius` of `center`, using the same crop policy as flight_chord_clear.
template<class Tree>
BlockingBoxes3D collect_blocking_boxes(
  const Tree & tree, const Point3 & center, double radius, bool allow_unknown)
{
  BlockingBoxes3D boxes;
  const auto r = tree.getResolution();
  boxes.half_size = r * 0.5;
  boxes.cap = radius;
  const double pad = radius + r;
  const Point3 origin{std::floor((center.x - pad) / r) * r,
    std::floor((center.y - pad) / r) * r, std::floor((center.z - pad) / r) * r};
  const auto n = static_cast<std::size_t>(std::ceil(2.0 * pad / r) + 1.0);
  VoxelGrid crop(n, n, n, r, origin, allow_unknown ? VoxelState::Free : VoxelState::Unknown);
  fill_native_octomap_grid(tree, crop);
  for (int z = 0; z < static_cast<int>(n); ++z) {
    for (int y = 0; y < static_cast<int>(n); ++y) {
      for (int x = 0; x < static_cast<int>(n); ++x) {
        const auto state = crop.at({x, y, z});
        if (state == VoxelState::Occupied || (!allow_unknown && state == VoxelState::Unknown)) {
          boxes.centers.push_back(crop.voxel_center({x, y, z}));
        }
      }
    }
  }
  return boxes;
}

/// Best short step out of a clearance violation for a held 3D vehicle.
///
/// Tries `headings` horizontal directions at three elevations plus straight
/// up/down, each `step` metres long, and returns the admissible endpoint that
/// gains the most clearance. Admissible is the 2D adapter's escape rule: no
/// point of the chord may be closer to a blocking voxel than the pose already
/// is, and the endpoint must gain at least `minimum_improvement`. So it can
/// only ever step away from the obstacle; no value means keep holding.
inline std::optional<Point3> best_clearance_escape_step_3d(
  const BlockingBoxes3D & boxes, const Point3 & pose, double step,
  double start_clearance, double minimum_improvement, int headings = 16)
{
  if (!(step > 0.0) || !(minimum_improvement > 0.0) || headings < 4) {return std::nullopt;}
  constexpr double tolerance = 1.0e-6;
  std::optional<Point3> best;
  double best_clearance = start_clearance + minimum_improvement - tolerance;
  const auto consider = [&](const Point3 & direction) {
      const Point3 end{pose.x + step * direction.x, pose.y + step * direction.y,
        pose.z + step * direction.z};
      const double end_clearance = boxes.clearance(end);
      if (end_clearance < best_clearance) {return;}
      if (boxes.chord_clearance(pose, end) < start_clearance - tolerance) {return;}
      best = end;
      best_clearance = end_clearance;
    };
  for (const double elevation : {0.0, M_PI / 6.0, -M_PI / 6.0}) {
    for (int i = 0; i < headings; ++i) {
      const double heading = 2.0 * M_PI * i / headings;
      consider({std::cos(elevation) * std::cos(heading),
        std::cos(elevation) * std::sin(heading), std::sin(elevation)});
    }
  }
  consider({0.0, 0.0, 1.0});
  consider({0.0, 0.0, -1.0});
  return best;
}
}
