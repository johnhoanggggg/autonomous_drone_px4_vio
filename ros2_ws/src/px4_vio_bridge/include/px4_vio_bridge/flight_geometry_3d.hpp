#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
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
}
