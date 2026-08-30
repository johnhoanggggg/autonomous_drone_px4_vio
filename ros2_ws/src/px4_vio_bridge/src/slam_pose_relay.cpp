#include "px4_vio_bridge/slam_pose_relay.hpp"

#include <cmath>

namespace px4_vio_bridge
{

bool is_unit_quaternion(const Quat & q, double tolerance)
{
  const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::abs(std::sqrt(norm_sq) - 1.0) <= tolerance;
}

Quat normalize(const Quat & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(norm > 0.0) || !std::isfinite(norm)) {
    return Quat{};
  }
  return Quat{q.x / norm, q.y / norm, q.z / norm, q.w / norm};
}

Quat multiply(const Quat & a, const Quat & b)
{
  return Quat{
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

Vec3 rotate(const Quat & q, const Vec3 & v)
{
  // v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
  const Vec3 u{q.x, q.y, q.z};
  const Vec3 t{
    u.y * v.z - u.z * v.y + q.w * v.x,
    u.z * v.x - u.x * v.z + q.w * v.y,
    u.x * v.y - u.y * v.x + q.w * v.z,
  };
  return Vec3{
    v.x + 2.0 * (u.y * t.z - u.z * t.y),
    v.y + 2.0 * (u.z * t.x - u.x * t.z),
    v.z + 2.0 * (u.x * t.y - u.y * t.x),
  };
}

Pose3 apply_correction(const Pose3 & map_from_odom, const Pose3 & odom_pose)
{
  const Quat correction = normalize(map_from_odom.orientation);
  const Vec3 rotated = rotate(correction, odom_pose.position);

  Pose3 out;
  out.position = Vec3{
    rotated.x + map_from_odom.position.x,
    rotated.y + map_from_odom.position.y,
    rotated.z + map_from_odom.position.z,
  };
  out.orientation = normalize(multiply(correction, odom_pose.orientation));
  return out;
}

}  // namespace px4_vio_bridge
