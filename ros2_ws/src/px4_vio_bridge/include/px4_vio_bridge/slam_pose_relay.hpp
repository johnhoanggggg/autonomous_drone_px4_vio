#ifndef PX4_VIO_BRIDGE__SLAM_POSE_RELAY_HPP_
#define PX4_VIO_BRIDGE__SLAM_POSE_RELAY_HPP_

// ROS-free pose composition for the SLAM pose relay.
//
// The on-device RTABMapSLAM published a loop-corrected pose at VIO rate.
// rtabmap_slam has no equivalent topic: it exposes the correction as the
// map->odom TF and only publishes localization_pose on a localization event,
// which never fires in SLAM mode. This rebuilds the corrected pose the way the
// on-device node did, so /rtabmap/pose and /rtabmap/odom_correction keep their
// previous meaning for the planner and follower.

namespace px4_vio_bridge
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quat
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose3
{
  Vec3 position;
  Quat orientation;
};

/// True when the quaternion is within tolerance of unit length. The on-device
/// node rejected non-unit corrections rather than propagating them.
bool is_unit_quaternion(const Quat & q, double tolerance = 1e-3);

/// Quaternion normalization. Returns identity for a degenerate input.
Quat normalize(const Quat & q);

/// Hamilton product, a applied after b.
Quat multiply(const Quat & a, const Quat & b);

/// Rotate v by q.
Vec3 rotate(const Quat & q, const Vec3 & v);

/// Compose a map<-odom correction with an odom-frame pose, giving the
/// map-frame pose. An identity correction returns the input unchanged, which
/// is what the on-device node published before the first loop closure.
Pose3 apply_correction(const Pose3 & map_from_odom, const Pose3 & odom_pose);

}  // namespace px4_vio_bridge

#endif  // PX4_VIO_BRIDGE__SLAM_POSE_RELAY_HPP_
