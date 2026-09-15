#pragma once

// Atomic, generation-tagged observation contracts carried by std_msgs/String.
// Separate pose/path visualization topics are never used to infer pairing.
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
#include "px4_vio_bridge/slam_pose_relay.hpp"

namespace px4_vio_bridge
{
inline bool fresh_stamp(std::int64_t stamp, std::int64_t now, double timeout)
{
  return stamp > 0 && stamp <= now &&
         static_cast<double>(now - stamp) * 1e-9 <= timeout;
}

inline Pose3 snapshot_pose(const nlohmann::json & value)
{
  if (!value.is_array() || value.size() != 7) {
    throw std::invalid_argument("pose requires xyz and xyzw");
  }
  for (const auto & number : value) {
    if (!number.is_number() || !std::isfinite(number.get<double>())) {
      throw std::invalid_argument("non-finite pose");
    }
  }
  Pose3 result{{value[0], value[1], value[2]}, {value[3], value[4], value[5], value[6]}};
  if (!is_unit_quaternion(result.orientation)) {
    throw std::invalid_argument("non-unit pose quaternion");
  }
  return result;
}

inline nlohmann::json pose_json(const Pose3 & pose)
{
  return {pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
}

inline Pose3 camera_to_body(const Pose3 & camera, const Vec3 & offset_frd)
{
  const auto offset = rotate(camera.orientation, {offset_frd.x, -offset_frd.y, -offset_frd.z});
  auto body = camera;
  body.position.x -= offset.x;
  body.position.y -= offset.y;
  body.position.z -= offset.z;
  return body;
}
}  // namespace px4_vio_bridge
