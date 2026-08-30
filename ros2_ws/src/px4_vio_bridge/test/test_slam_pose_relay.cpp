#include <cmath>

#include <gtest/gtest.h>

#include "px4_vio_bridge/slam_pose_relay.hpp"

using px4_vio_bridge::Pose3;
using px4_vio_bridge::Quat;
using px4_vio_bridge::Vec3;
using px4_vio_bridge::apply_correction;
using px4_vio_bridge::is_unit_quaternion;
using px4_vio_bridge::multiply;
using px4_vio_bridge::normalize;
using px4_vio_bridge::rotate;

namespace
{
Pose3 make_pose(double x, double y, double z, const Quat & q)
{
  Pose3 p;
  p.position = Vec3{x, y, z};
  p.orientation = q;
  return p;
}

// 90 degrees about +Z.
Quat yaw90()
{
  const double h = std::sqrt(0.5);
  return Quat{0.0, 0.0, h, h};
}
}  // namespace

TEST(SlamPoseRelay, IdentityCorrectionIsPassThrough)
{
  // What the on-device node published before its first loop closure.
  const Pose3 odom = make_pose(1.5, -2.0, 0.75, yaw90());
  const Pose3 out = apply_correction(Pose3{}, odom);

  EXPECT_NEAR(out.position.x, 1.5, 1e-9);
  EXPECT_NEAR(out.position.y, -2.0, 1e-9);
  EXPECT_NEAR(out.position.z, 0.75, 1e-9);
  EXPECT_NEAR(out.orientation.z, yaw90().z, 1e-9);
  EXPECT_NEAR(out.orientation.w, yaw90().w, 1e-9);
}

TEST(SlamPoseRelay, PureTranslationCorrectionAddsOffset)
{
  Pose3 correction;
  correction.position = Vec3{10.0, -5.0, 1.0};
  const Pose3 out = apply_correction(correction, make_pose(1.0, 2.0, 3.0, Quat{}));

  EXPECT_NEAR(out.position.x, 11.0, 1e-9);
  EXPECT_NEAR(out.position.y, -3.0, 1e-9);
  EXPECT_NEAR(out.position.z, 4.0, 1e-9);
}

TEST(SlamPoseRelay, YawCorrectionRotatesPositionAndComposesOrientation)
{
  Pose3 correction;
  correction.orientation = yaw90();
  // A pose one metre along +x in odom lands one metre along +y in map.
  const Pose3 out = apply_correction(correction, make_pose(1.0, 0.0, 0.0, Quat{}));

  EXPECT_NEAR(out.position.x, 0.0, 1e-9);
  EXPECT_NEAR(out.position.y, 1.0, 1e-9);
  EXPECT_NEAR(out.position.z, 0.0, 1e-9);
  EXPECT_NEAR(out.orientation.z, yaw90().z, 1e-9);
  EXPECT_NEAR(out.orientation.w, yaw90().w, 1e-9);
}

TEST(SlamPoseRelay, YawCorrectionAppliesRotationBeforeTranslation)
{
  Pose3 correction;
  correction.orientation = yaw90();
  correction.position = Vec3{2.0, 0.0, 0.0};
  const Pose3 out = apply_correction(correction, make_pose(1.0, 0.0, 0.0, Quat{}));

  EXPECT_NEAR(out.position.x, 2.0, 1e-9);
  EXPECT_NEAR(out.position.y, 1.0, 1e-9);
}

TEST(SlamPoseRelay, TwoYawCorrectionsCompose)
{
  const Quat half = multiply(yaw90(), yaw90());  // 180 degrees
  const Vec3 v = rotate(half, Vec3{1.0, 0.0, 0.0});
  EXPECT_NEAR(v.x, -1.0, 1e-9);
  EXPECT_NEAR(v.y, 0.0, 1e-9);
}

TEST(SlamPoseRelay, RejectsNonUnitQuaternion)
{
  EXPECT_TRUE(is_unit_quaternion(Quat{}));
  EXPECT_TRUE(is_unit_quaternion(yaw90()));
  EXPECT_FALSE(is_unit_quaternion(Quat{0.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(is_unit_quaternion(Quat{0.0, 0.0, 0.9, 0.9}));
}

TEST(SlamPoseRelay, NormalizeFallsBackToIdentity)
{
  const Quat out = normalize(Quat{0.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(out.w, 1.0, 1e-9);
  EXPECT_NEAR(out.x, 0.0, 1e-9);
}

TEST(SlamPoseRelay, OutputOrientationStaysUnit)
{
  Pose3 correction;
  // Slightly off-unit input still yields a unit output.
  correction.orientation = Quat{0.0, 0.0, 0.7075, 0.7075};
  const Pose3 out = apply_correction(correction, make_pose(0.0, 0.0, 0.0, yaw90()));
  EXPECT_TRUE(is_unit_quaternion(out.orientation, 1e-9));
}
