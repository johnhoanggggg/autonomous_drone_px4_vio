#include <gtest/gtest.h>
#include <limits>
#include "px4_vio_bridge/navigation_snapshot.hpp"

using namespace px4_vio_bridge;

TEST(NavigationSnapshot, SensorTimeMustBePositiveNotFutureAndWithinBudget)
{
  EXPECT_TRUE(fresh_stamp(1000000000, 1500000000, 0.5));
  EXPECT_FALSE(fresh_stamp(0, 1500000000, 0.5));
  EXPECT_FALSE(fresh_stamp(999999999, 1500000000, 0.5));
  EXPECT_FALSE(fresh_stamp(1500000001, 1500000000, 0.5));
}

TEST(NavigationSnapshot, InvalidGeometryCannotBecomeIdentity)
{
  EXPECT_THROW(snapshot_pose({0, 0, 0, 0, 0, 0, 0}), std::invalid_argument);
  EXPECT_THROW(snapshot_pose({0, 0, 0, 0, 0, 0, 2}), std::invalid_argument);
  EXPECT_THROW(snapshot_pose({0, 0, std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 1}),
    std::invalid_argument);
  EXPECT_THROW(snapshot_pose({0, 0, 0}), std::invalid_argument);
}

TEST(NavigationSnapshot, CorrectionMustMatchMapIncludingQuaternionSignEquivalence)
{
  const auto a = snapshot_pose({0.2, -0.1, 0.3, 0, 0, 0, 1});
  auto b = snapshot_pose({0.2, -0.1, 0.3, 0, 0, 0, -1});
  EXPECT_TRUE(same_snapshot_transform(a, b));
  b.position.x += 0.001;
  EXPECT_FALSE(same_snapshot_transform(a, b));
  b = snapshot_pose({0.2, -0.1, 0.3, 0, std::sin(0.1), 0, std::cos(0.1)});
  EXPECT_FALSE(same_snapshot_transform(a, b));
}

TEST(NavigationSnapshot, BodyOffsetRotatesAndComposesWithMapCorrection)
{
  const auto half = std::sqrt(0.5);
  const Pose3 camera{{1, 2, 3}, {0, 0, half, half}};
  const Vec3 offset{0.100, -0.036, 0.056};
  const auto body = camera_to_body(camera, offset);
  EXPECT_NEAR(body.position.x, 1.036, 1e-12);
  EXPECT_NEAR(body.position.y, 1.900, 1e-12);
  EXPECT_NEAR(body.position.z, 3.056, 1e-12);
  const Pose3 correction{{0.2, -0.3, 0.4}, {half, 0, 0, half}};
  const auto a = apply_correction(correction, body);
  const auto b = camera_to_body(apply_correction(correction, camera), offset);
  EXPECT_NEAR(a.position.x, b.position.x, 1e-12);
  EXPECT_NEAR(a.position.y, b.position.y, 1e-12);
  EXPECT_NEAR(a.position.z, b.position.z, 1e-12);
  EXPECT_EQ(pose_json(snapshot_pose(pose_json(a))), pose_json(a));
}
